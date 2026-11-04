/**
 * Logic Analyzer — 6-channel programmable logic analyzer (v2)
 * ======================================================
 * Arduino con 6 canali digitali (D13..D8), stream a baud rate variabile.
 *
 * NOVITA' v2 (rispetto alla v1)
 * -----------------------------
 *  1. BASE DEI TEMPI ESATTA — il campionamento non e' piu' fatto con
 *     delayMicroseconds() nel loop (periodo reale +45% e jitteroso: T15 dava
 *     21.8us veri). Ora il periodo lo scandisce il Timer1 in modalita' CTC con
 *     ISR: il periodo e' quello che chiedi, al cristallo.
 *
 *  2. RING BUFFER — l'ISR deposita il campione in un ring di RAM; il loop lo
 *     drena sulla UART senza mai bloccare (usa availableForWrite). Cosi' la
 *     cadenza di campionamento non dipende piu' dalla UART.
 *
 *  3. CONTATORI DI PERDITA — se il ring si riempie il campione viene perso, ma
 *     viene CONTATO (nDrops). Il backend puo' interrogare 'Q' e sapere se e
 *     quanto la linea temporale e' integra. Niente piu' calibrazioni a mano.
 *
 *  4. MODALITA' EVT — codifica a TRANSIZIONI. Pin Change Interrupt su PORTB +
 *     Timer1 free-running (tick 0.5us). Si trasmettono SOLO i cambi di stato
 *     con timestamp hardware: idle = zero byte, risoluzione ~0.5us (~44x
 *     meglio del campionamento fisso). Vedi protocollo sotto.
 *
 * PIN MAPPING (invariato)
 * -----------------------
 *   D13 (PORTB bit5) → CH0      D9  (bit1) → CH4
 *   D12 (bit4)       → CH1      D8  (bit0) → CH5
 *   D11 (bit3)       → CH2
 *   D10 (bit2)       → CH3
 * Tutti INPUT hi-Z puro (nessun pull-up). 'U' attiva le pull-up, 'Z' le toglie.
 *
 * PROTOCOLLO SERIALE (comandi \n terminated)
 * ------------------------------------------
 *   B<baud>   Cambio baud a caldo via UBRR. Risposta "BR<baud>"
 *   T<us>     Periodo di campionamento in microsecondi.
 *             T0 = AUTO: lo sketch cerca da solo la massima velocita' che la
 *                   UART sostiene senza perdere campioni (adatta ogni 500ms).
 *             Risposta "TR<us>" (us = periodo effettivo)
 *   M<6bit>   Maschera canali. Risposta "MR<6bit>"
 *   R<bit>    Risoluzione (solo R1 supportata dal motore a timer). "RR<bit>"
 *   RAW       Stream puro: 1 byte/campione, nessun header.
 *   OSC       Stream con header ogni 256 campioni: 0xAA <mask> <id>
 *   EVT       Stream a TRANSIZIONI (formato sotto). T<us> non si applica.
 *   X         Ferma lo stream. Risposta "X_OK"
 *   Q         Contatori: "Q:<periodo>,<campioni>,<persi>,<eventi>,<evt_persi>,<glitch>,<modo>"
 *   Z / U     Hi-Z puro / pull-up su tutti i canali
 *   PING / STATUS
 *
 * FORMATO EVT (auto-delimitante)
 * ------------------------------
 *   delta + stato. Il bit alto di ogni byte e' flag:
 *     [0x80|d0][0x80|d1]...  = pezzi di delta (7 bit ciascuno, little endian)
 *     [0x00-0x3F]            = stato dei 6 canali (bit0=CH5 ... bit5=CH0)
 *     [0x40-0x7F]            = gap: (val & 0x3F) eventi persi per ring pieno
 *   Il delta e' in tick di 0.5us. Un delta di 0 = due fronti nello stesso tick.
 */

// ============================================================
// CONFIGURAZIONE
// ============================================================
#define RING_SIZE 512                 // potenza di 2 (512 byte di SRAM dei 2048)
#define RING_MASK (RING_SIZE - 1)

const byte CH_MASK_ALL = 0x3F;        // 6 canali (bit0=D8 .. bit5=D13)

long     baudRate    = 2000000;
byte     channelMask = CH_MASK_ALL;
byte     resolution  = 1;

volatile bool     streaming = false;
volatile byte     mode      = 0;      // 0=idle 1=RAW 2=OSC 3=EVT
volatile uint32_t actPeriodUs = 15;   // periodo REALMENTE impostato nel timer
volatile bool     autoRate  = false;  // T0: adattamento automatico
uint32_t reqPeriodUs = 15;            // periodo richiesto (T<us>)

// ============================================================
// RING BUFFER (scritto dall'ISR, letto dal loop)
// ============================================================
volatile byte     ring[RING_SIZE];
volatile uint16_t rbHead = 0;         // prossima posizione di scrittura
volatile uint16_t rbTail = 0;         // prossima posizione di lettura

// ============================================================
// STATISTICHE (onesta' della base dei tempi)
// ============================================================
volatile uint32_t nSamples = 0;       // campioni acquisiti (RAW/OSC)
volatile uint32_t nDrops   = 0;       // campioni persi per ring pieno
volatile uint32_t nEvents  = 0;       // eventi acquisiti (EVT)
volatile uint32_t nEvtLost = 0;       // eventi persi per ring pieno (EVT)
volatile bool     gapPend  = false;   // c'e' un buco EVT da dichiarare
volatile uint16_t t1wraps  = 0;       // overflow di Timer1 (EVT, tick 32-bit)
volatile uint32_t lastTick = 0;       // tick dell'ultimo fronte (EVT)
volatile byte     lastEvtState = 0xFF; // ultimo stato registrato (0xFF = nessuno)
volatile uint32_t nGlitch  = 0;       // PCINT senza cambio di stato (glitch < latenza ISR)
// AUTO-MISURA DELLA ISR — l'analizzatore dichiara il PROPRIO limite.
// Senza questo, un impulso corto misurato a 24 µs non si sa se è del segnale o
// dell'istrumento: il minimo osservabile dipende da quanto dura la ISR (due
// fronti più vicini di così non sono separabili). Meglio misurarlo che
// dichiararlo per stima.
volatile uint16_t isrMax  = 0;        // durata massima ISR in tick (0.5 µs)
uint32_t reportedDrops = 0;           // campioni persi gia' dichiarati nel flusso
uint32_t reportedGlitch = 0;          // glitch gia' dichiarati nel flusso EVT
uint32_t lastGlitchMs  = 0;           // rate-limit del rapporto glitch
uint32_t lastMarkerMs  = 0;           // rate-limit dei marcatori di perdita

// ============================================================
// BUFFER DI CATTURA — la ISR fa SOLO questo
// ============================================================
// Perche': due fronti più vicini della durata della ISR non si separano. Con la
// ISR a 29 µs (misurati) il minimo impulso osservabile era 26 µs su TUTTI i
// canali, e non si sapeva se fosse del segnale o dell'istrumento.
// Qui la ISR non calcola NIENTE: legge il Timer e lo stato e li deposita in un
// buffer circolare. Codifica a lunghezza variabile, estensione a 32 bit, filtro
// anti-glitch e contatori passano al loop principale (costano zero sul tempo
// critico). Da ~190 cicli a ~60: la risoluzione cambia ordine di grandezza.
#define CAP_SIZE 128                  // potenza di 2 -> 128*3 = 384 byte
#define CAP_MASK (CAP_SIZE - 1)
volatile uint16_t capT[CAP_SIZE];     // TCNT1 al fronte   (scritto SOLO dalla ISR)
volatile byte     capS[CAP_SIZE];     // PINB  al fronte   (scritto SOLO dalla ISR)
volatile byte     capIn  = 0;         // indice di scrittura (solo ISR)
volatile byte     capOut = 0;         // indice di lettura  (solo main)
volatile uint32_t nCapLost = 0;       // catture perse per buffer pieno (dichiarate)

// prescaler Timer1: valore e bit CS12:CS10
const uint16_t PRESC[5] = {1, 8, 64, 256, 1024};
const byte     PCSEL[5] = {1, 2, 3, 4, 5};

// adattamento auto-rate
uint32_t autoNextCheck = 0;
uint32_t autoLastDrops = 0;
byte     autoStable    = 0;

// ============================================================
// HELPERS RING (chiamabili da ISR e loop)
// ============================================================
inline uint16_t ringUsed() { return (rbHead - rbTail) & RING_MASK; }
inline uint16_t ringFree() { return (RING_SIZE - 1) - ringUsed(); }

// ============================================================
// ISR — MOTORE A CAMPIONAMENTO FISSO (RAW / OSC)
// ============================================================
ISR(TIMER1_COMPA_vect) {
  uint16_t nx = (rbHead + 1) & RING_MASK;
  if (nx == rbTail) {           // ring pieno: perdo il campione, ma lo conto
    nDrops++;
    return;
  }
  ring[rbHead] = PINB & channelMask & CH_MASK_ALL;
  rbHead = nx;
  nSamples++;
}

// ============================================================
// ISR — MOTORE A TRANSIZIONI (EVT)
// ============================================================
ISR(TIMER1_OVF_vect) { t1wraps++; }

// Legge Timer1 su 32 bit gestendo la gara con l'overflow
inline uint32_t tickNow() {
  uint16_t t = TCNT1;
  uint8_t  f = TIFR1;
  uint16_t w = t1wraps;
  if ((f & _BV(TOV1)) && t < 0x8000) {
    return ((uint32_t)(w + 1) << 16) | t;   // overflow pendente non ancora servito
  }
  return ((uint32_t)w << 16) | t;
}

// Accoda un byte dal ring (ISR context). Ritorna false se pieno.
inline bool evtPush(uint16_t &h, byte b) {
  uint16_t nx = (h + 1) & RING_MASK;
  if (nx == rbTail) return false;
  ring[h] = b;
  h = nx;
  return true;
}

// Scrive un evento codificato. Chiamata solo da ISR o da codice con gli
// interrupt disabilitati.
void evtEmit(uint32_t delta, byte state, bool withGap) {
  // quanti byte servono?
  byte need = 1;                 // ultimo pezzo di delta
  uint32_t tmp = delta;
  while (tmp >= 128) { need++; tmp >>= 7; }
  need += 1;                     // byte di stato
  if (withGap) need += 1;        // marcatore di gap

  if (ringFree() < need) {       // non ci sta tutto l'evento: scarto e dichiaro
    nEvtLost++;
    gapPend = true;
    return;
  }
  uint16_t h = rbHead;
  if (withGap) {
    byte lost = (nEvtLost > 62) ? 62 : (byte)nEvtLost;   // 63 = 0x7F: riservato
    evtPush(h, 0x40 | lost);
    nEvtLost -= lost;
    if (nEvtLost == 0) gapPend = false;
  }
  tmp = delta;
  while (tmp >= 128) { evtPush(h, 0x80 | (byte)(tmp & 0x7F)); tmp >>= 7; }
  evtPush(h, 0x80 | (byte)tmp);
  evtPush(h, state);
  rbHead = h;
  nEvents++;
}

// Chiude il cronometraggio della ISR. La differenza a 16 bit regge anche il
// wraparound di TCNT1: la ISR dura molti ordini di grandezza meno di 65536 tick.
// Il costo dell'automisura è ~9 cicli: incide per ~0,6 µs sul valore dichiarato,
// che quindi è un limite SUPERIORE (conservativo, non ottimistico).
#define ISR_COST(t0) do { uint16_t _d = (uint16_t)(TCNT1 - (t0)); \
                          if (_d > isrMax) isrMax = _d; } while (0)

// ── ISR MINIMA ────────────────────────────────────────────────
// Legge il Timer e i pin e li deposita nel buffer. Nient'altro: ogni ciclo
// risparmiato qui è risoluzione guadagnata sui fronti ravvicinati.
//
// EVT_ISR_PAD — TARATURA SPERIMENTALE DEL LIMITE.
// Allunga DI PROPOSITO la ISR di N iterazioni di nop. Serve a distinguere un
// limite dello STRUMENTO da una caratteristica del SEGNALE: se allungando la
// ISR il minimo impulso osservato si sposta della stessa quantità, quel minimo
// era dell'istrumento; se resta fermo, è del segnale. Con 0 (produzione) non
// costa nulla.
#ifndef EVT_ISR_PAD
#define EVT_ISR_PAD 0
#endif

ISR(PCINT0_vect) {
  uint16_t t0 = TCNT1;
  byte nxt = (capIn + 1) & CAP_MASK;
  if (nxt == capOut) { nCapLost++; ISR_COST(t0); return; }   // buffer pieno
  capS[capIn] = PINB;
  capT[capIn] = t0;
  capIn = nxt;
#if EVT_ISR_PAD > 0
  for (byte _i = 0; _i < EVT_ISR_PAD; _i++) __asm__ __volatile__("nop");
#endif
  ISR_COST(t0);
}

// ── ELABORAZIONE DELLE CATTURE (loop principale) ───────────────
// 1) ricostruzione del tick assoluto a 32 bit: le catture sono in ordine
//    cronologico, quindi quando TCNT1 "torna indietro" è avvenuto un overflow;
// 2) filtro anti-glitch (stato invariato → non è un fronte osservabile);
// 3) codifica nel ring.
// Il filtro anti-glitch resta indispensabile: i pin in alta impedenza
// accoppiano capacitivamente e un disturbo più corto della latenza alza il flag
// PCINT senza che lo stato sia davvero cambiato. Si CONTA, non si inventa.
static uint16_t capLastT = 0;
static uint32_t capWraps = 0;

void processCapture() {
  if (capOut == capIn) return;

  // Risincronizzazione: il buffer era vuoto (pausa, oppure drain precedente
  // completato). Si prende il conteggio overflow corrente e si corregge con il
  // confronto fra la prima cattura e TCNT1 di adesso: se la cattura è "dopo" il
  // valore attuale, l'overflow è avvenuto DOPO la cattura.
  uint32_t abs0 = tickNow();
  capWraps = abs0 >> 16;
  if (capT[capOut] > (uint16_t)abs0 && capWraps > 0) capWraps--;
  capLastT = capT[capOut];

  while (capOut != capIn) {
    uint16_t tc = capT[capOut];
    byte     s  = capS[capOut];
    capOut = (capOut + 1) & CAP_MASK;

    if (tc < capLastT) capWraps++;          // un giro completo di Timer1
    capLastT = tc;
    uint32_t now = (capWraps << 16) | tc;

    s &= channelMask & CH_MASK_ALL;
    if (s == lastEvtState) { nGlitch++; continue; }   // disturbo non osservabile

    // Rete di sicurezza: se il tick è tornato indietro di più di un overflow
    // (main loop bloccato a lungo) non si inventa un delta assurdo.
    if (now < lastTick) { lastTick = now; lastEvtState = s; continue; }

    uint32_t d = now - lastTick;
    lastTick = now;
    lastEvtState = s;
    evtEmit(d, s, gapPend);
  }

  // Perdite del buffer di cattura: si dichiarano come un buco nella timeline
  // (stesso meccanismo degli eventi persi per ring pieno).
  if (nCapLost) {
    noInterrupts(); uint32_t l = nCapLost; nCapLost = 0; interrupts();
    nEvtLost += l; gapPend = true;
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  // hi-Z puro: il circuito osservato comanda, noi ascoltiamo
  for (byte p = 8; p <= 13; p++) pinMode(p, INPUT);

  Serial.begin(baudRate);
  while (!Serial) { ; }

  Serial.println(F("LOGIC-ANALYZER v2 — READY"));
  stampaStatus();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  if (streaming) {
    // 1) PRIMA il comando di stop: se il drenaggio affamasse l'RX non si
    //    uscirebbe piu' dallo stream. Consumo tutto l'RX disponibile.
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 'X' || c == 'x') { stopStream(); return; }
    }

    // 2) Marcatore di perdita IN-BAND (solo RAW/OSC): 0xFF <hi> <lo>
    //    Il dato e' mascherato a 6 bit (0x00-0x3F), quindi 0xFF non puo'
    //    comparire nei campioni: il backend lo riconosce senza ambiguita'.
    emitDropMarker();

    // 2b) Rapporto glitch IN-BAND (solo EVT): 0x7F <g_hi> <g_lo> <i_hi> <i_lo>
    emitGlitchMarker();

    // 2c) Catture grezze della ISR → tick assoluto → filtro glitch → ring.
    //     Deve stare PRIMA del drenaggio: prima si produce, poi si spedisce.
    if (mode == 3) processCapture();

    // 3) Drenaggio ring → UART
    drainRing();

    // 4) Adattamento automatico del rate (solo RAW/OSC in modalita' AUTO)
    if (autoRate && mode != 3) autoTune();
    return;
  }

  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  String val = line.substring(1);
  val.trim();

  switch (cmd) {
    case 'B': case 'b': cmdBaud(val); break;
    case 'T': case 't': cmdTimer(val); break;
    case 'M': case 'm': cmdMask(val); break;
    case 'R': case 'r':
      if (line.equalsIgnoreCase("RAW"))      startStream(1);
      else if (line.equalsIgnoreCase("RES")) { /* no-op */ }
      else                                   cmdResolution(val);
      break;
    case 'O': case 'o':
      if (line.equalsIgnoreCase("OSC")) startStream(2);
      break;
    case 'E': case 'e':
      if (line.equalsIgnoreCase("EVT")) startStream(3);
      break;
    case 'X': case 'x':
      Serial.println(F("X_OK"));
      break;
    case 'Q': case 'q':
      cmdQuery();
      break;
    case 'Z': case 'z':
      for (byte p = 8; p <= 13; p++) pinMode(p, INPUT);
      Serial.println(F("Z_OK"));
      break;
    case 'U': case 'u':
      for (byte p = 8; p <= 13; p++) pinMode(p, INPUT_PULLUP);
      Serial.println(F("U_OK"));
      break;
    case 'P': case 'p':
      if (line.equalsIgnoreCase("PING"))       Serial.println(F("PONG"));
      else if (line.equalsIgnoreCase("POWER")) Serial.println(F("POWER_OK"));
      break;
    case 'S': case 's':
      if (line.equalsIgnoreCase("STATUS")) stampaStatus();
      break;
    default:
      Serial.print(F("ERR_UNKNOWN: "));
      Serial.println(line);
      break;
  }
}

// ============================================================
// MARCATORE DI PERDITA (RAW/OSC)
// ============================================================
// Trasmette 0xFF <hi> <lo> quando l'ISR ha perso campioni per ring pieno.
// Serve a rendere VISIBILE nel flusso un buco della base dei tempi: senza,
// un overrun slitta la linea temporale in silenzio. In EVT i buchi sono gia'
// dichiarati in-stream (0x40|n).
//
// RATE-LIMIT OBBLIGATORIO: senza, a rate alti i marcatori inondano la UART,
// affamano il drenaggio dei dati e lo sketch si impianta (verificato: T12
// non rispondeva piu' a X). Max ~20 marcatori/s = 60 byte/s, trascurabile.
void emitDropMarker() {
  if (mode == 3) return;
  uint32_t d = nDrops - reportedDrops;
  if (d == 0) return;
  uint32_t now = millis();
  if (now - lastMarkerMs < 50) return;
  if (Serial.availableForWrite() < 3) return;
  lastMarkerMs = now;
  if (d > 65535) d = 65535;
  Serial.write((uint8_t)0xFF);
  Serial.write((uint8_t)(d >> 8));
  Serial.write((uint8_t)(d & 0xFF));
  reportedDrops += d;
}

// ============================================================
// RAPPORTO GLITCH (solo EVT)
// ============================================================
// Marcatore esteso 0x7F <g_hi> <g_lo> <i_hi> <i_lo>  (1 rapporto/s)
//   g = glitch contati dall'ultimo rapporto
//   i = durata MASSIMA della ISR in µs = LIMITE DELLO STRUMENTO
// Esiste perche' i pin hi-Z sono rumorosi: senza contatore i disturbi sotto la
// latenza dell'ISR sarebbero invisibili. E il limite va DICHIARATO: un impulso
// di 24 µs e' del segnale o dell'istrumento? Lo dice questo campo.
// 63 (0x7F) è riservato, percio' i gap si fermano a 62.
// Max 1 rapporto/s: 6 byte/s, trascurabile.
void emitGlitchMarker() {
  if (mode != 3) return;
  uint32_t now = millis();
  if (now - lastGlitchMs < 1000) return;   // 1 rapporto/s: 6 byte/s, trascurabile
  if (ringFree() < 6) return;
  uint32_t d = nGlitch - reportedGlitch;
  if (d > 65535) d = 65535;
  uint16_t im = isrMax / 2;                // limite dell'istrumento in µs
  lastGlitchMs = now;
  // Sezione critica: il ring è scritto dal loop principale (processCapture e
  // emitDropMarker), quindi qui non c'è più contesa con la ISR. Si tiene
  // comunque noInterrupts() perché evtPush è documentata per quel contesto e
  // perché un futuro uso da ISR resterebbe corretto.
  noInterrupts();
  uint16_t h = rbHead;
  // Il marcatore vive nella posizione di STATO, quindi deve essere preceduto
  // da un pezzo di delta (0x80 = delta 0) altrimenti il decoder lo scarta.
  // Il delta 0 non altera la timeline: il prossimo evento reale misura il
  // proprio delta dall'ultimo fronte vero (lastTick non viene toccato).
  evtPush(h, 0x80);
  evtPush(h, 0x7F);
  evtPush(h, (byte)(d >> 8));  evtPush(h, (byte)(d & 0xFF));
  evtPush(h, (byte)(im >> 8)); evtPush(h, (byte)(im & 0xFF));
  rbHead = h;
  interrupts();
  reportedGlitch += d;
}

// ============================================================
// DRENAGGIO DEL RING → UART (mai bloccante)
// ============================================================
void drainRing() {
  uint16_t used = ringUsed();
  if (!used) return;

  int space = Serial.availableForWrite();
  if (space <= 0) return;

  uint16_t n = used;
  if (n > (uint16_t)space) n = (uint16_t)space;

  uint16_t t = rbTail;
  uint16_t run = RING_SIZE - t;
  if (run > n) run = n;

  Serial.write((const uint8_t *)&ring[t], run);
  if (run < n) {                                  // la coda e' avvolta
    Serial.write((const uint8_t *)&ring[0], n - run);
  }
  rbTail = (t + n) & RING_MASK;
}

// ============================================================
// COMANDI
// ============================================================
void cmdBaud(String val) {
  long nuovo = val.toInt();
  if (nuovo < 9600 || nuovo > 2000000) {
    Serial.print(F("ERR_BAUD: ")); Serial.println(nuovo); return;
  }
  setBaudUBRR(nuovo);
  baudRate = nuovo;
  Serial.print(F("BR")); Serial.println(baudRate);
}

void cmdTimer(String val) {
  long us = val.toInt();
  if (us <= 0) {                       // T0 = AUTO
    autoRate = true;
    reqPeriodUs = 20;                  // punto di partenza, poi si adatta
    if (timerSetPeriod(reqPeriodUs)) {
      Serial.print(F("TR")); Serial.print(actPeriodUs); Serial.println(F(" AUTO"));
    } else {
      Serial.println(F("ERR_TIMER"));
    }
    return;
  }
  if (us > 4000000L) us = 4000000L;    // oltre 4s servirebbe l'overflow mode
  // PAVIMENTO DI SICUREZZA: sotto ~8 µs (125 kHz) l'ISR del Timer1 satura la
  // CPU e il loop non gira piu' — lo sketch smette di rispondere anche a X
  // (verificato: T5 = 200 kHz → zero byte e sketch muto). Il pavimento
  // garantisce che il loop abbia sempre tempo di drenare e di leggere i comandi.
  if (us < 8) us = 8;
  autoRate = false;
  reqPeriodUs = (uint32_t)us;
  if (timerSetPeriod(reqPeriodUs)) {
    Serial.print(F("TR")); Serial.println(actPeriodUs);
  } else {
    Serial.println(F("ERR_TIMER"));
  }
}

void cmdMask(String val) {
  byte m = 0;
  // Stringa binaria a 6 caratteri SOLO se tutti i char sono '0'/'1'.
  // (Nella v1 la condizione cercava uno '0': "111111" cadeva nel toInt()
  //  → 111111 mod 256 = 7 → maschera 000111, tre canali spenti in silenzio!)
  bool bin = (val.length() == 6);
  if (bin) {
    for (int i = 0; i < 6; i++) {
      char c = val.charAt(i);
      if (c != '0' && c != '1') { bin = false; break; }
    }
  }
  if (bin) {
    for (int i = 0; i < 6; i++) if (val.charAt(i) == '1') m |= (1 << (5 - i));
  } else if (val.startsWith("0x") || val.startsWith("0X")) {
    m = (byte)strtol(val.c_str(), NULL, 16);
  } else {
    m = (byte)val.toInt();
  }
  channelMask = m & CH_MASK_ALL;
  if (mode == 3) {
    PCMSK0 = channelMask;                        // EVT: aggiorna le sorgenti
    lastEvtState = PINB & channelMask & CH_MASK_ALL;  // niente evento spurio
  }
  Serial.print(F("MR"));
  for (int i = 5; i >= 0; i--) Serial.print((channelMask >> i) & 1);
  Serial.println();
}

void cmdResolution(String val) {
  byte r = (byte)val.toInt();
  if (r != 1) {
    Serial.println(F("ERR_RES: solo R1 col motore a timer"));
    return;
  }
  resolution = 1;
  Serial.println(F("RR1"));
}

void cmdQuery() {
  Serial.print(F("Q:"));     Serial.print(actPeriodUs);
  Serial.print(',');         Serial.print(nSamples);
  Serial.print(',');         Serial.print(nDrops);
  Serial.print(',');         Serial.print(nEvents);
  Serial.print(',');         Serial.print(nEvtLost);
  Serial.print(',');         Serial.print(nGlitch);
  Serial.print(',');         Serial.print(isrMax / 2);   // limite dell'istrumento in µs
  Serial.print(',');         Serial.print(mode == 3 ? F("EVT") : (autoRate ? F("AUTO") : F("FIX")));
  Serial.println();
}

// ============================================================
// MOTORE A CAMPIONAMENTO FISSO (Timer1 CTC)
// ============================================================
bool timerSetPeriod(uint32_t periodUs) {
  if (periodUs < 2) periodUs = 2;
  for (byte i = 0; i < 5; i++) {
    uint64_t ocr = ((uint64_t)F_CPU * (uint64_t)periodUs)
                 / ((uint64_t)PRESC[i] * 1000000ULL);
    if (ocr >= 2 && ocr <= 65536) {
      noInterrupts();
      TCCR1A = 0;
      TCCR1B = _BV(WGM12) | PCSEL[i];   // CTC (top = OCR1A) + prescaler
      TCNT1  = 0;
      OCR1A  = (uint16_t)(ocr - 1);
      TIMSK1 = _BV(OCIE1A);
      actPeriodUs = (uint32_t)(((uint64_t)ocr * PRESC[i] * 1000000ULL) / F_CPU);
      interrupts();
      return true;
    }
  }
  return false;
}

// ============================================================
// MOTORE A TRANSIZIONI (EVT)
// ============================================================
void evtArm() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(CS11);              // free-running, prescaler 8 → tick 0.5 µs
  TCNT1  = 0;
  t1wraps = 0;
  TIMSK1 = _BV(TOIE1);             // solo overflow: serve l'orologio a 32 bit
  PCMSK0 = 0;
  PCICR &= ~_BV(PCIE0);            // PCINT spento durante il setup
  nGlitch = 0;
  isrMax = 0;
  // Il buffer di cattura è condiviso con la ISR: si azzera con gli interrupt
  // spenti, e si riparte da "vuoto" (la risincronizzazione avviene alla prima
  // cattura elaborata).
  noInterrupts();
  capIn = 0; capOut = 0; nCapLost = 0;
  interrupts();
  capLastT = 0; capWraps = 0;
  interrupts();

  // Evento sintetico iniziale: il decoder deve sapere da che stato si parte
  lastTick = tickNow();
  byte s0 = PINB & channelMask & CH_MASK_ALL;
  lastEvtState = s0;
  evtEmit(0, s0, false);

  // Solo ora si abilita l'interrupt: cosi' non si genera un evento spuro
  // per il primo fronte mentre lastEvtState non è ancora valido.
  noInterrupts();
  PCMSK0 = channelMask;
  PCICR |= _BV(PCIE0);
  interrupts();
}

// ============================================================
// START / STOP STREAM
// ============================================================
void startStream(byte newMode) {
  mode = newMode;
  rbHead = rbTail = 0;                 // ring pulito
  nSamples = nDrops = nEvents = nEvtLost = 0;
  reportedDrops = 0;
  reportedGlitch = 0;
  lastGlitchMs = 0;
  lastMarkerMs = 0;
  nGlitch = 0;
  lastEvtState = 0xFF;
  gapPend = false;

  if (mode == 3) {                     // EVT
    Serial.println(F("EVT_GO"));
    streaming = true;
    evtArm();
    return;
  }

  if (mode == 1) Serial.println(F("RAW_GO"));
  else           Serial.println(F("OSC_START"));

  if (!timerSetPeriod(reqPeriodUs)) {
    Serial.println(F("ERR_TIMER"));
    streaming = false;
    mode = 0;
    return;
  }
  streaming = true;
}

void stopStream() {
  noInterrupts();
  TIMSK1 = 0;                          // ferma ogni interrupt di Timer1
  PCICR  = 0;                          // ferma i pin change
  PCMSK0 = 0;
  TCCR1B = 0;                          // ferma il timer
  streaming = false;
  interrupts();

  if (mode == 3) Serial.println(F("EVT_END"));
  else if (mode == 1) Serial.println(F("RAW_END"));
  else Serial.println(F("OSC_END"));
  mode = 0;
}

// ============================================================
// AUTO-RATE: cerca la massima velocita' che la UART sostiene
// ============================================================
void autoTune() {
  uint32_t now = millis();
  if (now < autoNextCheck) return;
  autoNextCheck = now + 1000;

  if (nDrops != autoLastDrops) {
    // Ho perso campioni: il tetto e' superato → indietreggia del 33%
    autoLastDrops = nDrops;
    uint32_t p = actPeriodUs + (actPeriodUs / 3) + 2;
    if (p > 200) p = 200;
    timerSetPeriod(p);
    autoStable = 0;
  } else if (++autoStable >= 5 && actPeriodUs > 8) {
    // Stabile da 5 secondi: prova a scendere di 1 µs (sonda prudente)
    autoStable = 0;
    timerSetPeriod(actPeriodUs - 1);
  }
}

// ============================================================
// STAMPA STATUS
// ============================================================
void stampaStatus() {
  Serial.print(F("S:"));
  Serial.print(baudRate);          Serial.print(',');
  Serial.print(actPeriodUs);       Serial.print(',');
  for (int i = 5; i >= 0; i--) Serial.print((channelMask >> i) & 1);
  Serial.print(',');
  Serial.print(resolution);        Serial.print(',');
  if (streaming) Serial.print(mode == 3 ? F("EVT") : (autoRate ? F("STREAMING-AUTO") : F("STREAMING")));
  else           Serial.print(F("IDLE"));
  Serial.print(F(","));
  Serial.print(F("ring=")); Serial.print(ringUsed()); Serial.print('/'); Serial.print(RING_SIZE - 1);
  Serial.print(F(",drops=")); Serial.print(nDrops);
  Serial.println();
}

// ============================================================
// CAMBIO BAUD A CALDO (invariato dalla v1)
// ============================================================
void setBaudUBRR(long baud) {
  // 1. Svuota il buffer TX dello sketch e attende la FINE dell'ultimo frame.
  //    ATTENZIONE: NON scrivere 1 in TXC0 per "azzerarlo" — e' write-1-to-clear,
  //    e se il trasmettitore e' inattivo il flag non torna mai a 1 → hang.
  //    (Bug latente presente anche nella v1: appena arrivava "B<baud>" con la
  //     seriale a riposo, lo sketch si piantava.)
  Serial.flush();
  while (!(UCSR0A & _BV(TXC0)));

  // 2. Calcola UBRR
  int  ubrr;
  bool u2x;
  if (baud <= 115200) {
    u2x  = false;
    ubrr = (F_CPU / 16 / baud) - 1;
  } else {
    u2x  = true;
    ubrr = (F_CPU / 8 / baud) - 1;
  }
  if (ubrr < 0)    ubrr = 0;
  if (ubrr > 4095) ubrr = 4095;

  // 3. Imposta U2X e UBRR senza toccare TXEN0/RXEN0 (e quindi senza
  //    disturbare lo stato di HardwareSerial e i suoi interrupt)
  if (u2x) UCSR0A |=  _BV(U2X0);
  else     UCSR0A &= ~_BV(U2X0);

  UBRR0L = (byte)(ubrr & 0xFF);
  UBRR0H = (byte)((ubrr >> 8) & 0x0F);

  // 4. Scarta eventuali byte RX spuri (parziali ricevuti al vecchio baud)
  while (UCSR0A & _BV(RXC0)) { (void)UDR0; }

  delay(10);
}
