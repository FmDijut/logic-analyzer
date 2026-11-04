"""
Logic Analyzer — backend web (Flask + SSE)
============================================
Filosofia: il server è TRASPARENTE. L'Arduino è un sensore grezzo che
streamma a max frequenza (RAW, nessun header). Il server accumula tutto
in un ring buffer e spinge al browser SOLO i campioni nuovi (incrementale).
Tutta la scansione temporale, lo zoom e la decodifica avvengono nel browser.

Esegui:  python server.py    →  http://127.0.0.1:8788
"""
import serial, time, json, threading, queue, os, re, hashlib, array
from collections import deque
from flask import Flask, Response, send_from_directory, request

# ── Config ─────────────────────────────────────────────────────
PORT     = os.environ.get("LA_PORT", "COM11")   # override: LA_PORT=COM4
BAUD_DEF = 2000000
CANALI   = 6
NOME     = ["CH0 (D13)","CH1 (D12)","CH2 (D11)","CH3 (D10)","CH4 (D9)","CH5 (D8)"]
COLORE   = ["#ffd700","#44cc44","#4488ff","#eeeeee","#ff4444","#66cc66"]
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(__name__, static_folder=os.path.join(BASE_DIR, "static"))
cur_rate = 0.0   # kS/s correnti (scritto dal worker, letto da /api/edges)

# ── Stato ──────────────────────────────────────────────────────
ser          = None
ser_lock     = threading.Lock()
running      = False
worker_th    = None
# ── Broadcast di controllo ───────────────────────────────
# PRIMA era una sola coda.Queue condivisa con UN consumatore per connessione:
# chi la leggeva per primo vinceva il messaggio. Con due tab aperte (o una tab
# "zombie" della pagina precedente ancora collegata) i messaggi venivano
# RUBATI alla pagina attiva, che si collegava ma restava muta senza che nulla
# lo segnalasse. Ora ogni connessione registra la PROPRIA coda: questi
# messaggi sono di controllo (stato, statistiche, reset), piccoli e
# idempotenti, quindi vanno a TUTTI.
# I dati campione/eventi NON passano di qui: ognuno ha il suo cursore.
class _Broadcast:
    def __init__(self):
        self.subs = []
        self.lock = threading.Lock()

    def register(self):
        q = queue.Queue(maxsize=64)
        with self.lock:
            self.subs.append(q)
        return q

    def unregister(self, q):
        with self.lock:
            try: self.subs.remove(q)
            except ValueError: pass

    def put_nowait(self, msg):
        with self.lock:
            for q in self.subs:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    # client lento: per lo stato vince l'ULTIMO, quindi butto il vecchio
                    try: q.get_nowait()
                    except queue.Empty: pass
                    try: q.put_nowait(msg)
                    except queue.Full: pass

    def drain(self):
        pass          # niente da svuotare: i dati non transitano più di qui

stream_queue = _Broadcast()

baud      = BAUD_DEF
timer_us  = 0
mask_val  = 0x3F
resol     = 1

# Base dei tempi REALE e integrità del flusso (task 1 e 2)
cur_period_us = 20.0    # periodo di campionamento effettivo in µs, letto dallo sketch
lost_total    = 0       # campioni persi dichiarati dallo sketch (marcatori 0xFF)

# ── Modalità EVT: stream a TRANSIZIONI (task 3) ──────────────
# Lo sketch trasmette solo i cambi di stato con timestamp hardware a 0.5 µs:
# idle = zero byte, risoluzione ~44x migliore del campionamento fisso.
stream_mode  = 'raw'                  # 'raw' | 'evt'
EVT_TICK_US  = 0.5                    # tick del Timer1 in EVT (prescaler 8 @ 16 MHz)
EVT_MAX      = 400_000                # eventi tenuti in memoria
stream_changed = threading.Event()    # sveglia il worker quando cambia stream_mode
evt_t     = array.array('q')          # timestamp cumulativi in tick
evt_s     = bytearray()               # stato 6 bit per evento
evt_total = 0                         # eventi totali mai ricevuti (indice assoluto)
evt_sent  = 0                         # indice assoluto già spedito via SSE
evt_lost  = 0                         # eventi persi dichiarati dallo sketch (gap)
evt_glitch = 0                        # disturbi sotto la latenza ISR (dichiarati dallo sketch)
evt_isrmax = 0                        # LIMITE DELLO STRUMENTO: durata max ISR in µs
evt_epoch = 0                         # incrementato a ogni RIAVVIO dello stream EVT
evt_lock  = threading.Lock()

# ── Ring buffer raw con indice assoluto ────────────────────────
RING_SIZE = 2_000_000                    # 2M campioni (~10 s a 200 kS/s)
ring       = bytearray(RING_SIZE)
ring_head  = 0                           # posizione circolare
total_idx  = 0                           # indice assoluto del prossimo campione
sent_idx   = 0                           # indice assoluto già spedito via SSE
ring_lock  = threading.Lock()

def ring_push(data):
    global ring_head, total_idx, cur_rate
    n = len(data)
    if n >= RING_SIZE:
        data = data[-RING_SIZE:]; n = RING_SIZE
    with ring_lock:
        end = ring_head + n
        if end <= RING_SIZE:
            ring[ring_head:end] = data
        else:
            first = RING_SIZE - ring_head
            ring[ring_head:] = data[:first]
            ring[:n-first]   = data[first:]
        ring_head = end % RING_SIZE
        total_idx += n

def ring_get(start_abs, n):
    """Ritorna (bytes, gap) dei campioni [start_abs, start_abs+n)."""
    with ring_lock:
        oldest = total_idx - ring_cnt()
        gap = False
        if start_abs < oldest:
            gap = True
            n -= (oldest - start_abs)
            start_abs = oldest
        if n <= 0:
            return b"", gap
        start = (ring_head - (total_idx - start_abs)) % RING_SIZE
        out = bytearray(n)
        end = start + n
        if end <= RING_SIZE:
            out[:] = ring[start:end]
        else:
            first = RING_SIZE - start
            out[:first] = ring[start:]
            out[first:] = ring[:n-first]
        return bytes(out), gap

def ring_cnt():
    return min(total_idx, RING_SIZE)

# ── Seriale ────────────────────────────────────────────────────

def serial_open():
    global ser
    with ser_lock:
        if ser and ser.is_open:
            try: ser.close()
            except: pass
        try:
            s = serial.Serial(PORT, baud, timeout=0.02)
            s.setDTR(False); s.setRTS(False)
            try:
                s.set_buffer_size(rx_size=65536, tx_size=65536)  # niente overflow driver Win
            except Exception:
                pass
            buf = b""
            t0 = time.time()
            while time.time() - t0 < 3:
                buf += s.read(256)
                if b"READY" in buf:
                    break
            ser = s
            return True
        except Exception as e:
            print(f"[LA] ERR porta: {e}", flush=True)
            return False

def serial_close():
    global ser
    with ser_lock:
        if ser and ser.is_open:
            try:
                ser.write(b"X\n")
                ser.close()
            except: pass
        ser = None

# ── Worker: lettura seriale → ring (trasparente) ──────────────

def _push_config():
    """Rispedisce la config corrente (B/T/M/R) allo sketch quando non streamma.
    Serve dopo un ricollegamento: lo sketch è tornato ai default di boot.
    Ritorna la risposta grezza (BR/TR/MR/RR) — contiene il periodo REALE."""
    with ser_lock:
        s = ser
        if s is None or not s.is_open:
            return b""
        try:
            for c, v in (("B", baud), ("T", timer_us), ("M", f"{mask_val:06b}"), ("R", resol)):
                s.write(f"{c}{v}\n".encode())
                time.sleep(0.05)
            resp = b""
            t0 = time.time()
            while time.time() - t0 < 0.6:
                chunk = s.read(256)
                if chunk:
                    resp += chunk
                    t0 = time.time()
                elif resp:
                    break
            return resp
        except Exception:
            return b""

def _parse_period(resp, default=20.0):
    """Estrae il periodo dai messaggi dello sketch: 'TR<us>' oppure 'Q:<us>,...'.
    È la fonte di verità della base dei tempi: il valore che il Timer1 ha
    DAVVERO impostato (OCR1A arrotondato), non quello richiesto."""
    try:
        txt = resp.decode(errors="replace")
    except Exception:
        return float(default)
    m = re.search(r"TR(\d+)", txt)
    if m:
        return float(m.group(1))
    m = re.search(r"Q:(\d+)", txt)
    if m:
        return float(m.group(1))
    return float(default)

def _decode_markers(data, carry):
    """Rimuove i marcatori di perdita '0xFF <hi> <lo>' dal flusso RAW/OSC.
    I campioni sono mascherati a 6 bit (0x00-0x3F), quindi 0xFF non può
    comparire nei dati: nessuna ambiguità.
    Ritorna (dati_puliti, campioni_persi, byte_residui_per_il_prossimo_chunk)."""
    buf = (carry + data) if carry else data
    out = bytearray()
    lost = 0
    i = 0
    n = len(buf)
    while i < n:
        if buf[i] == 0xFF:
            if i + 2 >= n:            # marcatore tagliato a fine chunk: conserva
                return bytes(out), lost, buf[i:]
            lost += (buf[i + 1] << 8) | buf[i + 2]
            i += 3
        else:
            out.append(buf[i])
            i += 1
    return bytes(out), lost, b""

# ── Decoder EVT: flusso a transizioni → eventi (tick, stato) ────────────────
class EvtDecoder:
    """Decodifica il formato EVT dello sketch, byte per byte, mantenendo lo
    stato tra un chunk e l'altro.

    Formato: [0x80|d0][0x80|d1]... [stato 0x00-0x3F]   (bit alto = continuazione)
              [0x40|n]  = gap: n eventi persi per ring pieno
    Il delta è in tick di 0.5 µs; i timestamp sono cumulativi.
    """
    def __init__(self):
        self.acc = 0; self.shift = 0; self.t = 0; self.inframe = False
        self.pay = 0; self.ext = 0
        self.isr_max = 0                     # limite dell'istrumento dichiarato dallo sketch

    def feed(self, data):
        out = []
        lost = 0
        glitch = 0
        for b in data:
            if self.pay > 0:                    # payload del marcatore esteso 0x7F
                self.ext = (self.ext << 8) | b
                self.pay -= 1
                if self.pay == 0:
                    glitch += self.ext >> 16     # primo mezzo: glitch
                    self.isr_max = self.ext & 0xFFFF   # secondo mezzo: ISR max in µs
                continue
            if b & 0x80:
                self.acc |= (b & 0x7F) << self.shift
                self.shift += 7
                self.inframe = True
            elif self.inframe:
                if b == 0x7F:                   # esteso: glitch + limite ISR
                    self.pay = 4; self.ext = 0
                elif b > 0x3F:
                    lost += (b & 0x3F)          # marcatore di gap
                else:
                    self.t += self.acc
                    out.append((self.t, b))
                self.acc = 0; self.shift = 0; self.inframe = False
            # byte di stato senza delta (impossibile nel formato): ignorato
        return out, lost, glitch

def _auto_probe(probe_s=3.0):
    """T0 = AUTO: fa convergere l'auto-rate dello sketch, legge il periodo
    raggiunto con Q e lo BLOCCA come valore FISSO. Così otteniamo insieme la
    massima velocità sostenibile E una base dei tempi esatta (niente più
    costante 45800 calibrata a mano). Ritorna il periodo in µs o None."""
    with ser_lock:
        s = ser
        if s is None or not s.is_open:
            return None
        try:
            s.reset_input_buffer()
            s.write(b"RAW\n")
            t0 = time.time()
            while time.time() - t0 < 1.0:
                if b"\n" in s.read(64):
                    break
            t0 = time.time()
            while running and time.time() - t0 < probe_s:
                s.read(65536)             # scarta i dati del probe
            s.write(b"X\n")
            t0 = time.time()
            while time.time() - t0 < 2.0:
                if b"RAW_END" in s.read(256):
                    break
            s.reset_input_buffer()
            s.write(b"Q\n")
            resp = b""
            t0 = time.time()
            while time.time() - t0 < 1.0:
                d = s.read(128)
                if d:
                    resp += d
                    if b"\n" in resp:
                        break
            p = _parse_period(resp, 20.0)
            s.write(f"T{int(p)}\n".encode())   # blocca il periodo come FISSO
            time.sleep(0.25)
            s.read(128)
            print(f"[LA] AUTO: periodo convergiuto {p:.0f} µs "
                  f"({1e6/p/1000:.1f} kS/s teorici) — bloccato come fisso", flush=True)
            return p
        except Exception as e:
            print(f"[LA] AUTO probe fallita: {e}", flush=True)
            return None

def worker():
    """Stream seriale → ring, con AUTO-RECONNECT: se l'Arduino si scollega
    (o la porta scompare), riprova ogni 1.5s finché running è True. Al
    ricollegamento ripristina la config e lo stream RAW. Nessun intervento manuale."""
    global cur_rate, running, lost_total, cur_period_us

    while running:
        # riusa la porta già aperta: riaprirla = DTR pulse = RESET Arduino = config persa!
        with ser_lock:
            _already = ser is not None and ser.is_open
        if not _already and not serial_open():
            stream_queue.put_nowait({"type": "status", "connected": False,
                                     "streaming": False, "reconnecting": True})
            print("[LA] porta COM11 assente — riconnessione automatica ogni 1.5s...", flush=True)
            time.sleep(1.5)
            continue

        # dopo un ricollegamento lo sketch è tornato ai default: ripristina config
        resp = _push_config()

        # ── MODALITÀ EVT (task 3): stream a transizioni ──
        # Idle = zero byte, timestamp hardware a 0.5 µs. La banda non dipende
        # più dal periodo di campionamento: è il tempo a essere esatto.
        # In EVT il periodo di campionamento non esiste → nessun probe AUTO.
        if stream_mode == 'evt':
            _run_evt()
            continue

        # ── BASE DEI TEMPI REALE (task 1) ──
        # In T<us> fisso lo sketch risponde "TR<us>" col periodo davvero impostato
        # nel Timer1 (dopo l'arrotondamento di OCR1A). In T0 (AUTO) il periodo
        # cambia nel tempo: lo facciamo convergere e lo blocchiamo.
        if timer_us and timer_us > 0:
            cur_period_us = _parse_period(resp, timer_us)
        else:
            cur_period_us = _auto_probe() or 20.0
        print(f"[LA] base dei tempi: {cur_period_us:.0f} µs "
              f"({1e6/cur_period_us:.0f} S/s)", flush=True)

        # Prova RAW (stream puro, no header); fallback su OSC se sketch vecchio.
        # Dopo il probe AUTO lo sketch può avere ancora residui nel buffer: si
        # ritenta svegliandolo con X, e si stampa la risposta per diagnosi.
        raw_mode = False
        _carry = b""
        for tentativo in range(3):
            with ser_lock:
                ser.reset_input_buffer()
                ser.write(b"RAW\n")
                resp = b""
                t0 = time.time()
                while time.time() - t0 < 1.5:
                    resp += ser.read(64)
                    if b"\n" in resp:
                        break
            print(f"[LA] probe RAW #{tentativo+1}: {resp[:70]!r}", flush=True)
            if b"RAW" in resp and b"ERR" not in resp:
                raw_mode = True
                break
            with ser_lock:
                ser.write(b"X\n")
                time.sleep(0.4)
                ser.read(512)
        if not raw_mode:
            with ser_lock:
                ser.write(b"OSC\n")
                t0 = time.time()
                while time.time() - t0 < 2:
                    if ser.read(1) == b"\n":
                        break
        print(f"[LA] modalità: {'RAW' if raw_mode else 'OSC (legacy)'}", flush=True)

        stream_queue.put_nowait({"type": "status", "connected": True, "streaming": True})
        print("[LA] Stream avviato", flush=True)

        t_stat = time.time()
        cnt_stat = 0

        while running and stream_mode == 'raw':
            try:
                with ser_lock:
                    data = ser.read(4096)   # cadenza 20ms: bajo il buffer del driver, zero perdite
            except Exception:
                break
            if not data:
                continue

            if raw_mode:
                # Marcatori di perdita (task 2): "0xFF <hi> <lo>" = n campioni persi.
                # Fast path: b"\xff" in data è una ricerca in C, costo nullo.
                if b"\xff" in data:
                    clean, lost, _carry = _decode_markers(data, _carry)
                    lost_total += lost
                    data = clean
                    if not data:
                        continue
                ring_push(data)
            else:
                # modalità legacy OSC: scarta header (AA + 2) e byte > 0x3F
                if b"\xff" in data:
                    clean, lost, _carry = _decode_markers(data, _carry)
                    lost_total += lost
                    data = clean
                    if not data:
                        continue
                valid = bytearray()
                skip = 0
                for b in data:
                    if skip > 0: skip -= 1; continue
                    if b == 0xAA: skip = 2; continue
                    if b > 0x3F: continue
                    valid.append(b)
                if valid:
                    ring_push(bytes(valid))
                    data = bytes(valid)
                else:
                    continue

            cnt_stat += len(data)
            now = time.time()
            if now - t_stat >= 1.0:
                krate = round(cnt_stat / (now - t_stat) / 1000, 1)
                cur_rate = krate
                try: stream_queue.put_nowait({"type": "stats", "rate": krate,
                                              "total": total_idx,
                                              "period": cur_period_us,
                                              "lost": lost_total})
                except queue.Full: pass
                t_stat = now; cnt_stat = 0

        # connessione persa (o running=False): chiudi e — se era una caduta — riconnetti
        serial_close()
        stream_queue.put_nowait({"type": "status", "connected": False,
                                 "streaming": False, "reconnecting": running})
        if not running:
            running = False
            print("[LA] Worker terminato", flush=True)
            return
        print("[LA] Stream caduto — riconnessione automatica in 1.5s...", flush=True)
        time.sleep(1.5)
    # il loop esterno while running: riprova il collegamento

# ── Pusher SSE: solo campioni nuovi, hex-encoded ──────────────
MAX_CHUNK = 30000   # byte per messaggio SSE (60KB hex)

def _evt_chunk(i0, max_evt=4000):
    """Eventi EVT dall'indice ASSOLUTO i0: ritorna (payload, nuovo_cursore).
    Il cursore appartiene al CHIAMANTE: ogni connessione legge lo stato
    condiviso per conto suo, senza rubare eventi alle altre connessioni.
    I delta in tick tengono il payload ~7x più piccolo dello stato assoluto."""
    with evt_lock:
        base = evt_total - len(evt_t)
        if i0 < base:
            i0 = base                       # client troppo indietro: gap dichiarato
        k0 = i0 - base
        if k0 >= len(evt_t):
            return None, evt_total
        n = min(len(evt_t) - k0, max_evt)
        ts = evt_t[k0:k0 + n]
        sh = bytes(evt_s[k0:k0 + n]).hex()
        first = ts[0]
        dl = [0] * n
        for k in range(1, n):
            dl[k] = ts[k] - ts[k - 1]
        lost = evt_lost; glitch = evt_glitch; epoch = evt_epoch; total = evt_total
        isr = evt_isrmax
    return ({"type": "evt", "i": i0, "t0": first, "tick_us": EVT_TICK_US,
             "dl": dl, "sh": sh, "lost": lost, "glitch": glitch,
             "isr_us": isr, "epoch": epoch, "total": total}, i0 + n)

# ── API REST ───────────────────────────────────────────────────

@app.route("/")
def serve_index():
    return send_from_directory(app.static_folder, "index.html")

@app.route("/api/evt/start")
def api_evt_start():
    """Passa allo stream a TRANSIZIONI (risoluzione 0.5 µs, idle = zero byte).
    Il worker corrente viene fermato e riavviato: il cambio di motore (Timer1
    CTC ↔ Pin Change Interrupt) richiede di riconfigurare lo sketch."""
    global stream_mode, running
    running = False
    try:
        with ser_lock:
            if ser and ser.is_open:
                ser.write(b"X\n")
    except Exception:
        pass
    if worker_th:
        worker_th.join(timeout=3)
    stream_mode = 'evt'
    return api_start()

@app.route("/api/evt/stop")
def api_evt_stop():
    """Torna al campionamento fisso (RAW)."""
    global stream_mode, running
    running = False
    try:
        with ser_lock:
            if ser and ser.is_open:
                ser.write(b"X\n")
    except Exception:
        pass
    if worker_th:
        worker_th.join(timeout=3)
    stream_mode = 'raw'
    return api_start()

@app.route("/api/evt/backfill/<int:n>")
def api_evt_backfill(n):
    """Ultimi n eventi (per chi si collega a stream in corso)."""
    with evt_lock:
        base = evt_total - len(evt_t)
        i0 = max(base, evt_total - n)
        k0 = i0 - base
        ts = evt_t[k0:]
        sh = bytes(evt_s[k0:]).hex()
        lost = evt_lost; glitch = evt_glitch; epoch = evt_epoch; total = evt_total
        isr = evt_isrmax
    if not ts:
        return {"i": 0, "n": 0, "tick_us": EVT_TICK_US, "dl": [], "sh": "",
                "lost": lost, "glitch": glitch, "isr_us": isr,
                "epoch": epoch, "total": total}
    dl = [0] * len(ts)
    for k in range(1, len(ts)):
        dl[k] = ts[k] - ts[k - 1]
    return {"i": i0, "n": len(ts), "tick_us": EVT_TICK_US,
            "t0": ts[0], "dl": dl, "sh": sh, "lost": lost,
            "glitch": glitch, "isr_us": isr, "epoch": epoch, "total": total,
            "nomi": NOME, "colori": COLORE}

@app.route("/api/evt/stats")
def api_evt_stats():
    """Misure ad ALTA RISOLUZIONE sugli eventi: per ogni canale fronti e
    durate di impulso alto/basso in µs (min/medio/max). È il valore aggiunto
    di EVT: la risoluzione è 0.5 µs, non il periodo di campionamento.
    Analizza gli ultimi 60000 eventi per restare veloce."""
    NMAX = 60000
    with evt_lock:
        ts = list(evt_t[-NMAX:]) if len(evt_t) > NMAX else list(evt_t)
        ss = bytes(evt_s[-NMAX:]) if len(evt_s) > NMAX else bytes(evt_s)
        lost = evt_lost; total = evt_total; glitch = evt_glitch; epoch = evt_epoch
        isr = evt_isrmax
    if len(ts) < 2:
        return {"events": len(ts), "total": total, "lost": lost,
                "glitch": glitch, "isr_us": isr, "epoch": epoch,
                "span_us": 0.0, "tick_us": EVT_TICK_US, "canali": []}
    span_us = (ts[-1] - ts[0]) * EVT_TICK_US
    canali = []
    for bit in range(6):
        st = None; t_edge = 0; hi = []; lo = []; edges = 0
        for t, s in zip(ts, ss):
            v = (s >> bit) & 1
            if st is None:
                st = v; t_edge = t; continue
            if v != st:
                edges += 1
                dur = (t - t_edge) * EVT_TICK_US
                (hi if st else lo).append(dur)
                st = v; t_edge = t
        def _st(a):
            if not a: return None
            return {"n": len(a), "min": round(min(a), 2),
                    "med": round(sum(a) / len(a), 2), "max": round(max(a), 2)}
        # ATTENZIONE all'indice: NOME è ordinato per CANALE (CH0=RST(D13) ...
        # CH5=RXD(D8)), e il canale ch corrisponde al bit (5-ch).
        # Quindi il nome del bit b è NOME[5-b], non NOME[b].
        canali.append({"bit": bit, "nome": NOME[5 - bit], "colore": COLORE[5 - bit],
                       "edges": edges,
                       "freq_hz": round(edges / 2 / (span_us / 1e6), 1) if span_us and edges else 0,
                       "alto": _st(hi), "basso": _st(lo)})
    return {"events": len(ts), "total": total, "lost": lost,
            "glitch": glitch, "isr_us": isr, "epoch": epoch,
            "span_us": round(span_us, 2), "tick_us": EVT_TICK_US,
            "canali": canali}

@app.route("/api/status")
def api_status():
    with ser_lock:
        _open = ser is not None and ser.is_open
    with evt_lock:
        _evt_n = len(evt_t)
    return {
        "connected": bool(running and _open), "baud": baud, "timer": timer_us,
        "mask": mask_val, "resolution": resol,
        "total": total_idx, "port": PORT,
        "nomi": NOME, "colori": COLORE,
        "period_us": cur_period_us, "rate_hz": round(1e6 / cur_period_us, 1),
        "lost": lost_total,
        "mode": stream_mode,
        "evt": {"events": _evt_n, "total": evt_total, "lost": evt_lost,
                "glitch": evt_glitch, "isr_us": evt_isrmax, "epoch": evt_epoch,
                "tick_us": EVT_TICK_US},
        # Client SSE collegati: se il numero è più alto delle tab aperte ci sono
        # connessioni "zombie" (pagine chiuse senza che il socket sia stato
        # rilevato). Con 6 connessioni Chrome blocca le richieste della pagina
        # nuova in coda → la pagina sembra lenta senza motivo.
        "sse_clients": len(stream_queue.subs),
    }

@app.route("/api/config/<path:params>")
def api_config(params=""):
    global baud, timer_us, mask_val, resol
    for p in params.split("/"):
        if not p: continue
        cmd = p[0].upper(); val = p[1:]
        if cmd == "B" and val.isdigit(): baud = int(val)
        elif cmd == "T" and val.isdigit(): timer_us = int(val)
        elif cmd == "R" and val in ("1","8"): resol = int(val)
        elif cmd == "M":
            if len(val) == 6 and all(c in "01" for c in val):
                m = 0
                for i, c in enumerate(val):
                    if c == "1": m |= (1 << (5 - i))
                mask_val = m
            else:
                try: mask_val = int(val) & 0x3F
                except: pass
    # invia config ad Arduino se lo stream non è attivo
    if not running:
        with ser_lock:
            _already = ser is not None and ser.is_open
        if _already or serial_open():
            for c, v in (("B",baud),("T",timer_us),("M",f"{mask_val:06b}"),("R",resol)):
                with ser_lock:
                    ser.write(f"{c}{v}\n".encode())
                time.sleep(0.05)
            with ser_lock:
                ser.read(1024)
    return api_status()

@app.route("/api/serialcmd")
def api_serialcmd():
    """Comando raw all'analizzatore (es. Z = HI-Z, U = pull-up)."""
    c = request.args.get("c", "")
    if not c or any(x in c for x in "\r\n"):
        return {"error": "comando vuoto"}
    with ser_lock:
        ok = ser is not None and ser.is_open
        if ok:
            ser.write((c + "\n").encode())
            time.sleep(0.1)
            resp = ser.read(256).decode(errors="replace").strip()
        else:
            resp = ""
    return {"ok": ok, "resp": resp}

@app.route("/api/start")
def api_start():
    global running, worker_th, total_idx, ring_head, sent_idx, lost_total
    if running:
        return {"ok": True, "already": True}   # già attivo: nessun errore
    with ring_lock:
        total_idx = 0; ring_head = 0; sent_idx = 0
    lost_total = 0
    stream_queue.drain()
    stream_queue.put_nowait({"type": "reset"})
    running = True
    worker_th = threading.Thread(target=worker, daemon=True)
    worker_th.start()
    return {"ok": True}

@app.route("/api/clear")
def api_clear():
    """Svuota ring + avvisa i client: scarta la STORIA senza toccare la seriale.
    Il worker continua a streammare: da qui in poi solo dati freschi."""
    global total_idx, ring_head, sent_idx, lost_total
    with ring_lock:
        total_idx = 0; ring_head = 0; sent_idx = 0
    lost_total = 0
    stream_queue.drain()          # la storia è già scartata azzerando il ring
    stream_queue.put_nowait({"type": "reset"})
    return {"ok": True}

@app.route("/api/edges")
def api_edges():
    """Misura Deterministica lato server: edges + frazione HIGH per un bit del ring.
    /api/edges?bit=5&n=58000 -> {edges, high_pct, samples, rate}
    NON passa dal browser: zero interferenze di buffering/rendering."""
    bit = int(request.args.get("bit", 5))
    n = min(int(request.args.get("n", 58000)), RING_SIZE - 1)
    if not (0 <= bit <= 5):
        return {"error": "bit 0..5"}
    # aspetta n campioni NOVIMISURANDO il wall-time della finestra: immune al jitter
    # del delivery USB (cur_rate stimato a 1s balla; il muro no)
    t0 = time.time()
    with ser_lock:
        idx0 = total_idx
    # margine M: il delivery USB ritarda l'ultimo chunk (~30ms) -> aspetto M campioni
    # extra e scalo il tempo: elimina il bias sistematico del latency
    M = min(2000, n // 4)
    target = idx0 + n + M
    deadline = t0 + max(0.05, (n + M) / 30000.0) + 2.0
    while time.time() < deadline:
        with ser_lock:
            if total_idx >= target:
                break
            cnt_avail = ring_cnt()
        time.sleep(0.01)
    with ser_lock:
        data, _ = ring_get(idx0, min(n, max(total_idx - idx0, 0)))
        n_eff = len(data)
    t1 = time.time()
    # BASE DEI TEMPI REALE (task 1): il periodo viene dallo sketch (Timer1),
    # non più da una costante calibrata a mano (era 45800 S/s). Il wall-time del
    # PC resta inaffidabile per la latenza di delivery USB; il conteggio no.
    sec = n_eff * cur_period_us / 1e6 if n_eff else 0
    mask = 1 << bit
    edges = 0
    high = 0
    prev = (data[0] & mask) >> bit if n_eff else 0
    for b in data:
        v = (b & mask) >> bit
        if v: high += 1
        if v != prev: edges += 1
        prev = v
    return {"edges": edges, "high_pct": round(100.0 * high / n_eff, 2) if n_eff else 0,
            "samples": n_eff, "sec": round(sec, 4), "rate": cur_rate,
            "freq_hz": round(edges / 2 / sec, 1) if sec else 0}

@app.route("/api/backfill/<int:n>")
def api_backfill(n):
    """Ultimi n campioni raw (hex) — per chi si collega a stream in corso."""
    n = min(n, RING_SIZE)
    data, gap = ring_get(max(0, total_idx - n), min(n, ring_cnt()))
    return {"i": total_idx - len(data), "d": data.hex()}

@app.route("/api/stop")
def api_stop():
    global running
    running = False
    # sveglia lo sketch dal RAW: senza X resta a streamare e le config vengono ignorate
    try:
        with ser_lock:
            if ser and ser.is_open:
                ser.write(b"X\n")
    except Exception:
        pass
    if worker_th: worker_th.join(timeout=3)
    return {"ok": True}

def _run_evt():
    """Loop di streaming EVT: decodifica le transizioni e le accumula
    nell'anello eventi. Ritorna quando running=False o si cambia modalità.
    Non tocca il ring dei campioni: EVT ha il suo spazio (evt_t / evt_s)."""
    global evt_total, evt_lost, evt_sent, evt_epoch, evt_glitch, evt_isrmax
    dec = EvtDecoder()
    with evt_lock:
        evt_t.clear(); evt_s.clear()
    with ser_lock:
        evt_total = 0; evt_sent = 0; evt_lost = 0; evt_glitch = 0; evt_isrmax = 0
        # EPOCH: firma del "run". Cambiandolo, il frontend capisce che la
        # timeline è ripartita e deve SCARTARE gli eventi vecchi invece di
        # appenderli (era il bug: due timeline mescolate → durate assurde).
        evt_epoch += 1

    if not running:
        return

    with ser_lock:
        s = ser
        if s is None or not s.is_open:
            return
        try:
            s.write(b"X\n"); time.sleep(0.25); s.read(512)
            s.reset_input_buffer()
            s.write(b"EVT\n")
            resp = b""
            t0 = time.time()
            while time.time() - t0 < 1.5:
                resp += s.read(64)
                if b"\n" in resp:
                    break
        except Exception:
            return
    print(f"[LA] EVT: {resp[:50]!r}", flush=True)
    stream_queue.put_nowait({"type": "status", "connected": True, "streaming": True})
    print("[LA] Stream EVT avviato", flush=True)

    while running and stream_mode == 'evt':
        try:
            with ser_lock:
                data = ser.read(4096)
        except Exception:
            break
        if not data:
            continue
        events, lost, glitch = dec.feed(data)
        if not events and not lost and not glitch:
            continue
        with evt_lock:
            evt_lost += lost
            evt_glitch += glitch
            if dec.isr_max > evt_isrmax: evt_isrmax = dec.isr_max
            if events:
                for t, st in events:
                    evt_t.append(t); evt_s.append(st)
                evt_total += len(events)
                n = len(evt_t)
                if n > EVT_MAX:
                    del evt_t[:n - EVT_MAX]
                    del evt_s[:n - EVT_MAX]

    with ser_lock:
        try: ser.write(b"X\n")
        except Exception: pass
    print("[LA] Stream EVT terminato", flush=True)

# ── SSE ────────────────────────────────────────────────────────

@app.route("/stream")
def stream():
    def gen():
        # Ogni connessione ha la PROPRIA coda di controllo e i PROPRI cursori.
        # I dati bulk sono letti dallo stato condiviso (ring / evt_t) tramite
        # indici assoluti: due tab aperte vedono la stessa cosa e una tab
        # zombie non può più rubare i messaggi a quella attiva.
        q = stream_queue.register()
        raw_idx = None
        evt_idx = None
        last_mode = None
        try:
            yield ("data: " + json.dumps({"type": "init", "nomi": NOME,
                    "colori": COLORE, "mode": stream_mode}) + "\n\n")
            while True:
                while True:                     # messaggi di controllo
                    try:
                        yield "data: " + json.dumps(q.get_nowait()) + "\n\n"
                    except queue.Empty:
                        break
                mode = stream_mode
                if mode != last_mode:
                    # Il backlog iniziale viaggia NELLA SSE stessa. Prima il
                    # browser si collegava (cursore = "adesso") e poi chiedeva
                    # il backfill: fra le due cose passavano decine di ms, il
                    # backfill conteneva eventi PIÙ RECENTI del primo pacchetto
                    # SSE e il frontend vedeva una timeline tornare indietro →
                    # reset spurio. Ora una sola sorgente, nessuna corsa.
                    last_mode = mode
                    if mode == 'evt':
                        evt_idx = max(0, evt_total - 60000)
                    else:
                        raw_idx = max(0, total_idx - 150000)
                if mode == 'evt':
                    payload, evt_idx = _evt_chunk(evt_idx)
                    if payload is not None:
                        yield "data: " + json.dumps(payload) + "\n\n"
                else:
                    n = total_idx - raw_idx
                    if n > 0:
                        if n > 30000:
                            n = 30000        # a pezzi: il backlog iniziale non
                                             # deve arrivare in un unico blob
                        chunk, gap = ring_get(raw_idx, n)
                        if chunk:
                            yield "data: " + json.dumps(
                                {"type": "raw", "i": raw_idx, "d": chunk.hex()}) + "\n\n"
                            raw_idx += len(chunk)
                        else:
                            raw_idx = total_idx     # oltre il ring: salta
                time.sleep(0.02)
        finally:
            stream_queue.unregister(q)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no",
                             "Access-Control-Allow-Origin": "*"})

# ── Main ───────────────────────────────────────────────────────

if __name__ == "__main__":
    # Niente thread pusher: i dati ora li pesca ogni connessione SSE col
    # proprio cursore (vedi /stream). Meno stato condiviso, meno bug.
    print("╔══ LOGIC ANALYZER · streaming ═══════════════════")
    print(f"║   http://127.0.0.1:8788")
    print(f"║   Seriale: {PORT} @ {baud}")
    print(f"║   Ring: {RING_SIZE:,} campioni · SSE incrementale")
    print("╚════════════════════════════════════════════════")
    app.run(host="127.0.0.1", port=8788, debug=False, threaded=True)