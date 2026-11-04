# Engineering notes

How this instrument was measured and improved. These notes exist because a
measurement instrument that doesn't know its own limits produces numbers that
look like data but aren't.

---

## 1. v1 → v2: the timing base was wrong

**v1** timed the sampling loop with `delayMicroseconds()` in the main loop:

```
requested T15  →  actual period 21.8 µs   (+45%, jittery)
```

The error was systematic and jitter was produced by everything else in the
loop. Every duration measured with v1 was inflated by the same amount.

**v2** moved the period to **Timer1 in CTC mode with an ISR**: the sample
instant is decided by the hardware timer, so the period is the one you ask
for, at the crystal.

```
requested T50  →  19977 samples/s  vs 20000 expected   (0.1% error)
```

The ISR deposits samples into a **512-byte ring buffer in SRAM**; the main
loop drains it to the UART without ever blocking (using `availableForWrite`).
Sampling cadence no longer depends on the UART.

### Loss counters

If the ring fills up, a sample is lost — but it is **counted** and declared
**in-band** with a marker (`0xFF <hi> <lo>`; sample data is masked to 6 bits,
so `0xFF` cannot appear among samples), rate-limited to 20/s. The backend can
query `Q` and know whether the timeline is intact, instead of calibrating by
hand.

`T0` (AUTO) probes for ~3 s, reads the converged period with `Q`, and then
**locks it**: maximum speed *and* an exact timing base (20–27 µs, zero lost
samples).

---

## 2. EVT mode: seeing faster than the sampling period

Fixed-period sampling cannot resolve a signal faster than its own period.
A 125 kHz SPI clock has a 4 µs half-period against a 20 µs sample period: it
**aliases**.

**EVT mode** uses a Pin Change Interrupt plus Timer1 free-running with a
**0.5 µs tick**, and transmits *only state changes*, each with a hardware
timestamp. Idle costs zero bytes.

```
[0x80|d0][0x80|d1]...   delta pieces (7 bits each, little-endian)
[0x00-0x3F]             channel state (bit0=CH5 ... bit5=CH0)
[0x40-0x7F]             gap: (value & 0x3F) events lost to a full ring
```

Resolution: **~0.5 µs instead of 21.8 µs — about 44× better.**

### Glitch accounting

Transitions shorter than the ISR latency are not observable. With pins in
high impedance, capacitive coupling raises the PCINT flag but by the time the
ISR reads the pin it is already back to its previous value. Such a change
**cannot be observed**, so it is counted rather than invented as an event
(marker `0x7F <hi> <lo>`, one report per second).

Before this correction, roughly **50% of EVT events had a state identical to
the previous one** — the instrument was reporting edges that did not exist.

---

## 3. v2.1: the resolution limit was the instrument, not the signal

### The symptom

The minimum observable pulse was **26 µs on every channel**, identical on
rising and falling edges. An identical minimum across unrelated signals is a
symptom: it belongs to the instrument.

### The cause

The PCINT ISR did **everything**: read a 32-bit timestamp, read the pins,
filter glitches, and encode the event with variable-length encoding into the
ring. **~190 cycles ≈ 12 µs** (measured: 29 µs worst case).

Two edges closer together than the ISR duration cannot be separated — they
are **merged into a single event**. Every duration near 26/34/46 µs was an
artefact.

### The fix

The ISR now computes **nothing**: it reads `TCNT1` and `PINB` and stores them
in a lock-free circular buffer (128 entries × 3 bytes).

```cpp
ISR(PCINT0_vect) {
  uint16_t t0 = TCNT1;
  byte nxt = (capIn + 1) & CAP_MASK;
  if (nxt == capOut) { nCapLost++; return; }   // buffer full: declared, not hidden
  capS[capIn] = PINB;
  capT[capIn] = t0;
  capIn = nxt;
}
```

The main loop (`processCapture()`) reconstructs the 32-bit tick (overflows are
detected because `TCNT1` goes "backwards"; captures are in chronological
order), filters glitches, and encodes into the ring.

### Result

| | Before | After |
|---|---|---|
| ISR duration | 29 µs | **2 µs** (14×) |
| Minimum observable pulse | 26 µs | **~7 µs** |
| MOSI edges resolved in 10 s | 64,848 | **179,847** (2.8×) |
| Glitches | 38,553 (37% of edges) | **9,109 (5%)** |

---

## 4. Proving that 13 µs is signal

A deliberate perturbation: `EVT_ISR_PAD` lengthens the ISR by N `nop`
instructions on purpose, to distinguish an instrument limit from a signal
characteristic.

| PAD | ISR (body) | Minimum observed | Signal peak |
|---|---|---|---|
| 0 | 2 µs | 7.00 µs | **13 µs** |
| 8 | 3 µs | 7.00 µs | **13 µs** |
| 20 | 7 µs | 11.50 µs | **13 µs** |

The minimum **moves** with the ISR (it is the instrument); the 13 µs peak
**does not move** (it is the signal). Derived calibration:
`limit ≈ ISR_body + 5 µs` (prologue, epilogue, interrupt entry).

### What the signal actually is

- **CH3**: 161,687 pulses of exactly 13 µs high out of 177,745 under 40 µs
  (91%). Period during bursts 26 µs → ~38 kHz, with pauses of ~674 µs.
  13 µs at 12 MHz = **13 machine cycles**: consistent with a scan cycle of
  the target firmware.
- **CH4**: pulses of **~363 µs, always identical**, separated by 1–5 s of low
  level. 363 µs ≈ 11 bits at ~30 kbaud, but all identical — a **strobe**, not
  UART traffic (which would vary in duration with the data).

> Before the ISR fix, this channel measured 389.5 µs: **+26 µs of error**,
> because all channels share PCINT0 and the fast channel's edges delayed the
> timestamps of the slow ones.

---

## 5. Known limitation (intrinsic)

**All 6 channels share a single interrupt (PCINT0 on PORTB).** A fast channel
degrades the measurement of slow channels. The short ISR reduces the effect
substantially but does not eliminate it.

Any measurement of a slow channel taken while a fast channel is switching
should be treated with suspicion. `EVT_ISR_PAD` is the tool to check it.
