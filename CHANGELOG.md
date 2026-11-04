# Changelog

Notable changes to this project. The interesting part of this repository is not
the feature list — it is *how the limits of the instrument were found and fixed*.
See [docs/engineering-notes.md](docs/engineering-notes.md) for the full story.

---

## v2.1 — The resolution limit was the instrument, not the signal

### Fixed
- **Interrupt service routine rewritten to be minimal** (29 µs → 2 µs). The ISR
  previously read a 32-bit timestamp, read the pins, filtered glitches and
  encoded the event into the ring buffer — all in one go. Two edges closer
  together than the ISR duration were merged into a single event, so every
  duration near 26/34/46 µs was an artefact, not a measurement.
  The ISR now only stores `TCNT1` and `PINB` into a lock-free circular buffer;
  decoding and glitch filtering happen in the main loop.
- **Minimum observable pulse: 26 µs → ~7 µs** (identical on every channel
  before, which is what gave the artefact away).
- Edges resolved on a fast channel in 10 s: 64,848 → **179,847** (2.8×).
- Glitches: 37% of edges → **5%**.

### Added
- **`EVT_ISR_PAD` build option** — deliberately lengthens the ISR by N `nop`
  instructions, to distinguish an instrument limit from a signal
  characteristic. A verified minimum *moves* with the ISR; a real signal does
  not. Derived calibration: `limit ≈ ISR_body + 5 µs`.

---

## v2 — Hardware timing base, transition capture, honest counters

### Added
- **Timer1 in CTC mode with an ISR** drives the sampling period, replacing
  `delayMicroseconds()` in the main loop. Timing error: +45% → **0.1%**
  (T50 measures 19977 samples/s against 20000 expected).
- **512-byte ring buffer in SRAM**, drained to the UART without blocking
  (`availableForWrite`), so sampling cadence no longer depends on the UART.
- **`T0` (AUTO) mode** — probes for the maximum rate the serial link sustains
  without losing samples, then locks it.
- **EVT mode** — transition-only encoding: pin-change interrupt plus a
  free-running Timer1 with a **0.5 µs tick**. Only state changes are sent, each
  with a hardware timestamp; idle costs zero bytes. This is the only way to see
  signals faster than the sampling period (~44× better resolution).
- **Loss and glitch counters, declared in-band.** Dropped samples
  (`0xFF <hi> <lo>`), dropped events and unobservable glitches
  (`0x7F <hi> <lo>`) are counted and reported. Sample bytes are masked to 6
  bits, so these markers can never be confused with data.
- `Q` command to query the counters.
- **Minimal web UI redesign**: EVT view with a time grid, frame-throttled
  drawing, binary search for the first visible event, one cursor per SSE
  connection (the initial backlog travels inside the same SSE stream — a single
  source of truth, no race with the backfill endpoint).

### Fixed
- Channels default to **hi-Z** (plain `INPUT`) instead of `INPUT_PULLUP`: the
  instrument must not influence the circuit under test. Pull-ups are available
  on demand via the `U` command.
- Removed a stray interrupt enable (`UCSR0B |= (3 << UDRIE0)`) that had no
  service routine — a latent reset hazard.

---

## v1 — First working version

### Added
- 6 digital channels (D13–D8) streamed over serial at up to 2 Mbaud.
- Runtime baud rate change via UBRR (`B`), selectable sampling period (`T`),
  channel mask (`M`), resolution (`R`).
- `RAW` (no framing) and `OSC` (periodic sync header) streaming modes.
- Configuration commands and `STATUS` readout.

### Known issues
- Sampling timing was +45% off and jittery (measured, not assumed).
- No loss accounting: the instrument could not tell whether its own timeline
  was intact.
