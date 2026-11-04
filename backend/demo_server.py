"""
Demonstration backend — runs the web UI without any hardware.

It generates a synthetic 6-channel signal with the same *shapes* the
instrument was built to observe (short pulses in bursts, a slow strobe, a
clock with data), and serves it through exactly the same HTTP + SSE API as
`server.py`. Use it to try the interface, to develop the front-end, or to
take screenshots.

    python demo_server.py      →  http://127.0.0.1:8788

The generated timeline is deliberately scaled down: durations (in
microseconds) are realistic, but wall-clock time advances slower so that a
browser can render it comfortably.
"""
import json
import os
import queue
import random
import threading
import time
from collections import deque

from flask import Flask, Response, jsonify, send_from_directory

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(__name__, static_folder=os.path.join(BASE_DIR, "static"))

TICK_US = 0.5                     # EVT resolution: same as the real firmware
NOMI = ["CH0 (D13)", "CH1 (D12)", "CH2 (D11)", "CH3 (D10)", "CH4 (D9)", "CH5 (D8)"]
COLORI = ["#ffd700", "#44cc44", "#4488ff", "#eeeeee", "#ff4444", "#66cc66"]

MAX_EVENTS = 400_000
evt_lock = threading.Lock()
evt_t = deque(maxlen=MAX_EVENTS)          # absolute tick of each event
evt_s = deque(maxlen=MAX_EVENTS)          # channel state after the event
evt_total = 0                             # absolute index of the next event
evt_lost = 0
evt_glitch = 0
evt_isrmax = 2
evt_epoch = 1

_subs = []                                # SSE control queues
_subs_lock = threading.Lock()


def push(tick, state):
    """Append one event (called only by the generator thread)."""
    global evt_total
    with evt_lock:
        evt_t.append(tick)
        evt_s.append(state)
        evt_total += 1


def generator():
    """Build one repeating frame of synthetic activity, scaled down.

    Frame = 2000 ticks (1 ms of signal time). Realistic durations:
      CH1 (D12)  clock, half period 16 ticks = 8 us
      CH3 (D10)  short pulses, 26 ticks high, period 52 ticks = 13 us / 26 us
      CH2 (D11)  data changing on clock edges
      CH4 (D9)   strobe 726 ticks high every 100 frames = 363 us
      CH5 (D8)   low 200 ticks every 5 frames
    """
    global evt_total
    frame = 0
    tick = 0
    state = 0
    rng = random.Random(7)

    while True:
        # ---- CH4 (D9): slow strobe, high for 363 us every ~100 frames ----
        if frame % 100 == 0:
            tick += 40
            state |= (1 << 1)                       # CH4 high
            push(tick, state)
            tick += 726                             # 363 us
            state &= ~(1 << 1) & 0xFF
            push(tick, state)

        # ---- CH5 (D8): occasional low pulse ----
        if frame % 5 == 0:
            tick += 30
            state &= ~1
            push(tick, state)
            tick += 200
            state |= 1
            push(tick, state)

        # ---- burst of activity ----
        for _ in range(40):
            tick += 16                              # CH1 rising edge
            state |= (1 << 4)
            if rng.random() < 0.5:                  # data changes with the clock
                state ^= (1 << 3)
            push(tick, state)

            tick += 16                              # clock falling edge
            state &= ~(1 << 4) & 0xFF
            push(tick, state)

            # CH3: 13 us high, 13 us low (period 26 us)
            tick += 4
            state |= (1 << 2)
            push(tick, state)
            tick += 26
            state &= ~(1 << 2) & 0xFF
            push(tick, state)
            tick += 4

        # pause between bursts (~674 us), then next frame
        tick += 1348
        frame += 1
        time.sleep(0.03)   # throttle: ~33 frame/s, la CPU resta libera


def broadcaster():
    """Push new events to every connected client (control queue)."""
    last = 0
    while True:
        time.sleep(0.1)
        with evt_lock:
            total = evt_total
            base = total - len(evt_t)
            if total <= last:
                last = min(last, total)
                continue
            k0 = max(0, last - base)
            ts = list(evt_t)[k0:]
            ss = bytes(evt_s)[k0:]
            lost, glitch, epoch = evt_lost, evt_glitch, evt_epoch
            isr = evt_isrmax
        if not ts:
            continue
        i0 = base + k0
        dl = [0] * len(ts)
        for k in range(1, len(ts)):
            dl[k] = ts[k] - ts[k - 1]
        msg = {"type": "evt", "i": i0, "t0": ts[0], "tick_us": TICK_US,
               "dl": dl, "sh": ss.hex(), "lost": lost, "glitch": glitch,
               "isr_us": isr, "epoch": epoch, "total": total}
        last = total
        with _subs_lock:
            for q in list(_subs):
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    pass


# ── HTTP ───────────────────────────────────────────────────────

@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/api/status")
def status():
    with evt_lock:
        n = len(evt_t)
        total, lost, glitch = evt_total, evt_lost, evt_glitch
    return jsonify({
        "connected": True, "baud": 2000000, "timer": 0, "mask": 63, "resolution": 1,
        "total": total, "port": "DEMO", "nomi": NOMI, "colori": COLORI,
        "period_us": 0, "rate_hz": 0, "lost": 0, "mode": "evt",
        "evt": {"events": n, "total": total, "lost": lost, "glitch": glitch,
                "isr_us": 2, "epoch": evt_epoch, "tick_us": TICK_US},
        "sse_clients": len(_subs),
    })


@app.route("/api/start")
@app.route("/api/evt/start")
@app.route("/api/stop")
@app.route("/api/evt/stop")
@app.route("/api/clear")
@app.route("/api/config/<path:params>")
def noop(params=""):
    return jsonify({"ok": True, "demo": True})


@app.route("/api/evt/backfill/<int:n>")
def evt_backfill(n):
    with evt_lock:
        total = evt_total
        base = total - len(evt_t)
        i0 = max(base, total - n)
        k0 = i0 - base
        ts = list(evt_t)[k0:]
        ss = bytes(evt_s)[k0:]
        lost, glitch, epoch = evt_lost, evt_glitch, evt_epoch
    dl = [0] * len(ts)
    for k in range(1, len(ts)):
        dl[k] = ts[k] - ts[k - 1]
    return jsonify({"i": i0, "n": len(ts), "tick_us": TICK_US,
                    "t0": ts[0] if ts else 0, "dl": dl, "sh": ss.hex(),
                    "lost": lost, "glitch": glitch, "isr_us": evt_isrmax,
                    "epoch": epoch, "total": total,
                    "nomi": NOMI, "colori": COLORI})


@app.route("/api/evt/stats")
def evt_stats():
    """Per-channel edge and pulse statistics — identical shape to server.py."""
    NMAX = 20000
    with evt_lock:
        ts = list(evt_t)[-NMAX:]
        ss = bytes(evt_s)[-NMAX:]
        total, lost, glitch = evt_total, evt_lost, evt_glitch
    canali = []
    span_us = (ts[-1] - ts[0]) * TICK_US if len(ts) > 1 else 0.0
    if len(ts) > 1:
        for bit in range(6):
            st = None
            t_edge = 0
            hi, lo, edges = [], [], 0
            for t, s in zip(ts, ss):
                v = (s >> bit) & 1
                if st is None:
                    st = v
                    t_edge = t
                    continue
                if v != st:
                    edges += 1
                    dur = (t - t_edge) * TICK_US
                    (hi if st else lo).append(dur)
                    st = v
                    t_edge = t

            def _st(a):
                if not a:
                    return None
                return {"n": len(a), "min": round(min(a), 2),
                        "med": round(sum(a) / len(a), 2), "max": round(max(a), 2)}

            # NOTE the index: NOMI is ordered by channel (CH0 first), and channel
            # ch maps to bit (5-ch) — so the name of bit b is NOMI[5-b].
            canali.append({"bit": bit, "nome": NOMI[5 - bit],
                           "colore": COLORI[5 - bit], "edges": edges,
                           "freq_hz": round(edges / 2 / (span_us / 1e6), 1)
                                      if span_us and edges else 0,
                           "alto": _st(hi), "basso": _st(lo)})
    return jsonify({"events": len(ts), "total": total, "lost": lost,
                    "glitch": glitch, "isr_us": evt_isrmax, "epoch": evt_epoch,
                    "span_us": round(span_us, 2), "tick_us": TICK_US,
                    "canali": canali})


@app.route("/stream")
def stream():
    def gen():
        q = queue.Queue(maxsize=400)
        with _subs_lock:
            _subs.append(q)
        try:
            yield "data: " + json.dumps({"type": "init", "nomi": NOMI,
                                         "colori": COLORI, "mode": "evt"}) + "\n\n"
            while True:
                try:
                    msg = q.get(timeout=5)
                    yield "data: " + json.dumps(msg) + "\n\n"
                except queue.Empty:
                    yield ": ka\n\n"
        finally:
            with _subs_lock:
                if q in _subs:
                    _subs.remove(q)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache"})


if __name__ == "__main__":
    threading.Thread(target=generator, daemon=True).start()
    threading.Thread(target=broadcaster, daemon=True).start()
    print("╔══ LOGIC ANALYZER · demo backend (nessun hardware) ══")
    print("║   http://127.0.0.1:8788")
    print("╚═════════════════════════════════════════════════════")
    app.run(host="127.0.0.1", port=8788, debug=False, threaded=True)
