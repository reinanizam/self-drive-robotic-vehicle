"""Direct firmware safety checks (synthetic inputs, no Pi vision): STOP / link loss / camera+IR loss / reverse cap / no-track cruise control.
Run:  python3 safety_test.py ./sim_v76e          (V7.6e: 11 checks, all must PASS)
      python3 safety_test.py ./sim_v76c v76c     (V7.6c: same checks, but three of them expect the V7.6c behaviour:
                                                  without the tape V7.6c stops backing off, it does not back off blind; reverse cap 2.5 s)"""
import sys
from closedloop import Firmware
exe = sys.argv[1]
V76C = len(sys.argv) > 2 and sys.argv[2] == 'v76c'
DT = 5000
def mk():
    fw = Firmware(exe); fw.init(393, 4); return fw
class T:
    def __init__(s, fw): s.fw = fw; s.t = 0.0; s.out = None; s.dist = 999; s.mask = 4; s.guide = 'GUIDE5,0,0,95,0,70,0'; s.hist = []
    def step(s, n=1, guide='auto', dist=None):
        for _ in range(n):
            g = None
            if guide == 'auto':
                if int(s.t / 0.05) != int((s.t - DT/1e6) / 0.05): g = s.guide      # 20 Hz
            else: g = guide
            d = s.dist if dist is None else dist
            echo = 0 if d >= 999 else int(d * 58)
            s.out, tele = s.fw.step(393, s.mask, echo, DT, g)
            s.t += DT / 1e6
            s.hist.append((s.t, s.out[0], s.out[1], s.out[2]))
        return s.out
    def secs(s, sec, **k): return self_step(s, int(sec / (DT/1e6)), **k)
def self_step(s, n, **k): return s.step(n, **k)
ok = True
def check(name, cond):
    global ok; ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name)
def drive_and_back_off(t):
    t.step(int(1.5/0.005))                           # arm + drive
    assert t.out[2] >= 0
    for d in (80, 60, 44, 40, 38):                   # object appears and stays still
        t.dist = d; t.step(int(0.15/0.005))
    t.step(int(1.0/0.005))                           # stopped, observing
    # object comes toward the car: 38 -> 20 in 0.9 s (20 cm/s)
    d = 38.0
    for i in range(int(0.9/0.005)):
        d -= 20 * 0.005; t.dist = d; t.step(1)
    t.dist = max(d, 18)
# 1) back off happens and STOP ends it at once
t = T(mk()); drive_and_back_off(t)
t.step(int(0.4/0.005))
rev = t.out[2] < 0 and t.out[0] > 0
check('reverse running before STOP', rev)
t.step(1, guide='STOP')
check('STOP -> both PWM 0 in the same step', t.out[0] == 0 and t.out[1] == 0)
t.step(int(0.5/0.005), guide='STOP'); check('stays stopped under STOP', t.out[0] == 0 and t.out[1] == 0)
# 2) link loss (no guide lines) while reversing
t = T(mk()); drive_and_back_off(t); t.step(int(0.4/0.005))
check('reverse running before link loss', t.out[2] < 0 and t.out[0] > 0)
t.guide = None; stopped_at = None
for i in range(int(1.0/0.005)):
    t.step(1, guide=None)
    if t.out[0] == 0 and t.out[1] == 0: stopped_at = i * 0.005; break
check('no Pi lines for 0.4 s -> motors 0 (stopped after %.2f s)' % (stopped_at or -1), stopped_at is not None and stopped_at <= 0.5)
# 3) camera lost + IR lost while reversing: back-off continues on the gyro
t = T(mk()); drive_and_back_off(t); t.step(int(0.3/0.005))
t.guide = 'TRACK_LOST'; t.mask = 0
t.step(int(0.3/0.005))
if V76C: check('TRACK_LOST + IR empty: V7.6c stops backing off (motors 0)', t.out[0] == 0 and t.out[1] == 0)
else: check('TRACK_LOST + IR empty: still backing off', t.out[2] < 0 and t.out[0] > 0)
# 4) reverse safety cap (object keeps coming forever)
t = T(mk()); drive_and_back_off(t)
t0 = t.t; last_rev = t.t
for i in range(int(12/0.005)):
    t.dist = 18; t.step(1)
    if t.out[2] < 0 and t.out[0] > 0: last_rev = t.t
if V76C: check('reverse stops at the safety limit (last reverse motion %.1f s after the back-off began; cap 2.5 s of motion)' % (last_rev - t0), last_rev - t0 < 3.5)
else: check('reverse stops at the safety limit (last reverse motion %.1f s after the back-off began; cap 4 s of motion)' % (last_rev - t0), 2.5 < last_rev - t0 < 6.0)
# 5) nothing in front, no track verified at start (IR empty, camera lost): car does NOT move
t = T(mk()); t.guide = 'TRACK_LOST'; t.mask = 0; t.step(int(2/0.005))
check('no track, nothing in front: motors 0', t.out[0] == 0 and t.out[1] == 0)
# 6) track confirmed a moment ago, then lost (camera blocked by the car ahead): the car stops, an object comes toward it -> it backs off on the gyro, never forward
t = T(mk()); t.step(int(1.5/0.005))
t.guide = 'TRACK_LOST'; t.mask = 0; t.dist = 40; t.step(int(0.5/0.005))
d = 40.0
for i in range(int(0.8/0.005)): d -= 20 * 0.005; t.dist = d; t.step(1)
t.dist = 18; t.step(int(0.8/0.005))
fwd = any(h[3] > 0 and (h[1] or h[2]) for h in t.hist[int(2.1/0.005):])
if V76C: check('track lost behind a car + object approaching: V7.6c does not move (motors 0)', t.out[0] == 0 and t.out[1] == 0)
else: check('track lost behind a car + object approaching: backs off', t.out[2] < 0 and t.out[0] > 0)
check('track lost: never drives forward', not fwd)
# 7) never had a track (car on a table, hand coming): it must not move at all
t = T(mk()); t.guide = 'TRACK_LOST'; t.mask = 0; t.dist = 40; t.step(int(1/0.005))
d = 40.0
for i in range(int(0.8/0.005)): d -= 20 * 0.005; t.dist = d; t.step(1)
t.dist = 18; t.step(int(1.0/0.005))
check('track never confirmed + object approaching: motors stay 0', all(h[1] == 0 and h[2] == 0 for h in t.hist))
print('ALL PASS' if ok else 'SOME FAILED')
