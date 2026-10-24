"""Octagon-lap test: the real Uno sketch + the real Pi program on a drawn octagon track (4.7 cm tape) in the closed-loop simulator.

Build the sketch first (see README.md), then:
    python3 run_octagon.py v76c          # runs ./sim_v76c, 13 cases, about 10 minutes with 2 workers
Environment: PITCH camera pitch in degrees (default 22), CAMH camera height in m (default 0.10),
             NP worker processes (default 2), PI path of the Pi program (default ../raspberry_pi/mavp_pi.py)
Expected for V7.6c / V7.6e: 9 COMPLETED, 4 stopped (IR flicker in one phase, IR dead, camera 14 cm high, car 30% slower).
"""
import sys, os, math
from multiprocessing import Pool
from dataclasses import replace
from closedloop import *
from oct import octagon
HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.environ.get('PI', os.path.join(HERE, '..', 'raspberry_pi', 'mavp_pi.py'))
BASE = dict(a_is_right=False, cam_h=float(os.environ.get('CAMH','0.10')), pitch_deg=float(os.environ.get('PITCH','22')), tape_w=0.019, ir_ahead=0.10)
def go(a):
    label, ver, verts, geo_over, kw, dur = a
    start = kw.pop('start', (0.0, 0.0, 0.0)); kw = dict(kw)
    fw, pi = f'./sim_{ver}', PI
    GEO = replace(Geo(), **{**BASE, **geo_over})
    try:
        r = run(fw, pi, Track(verts, tape_w=GEO.tape_w), GEO, 7, duration=dur, record=True, end_tol=0.18, start=start, **kw)
        hmax = max((abs(l[3]) for l in r.log), default=0); hend = r.log[-1][3] if r.log else 0
        st = 'COMPLETED' if r.completed else f'stopped {r.reason}' if r.stopped else f'{r.reason or "timeout"}'
        return label, ver, f'{st:16s} t={r.t_end:5.1f}s hend={hend:7.1f} max|h|={hmax:6.1f} cross={r.max_cross*100:5.1f}cm prog={r.progress:.2f}/{r.length:.2f} v={r.mean_speed*100:4.1f}cm/s'
    except Exception as ex:
        import traceback; return label, ver, f'ERROR {ex!r}'
def cases_tape47x():
    L7, R7 = octagon(+1), octagon(-1); c = []
    t = dict(tape_w=0.047)
    for sp in (0.013, 0.025, 0.04):
        c.append((f'T4.7 sp={sp*100:.1f} left', L7, dict(t, ir_spacing=sp), {}, 80.0))
    c.append(('T4.7 sp=1.3 right', R7, dict(t, ir_spacing=0.013), {}, 80.0))
    c.append(('T4.7 sp=2.5 right', R7, dict(t, ir_spacing=0.025), {}, 80.0))
    c.append(('T4.7 sp=1.3 mid-start', L7, dict(t, ir_spacing=0.013), dict(start=(0.0, 0.5, 0.0)), 80.0))
    c.append(('T4.7 sp=1.3 1.5cm off 2deg', L7, dict(t, ir_spacing=0.013), dict(start=(0.015, 0.0, math.radians(2))), 80.0))
    for ph in (0.0, 0.5):
        c.append((f'T4.7 IR flicker ph{ph}', L7, dict(t, ir_spacing=0.025), dict(ir_periodic=(1.0, 0.5, ph)), 80.0))
    c.append(('T4.7 IR dead', L7, dict(t), dict(ir_blank=True), 80.0))
    for ch in (0.08, 0.14): c.append((f'T4.7 sp=1.3 cam_h={ch}', L7, dict(t, ir_spacing=0.013, cam_h=ch), {}, 80.0))
    c.append(('T4.7 sp=1.3 vmax=0.28', L7, dict(t, ir_spacing=0.013, vmax=0.28), {}, 110.0))
    return c
def cases():
    return cases_tape47x()
if __name__ == '__main__':
    sel = sys.argv[1:]
    if not sel: sys.exit(__doc__)
    items = [(l, v, vt, g, kw, d) for (l, vt, g, kw, d) in cases() for v in sel]
    with Pool(int(os.environ.get("NP","2"))) as p:
        for l, v, t in p.imap(go, items): print(f'{l:34s} {v:8s} {t}', flush=True)
