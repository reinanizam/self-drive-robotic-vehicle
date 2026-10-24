"""Extra checks of the Arduino-only ACC demo.
Usage:  python3 run_demo_extra.py <check>      check = ng | cruise | budget | sign | start
  ng      gyro absent / gyro shaking at start-up (needs sim_acc_demo_gyro_absent and sim_acc_demo_gyro_shaking, see README.md)
  cruise  nothing ahead: the car must stop by itself after 20 s of cruising
  budget  lead keeps coming faster than the car backs off: backing stops after its 5 s limit
  sign    gyro wired the wrong way
  start   crooked start and weaker side
All except ng use ./sim_acc_demo.
"""
import sys, io, contextlib, os
import acc2
HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.environ.get('PI', os.path.join(HERE, '..', 'raspberry_pi', 'mavp_pi.py'))
def run(exe, label, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        r, out = acc2.scenario(exe, PI, label, verbose=False, **kw)
    rv = f"rev {out['rev_cm']:5.1f}cm x<={out['rev_x']:.1f} hdg<={out['rev_hdg']:.1f} t={out['rev_t'][0]:.1f}-{out['rev_t'][1]:.1f}s" if 'rev_cm' in out else 'NO REVERSE'
    print(f"{label:34s} {rv:50s} min_gap={out['min_gap']:6.1f} fwd={out['followed_fwd']} end_y={out['end_y']:.0f} max_cross={out['max_cross']:.1f}", flush=True)
    return r, out
if __name__ == '__main__':
    if len(sys.argv) < 2: sys.exit(__doc__)
    t = sys.argv[1]
    if t == 'ng':
        run('./sim_acc_demo_gyro_absent', 'N1 gyro ABSENT, default')
        run('./sim_acc_demo_gyro_absent', 'N2 gyro ABSENT, 5% weak', bank_gain=(1.0, 0.95))
        run('./sim_acc_demo_gyro_shaking', 'N3 gyro shaking in setup, default')
    elif t == 'cruise':
        r, o = run('./sim_acc_demo', 'T2 no car ahead 40 s', y0=60.0, legs=(), dur=40.0, track_len=8.0)
        print('   car stopped at y=%.0f cm' % o['end_y'])
        rows = r.log
        stop_t = next((l[0] for l in rows if l[0] > 5 and abs(l[4]) < 0.004), None)
        print('   first stand-still after start: t=%s' % stop_t)
    elif t == 'budget':
        run('./sim_acc_demo', 'T3 lead keeps coming 0.35 m/s', legs=((9.0, -0.35, 12.0),), dur=30.0)
    elif t == 'sign':
        run('./sim_acc_demo', 'T4 gyro sign REVERSED, default', geo_over=dict(gyro_sign=-1.0))
        run('./sim_acc_demo', 'T4b gyro sign REVERSED, 5% weak', geo_over=dict(gyro_sign=-1.0), bank_gain=(1.0, 0.95))
    elif t == 'start':
        run('./sim_acc_demo', 'T5 start 4 deg crooked', start=(0.0, 0.0, 0.07))
        run('./sim_acc_demo', 'T5b start crooked, 15% weak', start=(0.0, 0.0, 0.07), bank_gain=(1.0, 0.85))
        run('./sim_acc_demo', 'T5c 30% weak', bank_gain=(1.0, 0.70))
        run('./sim_acc_demo', 'T5d fast lead 0.2 m/s', legs=((9.0, -0.20, 2.0), (16.0, +0.10, 4.0)))
