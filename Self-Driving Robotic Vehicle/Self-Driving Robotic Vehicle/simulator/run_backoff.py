"""Lead-car back-off scenarios S1-S13: a car ahead stops, then moves toward the car, then drives away.
Usage (build the sketches first, see README.md):  python3 run_backoff.py v76c v76e
    runs ./sim_v76c and ./sim_v76e; prints, per case: how far the car backed off, the largest heading change,
    the closest gap and whether it drove forward again. About 6 minutes per version.
"""
import sys, os, math
from multiprocessing import Pool
import acc2
HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.environ.get('PI', os.path.join(HERE, '..', 'raspberry_pi', 'mavp_pi.py'))   # the Pi program the camera pictures go through
LIGHT = (0.20, 0.09, 215.0)
CASES = [
 ('S1 default', dict()),
 ('S2 right bank 15% weak', dict(bank_gain=(1.0, 0.85))),
 ('S3 left bank 15% weak', dict(bank_gain=(0.85, 1.0))),
 ('S4 left bank 30% weak', dict(bank_gain=(0.70, 1.0))),
 ('S5 light wide lead (camera view blocked)', dict(body=LIGHT)),
 ('S6 light lead + right 15% weak', dict(body=LIGHT, bank_gain=(1.0, 0.85))),
 ('S7 fast lead 0.2 m/s', dict(legs=((9.0, -0.20, 1.5), (16.0, +0.15, 3.0)))),
 ('S8 long back-off 0.9 m (tape behind)', dict(start=(0.0, 0.8, 0.0), y0=1.75, legs=((9.0, -0.12, 7.5), (20.0, +0.12, 4.0)), dur=36.0)),
 ('S10 camera blind from 8.5 s', dict(cam_blackout=(8.5, 40.0))),
 ('S11 camera blind from 8.5 s + left bank 30% weak', dict(cam_blackout=(8.5, 40.0), bank_gain=(0.70, 1.0))),
 ('S12 camera blind from 10.2 s (mid back-off)', dict(cam_blackout=(10.2, 40.0))),
 ('S13 right bank 30% weak', dict(bank_gain=(1.0, 0.70))),
 ('S9 2 cm off, 4 deg, right 15% weak', dict(start=(0.02, 0.0, 0.07), bank_gain=(1.0, 0.85))),
]
def go(a):
    name, fwtag, kw = a
    fw = f'./sim_{fwtag}'
    try:
        import io, contextlib
        with contextlib.redirect_stdout(io.StringIO()):
            r, out = acc2.scenario(fw, PI, name, verbose=False, **kw)
        return name, fwtag, out
    except Exception as ex:
        import traceback; traceback.print_exc()
        return name, fwtag, {'err': repr(ex)}
if __name__ == '__main__':
    tags = sys.argv[1:]            # binary names without the sim_ prefix, e.g. v76c v76e
    if not tags: sys.exit(__doc__)
    items = [(n, t, kw) for n, kw in CASES for t in tags]
    with Pool(2) as p:
        for n, t, o in p.imap(go, items):
            if 'err' in o: print(f'{n:42s} {t} ERROR {o}'); continue
            rv = f"rev {o['rev_cm']:5.1f}cm x<={o['rev_x']:.1f}cm hdg<={o['rev_hdg']:.1f}" if 'rev_cm' in o else 'NO REVERSE'
            print(f"TAB {n:42s} {t:8s} {rv:36s} min_gap={o['min_gap']:5.1f} followed_fwd={o['followed_fwd']} end_y={o['end_y']:.0f} lead_end={o['lead_end']:.0f}", flush=True)
