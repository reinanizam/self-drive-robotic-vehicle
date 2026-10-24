"""Arduino-only ACC demo scenarios D1-D5 (lead car stands 95 cm ahead, moves 24 cm toward the car at 9 s, away 40 cm at 16 s).
Usage:  python3 run_demo.py ./sim_acc_demo      (build it first: bash build.sh ../firmware/experimental/mavp_acc_demo/mavp_acc_demo.ino sim_acc_demo)
"""
import sys, io, contextlib, os
from multiprocessing import Pool
import acc2
HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.environ.get('PI', os.path.join(HERE, '..', 'raspberry_pi', 'mavp_pi.py'))   # the demo ignores the Pi, the harness still needs a Pi program for the camera side
CASES = [
 ('D1 default', dict()),
 ('D2 right bank 5% weak', dict(bank_gain=(1.0, 0.95))),
 ('D3 left bank 5% weak', dict(bank_gain=(0.95, 1.0))),
 ('D4 right bank 15% weak', dict(bank_gain=(1.0, 0.85))),
 ('D5 left bank 15% weak', dict(bank_gain=(0.85, 1.0))),
]
def go(a):
    name, kw, exe = a
    with contextlib.redirect_stdout(io.StringIO()):
        r, out = acc2.scenario(exe, PI, name, verbose=False, **kw)
    return name, out
if __name__ == '__main__':
    if len(sys.argv) < 2: sys.exit(__doc__)
    exe = sys.argv[1]
    extra = eval(sys.argv[2]) if len(sys.argv) > 2 else {}
    items = [(n, {**kw, **extra}, exe) for n, kw in CASES]
    with Pool(2) as p:
        for n, o in p.imap(go, items):
            rv = f"rev {o['rev_cm']:5.1f}cm x<={o['rev_x']:.1f}cm hdg<={o['rev_hdg']:.1f}" if 'rev_cm' in o else 'NO REVERSE'
            print(f"{n:28s} {rv:36s} min_gap={o['min_gap']:5.1f} followed_fwd={o['followed_fwd']} end_y={o['end_y']:.0f} lead_end={o['lead_end']:.0f}", flush=True)
