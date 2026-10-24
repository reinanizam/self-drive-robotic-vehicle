"""Lead-car scenario: car drives on tape -> sees lead -> stops -> lead backs toward it -> car backs off -> lead drives away -> car follows."""
import sys, os, math
from dataclasses import replace
from closedloop import *
PITCH = float(os.environ.get('PITCH', '22'))
BASE = dict(a_is_right=False, cam_h=0.10, pitch_deg=PITCH, tape_w=0.047, ir_ahead=0.10, ir_spacing=0.013)
def lead_fn(y0, legs):
    """legs: list of (t_start, speed m/s (+ = away from car, - = toward car), duration)"""
    def f(t):
        y = y0
        for (t0, v, d) in legs:
            if t > t0: y += v * (min(t, t0 + d) - t0)
        return (0.0, y)
    return f
def scenario(fw, pi, label, y0=0.95, legs=((9.0, -0.12, 2.0), (16.0, +0.10, 4.0)), dur=30.0, body=(0.12, 0.09, None), bank_gain=(1.0, 1.0),
             geo_over=None, verbose=True, step_print=0.5, start=(0.0, 0.0, 0.0), track_len=3.5, rev_min_pwm=165, x_sonar_off=0.0, **extra):
    GEO = replace(Geo(), **{**BASE, **(geo_over or {})})
    tr = Track([(0.0, -0.4), (0.0, track_len)], tape_w=GEO.tape_w)
    lead = lead_fn(y0, legs)
    r = run(fw, pi, tr, GEO, 7, duration=dur, record=True, end_tol=None, lead=lead, rev_min_pwm=rev_min_pwm, stop_break=False,
            start=start, bank_gain=bank_gain, lead_body=body, **extra)
    rows = r.log
    if verbose:
        print(f'--- {label}')
        last = -9
        for l in rows:
            if l[0] - last >= step_print:
                last = l[0]
                ly = lead(l[0])[1]
                gap = (ly - l[2]) * 100 - 12  # approx sonar gap
                print(f'  t={l[0]:5.1f} car y={l[2]*100:6.1f} x={l[1]*100:5.1f} hdg={l[3]:5.1f} v={l[4]*100:6.1f} ENA={l[7]:3d} ENB={l[8]:3d} gap~{gap:5.0f}  lead y={ly*100:6.1f} {l[9]}')
    rev = [l for l in rows if l[4] < -0.004]
    out = {}
    if rev:
        out['rev_cm'] = (rev[0][2]-rev[-1][2])*100
        out['rev_x'] = max(abs(l[1]) for l in rev)*100
        out['rev_hdg'] = max(abs(l[3]) for l in rev)
        out['rev_t'] = (rev[0][0], rev[-1][0])
    out['max_cross'] = max(l[6] for l in rows)*100
    out['end_y'] = rows[-1][2]*100
    out['lead_end'] = lead(rows[-1][0])[1]*100
    out['min_gap'] = min([(lead(l[0])[1]-l[2])*100-12 for l in rows if l[0] > 6] or [999.0])
    fwd_after = [l for l in rows if l[0] > 16 and l[4] > 0.004]
    out['followed_fwd'] = bool(fwd_after)
    print(f'  RESULT {label}: ' + ', '.join(f'{k}={v:.1f}' if isinstance(v, float) else f'{k}={v}' for k, v in out.items() if k != 'rev_t') + (f", rev {out['rev_t'][0]:.1f}-{out['rev_t'][1]:.1f}s" if rev else ' NO REVERSE'))
    return r, out
if __name__ == '__main__':
    scenario(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else 'case')
