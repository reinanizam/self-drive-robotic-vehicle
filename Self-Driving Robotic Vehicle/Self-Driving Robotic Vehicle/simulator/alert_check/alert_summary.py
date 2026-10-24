"""What the buzzer, the LED and the screen do in a back-off scenario (software only).
Build (run in this folder; the stubs here count tone() calls and log the LCD text):
    g++ -std=c++17 -O1 -I. -Istubs -DFW_PATH='"../../firmware/experimental/mavp_uno_v76e/mavp_uno_v76e.ino"' sim_fw.cpp -o sim_alerts
Run (about 4 minutes):   python3 alert_summary.py ./sim_alerts S1
Expected for V7.6e: BRAKING screen with ONE beep, then REVERSING screen with a beep every 700 ms (200 ms on),
then BRAKING (one beep) whenever it stops again.
"""
import sys, io, os, contextlib
sys.path.insert(0, '..')          # acc2.py and closedloop.py live in the simulator folder
import acc2
fw, label = sys.argv[1], sys.argv[2]
kw = eval(sys.argv[3]) if len(sys.argv) > 3 else {}
with contextlib.redirect_stdout(io.StringIO()):
    r, out = acc2.scenario(fw, os.environ.get('PI', '../../raspberry_pi/mavp_pi.py'), label, verbose=False, **kw)
al = [x[2].split(',') for x in r.trace if x[1] == 'AL']
print('rev', round(out['rev_cm'], 1), 'cm  hdg<=', round(out['rev_hdg'], 1), ' followed_fwd', out['followed_fwd'])
prevscr = None; prevled = 0; onsets = []; tones_at = {}
for p in al:
    ms, tones, led = int(p[1]), int(p[2]), int(p[3]); txt = ','.join(p[4:]); top = txt.split('],[')[0][1:].strip()
    scr = top if top.startswith(('!!', '<<')) else 'normal'
    if scr != prevscr:
        print(f'  {ms/1000:7.2f}s screen -> {scr!r:22} tones so far {tones}')
        prevscr = scr
    if led == 1 and prevled == 0: onsets.append(ms)
    prevled = led
gaps = [b - a for a, b in zip(onsets, onsets[1:])]
print('  beep onsets:', len(onsets), ' first:', [round(o/1000, 2) for o in onsets[:8]], ' gaps ms (first 12):', gaps[:12])
