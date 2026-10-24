"""Closed-loop SOFTWARE-IN-THE-LOOP simulator for the MAVP tape follower.

What is real here: the Arduino .ino (compiled for the host with Arduino stubs and
run as a child process), the Raspberry Pi vision/planner Python (imported
unchanged, fed rendered camera frames) and their serial protocol.

What is INVENTED (a test fixture, NOT measurements of the car): camera height/pitch,
focal length, wheel track, speed-versus-PWM, scrub efficiency, IR bar position
and spacing, latencies, floor brightness. The runner sweeps these over wide
ranges so a pass cannot depend on one made-up geometry. A pass here is a
software result; it is not real-floor validation.
"""
import importlib.util
import math
import subprocess
import sys
import types
from dataclasses import dataclass, field, replace
from pathlib import Path

import cv2
import numpy as np

W, H = 320, 240


@dataclass
class Geo:
    cam_h: float = 0.11          # camera height above floor (m)      INVENTED
    pitch_deg: float = 22.0      # camera pitch below horizontal      INVENTED
    f_px: float = 317.0          # ~53.5 deg HFOV at 320 px           spec-based
    cam_ahead: float = 0.10      # camera ahead of wheel midpoint (m) INVENTED
    ir_ahead: float = 0.04       # IR bar ahead of wheel midpoint (m) INVENTED
    ir_spacing: float = 0.013    # IR channel spacing (m)             INVENTED
    track_w: float = 0.14        # wheel track (m)                    INVENTED
    tape_w: float = 0.019        # tape width (m)                     INVENTED
    vmax: float = 0.38           # m/s at PWM 255 (ON slot)           INVENTED
    eta: float = 0.5             # skid-steer scrub efficiency         INVENTED
    tau: float = 0.06            # wheel speed lag (s)                INVENTED
    cam_latency: float = 0.05    # sensor timestamp -> processing (s) INVENTED
    proc: float = 0.03           # processing time (s)                INVENTED
    a_is_right: bool = True      # PHYSICAL truth: motor bank A drives the RIGHT side
    gyro_sign: float = 1.0       # +1: normal; -1: gyro mounted/wired reversed
    floor: float = 190.0
    tape: float = 40.0
    noise: float = 2.0
    shadow: bool = False


class SimClock:
    t = 0.0


class FakeTime:
    def __init__(self, clock):
        self.clock = clock

    def monotonic(self):
        return self.clock.t

    def sleep(self, _):
        pass


def load_vision(path, clock):
    sys.modules.setdefault('serial', types.SimpleNamespace(SerialException=OSError))
    sys.modules.setdefault('flask', types.SimpleNamespace(
        Flask=lambda _: types.SimpleNamespace(route=lambda *_: lambda f: f), Response=lambda *a, **k: None))
    sys.modules.setdefault('picamera2', types.SimpleNamespace(Picamera2=object))
    spec = importlib.util.spec_from_file_location('vision_under_test_%d' % id(clock), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.time = FakeTime(clock)
    return module


class Track:
    def __init__(self, vertices, tape_w=0.019):
        self.v = np.asarray(vertices, float)
        self.seg_a = self.v[:-1]
        self.seg_b = self.v[1:]
        self.length = float(np.sum(np.linalg.norm(self.seg_b - self.seg_a, axis=1)))

    def dist(self, P):
        best = np.full(len(P), 1e9)
        for a, b in zip(self.seg_a, self.seg_b):
            d = b - a
            t = np.clip(((P - a) @ d) / (d @ d), 0, 1)
            proj = a + t[:, None] * d
            best = np.minimum(best, np.linalg.norm(P - proj, axis=1))
        return best

    def progress(self, p):
        """Arc-length of the closest point (for completion / progress)."""
        best, s_best, acc = 1e9, 0.0, 0.0
        for a, b in zip(self.seg_a, self.seg_b):
            d = b - a
            L = float(np.linalg.norm(d))
            t = float(np.clip(((p - a) @ d) / (d @ d), 0, 1))
            dist = float(np.linalg.norm(p - (a + t * d)))
            if dist < best:
                best, s_best = dist, acc + t * L
            acc += L
        return s_best, best


def arc_track(points_before, radius, turn_deg, points_after, steps=10):
    """Polyline with a rounded corner: straight in, circular arc, straight out."""
    # Starts at points_before[0] heading north (x=0); corner at points_before[1].
    (x0, y0), (x1, y1) = points_before
    pts = [(x0, y0)]
    turn = math.radians(turn_deg)       # +ve = turn LEFT (counter-clockwise), -ve = right
    left = 1 if turn > 0 else -1
    T = radius * math.tan(abs(turn) / 2)
    pts.append((x1, y1 - T))
    cx, cy = x1 + (-left) * 0 - left * radius, y1 - T                 # arc centre (west for left turn)
    cx = x1 - left * radius
    for k in range(1, steps + 1):
        a = abs(turn) * k / steps
        ang0 = 0.0 if left > 0 else math.pi
        x = cx + left * radius * math.cos(a) if left > 0 else cx - radius * math.cos(a)
        y = cy + radius * math.sin(a)
        pts.append((x, y))
    hx, hy = -left * math.sin(abs(turn)), math.cos(abs(turn))
    hx = -math.sin(abs(turn)) * left
    hy = math.cos(abs(turn))
    ex, ey = pts[-1]
    pts.append((ex + hx * points_after, ey + hy * points_after))
    return pts


class Renderer:
    def __init__(self, geo):
        self.geo = geo
        th = math.radians(geo.pitch_deg)
        u = np.arange(W)[None, :] + 0.5
        v = np.arange(H)[:, None] + 0.5
        xc = (u - W / 2) / geo.f_px
        yc = (v - H / 2) / geo.f_px
        den = math.sin(th) + yc * math.cos(th)
        ok = den > 1e-3
        t = np.where(ok, geo.cam_h / np.where(ok, den, 1), 0)
        self.a = (math.cos(th) - yc * math.sin(th)) * t
        self.r = np.broadcast_to(xc, (H, W)) * t
        self.valid = ok & (self.a < 3.0)
        self.rng = np.random.default_rng(1)

    def frame(self, track, x, y, psi, shadow_poly=None, bags=None):
        g = self.geo
        fwd = np.array([math.sin(psi), math.cos(psi)])
        right = np.array([math.cos(psi), -math.sin(psi)])
        P = (np.array([x, y])[None, None, :] + (g.cam_ahead + self.a)[..., None] * fwd +
             self.r[..., None] * right).reshape(-1, 2)
        d = track.dist(P).reshape(H, W)
        edge = np.clip((g.tape_w / 2 - d) / 0.003 + 0.5, 0, 1)
        floor = np.full((H, W), g.floor) - 12.0 * np.clip(self.r, -1, 1)
        img = floor + (g.tape - floor) * edge
        if g.shadow and shadow_poly is not None:
            # soft darker region (a cast shadow), brightness ~ 0.62 of floor
            mask = cv2.fillPoly(np.zeros((H * 1, W), np.uint8), [shadow_poly], 255)
        img = np.where(self.valid, img, 150.0)
        for bag in (bags or []):
            bx, by, bw, bh = bag[:4]
            bcol = bag[4] if len(bag) > 4 else None
            btop = bool(bag[5]) if len(bag) > 5 else False
            # An upright dark object (bag): front face at world y=by, spanning bx+-bw/2, height bh.
            th = math.radians(g.pitch_deg)
            cam = np.array([x, y]) + g.cam_ahead * fwd
            pts = []
            for wx, z in ((bx - bw / 2, 0.0), (bx + bw / 2, 0.0), (bx + bw / 2, bh), (bx - bw / 2, bh)):
                d = np.array([wx, by]) - cam
                a, r = float(d @ fwd), float(d @ right)
                if a <= 0.03:
                    pts = None
                    break
                yd = g.cam_h - z
                yp, dp = yd * math.cos(th) - a * math.sin(th), yd * math.sin(th) + a * math.cos(th)
                if dp <= 0.02:
                    pts = None
                    break
                pts.append((W / 2 + g.f_px * r / dp, H / 2 + g.f_px * yp / dp))
            if pts:
                if btop:
                    # the object's body hides everything behind its near face inside its width: extend the top edge to the picture top
                    pts = [pts[0], pts[1], (pts[1][0], -5.0), (pts[0][0], -5.0)]
                poly = np.array(pts, np.float32)
                layer = np.zeros((H, W), np.uint8)
                cv2.fillPoly(layer, [np.round(poly).astype(np.int32)], 255)
                img = np.where(layer > 0, (g.tape + 8.0) if bcol is None else bcol, img)
        img = img + self.rng.normal(0, g.noise, img.shape)
        img = cv2.GaussianBlur(img.astype(np.float32), (3, 3), 0)
        img = np.clip(img, 0, 255).astype(np.uint8)
        return np.dstack([img, img, img])


class Firmware:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  text=True, bufsize=1)

    def _read(self):
        out, tele = None, []
        while True:
            line = self.p.stdout.readline()
            if not line:
                raise RuntimeError('firmware harness died')
            line = line.rstrip('\n')
            if line == 'E':
                return out, tele
            if line.startswith('O '):
                out = tuple(int(v) for v in line.split()[1:])
            elif line.startswith('T '):
                tele.append(line[2:])

    def init(self, raw, mask):
        self.p.stdin.write(f'I {raw} {mask}\n'); self.p.stdin.flush()
        return self._read()[1]

    def step(self, raw, mask, echo_us, dt_us, guide=None):
        self.p.stdin.write(f'S {raw} {mask} {echo_us} {dt_us}' + (f' {guide}' if guide else '') + '\n')
        self.p.stdin.flush()
        return self._read()

    def close(self):
        try:
            self.p.stdin.close(); self.p.wait(timeout=2)
        except Exception:
            self.p.kill()


@dataclass
class Result:
    completed: bool = False
    stopped: bool = False
    reason: str = ''
    t_end: float = 0.0
    max_cross: float = 0.0           # max pivot distance from the tape centreline (m)
    max_cross_after_start: float = 0.0
    progress: float = 0.0
    length: float = 0.0
    min_speed_mean: float = 0.0
    mean_speed: float = 0.0
    yaw_travel: float = 0.0
    fault: bool = False
    log: list = field(default_factory=list)
    trace: list = field(default_factory=list)


EMPTY_TRACK = None

def run(exe, vision_path, track, geo, protocol, duration=20.0, start=(0.0, 0.0, 0.0),
        seed=0, ir_dropout=None, record=False, ir_blank=False, end_tol=0.18, bags=None, walls=None, ir_extra=None, blind_turn=False, ir_periodic=None, lead=None, rev_min_pwm=None, stop_break=True, bank_gain=(1.0, 1.0), lead_body=None, cam_blackout=None):
    global EMPTY_TRACK
    if EMPTY_TRACK is None: EMPTY_TRACK = Track([(50.0, 50.0), (50.0, 51.0)], tape_w=track.tape_w if hasattr(track, 'tape_w') else 0.019)
    clock = SimClock(); clock.t = 0.0
    vision = load_vision(vision_path, clock)
    rend = Renderer(geo)
    rend.rng = np.random.default_rng(seed)
    steer = vision.PredictiveSteeringController()
    telem = vision.ArduinoSensorTelemetry()
    fw = Firmware(exe)
    x, y, psi = start
    vl = vr = 0.0
    ENA = ENB = 0
    adir = 0
    bias = 393
    tele = fw.init(bias, 0)
    res = Result(length=track.length)
    dt_us = 5000
    dt = dt_us / 1e6
    next_frame = 0.0
    pending = []                      # (deliver_time, guide)
    last_src = ''
    zero_since = None
    moved = 0.0
    last_left = last_right = 0
    steps = int(duration / dt)
    rate_dps = 0.0
    speed_acc = []
    for step in range(steps):
        clock.t = step * dt
        t = clock.t
        # --- camera frame -> Pi program --------------------------------------------------------
        if t >= next_frame:
            next_frame = t + 1 / 24
            pivoting = False
            if blind_turn:
                sn = telem.snapshot()
                pivoting = bool(sn.get('fresh') and sn['values'].get('PIVOT') == 1)
            bl = list(bags or [])
            if lead_body is not None and lead is not None:
                lp0 = lead(t)
                if lp0 is not None:
                    bl.append((lp0[0], lp0[1], lead_body[0], lead_body[1], lead_body[2], True))
            blk = cam_blackout is not None and cam_blackout[0] <= t < cam_blackout[1]
            img = rend.frame(EMPTY_TRACK if (pivoting or blk) else track, x, y, psi, bags=bl)
            steer.frame_age = geo.cam_latency
            steer.frame_interval = 1 / 24
            steer.frame_received_at = t
            steer.set_sensor_feedback(telem.snapshot())
            command, _ = vision.process_frame(img, steer)
            steer.frame_age += geo.proc
            steer.update_pace()
            guide = vision.drive_command(command, steer, protocol)
            pending.append((t + geo.proc + 0.01, guide))
            if record:
                res.trace.append((round(t, 3), 'PI', guide, round(steer.arrival.phase and 0 or 0)))
        # --- deliver the newest due guide line -------------------------------------------------
        guide = None
        due = [p for p in pending if p[0] <= t]
        if due:
            guide = due[-1][1]
            pending = [p for p in pending if p[0] > t]
        # --- sensors ---------------------------------------------------------------------------
        fwd = np.array([math.sin(psi), math.cos(psi)])
        right = np.array([math.cos(psi), -math.sin(psi)])
        mask = 0
        if not ir_blank:
            for i in range(5):
                p = np.array([x, y]) + geo.ir_ahead * fwd + (i - 2) * geo.ir_spacing * right
                if track.dist(p[None, :])[0] < geo.tape_w / 2 + 0.002:
                    mask |= 1 << i
            if ir_dropout and ir_dropout[0] <= t < ir_dropout[1]:
                mask = 0
            if ir_periodic and ((t - ir_periodic[2]) % ir_periodic[0]) < ir_periodic[1]:
                mask = 0
        if ir_extra is not None:
            mask |= ir_extra(x + geo.ir_ahead * fwd[0], y + geo.ir_ahead * fwd[1], psi)
        raw = int(round(bias - geo.gyro_sign * rate_dps * 65.5))
        echo = 0
        if walls:
            sp = np.array([x, y]) + (geo.ir_ahead + 0.02) * fwd
            best = 9.0
            for (x1, y1, x2, y2) in walls:
                a = np.array([x1, y1]); b = np.array([x2, y2]); e = b - a
                den = fwd[0] * e[1] - fwd[1] * e[0]
                if abs(den) < 1e-9: continue
                d_ = a - sp
                tt = (d_[0] * e[1] - d_[1] * e[0]) / den
                uu = (d_[0] * fwd[1] - d_[1] * fwd[0]) / den
                if tt > 0 and 0 <= uu <= 1: best = min(best, tt)
            if best < 3.5: echo = int(best * 100 * 58)
        if lead is not None:
            lp = lead(t)
            if lp is not None:
                sp2 = np.array([x, y]) + (geo.ir_ahead + 0.02) * fwd
                dv = np.array(lp) - sp2
                al = float(dv @ fwd); lat = float(dv @ right)
                if 0.02 < al < 3.5 and abs(lat) < 0.10:
                    e2 = int(al * 100 * 58)
                    echo = e2 if echo == 0 else min(echo, e2)
        out, tl = fw.step(raw, mask, echo, dt_us, guide)
        ENA, ENB, adir = out[:3]
        # Bank directions from the IN pins (forward = IN1/IN3 LOW, IN2/IN4 HIGH).
        if len(out) >= 7:
            dA = 1 if (out[3] == 0 and out[4] == 1) else -1 if (out[3] == 1 and out[4] == 0) else 0
            dB = 1 if (out[5] == 0 and out[6] == 1) else -1 if (out[5] == 1 and out[6] == 0) else 0
        else:
            dA = dB = (1 if adir >= 0 else -1)
        for line in tl:
            if line.startswith('SENS,'):
                telem.feed((line + '\n').encode('ascii'))
                last_src = line.rsplit(',', 1)[-1]
                if record:
                    res.trace.append((round(t, 3), 'FW', line, 0))
                parts = line.split(',')
                f = dict(zip(parts[2::2], parts[3::2]))
                if f.get('ERR', '0') not in ('0',):
                    res.fault = True
            elif record and line.startswith('DBG'):
                res.trace.append((round(t, 3), 'DBG', line, 0))
            elif record and line.startswith('ALERT'):          # only the alert_check build prints these
                res.trace.append((round(t, 3), 'AL', line, 0))
        # --- vehicle dynamics ------------------------------------------------------------------
        right_pwm, left_pwm = (ENA, ENB) if geo.a_is_right else (ENB, ENA)
        d_right, d_left = (dA, dB) if geo.a_is_right else (dB, dA)
        thr_l = 210 if (rev_min_pwm is None or d_left >= 0) else rev_min_pwm
        thr_r = 210 if (rev_min_pwm is None or d_right >= 0) else rev_min_pwm
        tl_ = d_left * (left_pwm / 255.0 * geo.vmax if left_pwm >= thr_l else 0.0)
        tr_ = d_right * (right_pwm / 255.0 * geo.vmax if right_pwm >= thr_r else 0.0)
        k = dt / (geo.tau + dt)
        tl_ *= bank_gain[0]; tr_ *= bank_gain[1]
        vl += k * (tl_ - vl)
        vr += k * (tr_ - vr)
        v = 0.5 * (vl + vr)
        w = (vl - vr) / geo.track_w * geo.eta          # clockwise (nose right) positive, rad/s
        psi += w * dt
        x += v * math.sin(psi) * dt
        y += v * math.cos(psi) * dt
        rate_dps = math.degrees(w)
        moved += abs(v) * dt
        speed_acc.append(v)
        s, d = track.progress(np.array([x, y]))
        res.max_cross = max(res.max_cross, d)
        res.progress = max(res.progress, s)
        if record and step % 4 == 0:
            res.log.append((t, x, y, math.degrees(psi), v, rate_dps, d, ENA, ENB, last_src, mask, out[7] if len(out) > 7 else 0, out[8] if len(out) > 8 else 0))
        if end_tol is not None and (s >= track.length - 0.05 or np.linalg.norm(np.array([x, y]) - track.v[-1]) < end_tol):
            res.completed = True
            break
        if stop_break and ENA == 0 and ENB == 0 and abs(v) < 0.01 and moved > 0.10:
            zero_since = t if zero_since is None else zero_since
            if t - zero_since > 1.0:
                res.stopped = True
                res.reason = last_src
                break
        else:
            zero_since = None
        if d > 0.45:
            res.reason = 'LEFT_TRACK'
            break
    res.t_end = clock.t
    res.mean_speed = float(np.mean(speed_acc)) if speed_acc else 0.0
    fw.close()
    return res
