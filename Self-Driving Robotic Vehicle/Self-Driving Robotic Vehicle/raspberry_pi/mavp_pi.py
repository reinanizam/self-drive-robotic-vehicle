"""MAVP Stage 8 FUSION V7.6c live vision-to-steering program.

V7.6c (2026-10-23): same decisions as V7.6b, about 3x faster per picture (80 -> 27 ms measured): the per-row median
calls in trace_ground_strip no longer go through numpy. The dashboard shows the Pi version, tape width and pace. New option --cruise N (default 70).

V7.6 (2026-10-22): the tape filters know the real tape is 4.7 cm wide (they were written for ~1.9 cm and cut the
real tape down to a thin edge strip). Pi only; same serial protocol; run it with the V7.5 Uno sketch.

V7.5 (2026-10-22): stricter tape filter (a pale dark patch such as marble is not tape),
overlay drawn only on a real tape lock, new live dashboard with /status.json. Same serial
protocol as V7.3; run it with the V7.5 Uno sketch (banner ends V75).

V7.1 (2026-10-21): a dark upright object at the end of the strip (a bag) is no longer read
as a corner. Same serial protocol and same Arduino firmware as V7.

Raise all four wheels before running this program. Open the printed address in
a laptop browser to see the camera, OpenCV mask and command. Tape loss, program
shutdown and Arduino serial timeout all stop the motors. Stable real camera
frames can authorize drive; the optional line array corrects current centre. A short,
time/angle-bounded one-source recovery can bridge a dropout after acquisition.
A recently observed, arriving bend can bridge a short loss of both optical
sources, subject to gyro/time/obstacle limits. This is not metric odometry.
"""

import argparse
import glob
import csv
import io
import json
import threading
import time
import zipfile
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import serial
from flask import Flask, Response
from picamera2 import Picamera2


BAUD_RATE = 115200

# Picamera2 "RGB888" is stored B,G,R in the numpy array (same as OpenCV). The
# earlier program treated it as R,G,B, so the stream and the grey conversion had
# red and blue swapped. Everything downstream of capture now receives genuine
# R,G,B. If the colour test (--colour-test) shows red as blue, run with --swap-rb.
CAMERA_ARRAY_IS_BGR = True

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_RATE = 24

# The OV5647 can give black tape a red colour cast.  Detect it by brightness,
# not hue or saturation.  The threshold follows the current floor brightness
# but is clamped so normal shadows are less likely to become tape.
FLOOR_DARKNESS_GAP = 40
MIN_BLACK_THRESHOLD = 35
# Glossy black tape can appear medium gray under a ceiling-light reflection.
# The previous ceiling of 80 rejected that reflected section completely.
MAX_BLACK_THRESHOLD = 145
MIN_TAPE_AREA = 100
MIN_TAPE_HEIGHT = 12
# Reject genuinely broad dark regions by their average thickness.  Do not use
# the contour's bounding-box width: a valid diagonal or curved tape segment
# can span most of the image horizontally while remaining physically narrow.
MAX_TAPE_AVERAGE_WIDTH_FRACTION = 0.55
# The track's tape is 4.7 cm wide (measured on the track). The size limits below were first written for ~1.9 cm tape
# and treated anything much wider as 'a bag': they cut the real tape down to a thin strip along one edge. Every
# width limit is scaled by TAPE_WIDTH_CM / TAPE_WIDTH_TUNED_CM. Other tape: python3 ... --tape-cm 1.9
TAPE_WIDTH_CM = 4.7
TAPE_WIDTH_TUNED_CM = 1.9
SIDE_EDGE_MARGIN = 2
GLARE_CLOSE_KERNEL_WIDTH = 7
GLARE_CLOSE_KERNEL_HEIGHT = 15
# Use more of the road ahead so curves are detected before the car reaches
# them.
ROI_TOP_FRACTION = 0.35
ERROR_DEADBAND = 4
STEERING_GAIN = 0.95
# Confirmed convention: image right is positive and image left is negative.
# Motor-bank reversal is handled in the Arduino, never by reversing the
# image coordinates (which would also corrupt camera/IR agreement).
STEERING_DIRECTION = 1

# Predictive steering samples the detected tape at several distances.  The
# far sample prepares a future turn; a separate near carrot corrects the
# current approach. The Arduino decides entry using local IR when available.
CENTERLINE_MIN_ROWS = 12
# Far geometry should warn the controller that a bend is coming, but it must
# not make the car turn too early.  Authority shifts progressively from the
# current/near tape to the upcoming/far tape as the bend moves toward the car.
BEND_URGENCY_START = 0.28
BEND_URGENCY_FULL = 0.72
NEAR_WEIGHT_FAR_BEND = 0.52
NEAR_WEIGHT_CLOSE_BEND = 0.30
MID_POSITION_WEIGHT = 0.30
FAR_WEIGHT_FAR_BEND = 0.18
FAR_WEIGHT_CLOSE_BEND = 0.40
HEADING_WEIGHT_FAR_BEND = 0.18
HEADING_WEIGHT_CLOSE_BEND = 0.38
CURVATURE_WEIGHT_FAR_BEND = 0.08
CURVATURE_WEIGHT_CLOSE_BEND = 0.16

# A short rolling history removes one-frame jumps. Response is adaptive:
# precise and calm around centre, faster for a normal bend, and nearly
# immediate for a sharp bend.
STEERING_HISTORY_LENGTH = 3
MEDIUM_TURN_ERROR = 22
SHARP_TURN_ERROR = 55
GENTLE_STEERING_SMOOTHING = 0.35
MEDIUM_STEERING_SMOOTHING = 0.55
SHARP_STEERING_SMOOTHING = 0.72
GENTLE_MAX_CHANGE_PER_FRAME = 10
MEDIUM_MAX_CHANGE_PER_FRAME = 22
SHARP_MAX_CHANGE_PER_FRAME = 36
# Live camera geometry remains the main input.  A smaller derivative-based
# prediction compensates processing/motor delay, and short turn commitment
# prevents the command collapsing when a corner moves underneath the camera.
LIVE_GEOMETRY_WEIGHT = 0.90
TEMPORAL_PREDICTION_WEIGHT = 0.10
MAX_PREDICTION_CORRECTION = 14
TURN_COMMIT_ENTRY_ERROR = 38
TURN_COMMIT_MINIMUM_PROXIMITY = 0.48
TURN_COMMIT_MINIMUM_FRACTION = 0.50
TURN_COMMIT_SECONDS = 0.35
# Briefly bridge an imperfect frame at explicitly low camera confidence.
# Current local IR can override that prediction; sustained loss stops guidance.
TRACK_LOSS_GRACE_SECONDS = 0.15

COMMAND_PERIOD_SECONDS = 0.025
PRINT_PERIOD_SECONDS = 0.50
JPEG_QUALITY = 70
WEB_PORT = 5001
MAX_FRAME_AGE_SECONDS = 0.25
# Full-power ON slots retain starting torque; OFF slots lower average travel.
# These are duty limits, NOT measured speed percentages or distances.
CRUISE_PACE = 70
APPROACH_PACE = 55
TURN_PACE = 45


ACTIVE_SOURCES = {'BOTH', 'CENTER', 'REC_CAM', 'REC_IR', 'REJOIN', 'TURN_MEM', 'CAM_TRACK', 'TURN'}

app = Flask(__name__)
controller = None


def camera_array_to_rgb(frame):
    """Return genuine R,G,B channel order from a Picamera2 'RGB888' array."""
    return np.ascontiguousarray(frame[..., ::-1]) if CAMERA_ARRAY_IS_BGR else frame


# The corner is this fraction of the way down the picture: it is about to vanish into the
# camera's blind zone. The Uno then dead-reckons the last distance to the wheels.
BLIND_ROW_FRACTION = 0.90
# Once a corner has been announced (phase 2) it is remembered this long even if the picture
# near the bottom flips direction or resets the planner (it does, a few cm before the corner).
CORNER_LATCH_S = 1.5
# V7.4 tape filter. Tape is far darker than the floor around it: in the recorded runs every
# IR-confirmed strip had (floor - tape) / floor >= 0.51. Marble veins, furniture edges and
# cabinet shadows that passed the old test were 0.12-0.27. Between the two there is a gap;
# below WEAK the quality is cut, below REJECT it falls under the steering threshold.
CONTRAST_RATIO_REJECT = 0.30
CONTRAST_RATIO_WEAK = 0.42
# Colour is NOT used: the OV5647 can tint black tape maroon, which looks like brown furniture.
BLACK_GATES = True      # python3 stage8_pi_V7_5.py --loose-tape turns the V7.4 tape gates off (demo-day fallback)

class ImagePathMemory:
    """Short-lived observed image path transported by checked optical flow.

    This is not metric odometry or a permanent map. A remembered point expires
    after 0.65 s; it can extend a visible path only if overlapping live points
    agree. Prediction is never promoted to fresh camera confidence.
    """

    def __init__(self):
        self.path = np.empty((0, 3), dtype=np.float64)
        self.previous_gray = None
        self.previous_points = None
        self.previous_at = None
        self.previous_yaw = None
        self.motion_valid = False
        self.flow_y_rate = 0.0
        self.used = False

    def advance(self, gray, yaw=None, now=None):
        now = time.monotonic() if now is None else now
        self.motion_valid = False
        if self.path.size:
            self.path = self.path[now - self.path[:, 2] <= 0.65]
        if (self.previous_gray is not None and self.previous_gray.shape == gray.shape and
                self.previous_points is not None and len(self.previous_points) >= 6 and
                self.previous_at is not None and 0.005 <= now - self.previous_at <= 0.20):
            try:
                next_points, ok, _ = cv2.calcOpticalFlowPyrLK(
                    self.previous_gray, gray, self.previous_points, None,
                    winSize=(15, 15), maxLevel=2)
                if next_points is not None:
                    returned, back_ok, _ = cv2.calcOpticalFlowPyrLK(
                        gray, self.previous_gray, next_points, None,
                        winSize=(15, 15), maxLevel=2)
                    if returned is not None:
                        good = (ok.ravel() == 1) & (back_ok.ravel() == 1)
                        good &= np.linalg.norm(returned[:, 0] - self.previous_points[:, 0], axis=1) < 1.5
                        old = self.previous_points[good, 0]
                        new = next_points[good, 0]
                        if len(old) >= 6 and np.std(old[:, 0]) >= 10 and np.std(old[:, 1]) >= 10:
                            matrix, inliers = cv2.estimateAffinePartial2D(
                                old, new, method=cv2.RANSAC, ransacReprojThreshold=2.5,
                                maxIters=100, confidence=0.99)
                            if matrix is not None and inliers is not None:
                                scale = float(np.linalg.norm(matrix[0, :2]))
                                angle = abs(float(np.degrees(np.arctan2(matrix[1, 0], matrix[0, 0]))))
                                shift = np.median(new - old, axis=0)
                                yaw_consistent = not (yaw is not None and self.previous_yaw is not None and
                                    abs(yaw - self.previous_yaw) > 3 and np.linalg.norm(shift) < 1)
                                self.motion_valid = bool(np.mean(inliers) >= 0.65 and 0.85 <= scale <= 1.18 and
                                                         angle < 12 and np.linalg.norm(shift) < 35 and yaw_consistent)
                                if self.motion_valid:
                                    if self.path.size:
                                        self.path[:, :2] = cv2.transform(
                                            self.path[:, :2].astype(np.float32)[None], matrix)[0]
                                    rate = float(shift[1]) / (now - self.previous_at)
                                    self.flow_y_rate += 0.3 * (float(np.clip(rate, -120, 400)) - self.flow_y_rate)
            except cv2.error:
                self.motion_valid = False
        if not self.motion_valid:
            self.flow_y_rate *= 0.6
        self.previous_gray = gray.copy()
        self.previous_points = cv2.goodFeaturesToTrack(gray, maxCorners=60, qualityLevel=0.01, minDistance=8)
        self.previous_at, self.previous_yaw = now, yaw

    def observe(self, live_points, now=None):
        now = time.monotonic() if now is None else now
        live = np.asarray(live_points, dtype=np.float32)
        self.used = False
        extra = np.empty((0, 3))
        if self.motion_valid and len(self.path) >= 6:
            old = self.path[np.argsort(self.path[:, 1])]
            overlap = live[(live[:, 1] >= old[0, 1]) & (live[:, 1] <= old[-1, 1])]
            if len(overlap) >= 6:
                difference = abs(np.interp(overlap[:, 1], old[:, 1], old[:, 0]) - overlap[:, 0])
                if float(np.median(difference)) <= 18:
                    extra = old[(old[:, 1] < live[:, 1].min() - 3) & (now - old[:, 2] <= 0.65)]
        self.used = len(extra) >= 3
        stamped = np.column_stack((live, np.full(len(live), now)))
        self.path = np.vstack((extra, stamped))
        self.path = self.path[np.argsort(self.path[:, 1])]
        return self.path[:, :2].astype(np.float32)


class FrameTiming:
    """Validate the timestamp belonging to the exact image, not capture call time."""

    def __init__(self):
        self.previous_stamp = None
        self.interval = 1.0 / FRAME_RATE
        self.age = 0.0
        self.valid = False

    def accept(self, metadata, boot_ns=None):
        if boot_ns is None:
            boot_ns = (time.clock_gettime_ns(time.CLOCK_BOOTTIME)
                       if hasattr(time, 'CLOCK_BOOTTIME') else time.monotonic_ns())
        stamp = metadata.get('SensorTimestamp')
        self.valid = False
        if not isinstance(stamp, (int, np.integer)) or stamp <= 0:
            return False
        self.age = (boot_ns - stamp) / 1e9
        if not 0 <= self.age <= MAX_FRAME_AGE_SECONDS:
            return False  # A corrupt future stamp must not poison later frames.
        if self.previous_stamp is not None:
            if stamp <= self.previous_stamp:
                return False
            self.interval = (stamp - self.previous_stamp) / 1e9
        self.previous_stamp = stamp
        self.valid = True
        return self.valid


class BendArrivalPlanner:
    """Track the approach of a seen bend; never label memory as a live detection.

    ETA is time to an image-space entry band, NOT time to an axle or cm to tape.
    Repeated bend motion supplies the estimate. A gyro rejects timing learned
    from a large change of camera heading. Only a fresh observed entry can
    authorize the Arduino's short turn continuation.
    """

    def __init__(self):
        self.height = None
        self.reset()

    def reset(self):
        self.sign = 0
        self.row = None
        self.seen_at = None
        self.yaw = None
        self.row_rate = 0.0
        self.motion_samples = 0
        self.confirmations = 0
        self.eta = None
        self.phase = 'STRAIGHT'
        self.preview = 0.0
        self.entry_yaw = None
        self.entry_at = None
        self.fresh_entry = False
        self.straight_frames = 0
        self.blind_now = False

    def observe(self, bend_row, height, preview, quality, yaw=None, moving=False,
                frame_age=0.0, interval=1 / FRAME_RATE, full_straight=False, now=None):
        now = time.monotonic() if now is None else now
        self.height = height
        self.fresh_entry = False
        self.blind_now = False
        if self.seen_at is not None and now - self.seen_at > 0.8:
            self.reset()
        sign = int(np.sign(preview))
        observed = bend_row is not None and quality >= 70 and abs(preview) >= 22
        if not observed:
            self.eta = None
            if (self.row is not None and self.seen_at is not None and now - self.seen_at <= 0.8 and
                    self.row >= height * BLIND_ROW_FRACTION * 0.9):
                self.blind_now = True  # the corner dropped into the camera's blind zone
            self.straight_frames = self.straight_frames + 1 if full_straight and quality >= 70 else 0
            if self.straight_frames >= 3:
                self.reset()  # Sustained real exit beats a remembered corner.
            elif self.entry_at is not None:
                progress = ((yaw - self.entry_yaw) * self.sign
                            if yaw is not None and self.entry_yaw is not None else None)
                if (progress is None or progress < -5 or progress >= 30 or
                        now - self.seen_at > 0.8):
                    self.reset()
                else:
                    self.phase = 'TURN MEMORY'
                    return self.sign * max(20.0, abs(self.preview) - progress * 1.4), 0.85
            return preview, 0.0

        self.straight_frames = 0
        if self.sign and sign != self.sign:
            self.reset()  # Never carry the previous direction into the next bend.
        dt = now - self.seen_at if self.seen_at is not None else 0.0
        if self.row is not None and 0.02 <= dt <= 0.25 and moving:
            dy = bend_row - self.row
            if abs(dy) > max(30.0, self.row_rate * dt * 2 + 8):
                self.confirmations = 0  # Reconfirm an unassociated geometry jump.
            heading_stable = yaw is not None and self.yaw is not None and abs(yaw - self.yaw) < 3
            rate = dy / dt
            if heading_stable and 5 <= rate <= 500:
                self.row_rate = rate if self.motion_samples == 0 else 0.6 * self.row_rate + 0.4 * rate
                self.motion_samples += 1
            else:
                self.row_rate *= 0.6
                self.motion_samples = 0
        else:
            self.row_rate = 0.0
            self.motion_samples = 0
        self.confirmations = self.confirmations + 1 if dt and dt <= 0.25 else 1
        if bend_row >= height * BLIND_ROW_FRACTION and self.confirmations >= 2:
            self.blind_now = True
        self.sign, self.row, self.seen_at, self.yaw = sign, float(bend_row), now, yaw
        self.preview = preview
        entry_row = height * 0.78
        self.eta = (max(0.0, (entry_row - bend_row) / self.row_rate)
                    if self.motion_samples >= 2 and self.row_rate >= 8 else None)
        # Delay is measured per frame; the remaining 180 ms is a conservative
        # actuator allowance pending a real drive recording, not learned speed.
        lead = float(np.clip(frame_age + interval + 0.18, 0.18, 0.45))
        arriving = bend_row >= entry_row or (self.eta is not None and self.eta <= lead)
        self.phase = 'TURN' if arriving and self.confirmations >= 3 else 'APPROACH'
        if self.phase == 'TURN' and moving:
            if self.entry_at is None:
                self.entry_at, self.entry_yaw = now, yaw
            self.fresh_entry = True
        predicted_row = bend_row + (self.row_rate * lead if self.motion_samples >= 2 else 0)
        urgency = float(np.clip(predicted_row / max(height - 1, 1), 0, 1))
        if self.phase == 'TURN':
            # A far preview must not collapse to zero as its visible extent
            # shortens at entry. IR retains lateral-centering authority.
            preview = sign * max(abs(preview), 45.0)
        return preview, urgency


class PredictiveSteeringController:
    """Turn raw path measurements into a stable, responsive command."""

    def __init__(self):
        self.history = deque(maxlen=STEERING_HISTORY_LENGTH)
        self.filtered_error = 0.0
        self.last_valid_at = None
        self.previous_live_error = None
        self.committed_turn_sign = 0
        self.committed_turn_strength = 0.0
        self.committed_turn_until = 0.0
        self.camera_quality = 0
        self.turn_proximity = 0.0
        self.previous_near_x = None
        self.floor_level = None
        self.last_observation_at = None
        self.straight_observations = 0
        self.near_error = 0.0
        self.near_control = 0.0
        self.preview_error = 0.0
        self.lock_message = "TUNED SIMPLE DETECTOR"
        self.path_memory = ImagePathMemory()
        self.feedback_yaw = None
        self.memory_used = False
        self.arrival = BendArrivalPlanner()
        self.feedback_moving = False
        self.frame_age = 0.0
        self.frame_interval = 1 / FRAME_RATE
        self.frame_received_at = None
        self.pace = CRUISE_PACE
        self.ir_now = None
        self.ir_quality = 0
        self.reject_why = ''        # V7.4: why the last candidate was refused (PALE)
        self.turn_deg = 0.0
        self.corner_latch = None    # [sign, turn_deg, armed_at, reached_bottom]
        self.feedback_pivot = False
        self.last_cue = {}
        self.width_pending = None   # (width px, agreeing frames, last time) while a new tape width is being confirmed
        self.tape_width = None      # learned normal width of the tape (px); used only as a ratio
        self.tape_width_at = None

    def set_sensor_feedback(self, status):
        fresh = status.get('fresh') and (status.get('age') is None or status['age'] <= 0.35)
        self.feedback_yaw = status['values'].get('YAW') if fresh else None
        values = status.get('values', {})
        self.ir_now = values.get('IR') if fresh and values.get('VALID') == 1 else None
        self.feedback_pivot = bool(fresh and values.get('PIVOT') == 1)
        self.ir_quality = values.get('Q', 0) if self.ir_now is not None else 0
        # Active drive state, NOT measured wheel speed. A telemetry snapshot can
        # land on an intentional OFF slot. Actual bend motion is still required
        # for ETA; the Arduino separately forbids recovery from a stopped state.
        self.feedback_moving = bool(fresh and values.get('ARM') == 1 and values.get('IMU') == 1 and
                                    values.get('SRC') in ACTIVE_SOURCES)

    def update_pace(self):
        self.pace = (TURN_PACE if self.arrival.phase in {'TURN', 'TURN MEMORY'} else
                     APPROACH_PACE if self.arrival.phase == 'APPROACH' or self.turn_proximity > 0.4 else
                     CRUISE_PACE)
        if self.frame_interval > 0.08 or self.frame_age > 0.10:
            self.pace = min(self.pace, 50)
        if self.frame_interval > 0.12 or self.frame_age > 0.18:
            self.pace = min(self.pace, 35)

    def update(self, raw_error, turn_proximity=0.0):
        now = time.monotonic()
        elapsed = 1.0 / FRAME_RATE if self.last_valid_at is None else now - self.last_valid_at
        self.last_valid_at = now

        live_error = float(raw_error)

        if self.previous_live_error is None:
            prediction_correction = 0.0
        else:
            prediction_correction = float(np.clip(
                live_error - self.previous_live_error,
                -MAX_PREDICTION_CORRECTION,
                MAX_PREDICTION_CORRECTION,
            ))

        self.previous_live_error = live_error
        predicted_error = live_error + prediction_correction
        adaptive_error = (
            LIVE_GEOMETRY_WEIGHT * live_error
            + TEMPORAL_PREDICTION_WEIGHT * predicted_error
        )

        # As the detected bend moves down toward the car, increase authority
        # slightly.  This is image-space progress, not an invented physical
        # distance measurement.
        proximity_gain = 1.0 + 0.22 * float(np.clip(
            turn_proximity,
            0.0,
            1.0,
        ))
        adaptive_error *= proximity_gain

        adaptive_sign = int(np.sign(adaptive_error))

        if abs(live_error) <= 8 and turn_proximity < 0.25:
            self.straight_observations += 1
        else:
            self.straight_observations = 0
        if self.straight_observations >= 2:
            self.committed_turn_sign = 0
            self.committed_turn_strength = 0.0
            self.committed_turn_until = 0.0

        if (
            abs(adaptive_error) >= TURN_COMMIT_ENTRY_ERROR
            and turn_proximity >= TURN_COMMIT_MINIMUM_PROXIMITY
        ):
            if (
                self.committed_turn_sign != 0
                and adaptive_sign != self.committed_turn_sign
            ):
                self.committed_turn_strength = 0.0

            self.committed_turn_sign = adaptive_sign
            self.committed_turn_strength = max(
                self.committed_turn_strength * 0.90,
                abs(adaptive_error),
            )
            self.committed_turn_until = now + TURN_COMMIT_SECONDS

        elif (
            self.committed_turn_sign != 0
            and now <= self.committed_turn_until
            and (
                adaptive_sign == 0
                or adaptive_sign == self.committed_turn_sign
            )
        ):
            minimum_committed_error = (
                self.committed_turn_strength
                * TURN_COMMIT_MINIMUM_FRACTION
            )
            adaptive_error = (
                self.committed_turn_sign
                * max(abs(adaptive_error), minimum_committed_error)
            )

        elif now > self.committed_turn_until:
            self.committed_turn_sign = 0
            self.committed_turn_strength = 0.0

        adaptive_error = float(np.clip(adaptive_error, -100, 100))
        self.history.append(adaptive_error)
        robust_error = float(np.median(self.history))

        # A newly appearing hard turn must not wait for the three-frame
        # median.  Smaller changes still use it to reject camera noise.
        if abs(adaptive_error - self.filtered_error) >= SHARP_TURN_ERROR:
            robust_error = adaptive_error

        magnitude = abs(robust_error)

        if magnitude >= SHARP_TURN_ERROR:
            smoothing = SHARP_STEERING_SMOOTHING
            max_change = SHARP_MAX_CHANGE_PER_FRAME
        elif magnitude >= MEDIUM_TURN_ERROR:
            smoothing = MEDIUM_STEERING_SMOOTHING
            max_change = MEDIUM_MAX_CHANGE_PER_FRAME
        else:
            smoothing = GENTLE_STEERING_SMOOTHING
            max_change = GENTLE_MAX_CHANGE_PER_FRAME

        # Keep the response consistent when actual frame rate differs from 24.
        smoothing = 1.0 - (1.0 - smoothing) ** float(np.clip(elapsed * FRAME_RATE, 0.25, 3.0))
        max_change *= float(np.clip(elapsed * FRAME_RATE, 0.25, 3.0))
        target = (
            smoothing * robust_error
            + (1.0 - smoothing) * self.filtered_error
        )

        change = np.clip(
            target - self.filtered_error,
            -max_change,
            max_change,
        )
        self.filtered_error += float(change)
        command_error = int(round(np.clip(self.filtered_error, -100, 100)))

        if abs(command_error) <= ERROR_DEADBAND:
            command_error = 0

        return command_error

    def command_during_brief_loss(self):
        self.arrival.fresh_entry = False
        self.pace = min(self.pace, TURN_PACE)
        if (
            self.last_valid_at is not None
            and time.monotonic() - self.last_valid_at
            <= TRACK_LOSS_GRACE_SECONDS
        ):
            # A held command is explicitly low confidence in GUIDE, so it
            # cannot overpower a fresh local IR reading as a real observation.
            self.camera_quality = 25
            self.turn_proximity *= 0.85
            return int(round(np.clip(self.filtered_error, -100, 100)))

        # Do not continue predicting after the physical end of the tape.
        self.history.clear()
        self.filtered_error = 0.0
        self.last_valid_at = None
        self.previous_live_error = None
        self.committed_turn_sign = 0
        self.committed_turn_strength = 0.0
        self.committed_turn_until = 0.0
        self.camera_quality = 0
        self.turn_proximity = 0.0
        return None


def find_arduino_port():
    """Find the Arduino even when Linux changes USB0 to USB1."""
    candidates = sorted(
        glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyACM*")
    )

    if not candidates:
        raise RuntimeError(
            "Arduino USB serial port not found. "
            "Connect its USB cable to the Raspberry Pi."
        )

    return candidates[0]


def send_command(arduino, command):
    arduino.write((command + "\n").encode("ascii"))
    arduino.flush()


def drive_command(command, steering_controller, guide_protocol):
    if guide_protocol and command.startswith("STEER,"):
        error = int(command.split(",", 1)[1])
        quality = int(np.clip(steering_controller.camera_quality, 0, 100))
        bend = int(np.clip(steering_controller.turn_proximity * 100, 0, 100))
        if guide_protocol == 3:
            near = int(round(np.clip(steering_controller.near_error, -100, 100)))
            preview = int(round(np.clip(steering_controller.preview_error, -100, 100)))
            return f"GUIDE2,{near},{preview},{quality},{bend}"
        if guide_protocol == 4:
            near = int(round(np.clip(steering_controller.near_error, -100, 100)))
            preview = int(round(np.clip(steering_controller.preview_error, -100, 100)))
            planned = int(steering_controller.arrival.fresh_entry and quality >= 70)
            return f"GUIDE3,{near},{preview},{quality},{bend},{steering_controller.pace},{planned}"
        if guide_protocol == 7:
            plan = steering_controller.arrival
            phase = 0
            now_ = time.monotonic()
            latch = steering_controller.corner_latch
            if plan.confirmations >= 3 and quality >= 70:
                phase = 2 if plan.phase == 'TURN' and plan.fresh_entry else 1 if plan.phase in {'APPROACH', 'TURN'} else 0
                at_bottom = plan.row is not None and plan.height and plan.row >= plan.height * BLIND_ROW_FRACTION
                if latch is not None and plan.sign and latch[0] != plan.sign and not at_bottom:
                    latch = steering_controller.corner_latch = None  # a clearly visible bend the other way wins
                tdeg = steering_controller.turn_deg
                if plan.phase == 'TURN' and plan.sign and tdeg * plan.sign >= 15:
                    latch = steering_controller.corner_latch = [plan.sign, float(tdeg), now_,
                                                                bool(latch and latch[3] and latch[0] == plan.sign)]
            if latch is not None and (now_ - latch[2] > CORNER_LATCH_S or steering_controller.feedback_pivot):
                latch = steering_controller.corner_latch = None
            if (latch is not None and not latch[3] and plan.row is not None and plan.height and
                    plan.row >= plan.height * BLIND_ROW_FRACTION):
                latch[3] = True  # the announced corner is now at the picture bottom, whatever the planner says
            if (plan.blind_now and plan.confirmations >= 3) or (latch is not None and latch[3]):
                phase = 3  # corner reached the blind zone: start the Uno's dead-reckoning timer
            # near_control is the short-carrot bearing. turnDeg is a FUTURE
            # corner (0 unless a bend is seen), never the lateral offset.
            near = int(round(np.clip(steering_controller.near_control, -100, 100)))
            turn = int(round(np.clip(steering_controller.turn_deg, -120, 120))) if phase else 0
            if phase == 3 and latch is not None and abs(turn) < 15:
                turn = int(round(np.clip(latch[0] * max(20.0, abs(latch[1])), -120, 120)))  # keep the announced direction
            pace = int(np.clip(steering_controller.pace, 35, 100))
            steering_controller.turn_sent = turn
            return f"GUIDE5,{near},{turn},{quality},{bend},{pace},{phase}"
        if guide_protocol in (5, 6):
            plan = steering_controller.arrival
            phase = 0
            preview = steering_controller.preview_error
            if plan.phase == 'TURN MEMORY':
                phase = 3
            elif plan.confirmations >= 3 and quality >= 70:
                phase = 2 if plan.phase == 'TURN' and plan.fresh_entry else 1 if plan.phase == 'APPROACH' else 0
                # This is a FUTURE direction, not the camera's lateral offset.
                if phase:
                    preview = plan.preview * STEERING_DIRECTION
            near = int(round(np.clip(steering_controller.near_control if guide_protocol == 6 else
                                     steering_controller.near_error, -100, 100)))
            preview = int(round(np.clip(preview, -100, 100)))
            return f"GUIDE4,{near},{preview},{quality},{bend},{steering_controller.pace},{phase}"
        return f"GUIDE,{error},{quality},{bend}"
    return command


def strip_is_bordered(strip, gray, floor_level):
    """True if floor-bright pixels lie just outside BOTH sides of the strip.

    The adaptive (local) threshold turns the rim of any wide dark area into a
    tape-width strip. Its inner side is dark, so it fails here. Sides outside the
    image cannot be judged and are ignored; too little contrast cannot be judged.
    """
    rows = np.flatnonzero(strip.any(axis=1))
    if rows.size < 8:
        return True
    median = float(np.median(gray[strip > 0]))
    if floor_level - median < 25.0:
        return True
    limit = median + 0.35 * (floor_level - median)
    width = strip.shape[1]
    fractions = []
    for side in (-1, 1):
        checks = passes = 0
        for row in rows[::4]:
            xs = np.flatnonzero(strip[row])
            x = int(xs[0]) - 5 if side < 0 else int(xs[-1]) + 5
            if 0 <= x < width:
                checks += 1
                passes += int(gray[row, x] >= limit)
        if checks >= 3:
            fractions.append(passes / checks)
    return (not fractions) or min(fractions) >= 0.6


def tape_candidate_score(region, gray, floor_level, previous_near_x=None):
    """Compare actual near-connected pixels, not filled contour area."""
    height, width = region.shape
    area = cv2.countNonZero(region)
    x, y, w, h = cv2.boundingRect(region)
    if area < MIN_TAPE_AREA or h < MIN_TAPE_HEIGHT:
        return None
    if area / max(h, 1) > width * MAX_TAPE_AVERAGE_WIDTH_FRACTION:
        return None
    reaches_driving_area = y + h >= height * 0.65
    if (x <= SIDE_EDGE_MARGIN or x + w >= width - SIDE_EDGE_MARGIN) and not reaches_driving_area:
        return None
    bottom_rows = region[max(y, y + h - 8):y + h]
    _, bottom_x = np.where(bottom_rows > 0)
    if bottom_x.size == 0:
        return None
    near_x = float(np.median(bottom_x))
    reference = width / 2 if previous_near_x is None else previous_near_x
    continuity = max(0.0, 1.0 - abs(near_x - reference) / (width * 0.65))
    score = 2.6 * (y + h) / height + 1.4 * h / height + 1.6 * continuity
    # Do not reward extra shadow pixels. Prefer the darker strip when two
    # thresholds trace the same route, without making darkness a hard gate:
    # reflected/gray tape remains eligible under the other thresholds.
    darkness = (floor_level - float(np.median(gray[region > 0]))) / max(floor_level, 1)
    score += 0.9 * float(np.clip(darkness, 0.0, 1.0))
    # Tape has crisp edges. A cast shadow or a lighting gradient is soft: its
    # boundary has a small image gradient. Reward the sharper boundary.
    boundary = cv2.morphologyEx(region, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    if boundary.any():
        grad = np.hypot(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3),
                        cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3))
        sharpness = float(np.mean(grad[boundary])) / max(floor_level * 2.5, 1.0)
        score += 0.9 * float(np.clip(sharpness, 0.0, 1.0))
    return score


# A dark patch (a bag, a shoe, a floor mat) must never be read as tape or as a corner.
# Perspective gives the test: a tape leg that runs SIDEWAYS across the picture is nearly
# edge-on to a camera ~10 cm above the floor, so it is far thinner in the image than the
# strip is wide (about strip_width * height / distance). A sideways dark band as thick as
# the strip itself is an upright object, not tape.
CAMERA_FOCAL_PX_AT_320 = 317.0  # ~53 deg horizontal field of view (Picamera spec)
CUE_MIN_ROW = 8
BEND_MIN_ROWS = 6
LATERAL_HORIZON_ROWS = 50.0     # generous: real horizon sits 30 rows above to 50 below the ROI top
LATERAL_MIN_EXTRA_WIDTH = 24.0


def lateral_thickness_limit(row, strip_width, frame_width):
    """Thickest believable sideways tape leg (px) whose junction is at ROI row `row`."""
    focal = CAMERA_FOCAL_PX_AT_320 * frame_width / 320.0
    return strip_width * min(1.1, 2.0 * (row + LATERAL_HORIZON_ROWS) / focal) + 2.0


def near_width_limit(bottom_row, frame_width):
    """Thickest believable tape (px) whose nearest visible row is `bottom_row`.

    1.9 cm tape is at most ~30-40 px thick at the bottom of the picture (this limit has ~2x margin), thinner
    higher up; 4.7 cm tape is 2.5x that. A far wider dark shape (a bag seen on its own) is not tape."""
    return frame_width / 320.0 * (0.30 * (bottom_row + 50.0) + 14.0) * (TAPE_WIDTH_CM / TAPE_WIDTH_TUNED_CM)


def corner_cue(region, row, last_run, info):
    """The traced strip was cut at `row` because the tape widened sideways.

    Look at the component just above the cut: a leg that runs off to ONE side
    is a corner (side -1 left / +1 right). Both sides (a T) or neither gives
    side 0 and never starts a turn. A fat dark patch is not a leg either.
    """
    if info is None or last_run is None:
        return
    if row < CUE_MIN_ROW:
        return          # the trace reached the top of the picture: no corner evidence
    centre = (float(last_run[0]) + float(last_run[-1])) * 0.5
    band = region[max(0, row - 10):row + 3]
    cols = np.flatnonzero(band.any(axis=0))
    if cols.size == 0:
        return
    left_extent, right_extent = centre - float(cols[0]), float(cols[-1]) - centre
    info['wide_row'] = float(row)
    info['side'] = (-1 if left_extent >= 24 and left_extent >= 2.0 * max(right_extent, 8.0) else
                    1 if right_extent >= 24 and right_extent >= 2.0 * max(left_extent, 8.0) else 0)


def _small_median(values):
    """np.median for a short list, without numpy's per-call overhead (same value)."""
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    return float(ordered[mid]) if n % 2 else (ordered[mid - 1] + ordered[mid]) * 0.5


def trace_ground_strip(region, reference=None, info=None):
    """Trace the near strip; stop if it opens into a broad attached patch.

    Distance to the mask boundary measures thickness normal to the tape.
    Unlike horizontal run width, it does not mistake a diagonal/sharp turn
    for a thick patch. Freeze the near width instead of letting a gradually
    widening shadow continually enlarge the accepted width.
    """
    height, width = region.shape
    result = np.zeros_like(region)
    reference = width / 2 if reference is None else reference
    normal_distance = cv2.distanceTransform(region, cv2.DIST_L2, 3)
    near_widths = []
    normal_width_limit = None
    began_with_visible_edges = None
    last_run = None
    last_row = None
    leg_run = None
    leg_widths = []
    leg_typical = None
    recent_widths = []
    for row in range(height - 1, -1, -1):
        xs = np.flatnonzero(region[row])
        if xs.size < 3:
            if last_row is not None and last_row - row > 3:
                break
            continue
        if xs[-1] - xs[0] + 1 == xs.size:
            runs = [xs]
        else:
            runs = np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1)
            runs = [run for run in runs if run.size >= 3]
        if last_run is not None:
            runs = [run for run in runs if run[0] <= last_run[-1] + 8 and run[-1] >= last_run[0] - 8]
        if not runs:
            break
        run = min(runs, key=lambda run: abs((float(run[0]) + float(run[-1])) * 0.5 - reference))
        clipped = run[0] <= SIDE_EDGE_MARGIN or run[-1] >= width - 1 - SIDE_EDGE_MARGIN
        if began_with_visible_edges is None:
            began_with_visible_edges = not clipped
        elif began_with_visible_edges and clipped and normal_width_limit is not None:
            # Once this strip leaves the view, do not resume on an attached
            # wall/shadow/return leg above it. Its hidden continuation is not
            # a measured path. Keep the visible approach for turn guidance.
            corner_cue(region, row, leg_run if leg_run is not None else last_run, info)
            break
        normal_width = 2.0 * float(np.max(normal_distance[row, run]))
        horizontal_width = float(run[-1] - run[0] + 1)
        if len(recent_widths) >= 6:
            strip_here = _small_median(recent_widths[-8:])
            if horizontal_width >= max(2.0 * strip_here, strip_here + LATERAL_MIN_EXTRA_WIDTH):
                # The strip opens sideways here. Measure the thing that starts here.
                above = slice(max(0, row - 28), max(1, row - 1))
                inside = region[above, run[0]:run[-1] + 1] > 0
                if inside.any():
                    thickness = 2.0 * float(np.percentile(normal_distance[above, run[0]:run[-1] + 1][inside], 90))
                    if thickness > lateral_thickness_limit(row, strip_here, width):
                        if info is not None:
                            info['patch'] = True
                            info['side'] = 0
                        break
        if normal_width_limit is not None and normal_width > normal_width_limit:
            corner_cue(region, row, leg_run if leg_run is not None else last_run, info)
            break
        result[row, run] = 255
        if len(recent_widths) < 6 or horizontal_width < max(2.0 * strip_here, strip_here + LATERAL_MIN_EXTRA_WIDTH):
            recent_widths.append(horizontal_width)
        if len(leg_widths) < 12:
            leg_widths.append(horizontal_width)
            leg_run = run
        else:
            if leg_typical is None:
                leg_typical = _small_median(leg_widths)
            typical = leg_typical
            if horizontal_width <= max(1.5 * typical, typical + 14.0):
                leg_run = run
        if len(near_widths) < 12:
            near_widths.append(normal_width)
            if len(near_widths) == 12:
                normal_width_limit = max(18.0, float(np.percentile(near_widths, 75)) * 2.2)
        last_run, last_row = run, row
        reference = (float(run[0]) + float(run[-1])) * 0.5
    if info is not None and near_widths:
        info['near_width'] = float(np.median(near_widths))
    return result


def process_frame(rgb_frame, steering_controller=None):
    height, width = rgb_frame.shape[:2]
    roi_top = int(height * ROI_TOP_FRACTION)
    roi = rgb_frame[roi_top:height, 0:width]

    gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    if steering_controller is not None:
        steering_controller.path_memory.advance(blurred, steering_controller.feedback_yaw)
        steering_controller.memory_used = False
        steering_controller.arrival.fresh_entry = False

    # Estimate the floor from the brighter part of the road view.  In the
    # supplied camera image this keeps the maroon-looking black tape while
    # dropping the lighter gray/green floor and most shadows.
    floor_level = float(np.percentile(blurred, 70))
    if steering_controller is not None:
        if steering_controller.floor_level is None:
            steering_controller.floor_level = floor_level
        else:
            steering_controller.floor_level += 0.25 * (floor_level - steering_controller.floor_level)
        floor_level = steering_controller.floor_level
    black_threshold = int(np.clip(
        floor_level - FLOOR_DARKNESS_GAP,
        MIN_BLACK_THRESHOLD,
        MAX_BLACK_THRESHOLD,
    ))
    _, mask = cv2.threshold(
        blurred,
        black_threshold,
        255,
        cv2.THRESH_BINARY_INV,
    )

    noise_kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        noise_kernel,
    )

    # Reconnect short bright gaps caused by reflections along the tape while
    # keeping the operation narrow enough not to merge nearby floor shadows.
    glare_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (
            GLARE_CLOSE_KERNEL_WIDTH,
            GLARE_CLOSE_KERNEL_HEIGHT,
        ),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        glare_kernel,
    )

    # Report evidence only. The Arduino requires both camera and IR tape;
    # an ultrasonic target alone cannot authorize movement.
    command = "TRACK_LOST"
    label = "CAMERA TRACK LOST"
    label_colour = (0, 0, 255)

    display = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
    selected_for_display = np.zeros_like(mask)
    prior_near_x = None
    if steering_controller is not None:
        steering_controller.camera_quality = 0
        steering_controller.lock_message = "SIMPLE TAPE DETECTOR"
        if (steering_controller.last_observation_at is not None and
                time.monotonic() - steering_controller.last_observation_at < 0.5):
            prior_near_x = steering_controller.previous_near_x
    cv2.rectangle(
        display,
        (0, roi_top),
        (width - 1, height - 1),
        (255, 255, 0),
        1,
    )
    cv2.line(
        display,
        (width // 2, roi_top),
        (width // 2, height - 1),
        (255, 0, 0),
        1,
    )

    # Small cleanup only: keep the tolerant mask and add a darker seed as a
    # candidate, so a lighter attached shadow need not win over black tape.
    _, dark_seed = cv2.threshold(blurred, min(black_threshold, int(floor_level * 0.55)),
                                 255, cv2.THRESH_BINARY_INV)
    dark_seed = cv2.morphologyEx(dark_seed, cv2.MORPH_OPEN, noise_kernel)
    dark_seed = cv2.morphologyEx(dark_seed, cv2.MORPH_CLOSE, glare_kernel)
    # An optional local mask helps when a shadow changes floor brightness.
    local_mask = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                       cv2.THRESH_BINARY_INV, 51, 12)
    local_mask = cv2.morphologyEx(local_mask, cv2.MORPH_OPEN, noise_kernel)
    local_mask = cv2.morphologyEx(local_mask, cv2.MORPH_CLOSE, glare_kernel)
    # Keep actual connected pixels, including holes. Filling an external
    # outline alone can invent foreground in the floor inside a bend/loop.
    # Evaluate the near-strip traces BEFORE choosing the winner, so a large
    # shadow cannot win first and discard a cleaner tape candidate.
    valid_tapes = []
    compared_masks = []
    for source_mask in (mask, dark_seed, local_mask):
        if any(np.array_equal(source_mask, previous) for previous in compared_masks):
            continue
        compared_masks.append(source_mask)
        count, components, stats, _ = cv2.connectedComponentsWithStats(source_mask, connectivity=8)
        for component in range(1, count):
            if (stats[component, cv2.CC_STAT_AREA] < MIN_TAPE_AREA or
                    stats[component, cv2.CC_STAT_HEIGHT] < MIN_TAPE_HEIGHT):
                continue
            actual_pixels = np.where(components == component, 255, 0).astype(np.uint8)
            strip_info = {}
            strip = trace_ground_strip(actual_pixels, prior_near_x, strip_info)
            score = tape_candidate_score(strip, blurred, floor_level, prior_near_x)
            if score is not None and strip_info.get('near_width'):
                strip_rows = np.flatnonzero(strip.any(axis=1))
                if (strip_rows.size and strip_info['near_width'] > near_width_limit(int(strip_rows[-1]), width) and
                        float(np.median(blurred[strip > 0])) < 0.3 * floor_level):
                    score -= 1.5                # a black shape far fatter than tape (a bag), not tape
                    strip_info['upright'] = True
            if score is not None:
                if not strip_is_bordered(strip, blurred, floor_level):
                    score -= 1.5            # rim of a shadow / dark area, not a tape with floor around it
                    strip_info['unbordered'] = True
                known = None if steering_controller is None else steering_controller.tape_width
                if (known and strip_info.get('near_width') and steering_controller.tape_width_at is not None and
                        time.monotonic() - steering_controller.tape_width_at < 3.0):
                    ratio = strip_info['near_width'] / known
                    strip_info['width_ratio'] = ratio
                    if ratio > 1.8 or ratio < 0.5:
                        score -= 0.6        # mild preference for the learned tape width (V7.6: was 1.5, which could pick a thin edge strip over the real band)
                valid_tapes.append((score, strip, strip_info))

    if valid_tapes:
        _, selected_mask, selected_info = max(valid_tapes, key=lambda item: item[0])
        if cv2.countNonZero(selected_mask) >= MIN_TAPE_AREA:
            roi_height = roi.shape[0]
            centreline_y = []
            centreline_x = []

            for row in range(selected_mask.shape[0]):
                row_x = np.flatnonzero(selected_mask[row])
                if row_x.size >= 3:
                    centreline_y.append(float(row))
                    # trace_ground_strip emits one contiguous run per row.
                    centreline_x.append((float(row_x[0]) + float(row_x[-1])) * 0.5)

            if len(centreline_y) < CENTERLINE_MIN_ROWS:
                centreline_y = []
            else:
                centreline_y = np.asarray(centreline_y, dtype=np.float32)
                centreline_x = np.asarray(centreline_x, dtype=np.float32)

                y_min = float(np.min(centreline_y))
                y_max = float(np.max(centreline_y))
                visible_span = max(y_max - y_min, 1.0)

                # All three evaluation points stay inside the actually seen
                # tape, avoiding unsafe extrapolation beyond its endpoint.
                far_y_roi = y_min + 0.15 * visible_span
                mid_y_roi = y_min + 0.50 * visible_span
                # A fixed near image band must not jump ahead simply because
                # the far part vanishes. Clamp to genuinely observed pixels.
                near_y_roi = float(np.clip(roi_height * 0.92, y_min, y_max))

                # Measure each target in a local band instead of forcing one
                # polynomial through the whole path.  This follows both
                # smooth curves and sharp corners without an artificial
                # S-shaped fit.
                band_half_height = max(3.0, visible_span * 0.08)

                def band_centre_x(y_value):
                    in_band = (
                        np.abs(centreline_y - y_value)
                        <= band_half_height
                    )

                    if np.any(in_band):
                        return float(np.median(centreline_x[in_band]))

                    nearest = int(np.argmin(
                        np.abs(centreline_y - y_value)
                    ))
                    return float(centreline_x[nearest])

                far_x = band_centre_x(far_y_roi)
                mid_x = band_centre_x(mid_y_roi)
                near_x = band_centre_x(near_y_roi)

                bend_threshold_pixels = max(10.0, width * 0.035)
                # A bend needs many rows that really leave the near line; a lone
                # outlier row (a speck, the edge of a dark object) is not a bend.
                try:
                    padded = np.pad(centreline_x, 4, mode='edge')
                    steady_x = np.median(np.lib.stride_tricks.sliding_window_view(padded, 9), axis=1)
                except (ValueError, AttributeError):
                    steady_x = centreline_x
                bend_rows = centreline_y[
                    np.abs(steady_x - near_x)
                    >= bend_threshold_pixels
                ]
                if bend_rows.size < BEND_MIN_ROWS:
                    bend_rows = bend_rows[:0]

                if bend_rows.size > 0:
                    bend_y_roi = float(np.max(bend_rows))
                    turn_proximity = float(np.clip(
                        bend_y_roi / max(roi_height - 1, 1),
                        0.0,
                        1.0,
                    ))
                else:
                    turn_proximity = 0.0

                pixel_to_error = 200.0 / width
                near_error = (near_x - width / 2) * pixel_to_error
                mid_error = (mid_x - width / 2) * pixel_to_error
                far_error = (far_x - width / 2) * pixel_to_error
                heading_error = (far_x - near_x) * pixel_to_error
                curvature_error = (
                    far_x - 2.0 * mid_x + near_x
                ) * pixel_to_error

                bend_urgency = float(np.clip(
                    (
                        turn_proximity - BEND_URGENCY_START
                    ) / (
                        BEND_URGENCY_FULL - BEND_URGENCY_START
                    ),
                    0.0,
                    1.0,
                ))

                near_weight = (
                    NEAR_WEIGHT_FAR_BEND
                    + bend_urgency
                    * (
                        NEAR_WEIGHT_CLOSE_BEND
                        - NEAR_WEIGHT_FAR_BEND
                    )
                )
                far_weight = (
                    FAR_WEIGHT_FAR_BEND
                    + bend_urgency
                    * (
                        FAR_WEIGHT_CLOSE_BEND
                        - FAR_WEIGHT_FAR_BEND
                    )
                )
                heading_weight = (
                    HEADING_WEIGHT_FAR_BEND
                    + bend_urgency
                    * (
                        HEADING_WEIGHT_CLOSE_BEND
                        - HEADING_WEIGHT_FAR_BEND
                    )
                )
                curvature_weight = (
                    CURVATURE_WEIGHT_FAR_BEND
                    + bend_urgency
                    * (
                        CURVATURE_WEIGHT_CLOSE_BEND
                        - CURVATURE_WEIGHT_FAR_BEND
                    )
                )

                raw_error = (
                    near_weight * near_error
                    + MID_POSITION_WEIGHT * mid_error
                    + far_weight * far_error
                    + heading_weight * heading_error
                    + curvature_weight * curvature_error
                )
                raw_error *= STEERING_GAIN * STEERING_DIRECTION
                raw_error = int(round(np.clip(raw_error, -100, 100)))

                span_quality = min(1.0, visible_span / max(roi_height * 0.65, 1))
                contrast = floor_level - float(np.median(blurred[selected_mask > 0]))
                quality = int(round(100 * (0.45 * span_quality +
                              0.35 * np.clip(contrast / 65, 0, 1) +
                              0.20 * np.clip(y_max / max(roi_height - 1, 1), 0, 1))))
                quality = int(quality * min(1.0, visible_span / max(roi_height * 0.40, 1)))
                contrast_ratio = contrast / max(float(floor_level), 1.0)
                reject_why = ''
                if BLACK_GATES:
                    if contrast_ratio < CONTRAST_RATIO_REJECT:
                        quality = int(quality * 0.3)    # pale patch, not tape: also never teaches the tape width
                        reject_why = 'PALE'
                    elif contrast_ratio < CONTRAST_RATIO_WEAK:
                        quality = int(quality * 0.6)
                        reject_why = 'PALE'
                if steering_controller is not None:
                    steering_controller.reject_why = reject_why if quality < 35 else ''
                width_ratio = selected_info.get('width_ratio')
                if selected_info.get('unbordered'):
                    quality = int(quality * 0.6)
                if selected_info.get('upright'):
                    quality = int(quality * 0.3)     # not tape: never drives, never teaches the tape width
                if steering_controller is not None and not (width_ratio is not None and (width_ratio > 2.2 or width_ratio < 0.4)):
                    steering_controller.width_streak = 0
                if width_ratio is not None and (width_ratio > 2.2 or width_ratio < 0.4):
                    quality = int(quality * 0.6)    # a lone band of the wrong width is weak evidence
                    # V7.6: ...but when the same 'odd' width keeps showing up (the camera was moved, or a thin edge strip
                    # was learned by mistake) the learned width is what is wrong: re-learn it after 6 frames.
                    if (steering_controller is not None and selected_info.get('near_width') and not selected_info.get('side') and
                            not selected_info.get('unbordered') and not selected_info.get('upright') and
                            contrast_ratio >= CONTRAST_RATIO_WEAK and
                            not selected_mask[:, :3].any() and not selected_mask[:, -3:].any()):
                        steering_controller.width_streak = getattr(steering_controller, 'width_streak', 0) + 1
                        if steering_controller.width_streak >= 6:
                            steering_controller.tape_width = float(selected_info['near_width'])
                            steering_controller.tape_width_at = time.monotonic()
                            steering_controller.width_streak = 0
                elif (steering_controller is not None and quality >= 80 and not selected_info.get('side') and
                      not selected_info.get('unbordered') and not selected_info.get('upright') and
                      selected_info.get('near_width') and
                      not selected_mask[:, :3].any() and not selected_mask[:, -3:].any()):
                    # Learn the tape width only from a strip that is fully inside the picture, and
                    # (after a gap) only once three frames in a row agree: one frame of a bag's
                    # edge or a shadow must not retrain it.
                    now_width = time.monotonic()
                    seen_width = float(selected_info['near_width'])
                    if (steering_controller.tape_width is not None and steering_controller.tape_width_at is not None and
                            now_width - steering_controller.tape_width_at < 3.0):
                        steering_controller.tape_width += 0.1 * (seen_width - steering_controller.tape_width)
                        steering_controller.tape_width_at = now_width
                    else:
                        pending = steering_controller.width_pending
                        if pending and now_width - pending[2] < 0.6 and abs(seen_width - pending[0]) <= 0.25 * pending[0]:
                            pending = (0.5 * (pending[0] + seen_width), pending[1] + 1, now_width)
                        else:
                            pending = (seen_width, 1, now_width)
                        steering_controller.width_pending = pending
                        if pending[1] >= 3:
                            steering_controller.tape_width = pending[0]
                            steering_controller.tape_width_at = now_width
                preview_error = float(np.clip(1.25 * heading_error + 0.25 * curvature_error, -100, 100))
                # Far geometry warns of an approach even while the fixed
                # steering lookahead is still on the straight. The nearest
                # deviation supplies direction, not a later leg of an S bend.
                live_bend_preview = preview_error
                turn_estimate = 0.0
                cue_side = int(selected_info.get('side', 0))
                cue_bend_row = (float(selected_info['wide_row'])
                                if cue_side and not len(bend_rows) and 'wide_row' in selected_info else None)
                if cue_bend_row is not None:
                    # The strip ran into a perpendicular leg: the vertex is at this row.
                    live_bend_preview = float(cue_side * max(30.0, abs(live_bend_preview)))
                    turn_estimate = 90.0 * cue_side
                    turn_proximity = max(turn_proximity, float(np.clip(
                        cue_bend_row / max(roi_height - 1, 1), 0.0, 1.0)))
                if len(bend_rows):
                    # Image-space angle between the approach leg and the leg
                    # beyond the bend. It only BOUNDS the Arduino's gyro turn
                    # (perspective distorts it); the turn ends on the exit leg.
                    bend_y = float(np.max(bend_rows))
                    at_bend = int(np.argmin(np.abs(centreline_y - bend_y)))
                    bend_pt = (float(centreline_x[at_bend]), bend_y)
                    far_i = int(np.argmin(centreline_y))
                    far_vec = (float(centreline_x[far_i]) - bend_pt[0], float(centreline_y[far_i]) - bend_pt[1])
                    sign_t = (float(np.sign(far_vec[0])) if abs(far_vec[0]) >= 4 else
                              float(np.sign(bend_pt[0] - near_x) or 1.0))
                    if abs(far_vec[0]) + abs(far_vec[1]) < 6:
                        far_angle = 90.0 * sign_t
                    else:
                        far_angle = float(np.degrees(np.arctan2(far_vec[0], -far_vec[1])))
                    bottom_i = int(np.argmax(centreline_y))
                    near_vec = (bend_pt[0] - float(centreline_x[bottom_i]), float(centreline_y[bottom_i]) - bend_y)
                    near_angle = (float(np.degrees(np.arctan2(near_vec[0], near_vec[1])))
                                  if near_vec[1] >= 8 else 0.0)
                    turn_estimate = float(np.clip(far_angle - near_angle, -120, 120))
                cue_turn = bool(cue_side) and 'wide_row' in selected_info
                if cue_turn:
                    # Perspective makes the image angle unreliable; a leg running off
                    # one side is a corner. It only bounds the gyro turn (a larger
                    # bound is the safe error: the turn ends on the exit leg).
                    turn_estimate = cue_side * max(90.0, abs(turn_estimate) if turn_estimate * cue_side > 0 else 0.0)
                if steering_controller is not None:
                    steering_controller.last_cue = {'side': cue_side, 'row': cue_bend_row, 'bend_rows': int(len(bend_rows)),
                                                    'turn': round(float(turn_estimate), 1), 'near_width': selected_info.get('near_width'),
                                                    'patch': bool(selected_info.get('patch')), 'wide_row': selected_info.get('wide_row')}
                    previous_turn = steering_controller.turn_deg
                    steering_controller.turn_deg = (turn_estimate if cue_bend_row is not None or cue_turn or
                                                    previous_turn * turn_estimate <= 0 or not turn_estimate
                                                    else 0.6 * previous_turn + 0.4 * turn_estimate)
                if len(bend_rows):
                    bend_index = int(np.argmin(abs(centreline_y - max(bend_rows))))
                    live_bend_preview = float(np.sign(centreline_x[bend_index] - near_x) *
                                              max(25.0, abs(preview_error)))
                if steering_controller is not None and quality >= 50:
                    memory = steering_controller.path_memory
                    observed = np.column_stack((centreline_x, centreline_y))
                    path = memory.observe(observed)
                    # Fixed image-space lookahead: unlike fractions of the
                    # visible contour span, this does not jump forward/back
                    # when a bend is clipped. No centimetre claim without a
                    # camera-ground calibration.
                    target_y = float(np.clip(roi_height * 0.45, path[:, 1].min(), min(path[:, 1].max(), near_y_roi)))
                    target_x = float(np.interp(target_y, path[:, 1], path[:, 0]))
                    bearing = np.arctan2(target_x - near_x, max(20.0, near_y_roi - target_y))
                    preview_error = float(np.clip(1.3 * np.degrees(bearing) / 90 * 100 +
                                                  0.15 * curvature_error, -100, 100))
                    steering_controller.memory_used = memory.used
                    raw_error = int(round(np.clip((near_error + 0.7 * preview_error) *
                                                  STEERING_GAIN * STEERING_DIRECTION, -100, 100)))
                if steering_controller is not None and quality >= 35:
                    # Only the LIVE strip supplies bend approach samples. Old
                    # transported points cannot refresh arrival confirmation.
                    planned_preview, arrival_urgency = steering_controller.arrival.observe(
                        float(max(bend_rows)) if len(bend_rows) else cue_bend_row,
                        roi_height, live_bend_preview, quality,
                        yaw=steering_controller.feedback_yaw,
                        moving=steering_controller.feedback_moving,
                        frame_age=steering_controller.frame_age +
                            (time.monotonic() - steering_controller.frame_received_at
                             if steering_controller.frame_received_at is not None else 0),
                        interval=steering_controller.frame_interval,
                        full_straight=(y_min < roi_height * 0.2 and y_max > roi_height * 0.9 and
                                       abs(live_bend_preview) < 12 and not len(bend_rows) and cue_bend_row is None),
                    )
                    if steering_controller.arrival.phase in {'TURN', 'TURN MEMORY'}:
                        if preview_error * planned_preview < 0 and abs(preview_error) > 20:
                            steering_controller.arrival.reset()  # Closer live path wins.
                        else:
                            preview_error = float(np.sign(planned_preview) *
                                max(abs(preview_error), abs(planned_preview) * 0.85, 45.0))
                    turn_proximity = max(turn_proximity, arrival_urgency)
                    raw_error = int(round(np.clip((near_error + 0.7 * preview_error) *
                                                  STEERING_GAIN * STEERING_DIRECTION, -100, 100)))
                    steering_controller.camera_quality = quality
                    steering_controller.turn_proximity = turn_proximity
                    steering_controller.near_error = near_error * STEERING_DIRECTION
                    # Conservative image-space pursuit bearing to a SHORT
                    # carrot, not width-percent -> full-lock yaw. This is not
                    # a calibrated ground-plane angle. Far bend direction is
                    # still sent separately and cannot authorize early entry.
                    aim_y = float(np.clip(roi_height * 0.80, y_min, near_y_roi))
                    aim_x = band_centre_x(aim_y)
                    near_bearing = np.arctan2(aim_x - width / 2,
                                              width * 0.65 + roi_height - aim_y)
                    steering_controller.near_control = float(np.clip(
                        np.degrees(near_bearing) * 100 / 60 * STEERING_DIRECTION, -65, 65))
                    steering_controller.preview_error = preview_error * STEERING_DIRECTION
                    steering_controller.previous_near_x = near_x
                    steering_controller.last_observation_at = time.monotonic()
                    steering_controller.update_pace()
                if quality >= 35:
                    selected_for_display = selected_mask

                raw_magnitude = abs(raw_error)
                if raw_magnitude >= SHARP_TURN_ERROR:
                    turn_name = "SHARP"
                elif raw_magnitude >= MEDIUM_TURN_ERROR:
                    turn_name = "TURN"
                else:
                    turn_name = "STRAIGHT"

                if steering_controller is None:
                    error = raw_error
                elif quality >= 35:
                    error = steering_controller.update(
                        raw_error,
                        turn_proximity,
                    )
                else:
                    error = 0

                centre_x = int(round(far_x))
                centre_y = int(round(far_y_roi)) + roi_top

                command = f"STEER,{error}" if quality >= 35 else "TRACK_LOST"
                lookahead = int(round(
                    100.0 * (1.0 - far_y_roi / max(roi_height - 1, 1))
                ))
                label = (f"NEAR:{steering_controller.near_control:+.0f} "
                         f"AHEAD:{steering_controller.preview_error:+.0f} Q:{quality}"
                         if steering_controller is not None else f"{command} Q:{quality}")
                if steering_controller is not None and steering_controller.memory_used:
                    label += ' MEM'
                label_colour = (0, 255, 0)
                if quality < 35:
                    label = f"NO TAPE LOCK  Q:{quality}"
                    label_colour = (0, 165, 255)

                if quality >= 35:     # V7.4: no outline, centreline or dots for a rejected candidate
                    outlines, _ = cv2.findContours(selected_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for outline in outlines:
                        shifted = outline.copy()
                        shifted[:, 0, 1] += roi_top
                        cv2.drawContours(display, [shifted], -1, (0, 255, 0), 2)

                    # Draw a piecewise median centreline. This is both the actual
                    # measured path and a useful visual check of steering input.
                    path_points_list = []
                    bin_edges = np.linspace(y_min, y_max, 18)

                    for bin_start, bin_end in zip(
                        bin_edges[:-1],
                        bin_edges[1:],
                    ):
                        in_bin = (
                            (centreline_y >= bin_start)
                            & (centreline_y <= bin_end)
                        )

                        if np.any(in_bin):
                            path_points_list.append([
                                int(round(np.median(centreline_x[in_bin]))),
                                int(round(np.median(centreline_y[in_bin])))
                                + roi_top,
                            ])

                    path_points = np.asarray(
                        path_points_list,
                        dtype=np.int32,
                    ).reshape((-1, 1, 2))
                    cv2.polylines(
                        display,
                        [path_points],
                        False,
                        (255, 0, 255),
                        2,
                    )

                    for point_x, point_y, colour in (
                        (near_x, near_y_roi, (255, 255, 0)),
                        (mid_x, mid_y_roi, (0, 200, 255)),
                        (far_x, far_y_roi, (0, 0, 255)),
                    ):
                        cv2.circle(
                            display,
                            (
                                int(round(point_x)),
                                int(round(point_y)) + roi_top,
                            ),
                            4,
                            colour,
                            -1,
                        )

                    cv2.line(
                        display,
                        (width // 2, centre_y),
                        (centre_x, centre_y),
                        (0, 200, 255),
                        2,
                    )
                    cv2.putText(display, 'CAM AHEAD', (7, max(38, centre_y - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (0, 200, 255), 1, cv2.LINE_AA)

    if command == "TRACK_LOST" and steering_controller is not None:
        held_error = steering_controller.command_during_brief_loss()

        if held_error is not None:
            command = f"STEER,{held_error}"
            label = "BRIEF GAP - BOUNDED GUIDANCE"
            label_colour = (0, 200, 255)

    cv2.putText(
        display,
        label,
        (7, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.46,
        label_colour,
        1,
        cv2.LINE_AA,
    )

    full_mask = np.zeros((height, width), dtype=np.uint8)
    full_mask[roi_top:height, 0:width] = selected_for_display
    mask_display = cv2.cvtColor(full_mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(
        mask_display,
        f"FUSION V7.6  THR:{black_threshold}",
        (7, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.46,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    if steering_controller is not None:
        plan = steering_controller.arrival
        eta = f'{plan.eta:.2f}s' if plan.eta is not None else '--'
        phase_label = {'APPROACH': 'PREPARE', 'TURN': 'ENTRY WINDOW', 'TURN MEMORY': 'SHORT MEMORY'}.get(plan.phase, plan.phase)
        cv2.putText(mask_display, f'{phase_label} ETA:{eta} PACE:{steering_controller.pace}',
                    (7, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1, cv2.LINE_AA)
        ir_label = '--' if steering_controller.ir_now is None else f'{steering_controller.ir_now:+d}'
        cam_near = '--' if steering_controller.camera_quality < 35 else f'{steering_controller.near_error:+.0f}'
        cv2.putText(mask_display, f'IR NOW:{ir_label}  CAM NEAR:{cam_near}',
                    (7, 57), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 220, 100), 1, cv2.LINE_AA)

    if steering_controller is not None:
        steering_controller.quality_shown = steering_controller.camera_quality   # V7.6: the web page reads this, never the mid-frame value
    return command, np.hstack((display, mask_display))



class ArduinoSensorTelemetry:
    """Parse bounded sensor-status lines without blocking camera heartbeats."""

    def __init__(self):
        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.fields = None
        self.received_at = None
        self.discarding = False

    def feed(self, chunk):
        for value in chunk:
            if value == 10:
                if not self.discarding:
                    self._accept(bytes(self.buffer).decode("ascii", errors="replace").strip())
                self.buffer.clear()
                self.discarding = False
            elif self.discarding:
                continue
            elif len(self.buffer) < 256:
                self.buffer.append(value)
            else:
                self.buffer.clear()
                self.discarding = True

    def _accept(self, line):
        parts = line.split(",")
        if len(parts) < 22 or len(parts) > 52 or len(parts) % 2 != 0 or parts[0] != "SENS":
            return
        mask = parts[1]
        if len(mask) != 5 or any(value not in "01" for value in mask):
            return
        fields = dict(zip(parts[2::2], parts[3::2]))
        numeric = {"VALID", "IR", "CAM", "CMD", "YAW", "RATE", "IMU", "L", "R"}
        optional = {"Q", "N", "SAT", "CQ", "BEND", "PIVOT", "ERR", "ARM", "REC", "YR", "EFF", "TS", "D", "W"}
        if (len(fields) != (len(parts) - 2) // 2 or
                not numeric | {"SRC"} <= set(fields) or
                not set(fields) <= numeric | {"SRC"} | optional):
            return
        try:
            for name in numeric | (set(fields) & optional):
                fields[name] = int(fields[name])
        except ValueError:
            return
        if fields["SRC"] not in {"FAULT", "OFF", "STOP", "BOTH", "IR", "CAM", "ACC", "LOST", "ALIGN", "CHECK", "CENTER",
                                 "REC_CAM", "REC_IR", "REJOIN", "REC_STOP", "TURN_MEM", "CAM_TRACK", "TURN"}:
            return
        if fields["VALID"] not in (0, 1) or fields["IMU"] not in (0, 1):
            return
        if any(not -100 <= fields[name] <= 100 for name in ("IR", "CAM", "CMD")):
            return
        if any(not 0 <= fields[name] <= 255 for name in ("L", "R")):
            return
        if any(not 0 <= fields[name] <= 100 for name in ("Q", "CQ", "BEND") if name in fields):
            return
        if "N" in fields and not 0 <= fields["N"] <= 5:
            return
        if any(fields[name] not in (0, 1) for name in ("SAT", "PIVOT", "ARM") if name in fields):
            return
        if "ERR" in fields and not 0 <= fields["ERR"] <= 6:
            return
        if "REC" in fields and not 0 <= fields["REC"] <= 6000:
            return
        if any(not -100 <= fields[name] <= 100 for name in ('YR', 'EFF') if name in fields):
            return
        if "TS" in fields and not 0 <= fields["TS"] <= 4:
            return
        if "D" in fields and not 0 <= fields["D"] <= 999:
            return
        if "W" in fields and not 0 <= fields["W"] <= 6:
            return
        fields["mask"] = mask
        with self.lock:
            self.fields = fields
            self.received_at = time.monotonic()

    def snapshot(self):
        with self.lock:
            fields = dict(self.fields) if self.fields is not None else {}
            age = None if self.received_at is None else time.monotonic() - self.received_at
        return {"fresh": age is not None and age < 1.0, "age": age, "values": fields}

    def summary(self):
        status = self.snapshot()
        if not status["fresh"]:
            return "Arduino sensor telemetry waiting / stale"
        values = status["values"]
        mask = "".join("#" if value == "1" else "." for value in values["mask"])
        return (
            f"IR:{mask} CMD:{values['CMD']:+d} "
            f"L/R:{values['L']}/{values['R']} "
            f"YawR:{values['YAW']:+d} RateR:{values['RATE']:+d} "
            f"IMU:{values['IMU']} {values['SRC']}"
            + (" WIDE-TAPE" if values.get("SAT") else "")
            + (" PIVOT" if values.get("PIVOT") else "")
            + (f" FAULT-{['OK', 'STARTUP', 'I2C', 'GAP', 'CLIP', 'MOUNT', 'TURN-SIGN'][values['ERR']]}"
               if values.get("ERR") else "")
            + (f" ARMED:{values['ARM']}" if "ARM" in values else "")
            + (f" TURN:{['-', 'L?', 'R?', 'LEFT', 'RIGHT'][values['TS']]}" if values.get("TS") else "")
            + (f" REC:{values['REC']}ms" if values.get("REC") else "")
        )


class DriveLogger:
    """Record real observations for tuning; no trained ML model is implied."""

    def __init__(self, directory=None):
        self.file = None
        self.last_recorded_at = 0.0
        self.last_flush_at = 0.0
        self.started_at = time.monotonic()
        self.path = None
        try:
            folder = Path.home() / "mavp_logs" if directory is None else Path(directory)
            folder.mkdir(parents=True, exist_ok=True)
            self.path = folder / datetime.now().strftime("drive_%Y%m%d_%H%M%S_%f.csv")
            self.file = self.path.open("w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                "seconds", "camera_command", "camera_quality", "bend_percent", "ir_mask",
                "ir_quality", "black_channels", "ir_error", "fused_command", "yaw_right_deg",
                "yaw_right_dps", "left_pwm", "right_pwm", "source", "pivot", "processing_ms",
                "camera_near_error", "camera_preview_error", "imu_fault",
                "camera_near_control",
                "track_armed",
                "recovery_remaining_ms",
                "requested_yaw_dps", "yaw_effort_percent", "image_flow_y_px_s", "path_memory_used",
                "frame_age_ms", "frame_interval_ms", "pace", "bend_eta_s", "arrival_phase",
                "distance_cm", "stop_why", "turn_sent_deg",
            ])
            print(f"Driving log: {self.path}")
        except OSError as error:
            print(f"Driving log unavailable: {error}")
            self.close()

    def record(self, command, steering, status, processing_ms):
        now = time.monotonic()
        if self.file is None or now - self.last_recorded_at < 0.1:
            return
        values = status["values"] if status["fresh"] else {}
        try:
            self.writer.writerow([
                round(now - self.started_at, 3), command, steering.camera_quality,
                round(steering.turn_proximity * 100), values.get("mask"), values.get("Q"),
                values.get("N"), values.get("IR"), values.get("CMD"), values.get("YAW"),
                values.get("RATE"), values.get("L"), values.get("R"), values.get("SRC"),
                values.get("PIVOT"), round(processing_ms, 2),
                round(steering.near_error, 2), round(steering.preview_error, 2), values.get("ERR"),
                round(steering.near_control, 2),
                values.get("ARM"),
                values.get("REC"),
                values.get("YR"), values.get("EFF"), round(steering.path_memory.flow_y_rate, 2),
                int(steering.memory_used),
                round(steering.frame_age * 1000, 2), round(steering.frame_interval * 1000, 2),
                steering.pace, steering.arrival.eta, steering.arrival.phase,
                values.get("D"), values.get("W"), getattr(steering, "turn_sent", None),
            ])
            self.last_recorded_at = now
            if now - self.last_flush_at >= 1.0:
                self.file.flush()
                self.last_flush_at = now
        except OSError as error:
            print(f"Driving log stopped: {error}")
            self.close()

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
            self.file = None


class DiagnosticRecorder:
    """Bounded 30-second/5-Hz raw-image + sensor ring, outside motor control."""

    def __init__(self, directory=None):
        self.frames = deque(maxlen=150)
        self.lock = threading.Lock()
        self.last_at = 0.0
        self.started_at = time.monotonic()
        self.directory = Path(directory) if directory is not None else None
        self.previous_source = None
        self.saved_events = 0
        self.export_worker = None

    def record(self, raw_rgb, display, steering, status, request, processing_ms):
        now = time.monotonic()
        if now - self.last_at < 0.2:
            return
        self.last_at = now
        ok_raw, raw = cv2.imencode('.jpg', cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
        ok_view, view = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok_raw or not ok_view:
            return
        metadata = {
            'seconds': round(now - self.started_at, 3), 'request': request,
            'near': round(steering.near_error, 2), 'preview': round(steering.preview_error, 2),
            'near_control': round(steering.near_control, 2),
            'camera_quality': steering.camera_quality, 'bend': round(steering.turn_proximity * 100),
            'memory_used': steering.memory_used, 'image_flow_y_px_s': round(steering.path_memory.flow_y_rate, 2),
            'processing_ms': round(processing_ms, 2), 'telemetry': status,
            'frame_age_ms': round(steering.frame_age * 1000, 2),
            'frame_interval_ms': round(steering.frame_interval * 1000, 2),
            'pace': steering.pace, 'arrival_phase': steering.arrival.phase,
            'bend_eta_s': steering.arrival.eta, 'bend_row_rate': steering.arrival.row_rate,
            'arrival_confirmations': steering.arrival.confirmations,
            'fresh_turn_entry': steering.arrival.fresh_entry,
        }
        with self.lock:
            self.frames.append((metadata, raw.tobytes(), view.tobytes()))
        values = status.get('values', {}) if status.get('fresh') else {}
        source = values.get('SRC')
        active = ACTIVE_SOURCES
        if source in {'LOST', 'REC_STOP', 'FAULT'} and self.previous_source in active:
            self.save_failure()
        self.previous_source = source

    def bundle(self):
        with self.lock:
            frames = list(self.frames)
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w') as archive:
            archive.writestr('README.txt', 'Raw camera frames, annotated view and matched sensor records.\n'
                              'Image motion is not metric distance. Pulsed inside-bank PWM may be zero intentionally.\n')
            archive.writestr('frames.json', json.dumps([item[0] for item in frames], indent=2))
            for index, (_, raw, view) in enumerate(frames):
                archive.writestr(f'raw/{index:04d}.jpg', raw)
                archive.writestr(f'view/{index:04d}.jpg', view)
        return stream.getvalue()

    def save_failure(self):
        if self.directory is None or self.saved_events >= 3 or (
                self.export_worker is not None and self.export_worker.is_alive()):
            return
        self.saved_events += 1
        path = self.directory / (datetime.now().strftime('track_failure_%Y%m%d_%H%M%S_%f') +
                                 f'_{self.saved_events}.zip')
        def save():
            try:
                path.write_bytes(self.bundle())
                print(f'Diagnostic saved: {path}')
            except OSError as error:
                print(f'Diagnostic save unavailable: {error}')
        self.export_worker = threading.Thread(target=save, daemon=True)
        self.export_worker.start()


class BenchController:
    def __init__(self):
        self.arduino = None
        self.camera = None
        self.camera_started = False
        self.worker = None

        self.stop_event = threading.Event()
        self.frame_condition = threading.Condition()
        self.latest_jpeg = None
        self.frame_number = 0
        self.worker_error = None
        self.steering_controller = PredictiveSteeringController()
        self.sensor_telemetry = ArduinoSensorTelemetry()
        self.guide_protocol = False
        self.drive_logger = None
        self.diagnostics = None
        self.frame_timing = FrameTiming()
        self.latest_raw = None

    def wait_for_arduino_ready(self):
        self.guide_protocol = False
        self.both_required_firmware = False
        self.auto_center_firmware = False
        self.bounded_recovery_firmware = False
        self.lookahead_firmware = False
        self.arrival_firmware = False
        self.rejoin_firmware = False
        self.handoff_firmware = False
        self.camera_led_firmware = False
        self.arc_guard_firmware = False
        deadline = time.monotonic() + 10.0
        print("Waiting for Arduino calibration. Keep the car upright and still.")
        while time.monotonic() < deadline:
            message = self.arduino.readline().decode("ascii", errors="replace").strip()
            if not message:
                continue
            print(f"Arduino: {message}")
            if message.startswith("SENSOR_FUSION_READY V2"):
                self.guide_protocol = True
            if message.startswith('SENSOR_FUSION_READY V5'):
                self.guide_protocol = 5
                self.handoff_firmware = 'TIMED_HANDOFF' in message
                self.camera_led_firmware = 'CAMERA_LED' in message
            if message.startswith('SENSOR_FUSION_READY V7'):
                self.guide_protocol = 7
                self.arc_guard_firmware = ('A_RIGHT' in message or 'A_LEFT' in message) and 'CORNER_TURN' in message
                self.fw72 = 'FW73' in message
                if not self.fw72:
                    print('NOTE: the Uno sketch is older than V7.3 (no FW73 in its banner). It will drive, but the log has no distance/stop reason and the new turn rules are missing.')
                elif 'V76' not in message:
                    print('NOTE: the Uno sketch is not V7.6 (no V76 in its banner). With 4.7 cm tape the older sketch can start a turn on a straight. Upload stage8_V7_6_cam22_arduino.')
            if message.startswith('SENSOR_FUSION_READY V6'):
                self.guide_protocol = 6
                self.arc_guard_firmware = 'A_RIGHT' in message and 'ARC_GUARD' in message
            if message.startswith("SENSOR_FUSION_READY V3"):
                self.guide_protocol = 3
                self.both_required_firmware = "BOTH_REQUIRED" in message
                self.auto_center_firmware = "AUTO_CENTER" in message
                self.bounded_recovery_firmware = "BOUNDED_RECOVERY" in message
                self.lookahead_firmware = "LOOKAHEAD_FOLLOW" in message
                self.arrival_firmware = "ARRIVAL_PACE" in message
                self.rejoin_firmware = "REJOIN_CRAWL" in message
                self.handoff_firmware = "TIMED_HANDOFF" in message
                if self.arrival_firmware:
                    self.guide_protocol = 4
            if message.startswith("SENSOR_FAULT"):
                raise RuntimeError("Arduino MPU calibration failed. Check mounting/wiring and keep still before restarting.")
            if message == "ARDUINO_READY":
                return
        raise RuntimeError("Arduino did not send ARDUINO_READY. Upload the driving firmware, close Serial Monitor, then retry.")

    def require_joint_track_firmware(self):
        if getattr(self, 'guide_protocol', None) == 7 and getattr(self, 'arc_guard_firmware', False):
            return
        raise RuntimeError('Upload matching FUSION V7 Arduino firmware (A_RIGHT|A_LEFT CORNER_TURN). '
                           'Older firmware cannot turn corners from IR + gyro.')

    def read_sensor_telemetry(self):
        available = self.arduino.in_waiting
        if available:
            self.sensor_telemetry.feed(self.arduino.read(min(available, 2048)))

    def start(self):
        serial_port = find_arduino_port()
        print(f"Arduino port: {serial_port}")

        self.arduino = serial.Serial(
            serial_port,
            BAUD_RATE,
            timeout=0.1,
            write_timeout=0.5,
        )

        # Opening USB serial resets an Arduino Uno.
        self.wait_for_arduino_ready()
        self.require_joint_track_firmware()
        print("FUSION V7: IR-triggered corner turns on the gyro, camera hand-back, bounded one-source recovery.")
        print(f"PI FILE V7.6c (2026-10-23): V7.6b, faster vision + tape filters for {TAPE_WIDTH_CM:g} cm tape. Use with the V7.6 Uno sketch (banner ends V76).")
        print("Recovery is time and angle bounded; STOP, stale frames and obstacles still cut the motors.")

        self.camera = Picamera2()
        configuration = self.camera.create_video_configuration(
            queue=False,
            buffer_count=4,
            main={
                "size": (FRAME_WIDTH, FRAME_HEIGHT),
                "format": "RGB888",
            },
            controls={"FrameRate": FRAME_RATE},
        )
        self.camera.configure(configuration)
        self.camera.start()
        self.camera_started = True
        time.sleep(1)
        self.drive_logger = DriveLogger()
        self.diagnostics = DiagnosticRecorder(self.drive_logger.path.parent if self.drive_logger.path else None)

        self.worker = threading.Thread(
            target=self.run_control_loop,
            name="mavp-stage8-control",
            daemon=True,
        )
        self.worker.start()

    def run_control_loop(self):
        last_command_at = 0.0
        last_print_at = 0.0
        previous_command = ""
        first_frame = True

        try:
            while not self.stop_event.is_set():
                # Metadata and pixels MUST belong to the same camera request.
                # Queue=False alone is not a guarantee that an image is fresh.
                request = self.camera.capture_request()
                try:
                    metadata = request.get_metadata()
                    rgb_frame = camera_array_to_rgb(request.make_array('main'))
                finally:
                    request.release()
                if self.stop_event.is_set():
                    break
                processing_started_at = time.monotonic()
                if not self.frame_timing.accept(metadata):
                    send_command(self.arduino, 'STOP')
                    self.steering_controller.arrival.reset()
                    print('CAMERA FRAME STALE/REPEATED/MISSING TIMESTAMP - STOPPED')
                    continue
                self.steering_controller.frame_age = self.frame_timing.age
                self.steering_controller.frame_interval = self.frame_timing.interval
                self.steering_controller.frame_received_at = processing_started_at
                if first_frame:
                    print('FRESH CAMERA FRAME RECEIVED - FUSION V7 guidance available.')
                    first_frame = False
                self.read_sensor_telemetry()
                self.steering_controller.set_sensor_feedback(self.sensor_telemetry.snapshot())
                command, display_frame = process_frame(
                    rgb_frame,
                    self.steering_controller,
                )

                now = time.monotonic()
                self.steering_controller.frame_age += now - processing_started_at
                self.steering_controller.update_pace()
                if (self.steering_controller.frame_age > MAX_FRAME_AGE_SECONDS or
                        self.frame_timing.interval > MAX_FRAME_AGE_SECONDS):
                    command = 'STOP'
                    self.steering_controller.arrival.reset()

                if now - last_command_at >= COMMAND_PERIOD_SECONDS:
                    if self.stop_event.is_set():
                        break
                    send_command(self.arduino, drive_command(command, self.steering_controller, self.guide_protocol))
                    last_command_at = now

                self.read_sensor_telemetry()

                sensor_status = self.sensor_telemetry.snapshot()
                self.drive_logger.record(
                    command, self.steering_controller, sensor_status,
                    (time.monotonic() - processing_started_at) * 1000,
                )
                if sensor_status["fresh"] and (command == "TRACK_LOST" or
                        sensor_status["values"]["SRC"] in {"ALIGN", "CHECK", "CENTER", "REC_CAM", "REC_IR", "REJOIN", "REC_STOP", "TURN_MEM", "CAM_TRACK", "TURN", "FAULT"}):
                    source = sensor_status["values"]["SRC"]
                    label = {
                        "IR": "LOCAL IR TRACK GUIDE",
                        "ACC": "OLD FIRMWARE ACC FALLBACK - STOP TEST",
                        "FAULT": ("TURN DIRECTION MISMATCH - RESET" if sensor_status['values'].get('ERR') == 6
                                  else "IMU FAULT - STOPPED"),
                        "LOST": "TRACK LOST - STOPPED",
                        "STOP": "MOTORS STOPPED",
                        "OFF": "PI HEARTBEAT OFF",
                        "ALIGN": "OLD FIRMWARE - UPDATE ARDUINO",
                        "CHECK": "CONFIRMING LIVE CAMERA",
                        "CAM_TRACK": "CAMERA TRACKING - IR OPTIONAL",
                        "CENTER": "SELF-CENTRING CAMERA + IR",
                        "REC_CAM": "CAMERA + GYRO RECOVERY",
                        "REC_IR": "IR + GYRO RECOVERY",
                        "REJOIN": "CONFIRMING TRACK REJOIN",
                        "REC_STOP": "RECOVERY LIMIT - STOPPED",
                        "TURN_MEM": "COMPLETING OBSERVED TURN",
                        "TURN": "CORNER TURN - GYRO",
                    }.get(source)
                    if label:
                        cv2.rectangle(display_frame, (0, 0), (FRAME_WIDTH, 28), (7, 17, 31), -1)
                        cv2.putText(
                            display_frame, label, (7, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (255, 220, 160), 1, cv2.LINE_AA,
                        )

                if (
                    command.split(',', 1)[0] != previous_command.split(',', 1)[0]
                    or now - last_print_at >= PRINT_PERIOD_SECONDS
                ):
                    print(command)
                    print(self.sensor_telemetry.summary())
                    previous_command = command
                    last_print_at = now

                cv2.rectangle(
                    display_frame,
                    (0, display_frame.shape[0] - 18),
                    (display_frame.shape[1], display_frame.shape[0]),
                    (7, 17, 31),
                    -1,
                )
                cv2.putText(
                    display_frame,
                    self.sensor_telemetry.summary(),
                    (6, display_frame.shape[0] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.36,
                    (255, 220, 160),
                    1,
                    cv2.LINE_AA,
                )

                encoded, jpeg = cv2.imencode(
                    ".jpg",
                    display_frame,
                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
                )

                if encoded:
                    self.latest_raw = rgb_frame
                    if self.diagnostics is not None:
                        self.diagnostics.record(rgb_frame, display_frame, self.steering_controller, sensor_status,
                            drive_command(command, self.steering_controller, self.guide_protocol),
                            (time.monotonic() - processing_started_at) * 1000)
                    with self.frame_condition:
                        self.latest_jpeg = jpeg.tobytes()
                        self.frame_number += 1
                        self.frame_condition.notify_all()

        except Exception as error:
            self.worker_error = repr(error)
            print(f"CONTROL LOOP ERROR: {error}")

        finally:
            try:
                send_command(self.arduino, "STOP")
            except (OSError, serial.SerialException):
                pass

            with self.frame_condition:
                self.frame_condition.notify_all()

    def stop(self):
        self.stop_event.set()

        if self.worker is not None:
            self.worker.join(timeout=2)

        if self.arduino is not None:
            try:
                send_command(self.arduino, "STOP")
                time.sleep(0.1)
            except (OSError, serial.SerialException):
                pass

            self.arduino.close()

        if self.camera is not None and self.camera_started:
            self.camera.stop()
        if self.drive_logger is not None:
            self.drive_logger.close()


def generate_stream():
    delivered_frame = -1

    while not controller.stop_event.is_set():
        with controller.frame_condition:
            controller.frame_condition.wait_for(
                lambda: (
                    controller.frame_number != delivered_frame
                    or controller.stop_event.is_set()
                    or controller.worker_error is not None
                ),
                timeout=1,
            )

            jpeg = controller.latest_jpeg
            delivered_frame = controller.frame_number

        if jpeg is None:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg
            + b"\r\n"
        )


DASHBOARD_HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MAVP Live</title>
<style>
  :root {
    color-scheme: dark;
    --bg: #050b16; --bg2: #0a1424;
    --panel: rgba(255,255,255,.045); --panel-hi: rgba(255,255,255,.08);
    --line: rgba(130,175,255,.20);
    --text: #e8f0ff; --dim: #8fa5c4; --faint: #5b7090;
    --accent: #4cc9f0; --accent2: #7b8cff;
    --ok: #5ef0a8; --warn: #ffcb5c; --bad: #ff6b7d;
    --mono: ui-monospace, "SF Mono", "Cascadia Mono", Consolas, "Liberation Mono", monospace;
    --sans: system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
  }
  * { box-sizing: border-box; }
  html, body { margin: 0; }
  body {
    background:
      radial-gradient(1100px 500px at 85% -10%, rgba(76,201,240,.13), transparent 60%),
      radial-gradient(900px 500px at -10% 10%, rgba(123,140,255,.12), transparent 60%),
      linear-gradient(180deg, var(--bg), var(--bg2));
    color: var(--text); font-family: var(--sans); min-height: 100vh;
  }
  body::before {            /* faint road-grid texture */
    content: ""; position: fixed; inset: 0; pointer-events: none; opacity: .5;
    background-image: linear-gradient(rgba(130,175,255,.05) 1px, transparent 1px),
                      linear-gradient(90deg, rgba(130,175,255,.05) 1px, transparent 1px);
    background-size: 44px 44px;
    mask-image: radial-gradient(ellipse at 50% 30%, #000 20%, transparent 75%);
    -webkit-mask-image: radial-gradient(ellipse at 50% 30%, #000 20%, transparent 75%);
  }
  main { position: relative; width: min(100% - 32px, 1280px); margin: 0 auto; padding: 18px 0 40px; }

  header { display: flex; align-items: center; gap: 14px; flex-wrap: wrap; margin-bottom: 16px; }
  .brand { display: flex; align-items: center; gap: 12px; margin-right: auto; }
  .brand svg { width: 46px; height: 46px; flex: none; }
  .brand h1 { font-size: 20px; margin: 0; letter-spacing: .04em; font-weight: 700; }
  .brand .sub { margin: 2px 0 0; color: var(--dim); font-size: 12.5px; letter-spacing: .02em; }
  .chip { display: inline-flex; align-items: center; gap: 8px; padding: 7px 12px; border-radius: 999px;
          background: var(--panel); border: 1px solid var(--line); font-size: 12.5px; color: var(--dim);
          font-variant-numeric: tabular-nums; }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--faint); flex: none; }
  .live .dot { background: var(--ok); box-shadow: 0 0 0 0 rgba(94,240,168,.6); animation: ping 1.6s infinite; }
  @keyframes ping { 70% { box-shadow: 0 0 0 9px rgba(94,240,168,0); } 100% { box-shadow: 0 0 0 0 rgba(94,240,168,0); } }
  header a.btn { color: var(--text); text-decoration: none; font-size: 12.5px; padding: 7px 12px; border-radius: 999px;
                 border: 1px solid var(--line); background: var(--panel); }
  header a.btn:hover { background: var(--panel-hi); }

  .grid { display: grid; gap: 14px; grid-template-columns: repeat(12, 1fr); }
  .card { background: var(--panel); border: 1px solid var(--line); border-radius: 16px; padding: 14px 16px;
          backdrop-filter: blur(6px); -webkit-backdrop-filter: blur(6px); min-width: 0; }
  .card h2 { margin: 0 0 10px; font-size: 11px; font-weight: 600; letter-spacing: .14em; text-transform: uppercase; color: var(--dim);
             display: flex; justify-content: space-between; align-items: center; gap: 8px; }
  .card h2 small { letter-spacing: .02em; text-transform: none; font-weight: 400; color: var(--faint); font-size: 11.5px; }

  .c-cam { grid-column: span 8; padding: 12px; }
  .c-state { grid-column: span 4; display: flex; flex-direction: column; justify-content: center; }
  .c-vision, .c-steer, .c-heading, .c-corner, .c-wheels, .c-ir, .c-lock, .c-sonar, .c-events { grid-column: span 4; }
  .c-trace { grid-column: span 12; }
  @media (max-width: 1000px) {
    .c-cam, .c-state, .c-trace { grid-column: span 12; }
    .c-vision, .c-steer, .c-heading, .c-corner, .c-wheels, .c-ir, .c-lock, .c-sonar, .c-events { grid-column: span 6; }
  }
  @media (max-width: 640px) { .grid > .card { grid-column: span 12 !important; } }

  .screen { position: relative; border-radius: 11px; overflow: hidden; background: #000; border: 1px solid var(--line); }
  .screen canvas { display: block; width: 100%; height: auto; }
  .screen .tag { position: absolute; left: 10px; top: 9px; font-size: 10.5px; letter-spacing: .14em; padding: 3px 8px; border-radius: 6px;
                 background: rgba(5,11,22,.72); color: var(--accent); font-weight: 600; }
  .screen .rec { position: absolute; right: 10px; top: 9px; font-size: 10.5px; letter-spacing: .1em; padding: 3px 8px; border-radius: 6px;
                 background: rgba(5,11,22,.72); color: var(--dim); font-family: var(--mono); }
  .screen::after { content: ""; position: absolute; inset: 0; pointer-events: none; border-radius: 11px;
                   box-shadow: inset 0 0 60px rgba(0,0,0,.45);
                   background: repeating-linear-gradient(0deg, rgba(255,255,255,.025) 0 1px, transparent 1px 3px); }
  .nofeed { position: absolute; inset: 0; display: none; align-items: center; justify-content: center; color: var(--dim); font-size: 14px; background: #02060d; z-index: 2; }

  /* state card */
  .state-main { font-size: 30px; font-weight: 800; letter-spacing: .02em; line-height: 1.1; margin: 4px 0 6px; transition: color .25s; }
  .state-sub { color: var(--dim); font-size: 14px; line-height: 1.4; min-height: 40px; }
  .stats { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-top: 18px; }
  .stats div { background: rgba(255,255,255,.04); border: 1px solid var(--line); border-radius: 12px; padding: 10px 12px; min-width: 0; }
  .stats b { display: block; font-family: var(--mono); font-size: 19px; font-weight: 600; white-space: nowrap; }
  .stats span { font-size: 10.5px; color: var(--faint); letter-spacing: .08em; text-transform: uppercase; }
  .state-badges { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 12px; }
  .badge { font-size: 11.5px; padding: 4px 9px; border-radius: 8px; border: 1px solid var(--line); color: var(--dim); background: rgba(255,255,255,.03); }
  .badge.on { color: var(--text); border-color: rgba(76,201,240,.55); background: rgba(76,201,240,.12); }
  .ok { color: var(--ok); } .warn { color: var(--warn); } .bad { color: var(--bad); } .dimc { color: var(--dim); } .acc { color: var(--accent); }

  /* gauges */
  .big { font-family: var(--mono); font-size: 26px; font-weight: 600; font-variant-numeric: tabular-nums; }
  .unit { color: var(--faint); font-size: 12px; margin-left: 4px; font-family: var(--sans); }
  .gauge { width: 100%; max-width: 280px; display: block; margin: 0 auto; }
  .gauge .needle { transition: transform .18s ease-out; transform-origin: 100px 100px; }
  .center { text-align: center; }
  .row { display: flex; align-items: center; gap: 10px; }
  .bar { height: 12px; border-radius: 7px; background: rgba(255,255,255,.07); overflow: hidden; position: relative; flex: 1; }
  .bar > i { position: absolute; left: 0; top: 0; bottom: 0; width: 0; border-radius: 7px; background: linear-gradient(90deg, var(--accent2), var(--accent)); transition: width .15s linear; }
  .lbl { width: 14px; color: var(--dim); font-size: 12px; font-weight: 700; }
  .val { width: 38px; text-align: right; font-family: var(--mono); font-size: 12.5px; color: var(--text); }
  .stack > * + * { margin-top: 10px; }
  .irbar { display: flex; justify-content: space-between; gap: 6px; margin: 8px 0 6px; }
  .irbar span { flex: 1; aspect-ratio: 1; max-width: 46px; border-radius: 50%; border: 2px solid var(--line); background: rgba(255,255,255,.04); transition: all .1s; }
  .irbar span.on { background: var(--accent); border-color: var(--accent); box-shadow: 0 0 14px rgba(76,201,240,.75); }
  .note { color: var(--faint); font-size: 11.5px; margin-top: 6px; line-height: 1.4; }
  .ring { width: 112px; height: 112px; display: block; margin: 0 auto; }
  .ring .arc { transition: stroke-dashoffset .2s linear, stroke .2s; }
  .compass { width: 100%; max-width: 190px; display: block; margin: 0 auto; }
  .compass .needle { transition: transform .15s ease-out; transform-origin: 90px 90px; }
  .kv { display: flex; justify-content: space-between; font-size: 13px; color: var(--dim); padding: 3px 0; }
  .kv b { color: var(--text); font-family: var(--mono); font-weight: 600; font-variant-numeric: tabular-nums; }
  .prox { height: 8px; border-radius: 5px; background: rgba(255,255,255,.07); overflow: hidden; margin-top: 8px; }
  .prox > i { display: block; height: 100%; width: 0; background: linear-gradient(90deg, var(--warn), #ff9c5c); transition: width .2s; }
  canvas.chart { width: 100%; height: 170px; display: block; }
  .legend { display: flex; gap: 14px; font-size: 11.5px; color: var(--dim); }
  .legend i { display: inline-block; width: 18px; height: 3px; border-radius: 2px; margin-right: 6px; vertical-align: middle; }
  ul.events { list-style: none; margin: 0; padding: 0; max-height: 210px; overflow: auto; font-size: 13px; }
  ul.events li { display: flex; gap: 10px; padding: 6px 0; border-bottom: 1px solid rgba(130,175,255,.10); }
  ul.events li:last-child { border-bottom: 0; }
  ul.events time { color: var(--faint); font-family: var(--mono); font-size: 11.5px; flex: none; padding-top: 1px; }
  ul.events .empty { color: var(--faint); }
  footer { margin-top: 18px; color: var(--faint); font-size: 12px; }
  details { margin-top: 12px; color: var(--dim); font-size: 13px; }
  details summary { cursor: pointer; color: var(--accent); }
  details li { margin: 6px 0; }
  details a { color: var(--accent); }
  .warnbar { color: var(--warn); font-size: 12.5px; margin: -4px 0 14px; }
  @media (prefers-reduced-motion: reduce) { * { animation: none !important; transition: none !important; } }
</style>
</head>
<body>
<main>
  <header>
    <div class="brand">
      <svg viewBox="0 0 48 48" aria-hidden="true">
        <defs><linearGradient id="lg" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#4cc9f0"/><stop offset="1" stop-color="#7b8cff"/></linearGradient></defs>
        <rect x="1.5" y="1.5" width="45" height="45" rx="12" fill="rgba(255,255,255,.04)" stroke="url(#lg)" stroke-width="1.5"/>
        <path d="M24 9 L31 17 H17 Z" fill="url(#lg)" opacity=".35"/>
        <rect x="16.5" y="17" width="15" height="22" rx="4" fill="none" stroke="url(#lg)" stroke-width="2"/>
        <rect x="14" y="20" width="3" height="6" rx="1" fill="url(#lg)"/><rect x="31" y="20" width="3" height="6" rx="1" fill="url(#lg)"/>
        <rect x="14" y="30" width="3" height="6" rx="1" fill="url(#lg)"/><rect x="31" y="30" width="3" height="6" rx="1" fill="url(#lg)"/>
        <circle cx="24" cy="21.5" r="2" fill="#4cc9f0"/>
      </svg>
      <div><h1>MAVP · LIVE</h1><div class="sub">Mini Autonomous Vehicle Platform · track follower</div></div>
    </div>
    <span class="chip" id="chipLive"><span class="dot"></span><span id="liveTxt">connecting…</span></span>
    <span class="chip" id="chipFps">— fps</span>
    <span class="chip" id="chipVer">Pi ?</span>
    <a class="btn" href="/diagnostics.zip">Download last run</a>
  </header>
  <div class="warnbar">Bench check: raise the wheels before connecting motor power.</div>

  <div class="grid">
    <section class="card c-cam">
      <h2>Camera <small>annotated live view</small></h2>
      <div class="screen">
        <canvas id="cam" width="320" height="194"></canvas>
        <span class="tag">CAMERA</span><span class="rec" id="camRec">Q —</span>
        <div class="nofeed" id="nofeed">Waiting for the camera stream…</div>
      </div>
    </section>

    <section class="card c-state">
      <h2>Car state</h2>
      <div class="state-main dimc" id="stateMain">CONNECTING</div>
      <div class="state-sub" id="stateSub">Waiting for the first sensor reading.</div>
      <div class="stats">
        <div><b id="stRun">0:00</b><span>tracking time</span></div>
        <div><b id="stTurns">0</b><span>turns done</span></div>
        <div><b id="stLast">&mdash;</b><span>last turn (gyro)</span></div>
      </div>
      <div class="state-badges">
        <span class="badge" id="bCam">CAMERA</span><span class="badge" id="bIr">IR BAR</span><span class="badge" id="bImu">GYRO</span><span class="badge" id="bArm">ARMED</span>
      </div>
    </section>

    <section class="card c-vision">
      <h2>What the car sees <small>tape mask</small></h2>
      <div class="screen"><canvas id="mask" width="320" height="138"></canvas><span class="tag">VISION</span></div>
    </section>

    <section class="card c-steer">
      <h2>Steering <small>left ◂ ▸ right</small></h2>
      <svg class="gauge" viewBox="0 0 200 118" aria-label="steering gauge">
        <path d="M20 100 A80 80 0 0 1 180 100" fill="none" stroke="rgba(255,255,255,.09)" stroke-width="12" stroke-linecap="round"/>
        <path id="steerArc" d="M100 20 A80 80 0 0 1 100 20" fill="none" stroke="url(#lg)" stroke-width="12" stroke-linecap="round"/>
        <g stroke="rgba(255,255,255,.25)" stroke-width="1.5"><line x1="100" y1="12" x2="100" y2="22"/><line x1="43" y1="43" x2="50" y2="50"/><line x1="157" y1="43" x2="150" y2="50"/></g>
        <g class="needle" id="steerNeedle"><line x1="100" y1="100" x2="100" y2="30" stroke="#e8f0ff" stroke-width="3" stroke-linecap="round"/></g>
        <circle cx="100" cy="100" r="7" fill="#0a1424" stroke="#e8f0ff" stroke-width="2"/>
      </svg>
      <div class="center"><span class="big" id="steerVal">0</span><span class="unit">command</span></div>
    </section>

    <section class="card c-heading">
      <h2>Heading <small>gyro, since power-up</small></h2>
      <svg class="compass" viewBox="0 0 180 180" aria-label="heading dial">
        <circle cx="90" cy="90" r="78" fill="rgba(255,255,255,.03)" stroke="rgba(130,175,255,.28)" stroke-width="2"/>
        <g id="ticks" stroke="rgba(255,255,255,.3)" stroke-width="1.5"></g>
        <text x="90" y="26" text-anchor="middle" fill="#8fa5c4" font-size="11">START</text>
        <g class="needle" id="yawNeedle"><path d="M90 24 L98 92 L90 86 L82 92 Z" fill="#4cc9f0"/><path d="M90 156 L96 92 L84 92 Z" fill="rgba(255,255,255,.22)"/></g>
        <circle cx="90" cy="90" r="5" fill="#0a1424" stroke="#e8f0ff" stroke-width="2"/>
      </svg>
      <div class="center" style="margin-top:6px"><span class="big" id="yawVal">0°</span><span class="unit" id="yawDir"></span></div>
      <div class="kv"><span>Last turn (gyro measured)</span><b id="lastTurn">—</b></div>
    </section>

    <section class="card c-corner">
      <h2>Corner ahead <small>camera estimate</small></h2>
      <div class="big" id="cornerVal">—</div>
      <div class="kv"><span>Direction</span><b id="cornerDir">—</b></div>
      <div class="kv"><span>Arrival estimate</span><b id="cornerEta">—</b></div>
      <div class="kv"><span>Approach phase</span><b id="cornerPhase">—</b></div>
      <div class="prox"><i id="cornerProx"></i></div>
      <div class="note">Angle is what the camera reads from the tape in the picture, not a ground measurement.</div>
    </section>

    <section class="card c-wheels">
      <h2>Wheel power <small>PWM</small></h2>
      <div class="stack">
        <div class="row"><span class="lbl">L</span><div class="bar"><i id="barL"></i></div><span class="val" id="valL">0</span></div>
        <div class="row"><span class="lbl">R</span><div class="bar"><i id="barR"></i></div><span class="val" id="valR">0</span></div>
      </div>
      <div class="note" id="paceNote">Pace limit —</div>
    </section>

    <section class="card c-ir">
      <h2>IR sensor bar <small>tape under car</small></h2>
      <div class="irbar" id="irbar"><span></span><span></span><span></span><span></span><span></span></div>
      <div class="kv"><span>Position</span><b id="irPos">—</b></div>
    </section>

    <section class="card c-lock">
      <h2>Camera lock <small>tape confidence</small></h2>
      <svg class="ring" viewBox="0 0 120 120">
        <circle cx="60" cy="60" r="50" fill="none" stroke="rgba(255,255,255,.08)" stroke-width="10"/>
        <circle class="arc" id="lockArc" cx="60" cy="60" r="50" fill="none" stroke="#4cc9f0" stroke-width="10" stroke-linecap="round"
                stroke-dasharray="314.16" stroke-dashoffset="314.16" transform="rotate(-90 60 60)"/>
        <text x="60" y="67" text-anchor="middle" fill="#e8f0ff" font-size="26" font-family="ui-monospace,Consolas,monospace" font-weight="600" id="lockTxt">0</text>
      </svg>
      <div class="kv"><span>Tape offset</span><b id="nearVal">—</b></div>
    </section>

    <section class="card c-sonar">
      <h2>Sonar <small>object ahead</small></h2>
      <div class="big"><span id="distVal">—</span><span class="unit">cm</span></div>
      <div class="bar" style="margin-top:10px"><i id="distBar"></i></div>
      <div class="note" id="distNote">Range 0–150 cm shown</div>
    </section>

    <section class="card c-events">
      <h2>Event log</h2>
      <ul class="events" id="events"><li class="empty">Nothing yet.</li></ul>
    </section>
    <section class="card c-trace">
      <h2>Last 20 seconds
        <span class="legend"><span><i style="background:#4cc9f0"></i>steering</span><span><i style="background:#ffcb5c"></i>heading</span><span><i style="background:#5ef0a8"></i>camera lock</span></span>
      </h2>
      <canvas class="chart" id="chart" width="900" height="170"></canvas>
    </section>

  </div>

  <footer>
    <details><summary>How to read this page</summary><ul>
      <li>Camera: yellow line is the camera's look-ahead offset (not an IR reading); magenta is the detected tape centreline. Overlays appear only when the tape lock is good enough to steer.</li>
      <li>Vision: the tape the car is steering on, white on black. Blank means no usable tape in view.</li>
      <li>IR bar lights show which sensors currently see tape, left to right.</li>
      <li>Corner ahead: the camera's reading of an upcoming bend. The car turns on its gyro; the Last turn value is the rotation the gyro actually measured.</li>
      <li>Wheel power is instantaneous PWM; both banks pause to slow down and the inside bank pauses more to steer.</li>
      <li><a href="/raw.jpg">/raw.jpg</a> is the unannotated frame. If red looks blue, restart the program with --swap-rb.</li>
    </ul></details>
  </footer>
</main>

<img id="feed" src="/video_feed" alt="" style="display:none">
<script>
(function () {
  "use strict";
  var $ = function (id) { return document.getElementById(id); };
  var feed = $("feed"), camC = $("cam"), maskC = $("mask");
  var camX = camC.getContext("2d"), maskX = maskC.getContext("2d");
  var ST = { turns: 0, armedAt: null, hist: [], events: [], state: null, pivot: false, pivotYaw: 0, cornerLogged: false, lastGood: 0, ok: false, t0: performance.now() };
  var WHY = { 1: "Something is in the way ahead.", 2: "Heading changed more than allowed.", 3: "Tape lost by both camera and IR bar.",
              4: "Something was in the way during the turn.", 5: "Turn limit reached before the next leg was found.", 6: "Turn took too long." };
  var FAULT = { 1: "Gyro starting up", 2: "Gyro bus error", 3: "Gyro data gap", 4: "Gyro saturated", 5: "Gyro mount check", 6: "Turned the wrong way - reset" };

  /* heading dial ticks */
  (function () {
    var g = $("ticks"), s = "";
    for (var a = 0; a < 360; a += 15) {
      var r1 = a % 90 === 0 ? 64 : 70, rad = (a - 90) * Math.PI / 180;
      s += '<line x1="' + (90 + r1 * Math.cos(rad)).toFixed(1) + '" y1="' + (90 + r1 * Math.sin(rad)).toFixed(1) +
           '" x2="' + (90 + 76 * Math.cos(rad)).toFixed(1) + '" y2="' + (90 + 76 * Math.sin(rad)).toFixed(1) + '"/>';
    }
    g.innerHTML = s;
  })();

  /* camera stream -> two canvases. The stream is 640x240: left half camera, right half mask panel. */
  function drawFeed() {
    if (feed.naturalWidth >= 640) {
      camX.drawImage(feed, 0, 28, 320, 194, 0, 0, 320, 194);
      maskX.drawImage(feed, 320, 84, 320, 138, 0, 0, 320, 138);
      $("nofeed").style.display = "none";
      ST.lastFrame = performance.now();
    } else if (performance.now() - ST.t0 > 3000) { $("nofeed").style.display = "flex"; }
    requestAnimationFrame(drawFeed);
  }
  requestAnimationFrame(drawFeed);
  feed.addEventListener("error", function () { setTimeout(function () { feed.src = "/video_feed?t=" + Date.now(); }, 1500); });

  function fmtTime() { var s = Math.floor((performance.now() - ST.t0) / 1000); return (s / 60 | 0) + ":" + ("0" + s % 60).slice(-2); }
  function logEvent(txt, cls) {
    ST.events.unshift({ t: fmtTime(), txt: txt, cls: cls || "" });
    if (ST.events.length > 40) ST.events.pop();
    $("events").innerHTML = ST.events.slice(0, 12).map(function (e) { return '<li><time>' + e.t + '</time><span class="' + e.cls + '">' + e.txt + '</span></li>'; }).join("");
  }
  function sideWord(v) { return v > 0 ? "right" : "left"; }

  function derive(d) {
    if (!d.online) return { key: "wait", text: "WAITING", sub: "No fresh readings from the Arduino.", cls: "dimc" };
    var u = d.uno, c = d.cam;
    if (u.err || u.src === "FAULT") return { key: "fault", text: "FAULT", sub: FAULT[u.err] || "Sensor or motor fault", cls: "bad" };
    var src = u.src;
    if (src === "TURN" || u.ts >= 3) return { key: "turn", text: "TURNING", sub: "Rotating on the gyro until the camera or IR bar finds the next leg.", cls: "warn" };
    if (src === "REC_STOP") return { key: "stop", text: "STOPPED", sub: WHY[u.w] || "A recovery limit was reached.", cls: "bad" };
    if (src === "LOST") return { key: "lost", text: "TRACK LOST", sub: "No usable tape from camera or IR bar.", cls: "bad" };
    if (src === "STOP" || src === "OFF") return { key: "stop", text: "STOPPED", sub: src === "OFF" ? "Camera heartbeat is off." : (u.d != null && u.d < 45 ? "Something is in the way ahead (" + u.d + " cm)." : "Stop command received."), cls: "bad" };
    if (src === "REC_CAM" || src === "REC_IR" || src === "TURN_MEM") return { key: "rec", text: "RECOVERING", sub: "Holding course on " + (src === "REC_IR" ? "IR bar" : "camera") + " + gyro.", cls: "warn" };
    if (src === "CHECK" || src === "ALIGN" || src === "REJOIN") return { key: "check", text: "CONFIRMING TRACK", sub: "Waiting for camera and IR bar to agree.", cls: "acc" };
    var how = src === "BOTH" || src === "CENTER" ? "Camera + IR bar" : src === "CAM_TRACK" || src === "CAM" ? "Camera" : src === "IR" ? "IR bar" : "Cruise control";
    var corner = c.quality >= 35 && (c.phase === "APPROACH" || c.phase === "TURN") && Math.abs(c.turn_deg) >= 15;
    if (corner) return { key: "follow-corner", text: "CORNER AHEAD", sub: how + " tracking. Camera reads a bend to the " + sideWord(c.turn_deg) + ".", cls: "acc" };
    return { key: "follow", text: "FOLLOWING TRACK", sub: how + " tracking.", cls: "ok" };
  }

  function setBadge(id, on, txt) { var e = $(id); e.className = "badge" + (on ? " on" : ""); if (txt) e.textContent = txt; }
  function setBar(id, frac) { $(id).style.width = Math.max(0, Math.min(1, frac)) * 100 + "%"; }

  function arcPath(val) {            /* steering arc from 12 o'clock to val */
    var a = Math.max(-100, Math.min(100, val)) * 0.9 * Math.PI / 180, r = 80;
    var x = 100 + r * Math.sin(a), y = 100 - r * Math.cos(a);
    if (Math.abs(val) < 1) return "M100 20 A80 80 0 0 1 100 20";
    return "M100 20 A80 80 0 0 " + (val > 0 ? 1 : 0) + " " + x.toFixed(1) + " " + y.toFixed(1);
  }

  function render(d) {
    var on = d.online;
    $("chipLive").className = "chip" + (on ? " live" : "");
    $("liveTxt").textContent = on ? "LIVE" : "no sensor data";
    $("chipFps").textContent = d.fps ? d.fps.toFixed(0) + " fps" : "— fps";
    $("chipVer").textContent = (d.version || "Pi ?") + " · tape " + d.tape_cm + " cm · pace " + d.cruise;
    var u = d.uno, c = d.cam;
    var s = derive(d);
    $("stateMain").textContent = s.text; $("stateMain").className = "state-main " + s.cls; $("stateSub").textContent = s.sub;
    setBadge("bCam", c.quality >= 35, "CAMERA " + (c.quality >= 35 ? c.quality + "%" : "—"));
    setBadge("bIr", on && u.mask && u.mask.indexOf("1") >= 0, "IR BAR");
    setBadge("bImu", on && u.imu === 1, "GYRO");
    setBadge("bArm", on && u.arm === 1, u.arm === 1 ? "TRACK ARMED" : "NOT ARMED");

    /* steering */
    var cmd = on ? u.cmd : 0;
    $("steerVal").textContent = (cmd > 0 ? "+" : "") + cmd;
    $("steerNeedle").style.transform = "rotate(" + (cmd * 0.9) + "deg)";
    $("steerArc").setAttribute("d", arcPath(cmd));

    /* heading and last turn (gyro) */
    var yaw = on ? u.yaw : 0;
    $("yawNeedle").style.transform = "rotate(" + yaw + "deg)";
    $("yawVal").textContent = Math.abs(yaw) + "°";
    $("yawDir").textContent = yaw === 0 ? "" : sideWord(yaw) + " of start";
    var turning = on && (u.pivot === 1 || u.src === "TURN" || u.ts >= 3);
    if (turning && !ST.pivot) { ST.pivotYaw = yaw; logEvent("Turning on the gyro…", "warn"); }
    if (!turning && ST.pivot) {
      var dy = yaw - ST.pivotYaw;
      if (Math.abs(dy) >= 5) { $("lastTurn").textContent = Math.abs(dy) + "° " + sideWord(dy); $("stLast").textContent = Math.abs(dy) + "° " + sideWord(dy); ST.turns++; $("stTurns").textContent = ST.turns; logEvent("Turn finished: gyro measured " + Math.abs(dy) + "° " + sideWord(dy) + ".", "ok"); }
    }
    ST.pivot = turning;
    if (on && u.arm === 1) { if (ST.armedAt === null) ST.armedAt = performance.now(); } else { ST.armedAt = null; }
    var runS = ST.armedAt === null ? 0 : Math.floor((performance.now() - ST.armedAt) / 1000);
    $("stRun").textContent = (runS / 60 | 0) + ":" + ("0" + runS % 60).slice(-2);

    /* corner (camera) */
    var cornerOn = c.quality >= 35 && c.phase !== "STRAIGHT" && Math.abs(c.turn_deg) >= 15;
    $("cornerVal").textContent = cornerOn ? Math.abs(c.turn_deg) + "°" : "—";
    $("cornerVal").className = "big " + (cornerOn ? "acc" : "dimc");
    $("cornerDir").textContent = cornerOn ? sideWord(c.turn_deg) : "—";
    $("cornerEta").textContent = cornerOn && c.eta != null ? c.eta.toFixed(1) + " s" : "—";
    $("cornerPhase").textContent = cornerOn ? ({ APPROACH: "preparing", TURN: "entry window", "TURN MEMORY": "from memory" }[c.phase] || c.phase.toLowerCase()) : "straight";
    $("cornerProx").style.width = (cornerOn ? c.bend : 0) + "%";
    if (cornerOn && !ST.cornerLogged && c.phase !== "TURN MEMORY") { ST.cornerLogged = true; logEvent("Camera sees a bend to the " + sideWord(c.turn_deg) + ", about " + Math.abs(c.turn_deg) + "°.", "acc"); }
    if (!cornerOn && c.quality >= 35 && c.phase === "STRAIGHT") ST.cornerLogged = false;

    /* wheels */
    var L = on ? u.l : 0, R = on ? u.r : 0;
    setBar("barL", L / 255); setBar("barR", R / 255); $("valL").textContent = L; $("valR").textContent = R;
    $("paceNote").textContent = "Pace limit " + (c.pace || "—") + "%";

    /* IR bar */
    var spans = $("irbar").children, mask = on && u.mask ? u.mask : "00000";
    for (var i = 0; i < 5; i++) spans[i].className = mask.charAt(i) === "1" ? "on" : "";
    $("irPos").textContent = on && mask.indexOf("1") >= 0 ? (u.ir > 0 ? "+" : "") + u.ir : "no tape";

    /* camera lock */
    var q = c.quality || 0, circ = 314.16;
    $("lockArc").setAttribute("stroke-dashoffset", (circ * (1 - q / 100)).toFixed(1));
    $("lockArc").setAttribute("stroke", q >= 70 ? "#5ef0a8" : q >= 35 ? "#4cc9f0" : "#ff6b7d");
    $("lockTxt").textContent = q;
    $("nearVal").textContent = c.near == null ? "—" : (c.near > 0 ? "+" : "") + c.near.toFixed(0);
    $("camRec").textContent = "Q " + q + (c.near == null ? "" : " · off " + (c.near > 0 ? "+" : "") + c.near.toFixed(0));

    /* sonar */
    var dist = on && u.d != null ? u.d : null;
    $("distVal").textContent = dist == null ? "—" : (dist >= 999 ? "150+" : dist);
    setBar("distBar", dist == null ? 0 : Math.min(dist, 150) / 150);
    $("distBar").style.background = dist != null && dist < 40 ? "linear-gradient(90deg,#ff6b7d,#ffcb5c)" : "";
    $("distNote").textContent = dist != null && dist < 45 ? "Object in range - cruise control active" : "Clear ahead";

    /* state change log */
    if (ST.state !== s.key && s.key !== "wait") {
      if (ST.state !== null && s.key !== "turn" && s.key !== "follow-corner") logEvent(s.text.charAt(0) + s.text.slice(1).toLowerCase() + (s.key === "stop" || s.key === "lost" ? " - " + s.sub : ""), s.cls);
      ST.state = s.key;
    }
    if (s.key === "wait" && ST.state === null) ST.state = "wait";

    /* history for the chart */
    var now = performance.now();
    ST.hist.push({ t: now, cmd: cmd, yaw: yaw, q: q });
    while (ST.hist.length && now - ST.hist[0].t > 20000) ST.hist.shift();
  }

  function drawChart() {
    var cv = $("chart"), r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    var w = Math.max(300, Math.round(r.width * dpr)), h = Math.round(170 * dpr);
    if (cv.width !== w) cv.width = w; if (cv.height !== h) cv.height = h;
    var x = cv.getContext("2d"); x.clearRect(0, 0, w, h);
    var pad = 6 * dpr;
    x.strokeStyle = "rgba(130,175,255,.14)"; x.lineWidth = 1;
    for (var i = 0; i <= 4; i++) { var yy = pad + (h - 2 * pad) * i / 4; x.beginPath(); x.moveTo(0, yy); x.lineTo(w, yy); x.stroke(); }
    x.strokeStyle = "rgba(255,255,255,.28)"; x.beginPath(); x.moveTo(0, h / 2); x.lineTo(w, h / 2); x.stroke();
    var now = performance.now(), H = ST.hist;
    function line(key, scale, color, offset) {
      x.beginPath(); x.strokeStyle = color; x.lineWidth = 2 * dpr; x.lineJoin = "round";
      var started = false;
      for (var k = 0; k < H.length; k++) {
        var px = w - (now - H[k].t) / 20000 * w;
        var v = key === "yaw" ? Math.max(-180, Math.min(180, H[k].yaw)) / 180 : key === "q" ? (H[k].q / 100) * 2 - 1 : H[k].cmd / 100;
        var py = h / 2 - v * (h / 2 - pad);
        if (!started) { x.moveTo(px, py); started = true; } else x.lineTo(px, py);
      }
      x.stroke();
    }
    line("q", 1, "rgba(94,240,168,.55)"); line("yaw", 1, "#ffcb5c"); line("cmd", 1, "#4cc9f0");
    requestAnimationFrame(drawChart);
  }
  requestAnimationFrame(drawChart);

  var failures = 0;
  function poll() {
    var ctl = new AbortController(), to = setTimeout(function () { ctl.abort(); }, 1500);
    fetch("/status.json", { cache: "no-store", signal: ctl.signal }).then(function (r) { return r.json(); }).then(function (d) {
      clearTimeout(to); failures = 0; ST.ok = true; render(d);
    }).catch(function () {
      clearTimeout(to); failures++;
      if (failures === 3) { $("chipLive").className = "chip"; $("liveTxt").textContent = "connection lost"; logEvent("Lost connection to the car.", "bad"); }
    }).then(function () { setTimeout(poll, 120); });
  }
  poll();
})();
</script>
</body>
</html>
'''


@app.route("/")
def index():
    return DASHBOARD_HTML


def build_status():
    """Everything the dashboard shows. Only measured or received values: nothing planned or assumed."""
    c = controller
    if c is None:
        return {'online': False}
    snap = c.sensor_telemetry.snapshot()
    v = snap['values'] if snap['fresh'] else {}
    ctl = c.steering_controller
    plan = ctl.arrival
    q = int(getattr(ctl, 'quality_shown', ctl.camera_quality))
    live = q >= 35
    interval = c.frame_timing.interval
    uno = {}
    if v:
        uno = {
            'mask': v['mask'], 'ir': v['IR'], 'cam': v['CAM'], 'cmd': v['CMD'], 'yaw': v['YAW'],
            'rate': v['RATE'], 'imu': v['IMU'], 'l': v['L'], 'r': v['R'], 'src': v['SRC'],
            'pivot': v.get('PIVOT', 0), 'ts': v.get('TS', 0), 'd': v.get('D'), 'w': v.get('W', 0),
            'arm': v.get('ARM', 0), 'err': v.get('ERR', 0), 'rec': v.get('REC', 0),
        }
    return {
        'online': bool(v),
        'uno': uno,
        'cam': {
            'quality': q,
            'near': round(float(ctl.near_error), 1) if live else None,
            'bend': round(float(ctl.turn_proximity) * 100) if live else 0,
            'phase': plan.phase if live else 'STRAIGHT',
            'eta': None if (not live or plan.eta is None) else round(float(plan.eta), 2),
            'turn_deg': round(float(ctl.turn_deg), 1) if live else 0.0,
            'pace': int(ctl.pace),
            'reject': ctl.reject_why if not live else '',
        },
        'fps': round(1.0 / interval, 1) if interval and interval > 0 else 0,
        'version': 'Pi V7.6c', 'tape_cm': TAPE_WIDTH_CM, 'cruise': CRUISE_PACE,
        'frame': c.frame_number,
        'error': c.worker_error,
    }


@app.route('/status.json')
def status_json():
    return Response(json.dumps(build_status()), mimetype='application/json',
                    headers={'Cache-Control': 'no-store'})


@app.route('/raw.jpg')
def raw_snapshot():
    frame = None if controller is None else controller.latest_raw
    if frame is None:
        return Response('No frame yet.', status=503, mimetype='text/plain')
    ok, jpeg = cv2.imencode('.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route('/diagnostics.zip')
def download_diagnostics():
    if controller is None or controller.diagnostics is None:
        return Response('No run recorded yet.', status=503, mimetype='text/plain')
    return Response(controller.diagnostics.bundle(), mimetype='application/zip',
                    headers={'Content-Disposition': 'attachment; filename="mavp-last-run.zip"'})


def colour_test():
    """Save one frame and print channel means. Hold a RED object in front of the lens."""
    camera = Picamera2()
    camera.configure(camera.create_still_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}))
    camera.start()
    time.sleep(1.5)
    frame = camera.capture_array('main')
    camera.stop()
    rgb = camera_array_to_rgb(frame)
    cv2.imwrite('colour_test.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    centre = rgb[FRAME_HEIGHT // 3: 2 * FRAME_HEIGHT // 3, FRAME_WIDTH // 3: 2 * FRAME_WIDTH // 3].reshape(-1, 3).mean(axis=0)
    print(f'Centre mean R,G,B = {centre[0]:.0f}, {centre[1]:.0f}, {centre[2]:.0f}  (saved colour_test.png)')
    print('Red object in the centre should show R clearly highest. If B is highest, rerun with --swap-rb.')


def main():
    global controller, CAMERA_ARRAY_IS_BGR, BLACK_GATES, TAPE_WIDTH_CM, CRUISE_PACE
    parser = argparse.ArgumentParser(description='MAVP Stage 8 FUSION V7 track follower')
    parser.add_argument('--swap-rb', action='store_true', help='camera array is already RGB: undo the BGR assumption')
    parser.add_argument('--loose-tape', action='store_true', help='turn the V7.4 black-only tape gates off (fallback if the lighting changed)')
    parser.add_argument('--tape-cm', type=float, default=None, help='tape width in cm (default 4.7)')
    parser.add_argument('--cruise', type=int, default=None, help='straight-line pace in percent (default 70, allowed 40-90)')
    parser.add_argument('--colour-test', action='store_true', help='save one frame, print R,G,B means, then exit')
    arguments = parser.parse_args()
    if arguments.swap_rb:
        CAMERA_ARRAY_IS_BGR = False
    if arguments.cruise:
        CRUISE_PACE = max(40, min(90, int(arguments.cruise)))
    if arguments.tape_cm:
        TAPE_WIDTH_CM = float(arguments.tape_cm)
    if arguments.loose_tape:
        BLACK_GATES = False
        print('V7.4 black-only tape gates are OFF (--loose-tape).')
    if arguments.colour_test:
        colour_test()
        return

    controller = BenchController()

    try:
        controller.start()
        print("Stage 8 camera worker started; waiting for fresh frames before guidance.")
        print(f"Open http://<PI-IP>:{WEB_PORT} in the laptop browser.")
        print("Press Ctrl+C here to stop the motors and finish.")

        app.run(
            host="0.0.0.0",
            port=WEB_PORT,
            threaded=True,
            use_reloader=False,
        )

    finally:
        controller.stop()


if __name__ == "__main__":
    main()
