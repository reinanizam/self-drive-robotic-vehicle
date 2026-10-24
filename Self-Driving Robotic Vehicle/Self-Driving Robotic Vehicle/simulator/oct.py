"""The test track: an octagon. Long sides 1.20 m (x2), diagonals ~0.44 m (x4), short sides ~0.30 m (x2), 45 deg corners."""
import math
LONG, DIAG, SHORT = 1.20, 0.441, 0.305
SIDES = [LONG, DIAG, SHORT, DIAG, LONG, DIAG, SHORT, DIAG]
def octagon(sgn=+1, start_back=0.0, corners=7, tail=0.25, sides=SIDES, scale=1.0):
    """Polyline of `corners` corners, starting at the beginning of a long side heading north.
    sgn=+1: left (counter-clockwise) loop, -1: right. The last piece stops `tail` m into the segment after the last corner."""
    x, y, h = 0.0, -start_back, 0.0           # heading 0 = north, positive h turns LEFT
    pts = [(x, y)]
    for k in range(corners + 1):
        L = sides[k % 8] * scale
        if k == corners: L = min(L, tail)
        x += -sgn * math.sin(h) * L * 1.0 if False else 0.0
        hx, hy = -sgn * math.sin(h), math.cos(h)
        x += hx * L; y += hy * L
        pts.append((round(x, 4), round(y, 4)))
        h += math.radians(45)
    return pts
if __name__ == '__main__':
    p = octagon(corners=8, tail=99)
    print(p); import numpy as np; print('closing error', np.hypot(*p[-1]))
