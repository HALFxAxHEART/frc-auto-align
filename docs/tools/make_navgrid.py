"""Builds src/main/deploy/pathplanner/navgrid.json for auto-align (see AUTO-ALIGN-GUIDE.md "Obstacles").

Starts from PathPlanner's default 2026 grid (docs/tools/navgrid-pathplanner-default.json, 30 cm squares), makes it
finer (10 cm squares), then redraws everything around the hubs from the real field measurements (team 6328's
FieldConstants) so the robot fits under the trenches:
  - hub, both bumps and the trench frames are blocked (bumps too: our auto-align should not drive over them)
  - each trench opening is left open; robot center must stay half a robot + a safety margin from the frame and wall
Run from the repo root:  python docs/tools/make_navgrid.py
"""
import json, math, os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "docs/tools/navgrid-pathplanner-default.json")
OUT = os.path.join(ROOT, "src/main/deploy/pathplanner/navgrid.json")

NODE = 0.10            # grid square size, meters
HALF_ROBOT = 0.45      # half the robot's width with bumpers, meters
MARGIN = 0.05          # extra safety distance, meters
L, W = 16.541, 8.069   # field size, meters

# Blue-side elements (x0, x1, y0, y1), meters. Red side = the same rotated 180 deg about the field center.
HUB_X0, HUB_X1 = 4.022, 4.022 + 1.194
BUMP_DEPTH = 1.128
BLUE = [
    ("hub", HUB_X0, HUB_X1, 3.438, 4.632),
    ("left bump", HUB_X0, HUB_X0 + BUMP_DEPTH, 4.632, 6.486),
    ("right bump", HUB_X0, HUB_X0 + BUMP_DEPTH, 1.584, 3.438),
    ("left trench frame", HUB_X0, HUB_X1, 6.486, 6.791),
    ("right trench frame", HUB_X0, HUB_X1, 1.279, 1.584),
]
# The area redrawn from these measurements (everything else is copied from PathPlanner's default grid)
REDRAW_X = (HUB_X0 - 1.5, HUB_X1 + 1.5)

def rotated(e):
    name, x0, x1, y0, y1 = e
    return ("red " + name, L - x1, L - x0, W - y1, W - y0)

def main():
    src = json.load(open(SRC))
    n0, g0 = src["nodeSizeMeters"], src["grid"]
    rows, cols = math.ceil(W / NODE), math.ceil(L / NODE)
    grid = [[g0[min(int((r + 0.5) * NODE / n0), len(g0) - 1)][min(int((c + 0.5) * NODE / n0), len(g0[0]) - 1)]
             for c in range(cols)] for r in range(rows)]
    pad = HALF_ROBOT + MARGIN
    elements = BLUE + [rotated(e) for e in BLUE]
    regions = [REDRAW_X, (L - REDRAW_X[1], L - REDRAW_X[0])]
    for r in range(rows):
        for c in range(cols):
            cx0, cx1, cy0, cy1 = c * NODE, (c + 1) * NODE, r * NODE, (r + 1) * NODE
            if not any(x0 <= cx0 and cx1 <= x1 for x0, x1 in regions):
                continue
            # a square is blocked if ANY point in it is closer than pad to an element or a side wall
            blocked = cy0 < pad or cy1 > W - pad
            for _, x0, x1, y0, y1 in elements:
                if cx1 > x0 - pad and cx0 < x1 + pad and cy1 > y0 - pad and cy0 < y1 + pad:
                    blocked = True
            grid[r][c] = blocked
    json.dump({"field_size": src["field_size"], "nodeSizeMeters": NODE, "grid": grid}, open(OUT, "w"))
    print(f"wrote {OUT}: {rows} x {cols} squares of {NODE} m")

if __name__ == "__main__":
    main()
