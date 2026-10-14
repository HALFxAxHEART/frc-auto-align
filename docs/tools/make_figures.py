"""Draws the pictures in AUTO-ALIGN-GUIDE.md (dark + light versions) into docs/images.

Run from the repo root:  python docs/tools/make_figures.py   (needs: pip install matplotlib)
sim-trace-blue.csv is the robot's position every 20 ms from simulation runs of the real AutoAlign code.
"""
import csv, json, math, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, Polygon

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(ROOT, "docs", "images")
NAVGRID = json.load(open(os.path.join(ROOT, "src/main/deploy/pathplanner/navgrid.json")))

# Tag poses from WPILib's 2026-rebuilt-welded.json (x, y, facing deg)
TAGS = {18: (4.626, 3.431, 270), 19: (5.229, 3.679, 0), 20: (5.229, 4.035, 0), 21: (4.626, 4.638, 90),
        24: (4.27, 4.638, 90), 25: (4.022, 4.39, 180), 26: (4.022, 4.035, 180), 27: (4.27, 3.431, 270),
        29: (0.008, 0.666, 0), 30: (0.008, 1.098, 0), 31: (0.008, 3.746, 0), 32: (0.008, 4.178, 0)}
HALF, GAP, APPROACH = 0.45, 0.10, 0.75

THEMES = {
    "dark": dict(bg="#0f1216", field="#161b22", grid="#2a1f24", gridedge="#3b2a31", text="#e6edf3", sub="#9aa4af",
                 hub="#f0883e", outpost="#2ec4b6", path="#58a6ff", final="#3fb950", goal="#3fb950", appr="#d2a8ff",
                 tag="#f2cc60", robot="#c9d1d9", wall="#8b949e"),
    "light": dict(bg="#ffffff", field="#f6f8fa", grid="#f3d9dc", gridedge="#e8b9bf", text="#1f2328", sub="#59636e",
                  hub="#d1600f", outpost="#0f8a80", path="#0969da", final="#1a7f37", goal="#1a7f37", appr="#8250df",
                  tag="#b08800", robot="#424a53", wall="#57606a"),
}

def standing(tag_id, side=0.0, extra=0.0):
    x, y, deg = TAGS[tag_id]
    a = math.radians(deg)
    out = HALF + GAP + extra
    # tag frame: +x out of face, +y to the tag's left. side offset is viewer-left positive -> tag -y
    px = x + out * math.cos(a) - (-side) * math.sin(a)
    py = y + out * math.sin(a) + (-side) * math.cos(a)
    return px, py, (deg + 180) % 360

def robot(ax, x, y, deg, color, t, alpha=1.0, ls="-", lw=1.6, label=None):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    pts = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in [(-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF)]]
    ax.add_patch(Polygon(pts, closed=True, fill=False, ec=color, lw=lw, ls=ls, alpha=alpha, zorder=5))
    ax.add_patch(FancyArrowPatch((x, y), (x + c * 0.38, y + s * 0.38), arrowstyle="-|>", mutation_scale=11,
                                 color=color, lw=lw, alpha=alpha, zorder=6))
    if label:
        ax.text(x, y - HALF - 0.12, label, color=color, ha="center", va="top", fontsize=8, zorder=7)

def field_base(ax, t, xlim, ylim, grid=True):
    ax.set_facecolor(t["field"])
    if grid:
        n = NAVGRID["nodeSizeMeters"]
        for r, row in enumerate(NAVGRID["grid"]):
            for c, blocked in enumerate(row):
                if blocked:
                    ax.add_patch(Rectangle((c * n, r * n), n, n, fc=t["grid"], ec=t["gridedge"], lw=0.4, zorder=1))
    # hub (blue) and outpost
    ax.add_patch(Rectangle((4.022, 3.431), 1.207, 1.207, fc=t["hub"], alpha=0.85, zorder=3))
    ax.text(4.625, 4.035, "HUB", color=t["bg"], ha="center", va="center", fontsize=9, weight="bold", zorder=4)
    ax.add_patch(Rectangle((-0.05, 0.55), 0.12, 0.66, fc=t["outpost"], zorder=3))
    ax.plot([0, 0], [0, 8.07], color=t["wall"], lw=3, zorder=3)
    for tid in (20, 26, 29, 30):
        x, y, deg = TAGS[tid]
        ax.plot(x, y, marker="s", ms=5, color=t["tag"], zorder=8)
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal")
    ax.tick_params(colors=t["sub"], labelsize=8)
    for sp in ax.spines.values():
        sp.set_color(t["sub"])
    ax.set_xlabel("field X (m)  →  toward the red wall", color=t["sub"], fontsize=8)
    ax.set_ylabel("field Y (m)", color=t["sub"], fontsize=8)

def load_trace():
    runs = {}
    with open(os.path.join(os.path.dirname(__file__), "sim-trace-blue.csv")) as f:
        for row in csv.DictReader(f):
            runs.setdefault(int(row["run"]), []).append(row)
    return runs

def fig_field(t, name):
    fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=150, facecolor=t["bg"])
    field_base(ax, t, (-0.3, 9.0), (-0.1, 8.17))
    runs = load_trace()
    labels = {1: "start A → hub", 2: "start B → outpost", 3: "start C → hub", 4: "start D → outpost"}
    for run, letter in zip((1, 2, 3, 4), "ABCD"):
        rows = runs[run]
        xs = [float(r["x"]) for r in rows]; ys = [float(r["y"]) for r in rows]
        ax.plot(xs, ys, color=t["path"], lw=2, zorder=6, alpha=0.95)
        r0 = rows[0]
        robot(ax, float(r0["x"]), float(r0["y"]), float(r0["deg"]), t["robot"], t, alpha=0.8, label=letter)
    for tid, side, col in ((26, 0, t["hub"]), (20, 0, t["hub"]), (29, -0.216, t["outpost"])):
        gx, gy, gd = standing(tid, side)
        robot(ax, gx, gy, gd, t["goal"], t, lw=2)
        ax_, ay_, ad_ = standing(tid, side, APPROACH)
        robot(ax, ax_, ay_, ad_, t["appr"], t, ls="--", lw=1.2, alpha=0.9)
    ax.text(4.625, 2.45, "bumps + hub = blocked", color=t["sub"], ha="center", fontsize=8, zorder=9)
    ax.text(0.15, 0.25, "OUTPOST", color=t["outpost"], fontsize=8, weight="bold")
    ax.set_title("Simulation of the real code: 4 starts, blue alliance (half the field shown)", color=t["text"], fontsize=11)
    from matplotlib.lines import Line2D
    leg = [Line2D([], [], color=t["path"], lw=2, label="path the robot drove"),
           Rectangle((0, 0), 1, 1, fc=t["grid"], ec=t["gridedge"], label="navgrid: blocked (obstacle)"),
           Line2D([], [], color=t["appr"], ls="--", label="approach pose"),
           Line2D([], [], color=t["goal"], lw=2, label="final pose"),
           Line2D([], [], color=t["tag"], marker="s", ls="", label="AprilTag")]
    lg = ax.legend(handles=leg, loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7.5, facecolor=t["bg"], edgecolor=t["sub"], labelcolor=t["text"])
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f"field-paths-{name}.png"), facecolor=t["bg"]); plt.close(fig)

def fig_two_stage(t, name):
    fig, ax = plt.subplots(figsize=(8, 5), dpi=150, facecolor=t["bg"])
    field_base(ax, t, (0.2, 5.6), (1.2, 5.3))
    rows = load_trace()[3]
    st1 = [r for r in rows if r["state"].startswith("Pathfinding")]
    st2 = [r for r in rows if not r["state"].startswith("Pathfinding")]
    ax.plot([float(r["x"]) for r in st1], [float(r["y"]) for r in st1], color=t["path"], lw=2.5, zorder=6, label="stage 1: PathPlanner pathfinding")
    ax.plot([float(r["x"]) for r in st2], [float(r["y"]) for r in st2], color=t["final"], lw=2.5, zorder=6, label="stage 2: DriveToPose (straight in)")
    r0 = rows[0]
    robot(ax, float(r0["x"]), float(r0["y"]), float(r0["deg"]), t["robot"], t, alpha=0.8, label="start")
    gx, gy, gd = standing(26); robot(ax, gx, gy, gd, t["goal"], t, lw=2)
    axp, ayp, adp = standing(26, 0, APPROACH); robot(ax, axp, ayp, adp, t["appr"], t, ls="--", lw=1.4)
    ax.text(axp - 0.1, ayp + 0.55, "APPROACH\npose", color=t["appr"], ha="center", fontsize=8.5, weight="bold")
    ax.text(gx + 0.15, gy + 0.55, "FINAL\npose", color=t["goal"], ha="center", fontsize=8.5, weight="bold")
    x26 = TAGS[26][0]
    y0 = 2.95
    for xa, xb, lbl, yy in ((x26 - HALF - GAP, x26, "0.45 + 0.10 m\n(half robot + gap)", y0 - 0.45), (axp, gx, "0.75 m\n(approach distance)", y0)):
        ax.annotate("", xy=(xa, yy), xytext=(xb, yy), arrowprops=dict(arrowstyle="<->", color=t["text"], lw=1), zorder=9)
        ax.text((xa + xb) / 2, yy - 0.08, lbl, color=t["text"], ha="center", va="top", fontsize=7.5, zorder=9)
    ax.text(x26 + 0.08, TAGS[26][1] - 0.22, "tag 26", color=t["tag"], fontsize=8, zorder=9,
            bbox=dict(boxstyle="round,pad=0.15", fc=t["bg"], ec="none", alpha=0.85))
    ax.legend(loc="upper left", fontsize=8, facecolor=t["bg"], edgecolor=t["sub"], labelcolor=t["text"])
    ax.set_title("Two stages: get close around obstacles, then drive straight in", color=t["text"], fontsize=11)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f"two-stages-{name}.png"), facecolor=t["bg"]); plt.close(fig)

def fig_tag_frame(t, name):
    fig, ax = plt.subplots(figsize=(7, 4.6), dpi=150, facecolor=t["bg"])
    field_base(ax, t, (-0.25, 2.9), (-0.55, 2.05), grid=False)
    for tid in (29, 30):
        x, y, _ = TAGS[tid]
        ax.text(x + 0.05, y, f" tag {tid}", color=t["tag"], va="center", fontsize=9, zorder=9)
    x, y, _ = TAGS[29]
    ax.add_patch(FancyArrowPatch((x, y), (x + 0.6, y), arrowstyle="-|>", mutation_scale=14, color="#ff6b6b", lw=2, zorder=9))
    ax.text(x + 0.62, y - 0.02, "+X: straight OUT of the tag", color="#ff6b6b", va="center", fontsize=8.5)
    ax.add_patch(FancyArrowPatch((x, y), (x, y - 0.45), arrowstyle="-|>", mutation_scale=14, color="#4dabf7", lw=2, zorder=9))
    ax.text(x + 0.05, y - 0.5, "+Y: the tag's left\n(= YOUR right when you look at it)", color="#4dabf7", va="top", fontsize=8)
    gx, gy, gd = standing(29, -0.216)
    robot(ax, gx, gy, gd, t["goal"], t, lw=2)
    ax.annotate("", xy=(gx + 0.62, TAGS[29][1]), xytext=(gx + 0.62, gy), arrowprops=dict(arrowstyle="<->", color=t["text"]))
    ax.text(gx + 0.68, (gy + TAGS[29][1]) / 2, "side offset 0.216 m\n(centered between tags 29 and 30)", color=t["text"], va="center", fontsize=8)
    ax.text(gx, gy + 0.6, "final pose\n(facing the tag)", color=t["goal"], ha="center", fontsize=8.5)
    ax.set_title("Every target is \"stand in front of tag N\": out along +X, sideways along Y", color=t["text"], fontsize=10.5)
    fig.subplots_adjust(left=0.1, right=0.97, top=0.9, bottom=0.12); fig.savefig(os.path.join(OUT, f"tag-frame-{name}.png"), facecolor=t["bg"]); plt.close(fig)

def fig_p_controller(t, name):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8), dpi=150, facecolor=t["bg"])
    d = [i / 100 for i in range(0, 101)]
    a1.plot(d, [min(3.0 * v, 1.5) for v in d], color=t["final"], lw=2.5)
    a1.axvline(0.5, color=t["sub"], ls=":", lw=1)
    a1.text(0.52, 0.4, "farther than 0.5 m:\nfull speed (1.5 m/s cap)", color=t["sub"], fontsize=8)
    a1.text(0.03, 1.25, "speed = 3.0 × distance", color=t["text"], fontsize=9)
    a1.set_xlabel("distance left to the final pose (m)", color=t["sub"], fontsize=8.5)
    a1.set_ylabel("speed we ask for (m/s)", color=t["sub"], fontsize=8.5)
    a1.set_title("The P controller rule", color=t["text"], fontsize=10.5)
    rows = [r for r in load_trace()[3] if not r["state"].startswith("Pathfinding")]
    gx, gy, _ = standing(26)
    t0 = float(rows[0]["t"])
    a2.plot([float(r["t"]) - t0 for r in rows], [math.hypot(float(r["x"]) - gx, float(r["y"]) - gy) for r in rows], color=t["path"], lw=2.5)
    a2.axhline(0.03, color=t["sub"], ls=":", lw=1)
    a2.text(0.02, 0.05, "3 cm tolerance", color=t["sub"], fontsize=8)
    a2.set_xlabel("time in stage 2 (s)", color=t["sub"], fontsize=8.5)
    a2.set_ylabel("distance left (m)", color=t["sub"], fontsize=8.5)
    a2.set_title("What the robot did (simulation)", color=t["text"], fontsize=10.5)
    for a in (a1, a2):
        a.set_facecolor(t["field"]); a.tick_params(colors=t["sub"], labelsize=8)
        for sp in a.spines.values():
            sp.set_color(t["sub"])
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f"p-controller-{name}.png"), facecolor=t["bg"]); plt.close(fig)

def fig_paddles(t, name):
    fig, ax = plt.subplots(figsize=(7.5, 3.8), dpi=150, facecolor=t["bg"])
    ax.set_facecolor(t["bg"]); ax.set_xlim(0, 10); ax.set_ylim(0, 5.2); ax.axis("off")
    from matplotlib.patches import FancyBboxPatch
    ax.add_patch(FancyBboxPatch((1.4, 2.1), 7.2, 2.0, boxstyle="round,pad=0.3,rounding_size=1.0", fc=t["field"], ec=t["sub"], lw=1.5))
    ax.text(5, 3.95, "BACK of the controller, seen as if it were see-through while you hold it", color=t["sub"], ha="center", fontsize=8)
    for cx, lbl, col, btn, tgt in ((3.0, "LEFT paddle", t["outpost"], "left stick click", "OUTPOST"),
                                   (7.0, "RIGHT paddle", t["hub"], "right stick click", "HUB")):
        ax.add_patch(FancyBboxPatch((cx - 0.3, 2.35), 0.6, 1.25, boxstyle="round,pad=0.05,rounding_size=0.28", fc=col, ec=col))
        ax.text(cx, 1.55, lbl, color=col, ha="center", va="top", fontsize=9.5, weight="bold")
        ax.text(cx, 1.1, f"copies the {btn}", color=t["text"], ha="center", va="top", fontsize=8.5)
        ax.text(cx, 0.65, f"hold = drive to the {tgt}", color=col, ha="center", va="top", fontsize=9, weight="bold")
    ax.text(5, 4.85, "Paddles aren't buttons the robot can see: the controller copies each one onto a normal button.",
            color=t["text"], ha="center", fontsize=8.5)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01); fig.savefig(os.path.join(OUT, f"paddles-{name}.png"), facecolor=t["bg"]); plt.close(fig)

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for name, t in THEMES.items():
        plt.rcParams["font.family"] = "DejaVu Sans"
        for f in (fig_field, fig_two_stage, fig_tag_frame, fig_p_controller, fig_paddles):
            f(t, name)
    print("wrote", sorted(os.listdir(OUT)))
