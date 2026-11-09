"""
UR-aMAZE-ing  maze design schematic for one level.
Dimensions live in maze_params.py, layouts in levels.py. All dimensions in mm.
Coordinate system: origin = bottom-left corner of base plate, x right, y up (top view).
Rows are counted from the TOP (row 0 = top), columns from the LEFT.

Run:  python3 maze_CAD/scripts/draw_schematic.py --level L1   # -> maze_CAD/levels/L1/schematic.png
"""
import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle, Circle  # noqa: E402
from matplotlib import font_manager as fm  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from levels import LEVELS  # noqa: E402
from maze_params import ARUCO, P, wall_rects, tag_pads, cell_center, line_pos, sticker_size  # noqa: E402

# ---------------- drawing ----------------
# labels are Chinese: use Noto Sans CJK if it is installed (apt: fonts-noto-cjk)
for f in ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
          "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"]:
    if os.path.exists(f):
        fm.fontManager.addfont(f)
        plt.rcParams["font.family"] = "Noto Sans CJK TC"

INK = "#1f2933"; WALL = "#3d5a80"; BASE = "#e9eef5"; PATH = "#e07a5f"
KEEP = "#f2cc8f"; DIM = "#6b7280"; GOAL = "#2a9d8f"

def dim_h(ax, x0, x1, y, text, off=0):
    ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.9))
    ax.text((x0 + x1) / 2, y + 3 + off, text, ha="center", va="bottom", fontsize=8, color=DIM)

def dim_v(ax, x, y0, y1, text):
    ax.annotate("", (x, y0), (x, y1), arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.9))
    ax.text(x - 3, (y0 + y1) / 2, text, ha="right", va="center", fontsize=8, color=DIM, rotation=90)

def draw_top(ax, L):
    B = P["base"]
    # gripper keep-out zone + handle (left side)
    hw, hl = P["handle_w"], P["handle_l"]
    ax.add_patch(Rectangle((-hl - 25, B / 2 - 60), hl + 25, 120, fc=KEEP, ec="none", alpha=0.45))
    ax.text(-hl / 2 - 12, B / 2 + 62, "夾爪淨空區\nGripper keep-out", ha="center", va="bottom", fontsize=8, color="#9a6b1f")
    ax.add_patch(Rectangle((-hl, B / 2 - hw / 2), hl, hw, fc="#98c1d9", ec=INK, lw=1))
    ax.text(-hl / 2, B / 2, "Grip tab\n夾取平板", ha="center", va="center", fontsize=7.5, color=INK)
    # finger pads (illustrative)
    ax.add_patch(Rectangle((-hl + 4, B / 2 - P["pad_y"] / 2), P["pad_x"], P["pad_y"], fc="none", ec=INK, lw=0.7, ls="--"))
    # ArUco corner pads
    tsz = sticker_size()
    for k, (x, y, S, (cx, cy)) in tag_pads().items():
        ax.add_patch(Rectangle((x, y), S, S, fc=BASE, ec=INK, lw=1.0))
        ax.add_patch(Rectangle((cx - tsz / 2, cy - tsz / 2), tsz, tsz, fc="white", ec=DIM, lw=0.6, ls="--"))
        ax.add_patch(Rectangle((cx - P["tag_marker"] / 2, cy - P["tag_marker"] / 2), P["tag_marker"], P["tag_marker"], fc="#222", ec="none"))
        ax.text(cx, cy, f"{k}\nid {ARUCO['ids'][k]}", ha="center", va="center", fontsize=7, color="white", weight="bold")
    # base plate
    ax.add_patch(Rectangle((0, 0), B, B, fc=BASE, ec=INK, lw=1.2))
    # walls
    for (x, y, w, h) in wall_rects(L):
        ax.add_patch(Rectangle((x, y), w, h, fc=WALL, ec="none"))
    # intended path (dashed), from the level definition
    pts = [cell_center(L, rc) for rc in L["path"]]
    xs, ys = zip(*pts)
    ax.plot(xs, ys, ls=(0, (4, 3)), color=PATH, lw=1.6, zorder=3)
    # start & goal
    sx, sy = cell_center(L, L["start"]); gx, gy = cell_center(L, L["goal"])
    ax.add_patch(Circle((sx, sy), P["ball_d"] / 2, fc="#c0c0c0", ec=INK, lw=0.8, zorder=4))
    ax.text(sx, sy, "S", ha="center", va="center", fontsize=8, weight="bold", zorder=5)
    ax.add_patch(Circle((gx, gy), P["goal_d"] / 2, fc="white", ec=GOAL, lw=1.6, zorder=4))
    ax.add_patch(Circle((gx, gy), P["goal_d"] / 2 - 4, fc=GOAL, ec="none", alpha=0.25, zorder=4))
    ax.text(gx, gy, "G", ha="center", va="center", fontsize=8, weight="bold", color=GOAL, zorder=5)
    # row / col labels
    for k in range(L["n"]):
        cx, _ = cell_center(L, (0, k)); _, cy = cell_center(L, (k, 0))
        ax.text(cx, B + 4, f"c{k}" if 1 <= k <= 4 else "", ha="center", va="bottom", fontsize=7, color=DIM)
        ax.text(B + 4, cy, f"r{k}", ha="left", va="center", fontsize=7, color=DIM)
    # dimensions
    S = P["tag_pad"]
    dim_h(ax, 0, B, -S - 18, "300")
    dim_v(ax, B + 30, 0, B, "300")
    dim_v(ax, B + 30, B, B + S, f"{S:.0f}")
    dim_v(ax, B + 30, -S, 0, f"{S:.0f}")
    ax.text(B + 46, B + S / 2, f"ArUco {P['tag_marker']:.0f} mm\n(貼紙 {tsz:.0f}×{tsz:.0f})", fontsize=7.5, va="center", color=INK)
    x_c0 = line_pos(L, 1) + P["wall_t"]; x_c1 = line_pos(L, 2)
    dim_h(ax, x_c0, x_c1, B + 22, f"{L['cell']:.0f} 通道", off=0)
    dim_h(ax, -P["handle_l"], 0, B / 2 - P["handle_w"] / 2 - 26, f'{P["handle_l"]:.0f}')
    dim_v(ax, -P["handle_l"] - 6, B / 2 - P["handle_w"] / 2, B / 2 + P["handle_w"] / 2, f'{P["handle_w"]:.0f}')
    ax.set_xlim(-110, B + 95); ax.set_ylim(-P["tag_pad"] - 40, B + P["tag_pad"] + 25)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(f"俯視圖 Top view — {L['name']}, {L['n']}×{L['n']} grid", fontsize=11, loc="left", color=INK)

def draw_section(ax, L):
    B = P["base"]; bt = P["base_t"]; wh = P["wall_h"]
    hl, hh = P["handle_l"], P["handle_h"]
    ax.add_patch(Rectangle((0, 0), B, bt, fc=BASE, ec=INK, lw=1))
    # walls cut on section through row 0 (outer walls + one stub)
    for x in (P["margin"], line_pos(L, 3), line_pos(L, L["n"])):
        ax.add_patch(Rectangle((x, bt), P["wall_t"], wh, fc=WALL, ec="none"))
    ax.add_patch(Rectangle((-hl, 0), hl, hh, fc="#98c1d9", ec=INK, lw=1))
    # fingers
    for zz in (hh, -8):
        ax.add_patch(Rectangle((-hl - 30, zz), hl - 4 + 30, 8, fc="#555", alpha=0.6, ec="none"))
    ax.annotate("", (-hl - 38, hh + 4), (-hl - 38, -4), arrowprops=dict(arrowstyle="<->", color=INK, lw=0.9))
    ax.text(-hl - 30, -20, "夾爪側向進入，上下夾住平板（厚 12）", ha="left", fontsize=7.5, color=INK)
    # ball
    bx = 30
    ax.add_patch(Circle((bx, bt + P["ball_d"] / 2), P["ball_d"] / 2, fc="#c0c0c0", ec=INK, lw=0.8))
    # goal pocket (illustrative position)
    gx = 270; gd = P["goal_d"]
    ax.add_patch(Rectangle((gx - gd / 2, bt - P["goal_depth"]), gd, P["goal_depth"], fc="white", ec=GOAL, lw=1))
    ax.text(gx, -9, "Goal 凹槽 Ø30 深3", ha="center", fontsize=7.5, color=GOAL)
    dim_v(ax, B + 14, 0, bt, "6")
    dim_v(ax, B + 34, bt, bt + wh, "18")
    ax.text(bx + 14, bt + P["ball_d"] / 2, "球 Ø20", fontsize=7.5, color=INK)
    ax.set_xlim(-110, B + 50); ax.set_ylim(-26, 50)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("側視剖面 Side section", fontsize=11, loc="left", color=INK)

def draw_plan(ax):
    ax.axis("off")
    ax.set_title("三階段迷宮規劃 Three-level plan", fontsize=11, loc="left", color=INK)
    rows = [
        ["", "Level 1 簡單", "Level 2 中等", "Level 3 困難"],
        ["格數 Grid", "6 × 6", "7 × 7", "8 × 8"],
        ["通道寬 Corridor", "42 mm (2.1×球)", "36 mm (1.8×球)", "30 mm (1.5×球)"],
        ["路徑", "3 條 S 型車道\n無死路", "多轉角\n3–4 條死路", "長路徑 + 窄彎\n死路多"],
        ["陷阱洞 Holes", "無", "無", "2–3 個 Ø26 洞"],
        ["測試目標", "MPC 基本收斂\n視覺追蹤", "轉角精度\n路徑規劃", "安全邊界\n約束 MPC"],
    ]
    tb = ax.table(cellText=rows, loc="upper left", cellLoc="center", colWidths=[0.22, 0.26, 0.26, 0.26],
                  bbox=[0, 0, 1, 0.92])
    tb.auto_set_font_size(False); tb.set_fontsize(8)
    for (r, c), cell in tb.get_celld().items():
        cell.set_edgecolor("#c9d2dc")
        if r == 0:
            cell.set_facecolor("#3d5a80"); cell.get_text().set_color("white"); cell.get_text().set_weight("bold")
        elif c == 0:
            cell.set_facecolor("#eef2f6")
        if c == 1 and r > 0:
            cell.set_facecolor("#fbe9e3")

def draw_notes(ax):
    ax.axis("off")
    notes = (
        "共用規格（三關一致，方便換關）\n"
        "• 底板 300×300×6，外框牆內距 294，牆厚 6、高 18\n"
        "• 夾取平板 50×60×12 在左側中央，夾爪側向進入、夾上下兩面\n"
        "  上下各有 1 mm 定位凹槽（36×30）防滑；三關同位置 → 姿態不必重教\n"
        "• 起點 S 為 0.6 mm 刻環（不是凹槽，球才滾得出去）；終點 G 為 Ø30×3 凹槽，球到達後會被留住\n"
        "• 四角 ArUco 平台 58×58，頂面與迷宮地板同高（z=6），\n"
        "  有 0.4 mm 定位凹槽放 50×50 貼紙（標記 40 + 白邊 5）\n\n"
        "假設（請確認）\n"
        "• 夾爪未定：平板厚 12，2F-85 / Hand-E 都夾得到\n"
        "• 球 Ø20（鋼珠或彩色樹脂球，視覺較好追）\n"
        "• 一體版含平板與四角平台共 350×416；台面不夠 → 用 4 片拼接版（最大 200×238）"
    )
    ax.text(0, 1, notes, va="top", ha="left", fontsize=8.3, color=INK, linespacing=1.55)

def main():
    ap = argparse.ArgumentParser(description="Draw the design schematic of a maze level.")
    ap.add_argument("--level", default="L1", choices=sorted(LEVELS))
    a = ap.parse_args()
    L = LEVELS[a.level]
    out = Path(__file__).resolve().parents[1] / "levels" / a.level / "schematic.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(14, 9.2), dpi=160)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.25, 1], height_ratios=[1, 0.95, 0.55], hspace=0.12, wspace=0.05)
    ax_top = fig.add_subplot(gs[0:2, 0]); draw_top(ax_top, L)
    ax_sec = fig.add_subplot(gs[2, 0]); draw_section(ax_sec, L)
    ax_plan = fig.add_subplot(gs[0, 1]); draw_plan(ax_plan)
    ax_note = fig.add_subplot(gs[1:3, 1]); draw_notes(ax_note)
    fig.suptitle(f"UR-aMAZE-ing 迷宮設計示意圖 {a.level}   (單位 mm)", fontsize=14, x=0.02, ha="left", color=INK, weight="bold")
    fig.savefig(out, bbox_inches="tight", facecolor="white")
    print("->", out)


if __name__ == "__main__":
    main()
