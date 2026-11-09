"""
UR-aMAZE-ing maze: shared board dimensions and geometry helpers, the same for every level.
Pure Python (no dependencies). Level layouts live in levels.py.

Design frame ("plate frame"): origin = bottom-left corner of the 300x300 base plate
(top view), x right, y up, z up, base bottom at z = 0. Units mm.
Grid rows are counted from the TOP (row 0 = top), columns from the LEFT.
The grip tab is on the -x edge.

The sim export (export_sim.py) moves this to the sim frame: origin at the plate center,
z = 0 on the maze floor.
"""

P = dict(
    base=300.0,      # base plate size (square, without tab and ArUco pads)
    base_t=6.0,      # base plate thickness (maze floor at z = base_t)
    margin=3.0,      # lip between plate edge and outer wall
    wall_t=6.0,      # wall thickness
    wall_h=18.0,     # wall height above the floor
    ball_d=20.0,     # ball diameter the print was sized for (sim uses its own radius)
    goal_d=30.0,     # goal pocket diameter
    goal_depth=3.0,  # goal pocket depth: the ball needs a ~45 deg tilt to climb out
    start_ring=(11.0, 13.0, 0.6),  # start marker: engraved ring (inner r, outer r, depth);
                                   # a ring, not a pocket, so the ball rolls off at a small tilt
    handle_w=60.0,   # grip tab width (along the plate edge, y)
    handle_l=50.0,   # grip tab protrusion from the plate edge (-x)
    handle_h=12.0,   # grip tab thickness (fingers clamp its top & bottom faces)
    tab_embed=9.0,   # printed tab runs this far under the outer wall
    tab_fillet=4.0,  # plan-view radius of the tab's two outer corners
    pad_x=36.0, pad_y=30.0, pad_depth=1.0,  # finger-pad locating recesses (print only)
    tag_marker=40.0,   # ArUco black-square size
    tag_border=5.0,    # white quiet zone on the printed sticker
    tag_pad=58.0,      # ArUco corner pad size, extends outward from the top/bottom edges
    tag_recess=0.4,    # sticker locating recess depth (print only)
)

# ArUco markers on the four corner pads. The same four markers go on every level's board.
# Confirm the dictionary and ids with perception before printing the stickers.
ARUCO = dict(
    dictionary="DICT_4X4_50",
    ids={"TL": 0, "TR": 1, "BR": 2, "BL": 3},
    # each marker's top edge faces board +y (away from the bottom edge in the top view)
)


def pitch(L):
    return L["cell"] + P["wall_t"]


def line_pos(L, k):
    """near edge of grid line k, measured from the plate's left (or top) edge"""
    return P["margin"] + k * pitch(L)


def to_y(top_dist):
    """distance from the plate's top edge -> y coordinate"""
    return P["base"] - top_dist


def wall_rects_named(L):
    """[(name, (x, y, w, h))] — every wall as one straight rectangle in plan view.
    Corners of touching walls overlap; that is intended."""
    t = P["wall_t"]; n = L["n"]; m = P["margin"]
    span = line_pos(L, n) + t - m
    out = [
        ("outer_top",    (m, to_y(line_pos(L, 0) + t), span, t)),
        ("outer_bottom", (m, to_y(line_pos(L, n) + t), span, t)),
        ("outer_left",   (line_pos(L, 0), to_y(line_pos(L, n) + t), t, span)),
        ("outer_right",  (line_pos(L, n), to_y(line_pos(L, n) + t), t, span)),
    ]
    for j, c0, c1 in L["h_walls"]:
        x0 = line_pos(L, c0); x1 = line_pos(L, c1 + 1) + t
        out.append((f"h_line{j}_c{c0}-{c1}", (x0, to_y(line_pos(L, j) + t), x1 - x0, t)))
    for i, r0, r1 in L["v_walls"]:
        y0 = line_pos(L, r0); y1 = line_pos(L, r1 + 1) + t
        out.append((f"v_line{i}_r{r0}-{r1}", (line_pos(L, i), to_y(y1), t, y1 - y0)))
    return out


def wall_rects(L):
    return [r for _, r in wall_rects_named(L)]


def tag_pads():
    """{name: (x, y, size, (cx, cy))} — lower-left corner, size and center of each ArUco pad"""
    S = P["tag_pad"]; B = P["base"]
    pads = {"TL": (0, B), "TR": (B - S, B), "BL": (0, -S), "BR": (B - S, -S)}
    return {k: (x, y, S, (x + S / 2, y + S / 2)) for k, (x, y) in pads.items()}


def sticker_size():
    """printed sticker side: marker + white quiet zone on both sides"""
    return P["tag_marker"] + 2 * P["tag_border"]


def cell_center(L, rc):
    r, c = rc
    t = P["wall_t"]
    return (line_pos(L, c) + t + L["cell"] / 2, to_y(line_pos(L, r) + t + L["cell"] / 2))
