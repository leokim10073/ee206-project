"""ArUco markers for the four corner pads: images, sim texture quads and a printable sticker sheet.

    python3 maze_CAD/scripts/aruco.py      # -> maze_CAD/aruco/: one PNG per marker + aruco_stickers_A4.pdf

The same four markers go on every level's board; ids and dictionary are in maze_params.ARUCO.
export_sim.py calls write_sim_quads() to put a textured quad per marker next to the sim parts.

Needs numpy and opencv with the aruco module (cv2.aruco: opencv-contrib, or opencv >= 4.7);
the sticker sheet also needs matplotlib.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from maze_params import ARUCO, P, sticker_size  # noqa: E402

PX_PER_MM = 10          # sticker image resolution (400 px across the 40 mm marker)
OUT = Path(__file__).resolve().parents[1] / "aruco"


def _dictionary():
    import cv2
    name = ARUCO["dictionary"]
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_image(marker_id, side_px):
    """black-and-white marker, side_px square, no quiet zone (uint8, 0 = black)"""
    import cv2
    d = _dictionary()
    if hasattr(cv2.aruco, "generateImageMarker"):   # opencv >= 4.7
        return cv2.aruco.generateImageMarker(d, marker_id, side_px)
    return cv2.aruco.drawMarker(d, marker_id, side_px)  # opencv 4.5 (sim container)


def sticker_image(marker_id):
    """marker plus its white quiet zone, as printed on the sticker (uint8)"""
    m_px = int(round(P["tag_marker"] * PX_PER_MM))
    b_px = int(round(P["tag_border"] * PX_PER_MM))
    img = np.full((m_px + 2 * b_px,) * 2, 255, np.uint8)
    img[b_px:b_px + m_px, b_px:b_px + m_px] = marker_image(marker_id, m_px)
    return img


def _write_png(path, img):
    import cv2
    if not cv2.imwrite(str(path), img):
        raise OSError(f"could not write {path}")


def write_sim_quads(out_dir, centers_mm, z_mm):
    """One textured sticker per marker for the sim: aruco_<corner>.obj and aruco_<corner>.png
    (the sticker image). centers_mm: {corner: (x, y)} in the sim frame. Each sticker is a closed
    slab from the floor (z = 0) up to z_mm, so MuJoCo accepts it as a mesh with volume; only its
    top face carries the image. Returns report entries."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in [*out_dir.glob("aruco_*.obj"), *out_dir.glob("aruco_*.png")]:
        old.unlink()
    h = sticker_size() / 2
    entries = {}
    for corner, (cx, cy) in centers_mm.items():
        mid = ARUCO["ids"][corner]
        _write_png(out_dir / f"aruco_{corner}.png", sticker_image(mid))
        # corners counter-clockwise from the bottom-left; image top (v = 1) on the +y edge
        xy = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]
        lines = [f"# ArUco {ARUCO['dictionary']} id {mid}, sticker {sticker_size():g} mm, units mm"]
        lines += [f"v {x:.4f} {y:.4f} {z:.4f}" for z in (z_mm, 0.0) for x, y in xy]  # 1-4 top, 5-8 bottom
        lines += ["vt 0 0", "vt 1 0", "vt 1 1", "vt 0 1", "vt 0.02 0.02"]  # 5: a point in the white border, for the hidden faces
        lines += ["f 1/1 2/2 3/3", "f 1/1 3/3 4/4",          # top: the sticker image
                  "f 5/5 7/5 6/5", "f 5/5 8/5 7/5"]          # bottom
        for a in range(4):                                     # sides
            b = (a + 1) % 4
            lines += [f"f {a + 5}/5 {b + 5}/5 {b + 1}/5", f"f {a + 5}/5 {b + 1}/5 {a + 1}/5"]
        (out_dir / f"aruco_{corner}.obj").write_text("\n".join(lines) + "\n")
        entries[corner] = {"id": mid, "center_mm": [round(cx, 3), round(cy, 3), z_mm],
                           "obj": f"aruco/aruco_{corner}.obj", "texture": f"aruco/aruco_{corner}.png"}
    return entries


def write_sticker_sheet(path):
    """A4 PDF at 1:1 scale with the four stickers, a cut outline and a 50 mm scale check."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    W, H = 210.0, 297.0
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
    s = sticker_size()
    slots = {"TL": (35, 180), "TR": (125, 180), "BL": (35, 85), "BR": (125, 85)}
    for corner, (x, y) in slots.items():
        mid = ARUCO["ids"][corner]
        ax.imshow(sticker_image(mid), cmap="gray", vmin=0, vmax=255, extent=(x, x + s, y, y + s),
                  interpolation="nearest")
        ax.add_patch(plt.Rectangle((x, y), s, s, fill=False, ec="#999", lw=0.3, ls=(0, (2, 2))))
        ax.text(x + s / 2, y - 4, f"{corner}  |  id {mid}  |  this edge toward the board's bottom edge",
                ha="center", va="top", fontsize=6.5, color="#444")
    ax.text(W / 2, 270, f"Maze corner markers — ArUco {ARUCO['dictionary']}, marker {P['tag_marker']:g} mm, "
            f"sticker {s:g} mm", ha="center", fontsize=9)
    ax.text(W / 2, 262, "Print at 100 % (no fit-to-page). Cut on the dashed lines.", ha="center", fontsize=8,
            color="#444")
    ax.plot([30, 80], [40, 40], color="black", lw=0.8)
    for xx in (30, 80):
        ax.plot([xx, xx], [38, 42], color="black", lw=0.8)
    ax.text(55, 44, "50 mm: measure to check the print scale", ha="center", fontsize=7)
    fig.savefig(path)
    plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    for corner, mid in ARUCO["ids"].items():
        _write_png(OUT / f"marker_{corner}_id{mid}.png", sticker_image(mid))
    write_sticker_sheet(OUT / "aruco_stickers_A4.pdf")
    print(f"{len(ARUCO['ids'])} markers ({ARUCO['dictionary']}) and aruco_stickers_A4.pdf -> {OUT}")


if __name__ == "__main__":
    main()
