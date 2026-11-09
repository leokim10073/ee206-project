import json

import numpy as np
import pytest

from conftest import PKG, REPO
from maze_perception.config import load_perception_config


def test_marker_corners_in_opencv_order(cfg):
    # top-left, top-right, bottom-right, bottom-left of the marker image; image top toward +y
    c = cfg.board.marker_corners(0)
    cx, cy, cz = cfg.board.markers[0]
    h = cfg.board.marker_size / 2
    np.testing.assert_allclose(c, [[cx - h, cy + h, cz], [cx + h, cy + h, cz], [cx + h, cy - h, cz], [cx - h, cy - h, cz]])


def test_rectified_size(cfg):
    assert cfg.rectified.size == (600, 600)


@pytest.mark.parametrize("level", ["L1"])
def test_markers_match_cad(cfg, level):
    # the marker layout here must be the one the CAD prints and the sim draws
    report = REPO / "maze_CAD" / "levels" / level / "sim" / "parts_report.json"
    if not report.exists():
        pytest.skip(f"{report} not in this checkout")
    aruco = json.loads(report.read_text())["aruco"]
    assert aruco["dictionary"] == cfg.board.dictionary
    assert aruco["marker_mm"] / 1000 == pytest.approx(cfg.board.marker_size)
    assert aruco["marker_up"] == "+y"
    cad = {m["id"]: np.asarray(m["center_mm"]) / 1000 for m in aruco["markers"].values()}
    assert sorted(cad) == sorted(cfg.board.markers)
    for mid, center in cad.items():
        np.testing.assert_allclose(cfg.board.markers[mid], center, atol=1e-6)


def _write(tmp_path, text):
    p = tmp_path / "p.yaml"
    p.write_text(text)
    return p


def test_missing_key_is_named(tmp_path):
    src = (PKG / "config" / "perception.yaml").read_text().replace("  min_markers: 3", "")
    with pytest.raises(ValueError, match="min_markers"):
        load_perception_config(_write(tmp_path, src))


def test_bad_range_is_named(tmp_path):
    src = (PKG / "config" / "perception.yaml").read_text().replace("x_range: [-0.150, 0.150]", "x_range: [0.1, -0.1]")
    with pytest.raises(ValueError, match="rectified.x_range"):
        load_perception_config(_write(tmp_path, src))


def test_min_markers_bounded(tmp_path):
    src = (PKG / "config" / "perception.yaml").read_text().replace("min_markers: 3", "min_markers: 5")
    with pytest.raises(ValueError, match="pose.min_markers"):
        load_perception_config(_write(tmp_path, src))
