from pathlib import Path

from igvc_competition_sim.course import load_course

PKG = Path(__file__).resolve().parents[1]


def test_course_loads_without_robot_block(tmp_path):
    # A course YAML with NO robot: block must load.
    src = (PKG / "config" / "igvc_competition_compact.yaml").read_text(
        encoding="utf-8")
    import re
    stripped = re.sub(r"(?ms)^robot:\n(?: .*\n)+", "", src)
    p = tmp_path / "course.yaml"
    p.write_text(stripped, encoding="utf-8")
    course = load_course(p)
    assert course.course_id == "igvc_competition_compact"
    assert not hasattr(course, "robot")


def test_course_ignores_stray_robot_block(tmp_path):
    # A course YAML with a legacy robot: block must still load (block ignored).
    src = (PKG / "config" / "igvc_competition_compact.yaml").read_text(
        encoding="utf-8")
    p = tmp_path / "course.yaml"
    p.write_text(src + "\nrobot:\n  wheel_track_m: 0.5\n", encoding="utf-8")
    course = load_course(p)
    assert course.course_id == "igvc_competition_compact"
