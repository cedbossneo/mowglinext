# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
from collections import defaultdict
from pathlib import Path
import sys
from types import SimpleNamespace


def _load_e2e_module():
    path = Path(__file__).resolve().parents[2] / "e2e_test.py"
    spec = importlib.util.spec_from_file_location("mowgli_e2e_ground_truth", path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _Logger:
    def info(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


def _area(module):
    points = [
        SimpleNamespace(x=0.0, y=0.0),
        SimpleNamespace(x=1.0, y=0.0),
        SimpleNamespace(x=1.0, y=1.0),
        SimpleNamespace(x=0.0, y=1.0),
    ]
    obstacle = SimpleNamespace(
        points=[
            SimpleNamespace(x=0.4, y=0.4),
            SimpleNamespace(x=0.6, y=0.4),
            SimpleNamespace(x=0.6, y=0.6),
            SimpleNamespace(x=0.4, y=0.6),
        ]
    )
    return SimpleNamespace(
        name="main",
        is_navigation_area=False,
        area=SimpleNamespace(points=points),
        obstacles=[obstacle],
    )


def test_immutable_target_excludes_obstacle_cells() -> None:
    module = _load_e2e_module()
    node = module.E2ETestNode.__new__(module.E2ETestNode)
    node.metrics = module.Metrics()
    node.get_logger = lambda: _Logger()

    assert node._set_mowing_area_target(_area(module), True)
    assert node.metrics.coverage_target_ready
    assert (1, 1) in node.metrics.mow_area_cells
    assert (10, 10) not in node.metrics.mow_area_cells


def test_blade_sweep_marks_footprint_not_only_robot_center() -> None:
    module = _load_e2e_module()
    node = module.E2ETestNode.__new__(module.E2ETestNode)
    node.metrics = module.Metrics()
    node.metrics.mow_area_cells = {
        (x, y)
        for x in range(-10, 11)
        for y in range(-10, 11)
    }
    node.metrics.covered_cell_visits = defaultdict(int)

    node._mark_blade_sweep(0.0, 0.0, 0.2, 0.0, 1.0)

    assert len(node.metrics.covered_cells) > 4
    assert any(y != 0 for _, y in node.metrics.covered_cells)


def test_completion_counters_are_read_before_state_transition() -> None:
    source = (
        Path(__file__).resolve().parents[2] / "e2e_test.py"
    ).read_text()
    callback = source.split("def _on_bt_status", 1)[1].split(
        "def _track_phase_transition", 1
    )[0]

    assert callback.index("self.metrics.swath_count =") < callback.index(
        "self._track_phase_transition("
    )
