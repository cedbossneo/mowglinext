import importlib.util
from pathlib import Path
import sys


def _load_e2e_module():
    path = Path(__file__).resolve().parents[2] / 'e2e_test.py'
    spec = importlib.util.spec_from_file_location(
        'e2e_test_under_test', path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


e2e = _load_e2e_module()


def test_informational_failure_does_not_fail_e2e_process():
    criteria = [
        ('core cycle', True, True),
        ('historical coverage metric', False, False),
    ]

    assert e2e._required_criteria_pass(criteria)


def test_required_failure_fails_e2e_process():
    criteria = [
        ('core cycle', False, True),
        ('historical coverage metric', True, False),
    ]

    assert not e2e._required_criteria_pass(criteria)


def test_mowing_efficiency_is_one_for_exact_distance():
    assert e2e._mowing_efficiency(100.0, 100.0) == 1.0


def test_mowing_efficiency_penalizes_incomplete_traversal():
    assert e2e._mowing_efficiency(300.0, 30.0) == 0.1


def test_mowing_efficiency_penalizes_excess_distance():
    assert e2e._mowing_efficiency(300.0, 400.0) == 0.75


def test_mowing_efficiency_rejects_missing_distance():
    assert e2e._mowing_efficiency(0.0, 100.0) == 0.0
    assert e2e._mowing_efficiency(100.0, 0.0) == 0.0
