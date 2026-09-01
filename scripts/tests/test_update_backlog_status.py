from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts/update_backlog_status.py"
MANIFEST = REPO_ROOT / "docs/status/mowgli_backlog.json"
GNSS_AUDIT = REPO_ROOT / "MOWGLINEXT_GNSS_AUDIT.md"
RUNTIME_TODO = REPO_ROOT / "TODO-runtime-backups.md"
SPEC = importlib.util.spec_from_file_location("update_backlog_status", SCRIPT)
assert SPEC and SPEC.loader
dashboard = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = dashboard
SPEC.loader.exec_module(dashboard)


class BacklogDashboardTest(unittest.TestCase):
    def setUp(self) -> None:
        self.data = json.loads(MANIFEST.read_text(encoding="utf-8"))

    def test_public_manifest_conserves_baseline_and_counts(self) -> None:
        dashboard.validate_manifest(self.data)
        summary = dashboard.summarize(self.data)
        self.assertEqual(18, summary.baseline)
        self.assertEqual(7, summary.remaining)
        self.assertEqual(11, summary.resolved)
        self.assertEqual(("DEPLOYMENT", 7), summary.scopes[0])
        self.assertEqual(0, summary.hardware_required)
        self.assertEqual(0, summary.hardware_pending)

    def test_manifest_items_map_to_durable_source_ledgers(self) -> None:
        audit = GNSS_AUDIT.read_text(encoding="utf-8")
        gnss_ids = {
            item["id"] for item in self.data["items"] if item["id"].startswith("MGNSS-")
        }
        self.assertEqual({f"MGNSS-{index:03d}" for index in range(1, 12)}, gnss_ids)
        for item_id in gnss_ids:
            self.assertIn(f"| {item_id} |", audit)

        future_section = RUNTIME_TODO.read_text(encoding="utf-8").split(
            "## Future improvements", 1
        )[1].split("## Important invariant", 1)[0]
        source_bullets = {
            line.removeprefix("- ")
            for line in future_section.splitlines()
            if line.startswith("- ")
        }

    def test_scope_sorting_is_count_descending_then_name(self) -> None:
        summary = dashboard.summarize(self.data)
        self.assertEqual(
            list(summary.scopes),
            sorted(summary.scopes, key=lambda pair: (-pair[1], pair[0])),
        )
        self.assertTrue(all(count > 0 for _, count in summary.scopes))

    def test_svg_is_deterministic_valid_xml_and_height_tracks_scope_rows(self) -> None:
        summary = dashboard.summarize(self.data)
        first = dashboard.render_svg(self.data, summary)
        second = dashboard.render_svg(self.data, summary)
        self.assertEqual(first, second)
        root = ET.fromstring(first)
        self.assertEqual("408", root.attrib["height"])
        self.assertIn("MowgliNext backlog status", first)

        fewer_scopes = copy.deepcopy(self.data)
        fewer_scopes["items"] = [
            item
            for item in fewer_scopes["items"]
            if item["scope"] in {"DEPLOYMENT", "LOCALIZATION"}
        ]
        fewer_scopes["baseline_total"] = len(fewer_scopes["items"])
        compact_root = ET.fromstring(
            dashboard.render_svg(fewer_scopes, dashboard.summarize(fewer_scopes))
        )
        self.assertLess(int(compact_root.attrib["height"]), int(root.attrib["height"]))

    def test_hardware_validation_is_orthogonal_to_status(self) -> None:
        data = copy.deepcopy(self.data)
        data["items"][0]["validation"] = "HARDWARE_REQUIRED"
        data["items"][1]["validation"] = "HARDWARE_PENDING"
        dashboard.validate_manifest(data)
        summary = dashboard.summarize(data)
        self.assertEqual(1, summary.hardware_required)
        self.assertEqual(1, summary.hardware_pending)
        self.assertEqual(11, summary.resolved)

    def test_directional_duplicate_requires_canonical_target(self) -> None:
        data = copy.deepcopy(self.data)
        duplicate = data["items"][0]
        duplicate["status"] = "DUPLICATE"
        duplicate["duplicate_of"] = data["items"][1]["id"]
        dashboard.validate_manifest(data)
        data["items"][1]["status"] = "DUPLICATE"
        with self.assertRaises(dashboard.ManifestError):
            dashboard.validate_manifest(data)

    def test_cli_check_detects_and_accepts_generated_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            svg = root / "status.svg"
            readme = root / "README.md"
            manifest.write_text(json.dumps(self.data), encoding="utf-8")
            readme.write_text("# Test\n\n## Dashboard\n", encoding="utf-8")
            command = [
                sys.executable,
                str(SCRIPT),
                "--manifest",
                str(manifest),
                "--svg",
                str(svg),
                "--readme",
                str(readme),
            ]
            subprocess.run(command, check=True)
            subprocess.run([*command, "--check"], check=True)
            svg.write_text(svg.read_text(encoding="utf-8") + "stale", encoding="utf-8")
            stale = subprocess.run([*command, "--check"], check=False)
            self.assertEqual(1, stale.returncode)


if __name__ == "__main__":
    unittest.main()
