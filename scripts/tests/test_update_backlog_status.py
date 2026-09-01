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
        self.assertEqual(45, summary.baseline)
        self.assertEqual(34, summary.remaining)
        self.assertEqual(11, summary.resolved)
        self.assertEqual(
            (("OPEN", 32), ("PARTIAL", 1), ("BLOCKED", 1), ("IMPLEMENTED", 11)),
            summary.statuses,
        )
        self.assertEqual(("DEPLOYMENT", 9), summary.scopes[0])
        self.assertEqual(7, summary.hardware_required)
        self.assertEqual(4, summary.hardware_pending)
        self.assertEqual(79, summary.total_sources)
        self.assertEqual(30, summary.tracked_issues)
        self.assertEqual(31, summary.tracked_prs)
        self.assertEqual(26, summary.open_issues)
        self.assertEqual(20, summary.open_prs)
        self.assertEqual(45, summary.canonical_findings)
        self.assertEqual(2, summary.duplicates)

    def test_document_sources_map_to_durable_ledgers(self) -> None:
        audit = GNSS_AUDIT.read_text(encoding="utf-8")
        gnss_ids = {
            finding["id"]
            for finding in self.data["findings"]
            if finding["id"].startswith("MGNSS-")
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
        todo_sources = {
            source["source_text"]
            for source in self.data["sources"]
            if source["source_type"] == "TODO"
        }
        self.assertEqual(source_bullets, todo_sources)

    def test_github_inventory_and_expansion_conserve_snapshot(self) -> None:
        inventory = self.data["github_inventory"]
        self.assertEqual(503, inventory["total_sources"])
        self.assertEqual(
            inventory["total_sources"],
            inventory["total_issues"] + inventory["total_prs"],
        )
        self.assertEqual(
            inventory["closed_sources"],
            inventory["expanded_related_closed_sources"]
            + inventory["closed_sources_not_expanded"],
        )
        self.assertEqual(46, inventory["expanded_open_sources"])
        self.assertEqual(15, inventory["expanded_related_closed_sources"])
        self.assertEqual(442, inventory["closed_sources_not_expanded"])

        github_sources = [
            source
            for source in self.data["sources"]
            if source["source_type"] in {"ISSUE", "PR"}
        ]
        self.assertEqual(61, len(github_sources))
        self.assertEqual(
            46,
            sum(source["github_state"] == "OPEN" for source in github_sources),
        )
        self.assertEqual(
            {
                ".agent/checkpoints/BACKLOG_DASHBOARD_CHECKPOINT.md",
            },
            {source["checkpoint"] for source in github_sources},
        )

    def test_sources_are_distinct_from_canonical_findings(self) -> None:
        finding_ids = {finding["id"] for finding in self.data["findings"]}
        self.assertEqual(45, len(finding_ids))
        self.assertEqual(79, len(self.data["sources"]))
        self.assertTrue(
            all(source["canonical_id"] in finding_ids for source in self.data["sources"])
        )
        maintenance_sources = [
            source
            for source in self.data["sources"]
            if source["canonical_id"] == "MN-MAINT-001"
        ]
        self.assertEqual(13, len(maintenance_sources))
        self.assertEqual(
            {"PR"}, {source["source_type"] for source in maintenance_sources}
        )

    def test_pr_migration_dispositions_are_durable(self) -> None:
        pull_requests = {
            source["source_id"]: source
            for source in self.data["sources"]
            if source["source_type"] == "PR"
        }
        for number in (340, 345, 432, 434, 440, 441, 485, 489):
            self.assertEqual(
                "ALREADY_PRESENT", pull_requests[number]["migration_disposition"]
            )
        for number in (433, 439):
            self.assertEqual("ADAPT", pull_requests[number]["migration_disposition"])
        self.assertEqual("SUPERSEDED", pull_requests[456]["migration_disposition"])
        self.assertTrue(
            all(
                source["migration_disposition"] == "ADAPT"
                for source in pull_requests.values()
                if source["github_state"] == "OPEN"
            )
        )

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
        self.assertGreater(int(root.attrib["height"]), 334)
        self.assertIn("MowgliNext backlog status", first)
        self.assertIn("79", first)
        self.assertIn("30 · 26 open", first)
        self.assertIn("31 · 20 open", first)

        fewer_scopes = copy.deepcopy(self.data)
        fewer_scopes["findings"] = [
            finding
            for finding in fewer_scopes["findings"]
            if finding["scope"] in {"DEPLOYMENT", "LOCALIZATION"}
        ]
        fewer_scopes["baseline_total"] = len(fewer_scopes["findings"])
        compact_root = ET.fromstring(
            dashboard.render_svg(fewer_scopes, dashboard.summarize(fewer_scopes))
        )
        self.assertLess(int(compact_root.attrib["height"]), int(root.attrib["height"]))

    def test_hardware_validation_is_orthogonal_to_status(self) -> None:
        data = copy.deepcopy(self.data)
        before = dashboard.summarize(data)
        data["findings"][0]["validation"] = "HARDWARE_REQUIRED"
        data["findings"][1]["validation"] = "HARDWARE_PENDING"
        for source in data["sources"]:
            if source["canonical_id"] == data["findings"][0]["id"]:
                source["validation"] = "HARDWARE_REQUIRED"
            if source["canonical_id"] == data["findings"][1]["id"]:
                source["validation"] = "HARDWARE_PENDING"
        dashboard.validate_manifest(data)
        summary = dashboard.summarize(data)
        self.assertEqual(before.hardware_required + 1, summary.hardware_required)
        self.assertEqual(before.hardware_pending + 1, summary.hardware_pending)
        self.assertEqual(before.resolved, summary.resolved)

    def test_directional_duplicate_requires_canonical_target(self) -> None:
        data = copy.deepcopy(self.data)
        dashboard.validate_manifest(data)
        sources = {source["source_key"]: source for source in data["sources"]}
        self.assertEqual("GH-ISSUE-446", sources["GH-ISSUE-486"]["duplicate_of"])
        self.assertEqual("GH-ISSUE-451", sources["GH-ISSUE-449"]["duplicate_of"])
        sources["GH-ISSUE-446"]["status"] = "DUPLICATE"
        sources["GH-ISSUE-446"]["duplicate_of"] = "GH-ISSUE-486"
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
