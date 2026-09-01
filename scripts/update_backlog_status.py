#!/usr/bin/env python3
"""Generate and verify the public MowgliNext backlog dashboard."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from html import escape
import json
from pathlib import Path
import re
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = REPO_ROOT / "docs/status/mowgli_backlog.json"
DEFAULT_SVG = REPO_ROOT / "docs/status/mowgli_backlog.svg"
DEFAULT_README = REPO_ROOT / "README.md"
README_START = "<!-- backlog-status:start -->"
README_END = "<!-- backlog-status:end -->"

ALLOWED_STATUSES = {
    "IMPLEMENTED",
    "PARTIAL",
    "OPEN",
    "BLOCKED",
    "SUPERSEDED",
    "OBSOLETE",
    "DUPLICATE",
}
STATUS_ORDER = (
    "OPEN",
    "PARTIAL",
    "BLOCKED",
    "IMPLEMENTED",
    "SUPERSEDED",
    "OBSOLETE",
    "DUPLICATE",
)
ALLOWED_VALIDATION = {
    "VALIDATED",
    "PENDING",
    "NOT_REQUIRED",
    "HARDWARE_REQUIRED",
    "HARDWARE_PENDING",
}
ALLOWED_SOURCE_TYPES = {"ISSUE", "PR", "TODO", "AUDIT", "FOLLOWUP"}
ALLOWED_EVIDENCE_STATES = {"CURRENT", "PARTIALLY_STALE", "SUPERSEDED"}
ALLOWED_MIGRATION_DISPOSITIONS = {
    "PORT",
    "ADAPT",
    "ALREADY_PRESENT",
    "SUPERSEDED",
    "REJECT",
}
ID_PATTERN = re.compile(r"^[A-Z][A-Z0-9]*(?:-[A-Z0-9]+)+$")


class ManifestError(ValueError):
    """Raised when the public backlog manifest violates its contract."""


@dataclass(frozen=True)
class Summary:
    baseline: int
    remaining: int
    resolved: int
    statuses: tuple[tuple[str, int], ...]
    scopes: tuple[tuple[str, int], ...]
    hardware_required: int
    hardware_pending: int
    total_sources: int
    tracked_issues: int
    tracked_prs: int
    open_issues: int
    open_prs: int
    canonical_findings: int
    duplicates: int


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = json.load(stream)
    validate_manifest(data)
    return data


def validate_manifest(data: dict[str, Any]) -> None:
    if data.get("schema_version") != 2:
        raise ManifestError("schema_version must be 2")
    findings = data.get("findings")
    sources = data.get("sources")
    if not isinstance(findings, list):
        raise ManifestError("findings must be a list")
    if not isinstance(sources, list):
        raise ManifestError("sources must be a list")
    if data.get("baseline_total") != len(findings):
        raise ManifestError("baseline_total must equal the canonical finding count")

    resolved_statuses = data.get("resolved_statuses")
    if not isinstance(resolved_statuses, list) or not resolved_statuses:
        raise ManifestError("resolved_statuses must be a non-empty list")
    unknown_resolved = set(resolved_statuses) - ALLOWED_STATUSES
    if unknown_resolved:
        raise ManifestError(f"unknown resolved statuses: {sorted(unknown_resolved)}")

    by_id: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(findings):
        if not isinstance(item, dict):
            raise ManifestError(f"finding {index} must be an object")
        item_id = item.get("id")
        if not isinstance(item_id, str) or not ID_PATTERN.fullmatch(item_id):
            raise ManifestError(f"finding {index} has invalid stable id")
        if item_id in by_id:
            raise ManifestError(f"duplicate stable id: {item_id}")
        by_id[item_id] = item

        for field in (
            "title",
            "scope",
            "priority",
            "analysis_summary",
            "evidence_state",
            "analyzed_at",
            "analyzed_head",
            "exact_next_step",
        ):
            if not isinstance(item.get(field), str) or not item[field].strip():
                raise ManifestError(f"{item_id} requires non-empty {field}")
        if item.get("status") not in ALLOWED_STATUSES:
            raise ManifestError(f"{item_id} has invalid status")
        if item["status"] == "DUPLICATE":
            raise ManifestError(f"{item_id} is canonical and may not be DUPLICATE")
        if item.get("validation") not in ALLOWED_VALIDATION:
            raise ManifestError(f"{item_id} has invalid validation")
        if item.get("evidence_state") not in ALLOWED_EVIDENCE_STATES:
            raise ManifestError(f"{item_id} has invalid evidence_state")
        for field in ("blocked_by", "dependencies"):
            if not isinstance(item.get(field), list) or not all(
                isinstance(value, str) for value in item[field]
            ):
                raise ManifestError(f"{item_id} requires a string list in {field}")

    for item_id, item in by_id.items():
        for field in ("blocked_by", "dependencies"):
            unknown = set(item[field]) - set(by_id)
            if unknown or item_id in item[field]:
                raise ManifestError(f"{item_id} has invalid {field}: {sorted(unknown)}")
        if item["status"] == "BLOCKED" and not item["blocked_by"]:
            raise ManifestError(f"{item_id} is BLOCKED without blocked_by")
        if item["status"] != "BLOCKED" and item["blocked_by"]:
            raise ManifestError(f"{item_id} sets blocked_by but is not BLOCKED")

    by_source_key: dict[str, dict[str, Any]] = {}
    source_count_by_finding: Counter[str] = Counter()
    for index, source in enumerate(sources):
        if not isinstance(source, dict):
            raise ManifestError(f"source {index} must be an object")
        source_key = source.get("source_key")
        if not isinstance(source_key, str) or not ID_PATTERN.fullmatch(source_key):
            raise ManifestError(f"source {index} has invalid source_key")
        if source_key in by_source_key:
            raise ManifestError(f"duplicate source_key: {source_key}")
        by_source_key[source_key] = source
        if source.get("source_type") not in ALLOWED_SOURCE_TYPES:
            raise ManifestError(f"{source_key} has invalid source_type")
        if not isinstance(source.get("source_id"), (str, int)):
            raise ManifestError(f"{source_key} requires source_id")
        for field in (
            "url",
            "title",
            "stable_id",
            "canonical_id",
            "scope",
            "priority",
            "analysis_summary",
            "evidence_state",
            "analyzed_at",
            "analyzed_head",
            "exact_next_step",
            "relation",
        ):
            if not isinstance(source.get(field), str) or not source[field].strip():
                raise ManifestError(f"{source_key} requires non-empty {field}")
        canonical_id = source["canonical_id"]
        if canonical_id not in by_id or source["stable_id"] != canonical_id:
            raise ManifestError(f"{source_key} requires an existing canonical_id")
        canonical_finding = by_id[canonical_id]
        for field in ("scope", "validation", "priority"):
            if source[field] != canonical_finding[field]:
                raise ManifestError(
                    f"{source_key} {field} differs from its canonical finding"
                )
        if source.get("checkpoint") is not None and not isinstance(
            source["checkpoint"], str
        ):
            raise ManifestError(f"{source_key} has invalid checkpoint")
        source_count_by_finding[canonical_id] += 1
        if source.get("status") not in ALLOWED_STATUSES:
            raise ManifestError(f"{source_key} has invalid status")
        if source.get("validation") not in ALLOWED_VALIDATION:
            raise ManifestError(f"{source_key} has invalid validation")
        if source.get("evidence_state") not in ALLOWED_EVIDENCE_STATES:
            raise ManifestError(f"{source_key} has invalid evidence_state")
        for field in ("blocked_by", "dependencies"):
            if not isinstance(source.get(field), list) or not all(
                isinstance(value, str) and value in by_id for value in source[field]
            ):
                raise ManifestError(f"{source_key} has invalid {field}")
        if source["source_type"] in {"ISSUE", "PR"}:
            if source.get("github_state") not in {"OPEN", "CLOSED"}:
                raise ManifestError(f"{source_key} has invalid github_state")
            if not isinstance(source["source_id"], int):
                raise ManifestError(f"{source_key} GitHub source_id must be numeric")
        if source["source_type"] == "PR":
            if source.get("migration_disposition") not in ALLOWED_MIGRATION_DISPOSITIONS:
                raise ManifestError(f"{source_key} has invalid migration_disposition")
            if source["github_state"] == "OPEN":
                for field in ("base_ref", "base_sha", "head_ref", "head_sha"):
                    if not isinstance(source.get(field), str) or not source[field]:
                        raise ManifestError(f"{source_key} requires non-empty {field}")

    unrepresented = set(by_id) - set(source_count_by_finding)
    if unrepresented:
        raise ManifestError(f"canonical findings without sources: {sorted(unrepresented)}")

    for source_key, source in by_source_key.items():
        canonical = source.get("duplicate_of")
        if source["status"] == "DUPLICATE":
            if not isinstance(canonical, str) or canonical not in by_source_key:
                raise ManifestError(f"{source_key} requires an existing duplicate_of")
            target = by_source_key[canonical]
            if (
                canonical == source_key
                or target["status"] == "DUPLICATE"
                or target["canonical_id"] != source["canonical_id"]
            ):
                raise ManifestError(f"{source_key} duplicate target must be canonical")
        elif canonical is not None:
            raise ManifestError(
                f"{source_key} may not set duplicate_of unless DUPLICATE"
            )

    inventory = data.get("github_inventory")
    if not isinstance(inventory, dict) or not inventory.get("snapshot_complete"):
        raise ManifestError("github_inventory must describe a complete snapshot")
    if inventory.get("total_sources") != inventory.get(
        "total_issues", 0
    ) + inventory.get("total_prs", 0):
        raise ManifestError("GitHub total source conservation failed")
    if inventory.get("closed_sources") != inventory.get(
        "expanded_related_closed_sources", 0
    ) + inventory.get("closed_sources_not_expanded", 0):
        raise ManifestError("GitHub closed source conservation failed")
    github_sources = [
        source for source in sources if source["source_type"] in {"ISSUE", "PR"}
    ]
    expanded_open = sum(source["github_state"] == "OPEN" for source in github_sources)
    expanded_closed = sum(source["github_state"] == "CLOSED" for source in github_sources)
    expected_counts = {
        "open_issues": sum(
            source["source_type"] == "ISSUE" and source["github_state"] == "OPEN"
            for source in github_sources
        ),
        "open_prs": sum(
            source["source_type"] == "PR" and source["github_state"] == "OPEN"
            for source in github_sources
        ),
        "expanded_open_sources": expanded_open,
        "expanded_related_closed_sources": expanded_closed,
    }
    for field, actual in expected_counts.items():
        if inventory.get(field) != actual:
            raise ManifestError(f"github_inventory {field} does not match sources")


def summarize(data: dict[str, Any]) -> Summary:
    items = data["findings"]
    sources = data["sources"]
    resolved_statuses = set(data["resolved_statuses"])
    statuses = Counter(item["status"] for item in items)
    scopes = Counter(item["scope"] for item in items)
    resolved = sum(count for status, count in statuses.items() if status in resolved_statuses)
    ordered_statuses = tuple(
        (status, statuses[status]) for status in STATUS_ORDER if statuses[status]
    )
    ordered_scopes = tuple(sorted(scopes.items(), key=lambda pair: (-pair[1], pair[0])))
    return Summary(
        baseline=len(items),
        remaining=len(items) - resolved,
        resolved=resolved,
        statuses=ordered_statuses,
        scopes=ordered_scopes,
        hardware_required=sum(
            item["validation"] == "HARDWARE_REQUIRED" for item in items
        ),
        hardware_pending=sum(
            item["validation"] == "HARDWARE_PENDING" for item in items
        ),
        total_sources=len(sources),
        tracked_issues=sum(source["source_type"] == "ISSUE" for source in sources),
        tracked_prs=sum(source["source_type"] == "PR" for source in sources),
        open_issues=sum(
            source["source_type"] == "ISSUE" and source["github_state"] == "OPEN"
            for source in sources
        ),
        open_prs=sum(
            source["source_type"] == "PR" and source["github_state"] == "OPEN"
            for source in sources
        ),
        canonical_findings=len(items),
        duplicates=sum(source["status"] == "DUPLICATE" for source in sources),
    )


def render_svg(data: dict[str, Any], summary: Summary) -> str:
    width = 900
    scope_columns = 2 if len(summary.scopes) > 4 else 1
    scope_rows = max(1, (len(summary.scopes) + scope_columns - 1) // scope_columns)
    scope_top = 334
    row_height = 30
    height = scope_top + scope_rows * row_height + 36
    column_width = 420 if scope_columns == 2 else 820
    max_scope = max((count for _, count in summary.scopes), default=1)

    status_text = "  ·  ".join(f"{status} {count}" for status, count in summary.statuses)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">',
        '  <title id="title">MowgliNext backlog status</title>',
        f'  <desc id="desc">{summary.canonical_findings} canonical findings from {summary.total_sources} tracked sources; baseline {summary.baseline}, remaining {summary.remaining}, resolved {summary.resolved}; counts by status and scope.</desc>',
        "  <style>",
        "    .bg{fill:#0f172a}.panel{fill:#172033}.muted{fill:#94a3b8}.text{fill:#e2e8f0}.accent{fill:#38bdf8}.good{fill:#34d399}.warn{fill:#fbbf24}",
        "    text{font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif}.title{font-size:24px;font-weight:700}.label{font-size:12px}.value{font-size:24px;font-weight:700}.small{font-size:13px}.scope{font-size:12px;font-weight:600}",
        "  </style>",
        f'  <rect class="bg" width="{width}" height="{height}" rx="14"/>',
        '  <text class="text title" x="32" y="40">MowgliNext backlog status</text>',
        f'  <text class="muted small" x="868" y="38" text-anchor="end">as of {escape(str(data.get("as_of", "")))}</text>',
    ]

    cards = (
        ("BASELINE", summary.baseline, "accent"),
        ("REMAINING", summary.remaining, "warn"),
        ("RESOLVED", summary.resolved, "good"),
    )
    for index, (label, value, css_class) in enumerate(cards):
        x = 32 + index * 188
        lines.extend(
            [
                f'  <rect class="panel" x="{x}" y="58" width="172" height="72" rx="10"/>',
                f'  <text class="muted label" x="{x + 16}" y="80">{label}</text>',
                f'  <text class="{css_class} value" x="{x + 16}" y="113">{value}</text>',
            ]
        )

    lines.extend(
        [
            '  <rect class="panel" x="596" y="58" width="272" height="72" rx="10"/>',
            '  <text class="muted label" x="612" y="80">HARDWARE VALIDATION</text>',
            f'  <text class="text small" x="612" y="104">Required <tspan class="warn">{summary.hardware_required}</tspan>  ·  Pending <tspan class="warn">{summary.hardware_pending}</tspan></text>',
            '  <text class="muted label" x="32" y="158">BY STATUS</text>',
            f'  <text class="text small" x="32" y="181">{escape(status_text)}</text>',
            '  <text class="muted label" x="32" y="213">TRACKED WORK SOURCES</text>',
        ]
    )

    source_cards = (
        ("SOURCES", summary.total_sources),
        ("ISSUES", f"{summary.tracked_issues} · {summary.open_issues} open"),
        ("PRS", f"{summary.tracked_prs} · {summary.open_prs} open"),
        ("CANONICAL", summary.canonical_findings),
        ("DUPLICATES", summary.duplicates),
    )
    for index, (label, value) in enumerate(source_cards):
        x = 32 + index * 168
        lines.extend(
            [
                f'  <rect class="panel" x="{x}" y="224" width="152" height="58" rx="8"/>',
                f'  <text class="muted label" x="{x + 12}" y="243">{label}</text>',
                f'  <text class="text small" x="{x + 12}" y="267" font-weight="700">{value}</text>',
            ]
        )

    lines.extend(
        [
            '  <line x1="32" y1="304" x2="868" y2="304" stroke="#334155"/>',
            '  <text class="text small" x="32" y="326" font-weight="700">By scope</text>',
        ]
    )

    for index, (scope, count) in enumerate(summary.scopes):
        column = index % scope_columns
        row = index // scope_columns
        x = 32 + column * column_width
        y = scope_top + row * row_height
        available_bar = 236 if scope_columns == 2 else 636
        bar_width = max(4, round(available_bar * count / max_scope))
        lines.extend(
            [
                f'  <text class="text scope" x="{x}" y="{y + 13}">{escape(scope)}</text>',
                f'  <rect fill="#243247" x="{x + 132}" y="{y + 3}" width="{available_bar}" height="12" rx="6"/>',
                f'  <rect class="accent" x="{x + 132}" y="{y + 3}" width="{bar_width}" height="12" rx="6"/>',
                f'  <text class="text scope" x="{x + 132 + available_bar + 10}" y="{y + 13}">{count}</text>',
            ]
        )

    lines.append("</svg>")
    return "\n".join(lines) + "\n"


def readme_block() -> str:
    return "\n".join(
        [
            README_START,
            "## Backlog status",
            "",
            "This dashboard is generated from the versioned backlog manifest at",
            "[`docs/status/mowgli_backlog.json`](docs/status/mowgli_backlog.json).",
            "",
            '<p align="center">',
            '  <img src="docs/status/mowgli_backlog.svg" alt="MowgliNext backlog status" width="900">',
            "</p>",
            "",
            "Regenerate with `python3 scripts/update_backlog_status.py`; verify committed",
            "artifacts with `python3 scripts/update_backlog_status.py --check`.",
            README_END,
        ]
    )


def render_readme(existing: str) -> str:
    block = readme_block()
    has_start = README_START in existing
    has_end = README_END in existing
    if has_start != has_end:
        raise ManifestError("README backlog markers are incomplete")
    if has_start:
        prefix, remainder = existing.split(README_START, 1)
        _, suffix = remainder.split(README_END, 1)
        return prefix + block + suffix

    insertion = "\n## Dashboard\n"
    if insertion not in existing:
        raise ManifestError("README insertion point '## Dashboard' not found")
    return existing.replace(insertion, "\n" + block + "\n\n## Dashboard\n", 1)


def update_or_check(path: Path, expected: str, check: bool) -> bool:
    actual = path.read_text(encoding="utf-8") if path.exists() else None
    if actual == expected:
        return True
    if check:
        print(f"out of date: {path}", file=sys.stderr)
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(expected, encoding="utf-8")
    return True


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if generated files differ")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--svg", type=Path, default=DEFAULT_SVG)
    parser.add_argument("--readme", type=Path, default=DEFAULT_README)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        data = load_manifest(args.manifest)
        summary = summarize(data)
        expected_svg = render_svg(data, summary)
        existing_readme = args.readme.read_text(encoding="utf-8")
        expected_readme = render_readme(existing_readme)
        results = (
            update_or_check(args.svg, expected_svg, args.check),
            update_or_check(args.readme, expected_readme, args.check),
        )
    except (OSError, json.JSONDecodeError, ManifestError) as error:
        print(f"backlog dashboard error: {error}", file=sys.stderr)
        return 2
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
