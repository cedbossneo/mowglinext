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


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = json.load(stream)
    validate_manifest(data)
    return data


def validate_manifest(data: dict[str, Any]) -> None:
    if data.get("schema_version") != 1:
        raise ManifestError("schema_version must be 1")
    items = data.get("items")
    if not isinstance(items, list):
        raise ManifestError("items must be a list")
    if data.get("baseline_total") != len(items):
        raise ManifestError("baseline_total must equal the expanded item count")

    resolved_statuses = data.get("resolved_statuses")
    if not isinstance(resolved_statuses, list) or not resolved_statuses:
        raise ManifestError("resolved_statuses must be a non-empty list")
    unknown_resolved = set(resolved_statuses) - ALLOWED_STATUSES
    if unknown_resolved:
        raise ManifestError(f"unknown resolved statuses: {sorted(unknown_resolved)}")

    by_id: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise ManifestError(f"item {index} must be an object")
        item_id = item.get("id")
        if not isinstance(item_id, str) or not ID_PATTERN.fullmatch(item_id):
            raise ManifestError(f"item {index} has invalid stable id")
        if item_id in by_id:
            raise ManifestError(f"duplicate stable id: {item_id}")
        by_id[item_id] = item

        for field in ("title", "scope", "source"):
            if not isinstance(item.get(field), str) or not item[field].strip():
                raise ManifestError(f"{item_id} requires non-empty {field}")
        if item.get("status") not in ALLOWED_STATUSES:
            raise ManifestError(f"{item_id} has invalid status")
        if item.get("validation") not in ALLOWED_VALIDATION:
            raise ManifestError(f"{item_id} has invalid validation")

    for item_id, item in by_id.items():
        canonical = item.get("duplicate_of")
        if item["status"] == "DUPLICATE":
            if not isinstance(canonical, str) or canonical not in by_id:
                raise ManifestError(f"{item_id} requires an existing duplicate_of")
            if canonical == item_id or by_id[canonical]["status"] == "DUPLICATE":
                raise ManifestError(f"{item_id} duplicate target must be canonical")
        elif canonical is not None:
            raise ManifestError(f"{item_id} may not set duplicate_of unless DUPLICATE")


def summarize(data: dict[str, Any]) -> Summary:
    items = data["items"]
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
    )


def render_svg(data: dict[str, Any], summary: Summary) -> str:
    width = 900
    scope_columns = 2 if len(summary.scopes) > 4 else 1
    scope_rows = max(1, (len(summary.scopes) + scope_columns - 1) // scope_columns)
    scope_top = 252
    row_height = 30
    height = scope_top + scope_rows * row_height + 36
    column_width = 420 if scope_columns == 2 else 820
    max_scope = max((count for _, count in summary.scopes), default=1)

    status_text = "  ·  ".join(f"{status} {count}" for status, count in summary.statuses)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">',
        '  <title id="title">MowgliNext backlog status</title>',
        f'  <desc id="desc">Baseline {summary.baseline}, remaining {summary.remaining}, resolved {summary.resolved}; backlog counts by status and scope.</desc>',
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
            '  <line x1="32" y1="202" x2="868" y2="202" stroke="#334155"/>',
            '  <text class="text small" x="32" y="231" font-weight="700">By scope</text>',
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
