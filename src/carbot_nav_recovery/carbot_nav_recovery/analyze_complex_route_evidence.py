"""Command-line entry point for Issue #12 evidence summaries."""

import argparse
import json
from pathlib import Path

from .evidence_analysis import summarize_advisories


def _read(path):
    text = Path(path).read_text(encoding='utf-8')
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        value = [json.loads(line) for line in text.splitlines()
                 if line.strip()]
    return value if isinstance(value, list) else [value]


def main(args=None):
    """Load evidence files and print one deterministic JSON summary."""
    parser = argparse.ArgumentParser()
    parser.add_argument('evidence', nargs='+')
    parsed = parser.parse_args(args)
    records = []
    for path in parsed.evidence:
        records.extend(_read(path))
    print(json.dumps(summarize_advisories(records), indent=2,
                     sort_keys=True, allow_nan=False))
