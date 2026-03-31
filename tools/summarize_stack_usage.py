#!/usr/bin/env python3

from __future__ import annotations

import argparse
import pathlib
import re
import sys


STACK_LINE_RE = re.compile(
    r"^(?P<file>.*?):(?P<line>\d+):(?P<column>\d+):(?P<function>[^\t]+)\t(?P<bytes>\d+)\t(?P<kind>.+)$"
)


def iter_su_files(paths: list[str]) -> list[pathlib.Path]:
    su_files: list[pathlib.Path] = []
    for raw_path in paths:
        path = pathlib.Path(raw_path)
        if path.is_dir():
            su_files.extend(sorted(path.rglob("*.su")))
        elif path.is_file() and path.suffix == ".su":
            su_files.append(path)
    return sorted(set(su_files))


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize GCC .su stack usage files.")
    parser.add_argument("paths", nargs="+", help="Files or directories to scan")
    parser.add_argument("--top", type=int, default=20, help="Number of largest frames to print")
    args = parser.parse_args()

    entries: list[tuple[int, str, int, int, str, str]] = []

    for su_file in iter_su_files(args.paths):
        for raw_line in su_file.read_text(encoding="utf-8").splitlines():
            match = STACK_LINE_RE.match(raw_line.strip())
            if match is None:
                continue

            entries.append(
                (
                    int(match.group("bytes")),
                    match.group("file"),
                    int(match.group("line")),
                    int(match.group("column")),
                    match.group("function"),
                    match.group("kind"),
                )
            )

    if not entries:
        print("No .su entries found.", file=sys.stderr)
        return 1

    entries.sort(key=lambda entry: (-entry[0], entry[1], entry[2], entry[4]))

    total_entries = len(entries)
    max_bytes = entries[0][0]
    print(f"Parsed {total_entries} stack-usage entries")
    print(f"Largest single frame: {max_bytes} bytes")
    print("")
    print(f"Top {min(args.top, total_entries)} frames:")

    for stack_bytes, file_path, line, column, function, kind in entries[: args.top]:
        print(f"{stack_bytes:5d}  {function}  {file_path}:{line}:{column}  {kind}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
