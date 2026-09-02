"""Extract package release notes from CHANGELOG.md given a release tag `<package>/<version>`."""
# used by: .github/workflows/release.yml.

from __future__ import annotations

import argparse
from pathlib import Path
from typing import NamedTuple


class Release(NamedTuple):
    package: str
    version: str


def parse_release_tag(tag: str) -> Release:
    parts = tag.split("/")
    if len(parts) != 2 or not all(parts):
        raise ValueError(f"invalid release tag: {tag!r}")
    return Release(*parts)


def extract_release_notes(changelog: str, package: str, version: str) -> str:
    section = changelog.partition(f"## `{package}` v{version}\n")[2]
    return section.partition("\n## `")[0].strip()


_CHANGELOG = Path(__file__).resolve().parent.parent / "CHANGELOG.md"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("release")
    args = parser.parse_args()

    try:
        release = parse_release_tag(args.release)
    except ValueError as exc:
        parser.error(str(exc))

    if notes := extract_release_notes(_CHANGELOG.read_text(), release.package, release.version):
        print(notes)


if __name__ == "__main__":
    main()
