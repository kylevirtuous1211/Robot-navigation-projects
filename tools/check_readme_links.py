#!/usr/bin/env python3
"""Check that every relative image and link in the repo's Markdown files resolves.

A target passes only if it exists on disk AND is tracked by git, so a README never
points at a file that is ignored and therefore missing on GitHub.

Usage: python3 tools/check_readme_links.py [markdown files...]
       (default: every tracked README.md and report.md)
"""

import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote

REPO_ROOT = Path(__file__).resolve().parent.parent
# Top-level docs we curate; vendored upstream READMEs under workspace/ are skipped.
DEFAULT_GLOBS = ["README.md", "*/README.md", "*/report.md", "*/*/report.md", "*/*/README.md"]

MARKDOWN_TARGET = re.compile(r"!?\[[^\]]*\]\(\s*<?([^)\s>]+)>?(?:\s+\"[^\"]*\")?\s*\)")
HTML_TARGET = re.compile(r"""<(?:img|a|source|video)\b[^>]*?\b(?:src|href)\s*=\s*["']([^"']+)["']""", re.IGNORECASE)
FENCED_CODE = re.compile(r"^(```|~~~).*?^\1", re.MULTILINE | re.DOTALL)


def tracked_files() -> set[str]:
    listing = subprocess.run(
        ["git", "ls-files", "-z"], cwd=REPO_ROOT, check=True, capture_output=True, text=True
    ).stdout
    return {path for path in listing.split("\0") if path}


def default_markdown_files(tracked: set[str]) -> list[Path]:
    files = set()
    for pattern in DEFAULT_GLOBS:
        for path in REPO_ROOT.glob(pattern):
            if path.relative_to(REPO_ROOT).as_posix() in tracked:
                files.add(path)
    return sorted(files)


def is_external(target: str) -> bool:
    return bool(re.match(r"^[a-z][a-z0-9+.-]*:", target, re.IGNORECASE)) or target.startswith("#")


def broken_targets(markdown_path: Path, tracked: set[str]) -> list[str]:
    text = FENCED_CODE.sub("", markdown_path.read_text(encoding="utf-8"))
    targets = MARKDOWN_TARGET.findall(text) + HTML_TARGET.findall(text)
    problems = []
    for target in targets:
        if is_external(target):
            continue
        relative = unquote(target.split("#", 1)[0].split("?", 1)[0])
        if not relative:
            continue
        resolved = (markdown_path.parent / relative).resolve()
        try:
            repo_path = resolved.relative_to(REPO_ROOT).as_posix()
        except ValueError:
            problems.append(f"{target} (points outside the repo)")
            continue
        if not resolved.exists():
            problems.append(f"{target} (missing)")
        elif resolved.is_file() and repo_path not in tracked:
            problems.append(f"{target} (exists but is not tracked by git)")
        elif resolved.is_dir() and not any(path.startswith(repo_path + "/") for path in tracked):
            problems.append(f"{target} (directory has no tracked files)")
    return problems


def main() -> int:
    tracked = tracked_files()
    if len(sys.argv) > 1:
        markdown_files = [Path(argument).resolve() for argument in sys.argv[1:]]
    else:
        markdown_files = default_markdown_files(tracked)

    failure_count = 0
    for markdown_path in markdown_files:
        for problem in broken_targets(markdown_path, tracked):
            print(f"{markdown_path.relative_to(REPO_ROOT)}: {problem}")
            failure_count += 1

    print(f"Checked {len(markdown_files)} Markdown files, {failure_count} broken link(s).")
    return 1 if failure_count else 0


if __name__ == "__main__":
    sys.exit(main())
