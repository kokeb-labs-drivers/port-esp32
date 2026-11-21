#!/usr/bin/env python3
"""The pinned copy of kb-core in third_party/kb-core: refresh it from a kb-core checkout, or check it.

kb-core is a private repository of another organisation, so this repository's CI cannot fetch it. The port
therefore carries an unchanged copy of the kb-core files it builds against, at one recorded commit. Nothing in
third_party/kb-core is ever edited by hand: CMake re-hashes every file at configure time against
third_party/kb-core.lock (any difference stops the build), and this tool is the only writer.

    python3 tools/kb_core_snapshot.py --check                     # the copy equals the lock (CI and pre-flight)
    python3 tools/kb_core_snapshot.py --update /path/to/kb-core   # copy from a checkout at its HEAD commit

The lock records the source repository, the commit, the ref it was taken from and the SHA-256 of every file.
--update takes only a commit that kb-core has released or merged: a release tag (preferred), or a commit on
kb-core's main branch (`origin/main` of the checkout, fetched first). A commit that exists only on a pull-request
branch is refused, so the lock never names a state kb-core can delete. kb-core has no release tag yet; once it has,
the lock moves to the tag and later to the ESP-IDF component of kb-core (docs/soup.md, "kb-core").

SPDX-License-Identifier: MIT
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEST = ROOT / "third_party" / "kb-core"
LOCK = ROOT / "third_party" / "kb-core.lock"
REPO = "https://github.com/kokeb-labs/kb-core"
# What the port builds against when the checkout cannot say (kb-core before `tools/layout/layout.py vendor`): the
# CMake entry and modules, the parts it uses (utils, rtos, interfaces, dma with its host model), the RTOS-free and
# FreeRTOS backends, the host test port and the test harness. A checkout that has `layout.py vendor` is copied by
# that list instead, so the port takes exactly what kb-core says a vendoring consumer needs.
PATHS = (
    "LICENSE",
    "CMakeLists.txt",
    "cmake",
    "tools/layout/paths.yml",
    "components/lib/utils/CMakeLists.txt",
    "components/lib/utils/inc",
    "components/lib/rtos/CMakeLists.txt",
    "components/lib/rtos/inc",
    "components/lib/rtos/backends/none",
    "components/lib/rtos/backends/freertos",
    "components/lib/dma/CMakeLists.txt",
    "components/lib/dma/inc",
    "components/lib/dma/host",
    "components/lib/interfaces/CMakeLists.txt",
    "components/lib/interfaces/inc",
    "components/lib/interfaces/host",
    "components/ports/host",
    "tests/harness",
)
SKIP_NAMES = {"README.md"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vendor_paths(base: Path) -> tuple[str, ...]:
    """The paths `tools/layout/layout.py vendor` of the checkout prints, else PATHS."""
    tool = base / "tools" / "layout" / "layout.py"
    if not tool.is_file():
        return PATHS
    run = subprocess.run([sys.executable, "-I", str(tool), "vendor"], cwd=base, check=False, capture_output=True,
                         text=True)
    listed = tuple(line.strip() for line in run.stdout.splitlines() if line.strip())
    if run.returncode != 0 or not listed:
        raise SystemExit(f"kb_core_snapshot: `layout.py vendor` failed in {base}: {run.stderr.strip()}")
    return listed


def files_under(base: Path, paths: tuple[str, ...]) -> list[str]:
    out: list[str] = []
    for rel in paths:
        p = base / rel
        if p.is_file():
            out.append(rel)
        elif p.is_dir():
            out.extend(str(f.relative_to(base)) for f in sorted(p.rglob("*")) if f.is_file()
                       and f.name not in SKIP_NAMES)
        else:
            raise SystemExit(f"kb_core_snapshot: {rel} is missing in {base}")
    return sorted(out)


def read_lock() -> tuple[str, dict[str, str]]:
    commit = ""
    entries: dict[str, str] = {}
    for line in LOCK.read_text(encoding="utf-8").splitlines():
        if line.startswith("commit "):
            commit = line.split()[1]
        elif line and not line.startswith(("#", "repo ", "ref ")):
            digest, rel = line.split(None, 1)
            entries[rel] = digest
    return commit, entries


def check() -> int:
    commit, entries = read_lock()
    refs = [line for line in LOCK.read_text(encoding="utf-8").splitlines() if line.startswith("ref ")]
    if not refs or refs[0] not in ("ref main",) and not refs[0].startswith("ref tag "):
        print("third_party/kb-core.lock: no \"ref main\" or \"ref tag <name>\" line (re-run --update)")
        return 1
    present = sorted(str(f.relative_to(DEST)) for f in DEST.rglob("*") if f.is_file())
    bad = [rel for rel, digest in entries.items() if not (DEST / rel).is_file() or sha256(DEST / rel) != digest]
    extra = [rel for rel in present if rel not in entries]
    for rel in bad:
        print(f"third_party/kb-core/{rel}: differs from the lock (or is missing)")
    for rel in extra:
        print(f"third_party/kb-core/{rel}: not in the lock")
    if bad or extra:
        return 1
    print(f"kb-core copy: {len(entries)} files equal the lock at {commit[:12]}")
    return 0


def git(src: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["git", "-C", str(src), *args], check=check, capture_output=True, text=True)


def released_ref(src: Path, commit: str) -> str:
    """The tag at `commit`, else "main" when origin/main contains it; refuses anything else."""
    git(src, "fetch", "--quiet", "--tags", "origin", "main")
    tags = git(src, "tag", "--points-at", commit).stdout.split()
    if tags:
        return f"tag {sorted(tags)[-1]}"
    if git(src, "merge-base", "--is-ancestor", commit, "origin/main", check=False).returncode == 0:
        return "main"
    raise SystemExit(f"kb_core_snapshot: {commit[:12]} is neither tagged nor on kb-core's main branch "
                     "(a pull-request branch can disappear); check out a tag or a commit of origin/main")


def update(src: Path) -> int:
    commit = git(src, "rev-parse", "HEAD").stdout.strip()
    ref = released_ref(src, commit)
    paths = vendor_paths(src)
    dirty = subprocess.run(["git", "-C", str(src), "status", "--porcelain", "--", *paths], check=True,
                           capture_output=True, text=True).stdout.strip()
    if dirty:
        raise SystemExit("kb_core_snapshot: the checkout has local changes in the copied paths")
    rels = files_under(src, paths)
    if DEST.exists():
        shutil.rmtree(DEST)
    lines = ["# The kb-core files in third_party/kb-core, written by tools/kb_core_snapshot.py --update. Never edit.",
             f"repo {REPO}", f"commit {commit}", f"ref {ref}"]
    for rel in rels:
        (DEST / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src / rel, DEST / rel)
        lines.append(f"{sha256(DEST / rel)}  {rel}")
    LOCK.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"kb-core copy: {len(rels)} files at {commit[:12]} ({ref})")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--update", metavar="KB_CORE_CHECKOUT")
    a = ap.parse_args()
    if a.update:
        return update(Path(a.update).resolve())
    return check()


if __name__ == "__main__":
    sys.exit(main())
