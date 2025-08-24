# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

import argparse, sys, os, io, re, datetime

SPDX = "MIT"
HEADER_TEMPLATE = """# SPDX-License-Identifier: {spdx}
# Copyright (c) {year} {author}
"""

def has_header(text):
    head = text[:500].lower()
    return ("spdx-license-identifier" in head) or ("mit license" in head) or ("copyright (c)" in head)

def insert_header(text, header):
    lines = text.splitlines(keepends=True)
    i = 0
    if i < len(lines) and lines[i].startswith("#!"): i += 1
    if i < len(lines) and re.match(r"#.*coding[:=]\s*[-\w.]+", lines[i]): i += 1
    ins = header if header.endswith("\n") else header + "\n"
    if i < len(lines) and lines[i].strip(): ins += "\n"
    return "".join(lines[:i]) + ins + "".join(lines[i:])

def process(path, header, dry=False):
    try:
        with io.open(path, "r", encoding="utf-8") as f:
            original = f.read()
    except Exception:
        return "skip"
    if has_header(original): return "exists"
    updated = insert_header(original, header)
    if not dry:
        with io.open(path, "w", encoding="utf-8", newline="") as f:
            f.write(updated)
    return "added"

def main():
    ap = argparse.ArgumentParser(description="Add SPDX + copyright header to Python files.")
    ap.add_argument("--author", required=True)
    ap.add_argument("--year", default=str(datetime.date.today().year))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("root")
    args = ap.parse_args()

    header = HEADER_TEMPLATE.format(spdx=SPDX, year=args.year, author=args.author).rstrip() + "\n"
    added = exist = skip = 0
    for dp, dns, fns in os.walk(args.root):
        dns[:] = [d for d in dns if d not in (".git",".venv","venv","__pycache__",".mypy_cache",".pytest_cache")]
        for fn in fns:
            if fn.endswith(".py"):
                r = process(os.path.join(dp, fn), header, args.dry_run)
                if r=="added": added+=1
                elif r=="exists": exist+=1
                else: skip+=1
    print(f"Added: {added}, already had header: {exist}, skipped: {skip}")

if __name__ == "__main__":
    sys.exit(main())
