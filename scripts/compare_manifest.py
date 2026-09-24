#!/usr/bin/env python3
"""Check that every .bin listed in a freshly built manifest matches a reference manifest.

Usage: compare_manifest.py NEW_MANIFEST REFERENCE_MANIFEST [--expect-count N]

Both files use the build_all_firmwares.sh format: "SHA256 CRC32 SIZE REL_PATH" per line.
Exits non-zero if any built image is missing from the reference or has a different hash,
or, with --expect-count, if the number of built images is not exactly N.
"""

import argparse
import sys


def load(path):
    entries = {}
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            sha256, _crc32, _size, rel = line.rstrip("\n").split(" ", 3)
            entries[rel] = sha256
    return entries


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("built")
    ap.add_argument("reference")
    ap.add_argument("--expect-count", type=int)
    args = ap.parse_args()

    built, reference = load(args.built), load(args.reference)
    images = sorted(rel for rel in built if rel.endswith(".bin"))
    if not images:
        sys.exit("no .bin entries in %s" % args.built)

    bad = 0
    for rel in images:
        if reference.get(rel) == built[rel]:
            print("OK        %s" % rel)
        else:
            bad += 1
            print("MISMATCH  %s  built=%s reference=%s" % (rel, built[rel], reference.get(rel, "<missing>")))

    print("%d/%d images identical to the reference" % (len(images) - bad, len(images)))
    if args.expect_count is not None and len(images) != args.expect_count:
        print("expected %d built images, got %d" % (args.expect_count, len(images)))
        bad += 1
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
