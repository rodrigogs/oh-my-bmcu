#!/usr/bin/env python3
"""Check that every .bin listed in a freshly built manifest matches a reference manifest.

Usage: compare_manifest.py NEW_MANIFEST REFERENCE_MANIFEST

Both files use the build_all_firmwares.sh format: "SHA256 CRC32 SIZE REL_PATH" per line.
Exits non-zero if any built image is missing from the reference or has a different hash.
"""

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
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    built, reference = load(sys.argv[1]), load(sys.argv[2])
    images = sorted(rel for rel in built if rel.endswith(".bin"))
    if not images:
        sys.exit("no .bin entries in %s" % sys.argv[1])

    bad = 0
    for rel in images:
        if reference.get(rel) == built[rel]:
            print("OK        %s" % rel)
        else:
            bad += 1
            print("MISMATCH  %s  built=%s reference=%s" % (rel, built[rel], reference.get(rel, "<missing>")))

    print("%d/%d images identical to the reference" % (len(images) - bad, len(images)))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
