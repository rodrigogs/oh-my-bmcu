#!/usr/bin/env python3
"""Copy the platform pins of [env:base] from one platformio.ini into another.

Usage: apply_platform_pins.py SOURCE_INI TARGET_INI

CI rebuilds the upstream V10.5 sources with upstream's own platformio.ini, so fork changes
to build flags cannot be mistaken for toolchain drift; only `platform` and
`platform_packages` are taken from this branch. TARGET_INI is rewritten in place
(comments are dropped, which is fine for the throwaway CI copy).
"""

import configparser
import sys

PINNED = ("platform", "platform_packages")


def read(path):
    cfg = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(";",))
    cfg.optionxform = str
    cfg.read(path)
    return cfg


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    source, target = read(sys.argv[1]), read(sys.argv[2])
    for key in PINNED:
        if source.has_option("env:base", key):
            target.set("env:base", key, source.get("env:base", key))
            print("%s = %s" % (key, source.get("env:base", key).strip()))
        elif target.has_option("env:base", key):
            target.remove_option("env:base", key)
    with open(sys.argv[2], "w") as f:
        target.write(f)


if __name__ == "__main__":
    main()
