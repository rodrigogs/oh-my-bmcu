# PlatformIO pre-script for env:fw.
#
# env:fw takes every variant option from environment variables (-DX=${sysenv.X}). An unset
# variable becomes an empty macro ("#if with no expression"), and a typo such as "ON" or a
# retract length in the wrong unit silently builds the wrong variant. Fail early instead.

Import("env")  # noqa: F821 (provided by PlatformIO/SCons)

import os
import re
import sys

errors = []

# Nothing is compiled for `-t clean` or IDE metadata dumps, so do not demand the variant there.
if env.IsCleanTarget() or env.IsIntegrationDump():  # noqa: F821
    Return()  # noqa: F821

for name in ("BMCU_DM_TWO_MICROSWITCH", "BMCU_ONLINE_LED_FILAMENT_RGB", "DBMCU_P1S", "BMCU_SOFT_LOAD"):
    value = os.environ.get(name)
    if value not in ("0", "1"):
        errors.append("%s must be 0 or 1 (got %r)" % (name, value))

ams_num = os.environ.get("BAMBU_BUS_AMS_NUM")
if ams_num not in ("0", "1", "2", "3"):
    errors.append("BAMBU_BUS_AMS_NUM must be 0..3 (got %r)" % ams_num)

retract = os.environ.get("AMS_RETRACT_LEN")
match = re.fullmatch(r"(\d+(?:\.\d+)?)f?", retract or "")
if not match or not (0.05 <= float(match.group(1)) <= 2.0):
    errors.append("AMS_RETRACT_LEN must be a length in metres from 0.05 to 2.0, e.g. 0.095f (got %r)" % retract)

if os.environ.get("DBMCU_P1S") == "1" and os.environ.get("BMCU_SOFT_LOAD") == "1":
    errors.append("DBMCU_P1S=1 and BMCU_SOFT_LOAD=1 select different load modes; set only one")

if errors:
    sys.stderr.write("env:fw: invalid variant environment:\n  - %s\n" % "\n  - ".join(errors))
    env.Exit(1)  # noqa: F821
