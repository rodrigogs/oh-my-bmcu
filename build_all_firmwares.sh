#!/usr/bin/env bash
set -euo pipefail

# CDPATH= : an exported CDPATH must not send a relative cd to another directory of the same name.
CDPATH= cd -- "$(dirname "$0")"

command -v pio >/dev/null 2>&1 || { echo "ERROR: nie ma 'pio' w PATH"; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "ERROR: nie ma 'python3' w PATH"; exit 1; }

# oh-my-bmcu: firmwares/ is the upstream mirror (and CI's reference), so builds from this fork's
# sources go to an untracked dir by default. Set OUT_DIR=firmwares to regenerate the mirror.
OUT_DIR="${OUT_DIR:-build/firmwares}"
# Absolute and without a trailing slash, so the staging dir never ends up inside OUT_DIR.
OUT_DIR="$(python3 -c 'import os, sys; print(os.path.abspath(sys.argv[1]))' "${OUT_DIR}")"
PIO_ENV="fw"

# BUILD_ONLY_MIN=1 builds only the two 9.5 cm images that replaced V10.5's SOLO folder, AMS_AUTO and
# AMS_A (ams_auto_0.095f.bin, ams_a_0.095f.bin), for every mode/AUTOLOAD/RGB combination (24 builds).
BUILD_ONLY_MIN="${BUILD_ONLY_MIN:-0}"
MIN_RETRACT="0.095"
# FORCE=1 lets the run replace an OUT_DIR (or OUT_DIR.new / OUT_DIR.old) that holds something else.
FORCE="${FORCE:-0}"

# Refused loudly: with V11's matrix a stale BUILD_ONLY_SOLO=1 would otherwise start all 1020 builds.
if [[ -n "${BUILD_ONLY_SOLO:-}" ]]; then
  echo "ERROR: BUILD_ONLY_SOLO is gone (V11 has no SOLO folder); use BUILD_ONLY_MIN=1 (24 images)"
  exit 1
fi

# Build into a staging dir and only replace ${OUT_DIR} once every variant has been built,
# so a failed build no longer leaves the previous output deleted.
STAGE_DIR="${OUT_DIR}.new"
MANIFEST_TMP="${STAGE_DIR}.manifest.txt"
MANIFEST_HEADER="# format: SHA256_HEX CRC32_HEX SIZE_BYTES REL_PATH"

# The run deletes ${STAGE_DIR} and ${OUT_DIR}.old and everything in ${OUT_DIR}, so check what they
# are before anything is touched. Directories are compared by identity (-ef), not by name: through a
# symlinked checkout (/tmp is /private/tmp on macOS) or in another letter case on a case-insensitive
# file system the names differ.

# True if directory $1 is directory $2 or one above it.
is_or_above() {
  local d
  d="$(CDPATH= cd -P -- "$2" 2>/dev/null && pwd -P)" || return 1
  while :; do
    [[ "$1" -ef "${d}" ]] && return 0
    [[ "${d}" == "/" ]] && return 1
    d="${d%/*}"
    d="${d:-/}"
  done
}

# True if $1 is an empty directory, or an earlier output of this script: its manifest.txt starts
# with the header written below (the committed firmwares/ mirror has it too).
is_build_output() {
  local first
  [[ -d "$1" ]] || return 1
  first="$(ls -A "$1")" || return 1
  [[ -z "${first}" ]] && return 0
  [[ -f "$1/manifest.txt" ]] || return 1
  first="$(head -n 1 "$1/manifest.txt")" || return 1
  [[ "${first}" == "${MANIFEST_HEADER}" ]]
}

if is_or_above "${OUT_DIR}" .; then
  echo "ERROR: OUT_DIR=${OUT_DIR} is this checkout or a directory above it"
  exit 1
fi
if [[ -n "${HOME:-}" ]] && is_or_above "${OUT_DIR}" "${HOME}"; then
  echo "ERROR: OUT_DIR=${OUT_DIR} is \$HOME or a directory above it"
  exit 1
fi
if [[ "${BUILD_ONLY_MIN}" == "1" ]] && is_or_above "${OUT_DIR}" firmwares; then
  echo "ERROR: BUILD_ONLY_MIN=1 would replace firmwares/ with a 24-image subset; use another OUT_DIR"
  exit 1
fi
for d in "${OUT_DIR}" "${STAGE_DIR}" "${OUT_DIR}.old"; do
  [[ -e "${d}" || -L "${d}" ]] || continue
  is_build_output "${d}" && continue
  if [[ "${FORCE}" != "1" ]]; then
    if [[ -d "${d}" ]]; then
      echo "ERROR: ${d} is not empty and has no manifest.txt from this script, so it is not an"
      echo "       earlier build output; the run would delete it. Use another OUT_DIR, or FORCE=1."
    else
      echo "ERROR: ${d} exists and is not a directory; the run would delete it."
      echo "       Use another OUT_DIR, or FORCE=1."
    fi
    exit 1
  fi
done

trap 'rm -rf "${STAGE_DIR}" "${MANIFEST_TMP}"' EXIT

TXT_MODE="which_to_choose_mode.txt"
TXT_AUTOLOAD="which_to_choose_autoload.txt"
TXT_RGB="which_to_choose_filament_rgb.txt"
TXT_SLOTS="which_to_choose_slots.txt"
OUT_GUIDE="which_to_choose.txt"

[[ -f "${TXT_MODE}" ]]     || { echo "ERROR: brak ${TXT_MODE}"; exit 1; }
[[ -f "${TXT_AUTOLOAD}" ]] || { echo "ERROR: brak ${TXT_AUTOLOAD}"; exit 1; }
[[ -f "${TXT_RGB}" ]]      || { echo "ERROR: brak ${TXT_RGB}"; exit 1; }
[[ -f "${TXT_SLOTS}" ]]    || { echo "ERROR: brak ${TXT_SLOTS}"; exit 1; }

RETRACTS=(
  "0.095"
  "0.10"
  "0.20" "0.25" "0.30" "0.35" "0.40"
  "0.45" "0.50" "0.55" "0.60" "0.65"
  "0.70" "0.75" "0.80" "0.85" "0.90"
)

# dir|P1S|SOFT_LOAD
MODES=(
  "soft_load(A1)|0|1"
  "standard(A1)|0|0"
  "high_force_load(P1S)|1|0"
)

build_and_copy() {
  local out_path="$1"
  local ams_num="$2"
  local retract_len="$3"
  local dm="$4"
  local rgb="$5"
  local p1s="$6"
  local soft_load="$7"

  echo "=== BUILD: P1S=${p1s} SOFT_LOAD=${soft_load} DM=${dm} RGB=${rgb} AMS_NUM=${ams_num} RETRACT=${retract_len} -> ${out_path}"

  BAMBU_BUS_AMS_NUM="${ams_num}" \
  AMS_RETRACT_LEN="${retract_len}" \
  BMCU_DM_TWO_MICROSWITCH="${dm}" \
  BMCU_ONLINE_LED_FILAMENT_RGB="${rgb}" \
  DBMCU_P1S="${p1s}" \
  BMCU_SOFT_LOAD="${soft_load}" \
  pio run -e "${PIO_ENV}"

  local src=".pio/build/${PIO_ENV}/firmware.bin"
  [[ -f "${src}" ]] || { echo "ERROR: brak ${src}"; exit 1; }

  mkdir -p "$(dirname "${out_path}")"
  cp -f "${src}" "${out_path}"
}

rm -rf "${STAGE_DIR}"
mkdir -p "$(dirname "${OUT_DIR}")" "${STAGE_DIR}"

cp -f "${TXT_MODE}" "${STAGE_DIR}/${OUT_GUIDE}"

for mode_spec in "${MODES[@]}"; do
  IFS='|' read -r mode_dir p1s soft_load <<< "${mode_spec}"

  mode_base="${STAGE_DIR}/${mode_dir}"
  mkdir -p "${mode_base}"
  cp -f "${TXT_AUTOLOAD}" "${mode_base}/${OUT_GUIDE}"

  for dm in 1 0; do
    if [[ "${dm}" == "1" ]]; then
      dm_dir="AUTOLOAD"
    else
      dm_dir="NO_AUTOLOAD"
    fi

    dm_base="${mode_base}/${dm_dir}"
    mkdir -p "${dm_base}"
    cp -f "${TXT_RGB}" "${dm_base}/${OUT_GUIDE}"

    for rgb in 1 0; do
      if [[ "${rgb}" == "1" ]]; then
        rgb_dir="FILAMENT_RGB_ON"
      else
        rgb_dir="FILAMENT_RGB_OFF"
      fi

      base="${dm_base}/${rgb_dir}"
      mkdir -p "${base}"/{AMS_AUTO,AMS_A,AMS_B,AMS_C,AMS_D}
      cp -f "${TXT_SLOTS}" "${base}/${OUT_GUIDE}"

      for slot in AUTO A B C D; do
        if [[ "${BUILD_ONLY_MIN}" == "1" && "${slot}" != "AUTO" && "${slot}" != "A" ]]; then
          continue
        fi
        case "${slot}" in
          AUTO) ams_num=4; slot_dir="AMS_AUTO"; file_prefix="ams_auto" ;;
          A)    ams_num=0; slot_dir="AMS_A";    file_prefix="ams_a" ;;
          B)    ams_num=1; slot_dir="AMS_B";    file_prefix="ams_b" ;;
          C)    ams_num=2; slot_dir="AMS_C";    file_prefix="ams_c" ;;
          D)    ams_num=3; slot_dir="AMS_D";    file_prefix="ams_d" ;;
        esac

        for r in "${RETRACTS[@]}"; do
          if [[ "${BUILD_ONLY_MIN}" == "1" && "${r}" != "${MIN_RETRACT}" ]]; then
            continue
          fi
          build_and_copy \
            "${base}/${slot_dir}/${file_prefix}_${r}f.bin" \
            "${ams_num}" \
            "${r}f" \
            "${dm}" \
            "${rgb}" \
            "${p1s}" \
            "${soft_load}"
        done
      done
    done
  done
done

# Written outside the tree first: redirecting straight into ${OUT_DIR}/manifest.txt truncated it
# before the walk, so the manifest listed itself as an empty file.
python3 - "${STAGE_DIR}" "${MANIFEST_HEADER}" > "${MANIFEST_TMP}" <<'PYMAN'
import sys, os, zlib, hashlib

root = sys.argv[1]
entries = []

for dirpath, _, filenames in os.walk(root):
    for fn in filenames:
        p = os.path.join(dirpath, fn)
        rel = os.path.relpath(p, root).replace(os.sep, "/")

        crc = 0
        size = 0
        h = hashlib.sha256()

        with open(p, "rb") as f:
            while True:
                b = f.read(1024 * 1024)
                if not b:
                    break
                size += len(b)
                crc = zlib.crc32(b, crc)
                h.update(b)

        entries.append((rel, h.hexdigest(), f"{crc & 0xffffffff:08X}", size))

entries.sort(key=lambda x: x[0])

out = sys.stdout
out.write(sys.argv[2] + "\n")
for rel, sha256_hex, crc32_hex, size in entries:
    out.write(f"{sha256_hex} {crc32_hex} {size} {rel}\n")
PYMAN

mv "${MANIFEST_TMP}" "${STAGE_DIR}/manifest.txt"

# Move the previous output aside instead of deleting it first, so a failure here cannot lose
# both the old tree and the fresh build. The finished build is no longer the trap's to delete.
rm -rf "${OUT_DIR}.old"
[[ -e "${OUT_DIR}" ]] && mv "${OUT_DIR}" "${OUT_DIR}.old"
mv "${STAGE_DIR}" "${OUT_DIR}"
trap - EXIT
rm -rf "${OUT_DIR}.old"

echo
echo "DONE. Wyniki w: ${OUT_DIR}/"
echo "Manifest: ${OUT_DIR}/manifest.txt"
