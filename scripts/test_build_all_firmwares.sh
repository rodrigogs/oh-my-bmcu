#!/usr/bin/env bash
# Checks the OUT_DIR guards of build_all_firmwares.sh with a stub pio (no toolchain, a few seconds):
# every refused OUT_DIR must fail before the first build and leave every file where it was, and the
# accepted ones must still build. Runs a copy of the script in a temporary tree, also through a
# symlinked path. Usage: bash scripts/test_build_all_firmwares.sh (macOS /bin/bash 3.2 works too).
set -euo pipefail

repo="$(CDPATH= cd -- "$(dirname "$0")/.." && pwd -P)"
base="$(mktemp -d)"
logs="$(mktemp -d)"
trap 'rm -rf "${base}" "${logs}"' EXIT
base="$(cd "${base}" && pwd -P)"

tree="${base}/repo"   # the checkout the script runs in
link="${base}/link"   # a symlink to it, like /tmp -> /private/tmp on macOS
mkdir -p "${tree}/firmwares/SOLO" "${base}/bin" "${base}/home/user"
# Decoys for an exported CDPATH: a relative cd to firmwares or repo must not end up here.
mkdir -p "${base}/cdpath/firmwares" "${base}/cdpath/repo"
cp "${repo}/build_all_firmwares.sh" "${repo}"/which_to_choose_*.txt "${tree}/"
ln -s "${tree}" "${link}"
# A small stand-in for the upstream mirror, with the header of the real one.
head -n 1 "${repo}/firmwares/manifest.txt" > "${tree}/firmwares/manifest.txt"
echo "mirror image" > "${tree}/firmwares/SOLO/solo_0.095f.bin"
echo "0 0 12 SOLO/solo_0.095f.bin" >> "${tree}/firmwares/manifest.txt"

# The stub logs each call; with STUB_PIO_FAIL=1 the first build fails.
cat > "${base}/bin/pio" <<'EOF'
#!/usr/bin/env bash
echo "$*" >> "${STUB_PIO_LOG}"
[[ "${STUB_PIO_FAIL:-0}" == "1" ]] && exit 7
mkdir -p .pio/build/fw
echo "${DBMCU_P1S} ${BMCU_SOFT_LOAD} ${BMCU_DM_TWO_MICROSWITCH} ${BMCU_ONLINE_LED_FILAMENT_RGB} ${BAMBU_BUS_AMS_NUM} ${AMS_RETRACT_LEN}" \
  > .pio/build/fw/firmware.bin
EOF
chmod +x "${base}/bin/pio"
export PATH="${base}/bin:${PATH}"
export STUB_PIO_LOG="${logs}/pio.log"
export HOME="${base}/home/user"

failures=0
fail() { echo "FAIL  $*"; failures=$((failures + 1)); }

# Every path under ${base} with the checksum of every file (the logs live outside it).
snapshot() {
  (cd "${base}" && find . | LC_ALL=C sort && find . -type f -exec cksum {} + | LC_ALL=C sort)
}

# run SCRIPT_DIR ENV... : runs the script copy from SCRIPT_DIR with the given environment.
run() {
  local dir="$1"
  shift
  : > "${STUB_PIO_LOG}"
  (cd "${base}" && env -u OUT_DIR -u BUILD_ONLY_SOLO -u FORCE -u CDPATH "$@" bash "${dir}/build_all_firmwares.sh") \
    > "${logs}/out.txt" 2>&1
}

# refused NAME SCRIPT_DIR ENV... : must exit with an ERROR before the first build, touching nothing.
refused() {
  local name="$1"
  shift
  local before after rc=0
  before="$(snapshot)"
  run "$@" || rc=$?
  after="$(snapshot)"
  if [[ ${rc} -eq 0 ]]; then fail "${name}: accepted"; return; fi
  grep -q '^ERROR: ' "${logs}/out.txt" || { fail "${name}: failed without an ERROR line"; cat "${logs}/out.txt"; return; }
  [[ -s "${STUB_PIO_LOG}" ]] && { fail "${name}: pio ran"; return; }
  [[ "${before}" == "${after}" ]] || { fail "${name}: files changed"; return; }
  echo "ok    ${name}: $(grep -m 1 '^ERROR: ' "${logs}/out.txt")"
}

# built NAME OUT_DIR SCRIPT_DIR ENV... : must build the 12 SOLO images into OUT_DIR, mirror untouched.
built() {
  local name="$1" out="$2"
  shift 2
  local mirror_before rc=0
  mirror_before="$(cd "${tree}/firmwares" && find . -type f -exec cksum {} + | LC_ALL=C sort)"
  run "$@" || rc=$?
  if [[ ${rc} -ne 0 ]]; then fail "${name}: exit ${rc}"; cat "${logs}/out.txt"; return; fi
  [[ "$(wc -l < "${STUB_PIO_LOG}")" -eq 12 ]] || { fail "${name}: $(wc -l < "${STUB_PIO_LOG}") builds, not 12"; return; }
  [[ "$(grep -c '\.bin$' "${out}/manifest.txt")" -eq 12 ]] || { fail "${name}: manifest does not list 12 images"; return; }
  [[ -e "${out}.new" || -e "${out}.old" || -e "${out}.new.manifest.txt" ]] && { fail "${name}: staging left behind"; return; }
  [[ "$(cd "${tree}/firmwares" && find . -type f -exec cksum {} + | LC_ALL=C sort)" == "${mirror_before}" ]] \
    || { fail "${name}: firmwares/ changed"; return; }
  echo "ok    ${name}: 12 images in ${out#${base}/}"
}

solo="BUILD_ONLY_SOLO=1"

# The upstream mirror with BUILD_ONLY_SOLO=1, however it is spelled or reached.
refused "SOLO into firmwares" "${tree}" "${solo}" OUT_DIR=firmwares
refused "SOLO into firmwares/" "${tree}" "${solo}" OUT_DIR=firmwares/
refused "SOLO into ./firmwares" "${tree}" "${solo}" OUT_DIR=./firmwares
refused "SOLO into the absolute firmwares" "${tree}" "${solo}" OUT_DIR="${tree}/firmwares"
refused "SOLO into firmwares through a symlinked checkout" "${link}" "${solo}" OUT_DIR=firmwares
refused "SOLO into link/firmwares" "${tree}" "${solo}" OUT_DIR="${link}/firmwares"
if [[ "${tree}/Firmwares" -ef "${tree}/firmwares" ]]; then
  refused "SOLO into Firmwares (case-insensitive file system)" "${tree}" "${solo}" OUT_DIR=Firmwares
fi
refused "SOLO with FORCE=1 into firmwares" "${tree}" "${solo}" FORCE=1 OUT_DIR=firmwares
refused "SOLO into firmwares with CDPATH set" "${tree}" CDPATH="${base}/cdpath" "${solo}" OUT_DIR=firmwares

# The checkout, the directory above it, \$HOME and the one above it, each also with FORCE=1 (the
# checkout also with BUILD_ONLY_SOLO=1). Without FORCE the non-empty check alone would refuse a
# directory above either; with FORCE=1 only the walk up to / does.
refused "the checkout (.)" "${tree}" OUT_DIR=.
refused "the checkout, SOLO" "${tree}" "${solo}" OUT_DIR=.
refused "the checkout by absolute path" "${tree}" OUT_DIR="${tree}"
refused "the checkout through the symlink" "${tree}" OUT_DIR="${link}"
refused "the checkout from the symlinked path" "${link}" OUT_DIR=.
refused "the checkout with FORCE=1" "${tree}" FORCE=1 OUT_DIR=.
refused "the parent of the checkout" "${tree}" OUT_DIR=..
refused "\$HOME" "${tree}" OUT_DIR="${HOME}"
refused "\$HOME with FORCE=1" "${tree}" FORCE=1 OUT_DIR="${HOME}"
refused "the parent of \$HOME" "${tree}" OUT_DIR="${base}/home"
refused "the parent of the checkout with FORCE=1" "${tree}" FORCE=1 OUT_DIR=..
refused "the parent of \$HOME with FORCE=1" "${tree}" FORCE=1 OUT_DIR="${base}/home"

# Something that is not an earlier build output, as OUT_DIR, OUT_DIR.new or OUT_DIR.old.
mkdir -p "${base}/other" && echo keep > "${base}/other/notes.txt"
refused "a non-empty unrelated dir" "${tree}" "${solo}" OUT_DIR="${base}/other"
mkdir -p "${base}/other/sub" && echo "# format: something else" > "${base}/other/manifest.txt"
refused "a dir whose manifest.txt is not ours" "${tree}" "${solo}" OUT_DIR="${base}/other"
rm -rf "${base}/other/sub" "${base}/other/manifest.txt"
mkdir -p "${base}/out.new" && echo keep > "${base}/out.new/notes.txt"
refused "a non-empty unrelated OUT_DIR.new" "${tree}" "${solo}" OUT_DIR="${base}/out"
mv "${base}/out.new" "${base}/out.old"
refused "a non-empty unrelated OUT_DIR.old" "${tree}" "${solo}" OUT_DIR="${base}/out"
rm -rf "${base}/out.old"
echo keep > "${base}/afile"
refused "a regular file" "${tree}" "${solo}" OUT_DIR="${base}/afile"
rm -f "${base}/afile"
ln -s "${base}/nonexistent" "${base}/dangling"
refused "a dangling symlink" "${tree}" "${solo}" OUT_DIR="${base}/dangling"
rm -f "${base}/dangling"

# Accepted: the default, a rebuild over an earlier output, the symlinked path, a relative script path
# with CDPATH set, an empty dir, FORCE=1.
built "default build/firmwares" "${tree}/build/firmwares" "${tree}" "${solo}"
built "rebuild over the earlier output" "${tree}/build/firmwares" "${tree}" "${solo}"
built "default from the symlinked path" "${tree}/build/firmwares" "${link}" "${solo}"
# Run by a relative path, the script's own cd must not follow CDPATH to the decoy repo (empty, so it
# would stop there with an ERROR).
built "default, run as repo/build_all_firmwares.sh with CDPATH set" "${tree}/build/firmwares" repo \
  CDPATH="${base}/cdpath" "${solo}"
mkdir -p "${base}/empty"
built "an empty dir" "${base}/empty" "${tree}" "${solo}" OUT_DIR="${base}/empty"
built "a non-empty unrelated dir with FORCE=1" "${base}/other" "${tree}" "${solo}" FORCE=1 OUT_DIR="${base}/other"
[[ -e "${base}/other/notes.txt" ]] && fail "FORCE=1: the old contents are still there"

# OUT_DIR=firmwares without BUILD_ONLY_SOLO regenerates the mirror: the guards let it through to
# the first build (which the stub fails here, so the mirror is kept and nothing is left behind).
before="$(snapshot)"
rc=0
run "${tree}" STUB_PIO_FAIL=1 OUT_DIR=firmwares || rc=$?
after="$(snapshot)"
if [[ ${rc} -eq 0 ]] || grep -q '^ERROR: ' "${logs}/out.txt" || [[ "$(wc -l < "${STUB_PIO_LOG}")" -ne 1 ]]; then
  fail "full build into firmwares: did not reach the first build"
elif [[ "${before}" != "${after}" ]]; then
  fail "full build into firmwares: a failed build changed files"
else
  echo "ok    full build into firmwares: reaches the first build; a failed build keeps the mirror"
fi

if [[ ${failures} -ne 0 ]]; then
  echo "${failures} check(s) failed"
  exit 1
fi
echo "all build_all_firmwares.sh OUT_DIR checks passed"
