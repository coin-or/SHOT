#!/bin/bash
# Build Cbc and Ipopt (with MUMPS) with coinbrew into the prefix given as the first argument, at the
# releases in ThirdParty/versions.json. Run by cibuildwheel before the wheels are built; the libraries
# are built here instead of taken from a package manager so that the wheels contain the same versions
# on every platform, built for the wheel's platform tag (manylinux_2_28 or the macOS deployment target).
set -euo pipefail

prefix="$1"
root="$(cd "$(dirname "$0")/.." && pwd)"

python="$(command -v python3 || echo /opt/python/cp312-cp312/bin/python)"
release() {
    "$python" -c "import json, sys; print(json.load(open(sys.argv[1]))['external_solvers'][sys.argv[2]]['release'])" \
        "$root/ThirdParty/versions.json" "$1"
}
cbc="$(release Cbc)"
ipopt="$(release Ipopt)"

if [ "$(uname)" = "Linux" ]; then
    # manylinux_2_28 (AlmaLinux 8): gfortran comes with the gcc toolset, BLAS and LAPACK from PowerTools
    dnf install -y --enablerepo=powertools blas-devel lapack-devel pkgconf-pkg-config
    jobs="$(nproc)"
else
    # Ipopt uses LAPACK from the Accelerate framework; MUMPS needs gfortran from Homebrew's gcc
    brew install gcc pkgconf
    export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-14.0}"
    jobs="$(sysctl -n hw.ncpu)"
fi

work="$(mktemp -d)"
git clone --depth 1 https://github.com/coin-or/coinbrew "$work/coinbrew"
cd "$work/coinbrew"
./coinbrew build "Ipopt@$ipopt" --prefix="$prefix" -j "$jobs" --no-prompt --tests none
rm -rf build
./coinbrew build "Cbc@$cbc" --prefix="$prefix" -j "$jobs" --no-prompt --tests none --no-third-party
rm -rf "$work"
