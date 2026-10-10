#!/usr/bin/env bash
# Regenerate SHOT's Boost subset from the pinned official source archive.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
archive=${1:?Usage: bash misc/vendor-boost.sh /path/to/boost_1_92_0.tar.bz2}
archive=$(cd "$(dirname "$archive")" && pwd)/$(basename "$archive")
expected=5c1d40cb8e19adbf740a4ec2da35b3e58f3f5804b1dce44deb53df72193cbc6c
actual=$(shasum -a 256 "$archive" | cut -d ' ' -f 1)
if [[ "$actual" != "$expected" ]]; then
    echo "Boost archive checksum mismatch" >&2
    exit 1
fi
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
tar -xjf "$archive" -C "$stage"
cd "$stage/boost_1_92_0"
./bootstrap.sh
./b2 tools/bcp
mkdir "$stage/subset"
./dist/bin/bcp --boost=. \
    boost/functional/hash/hash.hpp boost/math/tools/roots.hpp \
    boost/math/tools/minima.hpp boost/cstdint.hpp boost/version.hpp \
    "$stage/subset"
# bcp also exports library sources/docs. SHOT consumes only headers and license.
mkdir "$stage/vendor"
cp -R "$stage/subset/boost" "$stage/vendor/boost"
cp LICENSE_1_0.txt "$stage/vendor/LICENSE_1_0.txt"
rm -rf "$root/ThirdParty/boost"
mv "$stage/vendor" "$root/ThirdParty/boost"
