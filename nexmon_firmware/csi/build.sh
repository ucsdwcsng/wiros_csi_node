#!/usr/bin/env bash
# Build locally. Never installs firmware or connects to a router.
set -euo pipefail
if [[ $# != 3 ]]; then
    echo "usage: $0 NEXMON_CHECKOUT NEXMON_CSI_CHECKOUT OUTPUT_KO" >&2
    exit 2
fi
snapshot_resources=$(cd -- "$(dirname -- "$0")" && pwd)
nexmon_checkout=$(cd -- "$1" && pwd)
csi_checkout=$(cd -- "$2" && pwd)
output_module=$(realpath -m -- "$3")
source "$nexmon_checkout/setup_env.sh"
firmware_dir="$nexmon_checkout/firmwares/bcm4366c0/10_10_122_20"
stock_md5=$(md5sum "$firmware_dir/dhd.ko" | cut -d ' ' -f 1)
[[ "$stock_md5" == e5ecd7420088fdf7aa933eed93d6128b ]] || {
    echo "unsupported stock driver; refusing to build" >&2
    exit 1
}
make -C "$firmware_dir"
build_parent="$nexmon_checkout/patches/bcm4366c0/10_10_122_20"
snapshot_workdir=$(mktemp -d "$build_parent/ap-snapshot.XXXXXXXX")
git -C "$csi_checkout" archive 1232edf2cf1e7ae0561676c49e4dd4eb22c8a27e | tar -x -C "$snapshot_workdir"
python3 "$snapshot_resources/prepare.py" --nexmon "$nexmon_checkout" \
    --upstream "$csi_checkout" --workdir "$snapshot_workdir" \
    --patch "$snapshot_workdir/ap-csi-concurrent.patch"
make -C "$snapshot_workdir" GIT_VERSION=ap-snapshot-1.0
install -m 0644 "$snapshot_workdir/dhd.ko" "$output_module"
sha256sum "$output_module"
printf 'Build sources and logs: %s\n' "$snapshot_workdir"
