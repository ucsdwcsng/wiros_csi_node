#!/bin/sh
# CSI configuration that preserves the existing AP radio mode.
# It deliberately does not reload dhd, retune the radio, or enable monitor mode.
set -eu

CH=${1:?channel required}
BW=${2:?bandwidth required}
SS=${3:?spatial stream count required}
MAC=${4:-}
MIN_INTERVAL_MS=${5:-1}
CORES=${6:-4}
IFACE=${AP_CSI_INTERFACE:-eth6}
WL=${AP_CSI_WL:-/usr/sbin/wl}
NEXUTIL=${AP_CSI_NEXUTIL:-./nexutil}
MAKECSIPARAMS=${AP_CSI_MAKECSIPARAMS:-./makecsiparams}

# The native snapshot transport uses a fixed 1 ms immutable-buffer export tick.
[ "$MIN_INTERVAL_MS" = 1 ] || {
  echo "native AP snapshots require a 1 ms export tick" >&2
  exit 2
}

cd "$(dirname "$0")"

case "$SS" in
  1) SS_HEX=1 ;;
  2) SS_HEX=3 ;;
  3) SS_HEX=7 ;;
  4) SS_HEX=f ;;
  *) echo "invalid spatial stream count: $SS" >&2; exit 2 ;;
esac

case "$CORES" in
  1) CORE_HEX=1 ;;
  2) CORE_HEX=3 ;;
  3) CORE_HEX=7 ;;
  4) CORE_HEX=f ;;
  *) echo "invalid core count: $CORES" >&2; exit 2 ;;
esac

CURRENT=$("$WL" -i "$IFACE" chanspec)
echo "existing_chanspec=$CURRENT"
echo "$CURRENT" | grep -q "$CH/$BW" || {
  echo "AP is not already on requested channel $CH/$BW" >&2
  exit 3
}

if [ -n "$MAC" ]; then
  PARAMS=$("$MAKECSIPARAMS" -e 1 -m "$MAC" -b 0x88 -c "$CH/$BW" -C "0x$CORE_HEX" -N "0x$SS_HEX" -d "$MIN_INTERVAL_MS")
else
  PARAMS=$("$MAKECSIPARAMS" -e 1 -b 0x88 -c "$CH/$BW" -C "0x$CORE_HEX" -N "0x$SS_HEX" -d "$MIN_INTERVAL_MS")
fi

"$NEXUTIL" -I "$IFACE" -s500 -b -l38 -v "$PARAMS"
CSI_STATE=$("$NEXUTIL" -I "$IFACE" -g501 -l2)
case "$CSI_STATE" in
  "0x000000: 01 00"*) ;;
  *) echo "CSI configuration was not accepted: $CSI_STATE" >&2; exit 4 ;;
esac

echo "csi_collect_enabled=1"
echo "csi_stream_tick_ms=$MIN_INTERVAL_MS"
echo "ap_mode_preserved=1"
"$WL" -i "$IFACE" status | head -4
