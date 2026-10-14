#!/bin/sh
# Install after stock AP startup; the paired Pi supervises recovery.
set -eu
PATH=/sbin:/usr/sbin:/bin:/usr/bin
export PATH
CONFIG_FILE=${AP_CSI_CONFIG_FILE:-/jffs/csi/ap-csi.env}
if [ -r "$CONFIG_FILE" ]; then . "$CONFIG_FILE"; fi
MODULE_PATH=${AP_CSI_MODULE_PATH:-/lib/modules/4.1.27/extra/dhd.ko}
CANDIDATE=${AP_CSI_CANDIDATE:-/jffs/csi/dhd-ap-concurrent.ko}
EXPECTED_CANDIDATE=${AP_CSI_EXPECTED_CANDIDATE_MD5:-1194d18691a7f7db8476749fd4ec0ea0}
EXPECTED_STOCK=${AP_CSI_EXPECTED_STOCK_MD5:-ac4f5be9e63816e1eea59490853c1b6b}
CONFIGURE=${AP_CSI_CONFIGURE:-/jffs/csi/configcsi-ap.sh}
NEXUTIL=${AP_CSI_NEXUTIL:-/jffs/csi/nexutil}
WL=${AP_CSI_WL:-/usr/sbin/wl}
SERVICE=${AP_CSI_SERVICE:-/sbin/service}
RMMOD=${AP_CSI_RMMOD:-/sbin/rmmod}
INSMOD=${AP_CSI_INSMOD:-/sbin/insmod}
MOUNT=${AP_CSI_MOUNT:-/bin/mount}
MD5SUM=${AP_CSI_MD5SUM:-/usr/bin/md5sum}
SLEEP=${AP_CSI_SLEEP:-/bin/sleep}
KILLALL=${AP_CSI_KILLALL:-/usr/bin/killall}
RADIO_INTERFACE=${AP_CSI_INTERFACE:-eth6}
SSID=${AP_CSI_SSID:-WIRES-AP}
CHANNEL=${AP_CSI_CHANNEL:-36}
BANDWIDTH=${AP_CSI_BANDWIDTH_MHZ:-80}
TX_STREAMS=${AP_CSI_TX_STREAMS:-4}
TARGET_MAC=${AP_CSI_TARGET_MAC:-}
RX_CORES=${AP_CSI_RX_CORES:-4}
MIN_INTERVAL_MS=${AP_CSI_MIN_INTERVAL_MS:-1}
QUIESCE_PROCESSES=${AP_CSI_QUIESCE_PROCESSES:-"dhd_monitor roamast wlc_nt networkmap wps_monitor wpsaide wlceventd watchdog bwdpi_check cfg_server mastiff"}
LOG=${AP_CSI_LOG:-/tmp/ap-csi-autostart.log}
LOCK=${AP_CSI_LOCK:-/tmp/ap-csi-autostart.lock}
ATTEMPT=${AP_CSI_ATTEMPT_FILE:-/tmp/ap-csi-install-attempted}
SIGNATURE_FILE=${AP_CSI_CONFIG_SIGNATURE_FILE:-/tmp/ap-csi-config.signature}
SIGNATURE="$EXPECTED_CANDIDATE|$RADIO_INTERFACE|$SSID|$CHANNEL/$BANDWIDTH|$TX_STREAMS|$TARGET_MAC|$RX_CORES|$MIN_INTERVAL_MS"

mkdir "$LOCK" 2>/dev/null || exit 1
trap 'rmdir "$LOCK" 2>/dev/null || true' EXIT
trap 'exit 1' HUP INT TERM
exec >>"$LOG" 2>&1
file_md5() { "$MD5SUM" "$1" | awk '{print $1}'; }
log() { printf '%s %s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$1"; }
snapshot_enabled() {
    # Query 501 reads shared memory without touching PHY registers.
    "$NEXUTIL" -I "$RADIO_INTERFACE" -g501 -l16 | grep -q '^0x000000: 01 00 51 53'
}
radio_ready() {
    "$WL" -i "$RADIO_INTERFACE" status | grep -F -q "SSID: \"$SSID\"" \
        && "$WL" -i "$RADIO_INTERFACE" bss | grep -q '^up$' \
        && "$WL" -i "$RADIO_INTERFACE" chanspec | grep -F -q "$CHANNEL/$BANDWIDTH"
}

[ "$(file_md5 "$CANDIDATE")" = "$EXPECTED_CANDIDATE" ] || {
    log 'candidate_hash_invalid=1'; exit 1;
}
current_hash=$(file_md5 "$MODULE_PATH")
if [ "$current_hash" = "$EXPECTED_CANDIDATE" ]; then
    [ -r "$SIGNATURE_FILE" ] && [ "$(cat "$SIGNATURE_FILE")" = "$SIGNATURE" ] \
        && snapshot_enabled || exit 1
    exit 0
fi
[ "$current_hash" = "$EXPECTED_STOCK" ] || {
    log "unsupported_loaded_module hash=$current_hash"; exit 1;
}
[ ! -e "$ATTEMPT" ] || { log 'already_attempted_this_boot=1'; exit 1; }
radio_ready || { log 'stock_ap_identity_or_channel_not_ready=1'; exit 1; }

# Record the attempt before changing the radio so recovery can detect it.
: > "$ATTEMPT"
log 'candidate_install_started=1'
for process in $QUIESCE_PROCESSES; do "$KILLALL" "$process" 2>/dev/null || true; done
"$RMMOD" dhd
"$MOUNT" -o bind "$CANDIDATE" "$MODULE_PATH"
"$INSMOD" "$MODULE_PATH"
"$SERVICE" restart_wireless
ready=0
attempt=0
while [ "$attempt" -lt 45 ]; do
    if radio_ready; then ready=1; break; fi
    attempt=$((attempt + 1))
    "$SLEEP" 1
done
[ "$ready" = 1 ] || { log 'candidate_ap_not_ready=1'; exit 1; }
AP_CSI_INTERFACE="$RADIO_INTERFACE" AP_CSI_WL="$WL" AP_CSI_NEXUTIL="$NEXUTIL" \
    "$CONFIGURE" "$CHANNEL" "$BANDWIDTH" "$TX_STREAMS" "$TARGET_MAC" "$MIN_INTERVAL_MS" "$RX_CORES"
snapshot_enabled || { log 'snapshot_protocol_or_capture_not_ready=1'; exit 1; }
printf '%s\n' "$SIGNATURE" > "$SIGNATURE_FILE"
log "candidate_install_complete=1 ap_mode=1 coherent_snapshot=1 chanspec=$CHANNEL/$BANDWIDTH"
