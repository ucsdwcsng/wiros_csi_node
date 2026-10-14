#!/bin/sh
# Stock boots first. One failed switch/health check latches future switches off.
set -eu
ROUTER_HOST=${AP_CSI_ROUTER_HOST:-192.168.48.6}
SOURCE_IP=${AP_CSI_SOURCE_IP:-192.168.48.20}
LINK_INTERFACE=${AP_CSI_LINK_INTERFACE:-eth0}
ROUTER_USER=${AP_CSI_ROUTER_USER:-wiloc}
PASSWORD_FILE=${AP_CSI_PASSWORD_FILE:-/home/wiloc/.config/wiros/ap_csi_router_password}
KNOWN_HOSTS=${AP_CSI_KNOWN_HOSTS:-/home/wiloc/.ssh/ap_csi_router_known_hosts}
DISABLE_FILE=${AP_CSI_DISABLE_FILE:-/home/wiloc/.config/wiros/ap_csi_disabled}
INFLIGHT_FILE=${AP_CSI_INFLIGHT_FILE:-/home/wiloc/.config/wiros/ap_csi_install_inflight}
REMOTE_COMMAND=${AP_CSI_REMOTE_COMMAND:-/jffs/csi/ap-csi-autostart.sh}
RADIO_INTERFACE=${AP_CSI_RADIO_INTERFACE:-eth6}
BOOT_SETTLE=${AP_CSI_BOOT_SETTLE:-120}
INTERVAL=${AP_CSI_HEALTH_INTERVAL:-60}
RETRY_DELAY=${AP_CSI_RETRY_DELAY:-20}
COMMAND_TIMEOUT=${AP_CSI_COMMAND_TIMEOUT:-120}
HEALTH_TIMEOUT=${AP_CSI_HEALTH_TIMEOUT:-20}
MAX_FAILURES=${AP_CSI_MAX_FAILURES:-3}
SSH=${AP_CSI_SSH:-/usr/bin/ssh}
SSHPASS=${AP_CSI_SSHPASS:-/usr/bin/sshpass}
PING=${AP_CSI_PING:-/usr/bin/ping}
TIMEOUT=${AP_CSI_TIMEOUT:-/usr/bin/timeout}
SLEEP=${AP_CSI_SLEEP:-/usr/bin/sleep}
LOGGER=${AP_CSI_LOGGER:-/usr/bin/logger}
failures=0
configured_boot=
last_state=

announce() {
    if [ "$1" != "$last_state" ]; then
        printf '%s\n' "$2"
        "$LOGGER" -t ap-csi-rpi-watchdog -- "$2"
        last_state=$1
    fi
}
router_ssh() {
    "$TIMEOUT" --signal=TERM --kill-after=5 "${2:-$COMMAND_TIMEOUT}" \
        "$SSHPASS" -f "$PASSWORD_FILE" "$SSH" -n -b "$SOURCE_IP" \
        -o ConnectTimeout=5 -o ConnectionAttempts=1 -o HostkeyAlgorithms=+ssh-rsa \
        -o PubkeyAuthentication=no -o NumberOfPasswordPrompts=1 \
        -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
        -o StrictHostKeyChecking=accept-new -o UserKnownHostsFile="$KNOWN_HOSTS" \
        "$ROUTER_USER@$ROUTER_HOST" "$1"
}
fail_closed() {
    # Persist the disable latch before rebooting to prevent reinstall loops.
    mkdir -p "$(dirname "$DISABLE_FILE")"
    printf '%s\n' "$1" > "$DISABLE_FILE.tmp"
    mv "$DISABLE_FILE.tmp" "$DISABLE_FILE"
    rm -f "$INFLIGHT_FILE"
    announce disabled "AP+CSI auto-install disabled: $1; retaining stock boot fallback"
    # Avoid rebooting an unchanged stock AP when a preflight check failed.
    if router_ssh 'test -e /tmp/ap-csi-install-attempted' "$HEALTH_TIMEOUT"; then
        announce recovery 'Requesting one stock recovery reboot over Ethernet'
        router_ssh /sbin/reboot 15 || true
    fi
}
health_check() {
    router_ssh "
        test -d /sys/class/net/$RADIO_INTERFACE || exit 10
        /bin/grep -q '^dhd ' /proc/modules || exit 11
        /jffs/csi/nexutil -I '$RADIO_INTERFACE' -g501 -l16 |
            /bin/grep -q '^0x000000: 01 00 51 53' || exit 12
        errors=\$(/bin/grep -c 'wl1: PHYTX error' /tmp/syslog.log || true)
        test \"\$errors\" -lt 5 || exit 13
        /bin/dmesg | /bin/grep -q 'timeout > MAX_CNTL_TX_TIMEOUT' && exit 14
        exit 0
    " "$HEALTH_TIMEOUT"
}

while :; do
    if [ -e "$DISABLE_FILE" ]; then
        announce disabled 'AP+CSI auto-install is disabled; stock AP recovery remains available'
        "$SLEEP" "$INTERVAL"; continue
    fi
    if [ -e "$INFLIGHT_FILE" ]; then
        fail_closed 'supervisor restarted during an unfinished driver switch'
        continue
    fi
    if [ ! -x "$SSHPASS" ] || [ ! -r "$PASSWORD_FILE" ]; then
        announce credentials 'Waiting for AP+CSI router credentials'
        "$SLEEP" "$RETRY_DELAY"; continue
    fi
    if [ ! -r "/sys/class/net/$LINK_INTERFACE/carrier" ] \
        || [ "$(cat "/sys/class/net/$LINK_INTERFACE/carrier")" != 1 ] \
        || ! "$PING" -I "$LINK_INTERFACE" -c 1 -W 2 "$ROUTER_HOST" >/dev/null 2>&1; then
        announce network 'Waiting for router Ethernet reachability'
        "$SLEEP" "$RETRY_DELAY"; continue
    fi
    state=$(router_ssh 'cut -d. -f1 /proc/uptime; cat /proc/sys/kernel/random/boot_id' "$HEALTH_TIMEOUT") || state=
    uptime=$(printf '%s\n' "$state" | sed -n '1p')
    boot=$(printf '%s\n' "$state" | sed -n '2p')
    case "$uptime" in ''|*[!0-9]*) "$SLEEP" "$RETRY_DELAY"; continue ;; esac
    case "$boot" in ''|*[!0-9a-fA-F-]*) "$SLEEP" "$RETRY_DELAY"; continue ;; esac
    if [ "$uptime" -lt "$BOOT_SETTLE" ]; then
        announce settle 'Waiting for normal stock AP startup to finish'
        "$SLEEP" "$RETRY_DELAY"; continue
    fi
    if [ "$configured_boot" = "$boot" ]; then
        if health_check; then
            failures=0
            announce healthy 'AP+CSI snapshot firmware is healthy'
            "$SLEEP" "$INTERVAL"
        else
            failures=$((failures + 1))
            announce "failure_$failures" "AP+CSI health probe failed ($failures/$MAX_FAILURES)"
            if [ "$failures" -ge "$MAX_FAILURES" ]; then
                fail_closed 'repeated snapshot firmware health failures'
            else
                "$SLEEP" "$RETRY_DELAY"
            fi
        fi
        continue
    fi
    mkdir -p "$(dirname "$INFLIGHT_FILE")"
    printf '%s\n' "$boot" > "$INFLIGHT_FILE"
    if router_ssh "$REMOTE_COMMAND" && health_check; then
        rm -f "$INFLIGHT_FILE"
        configured_boot=$boot
        failures=0
        announce healthy 'Stock startup and AP+CSI switch verified'
        "$SLEEP" "$INTERVAL"
    else
        fail_closed 'driver switch or initial snapshot health check failed'
    fi
done
