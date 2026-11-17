#!/bin/sh
### BEGIN INIT INFO
# Provides:          uav-wifi
# Required-Start:    $local_fs
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: RTL8188EU Wi-Fi with DHCP refresh after reconnection
### END INIT INFO
# Version: 2.0 (2026-11-10)
# Target: PetaLinux 2020.1 / SysVinit / BusyBox / one DHCP-only wlan0.
# wpa_cli invokes this same file as: uav-wifi wlan0 CONNECTED|DISCONNECTED.
# Install at /etc/init.d/uav-wifi. Do not run a second wlan0 network manager.

PATH=/sbin:/usr/sbin:/bin:/usr/bin
export PATH
umask 077

IFACE=wlan0
CONF=/etc/wpa_supplicant.conf
CTRL=/var/run/wpa_supplicant
SELF=/etc/init.d/uav-wifi
RUN=/run/uav-wifi
WPA_PID=/run/uav-wpa.pid
CLI_PID=/run/uav-wpa-cli.pid
DHCP_PID=/run/uav-udhcpc.pid
LOCK=$RUN/lock
MODE=${1:-}
[ "$MODE" = "$IFACE" ] && MODE=event

log() {
    # Logs live in RAM, not in the persistent SD rootfs.
    if [ -f "$RUN/events.log" ] && [ "$(wc -c < "$RUN/events.log")" -gt 65536 ]; then
        mv -f "$RUN/events.log" "$RUN/events.log.1"
    fi
    printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >> "$RUN/events.log"
}

error() {
    echo "uav-wifi: ERROR: $*" >&2
    log "ERROR: $*"
    return 1
}

# Match full /proc arguments, not the possibly truncated BusyBox ps output.
# PID files are also checked against the program name and the PID-file argument.
pid_matches() {
    case "$1" in ''|*[!0-9]*) return 1 ;; esac
    [ "$1" -gt 1 ] || return 1
    [ -r "/proc/$1/cmdline" ] || return 1
    kill -0 "$1" 2>/dev/null || return 1
    tr '\000' '\n' < "/proc/$1/cmdline" 2>/dev/null |
        awk -v prog="$2" -v marker="$3" '
            { n=$0; sub(/^.*\//,"",n); if (NR<=2 && n==prog) p=1; if ($0==marker) m=1 }
            END { exit !(p && m) }'
}

owned_pid() {
    [ -r "$1" ] || return 1
    OWN_PID=$(cat "$1" 2>/dev/null)
    pid_matches "$OWN_PID" "$2" "$1"
}

stop_owned() {
    if owned_pid "$1" "$2"; then
        STOP_PID=$OWN_PID
        kill -TERM "$STOP_PID" 2>/dev/null || true
        STOP_WAIT=0
        while pid_matches "$STOP_PID" "$2" "$1"; do
            STOP_WAIT=$((STOP_WAIT + 1))
            if [ "$STOP_WAIT" -ge 8 ]; then
                error "$2 PID $STOP_PID did not stop; refusing to create a duplicate."
                return 1
            fi
            sleep 1
        done
    fi
    rm -f "$1"
}

# Refuse to compete with a manually started client or another network manager.
# Only processes explicitly targeting wlan0 are considered.
check_foreign() {
    for PROC in /proc/[0-9]*/cmdline; do
        [ -r "$PROC" ] || continue
        PROC_PID=${PROC#/proc/}; PROC_PID=${PROC_PID%/cmdline}
        if tr '\000' '\n' < "$PROC" 2>/dev/null |
            awk -v prog="$1" -v iface="$IFACE" '
                {
                    n=$0; sub(/^.*\//,"",n); if (NR<=2 && n==prog) p=1
                    if ((prev=="-i" || prev=="--interface") && $0==iface) i=1
                    if ($0=="-i" iface || $0=="--interface=" iface) i=1
                    if ($0=="-a" || $0 ~ /^-a.+/) a=1
                    prev=$0
                }
                END { exit !(p && i && (prog!="wpa_cli" || a)) }'; then
            if owned_pid "$2" "$1" && [ "$PROC_PID" = "$OWN_PID" ]; then
                continue
            fi
            error "Unmanaged $1 on $IFACE (PID $PROC_PID). Use UART and reboot after upgrading; do not start duplicate clients."
            return 1
        fi
    done
}

unlock() {
    if [ "$(cat "$LOCK/pid" 2>/dev/null)" = "$$" ]; then
        rm -f "$LOCK/pid"
        rmdir "$LOCK" 2>/dev/null || true
    fi
}

lock_service() {
    LOCK_WAIT=0
    while ! mkdir "$LOCK" 2>/dev/null; do
        # A stop command may be waiting for the watcher to exit.
        if [ "$MODE" = event ] && [ ! -f "$RUN/enabled" ]; then
            exit 0
        fi
        LOCK_OWNER=$(cat "$LOCK/pid" 2>/dev/null)
        case "$LOCK_OWNER" in
            ''|*[!0-9]*) ;;
            *) if ! kill -0 "$LOCK_OWNER" 2>/dev/null; then
                   rm -f "$LOCK/pid"
                   rmdir "$LOCK" 2>/dev/null || true
               fi ;;
        esac
        LOCK_WAIT=$((LOCK_WAIT + 1))
        if [ "$LOCK_WAIT" -ge 60 ]; then
            error "Service lock is busy: $LOCK"
            return 1
        fi
        sleep 1
    done
    echo "$$" > "$LOCK/pid"
    trap 'unlock' 0
    trap 'exit 1' HUP INT TERM
}

clear_ipv4() {
    # This service owns DHCP IPv4 on wlan0 only. Do not touch docker0/eth0/IPv6.
    [ -d "/sys/class/net/$IFACE" ] || return 0
    ip -4 route flush dev "$IFACE" 2>/dev/null || return 1
    ip -4 addr flush dev "$IFACE" 2>/dev/null || return 1
}

wpa_status() {
    wpa_cli -p "$CTRL" -i "$IFACE" status 2>/dev/null
}

stop_dhcp() {
    stop_owned "$DHCP_PID" udhcpc || return 1
    rm -f "$RUN/network"
    clear_ipv4 || { error "Could not clear IPv4 on $IFACE"; return 1; }
}

start_dhcp() {
    # Called only after authentication completes. A fresh process starts DHCP
    # discovery, rather than trying to renew a lease on the previous subnet.
    check_foreign udhcpc "$DHCP_PID" || return 1
    stop_dhcp || return 1
    : > "$RUN/udhcpc.log"
    (
        trap '' HUP
        exec udhcpc -f -i "$IFACE" -p "$DHCP_PID" -t 3 -T 3 -A 5
    ) < /dev/null >> "$RUN/udhcpc.log" 2>&1 &
    # -f prevents an internal fork, so $! remains the client PID.
    echo "$!" > "$DHCP_PID"
    DHCP_WAIT=0
    while ! owned_pid "$DHCP_PID" udhcpc; do
        DHCP_WAIT=$((DHCP_WAIT + 1))
        if [ "$DHCP_WAIT" -ge 5 ]; then
            error "udhcpc did not start; see $RUN/udhcpc.log"
            return 1
        fi
        sleep 1
    done
    log "DHCP discovery started on $IFACE (PID $OWN_PID)."
    # Use the system's existing udhcpc default hook for address/gateway/DNS.
    # No -q: the client stays alive to renew the lease. No -R: do not transmit
    # a release for the old subnet after switching to a different AP.
}

handle_event() {
    [ -f "$RUN/enabled" ] || return 0
    case "$1" in
        DISCONNECTED)
            log "DISCONNECTED: stop old DHCP and clear $IFACE IPv4."
            stop_dhcp
            ;;
        CONNECTED|SYNC|RENEW)
            STATUS=$(wpa_status) || STATUS=
            if ! printf '%s\n' "$STATUS" | grep -q '^wpa_state=COMPLETED$'; then
                [ "$1" = RENEW ] && { error "Wi-Fi is not COMPLETED; DHCP was not started."; return 1; }
                # A queued CONNECTED event can be stale; use the current state.
                stop_dhcp
                return $?
            fi
            NETWORK=$(printf '%s\n' "$STATUS" | grep -E '^(id|ssid|bssid)=')
            # Avoid duplicate DHCP starts when bootstrap and CONNECTED overlap.
            if [ "$1" != RENEW ] && [ -f "$RUN/network" ] &&
               [ "$(cat "$RUN/network")" = "$NETWORK" ] &&
               owned_pid "$DHCP_PID" udhcpc; then
                return 0
            fi
            start_dhcp || return 1
            printf '%s\n' "$NETWORK" > "$RUN/network"
            log "$1: authenticated; a fresh DHCP acquisition is in progress."
            ;;
    esac
}

start_wifi() {
    for CMD in modprobe ip wpa_supplicant wpa_cli udhcpc awk tr; do
        command -v "$CMD" >/dev/null 2>&1 || { error "Missing command: $CMD"; return 1; }
    done
    [ -r "$CONF" ] || { error "Cannot read $CONF"; return 1; }
    [ -x "$SELF" ] || { error "Install this script as $SELF and chmod 755 it."; return 1; }
    wpa_cli -h 2>&1 | grep -q -- '-a' || { error "wpa_cli lacks action mode (-a)."; return 1; }
    check_foreign wpa_supplicant "$WPA_PID" || return 1
    check_foreign wpa_cli "$CLI_PID" || return 1
    check_foreign udhcpc "$DHCP_PID" || return 1

    modprobe r8188eu || { error "modprobe r8188eu failed"; return 1; }
    IF_WAIT=0
    while [ ! -d "/sys/class/net/$IFACE" ]; do
        IF_WAIT=$((IF_WAIT + 1))
        [ "$IF_WAIT" -le 30 ] || { error "$IFACE not found"; return 1; }
        sleep 1
    done
    ip link set "$IFACE" up || return 1

    if ! owned_pid "$WPA_PID" wpa_supplicant; then
        rm -f "$WPA_PID"
        wpa_supplicant -B -D wext -c "$CONF" -i "$IFACE" -P "$WPA_PID" \
            < /dev/null > "$RUN/wpa.log" 2>&1 || {
                error "wpa_supplicant failed; see $RUN/wpa.log"; return 1;
            }
    fi
    CTRL_WAIT=0
    while ! wpa_cli -p "$CTRL" -i "$IFACE" ping 2>/dev/null | grep -q '^PONG$'; do
        CTRL_WAIT=$((CTRL_WAIT + 1))
        if [ "$CTRL_WAIT" -ge 10 ]; then
            error "No WPA control socket. Check ctrl_interface=$CTRL in $CONF."
            stop_owned "$WPA_PID" wpa_supplicant
            return 1
        fi
        sleep 1
    done

    : > "$RUN/enabled"
    if ! owned_pid "$CLI_PID" wpa_cli; then
        rm -f "$CLI_PID"
        wpa_cli -B -p "$CTRL" -i "$IFACE" -a "$SELF" -P "$CLI_PID" \
            < /dev/null > "$RUN/monitor.log" 2>&1 || {
                rm -f "$RUN/enabled"
                error "wpa_cli action monitor failed; see $RUN/monitor.log"
                return 1
            }
        CLI_WAIT=0
        while ! owned_pid "$CLI_PID" wpa_cli; do
            CLI_WAIT=$((CLI_WAIT + 1))
            if [ "$CLI_WAIT" -ge 5 ]; then
                rm -f "$RUN/enabled"
                error "wpa_cli monitor did not start; see $RUN/monitor.log"
                return 1
            fi
            sleep 1
        done
    fi
    # Handles a connection that completed before the event listener attached.
    handle_event SYNC || return 1
    log "Service started; monitoring $IFACE connect/disconnect events."
    echo "uav-wifi started; Wi-Fi authentication and DHCP may still be pending."
}

stop_wifi() {
    rm -f "$RUN/enabled"
    stop_owned "$CLI_PID" wpa_cli || return 1
    stop_dhcp || return 1
    stop_owned "$WPA_PID" wpa_supplicant || return 1
    log "Service stopped."
    echo "uav-wifi stopped."
}

reload_wifi() {
    owned_pid "$WPA_PID" wpa_supplicant && owned_pid "$CLI_PID" wpa_cli || {
        error "Managed Wi-Fi service is not running; use start or restart via UART."; return 1;
    }
    REPLY=$(wpa_cli -p "$CTRL" -i "$IFACE" reconfigure 2>&1)
    printf '%s\n' "$REPLY"
    printf '%s\n' "$REPLY" | grep -q '^OK$' || return 1
    log "Configuration reloaded; DHCP refresh follows actual reconnection events."
}

show_status() {
    if owned_pid "$WPA_PID" wpa_supplicant; then echo "wpa_supplicant: running ($OWN_PID)"; else echo "wpa_supplicant: not managed/running"; fi
    if owned_pid "$CLI_PID" wpa_cli; then echo "event monitor: running ($OWN_PID)"; else echo "event monitor: not running"; fi
    if owned_pid "$DHCP_PID" udhcpc; then echo "DHCP client: running ($OWN_PID)"; else echo "DHCP client: not running (normal while disconnected)"; fi
    echo "--- WPA ---"
    wpa_status || true
    echo "--- IPv4 ---"
    ip -4 addr show dev "$IFACE" 2>/dev/null || true
    echo "--- Routes ---"
    ip -4 route show dev "$IFACE" 2>/dev/null || true
}

[ "$(id -u)" = 0 ] || { echo "Run as root on the PetaLinux host." >&2; exit 1; }
mkdir -p "$RUN" || exit 1
chmod 700 "$RUN"
case "$MODE" in
    status) show_status; exit 0 ;;
    start|stop|restart|reload|renew) ;;
    event)
        [ -f "$RUN/enabled" ] || exit 0
        case "${2:-}" in CONNECTED|DISCONNECTED) ;; *) exit 0 ;; esac
        ;;
    *) echo "Usage: $0 {start|stop|restart|reload|renew|status}"; exit 1 ;;
esac
lock_service || exit 1
case "$MODE" in
    start) start_wifi ;;
    stop) stop_wifi ;;
    restart) stop_wifi && start_wifi ;;
    reload) reload_wifi ;;
    renew) [ -f "$RUN/enabled" ] && handle_event RENEW ;;
    event) handle_event "$2" ;;
esac
