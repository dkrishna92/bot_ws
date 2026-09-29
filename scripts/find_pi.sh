#!/usr/bin/env bash
# Find the race Pi's IP address on the local network.
#
# The Pi gets its address from DHCP, so it changes between networks (and
# sometimes between days on the same one). This tries, in order:
#   1. mDNS: <hostname>.local (fast, when the network passes multicast)
#   2. A ping sweep of this laptop's subnet(s), then the ARP table for
#      Raspberry Pi MAC prefixes and anything with SSH open
# Each candidate is confirmed by logging in with the robot's SSH key and
# reading its hostname, so a different Pi on the same network isn't
# mistaken for ours.
#
# Usage:
#   scripts/find_pi.sh                  # search this laptop's subnet(s)
#   scripts/find_pi.sh 10.50.189.0/24   # search a specific /24 as well
#   scripts/find_pi.sh --update         # also point 'robot-pi' in
#                                       # ~/.ssh/config at the found IP
#
# Needs only ping, ip and ssh (no nmap/arp-scan, no sudo). Networks with
# client isolation (some campus/guest Wi-Fi) block this entirely -- if it
# finds nothing, check the router's DHCP client list or plug a monitor
# into the Pi and run 'hostname -I'.

set -u

PI_HOSTNAME="claudebot-desktop"
PI_USER="claudebot"
PI_KEY="$HOME/.ssh/id_ed25519_robot_pi"
SSH_ALIAS="robot-pi"
# Raspberry Pi Ltd / Foundation OUIs (Pi 5 Wi-Fi/Ethernet use the first two)
PI_OUIS="2c:cf:67|d8:3a:dd|dc:a6:32|e4:5f:01|b8:27:eb|28:cd:c1|88:a2:9e"

update=0
extra_subnets=()
for arg in "$@"; do
    case "$arg" in
        --update) update=1 ;;
        -h|--help) sed -n '2,23p' "$0"; exit 0 ;;
        *) extra_subnets+=("$arg") ;;
    esac
done

# Succeeds (and prints the hostname) only if IP is our Pi.
confirm() {
    local ip=$1 name
    name=$(ssh -n -o BatchMode=yes -o ConnectTimeout=4 -o IdentitiesOnly=yes \
               -o StrictHostKeyChecking=accept-new -i "$PI_KEY" \
               "$PI_USER@$ip" hostname 2>/dev/null) || return 1
    [ "$name" = "$PI_HOSTNAME" ]
}

port22_open() {
    timeout 1 bash -c "exec 3<>/dev/tcp/$1/22" 2>/dev/null
}

found() {
    local ip=$1
    echo "Found $PI_HOSTNAME at $ip"
    if [ "$update" = 1 ]; then
        local cfg="$HOME/.ssh/config"
        # Rewrite HostName only inside the 'Host robot-pi' block.
        sed -i "/^Host $SSH_ALIAS\$/,/^Host /s/^\(\s*HostName\s\+\).*/\1$ip/" "$cfg"
        echo "Updated '$SSH_ALIAS' in $cfg -> $ip"
    elif [ "${alias_ok:-0}" = 1 ]; then
        echo "  ssh $SSH_ALIAS"
    else
        echo "  ssh $PI_USER@$ip   (or rerun with --update to fix the '$SSH_ALIAS' alias)"
    fi
    echo "  dashboard: http://$ip:8080"
    exit 0
}

# 0. The address the alias already points at
current=$(ssh -G "$SSH_ALIAS" 2>/dev/null | awk '/^hostname /{print $2}')
if [ -n "$current" ]; then
    echo "Trying current '$SSH_ALIAS' address $current ..."
    confirm "$current" && { echo "'$SSH_ALIAS' is already correct."; update=0; alias_ok=1; found "$current"; }
fi

# 1. mDNS
echo "Trying mDNS ($PI_HOSTNAME.local) ..."
ip=$(timeout 5 getent ahostsv4 "$PI_HOSTNAME.local" 2>/dev/null | awk 'NR==1{print $1}')
if [ -n "$ip" ] && confirm "$ip"; then found "$ip"; fi

# 2. Ping sweep + ARP. Only /24s are swept; a wider local prefix is
#    narrowed to this laptop's own /24.
subnets=()
while read -r cidr; do
    subnets+=("${cidr%.*}.0/24")
done < <(ip -4 -o addr show scope global | awk '$2 !~ /^(docker|br-|veth|virbr)/ {split($4,a,"/"); print a[1]}')
subnets+=("${extra_subnets[@]}")

for net in "${subnets[@]}"; do
    base=${net%.*}
    echo "Sweeping $base.1-254 ..."
    for i in $(seq 1 254); do
        ping -c1 -W1 "$base.$i" >/dev/null 2>&1 &
    done
    wait

    # Pi MAC prefixes first, then any other responder with SSH open.
    pis=$(ip neigh show | grep -iE "lladdr ($PI_OUIS)" | awk -v b="$base." 'index($1,b)==1{print $1}')
    for ip in $pis; do
        echo "  Raspberry Pi MAC at $ip, checking ..."
        confirm "$ip" && found "$ip"
    done
    others=$(ip neigh show | grep -E "REACHABLE|STALE|DELAY" | awk -v b="$base." 'index($1,b)==1{print $1}')
    for ip in $others; do
        case " $pis " in *" $ip "*) continue ;; esac
        port22_open "$ip" || continue
        echo "  SSH open at $ip, checking ..."
        confirm "$ip" && found "$ip"
    done
done

echo "Pi not found on: ${subnets[*]}" >&2
echo "Is it powered and on the same network as this laptop? Try giving its" >&2
echo "subnet explicitly (e.g. $0 10.50.189.0/24) or check the router's DHCP list." >&2
exit 1
