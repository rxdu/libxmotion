#!/bin/bash

# Default values
DEFAULT_OUT_IFACE="eno1"

# Function to print usage
print_usage() {
    echo "Usage: $0 <IN_INTERFACE> [OUT_INTERFACE]"
    echo "  IN_INTERFACE: Required. The incoming interface to be replaced."
    echo "  OUT_INTERFACE: Optional. The outgoing interface to be replaced. Defaults to '$DEFAULT_OUT_IFACE'."
}

# Check arguments
if [ $# -lt 1 ]; then
    echo "Error: Missing arguments."
    print_usage
    exit 1
fi

# Assign arguments
IN_INTERFACE="$1"
OUT_INTERFACE="${2:-$DEFAULT_OUT_IFACE}"

# Execute iptables commands
iptables --table nat --append POSTROUTING --out-interface "$OUT_INTERFACE" -j MASQUERADE
iptables --append FORWARD --in-interface "$IN_INTERFACE" --out-interface "$OUT_INTERFACE" -j ACCEPT
iptables --append FORWARD --in-interface "$OUT_INTERFACE" --out-interface "$IN_INTERFACE" -m state --state RELATED,ESTABLISHED -j ACCEPT
iptables --append FORWARD --in-interface "$IN_INTERFACE" -j ACCEPT

echo "Iptables rules successfully added for IN_INTERFACE=$IN_INTERFACE and OUT_INTERFACE=$OUT_INTERFACE."
