#!/usr/bin/env bash

# Robust shell settings: exit on error, unset vars, and fail pipelines
set -euo pipefail

###############################################################################
# Network reconfiguration for ad-hoc or custom static Wi‑Fi setup
#
# HOW TO USE:
#   1) Edit the CONFIG block below to match your system:
#      - IFACE:      your Wi‑Fi interface (e.g., wlp2s0, wlan0)
#      - MODE:       'ad-hoc' for 802.11s IBSS or 'managed' for normal AP
#      - SSID:       network name to form/join in ad‑hoc mode
#      - CHANNEL:    Wi‑Fi channel number (1..11/13 depending on region)
#      - IP_ADDR:    static IPv4 address for this host (unique per host)
#      - BROADCAST:  broadcast address of your subnet
#      - NETMASK:    netmask (e.g., 255.255.255.0)
#   2) Run with sudo:  sudo bash reconfigure_network.sh
#   3) Restore with NetworkManager: sudo systemctl start NetworkManager
#
# Power management: this script also disables Wi‑Fi power saving to reduce
# latency/jitter (large ping spikes). See the POWER MGMT section below.
# EXAMPLE (edit below):
#   IFACE=wlp2s0
#   MODE=ad-hoc
#   SSID=swarm_adhoc
#   CHANNEL=1
#   IP_ADDR=10.0.0.10
#   BROADCAST=10.0.0.255
#   NETMASK=255.255.255.0
###############################################################################

# ============================= CONFIG START ===============================
IFACE="wlo1"            # TODO: change to your Wi‑Fi interface (e.g., wlp2s0, wlan0)
MODE="ad-hoc"           # 'ad-hoc' or 'managed'
SSID="swarm_adhoc"      # TODO: change network name for ad‑hoc mode
CHANNEL="1"             # TODO: set Wi‑Fi channel
IP_ADDR="10.0.0.10"     # TODO: set unique static IP for this host
BROADCAST="10.0.0.255"  # TODO: broadcast address of your /24 subnet
NETMASK="255.255.255.0" # TODO: netmask
# ============================== CONFIG END ================================

echo "[INFO] Reconfiguring interface: ${IFACE} (mode=${MODE})"

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "[ERROR] Interface '${IFACE}' not found. Edit IFACE in this script." >&2
  exit 1
fi

echo "[INFO] Unblocking wireless radios"
rfkill unblock all || true

echo "[INFO] Flushing addresses on ${IFACE}"
ip addr flush dev "${IFACE}" || true

echo "[INFO] Bringing ${IFACE} down"
ifconfig "${IFACE}" down || true

echo "[INFO] Stopping NetworkManager (temporary). Use systemctl start NetworkManager to restore."
systemctl stop NetworkManager || service network-manager stop || true

if [[ "${MODE}" == "ad-hoc" ]]; then
  echo "[INFO] Configuring ad‑hoc (IBSS) mode: SSID='${SSID}', channel=${CHANNEL}"
  iwconfig "${IFACE}" mode ad-hoc
  iwconfig "${IFACE}" channel "${CHANNEL}"
  iwconfig "${IFACE}" essid "${SSID}"
else
  echo "[INFO] Configuring managed mode (client). SSID must be configured via wpa_supplicant/NetworkManager."
  iwconfig "${IFACE}" mode managed
fi

echo "[INFO] Assigning static IP ${IP_ADDR}/${NETMASK} broadcast ${BROADCAST}"
ip addr add "${IP_ADDR}/24" broadcast "${BROADCAST}" dev "${IFACE}" || true

echo "[INFO] Bringing ${IFACE} up"
ifconfig "${IFACE}" up

# =========================== POWER MGMT OFF ===============================
# Disable Wi‑Fi power saving to prevent high latency/jitter.
# Prefer modern 'iw' command; fallback to 'iwconfig' if needed.
if command -v iw >/dev/null 2>&1; then
  echo "[INFO] Disabling Wi‑Fi power save via 'iw'"
  # Some drivers accept 'set power_save off'; others require 'set power_save 0'
  if iw dev "${IFACE}" set power_save off 2>/dev/null; then
    :
  else
    iw dev "${IFACE}" set power_save 0 || true
  fi
fi
if command -v iwconfig >/dev/null 2>&1; then
  echo "[INFO] Disabling Wi‑Fi power save via 'iwconfig'"
  iwconfig "${IFACE}" power off || true
fi
# ========================================================================

echo "[INFO] Current IP configuration:"
ifconfig "${IFACE}" || true

echo "[INFO] Current Wi‑Fi settings:"
iwconfig "${IFACE}" || true

echo "[INFO] Done. Adjust IFACE/SSID/CHANNEL/IP in this script for your setup."


