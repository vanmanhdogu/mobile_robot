#!/usr/bin/env bash
#
# antbot_dock.sh — docking / charging helper for the real ANTBot.
#
# There is NO autonomous docking node in this workspace. Docking is a physical
# reverse manoeuvre into the wireless station; charging then starts by itself.
# This script gives you (a) live feedback while you drive in, and (b) access to
# the one writable docking register, Charge_Command (addr 237).
#
# Register reference: ../hw_info.md §9
#
# Usage:
#   ./antbot_dock.sh status        # one-shot snapshot            (read-only)
#   ./antbot_dock.sh monitor       # live, 1 Hz, until Ctrl-C     (read-only)
#   ./antbot_dock.sh align         # hall-sensor alignment bar    (read-only)
#   ./antbot_dock.sh grip          # Charge_Command = 1   ** MOVES HARDWARE **
#   ./antbot_dock.sh release       # Charge_Command = 2   ** MOVES HARDWARE **
#   ./antbot_dock.sh force-on      # Charge_Command = 3   (diagnostics only)
#   ./antbot_dock.sh force-off     # Charge_Command = 4   (diagnostics only)
#   ./antbot_dock.sh clear         # Charge_Command = 5   (clear forced state)
#
# Write commands ask for confirmation unless ANTBOT_YES=1 is set.

set -uo pipefail

TIMEOUT="${ANTBOT_TIMEOUT:-8}"
INTERVAL="${ANTBOT_INTERVAL:-1}"

if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
else R=''; G=''; Y=''; B=''; N=''; fi

rcu_read() {
  local out data
  out=$(timeout "$TIMEOUT" ros2 service call /hw/direct_read \
          antbot_interfaces/srv/DirectRead "{item_name: '$1'}" 2>/dev/null) || return 1
  printf '%s' "$out" | grep -q "item not found" && return 1
  data=$(printf '%s' "$out" | sed -n 's/.*data=\(-\{0,1\}[0-9]\{1,\}\).*/\1/p' | tail -1)
  [ -z "$data" ] && return 1
  printf '%s' "$data"
}

rcu_write() {  # rcu_write <item> <value>
  local out
  out=$(timeout "$TIMEOUT" ros2 service call /hw/direct_write \
          antbot_interfaces/srv/DirectWrite "{item_name: '$1', data: $2}" 2>/dev/null) || {
    printf '%sservice call failed / timed out%s\n' "$R" "$N"; return 1; }
  printf '%s\n' "$out" | tail -2
  printf '%s' "$out" | grep -q 'success=True'
}

label() { printf '%s' "$2" | awk -F: -v i="$1" '{print (i+1<=NF && $(i+1)!="") ? $(i+1) : "?"}'; }

SEQ_LABELS='Standby:Optimize Current:On Charge:Full Charged:Charge Error:Stop Charge'

snapshot() {
  local dock seq grip force cerr hl hr sid pct chg
  dock=$(rcu_read Docking)              || { printf '%s/hw/direct_read unavailable — is controller.launch.py running?%s\n' "$R" "$N"; return 1; }
  seq=$(rcu_read Charge_Sequence)
  grip=$(rcu_read Charge_Swerve_Status)
  force=$(rcu_read Charge_Force_Status)
  cerr=$(rcu_read Charge_Error)
  hl=$(rcu_read Charge_Hall_Left)
  hr=$(rcu_read Charge_Hall_Right)
  sid=$(rcu_read Charging_Station_ID)
  pct=$(rcu_read Battery_Percentage)
  chg=$(rcu_read Battery_Is_Charging)

  printf '%s%-22s%s %s\n' "$B" 'Docking' "$N" \
    "$([ "${dock:-0}" = 1 ] && printf '%sYES%s' "$G" "$N" || printf 'no')"
  printf '%s%-22s%s %s (%s)\n' "$B" 'Charge_Sequence' "$N" "${seq:-?}" "$(label "${seq:-9}" "$SEQ_LABELS")"
  printf '%s%-22s%s %s\n' "$B" 'Charge_Swerve_Status' "$N" \
    "$([ "${grip:-0}" = 1 ] && printf 'Gripped' || printf 'Released')"
  printf '%s%-22s%s %s\n' "$B" 'Charge_Force_Status' "$N" "$(label "${force:-0}" 'None:Force Charge Enabled:Force Charge Disabled')"
  printf '%s%-22s%s %s%s\n' "$B" 'Charge_Error' "$N" "${cerr:-?}" \
    "$([ "${cerr:-0}" != 0 ] && printf ' %s<-- see error-codes doc%s' "$R" "$N")"
  printf '%s%-22s%s L=%s  R=%s\n' "$B" 'Hall sensors' "$N" "${hl:-?}" "${hr:-?}"
  printf '%s%-22s%s %s\n' "$B" 'Charging_Station_ID' "$N" "${sid:-?}"
  printf '%s%-22s%s %s%%  (%s)\n' "$B" 'Battery' "$N" "${pct:-?}" \
    "$([ "${chg:-0}" = 1 ] && printf '%scharging%s' "$G" "$N" || printf 'on battery')"
}

hint() {
  local dock="$1" seq="$2" grip="$3" cerr="$4" bms="$5"
  if [ "${bms:-1}" = '0' ]; then
    printf '%s-> BMS not connected. Charging cannot proceed regardless of docking.%s\n' "$R" "$N"
  elif [ "${dock:-0}" != '1' ]; then
    printf '   -> Not seated. Keep reversing; watch the hall sensors converge.\n'
  elif [ "${seq:-0}" = '4' ]; then
    printf '%s-> CHARGE ERROR %s. Clear it:  ./antbot_dock.sh clear%s\n' "$R" "${cerr:-?}" "$N"
  elif [ "${grip:-0}" != '1' ]; then
    printf '%s-> Docked but contacts not gripped. Check Charge_Error (%s).%s\n' "$Y" "${cerr:-?}" "$N"
  elif [ "${seq:-0}" = '0' ]; then
    printf '%s-> Gripped but sequence still Standby.%s\n' "$Y" "$N"
  elif [ "${seq:-0}" = '2' ]; then
    printf '%s-> Charging normally.%s\n' "$G" "$N"
  elif [ "${seq:-0}" = '3' ]; then
    printf '%s-> Fully charged.%s\n' "$G" "$N"
  fi
}

do_status() { snapshot; }

do_monitor() {
  printf 'Polling /hw/direct_read at %ss. Ctrl-C to stop.\n' "$INTERVAL"
  trap 'printf "\nstopped\n"; exit 0' INT
  while true; do
    clear
    printf '%s== ANTBot docking monitor ==%s   %s\n\n' "$B" "$N" "$(date '+%H:%M:%S')"
    snapshot || exit 1
    printf '\n'
    hint "$(rcu_read Docking)" "$(rcu_read Charge_Sequence)" \
         "$(rcu_read Charge_Swerve_Status)" "$(rcu_read Charge_Error)" \
         "$(rcu_read BMS_Is_Connected)"
    sleep "$INTERVAL"
  done
}

do_align() {
  printf 'Hall-sensor alignment. Reverse slowly; aim for L and R to rise together.\n'
  printf 'Ctrl-C to stop.\n\n'
  trap 'printf "\nstopped\n"; exit 0' INT
  local hl hr d bar
  while true; do
    hl=$(rcu_read Charge_Hall_Left)  || { printf '%sread failed%s\n' "$R" "$N"; exit 1; }
    hr=$(rcu_read Charge_Hall_Right) || hr=0
    d=$((hl - hr))
    # 40-char bar, centre at 20, 1 char per 5 raw counts of imbalance
    bar=$(awk -v d="$d" 'BEGIN{
      p=int(20 + d/5); if(p<0)p=0; if(p>40)p=40;
      s=""; for(i=0;i<=40;i++) s = s (i==p ? "|" : (i==20 ? "+" : "-"));
      print s}')
    printf '\rL=%-6s R=%-6s  diff=%-6s  [%s]  %s' "$hl" "$hr" "$d" "$bar" \
      "$([ "${d#-}" -le 20 ] && printf '%scentred%s ' "$G" "$N" || printf '%soff-centre%s' "$Y" "$N")"
    sleep "${ANTBOT_INTERVAL:-0.5}"
  done
}

confirm() {
  [ "${ANTBOT_YES:-0}" = '1' ] && return 0
  printf '%s%s%s\n' "$Y" "$1" "$N"
  printf 'Type yes to proceed: '
  local a; read -r a
  [ "$a" = 'yes' ]
}

do_write() {  # do_write <value> <name> <warning>
  local val="$1" name="$2" warning="$3"
  if [ -n "$warning" ]; then
    confirm "$warning" || { printf 'aborted\n'; exit 1; }
  fi
  printf 'Writing Charge_Command = %s (%s) ...\n' "$val" "$name"
  if rcu_write Charge_Command "$val"; then
    printf '%swrite accepted%s\n\n' "$G" "$N"
  else
    printf '%swrite reported failure%s\n\n' "$R" "$N"
  fi
  sleep 1
  snapshot
}

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'ros2 not on PATH. source /opt/ros/humble/setup.bash and install/setup.bash first.\n' >&2
  exit 1
fi

case "${1:-status}" in
  status)    do_status ;;
  monitor)   do_monitor ;;
  align)     do_align ;;
  grip)      do_write 1 Grip \
               'Grip moves the physical charging mechanism. Only do this while docked.' ;;
  release)   do_write 2 Release \
               'Release will ungrip the charging contacts and stop charging.' ;;
  force-on)  do_write 3 'Force Charge Enable' \
               'Force Charge bypasses the normal charge sequence. Diagnostics only — clear it afterwards.' ;;
  force-off) do_write 4 'Force Charge Disable' \
               'This forcibly disables charging until cleared.' ;;
  clear)     do_write 5 'Force Charge Clear' '' ;;
  *) awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' "$0"; exit 2 ;;
esac
