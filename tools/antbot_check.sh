#!/usr/bin/env bash
#
# antbot_check.sh — health check for the real ANTBot.
#
# Unit assumed: 3D LiDAR, GNSS, camera and headlight REMOVED.
# Reference for every register and constant used here: ../hw_info.md
#
# Usage:
#   ./antbot_check.sh                 # same as 'all'
#   ./antbot_check.sh all
#   ./antbot_check.sh battery docking
#
# Sections: ports hw controllers topics battery encoder motors steering
#           power docking cargo
#
# Read-only. Nothing in this script writes to the board; use antbot_dock.sh
# for the one register that moves hardware.

set -uo pipefail

TIMEOUT="${ANTBOT_TIMEOUT:-8}"
READ_SRV='/hw/direct_read'
SRV_TYPE='antbot_interfaces/srv/DirectRead'

# ---------------------------------------------------------------- formatting
if [ -t 1 ]; then
  R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
else
  R=''; G=''; Y=''; B=''; N=''
fi

FAILURES=0
WARNINGS=0

hdr()  { printf '\n%s== %s %s\n' "$B" "$*" "$N"; }
ok()   { printf '  %s[ ok ]%s %s\n'   "$G" "$N" "$*"; }
warn() { printf '  %s[warn]%s %s\n'   "$Y" "$N" "$*"; WARNINGS=$((WARNINGS+1)); }
bad()  { printf '  %s[FAIL]%s %s\n'   "$R" "$N" "$*"; FAILURES=$((FAILURES+1)); }
info() { printf '  %s\n' "$*"; }

# ------------------------------------------------------------ register reads
# rcu_read <item_name>
# Echoes the raw int32 the service returned, or nothing on failure.
# Sets RCU_MSG to the service's message field ("addr[N]" or "item not found").
RCU_MSG=''
rcu_read() {
  local item="$1" out data
  RCU_MSG=''
  out=$(timeout "$TIMEOUT" ros2 service call "$READ_SRV" "$SRV_TYPE" \
          "{item_name: '$item'}" 2>/dev/null) || return 1
  RCU_MSG=$(printf '%s' "$out" | sed -n "s/.*message='\([^']*\)'.*/\1/p" | tail -1)
  [ "$RCU_MSG" = 'item not found' ] && return 1
  data=$(printf '%s' "$out" | sed -n 's/.*data=\(-\{0,1\}[0-9]\{1,\}\).*/\1/p' | tail -1)
  [ -z "$data" ] && return 1
  printf '%s' "$data"
}

# See hw_info.md §10.6 — direct_read never sign-extends. Undo that.
s8()  { local v="$1";  [ "$v" -gt 127 ]   && v=$((v - 256));   printf '%s' "$v"; }
s16() { local v="$1";  [ "$v" -gt 32767 ] && v=$((v - 65536)); printf '%s' "$v"; }

# scaled <raw> <scale> <decimals> — integer register * float scale
scaled() { awk -v v="$1" -v s="$2" -v d="${3:-2}" 'BEGIN{printf "%.*f", d, v*s}'; }

# label <value> <colon:separated:labels>
label() {
  local v="$1"; shift
  printf '%s' "$1" | awk -F: -v i="$v" '{print (i+1<=NF && $(i+1)!="") ? $(i+1) : "?"}'
}

# Read a register and print "name = raw (label)"; returns the raw value in REG.
REG=''
show() {  # show <item> <labels|-> <suffix>
  local item="$1" labels="${2:--}" suffix="${3:-}"
  REG=$(rcu_read "$item") || { bad "$item — ${RCU_MSG:-no response}"; REG=''; return 1; }
  if [ "$labels" != '-' ]; then
    info "$(printf '%-28s %6s  %s' "$item" "$REG" "$(label "$REG" "$labels")")"
  else
    info "$(printf '%-28s %6s  %s' "$item" "$REG" "$suffix")"
  fi
}

require_service() {
  if ! timeout "$TIMEOUT" ros2 service type "$READ_SRV" >/dev/null 2>&1; then
    bad "$READ_SRV is not available — is controller.launch.py running?"
    return 1
  fi
  return 0
}

# ===================================================================== checks

check_ports() {
  hdr 'Serial ports and input devices'
  local found=0 p
  for p in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2 /dev/ttyUSB3; do
    if [ -e "$p" ]; then ok "$p present  ($(stat -c '%U:%G %a' "$p"))"; found=$((found+1))
    else bad "$p missing"; fi
  done
  [ "$found" -eq 0 ] && info "No FTDI ports at all. Try: gpioset 0 1=0   (documented USB reset), then reboot."
  info 'Expected: ttyUSB0 RCU, ttyUSB1 IMU, ttyUSB2 2D LiDAR back, ttyUSB3 2D LiDAR front'

  if compgen -G '/dev/input/js*' >/dev/null; then
    for p in /dev/input/js*; do
      ok "$p — $(cat "/sys/class/input/$(basename "$p")/device/name" 2>/dev/null || echo 'name unavailable')"
    done
  else
    warn 'No /dev/input/js* — gamepad not connected (teleop will not work)'
  fi
}

check_hw() {
  hdr 'ros2_control hardware component'
  local out
  out=$(timeout "$TIMEOUT" ros2 control list_hardware_components 2>/dev/null)
  if [ -z "$out" ]; then
    bad 'list_hardware_components returned nothing — controller_manager not running'
    return
  fi
  printf '%s\n' "$out" | sed 's/^/  /'
  if printf '%s' "$out" | grep -q 'ant_hardware'; then
    if printf '%s' "$out" | grep -A2 'ant_hardware' | grep -qi 'active'; then
      ok 'ant_hardware is active'
    else
      bad 'ant_hardware exists but is not active — check serial_port / baud_rate / board_id'
    fi
  else
    bad 'ant_hardware component not found'
  fi

  hdr 'Exported interfaces'
  timeout "$TIMEOUT" ros2 control list_hardware_interfaces 2>/dev/null | sed 's/^/  /'
  info 'Expect per module: wheel_*/velocity,effort,position and steering_*/position,effort'
  info 'NOTE: "effort" here is motor CURRENT in amperes, not torque (hw_info.md §11.1)'
}

check_controllers() {
  hdr 'Controllers'
  local out
  out=$(timeout "$TIMEOUT" ros2 control list_controllers 2>/dev/null)
  if [ -z "$out" ]; then bad 'list_controllers returned nothing'; return; fi
  printf '%s\n' "$out" | sed 's/^/  /'
  local c
  for c in joint_state_broadcaster antbot_swerve_controller; do
    if printf '%s' "$out" | grep "$c" | grep -q 'active'; then ok "$c active"
    else bad "$c not active"; fi
  done
}

check_topics() {
  hdr 'Topics'
  local t
  for t in /joint_states /odom /battery /ultrasound /cargo/status /cmd_vel; do
    if timeout "$TIMEOUT" ros2 topic info "$t" >/dev/null 2>&1; then ok "$t exists"
    else bad "$t missing"; fi
  done
  info 'Rate check (control loop is 20 Hz):'
  timeout 5 ros2 topic hz /joint_states 2>/dev/null | head -3 | sed 's/^/    /' \
    || warn '/joint_states produced no rate sample in 5 s'
}

check_battery() {
  hdr 'Battery / BMS'
  require_service || return

  local v i pct cap chg bms soc t1 t2 heater
  v=$(rcu_read Battery_Voltage)      && info "$(printf '%-28s %8s V' 'Battery_Voltage' "$(scaled "$v" 0.01)")"
  i=$(rcu_read Battery_Current)      && info "$(printf '%-28s %8s A' 'Battery_Current' "$(scaled "$(s16 "$i")" 0.01)")"
  pct=$(rcu_read Battery_Percentage) && info "$(printf '%-28s %8s %%' 'Battery_Percentage' "$pct")"
  cap=$(rcu_read Battery_Capacity)   && info "$(printf '%-28s %8s Ah' 'Battery_Capacity' "$(scaled "$cap" 0.01)")"
  soc=$(rcu_read BMS_SOC)            && info "$(printf '%-28s %8s %%' 'BMS_SOC' "$soc")"
  t1=$(rcu_read BMS_Temperature)     && info "$(printf '%-28s %8s C' 'BMS_Temperature' "$(s8 "$t1")")"
  t2=$(rcu_read BMS_Temperature_2)   && info "$(printf '%-28s %8s C' 'BMS_Temperature_2' "$(s8 "$t2")")"
  show Battery_Is_Charging 'Discharging:Charging'
  show BMS_Is_Connected    'Not Connected:Connected'
  show BMS_Heater_Mode     'Off:Auto:Manual'

  bms=$(rcu_read BMS_Is_Connected)
  [ "${bms:-1}" = '0' ] && bad 'BMS not connected — charging cannot proceed, SOC unreliable'
  chg=$(rcu_read Battery_Is_Charging)
  if [ -n "${pct:-}" ]; then
    if   [ "$pct" -le 5 ]  && [ "${chg:-0}" = '0' ]; then bad "Battery at ${pct}% — below the 5% auto-shutdown threshold"
    elif [ "$pct" -le 20 ] && [ "${chg:-0}" = '0' ]; then warn "Battery at ${pct}% — dock soon"
    else ok "Battery at ${pct}%"; fi
  fi

  hdr 'Battery topic (/battery)'
  timeout "$TIMEOUT" ros2 topic echo /battery --once 2>/dev/null | sed 's/^/  /' \
    || bad '/battery produced no message'
  info 'power_supply_status is ONLY 1 (CHARGING) or 3 (NOT_CHARGING). 3 = running on'
  info 'battery, NOT a fault. health/technology are hardcoded UNKNOWN (hw_info.md §8.2).'
}

check_encoder() {
  hdr 'Encoders'
  require_service || return

  local ms mr
  show Motor_State         'IDLE:READY_ENTER:READY:FAULT_ENTER:FAULT:FAULT_EXIT:NOT_CONNECT:BRAKE'
  ms="$REG"
  show Motor_Reboot_Check  'FALSE:TRUE'
  mr="$REG"

  if [ "${ms:-}" = '2' ]; then
    ok 'Motor_State = READY — encoder ticks are accumulating'
  else
    bad "Motor_State = ${ms:-?} (not READY=2) — wheel positions in /joint_states are FROZEN"
    info 'This is the usual cause of "encoders look dead". Not an encoder fault.'
  fi
  [ "${mr:-0}" != '0' ] && warn "Motor_Reboot_Check = $mr — a motor rebooted; tick reference is being re-latched"

  info ''
  info 'Raw motor position registers (pulses, 16384 ticks/rev, 39.5 um/tick at the tread):'
  local n
  for n in 1 2 3 4; do show "M${n}_Present_Position" - 'pulses'; done

  info ''
  info 'Accumulated joint positions from /joint_states (relative — zeroed at every launch):'
  timeout "$TIMEOUT" ros2 topic echo /joint_states --once 2>/dev/null | sed 's/^/  /' \
    || bad '/joint_states produced no message'

  info ''
  info 'Rolling-check: push one wheel by hand and watch its position change ->'
  info "  ros2 topic echo /joint_states --field position"
}

check_motors() {
  hdr 'Drive motors M1-M4'
  require_service || return
  show Motor_State 'IDLE:READY_ENTER:READY:FAULT_ENTER:FAULT:FAULT_EXIT:NOT_CONNECT:BRAKE'
  show Estop_State 'OFF:ON'
  [ "${REG:-0}" = '1' ] && bad 'E-STOP IS ENGAGED — motors will not move'
  show System_State 'START:CHECKING:READY:RUNNING:ERROR:ESTOP:IDLE'

  local n e rpm cur dt mt
  local errlab='No_Error:Over_Volt:Low_Volt:Hot_Inverter:Hot_Motor:Overload:-:Inverter:-:-:-:-:-:Encoder:Hall_Sensor:Calibration:STO:-:BUS_WDG:Over_Speed'
  info ''
  info "$(printf '%-4s %10s %10s %8s %8s  %s' motor rev/min amps drv_C mot_C hw_error)"
  for n in 1 2 3 4; do
    rpm=$(rcu_read "M${n}_Present_RPM");          rpm=${rpm:-0}
    cur=$(rcu_read "M${n}_Present_Current");      cur=${cur:-0}
    dt=$(rcu_read "M${n}_Driver_Temperature");    dt=$(s8 "${dt:-0}")
    mt=$(rcu_read "M${n}_Motor_Temperature");     mt=$(s8 "${mt:-0}")
    e=$(rcu_read "M${n}_HW_ERROR");               e=${e:-0}
    info "$(printf 'M%-3s %10s %10s %8s %8s  %s' "$n" \
      "$(scaled "$rpm" 0.01)" "$(scaled "$cur" 0.001 3)" "$dt" "$mt" "$(label "$e" "$errlab")")"
    [ "$e" != '0' ] && bad "M${n} hardware error code $e ($(label "$e" "$errlab"))"
    [ "$mt" -gt 70 ] 2>/dev/null && warn "M${n} motor at ${mt} C"
  done
}

check_steering() {
  hdr 'Steering modules S1-S4'
  require_service || return
  show Swerve_Homing_Command 'None:Auto:Manual_Start:Manual_Done'

  local n p e cur t deg
  info ''
  info "$(printf '%-4s %10s %10s %8s %8s  %s' module pulses joint_deg amps temp_C err)"
  for n in 1 2 3 4; do
    p=$(rcu_read "S${n}_Present_Position"); p=${p:-0}
    cur=$(rcu_read "S${n}_Present_Current"); cur=${cur:-0}
    t=$(rcu_read "S${n}_Motor_Temperature"); t=$(s8 "${t:-0}")
    e=$(rcu_read "S${n}_Error_Code");        e=${e:-0}
    # joint_deg = (pulse - 2048) * (180/2048) / 2.43     (hw_info.md §4.2)
    deg=$(awk -v p="$p" 'BEGIN{printf "%.2f", (p-2048)*(180.0/2048.0)/2.43}')
    info "$(printf 'S%-3s %10s %10s %8s %8s  %s' "$n" "$p" "$deg" \
      "$(scaled "$cur" 0.001 3)" "$t" "$e")"
    [ "$e" != '0' ] && bad "S${n} error code $e"
    awk -v d="$deg" 'BEGIN{exit !(d>55.5 || d<-55.5)}' && warn "S${n} at ${deg} deg — outside the +/-55 deg limit"
  done
  info ''
  info 'Zero = 2048 pulses. Limit +/-55 deg = pulses 527 .. 3569.'
  info 'All four should read near 0 deg when the wheels point straight ahead.'
}

check_power() {
  hdr 'Power rails and currents'
  require_service || return
  local r
  info 'Voltages (0.1 V per LSB):'
  for r in Voltage_Batt Voltage_RCU_14V Voltage_RCU_12V Voltage_RCU_12V_2 \
           Voltage_RCU_5V Voltage_RCU_3V3 Voltage_ORIN_17V Voltage_ORIN_12V \
           Voltage_ORIN_5V Voltage_ORIN_3V3 Voltage_ORIN_1V8 Voltage_ORIN_1V2 \
           Voltage_ORIN_LIDAR Voltage_ORIN_ROUTER; do
    v=$(rcu_read "$r") && info "$(printf '  %-24s %8s V' "$r" "$(scaled "$v" 0.1 1)")"
  done
  info 'Currents (0.1 A per LSB):'
  for r in Current_Batt Current_Motor Current_Charge Current_RCU_12V \
           Current_RCU_12V_2 Current_RCU_5V Current_HEAT_POWER Current_ORIN_Batt; do
    v=$(rcu_read "$r") && info "$(printf '  %-24s %8s A' "$r" "$(scaled "$v" 0.1 1)")"
  done
  info ''
  info 'Voltage_ORIN_LIDAR is the 3D-LiDAR rail; a low reading is expected on this'
  info 'unit because the 3D LiDAR is removed.'

  hdr 'Board identity'
  show Model_Number - '(expect 526)'
  show Firmware_Version_Major - ''
  show Firmware_Version_Minor - ''
  show Board_Version - ''
  show Error_Code - '(0 = no board error)'
  v=$(rcu_read Sec_Since_Power_On) && info "$(printf '%-28s %6s s  (%s min)' 'Sec_Since_Power_On' "$v" "$((v/60))")"
}

check_docking() {
  hdr 'Docking / charging'
  require_service || return
  show Docking              'No:Yes'
  local docked="$REG"
  show Charge_Sequence      'Standby:Optimize Current:On Charge:Full Charged:Charge Error:Stop Charge'
  local seq="$REG"
  show Charge_Swerve_Status 'Released:Gripped'
  show Charge_Force_Status  'None:Force Charge Enabled:Force Charge Disabled'
  show Charge_Error         - '(0 = none)'
  local cerr="$REG"
  show Charge_Hall_Left     - 'raw'
  local hl="$REG"
  show Charge_Hall_Right    - 'raw'
  local hr="$REG"
  show Charging_Station_ID  - ''
  show Charge_FW_Version_Major - ''
  show Charge_FW_Version_Minor - ''

  info ''
  if [ "${docked:-0}" = '1' ]; then
    ok 'Robot reports DOCKED'
    case "${seq:-}" in
      0) warn 'Docked but Charge_Sequence = Standby — contacts probably not gripped' ;;
      1) ok 'Optimizing charge current' ;;
      2) ok 'ON CHARGE' ;;
      3) ok 'Fully charged' ;;
      4) bad "CHARGE ERROR — Charge_Error = ${cerr:-?}. Clear with: ./antbot_dock.sh clear" ;;
      5) warn 'Charging stopped' ;;
    esac
  else
    info 'Not docked (this is normal when driving).'
  fi
  if [ -n "${hl:-}" ] && [ -n "${hr:-}" ]; then
    info "Hall sensors L=$hl R=$hr — symmetric values mean the coil is centred."
  fi
  info ''
  info 'There is no autonomous docking node in this workspace. Reverse the robot'
  info 'into the station manually; charging starts on its own. Use'
  info '  ./tools/antbot_dock.sh monitor   to watch this live while you approach.'
}

check_cargo() {
  hdr 'Cargo / wiper / headlight'
  require_service || return
  show Module_Type      'None:Single:Dispenser:Double:Open:Single_Auto:Single_Dump:Single_Manual'
  show Cargo_Door_State 'Closed:Opened'
  show Cargo_Lock_State 'Neutral:Locked:Unlocked:No_Response'
  show Cargo_Detect_Adc - 'raw'
  show Wiper_Mode       'Off:Once:INT'
  show Wiper_Connected_1 'Disconnected:Connected'
  show Headlight_State  'OFF:ON'
  info ''
  info 'The headlight is removed on this unit. Headlight_State still reads/writes'
  info 'because the device class is compiled in unconditionally, and'
  info '/headlight/operation still returns success: true (hw_info.md §2).'

  local r
  info ''
  info 'Ultrasound (cm registers -> /ultrasound publishes metres):'
  for r in UltraSonic_1 UltraSonic_2; do show "$r" - 'cm'; done
}

# ======================================================================= main
run() {
  case "$1" in
    ports)       check_ports ;;
    hw)          check_hw ;;
    controllers) check_controllers ;;
    topics)      check_topics ;;
    battery)     check_battery ;;
    encoder)     check_encoder ;;
    motors)      check_motors ;;
    steering)    check_steering ;;
    power)       check_power ;;
    docking)     check_docking ;;
    cargo)       check_cargo ;;
    all)         check_ports; check_hw; check_controllers; check_topics
                 check_motors; check_steering; check_encoder
                 check_battery; check_power; check_docking; check_cargo ;;
    *) printf 'unknown section: %s\n' "$1" >&2
       printf 'sections: ports hw controllers topics battery encoder motors steering power docking cargo all\n' >&2
       exit 2 ;;
  esac
}

if ! command -v ros2 >/dev/null 2>&1; then
  printf 'ros2 not on PATH. Source the workspace first:\n' >&2
  printf '  source /opt/ros/humble/setup.bash && source ~/ros2_humble_ws/install/setup.bash\n' >&2
  exit 1
fi

[ "$#" -eq 0 ] && set -- all
for section in "$@"; do run "$section"; done

printf '\n%s== summary %s\n' "$B" "$N"
printf '  failures: %s%d%s   warnings: %s%d%s\n' \
  "$([ "$FAILURES" -gt 0 ] && printf '%s' "$R" || printf '%s' "$G")" "$FAILURES" "$N" \
  "$([ "$WARNINGS" -gt 0 ] && printf '%s' "$Y" || printf '%s' "$G")" "$WARNINGS" "$N"
[ "$FAILURES" -gt 0 ] && exit 1
exit 0
