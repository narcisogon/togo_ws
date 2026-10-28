# shellcheck shell=bash
# Helpers shared by scripts/run_apollo16_sim.sh and scripts/open_gui.sh (sourced by them, not a program).
#
# State folder, one per user: ${APOLLO16_STATE_DIR:-/tmp/apollo16_sim-<uid>} (mode 700). /tmp is emptied
# when WSL restarts; entries whose process is gone are removed by the next script that starts.
#   run.<wrapper pid>/session     key=value facts about a running run_apollo16_sim.sh: world, spawn pose,
#                                 GZ_PARTITION, ready marker paths (open_gui.sh reads it)
#   run.<wrapper pid>/ready       the launch's ready marker (ready.not_ready when its READY check failed)
#   run.<wrapper pid>/gui.config  the wrapper's GUI config with the camera at the spawn
#   gui.<pgid>                    a Gazebo GUI one of the scripts started: its process group and that process's
#                                 start time (so a reused PID is never taken for it); open_gui.sh --close
#   open_gui.<pid>/gui.config     open_gui.sh's GUI config
#
# WSL crash dumps: WSL pipes the core of every crashing Linux process to the Windows folder
# %TEMP%\wsl-crashes (core_pattern "|/wsl-capture-crash %t %E %p %s"), as
#   wsl-crash-<unix time>-<pid>-<executable path, / as _>-<signal>.dmp   (1.5-3.6 GB for gz)
# a16_crash_dumps_after_run deletes only dumps of processes this run started (their PIDs are recorded while
# they run), written after this run began, for an executable whose name contains ruby or gz; any other dump
# is listed, never touched.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "apollo16_lib.sh is sourced by run_apollo16_sim.sh and open_gui.sh; it does nothing on its own" >&2
  exit 2
fi

# ------------------------------------------------------------------ arguments
a16_usage() { awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"; }
a16_die_usage() { echo "ERROR: $*" >&2; echo "See: $0 --help" >&2; exit 2; }
a16_need_val() {   # a16_need_val <option> <count> <the arguments after the option...>
  local opt="$1" n="$2" v
  shift 2
  (( $# >= n )) || a16_die_usage "${opt} needs ${n} value(s)"
  for v in "${@:1:n}"; do
    [[ "${v}" != --* ]] || a16_die_usage "${opt} is missing a value (got the option '${v}')"
  done
}
a16_is_number() { [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]; }
a16_is_seconds() { [[ "$1" =~ ^[0-9]+$ ]] && (( 10#$1 > 0 )); }

# ------------------------------------------------------------------ processes
A16_FORCE_STOP=0
A16_OUR_PIDS=" "
a16_nap() { sleep "$1" & wait "$!" 2>/dev/null; }      # a sleep that a trapped signal interrupts at once
a16_group_alive() { [[ -n "${1:-}" ]] && kill -0 -- "-$1" 2>/dev/null; }
a16_proc_start() {   # start time of a process (clock ticks after boot, /proc/PID/stat field 22); fails if gone
  local stat rest
  stat="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  [[ -n "${stat}" ]] || return 1
  rest="${stat##*) }"
  local -a f
  read -r -a f <<< "${rest}"
  [[ -n "${f[19]:-}" ]] || return 1
  printf '%s\n' "${f[19]}"
}
a16_record_pids() {  # a16_record_pids SID...: remember these session leaders and every process of their sessions
  local sid p
  for sid in "$@"; do
    [[ -n "${sid}" ]] || continue
    # the leader itself even before its setsid() has run (then pgrep -s does not list it yet)
    for p in "${sid}" $(pgrep -s "${sid}" 2>/dev/null); do
      [[ "${A16_OUR_PIDS}" == *" ${p} "* ]] || A16_OUR_PIDS+="${p} "
    done
  done
}
a16_core_signal_rc() {   # exit status 128+N of a signal that dumps core (QUIT ILL TRAP ABRT BUS FPE SEGV XCPU XFSZ SYS)
  case "$1" in 131|132|133|134|135|136|139|152|153|159) return 0 ;; esac
  return 1
}

# ------------------------------------------------------------------ memory
A16_MEM_LEVEL=0
a16_mem_available_mb() { awk '/^MemAvailable:/ { printf "%d\n", $2 / 1024; exit }' /proc/meminfo 2>/dev/null; }
a16_sid_rss_mb() {   # resident memory of all processes of a session, MB
  [[ -n "${1:-}" ]] || { echo 0; return 0; }
  ps -o rss= -s "$1" 2>/dev/null | awk '{ s += $1 } END { printf "%d\n", s / 1024 }'
}
a16_check_memory() {   # a16_check_memory GUI_SID SERVER_SID: one WARNING per level when WSL runs low on memory
  local avail warn="${APOLLO16_LOW_MEM_MB:-3000}" level=0 parts=""
  avail="$(a16_mem_available_mb)"
  [[ "${avail}" =~ ^[0-9]+$ && "${warn}" =~ ^[0-9]+$ ]] || return 0
  (( avail < warn )) && level=1
  (( avail < warn / 2 )) && level=2
  if (( level > A16_MEM_LEVEL )); then
    A16_MEM_LEVEL=${level}
    [[ -z "${1:-}" ]] || parts+="Gazebo GUI $(a16_sid_rss_mb "$1") MB, "
    [[ -z "${2:-}" ]] || parts+="simulation $(a16_sid_rss_mb "$2") MB, "
    echo "WARNING: WSL is running out of memory: ${avail} MB left (${parts%, }). When it runs out, WSL stops" \
         "every program. Close the Gazebo GUI (Ctrl+C in its terminal, or open_gui.sh --close) or stop the" \
         "simulation before that." >&2
  elif (( level == 0 && avail > warn + 1000 )); then
    A16_MEM_LEVEL=0                                     # recovered: warn again next time
  fi
}

# ------------------------------------------------------------------ state folder, sessions, GUI registrations
a16_state_dir() {    # prints the state folder (created, mode 700); fails if it is not usable
  local d="${APOLLO16_STATE_DIR:-/tmp/apollo16_sim-$(id -u)}"
  [[ -d "${d}" ]] || { mkdir -p -- "$(dirname -- "${d}")" && mkdir -m 700 -- "${d}"; } 2>/dev/null
  if [[ ! -d "${d}" || -L "${d}" || ! -O "${d}" || ! -w "${d}" ]]; then
    echo "ERROR: ${d} is not a writable folder owned by you; remove it or set APOLLO16_STATE_DIR" >&2
    return 1
  fi
  printf '%s\n' "${d}"
}
a16_kv() {           # a16_kv FILE KEY: the value of a key=value line
  awk -v k="$2" 'index($0, k "=") == 1 { print substr($0, length(k) + 2); exit }' "$1" 2>/dev/null
}
a16_write_kv() {     # a16_write_kv FILE key=value...: written atomically (temporary file, then rename)
  local f="$1" tmp
  shift
  tmp="$(dirname -- "${f}")/.$(basename -- "${f}").$$.tmp"
  printf '%s\n' "$@" > "${tmp}" && mv -f -- "${tmp}" "${f}"
}
a16_kv_set() {       # a16_kv_set FILE key=value...: change or add keys of a key=value file (atomically)
  local f="$1" kv i found
  local -a lines=()
  shift
  [[ ! -f "${f}" ]] || mapfile -t lines < "${f}"
  for kv in "$@"; do
    found=0
    for i in "${!lines[@]}"; do
      if [[ "${lines[i]%%=*}" == "${kv%%=*}" ]]; then lines[i]="${kv}"; found=1; fi
    done
    (( found )) || lines+=("${kv}")
  done
  a16_write_kv "${f}" "${lines[@]}"
}
a16_is_wrapper() {   # a16_is_wrapper PID: that process is a running run_apollo16_sim.sh
  [[ "${1:-}" =~ ^[0-9]+$ && -r "/proc/$1/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -q 'run_apollo16_sim[.]sh'
}
a16_prune_state() {  # a16_prune_state DIR: remove entries (<name>.<pid>) whose process is gone
  local e pid
  for e in "$1"/*.[0-9]*; do
    [[ -e "${e}" || -L "${e}" ]] || continue
    pid="${e##*.}"
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    kill -0 "${pid}" 2>/dev/null && continue
    rm -rf -- "${e}"
  done
}
a16_live_sessions() {   # a16_live_sessions DIR: session files of running run_apollo16_sim.sh, one per line
  local f
  for f in "$1"/run.*/session; do
    [[ -f "${f}" ]] || continue
    a16_is_wrapper "$(a16_kv "${f}" wrapper_pid)" && printf '%s\n' "${f}"
  done
  return 0
}
a16_register_gui() {    # a16_register_gui DIR PGID: record a GUI this script started
  a16_write_kv "$1/gui.$2" "pgid=$2" "start=$(a16_proc_start "$2")" "owner_pid=$$" "owner=${0##*/}" \
               "gz_partition=${GZ_PARTITION:-}" "started=$(date +%s)"
}
a16_unregister_gui() { [[ -z "${2:-}" ]] || rm -f -- "$1/gui.$2"; }
a16_gui_alive() {       # a16_gui_alive REGISTRATION: the registered GUI process still runs (same PID and start time)
  local pg st
  pg="$(a16_kv "$1" pgid)"
  st="$(a16_kv "$1" start)"
  [[ "${pg}" =~ ^[0-9]+$ && -n "${st}" ]] || return 1
  [[ "$(a16_proc_start "${pg}" 2>/dev/null)" == "${st}" ]]
}
a16_open_guis() {       # a16_open_guis DIR: registrations of running GUIs (of this GZ_PARTITION if it is set)
  local f
  for f in "$1"/gui.*; do
    [[ -f "${f}" ]] || continue
    a16_gui_alive "${f}" || continue
    if [[ -n "${GZ_PARTITION+x}" && "$(a16_kv "${f}" gz_partition)" != "${GZ_PARTITION}" ]]; then continue; fi
    printf '%s\n' "${f}"
  done
  return 0
}

# ------------------------------------------------------------------ the Gazebo GUI
a16_gui_notice() {   # a16_gui_notice LINE...: what to expect from the window, then the given lines
  local line
  echo "=========================================="
  echo "Opening the Gazebo GUI. The window appears within a few seconds but stays blank (black or grey)"
  echo "for about 30-60 s while it loads the world: wait for the terrain."
  echo "Its title should be \"Gazebo Sim (Ubuntu)\" (WSLg appends the distro name). If it starts with"
  echo "\"[WARN:COPY MODE]\", WSLg has fallen back to a slow copy mode: close all WSL terminals, run"
  echo "\"wsl --shutdown\" in PowerShell (this stops every WSL program, Docker in WSL too), then start again."
  for line in "$@"; do echo "${line}"; done
  echo "=========================================="
}

# a16_resolve [POSE values]: world, spawn pose and GUI config through the launch file's own functions.
# Input: A16_LAUNCH A16_WORLD A16_VARIANT A16_SITE A16_WORLD_INFO A16_CHECK_PLUGIN (1/0) A16_GUI_MODE
# (follow: write A16_GUI_SRC with the camera moved to the spawn to A16_GUI_OUT; A16_GUI_OUT empty = dry run).
# Output: KEY<TAB>value lines (WORLD_FILE PHYSICS POSE POSE_VALUES GRAVITY NOTE... GUI_NOTE).
a16_resolve() {
  A16_LAUNCH="${A16_LAUNCH:-}" A16_WORLD="${A16_WORLD:-}" A16_VARIANT="${A16_VARIANT:-}" A16_SITE="${A16_SITE:-flat}" \
  A16_WORLD_INFO="${A16_WORLD_INFO:-}" A16_CHECK_PLUGIN="${A16_CHECK_PLUGIN:-1}" A16_GUI_MODE="${A16_GUI_MODE:-none}" \
  A16_GUI_SRC="${A16_GUI_SRC:-}" A16_GUI_OUT="${A16_GUI_OUT:-}" python3 - "$@" <<'PY'
import importlib.util, os, sys
E = os.environ
spec = importlib.util.spec_from_file_location("apollo16_sim_launch", E["A16_LAUNCH"])
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
pose = [p for p in sys.argv[1:] if p]
try:
    wf, notes = m.resolve_world(E["A16_WORLD"], E["A16_VARIANT"], check_plugin=E["A16_CHECK_PLUGIN"] == "1")
    p = pose + [""] * (4 - len(pose))
    (x, y, z, yaw), src = m.resolve_pose(wf, E["A16_WORLD_INFO"], E["A16_SITE"], p[0], p[1], p[2], p[3])
except Exception as exc:  # noqa: BLE001
    print(f"ERROR: {exc}", file=sys.stderr); sys.exit(1)
out = lambda k, v: print(f"{k}\t{v}")  # noqa: E731
out("WORLD_FILE", wf)
out("PHYSICS", m._engine_filename(wf) or "(default)")
out("POSE", f"x={x:.3f} y={y:.3f} z={z:.3f} yaw={yaw:.3f} rad ({src})")
out("POSE_VALUES", f"{x:.6f} {y:.6f} {z:.6f} {yaw:.6f}")
g = m.world_gravity(wf, E["A16_WORLD_INFO"])
out("GRAVITY", "" if g is None else f"{g:g}")
for n in notes:
    out("NOTE", n)
if E["A16_GUI_MODE"] == "follow":
    src = open(E["A16_GUI_SRC"]).read()
    try:
        text, note = m.gui_config_for_pose(src, wf, E["A16_WORLD_INFO"], (x, y, z, yaw))
    except Exception as exc:  # noqa: BLE001
        text, note = src, f"camera as in the file ({exc})"
    if E["A16_GUI_OUT"]:
        open(E["A16_GUI_OUT"], "w").write(text)
    out("GUI_NOTE", note)
PY
}
a16_field() { printf '%s\n' "$1" | awk -F '\t' -v k="$2" '$1 == k { sub(/^[^\t]*\t/, ""); print }'; }

# ------------------------------------------------------------------ WSL crash dumps
a16_crash_dir() {    # the Linux path of %TEMP%\wsl-crashes (APOLLO16_WSL_CRASH_DIR overrides); fails if unknown
  # (every return here has an explicit status: a bare 'return' in code run from a trap handler returns
  # the status of the command before the trap, e.g. 130 after Ctrl+C)
  if [[ -n "${APOLLO16_WSL_CRASH_DIR+x}" ]]; then
    [[ -n "${APOLLO16_WSL_CRASH_DIR}" ]] || return 1
    printf '%s\n' "${APOLLO16_WSL_CRASH_DIR}"
    return 0
  fi
  local t="" d cands=()
  if command -v cmd.exe >/dev/null 2>&1 && command -v wslpath >/dev/null 2>&1 && [[ -d /mnt/c ]]; then
    # from a Windows folder: cmd.exe refuses a Linux (UNC) working directory with a warning
    t="$(cd /mnt/c && timeout 10 cmd.exe /d /c 'echo %TEMP%' 2>/dev/null | tr -d '\r' | tail -n 1)"
    if [[ "${t}" =~ ^[A-Za-z]:\\ ]]; then t="$(wslpath -u "${t}" 2>/dev/null)"; else t=""; fi
  fi
  if [[ -z "${t}" || ! -d "${t}" ]]; then   # no Windows interop: the only profile that has the folder
    for d in /mnt/c/Users/*/AppData/Local/Temp/wsl-crashes; do [[ -d "${d}" ]] && cands+=("${d%/wsl-crashes}"); done
    t=""
    (( ${#cands[@]} != 1 )) || t="${cands[0]}"
  fi
  [[ -n "${t}" ]] || return 1
  printf '%s/wsl-crashes\n' "${t}"
  return 0
}
a16_win_path() { { command -v wslpath >/dev/null 2>&1 && wslpath -w "$1" 2>/dev/null; } || printf '%s\n' "$1"; }
a16_gib() { awk -v b="${1:-0}" 'BEGIN { printf "%.2f", b / 1073741824 }'; }
a16_parse_dump() {   # a16_parse_dump NAME: sets A16_DUMP_TS A16_DUMP_PID A16_DUMP_EXE A16_DUMP_SIG
  [[ "$1" =~ ^wsl-crash-([0-9]+)-([0-9]+)-(.+)-([0-9]+)\.dmp$ ]] || return 1
  A16_DUMP_TS="${BASH_REMATCH[1]}"
  A16_DUMP_PID="${BASH_REMATCH[2]}"
  A16_DUMP_EXE="${BASH_REMATCH[3]}"
  A16_DUMP_SIG="${BASH_REMATCH[4]}"
}
a16_our_dumps() {    # a16_our_dumps DIR START: "name:size:mtime" of this run's gz/ruby dumps (one line)
  local f out=""
  for f in "$1"/wsl-crash-*.dmp; do
    [[ -e "${f}" ]] || continue
    a16_parse_dump "${f##*/}" || continue
    (( 10#${A16_DUMP_TS} >= $2 )) || continue
    [[ "${A16_DUMP_EXE}" == *ruby* || "${A16_DUMP_EXE}" == *gz* ]] || continue
    [[ "${A16_OUR_PIDS}" == *" ${A16_DUMP_PID} "* ]] || continue
    out+="${f##*/}:$(stat -c %s:%Y -- "${f}" 2>/dev/null || echo 0:0) "
  done
  printf '%s\n' "${out}"
}
a16_dumps_settled() {   # a16_dumps_settled "name:size:mtime ..." NOW: no dump was written to in the last 2 s
  local d
  for d in $1; do (( $2 - 10#${d##*:} >= 2 )) || return 1; done
  return 0
}

# a16_crash_dumps_after_run START KEEP [EXPECT]
#   START   epoch s when the run began; KEEP 1 = only list this run's dumps, 0 = delete them
#   EXPECT  "pid:epoch ..." processes of ours that died of a core-dumping signal at that time: wait up to
#           15 s after that for their dump to appear
# Waits (at most APOLLO16_CRASH_WAIT_S, default 60 s; a second Ctrl+C skips) while WSL still writes one of
# this run's dumps, then deletes or lists them. Dumps of other processes are listed, older dumps counted.
a16_crash_dumps_after_run() {
  local start="$1" keep="$2" expect="${3:-}" dir f name size e p t now pending sizes prev="" waited=0
  local deadline=$(( SECONDS + ${APOLLO16_CRASH_WAIT_S:-60} )) n_old=0 b_old=0 what
  dir="$(a16_crash_dir)" || return 0
  [[ -n "${dir}" && -d "${dir}" ]] || return 0
  while :; do
    now="$(date +%s)"
    pending=""
    for e in ${expect}; do
      p="${e%%:*}"; t="${e##*:}"
      compgen -G "${dir}/wsl-crash-*-${p}-*.dmp" >/dev/null && continue
      (( now - t < 15 )) && pending+=" ${p}"
    done
    sizes="$(a16_our_dumps "${dir}" "${start}")"
    if [[ -z "${pending}" ]]; then
      [[ -n "${sizes// /}" ]] || break                   # nothing of ours
      a16_dumps_settled "${sizes}" "${now}" && break     # no dump written to in the last 2 s
      [[ "${sizes}" != "${prev}" ]] || break             # no dump grew during the last wait
    fi
    (( SECONDS < deadline && ! ${A16_FORCE_STOP:-0} )) || break
    if (( ! waited )); then
      echo "Waiting for WSL to finish writing a crash dump of this run (at most ${APOLLO16_CRASH_WAIT_S:-60} s;" \
           "Ctrl+C skips)..."
      waited=1
    fi
    prev="${sizes}"
    a16_nap 2
  done
  for f in "${dir}"/wsl-crash-*.dmp; do
    [[ -e "${f}" ]] || continue
    name="${f##*/}"
    size="$(stat -c %s -- "${f}" 2>/dev/null || echo 0)"
    if ! a16_parse_dump "${name}" || (( 10#${A16_DUMP_TS} < start )); then
      n_old=$(( n_old + 1 )); b_old=$(( b_old + size ))
      continue
    fi
    [[ "${A16_DUMP_EXE}" == *ruby* || "${A16_DUMP_EXE}" == *gz* ]] || continue
    what="${A16_DUMP_EXE##*_} pid ${A16_DUMP_PID}, signal ${A16_DUMP_SIG}, $(a16_gib "${size}") GB"
    if [[ "${A16_OUR_PIDS}" != *" ${A16_DUMP_PID} "* ]]; then
      echo "Note: WSL crash dump ${name} (${what}) was written during this run, but not for one of its" \
           "processes; left alone."
    elif [[ "${keep}" == 1 ]]; then
      echo "Kept (--keep-crash-dumps) the WSL crash dump of this run: $(a16_win_path "${f}") (${what})"
    elif rm -f -- "${f}" 2>/dev/null && [[ ! -e "${f}" ]]; then
      echo "Deleted the WSL crash dump of this run: ${name} (${what})"
    else
      echo "WARNING: could not delete the WSL crash dump $(a16_win_path "${f}") (${what}); delete it in" \
           "Windows Explorer." >&2
    fi
  done
  if (( n_old )); then
    echo "Note: $(a16_win_path "${dir}") also holds ${n_old} older WSL crash dump(s), $(a16_gib "${b_old}") GB," \
         "not from this run (left alone; delete them if you do not need them)."
  fi
  return 0
}
