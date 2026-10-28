# Start-up after READY, `open_gui.sh`, crash-dump handling (leak round, `script`)

Owner: script agent, 2026-10-25, 22:10-23:00. Files owned: `scripts/run_apollo16_sim.sh`, `scripts/open_gui.sh`
(new), `scripts/apollo16_lib.sh` (new, shared by both), `launch/apollo16_sim.launch.py` (ready marker only),
`tests/test_sim_scripts.py`, `tests/test_sim_launch.py`, `tests/test_docs.py`, the launching and GUI parts of
`README.md` and `docs/USER_GUIDE.md`, this file. Scratch: `/home/nargon/togo_assets/lunar/experiments/leak_script/`.

No Gazebo and no ROS launch was started (another agent had the GPU for the leak measurements). Everything below
was checked with dry runs, unit tests, `bash -n`, shellcheck, and end-to-end runs of the real scripts against test
doubles (a fake `ros2 launch` that writes the ready marker, a fake `gz` server and GUI that record every signal they
get). **Not checked: a real Gazebo GUI opened by the new flow** (section 7).

## 0. What changed, in short

| request | change | checked by |
|---|---|---|
| "make it launch headless, then launch the GUI" | `run_apollo16_sim.sh` starts the launch without a GUI, waits for the launch's ready marker, then opens the GUI | sandbox runs: GUI start 0.49-0.66 s after the marker (0.5 s polling), never before it |
| robust READY detection | the launch writes `ready_file:=PATH` atomically when `wait_ready` passes (`PATH.not_ready` when it fails) and deletes both at start; the wrapper passes a per-run path and waits at most `--ready-timeout` (600 s) while the launch is alive | unit tests (paths, content, atomicity with a polling reader, failure cases); sandbox runs for READY, NOT READY, timeout, launch exit |
| no READY: no GUI, say why | NOT READY (with the reason from the marker), timeout (with WSL's free memory) or the launch's exit code; the hint to open the GUI anyway | sandbox runs |
| message when the GUI opens | blank for about 30-60 s, title "Gazebo Sim", `[WARN:COPY MODE]` -> close WSL terminals and `wsl --shutdown`, Ctrl+C instead of the X button | sandbox output |
| `--headless`, `--gui-early`, keep the other options | `--headless` prints the `open_gui.sh` command; `--gui-early` = old order (GUI 8 s after the start); new `--ready-timeout S`, `--keep-crash-dumps` | dry-run tests, argument tests |
| Ctrl+C: gz with SIGKILL, ROS nodes with SIGINT | unchanged order (GUI group SIGKILL, gz guard / gz SIGKILL, then SIGINT to `ros2 launch`); a fake gz that traps TERM/INT/HUP never got one | sandbox runs (all flows) |
| `scripts/open_gui.sh` | opens the GUI for a running simulation from another terminal (session file: partition, spawn pose, ready marker), waits for READY, own process group, SIGKILL on Ctrl+C/exit, closes when the simulation stops; `--close` closes the open GUI; `--default-gui`, `--gui-config`, `--site`, `--pose`, `--no-wait` | sandbox runs: open before READY (waits), refuse a second GUI, Ctrl+C, reopen, closes with the simulation, `--close` of the wrapper's GUI |
| crash-dump hygiene | at exit both scripts delete the WSL dumps of their own ruby/gz processes written during the run (PID from the file name, recorded PIDs), list other dumps written meanwhile, count older ones; wait while WSL still writes one; `--keep-crash-dumps` | sandbox run with a growing fake dump, a foreign one, a non-gz one, an old one |
| tests, docs | 22 new wrapper / `open_gui.sh` tests, 6 new launch tests, 1 new docs test; README 4/6/8/10, USER_GUIDE 1.1/1.4 | 121 passed (section 9) |

## 1. The ready marker (launch)

`launch/apollo16_sim.launch.py` has a new launch argument `ready_file` (default empty; the environment variable
`APOLLO16_READY_FILE` is used when the argument is empty). Nothing changes when neither is set, so
`tools/run_robot_tests.py` and plain `ros2 launch` users see the old behaviour.

- **At start** (`launch_setup`): `PATH` and `PATH.not_ready` are deleted (a marker of an earlier run must never
  announce this one) and one log line names the marker.
- **At READY**: the `wait_ready` exit handler appends an `OpaqueFunction` after the READY banner (so the banner is
  printed when the marker appears) and before RViz/the point fixer. It calls `on_wait_ready_exit(rc, ready_file,
  fields)`: rc 0 writes `PATH`; any other rc writes `PATH.not_ready` with `wait_ready_exit_code` and `reason`
  ("timed out: controllers not active or /clock not advancing" for rc 2). The other marker is deleted.
- **Content**: `key=value` lines, first `status=READY` or `status=NOT_READY`, then `ready_after_s`, `launch_pid`,
  `time`, `world`, `pose`, `pose_source`, `gz_partition`, `ros_domain_id` (values on one line).
- **Atomic**: written to `.<name>.<pid>.tmp` in the same folder, `fsync`, `os.replace`; the folder is created. A
  write error never reaches the launch: it logs `WARNING: cannot write the ready marker ...`.

Functions (unit-tested without ROS): `ready_marker_paths`, `clear_ready_markers`, `write_marker_atomic`,
`write_ready_marker`, `on_wait_ready_exit`. The atomicity test runs a reader thread against 40 rewrites of a
300 kB marker: 0 partial reads; the same test against a plain `open(path, "w").write()` saw 183 partial or empty
reads out of 237 (scratch `check_atomic_test.py`), so the test does detect a non-atomic writer.

## 2. `run_apollo16_sim.sh`: the new flow

```text
[1/2] Starting the Gazebo server and ROS nodes (the GUI follows at READY)...      setsid ros2 launch ... ready_file:=<run dir>/ready
  ... launch output, [wait_for_sensors] progress, [apollo16_sim] READY banner ...
[2/2] READY after N s: starting the Gazebo GUI through WSLg...                    setsid gz sim -g --gui-config <run dir>/gui.config
  (notice: blank 30-60 s, title, COPY MODE, Ctrl+C / open_gui.sh --close, not the X button)
  ... monitor: GUI exit, memory, PIDs ...
Ctrl+C -> GUI group SIGKILL -> gz guard SIGINT (gz SIGKILL) / gz SIGKILL -> SIGINT to ros2 launch -> crash dumps
```

- **Waiting**: every 0.5 s (a `sleep & wait`, so Ctrl+C is handled at once) for `ready`, `ready.not_ready`, the
  launch's exit, or `--ready-timeout` (default 600 s, counted from the launch start). The launch's own
  `sensor_wait_timeout` is also 600 s, so a very slow start can still reach READY after the wrapper gave up; the
  wrapper then has already said that it will not open the GUI and printed the `open_gui.sh` command.
- **Results**: READY -> GUI. NOT READY -> no GUI, `[2/2] The launch reported NOT READY after N s (<reason>)`, the
  `list_controllers` check and the `open_gui.sh` command; the simulation keeps running. Timeout -> no GUI, the
  same plus WSL's available memory. Launch exited -> its exit code is the wrapper's, no GUI.
- **Modes**: default (`gui=ready`), `--gui-early` (old: GUI 8 s after the launch start, no READY wait),
  `--headless` (no GUI; the banner and `[2/2]` line print `open_gui.sh`). `--headless --gui-early` is an error.
  A `ready_file:=X` among the extra launch arguments is used instead of the per-run path (not passed twice).
- **Run folder** `/tmp/apollo16_sim-<uid>/run.<wrapper pid>/` (`APOLLO16_STATE_DIR` overrides the parent; mode
  700, must be owned by the user): `session`, `ready`, `ready.not_ready`, `gui.config` (camera at the spawn,
  replacing the old `mktemp` file). Removed at exit; entries of dead processes are pruned by the next start.
  `session` (key=value, atomic) holds `wrapper_pid`, `launch_pgid`, `world`, `world_info`, `site`, `pose`
  (resolved x y z yaw), `gz_partition`, `ros_domain_id`, `ready_file`, `gui` (ready/early/never) and `gui_state`
  (pending, open, closed, skipped, none). A dry run creates nothing.
- **The GUI process group** is registered as `/tmp/apollo16_sim-<uid>/gui.<pgid>` with the process's start time
  from `/proc/<pid>/stat`, so a reused PID is never taken for it. The wrapper notices within 2 s when its GUI ends
  while the simulation runs and says how: crash (exit 128+N of a core-dumping signal: a dump is coming and will
  be deleted), closed by SIGKILL (`open_gui.sh --close`), or normal exit; and how to reopen it.
- **Ctrl+C** is unchanged in substance. Two small fixes: the GUI is killed and reaped in one block with stderr
  silenced (bash otherwise printed `line N: PID Killed  setsid gz sim -g ...` after "Stopping"), and the 3 s wait for
  the gz guard is skipped when there is no guard process (gz gets SIGKILL at once).
- **Low memory**: every 2 s the wrapper reads `MemAvailable`; below `APOLLO16_LOW_MEM_MB` (default 3000) and again
  below half of it, one `WARNING: WSL is running out of memory: N MB left (Gazebo GUI X MB, simulation Y MB)` with
  what to do. It re-arms after memory recovers by 1 GB.
- **Old bug fixed**: `--default-gui` printed `setsid gz sim -g ''` in dry runs (printf with an empty array).

## 3. `open_gui.sh`

1. Sources `env.sh` (same GPU/WSLg environment), finds the live sessions (`run.*/session` whose `wrapper_pid` is a
   running `run_apollo16_sim.sh`). With `GZ_PARTITION` set it takes the session of that partition; otherwise the
   only one (several: error listing them). It exports the session's `GZ_PARTITION` if the terminal has none.
2. Refuses while the wrapper will open its own GUI (`gui_state=pending`: started without `--headless`, not READY
   yet) and while any registered GUI runs (one GUI at a time, about 3.5 GB each); the message names
   `open_gui.sh --close`.
3. Waits for the session's ready marker (at most `--ready-timeout`, default 600 s; `--no-wait` skips); NOT READY:
   warning, opens anyway (explicit request); the wrapper gone: error; timeout: error with `--no-wait` hint.
4. GUI config as in the wrapper (packaged config with the camera moved to the session's spawn pose; `--site` /
   `--pose` / `--world` / `--world-info` override; `--gui-config`, `--default-gui`). If the camera cannot be moved,
   a warning and the packaged file as it is.
5. Prints the same notice (blank 30-60 s, COPY MODE, Ctrl+C here, not the X button), starts `setsid gz sim -g`,
   registers it, and then every 2 s: PIDs, memory, and whether the wrapper still runs (if not: "The simulation
   stopped: closing the Gazebo GUI").
6. Exit (Ctrl+C = exit 0, terminal closed, simulation stopped, GUI ended): the GUI group gets SIGKILL, the
   registration and its config folder `open_gui.<pid>/` go, crash dumps are handled as in section 4.

Without a session (a simulation started by `ros2 launch` or `togo_sim_wsl.sh`) it opens at once in the terminal's
`GZ_PARTITION`, camera at the `flat` site of the default world. `--close` sends SIGKILL to every registered,
still running GUI group (of the terminal's `GZ_PARTITION` if set) and waits up to 3 s for it to go.

## 4. WSL crash dumps

Facts on this machine: `/proc/sys/kernel/core_pattern` is `|/wsl-capture-crash %t %E %p %s`; the files are
`%TEMP%\wsl-crashes\wsl-crash-<unix time>-<pid>-<exe path, / as _>-<signal>.dmp` (e.g.
`wsl-crash-1792966979-224896-_usr_bin_ruby3.2-11.dmp` from fix_runtime.md). `%p` is the PID as the distro sees
it, i.e. the PID the scripts know. `cmd.exe /d /c "echo %TEMP%"` from `/mnt/c` takes 0.10-0.12 s and gives
`C:\Users\Username\AppData\Local\Temp`; `wslpath -u` makes it `/mnt/c/Users/Username/AppData/Local/Temp`. Without
Windows interop the scripts take the only `/mnt/c/Users/*/AppData/Local/Temp/wsl-crashes` that exists.
`APOLLO16_WSL_CRASH_DIR` overrides the folder (empty: no crash-dump handling).

At exit (`a16_crash_dumps_after_run`):

- **This run's dumps**: name parses, crash time >= the run's start, program name contains `ruby` or `gz`, and the
  PID is one the script recorded for its own processes (both process groups' leaders at start, then every
  process of their sessions every 2 s via `pgrep -s`). These are **deleted** by default and reported with
  program, PID, signal and size; `--keep-crash-dumps` keeps and lists them.
- **Other dumps written during the run** (another PID, e.g. another agent's gz): listed as "left alone".
- **Older dumps**: one line with their number and size, never touched. Dumps of other programs (no ruby/gz in
  the name) are ignored.
- **Still being written**: if one of this run's dumps was modified within the last 2 s, or a process of ours died
  of a core-dumping signal less than 15 s ago and its dump is not there yet, the script waits (2 s steps, at most
  `APOLLO16_CRASH_WAIT_S`, default 60 s; a second Ctrl+C skips) until no dump grows. A crashed process only exits
  after the kernel has pushed its whole core into WSL's pipe, so a dump of a crash that happened earlier in the
  run is complete and costs no wait.

Why deleting by default is safe: only dumps of processes the script itself started and recorded, written
during the run, of ruby/gz programs; every such dump comes from the known gz-sim 8 crash (or a GUI closed with
the X button) and is 1.5-3.6 GB of no use to the user; WSL keeps up to 10 (30 GB). Anything not provably ours is
left alone. `--keep-crash-dumps` exists for a bug report.

A bash detail that broke this at first: in a function called from a trap handler, a bare `return` returns the
status of the command *before the trap* (130 after Ctrl+C), not of the function's last command. `a16_crash_dir`
ended with `return`, so `dir="$(a16_crash_dir)" || return 0` skipped all crash-dump handling after Ctrl+C. Every
`return` in the scripts now has an explicit status; the sandbox test caught it.

## 5. Shared helper `scripts/apollo16_lib.sh`

Sourced by both scripts (exits with an error when executed): argument helpers (`a16_need_val`, `a16_is_number`,
`a16_is_seconds`, `a16_usage`, `a16_die_usage`), process helpers (`a16_nap`, `a16_group_alive`, `a16_proc_start`,
`a16_record_pids`, `a16_core_signal_rc`), memory warning, state folder / session / GUI registration, the GUI notice,
`a16_resolve` (the world, pose and GUI-config resolution through the launch file's own functions, moved here from
the wrapper unchanged apart from the explicit plugin-check flag and the extra `POSE_VALUES` output), and the
crash-dump functions. One copy of each so that the two scripts cannot drift apart.

## 6. Tests

| file | new tests | what |
|---|---|---|
| `tests/test_sim_launch.py` | 6 | marker paths (empty, `~`, relative), success then failure content and file switch, no temporary file left, `clear_ready_markers`, `on_wait_ready_exit` for rc 0/2/3/no path/unwritable folder, atomicity with a polling reader, wiring in `launch_setup` (cleared at start from the argument or the env var; written after the READY banner, before RViz); `ready_file` declared |
| `tests/test_sim_scripts.py` | 22 (+ extended `--help`, `bash -n`) | sandbox dry runs (marker path, order of the printed commands, the user's `ready_file:=`, `--ready-timeout`, `--gui-early`, `--headless`, no `''`); new option errors (wrapper 4, `open_gui.sh` 7); `open_gui.sh` dry run and `--close` without a simulation; end-to-end flows with test doubles: GUI only after the marker, per-run camera config equal to the launch module's result, session and registration content, low-memory warning once, Ctrl+C -> no TERM/INT/HUP to any gz, the server SIGKILLed before the launch got SIGINT, nothing left; NOT READY and timeout (no GUI, simulation keeps running); launch exit 3 before READY; `--headless` + `open_gui.sh` (waits, refuses a second GUI, Ctrl+C closes only the GUI, reopens at once, closes when the simulation stops); `open_gui.sh --close` of the wrapper's GUI and reopening; crash dumps (delete / `--keep-crash-dumps`, wait for a growing dump, foreign/other/old dumps untouched); dump-name parsing and the real `%TEMP%` lookup |
| `tests/test_docs.py` | 1 | README section 6 documents exactly the `open_gui.sh` options (the wrapper's own test now also covers `--gui-early`, `--ready-timeout`, `--keep-crash-dumps`) |

Results: see section 9.

The test doubles live in the tests: `env.sh` (no ROS, fake `bin` first on `PATH`), `ros2` (re-executes a python
fake launch under the name `fake-ros2`, so that no `pgrep -f "ros2 launch"` of another tool takes it for a real
simulation), `gz` (`gz fake-server` keeps the process name `gz` the wrapper looks for; `gz sim -g` re-executes as
`fake-gz-gui`). The fake GUI "crashes" with `exit 139` and a fake dump written by a detached writer for 3 s; no real
core dump is ever made. A fixture kills any process whose command line contains the test's temporary folder.

## 7. Not verified, open points

- **Real Gazebo**: the new flow was not run with Gazebo (GPU reserved for the leak measurements). The parts that
  touch real gz are unchanged from the verified fix_runtime wrapper (same launch command plus `ready_file:=`, same
  `setsid gz sim -g [--gui-config]`, same stop order). First real run: check that `[2/2] READY after N s` follows the
  launch's READY banner within a second and that the window appears then.
- **`[WARN:COPY MODE]` is not detected**: the prefix is added on the Windows side (weston-rdprail with
  `enable_copy_warning_title = 1` in `/mnt/wslg/weston.log`), so the scripts only explain it. A PowerShell window
  lister could check it, at 1-2 s per call; not added.
- **X button**: the scripts cannot intercept it. If gz crashes, the dump is deleted at exit (it is the script's
  PID); if the GUI exits normally nothing is left.
- **Exit code** after Ctrl+C: 130 for the wrapper (as before), 0 for `open_gui.sh` (Ctrl+C is its normal end).
- **README section 2** still says the GUI memory is an unmeasured estimate (about 3 GB); the measured value is
  about 3.5 GB after loading (leak round). Not changed here (requirements, not launching); the RAM numbers are
  due after the leak fix.

## 8. How to use

```bash
cd $REPO
./src/lunar_world_gen/scripts/run_apollo16_sim.sh                 # server first, GUI at READY (35-60 s), blank 30-60 s more
./src/lunar_world_gen/scripts/run_apollo16_sim.sh --headless      # no GUI; then in a second terminal:
./src/lunar_world_gen/scripts/open_gui.sh                         # waits for READY, opens the GUI; Ctrl+C closes only the GUI
./src/lunar_world_gen/scripts/open_gui.sh --close                 # close whichever GUI is open (also the wrapper's)
./src/lunar_world_gen/scripts/run_apollo16_sim.sh --gui-early     # the old order
./src/lunar_world_gen/scripts/run_apollo16_sim.sh --dry-run       # print what would run
```

## 9. Results, reproduction, cleanliness

**Results** (22:52, WSL, project venv, while the leak agent's Gazebo runs had the GPU):

| check | result |
|---|---|
| `pytest tests/test_sim_scripts.py tests/test_sim_launch.py tests/test_docs.py tests/test_world_sdf.py tests/test_sim_robot_collision.py` | **121 passed in 67 s** (47 + 25 + 17 + 27 + 5; new: 22 + 6 + 1 + 0 + 0) |
| `bash -n` on `run_apollo16_sim.sh`, `open_gui.sh`, `apollo16_lib.sh`, `env.sh` | OK (also a test) |
| shellcheck 0.10.0 (`shellcheck-py` in a scratch venv), `-x -e SC2317` | no findings. SC2317 ("unreachable") is a false positive: shellcheck follows `env.sh`, whose top-level `return` it reads as leaving the caller |
| marker -> GUI start (sandbox) | 0.49-0.66 s (0.5 s polling; `open_gui.sh` 0.66 s including its camera config) |
| Ctrl+C -> stopped (sandbox doubles) | 0.1 s; 4.2 s when a crash dump was still being written (waited until it stopped growing). Real runs: 1.2-2.2 s (fix_runtime.md) |
| `--dry-run` on the real world | 1.35 s (resolution including the dlopen check of the physics plugin) |
| `cmd.exe /d /c "echo %TEMP%"` from WSL | 0.10-0.12 s, `C:\Users\Username\AppData\Local\Temp` |

**Reproduce** (WSL, from `src/lunar_world_gen`):

```bash
~/.venvs/lunar_world/bin/python -m pytest tests/test_sim_scripts.py tests/test_sim_launch.py tests/test_docs.py -q
scripts/run_apollo16_sim.sh --dry-run --site slope          # the launch line has ready_file:=/tmp/apollo16_sim-<uid>/run.<pid>/ready
scripts/open_gui.sh --dry-run                               # without a simulation: opens at once, camera at 'flat'
```

**Cleanliness.** No git command; no Gazebo, no ROS launch, no GPU use. Files written only in `src/lunar_world_gen`
(the list at the top) and the scratch folder (probes, test output `pt/`, the shellcheck venv `sc_venv/`). The test
doubles ran under their own `GZ_PARTITION`, state folder and crash folder; no process was left (checked by command
line), no `/tmp/apollo16_sim-*` folder exists, and the real `%TEMP%\wsl-crashes` was empty before and after (the
tests never point the scripts at it; one test only reads its path).
