# Final documentation update (`docs`, phase 7)

Owner: docs agent, 2026-10-26, about 21:58-22:15. Scope: bring the user-facing docs up to date with the final
world (`final_build.md`), the WSL memory-leak fix (`leak_diagnosis.md`) and the headless-first launcher
(`leak_script.md`), and write a Claude Code skill for this package. Files owned: `README.md`, `docs/USER_GUIDE.md`,
`docs/DESIGN.md`, `docs/impl/integration.md`, `docs/PROGRESS.md` (heading only), `tests/test_docs.py` (not needed,
not changed), this file, and outside the package `REPO/.claude/skills/lunar-world/SKILL.md` (new) and
`REPO/.claude/skills/togo-sim-slam-nav/SKILL.md` (pointer edit).

**No Gazebo, no GPU, no Docker, no ROS launch** was started (the validation lane had the GPU; its e2e simulation
was running at 22:13 and was not touched). Everything below was checked by reading the sources, the built world's
`world_info.yaml`, `reports/static_checks.json` and `reports/validation_report.md` (read only), the scripts, and by
running `tests/test_docs.py` plus a path check. Every number in the docs comes from a cited report or file; the
few derived ones are listed in section 4.

## 1. Result

| item | result |
|---|---|
| README.md | rewritten in place, same 10 sections and anchors; every stale number replaced (section 2.1) |
| docs/USER_GUIDE.md | rewritten in place; new section 1.10 (memory and the WSL GPU driver) |
| docs/DESIGN.md | still a spec; short "as built" notes in sections 1-14 (37 lines mention "as built"), a status block in 13, an as-built budget table in 13a, an as-built file tree and stage list in 14 |
| docs/impl/integration.md | two "superseded" notes at sections 2 and 3 (the phase-3 text is kept as the record) |
| docs/PROGRESS.md | only the phase 7 heading changed |
| `.claude/skills/lunar-world/SKILL.md` | new, 188 lines |
| `.claude/skills/togo-sim-slam-nav/SKILL.md` | new 2-bullet section "Apollo 16 lunar world", corrected "Gravity is coupled" bullet |
| placeholders | 18 `<!-- FINAL_VALIDATION: ... -->` comments (README 8, USER_GUIDE 7, DESIGN 3), each next to a visible "not yet measured on the final world" / "not yet run" (section 3) |
| `tests/test_docs.py` | **17 passed** (unchanged test file); extra path check of DESIGN, integration, PROGRESS and both skills: 0 missing package paths or links |

## 2. Every changed section

### 2.1 README.md

| section | change | source |
|---|---|---|
| intro | "about 216,000 rocks" -> about 250,000 (215,677 in the core); the "commands were run on 2026-10-25" claim replaced by what was verified how (build, static and world-only checks on the final world; launcher with test doubles; leak fix with real GUI/LiDAR runs) and the "not yet measured on the final world" convention | final_build 1, leak_script 0, leak_diagnosis 4.2 |
| images | first image: the phase-3 "four real tiles" look test replaced by `img/leak_gui_workaround.png`, the final world in the GUI (inspect lighting, default camera) after 7 min with the fix. **Removed** `img/gzval_robot_paths.png`: its caption said "final height field", but it shows the phase-3 world and the old sites. Placeholder for the visual-review images and the new path figure | leak_diagnosis 4.2 |
| 1 zones | core mesh tolerance 5 mm -> 3 mm; ring: + the 6 m core-edge band of smaller rocks | config, fix_rocks 5 |
| 1 as built | 9.5 M -> 17.13 M visual (with breakdown, soft target exceeded by 7 %), 0.9 -> 0.81 M collision, 215,678 / 16,493 -> 215,677 / 16,546 rocks, 2,946 ring rocks -> 37,029 (3,278 + 33,751 band), largest rock 1.52 m, steepest ground 33.5° (talus limiter), 8/8 static checks | final_build 1, 5, 6; world_info |
| 1 lighting | earthshine at the calibrated 3e-5 (renders as nothing), ambient 0.01 | config, fix_docs 10 |
| 1 files | world 0.86 GB (GLBs 758 MB), cache 2.8 -> 3.2 GB | final_build 4 |
| 1 variants | `inspect_stock` no longer "(when built)"; fallback of `inspect` named | final_build 8 |
| 1 sites | all four sites replaced (flat (-2.00, -4.50) -22.5°, slope (23.50, -16.50) -157.5° 12.4°, rock (-4.12, -7.41) 67.5° 8.7 cm rock, turn (-3.00, 3.00) -67.5°), compass names added; new paragraph on the non-level flat corridor (-7.4° over 3 m, 9.4°, 2 rocks; 164 of 35,721 candidates; the 5° alternative at (-25.5, -4.0)) as an open decision | world_info, final_build 9, 11.2 |
| 2 requirements | memory: "about 3 GB ... GUI estimate, not measured" -> measured server 3.5-3.7 GB, GUI 3.5 GB, flat with the fix, ~7 GB of WSL's 15.9 GB, the SLAM container shares the limit; VRAM 2,379 -> 4,834 MiB whole GPU; disk 0.86 / 3.2 GB | leak_diagnosis 2, 4.3; final_build 8 |
| 3.3 build | eight -> ten stages; 4-5 min -> 7:18 from scratch with the stage split and peaks (2.4 GiB build, 2.7 GiB dtm_extract); cached-terrain time; determinism; `--force-downstream`, `--lock-timeout`; `--set` example 0.003 -> 0.0035 (the open decision, ~15.1 M); exit code 2 also when a check did not run; **`--preset fast` no longer "would replace the full world"**: it builds into `..._fast` (CR-02); tests 80 s -> 273 tests, about 4 min | final_build 4, 5, 10; build.py `apply_preset_dirs`; fix_build 1, 10 |
| 4 commands | `--pose` example moved to the new flat site | world_info |
| 4 how it starts | rewritten: `[1/2]` server first, stall (27 s first cloud on the final world), READY about 35-60 s (30-44 s earlier), `[2/2]` GUI at READY, blank 30-60 s; `--headless` + `open_gui.sh` (saves ~3.5 GB), `--gui-early`; COPY MODE recovery; placeholders for READY and GUI timings | leak_script 2, fix_runtime 2, final_build 8 |
| 4 memory (new) | RSS table (server 3.54 GB flat 7 min with LiDAR, ~3.7 GB with the rover; GUI 3.52 GB flat), GUI by content (2.1 rocks / 1.0 ground / 0.5 base), how to use less: headless + `open_gui.sh`, RViz instead of the GUI (checked: the `--rviz` config shows the RobotModel and the LiDAR remapped to `/husky/sensors/seyond/points`), the `.wslconfig` `memory=` limit as the user's choice, the open RAM decisions; placeholder for the e2e RSS | leak_diagnosis 2, 4.3, 5; PROGRESS phase 5-6; `togo_deploy/rviz/robot_sensor_checkout.rviz` |
| 4 leak fix (new) | cause in plain words, growth rates without the fix (GUI 5.5-10, server 2.7-3.5 MB/s; your 9.05 GB in ~10 min), the variable, identical images / ranges / speed, where it is set (env.sh, launcher, open_gui.sh, tools, togo_sim_wsl.sh) | leak_diagnosis 0, 4 |
| 4 robot collision | 14 meshes / 368,456 triangles -> 88 boxes within 15 mm; measured RTF 0.44-0.70 busy / 0.90 quiet on fix-round worlds; placeholder for the final quiet slope RTF | fix_runtime 1, fix_build 6 |
| 4 stopping, low memory | second Ctrl+C; Ctrl+C in the `open_gui.sh` terminal; "memory climbed steadily = leak fix missing" | leak_script 2-3 |
| 5 SLAM | slope example 1.75 m/s² at 12.3° -> about 1.8 at the 12.4° site; `COMBINED_PARAM_FILE` default chain explained from the script; start-up check "`Accel biases [xyz]:` ends near 0" | `run_live_seyond_dlio_slam.sh` lines 11-13, `docker-compose.yml` |
| 6 GUI | flat start view: "sun behind you on the right, shadows forward-left" (phase-3 site) -> faces east-south-east towards the low sun, 28° left, shadows towards you; camera check "at least 2.2 m above the ground" -> 1.9-2.3 m, line of sight clears by at least 0.37 m; GUI memory tip; placeholder for the e2e GUI result | `config/gui/apollo16.gui.config` camera comment, final_build 9 |
| 7 old launcher | pose -> new flat site; the leak fix is in `togo_sim_wsl.sh` too; spawner wording from fix_runtime 6 (one of two spawners always exits 1, a race during the ~30 s stall; both end active); marked "measured on the phase-3 world" | fix_runtime 6, PROGRESS phase 6 |
| 8 troubleshooting | new row "memory keeps growing" (env check via `/proc/<pid>/environ`, `grep -c 'rw-s.*/dev/zero' /proc/<pid>/maps` twice a minute apart, ~4/s growth in a leaking GUI); low-memory row points to it; COPY MODE row explains the cause and the full recovery (new PowerShell window); segfault row: a plain `ros2 launch` of our launch file also avoids it | leak_diagnosis 3.1, 4.1; fix_runtime 8 |
| 9 limitations | LiDAR numbers marked phase 3 + placeholder; core-edge text (3 m fade, 6 m band); pebbles 199,000 -> 199,131; new: ring rocks collide up to 25 cm early (FID-08); "rock_mu 0.6 not used" removed (key deleted); "a fix round is in progress" -> artefacts fixed; earthshine "1 % ... 100-1000 times too bright" -> calibrated, ~1 grey level (estimate) + placeholder; RTF, memory, load time, visual budget updated | final_build 3.3, 6, 11; fix_rocks, fix_terrain, fix_docs 6 |
| 10 more docs | + final_build, leak_diagnosis, docs_final, the skill; integration and gz_validation marked phase 3 | |

### 2.2 docs/USER_GUIDE.md

| section | change |
|---|---|
| intro | sources extended; final world named; the *phase 3* and "not yet measured on the final world" conventions |
| 1.1 table | in-place turns marked phase 3; rocks 215,678 + 2,946 -> 215,677 + 3,278 + 33,751 band; lighting earthshine 0.01 -> 3e-5; controller start: 26 + 33 launches without a spawner failure; ready to drive 35-40 s -> about 35-60 s (30-44 s earlier); robot collision "88 boxes in place of the 14 CAD meshes"; **new rows** "memory of the Gazebo GUI" (old 0.62 GB, new 3.5 GB; server 3.5-3.7 GB) and "memory growth without the leak fix" (old 0.46 MB/s after ~3 min; new 5.5-10 / 2.7-3.5 MB/s) |
| 1.2 gravity | slope example 1.75 -> about 1.8 at 12.4°; flat drive numbers marked phase 3 (corridor rose 3°), the final corridor falls 7.4°: placeholder; bouncing marked phase 3; slopes: phase-3 12.3° climb + fix-round 10.4° climb (90 %), final slope site placeholder |
| 1.4 launch sequencing | "this 9.5 M-triangle world ... 25 s" -> phase 3 25 s, final world 27 s first cloud; gz guard in step 1; `--headless`/`open_gui.sh` in step 5; measured start-up generalised to the earlier worlds with sources; fix-round launch counts and the one GPU-context failure; placeholder |
| 1.5 lighting | earthshine row 0.01 -> 3e-5 (both modes); shadow text: 3 grey levels were measured with the old 1 % earthshine, ~1 expected now |
| 1.7 terrain and rocks | statistics with the final ratios; new paragraph on the talus limiter (47.6° -> 33.5°) and crater shapes; zone table (core fade 3 m, ring band); rock counts 16,546 / 199,131; contact facts (251,001 re-seated, 0 hovering, 5 almost fully buried just under the surface) |
| 1.8 sites | table replaced; flat-site paragraph (rules, corridor, alternatives, open decision); the old sites listed for reference |
| 1.9 what stayed the same | "nothing outside src/lunar_world_gen was changed" was no longer true: now names the one `MESA_EXTENSION_OVERRIDE` line in `src/togo_sim_wsl.sh` (user's request) |
| 1.10 (new) | memory and the WSL GPU driver: GUI 3.5 GB by content, server 3.5-3.7 GB, old world 0.62 GB; the leak and the fix in plain words; placeholder for the e2e RSS |
| 2.2 earthshine | rewritten from "too bright, proposed 3e-5" to "applied 3e-5, why it renders as nothing, what changed in the images (phase-3 3 grey levels with 1e-2; ~1 expected)"; the obsolete test note removed; placeholder |
| 2.3 regolith | measured on the final world: dz median -20.3 mm, 1st-99th percentile 6-34 mm (was 7-33); pebbles 199,185 -> 199,131; rocks 16,493 -> 16,546; `rock_mu` removed; new bullet on ring-rock collision dilation |
| 2.4 LiDAR | marked phase 3; core-edge softening; placeholder |
| 2.5 camera notes | artefacts "the fix round addresses some" -> fixed (with the reports), visual review placeholder |

### 2.3 docs/DESIGN.md (spec kept; "as built" notes)

| section | as-built note |
|---|---|
| header | what "as built" means; notes describe the code where they disagree with the original text |
| 1 hard rules | the one exception outside the package (`MESA_EXTENSION_OVERRIDE` in `togo_sim_wsl.sh`) |
| 2 zones | FID-04 core-edge rock band |
| 3 height function | H formula with `+ T`; two fBm gains (FID-06: 0.109 / 0.578; best single gain 0.222 misses 3 lags); thread-invariant FFT; the talus term T (input, relaxation, sampling, 32° limit, as-built numbers); core fade 3 m; cross-zone consistency includes T; two-gain site-aware calibration, recalibration rule, as-built ratios |
| 4 craters | FID-05 crest-exact rims (k = 0.5 hp, hp solved); FID-16 shape variation |
| 5 rocks | FID-02 smooth size-gated enhancement; FID-09 overlap repair; FID-04 edge band; FID-01 rest facets + settling, tilt 8°; FID-12 burial [0, 1] + bedding; contact rule and the `contact` stage; FID-11 wedge fillets |
| 6 meshes | core 3 mm (8.74 M) and the other tolerances; FID-07 seam band; ground TINs before contact |
| 7 collision | core tolerance 1.5 cm; as-built counts; **max-edge refinement implemented but off, with the measured reason** (climb 0.83-0.85 -> 0.34-0.73; simplified robot no RTF gain); FID-08 ring footprint dilation and its price |
| 8 SDF | earthshine 3e-5 (FID-14), ambient 0.01; far/horizon `cast_shadows false`; the fourth variant `inspect_stock` |
| 10 plugin | incline 8/8, turn tracking, dlopen check and fallback |
| 11 spawn | turn site 10 cm; final sites; the flat rule ignores the corridor (open decision) |
| 12 launch | launch: READY check + ready marker, gz guard (SIGKILL), plugin fallback, simplified robot collision (the slope RTF fix); wrapper: `env.sh` with the leak fix, headless-first start, `open_gui.sh`, Ctrl+C/crash dumps/low-memory warning, options; "Unchanged" bullet replaced with the fix_docs 8 wording (DLIO gravity exception) plus the `togo_sim_wsl.sh` line |
| 13 acceptance | calibration bullet: one gain 0.278 (phase 2) -> two gains; new "Status of the final world" block (static 8/8, world-only bench, robot phase 3 23/25, final robot/LiDAR/visual placeholders) |
| 13a budgets | as-built soft/hard table (rocks 6.6/7.5 M, visual 16/20 M, collision 1/1.5 M) with final values; why 17.1 M; measured load/RSS/VRAM vs the fits; memory over time needs the leak fix |
| 14 layout | as-built additions (`talus`, `recontact`, `refine`, `locate`, tools, config subfolders, scripts); the ten stages and the new build options |

### 2.4 Other files

- `docs/impl/integration.md`: a quoted note at the top of section 2 ("Superseded ... ten stages, new options,
  per-preset folders; see build.py, README 3.3, final_build 2/4; kept as the phase-3 record") and a one-line note
  at section 3. Nothing else changed.
- `docs/PROGRESS.md`: heading `## 2026-10-26: phase 7, final validation (started 00:01, stopped at 00:11: the user
  called it a day; restarted at about 21:55 on 2026-10-26, running)`. The phase 6 results, the to-do list and the
  open decisions are unchanged.
- `REPO/.claude/skills/lunar-world/SKILL.md` (new): frontmatter (name, description with the triggers: building,
  regenerating, tuning, running, debugging, validating; RAM/GPU/crash-dump/GUI-window problems of Gazebo in WSL);
  what it is; where things are; run (wrapper, `open_gui.sh`, `env.sh`, READY, RAM, SLAM gravity config); build
  (presets, stages, `--set`, `--force`, `--status`, outputs and caches under `~/togo_assets/lunar/`, ~7.3 min);
  package map and the config keys that matter; validation tools; 12 hard-won lessons (SIGKILL only and crash
  dumps; `GALLIUM_DRIVER=d3d12`; the `MESA_EXTENSION_OVERRIDE` leak fix and how to check it; WSLg COPY MODE;
  MSYS path conversion and `wsl.exe --exec`; sensors render only when subscribed; the patched Bullet plugin and
  its fallback; the collision max-edge cap hurts climbing; DLIO gravity coupling; merged rocks / one missing mesh;
  the start-up stall; measuring pitfalls); rules (no git; only inside the package unless the user agrees; one
  Gazebo at a time; assets outside OneDrive; isolated partition/domain/ROS_HOME; no unverified claims). Links to
  the docs instead of copying them.
- `REPO/.claude/skills/togo-sim-slam-nav/SKILL.md`: new section "Apollo 16 lunar world" after "Simulation setup
  specifics" (see the lunar-world skill; `run_apollo16_sim.sh` is its launcher; gravity 1.62, so DLIO uses
  `config/slam/seyond_dlio_graph_lunar.yaml` via `COMBINED_PARAM_FILE`), and the "Gravity is coupled" bullet now
  names the lunar copy instead of suggesting to change the shared file to 1.62. Nothing else changed.

## 3. Placeholders for the final validation

`grep -n FINAL_VALIDATION README.md docs/USER_GUIDE.md docs/DESIGN.md` lists them; each sits next to visible text
that says **not yet measured on the final world** (README, USER_GUIDE; sometimes wrapped over two lines, so also
`grep -n "not yet"`) or "not yet run" (DESIGN 13). Replace both.

| file:line (at 22:13) | what goes there |
|---|---|
| README:22 | visual-review images (four sites + overview) and the robot paths at the final sites |
| README:250 | READY time with the launcher, default mode and `--headless` (e2e) |
| README:253 | READY -> terrain visible in the GUI (e2e) |
| README:279 | e2e RSS of GUI and server over 15+ min (both modes), `rw-s /dev/zero` counts |
| README:314 | quiet-machine slope RTF, simplified robot collision |
| README:463 | e2e GUI result: title, time to terrain, camera tour (far views, zoom-ins), `open_gui.sh --close` / reopen |
| README:524 | LiDAR captures and checks |
| README:552 | visual review findings (shadow grey levels, artefacts, horizon) |
| USER_GUIDE:64 | standard drive at the final flat site |
| USER_GUIDE:70 | slope climb, rock traverse, turn tracking at the final sites |
| USER_GUIDE:116 | READY and controller activation, spawner failures |
| USER_GUIDE:226 | e2e RSS |
| USER_GUIDE:268 | measured shadow / sunlit grey levels |
| USER_GUIDE:306 | LiDAR captures and checks |
| USER_GUIDE:318 | visual review findings |
| DESIGN:296, 298, 300 | LiDAR checks; robot acceptance per site and quiet RTF while driving; visual review |

The summary table of USER_GUIDE 1.1 and README section 9 ("The final world has not been measured yet") carry no
comment of their own (comments inside table cells were avoided except README:279); update them with the same
numbers.

## 4. How the numbers were checked; derived values

- Final counts, sites, budgets, timings: `final_build.md` 1, 4, 5, 8, 9, 10 and the built `world_info.yaml`
  (read in WSL: sites, `triangle_counts`, `rock_counts`, `budgets`). They agree.
- Static results: the built `reports/static_checks.json` and `validation_report.md` (dz p1 / p99 -34.2 / -6.4 mm,
  used for USER_GUIDE 2.3).
- RAM, leak rates, by-content split, fps: `leak_diagnosis.md` 0, 2, 4.2, 4.3, 5 and PROGRESS phases 5-6.
- Launcher flow and options: the scripts' header comments (`run_apollo16_sim.sh`, `open_gui.sh`), `env.sh`,
  `apollo16_lib.sh` (notice and memory-warning text), `leak_script.md`.
- SLAM command: `lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh` lines 11-13 (`COMBINED_PARAM_FILE`
  defaults `SLAM_PARAM_FILE` and `DLIO_PARAM_FILE`), `docker-compose.yml` (`./src` -> `/home/er4-user/ws/src` in
  `dev` and `hw-dev`), repo `README.md` (`docker compose up dev -d`, `exec dev terminator`).
- `.wslconfig`: `%UserProfile%\.wslconfig` exists and is empty; the PC has 31.9 GiB, so WSL's default limit is
  half (15.9 GB, as in the task). `memory=` and `maxCrashDumpCount=` are documented `[wsl2]` keys; nothing was
  applied (machine-wide settings are the user's choice).
- Derived (not measured as such): 8.2 x sin(12.43°) = 1.77 m/s², written "about 1.8"; "about 5.4 minutes" with the
  terrain cached = the sum of the other nine stage times of the final build (322-325 s); VRAM "about 2.4 GiB" =
  4,834 - 2,379 MiB (final_build section 1 writes "4.83 GiB" for the same 4,834 MiB, a unit slip; the docs use the
  MiB values); "together about 7 GB" = 3.5 + 3.5-3.7 GB; compass names of the site headings from the ENU yaw; the
  fit predictions in DESIGN 13a (16.8 s, 2.2 GB, 1.1-1.4 GiB) from the 13a formulas at T = 17.13; the flat-site sun
  geometry (28° left) from the camera comment written by `make_apollo16_gui_config.py`.
- The `--rviz` claim (RViz shows the LiDAR and the robot model, not the world meshes): `togo_deploy/rviz/
  robot_sensor_checkout.rviz` has Grid, RobotModel, two camera images and three PointCloud2 displays; the launch
  remaps `/iv_points` to `/husky/sensors/seyond/points`. Its memory was not measured.

## 5. Tests

| check | result |
|---|---|
| `~/.venvs/lunar_world/bin/python -m pytest tests/test_docs.py -q` (WSL, from `PKG`) | **17 passed** (first run after the README/USER_GUIDE rewrite, and again after all edits) |
| path check with the test's own regex, extended to `DESIGN.md`, `integration.md`, `PROGRESS.md` and both skills (`$PKG/` prefix allowed), plus `terrain/...py`-style module paths and Markdown links | 0 missing; 2 false positives in pre-existing text (`tools/bench_world.GzServer` in integration.md is a module attribute; the togo-sim-slam-nav skill's `scripts/togo/...` are relative to `lidarslam_ros2`) |
| the repo-level paths named in the skills (`run_live_seyond_dlio_slam.sh`, `seyond_dlio_graph.yaml`, `src/togo_sim_wsl.sh`, `docker-compose.yml`, `config/cyclonedds.xml`) | all exist |

No test encoded anything that changed, so `tests/test_docs.py` was not edited.

## 6. Problems, open points, not verified

1. **Not verified by running:** none of the README commands were run by me (no Gazebo, no Docker). The launcher's
   real end-to-end run, the robot tests, the LiDAR checks and the visual review are the validation lane's; their
   results are placeholders. The SLAM container command was checked against the script and the compose file only.
2. **Shadow brightness (~1 grey level)** is fix_docs' estimate for the calibrated earthshine, never measured; it is
   marked as an estimate with a placeholder.
3. **Wrapper header vs banner** (`scripts/run_apollo16_sim.sh`, not my file): the `--help` header still says READY
   comes after "about 35-40 s", the runtime banner says "about 35-60 s". The docs use 35-60 s. Proposal below.
4. **Config comment** (`config/apollo16_core.yaml`, not my file): `mesh.core_visual_max_error_m: 0.003  # finer
   ground detail (was 5 mm; core ground ~4.7 M triangles, within budget)` is stale: the core ground is 8.74 M and
   the visual total exceeds the soft budget. Proposal below.
5. **`rocks/__init__.py` docstring** lists no `recontact` module (code owner).
6. **Removed figure:** `img/gzval_robot_paths.png` is no longer shown in the README (it showed the old sites as
   "final"); the file stays in `docs/impl/img/` and in `gz_validation.md`.
7. **Phase-3 numbers kept, labelled:** LiDAR detection rates, bouncing, the old-launcher behaviour (section 7) and
   the controller timing were measured on the phase-3 or fix-round worlds; they are marked as such.
8. `src/togo_sim_wsl.sh` still ends with the stray "```" line (user decision pending; not touched, not mentioned in
   the user docs).
9. The `--preset fast` numbers (3.65 M triangles) are from the phase-3 code; that preset was not rebuilt.

## 7. Proposals (not applied)

| proposal | owner | cost / benefit |
|---|---|---|
| Change the `run_apollo16_sim.sh` header line "(about 35-40 s)" to "(about 35-60 s)", like its banner and the README | runtime scripts | one comment line; `--help` then matches what the user sees; `test_docs` is unaffected (it reads options only) |
| Refresh the `mesh.core_visual_max_error_m` comment in `config/apollo16_core.yaml` (8.74 M core ground, 17.13 M visual, over the 16 M soft target; 3.5 mm ~15.1 M, 4 mm ~13.7 M) | orchestrator (config) | comment only; no stage key changes if comments are not hashed (the build hashes config values, not comments; not verified) |
| After the final validation: replace the 18 placeholders and the visible "not yet measured" texts, add the e2e and robot-test figures, and update the USER_GUIDE 1.1 rows and README section 9 | later docs agent | about 30 min; `grep -n "FINAL_VALIDATION\|not yet"` finds every spot |
| If the user decides on 3.5 / 4 mm or simpler pebbles, or moves the flat site: rerun the build, then update README 1 (as built, sites), 2 (RAM), 6 (`make_apollo16_gui_config.py`), USER_GUIDE 1.1/1.8/1.10, DESIGN 6/11/13a and the skill's numbers | whoever applies the decision | the numbers to change are all in `world_info.yaml` and the new build report |

## 8. Reproduce (WSL)

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/togo_ws-slam-nav/togo_ws-slam-nav/src/lunar_world_gen
~/.venvs/lunar_world/bin/python -m pytest tests/test_docs.py -q          # 17 tests, about 1 s
grep -n "FINAL_VALIDATION" README.md docs/USER_GUIDE.md docs/DESIGN.md   # 18 placeholders
grep -n "not yet" README.md docs/USER_GUIDE.md docs/DESIGN.md            # their visible markers
cat ~/togo_assets/lunar/worlds/apollo16_core/world_info.yaml             # the numbers behind sections 1 / 1.8
```

## 9. Cleanliness

No git command. No Gazebo, ROS launch, Docker or GPU use; no process was started except `pytest`, `python3 -I`
(the path check, from the session scratchpad) and read-only shell commands; none is left. `%TEMP%\wsl-crashes`
was empty before (21:5x) and after (22:13). Files written: the seven listed at the top and this report; nothing
in the built world folder. The validation lane's simulation that was running at 22:13
(`run_apollo16_sim.sh`, gz pid 34796) was not touched.

## 10. Final pass (2026-10-27, about 00:10-00:25)

Owner: docs-final agent. Scope: replace the placeholders of section 3 with the measured numbers of `e2e_launch.md`
and `final_validation.md`, add a short status and a known-limitations list, write the phase 7 results, update the
skills, and make two one-line script fixes. Nothing was measured anew. No Gazebo, ROS launch, GPU use or git. The
user's own simulation (`run_apollo16_sim.sh --headless` + `open_gui.sh`, started about 23:58) was running the whole
time and was not touched.

| file | change |
|---|---|
| `README.md` | intro: status "ready for SLAM testing" and what was validated; image placeholder removed (no visual-review images exist); 1 stock turns 40-66 %; 2 memory and VRAM (e2e 3, 9); 4 READY 37-47 s, window 1-2 s and terrain 35-41 s after READY, title `Gazebo Sim (Ubuntu)`, memory table from e2e, slope RTF 0.93-0.99 and all drives 0.93-1.0, topic rates and frames, Ctrl+C 2-3 s; 6 the e2e GUI result; 9 a six-item list of the main limitations ahead of the details, final LiDAR checks, RTF and memory bullets; 10 rows for `final_validation.md` and `e2e_launch.md` |
| `docs/USER_GUIDE.md` | intro; 1.1 rows (in-place turns, controller start, ready to drive, GUI memory, shutdown); 1.2 flat drive, airborne, slope / rock / turn; 1.3 stock turns; 1.4 READY and controllers; 1.10 RAM; 2.2 shadow estimate marked not measured; 2.4 LiDAR on the final world; 2.5 visual review deferred |
| `docs/DESIGN.md` | 13 status: the three "not yet run" lines (LiDAR checks; robot 9 of 10 with the slope failure; visual deferred) |
| `docs/PROGRESS.md` | phase 7: heading, results, what was cut and why, open decisions (new: slope criterion; the flat site needs no change), Next |
| `REPO/.claude/skills/lunar-world/SKILL.md` | status line, newest reports, READY / GUI / Ctrl+C numbers, RAM, RTF advice, known limits, the "not measured" rule |
| `REPO/.claude/skills/togo-sim-slam-nav/SKILL.md` | one status bullet in "Apollo 16 lunar world" |
| `scripts/run_apollo16_sim.sh` | `--help` header: READY "about 35-40 s" -> "about 35-60 s", as the banner says |
| `scripts/apollo16_lib.sh` | `a16_gui_notice`: title "Gazebo Sim (Ubuntu)" (WSLg appends the distro name), the sentence re-wrapped over its three lines; no test checks the title, and the strings `test_sim_scripts.py` checks are unchanged (test file not edited) |

Values that differ from the task's list, after checking them against the reports:
- RTF with the GUI open: 0.82-0.92, not 0.82-0.93. The 0.93 (0.926) is e2e 9's no-GUI value; with a GUI it was
  0.82-0.918.
- READY: 37-47 s, not 39-47 s. The 12 robot-test launches had READY 37.0-47.3 s after the launch start
  (final_validation 1.1); the 3 launcher runs 38.9-44.4 s after the command (e2e 2).
- Creep: 2-4 mm/s on 4-5.5° ground (e2e 5: 2-3 mm/s on 4-5°; final_validation 6: 4.4 mm/s at the flat site with the
  rover pitched 5.5°, 2.8 mm/s at level spot A).
- Smooth inclines 91-95 % means 10-15° (0.953 / 0.938 / 0.913); 17° and 19° gave 0.895 / 0.837.

Tests: `~/.venvs/lunar_world/bin/python -m pytest tests -q` (WSL, from `PKG`): **302 passed** in 363 s. Two
docs edits (the stock-turn range) landed during that run, so `tests/test_docs.py` was run again on the final files:
17 passed. Both scripts pass `bash -n`; all edited files keep LF line endings.
