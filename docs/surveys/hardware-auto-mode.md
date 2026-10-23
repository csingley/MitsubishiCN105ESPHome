# Survey: hardware AUTO mode and vendor-side thermostat affordances

Working notes on what the CN105 wire protocol and this codebase actually
expose about the heat pump's own AUTO logic, kept for ongoing research into
whether/how `dead_band_actuation` should lean on vendor hardware affordances
instead of (or alongside) driving HEAT/COOL/OFF directly from the ESP32.

**Test-zone policy (2026-10-23): any live hardware experimentation for this
research uses the back guest bedroom unit (`climate.minisplit_guest_rear`
in the deployer's `nixos-config`, currently unoccupied/manual-only), never
the kitchen unit (`climate.minisplit_main`, in daily use and the only unit
with `dead_band_actuation` actually governing comfort today).** See
`reference_nixos-config-deployment` memory and that repo's
`docs/plans/Minisplit_Comfort_Band_Architecture_Survey.md` for the full
production context this firmware is deployed into.

## Why this matters

`dead_band_actuation` currently treats the heat pump as a dumb HEAT/COOL/OFF
actuator and does all comfort/energy-saving reasoning on-device. But the
wire protocol and diagnostics suggest hardware AUTO is not a dumb single-
setpoint thermostat either — it's a firmware state machine with its own
energy-saving logic, multi-zone coordination, and (separately) continuous
compressor speed modulation. Before concluding that replacing hardware AUTO
is the best path, it's worth understanding what hardware AUTO is actually
doing, since there may be a way to cooperate with it rather than override it.

## The wire protocol has no dual-setpoint concept

Confirmed in `components/cn105/cn105_types.h`: the settings packet carries a
single `TEMP` byte/enum (`TEMP[16]` / `TEMP_MAP[16]`, 16–31°C in 1°C or
0.5°C steps depending on packet). There is no second setpoint field on the
wire.

`dual_low_target` / `dual_high_target` in `heatpumpSettings`
(`cn105_types.h:142-143`) are ESPHome-side state only — populated from
`getTargetTemperatureLow()`/`getTargetTemperatureHigh()` in
`heatpump_profile.cpp`, never read off the wire. This is the structural
mismatch motivating `dead_band_actuation`: `supports.dual_setpoint` is a
Home Assistant trait/UI affordance layered on top of a single-setpoint
hardware protocol.

The existing fallback, `HeatPumpProtocol::deadband_setting()`
(`heatpump_profile_protocol.h:28`), collapses `{low, high}` to one wire value
once, when a setpoint changes (clamping current temperature into the band,
or passing it through if already inside). It does not re-evaluate as the
room drifts — that's the gap `dead_band_actuation` fills by re-running
comparable logic continuously and commanding the wire mode directly.

## Hardware AUTO is not a dumb thermostat

Evidence that hardware AUTO has nontrivial internal logic, independent of
whatever `dead_band_actuation` does on-device:

- **`AUTO_SUB_MODE`** (`cn105_types.h:100-125`) — a diagnostic field decoded
  from status byte `data[5]`, exposed via `auto_sub_mode_sensor`
  (`extraComponents.cpp:241`, decoded in `heatpump_profile.cpp:235-247`).
  Two distinct protocol revisions have been observed:
  - Older units (4-state enum): `AUTO_OFF`, `AUTO_COOL`, `AUTO_HEAT`,
    `AUTO_LEADER`. `AUTO_LEADER` is unexplained but suggestive of multi-zone
    coordination (e.g. one indoor unit on a shared outdoor unit directing
    siblings).
  - Newer MFZ units (bitfield): `AUTO_INACTIVE` (0x40, AUTO not selected),
    `AUTO_IDLE` (0x41, AUTO selected, compressor not currently engaged),
    `AUTO_ACTIVE` (0x43, compressor has engaged at least once this session
    — **sticky**, stays set after the compressor cycles back off). Does
    *not* distinguish current heat-vs-cool direction; that still has to be
    inferred from setpoint vs. room temperature. Bits 2–5 are unprobed.
- **ISU code 110, "Auto Mode Logic"** (`README.md:1232-1237`) — a
  hardware-side installer setting with two options, "Energy Saving"
  (default) vs. "Comfort / Performance". This is a tunable parameter
  controlling how AUTO decides to switch direction — further evidence AUTO
  is a real decision algorithm, not a passthrough, and that the vendor
  already encodes a comfort/energy-savings tradeoff knob we are trying to
  reimplement independently.
- **`compressor_frequency_sensor`** (`README.md:285,811`) — exposes the
  inverter's actual compressor frequency, which varies continuously and
  independently of HEAT/COOL/IDLE direction. This is the modulation signal:
  whichever side (hardware AUTO or `dead_band_actuation`) picks the
  direction, the compressor's speed within that direction is modulated by
  the unit's own control loop, not commanded by ESPHome either way.

## Open questions for next steps

1. Does ISU 110 ("Energy Saving" vs "Comfort/Performance") give us a lever
   to tune hardware AUTO's own behavior that's worth exposing/documenting,
   independent of `dead_band_actuation`?
2. Can `AUTO_SUB_MODE` (specifically `AUTO_ACTIVE`'s stickiness, and the
   unprobed bits 2-5) be decoded further to determine hardware AUTO's
   current direction without inferring it from room/setpoint deltas?
3. What does `AUTO_LEADER` actually mean, and is it relevant to the
   cross-zone veto (`set_allow_heat()`/`set_allow_cool()`) that
   `dead_band_actuation` added — i.e. is there a vendor-native multi-zone
   coordination mechanism we're duplicating?
4. Is there any way to keep hardware AUTO engaged (for its own modulation
   and multi-zone logic) while still enforcing the `{low, high}` band from
   ESPHome — e.g. by nudging the single wire setpoint dynamically rather
   than overriding mode/power outright? This would be a middle path between
   today's one-shot `deadband_setting()` and full mode/power takeover.

## Full UART instrument/control catalog

Bottoms-up survey of everything the CN105 protocol, as implemented in this
codebase, lets us read or write. All packets are 22 bytes (`PACKET_LEN`,
`cn105_types.h:36`), command byte 2, payload from byte 5.

### 1. Packet types

Request codes sent in an info-request packet's byte 5 (`createInfoPacket()`,
`hp_writings.cpp:313-323`), registered with the scheduler in
`registerInfoRequests()`/`registerHardwareSettingsRequests()`
(`cn105.cpp:94-195`):

| Code | Purpose | Status |
| --- | --- | --- |
| `0x02` | Settings (power/mode/temp/fan/vane/widevane/humidity/airflow) | implemented |
| `0x03` | Room temperature / outside air temp / runtime hours | implemented |
| `0x04` | Error info | implemented (`getErrorInfoFromResponsePacket`, `hp_readings.cpp:117`) |
| `0x05` | Timers | registered but `disabled = true` (`cn105.cpp:131-133`); response case is a no-op warning (`hp_readings.cpp:138-142`, "not implemented") |
| `0x06` | Status: operating flag, compressor frequency, input power, energy usage | implemented |
| `0x09` | Standby/power sub-modes, stage, auto_sub_mode | implemented |
| `0x20`/`0x22` | Function/ISU codes part 1 & 2 (read) | implemented, gated on `AUXILIARY_CONTROLS` + configured `hardware_settings` |
| `0x42` | HVAC auxiliary options (air purifier, night mode, circulator) | implemented |
| `0x10` | Seen on the wire, decode not implemented (`hp_readings.cpp:149-151`, "[0x10 is Unknown : not implemented]") | **frontier** |

Write/control packets (command byte fixed, content built by `profile_->encode_*`):

| Packet | Builder | Purpose |
| --- | --- | --- |
| Connect (`0x5A`/`0x5B`) | `sendFirstConnectionPacket()` (`hp_writings.cpp:11-57`) | handshake; `0x5B` = installer mode, falls back to `0x5A` if unanswered |
| Set (`0x..` via `HEADER`) | `createPacket()` → `encode_control()` (`hp_writings.cpp:211-216`) | power/mode/temp/fan/vane/widevane |
| Remote temp (`0x07`) | `encode_remote_temperature()` via `sendRemoteTemperaturePacket()` (`hp_writings.cpp:326-384`) | external temperature sensor injection |
| Run states (`0x08`) | `encode_run_states()` via `sendWantedRunStates()` (`hp_writings.cpp:393-414`) | air purifier / night mode / circulator / airflow control |
| Functions set (`0x1F`/`0x21`) | ISU code writes (see §3) | installer-level hardware settings |

### 2. Readable instruments

- **Settings packet `0x02`** (`heatpump_profile.cpp:33-169`): `data[3]` power,
  `data[4]` mode (+ iSee flag when `>0x08`), `data[6]` fan, `data[7]` vane,
  `data[10]` widevane low nibble (`&0x0F`) + "adjust" flag (high nibble
  `==0x80`), `data[12]` target humidity (premium models only, 1-100 raw,
  `heatpump_profile.cpp:122-138`), `data[14]` airflow control (only valid
  when `data[10]==0x80` and iSee active, `heatpump_profile.cpp:140-165`).
- **Status packet `0x06`** (`heatpump_profile.cpp:172-205`, byte map
  documented inline from real captures): `data[4]` operating (1=compressor
  running), `data[5..6]` input power (16-bit W), `data[7..8]` energy usage
  (kWh = value/10, counter width/rollover unknown — flagged TODO at
  `heatpump_profile.cpp:181-182`). Compressor frequency itself is decoded
  separately via `HeatPumpProtocol::compressor_frequency()`
  (`heatpump_profile.cpp:195`), with an opt-in `report_when_idle` flag
  because some models (PAA/PUZ combo) report noise on this byte while idle.
- **Room/temp packet `0x03`** (`CommonProtocol::decode_room_status`,
  `profile_protocol.h:10-21`): `data[5]` outside air temp
  (`(data[5]-128)/2`, only valid if `>1`), `data[6]` room temp
  (`(data[6]-128)/2`, falls back to coarse `ROOM_TEMP_MAP` lookup on
  `data[3]` if `data[6]==0`), `data[11..13]` runtime hours (24-bit,
  minutes/60).
- **Standby/sub-mode packet `0x09`** (`heatpump_profile.cpp:207-278`):
  `data[3]` → `sub_mode` (NORMAL/WARMUP/DEFROST/PREHEAT/STANDBY/OFF,
  `cn105_types.h:97-101`), `data[4]` → `stage` (IDLE/LOW/GENTLE/MEDIUM/
  MODERATE/HIGH/DIFFUSE, `cn105_types.h:94-95`), `data[5]` →
  `auto_sub_mode` (see survey above; old 4-state enum vs. newer bitfield,
  `cn105_types.h:103-125`).
- **Error packet `0x04`** (`CommonProtocol::decode_error`,
  `profile_protocol.h:23-31`): `data[4]&0x7F` code, `data[5]` sub-code; bit 7
  of `data[4]` is documented as "announces error-reporting availability,
  rather than an error" — i.e. not itself an error flag, a capability bit.
- **HVAC options packet `0x42`** (`heatpump_profile.cpp:280-314`, byte map
  from an MSZ-LN25VG2W capture): `data[1]` air purifier, `data[2]` night
  mode, `data[3]` circulator — comment flags circulator "MIGHT BE SAME BYTE
  AS ECONOCOOL — NEEDS TESTING!" (`heatpump_profile.cpp:286`), **frontier**.
- **Diagnostic sensors derived from the above**: `compressor_frequency_sensor`,
  `input_power_sensor`, `kwh_sensor`, `runtime_hours_sensor`,
  `outside_air_temperature_sensor`, `isee_sensor`, `stage_sensor`,
  `sub_mode_sensor`, `auto_sub_mode_sensor`, `error_code_sensor`,
  `target_humidity_sensor` (`hp_readings.cpp:216-255`, `extraComponents.cpp`).

### 3. Writable controls

- **Core climate** (`heatpump_profile_protocol.h:273-310`, `encode_control`):
  power, mode, temperature (half-degree via `TEMP_B`/`use_temperature_encoding_b_`
  or legacy `TEMP` table), fan, vane — each gated behind its own dirty-bit in
  `CONTROL_PACKET_1`/`CONTROL_PACKET_2` (`cn105_types.h:68-69`) so only
  changed fields are sent.
- **Widevane / airflow control / run states** (`heatpump_profile_protocol.h:
  225-241`, `encode_run_states`): widevane byte, `RUN_STATE_PACKET_1[4]` for
  airflow control index, `RUN_STATE_PACKET_2[1..3]` for air purifier / night
  mode / circulator — same dirty-bit pattern via `RUN_STATE_PACKET_1`/`_2`
  (`cn105_types.h:70-71`).
- **Remote temperature** (`hp_writings.cpp:326-384`): external temp
  injection, debounced (min interval = half the keep-alive interval, floor
  5s) with a one-time warning if the caller is double-driving it.
- **Function/ISU codes** (`heatpump_functions_protocol.cpp`,
  `heatpumpFunctions.h`/`.cpp`): generic 101-128 code space, 2 bits of
  encode per code byte (`getCode`/`getValue`, `(b>>2)+100` / `b&3`), 15
  codes per 0x20/0x22 packet half (`MAX_FUNCTION_CODE_COUNT` = 30 total).
  This is a **fully generic transport** — any code 101-128 can be read/set
  by `hardware_setting_select`, but only a subset is documented/labeled in
  README (`README.md:1174-1285`): 101 (auto restart), 102 (temp sensing
  source — indoor/remote/external), 103 (ventilation/Lossnay link), 105
  (auto energy saving), 107 (filter sign interval), 108 (ceiling
  height/static pressure), 109 (air outlets), **110 (auto mode switching
  logic: energy-saving vs. comfort/performance)**, 111 (vane geometry), 117
  (defrost logic), 124 (heating offset), **125 (fan behavior during
  thermo-OFF, heating)**, **127 (fan behavior during thermo-OFF, cooling)**,
  128 (error display). Codes 104, 106, 112-116, 118-123, 126 are in-range
  but undocumented/unexplored — **frontier**.

### 4. Multi-zone / vendor tuning hints

- `AUTO_LEADER` (old-protocol `auto_sub_mode` enum value, `cn105_types.h:125`)
  — unexplained, name strongly suggests one indoor unit on a shared outdoor
  unit signaling a coordinating role. Not decoded further anywhere in the
  codebase. **frontier**, called out already in "Hardware AUTO is not a dumb
  thermostat" above.
- ISU 110 ("Auto Mode Logic": Energy Saving vs. Comfort/Performance) — a
  vendor-exposed, hardware-side comfort/energy tradeoff knob, i.e. exactly
  the kind of decision `dead_band_actuation`'s hysteresis/dwell parameters
  are trying to make on-device.
- **ISU 125/127 ("Fan behavior during Thermo-OFF", heat/cool
  respectively)** — directly analogous to what `dead_band_actuation` does
  when it idles a running direction: the vendor already has an installer
  setting for "what the fan does while the compressor is thermo-off in this
  direction" (extra-low / stop / set-speed for heat; set-speed / stop for
  cool). `dead_band_actuation` currently has no fan-behavior-on-idle logic
  of its own and could either defer to this setting or mirror it.
- Bits 2-5 of the newer `auto_sub_mode` bitfield are explicitly unprobed
  (`cn105_types.h:112`) — may encode direction (heat vs. cool) directly,
  which would remove the need to infer direction from room-vs-setpoint
  deltas as `dead_band_actuation` currently must.

### 5. Other TODO/unknown markers found

- `heatpump_profile.cpp:181-182` — energy usage (`0x06` status) counter
  width and rollover behavior unknown ("Currently the maximum size of the
  counter is not known and if the counter extends to other bytes").
- `heatpump_profile.cpp:183` — trailing bytes in the MSZ-RW25VGHZ-SC1 `0x06`
  capture noted as "unknown bytes that appear to have a fixed/constant
  value."
- `hp_readings.cpp:149-151` — `0x10` packet type seen on the wire, decode
  explicitly not implemented.
- `hp_readings.cpp:138-142` — `0x05` timer packet decode explicitly not
  implemented (request itself is also disabled).

### Candidate exploits

1. Decode the unprobed bits 2-5 of the newer bitfield `auto_sub_mode`
   (`cn105_types.h:103-125`) on a unit that exposes it — if any bit encodes
   current heat/cool direction, `dead_band_actuation` (and hardware-AUTO
   monitoring generally) would no longer need to infer direction from
   room-vs-setpoint deltas.
2. Expose/document ISU 125 and 127 ("Fan behavior during Thermo-OFF") next
   to `dead_band_actuation`, since they control exactly the "what happens
   to the fan while this direction is idled" question the feature currently
   leaves unaddressed.
3. Investigate whether ISU 110 ("Auto Mode Logic") can be read back and
   surfaced as a diagnostic alongside `dead_band_actuation`'s own
   hysteresis/dwell settings, so users tuning one can see the other.
4. Probe a multi-unit/shared-outdoor-unit installation for `AUTO_LEADER`
   behavior (old-protocol units) to see whether it's a vendor-native
   analogue to `set_allow_heat()`/`set_allow_cool()`, which could simplify
   or validate the cross-zone veto design. **Caveat (see provenance note
   below): treat this as a name to investigate, not a confirmed
   capability** — nothing in this repo's history ties "AUTO_LEADER" to an
   actual observed multi-zone behavior.
5. Capture real `0x06` status frames across a full charge/discharge cycle
   on a unit with a visible energy meter to pin down the kWh counter width
   (currently unknown) — useful for the `kwh_sensor` and for any future
   energy-aware control logic.
6. Decode `0x10` on a unit that emits it, since it is currently an
   unimplemented but observed packet type — low-cost, might reveal another
   instrument or confirm it's redundant with `0x02`/`0x06`/`0x09`.

## Provenance check: `AUTO_LEADER` is unverified folklore, not a finding

Traced `AUTO_LEADER` through the full git history (`git log -S"AUTO_LEADER"`,
reverse order). It was introduced bare — no comment, no commit message
context — in `cbab6cf` ("clean up", 2024-02-16, a different contributor
than this project's current maintainer), already alongside `AUTO_COOL`/
`AUTO_HEAT` as a 3-value enum with no `AUTO_OFF`. It has been carried
forward, renamed around, and re-commented (the bitfield-vs-enum distinction
added in `c78850a`) ever since, but no commit, comment, or test anywhere in
history attaches an observed behavior to it beyond the raw byte value 0x03.
The name itself is the only evidence for a "leader" / multi-zone-coordinator
meaning — it may equally be a guess that stuck, or a mistranslation/misread
of vendor documentation. Downgraded candidate exploit #4 above accordingly:
worth probing on real hardware, but currently a hypothesis with a plausible
name, not a documented capability.

## `0x10` resolved as still-genuine frontier (not the handshake)

Traced the protocol framing one level up: `processCommand()`
(`hp_readings.cpp:173-191`) switches on the outer frame command — `0x61`
ack, `0x62` data, `0x7a`/`0x7b` handshake (user/installer). Only inside a
`0x62` data frame does `getDataFromResponsePacket()` switch on `data[0]`
(the inner packet *type*: `0x02` settings, `0x03` room temp, `0x04` error,
`0x05` timer, `0x06` status, `0x09` standby, `0x20`/`0x22` functions,
`0x42` HVAC options, and the unresolved `0x10`).

`utils.cpp:289` has a logging-helper comment labeling a `0x10` sub-case as
`":Hello" // Connect response`, which looked at first like it might resolve
the mystery — but that helper switches on a completely different byte
(`packet[5]` in a separate, explicitly-hedged debug-logging function full
of "Wait, need to check" comments), and the actual handshake path confirmed
in `processCommand()` uses command bytes `0x7a`/`0x7b`, not `0x10`. So that
comment is very likely a wrong guess left over in unrelated code, not
evidence about the real `0x10` data-packet type. **`0x10` remains genuinely
unidentified** — worth fixing/removing the misleading `utils.cpp:289`
comment separately so it doesn't mislead the next person chasing this.

## Scheduling note: how fresh is any of this data?

`registerInfoRequests()` (`cn105.cpp:94-133`) registers `0x02`/`0x03`/`0x06`
at priority 3 / 0ms extra delay and `0x09` (carries `auto_sub_mode`) at
priority 3 / 500ms extra delay, cycled by `request_scheduler.h`/`.cpp`.
This is the existing cadence `dead_band_actuation` already relies on
(verified on hardware at ~1s reaction time per the feature's commit
message) — any new logic reading `auto_sub_mode` or ISU codes inherits the
same freshness bound, nothing additional to design around.

## Deep dive: reframing `GO_IDLE` around ISU 125/127

Followed the two most promising candidate exploits further.

**ISU 125/127 are already live data, not just a select-entity label.**
`hp_.functions.getValue(code)` (`heatpump_functions_protocol.cpp:44-54`) is
a plain lookup over `heatpumpFunctions::raw`, which is populated by whatever
`0x20`/`0x22` responses have come in (`heatpump_profile.cpp:1405` →
`HeatPumpProtocol::decode_functions`). Nothing about reading code 125 or 127
requires the select-entity machinery in `hardware_setting_select.cpp` —
that's just one consumer. The catch: the `0x20`/`0x22` requests are only
registered at all when `AUXILIARY_CONTROLS` is supported *and*
`hardware_settings_` is non-empty (`cn105.cpp:144`), which today only
happens if the user configured at least one `hardware_settings.list` entry
in YAML. So `dead_band_actuation` could read ISU 125/127 for free once
populated, but can't assume they're populated unless it also arranges for
the request to be sent (or the user happens to have configured any
`hardware_settings` entry, since the packet returns all 30 codes at once
regardless of which ones are declared/exposed).

**The real mismatch: `GO_IDLE` power-cycles the unit; ISU 125/127 govern a
state `dead_band_actuation` never enters.** `dead_band_decision()`'s
`GO_IDLE` is turned into `power=OFF` by `stage_transition_(nullptr, "OFF",
NAN, now_ms)` (`heatpump_profile.cpp:941`) — a full power-off, every time
the room is inside the band. But ISU 125 ("Fan behavior during Thermo-OFF,
Heating") and 127 (cooling) describe the fan behavior for a *different*
state: the unit powered ON, in HEAT or COOL mode, with the compressor
satisfied (thermo-off) — the state hardware AUTO/HEAT/COOL settle into on
their own once their single setpoint is met, without cutting power. These
settings are currently unreachable by `dead_band_actuation` because it
never leaves the unit in that state; it either drives the compressor or
cuts power outright. That also means every band-satisfied moment is a
power-cycle, not a thermo-off — more wear-prone and noisier (no idle
circulation fan, unless the user is relying on ISU 125/127's "Extra
Low"/"Set Speed" defaults, which only apply when thermo-off, not when
powered off) than the hardware's own idle state would be.

**Candidate redesign**: instead of `GO_IDLE` → power off, send `mode=HEAT`
or `COOL` (whichever direction last ran, or neutral) with a setpoint chosen
so the unit's *own* thermostat immediately considers itself satisfied (e.g.
current room temperature, or the nearer band edge) — letting the hardware
settle into its native thermo-off state, fan behavior governed by ISU
125/127, instead of forcing `power=OFF`. This is close in spirit to the
original single-setpoint `deadband_setting()` fallback
(`heatpump_profile_protocol.h:28`), but invoked only for the idle case,
with `dead_band_actuation` still fully owning the HEAT/COOL/band-edge
decisions. Open question: does leaving the unit powered on in this state
cost any standby power relative to power-off (relevant to `kwh_sensor`,
`input_power_sensor`), and is thermo-off vs. power-off the better behavior
default, or should it become a `dead_band_actuation` sub-option?

## Mechanism validated on real hardware (2026-10-23)

Ran the predicted-then-tested protocol this project's own standard demands
for this codepath (see `reference_nixos-config-deployment` memory) against
`climate.minisplit_guest_rear` — not code reading, a live UART-logged
experiment. Full method and raw log evidence in the deployer's
`docs/plans/Minisplit_Comfort_Band_Architecture_Survey.md`; summary here:

- Entered plain `HEAT` mode with a setpoint *at* the then-current room
  temperature. Compressor kept running (11→13 Hz). **This looked like a
  falsification at first** — exactly the moment to stop and find one model
  explaining the conflict, not declare the hypothesis dead or alive. Root
  cause: the setpoint matched the coarse room-temp readout, but the actual
  comparison point is the injected remote-temperature value, which was
  0.1°C colder — genuinely unsatisfied, so continued heating was correct
  hardware behavior, not a refutation. (Also observed, unrelated: a ~90s
  `PREHEAT` sub-mode on entry, a startup transient independent of setpoint.)
- Corrected: lowered the setpoint to clearly below the live remote-temp
  reading. ~84 seconds later, the raw `0x06` status packet reported
  `operating: NO, compressor freq: 0.0 Hz` — sustained over the following
  polls, not a blip.
- Throughout, `Power: ON` never changed, and HA's `hvac_action` correctly
  settled to `idle` while `state` stayed `heat`.

**This confirms the mechanism `GO_IDLE`'s redesign depends on**: the
hardware will idle its own compressor while staying powered on in a plain
`HEAT`/`COOL` mode, given a setpoint it considers satisfied — exactly what
`resolve_mode()` (`heatpump_profile_protocol.h:327`) already assumes is
possible for `HOLD_HEATING`/`HOLD_COOLING`, just never deliberately
engaged today for the idle case. Combined with the earlier code-level
finding that `resolve_mode()` needs no changes and ownership tracking
plausibly simplifies, the redesign is no longer just plausible — it's
empirically grounded.

**Standby-power-cost follow-up (2026-10-23, same day)**: temporarily added
`compressor_frequency_sensor`/`input_power_sensor` to `guest_rear` to
compare power-off vs. thermo-off draw directly.
`input_power_sensor` turned out to read a hardcoded `0.0` on this unit
model (MSZ-GL06NA) — confirmed by forcing active heating
(`compressor_frequency` climbed 18→51 Hz) while input power stayed exactly
`0.0 W` throughout; this model's `0x06` status frame apparently doesn't
carry real wattage, matching the README's "not all sensors are supported
by all heatpumps" caveat. `compressor_frequency_sensor` *did* work reliably
as an idle-vs-running signal (clean 0 Hz at both power-off idle and
thermo-off idle, climbing cleanly during forced heating). No hard Watts
number is available from this specific hardware, but combined with the
general appliance-class fact that idle minisplit standby draw (control
board + optional low-speed fan) is reliably single-digit watts versus
tens-to-hundreds while the compressor runs, the qualitative signal is
sufficient to judge the standby cost of thermo-off-over-power-off as very
likely small relative to the wear/comfort benefit — without overclaiming a
number this hardware can't give. Full record, including the exact test
sequence and revert confirmation, in the deployer's
`docs/plans/Minisplit_Comfort_Band_Architecture_Survey.md`.

## Code change implemented (2026-10-23)

`GO_IDLE` (`heatpump_profile.cpp`'s `update_action()`) now stages a
satisfied setpoint (`HeatPumpProtocol::dead_band_idle_setpoint()`) with
`mode`/`power` left untouched, instead of unconditionally commanding
`power=OFF`, whenever a known direction (heating or cooling) is the one
idling. The `foreign_running` fallback (idling a leftover hardware-AUTO
run, no direction of ours to anchor an idle setpoint to) is unchanged —
still a real power-off.

**A real correctness bug was found and fixed while writing the regression
test, before touching hardware again** — exactly the kind of thing the
"mechanism before empirics" standard exists to catch. `dead_band_decision()`
and the safety clamp both infer `heating`/`cooling` from raw wire
`power`+`mode` (`compressor_running_as()`). That was fine when idling meant
`power=OFF` (wire correctly read "not heating" while idle), but once idling
means staying at `power=ON`/`mode=HEAT`, the wire reads "heating" forever,
even while genuinely idle. Left unfixed, this would have:
- Silently stopped the dead-band from ever firing a fresh `ENTER_HEAT`/
  `ENTER_COOL` once idling began — control would have quietly passed to the
  unit's own thermostat against whatever fixed setpoint happened to be
  staged at the moment of idling, not the user's actual configured band
  edge, decoupling the two over time.
- Made the **safety clamp** — explicitly documented as never-deferred —
  silently no-op (`HOLD_HEATING`/`HOLD_COOLING`) during a real freeze/
  overheat breach reached while idling, instead of forcing real heat/cool.
  This is the more serious of the two: it would have broken the one
  guarantee this feature treats as absolute.

Fixed with a new tracked flag, `dead_band_idling_` (`cn105.h`), fed into
both the dead-band decision and the safety clamp as
`compressor_running_as(...) && !dead_band_idling_` instead of the raw wire
read, restoring continuous real-band-edge comparison regardless of which
wire state idling leaves things in. `dead_band_owns_off_after()` gained a
matching `idling_on` parameter so `STAY_IDLE` doesn't claim ownership of a
power-off that was never going to happen from an idling-but-on state (that
ownership now only applies to the genuine power-off idle state, same as
before the redesign).

The cold-boot/`restore_setpoints` idle-without-power edge case flagged
earlier turned out to need no special-case code: traced both boot
scenarios (booting into a genuine power-off vs. into a leftover
thermo-off-idle state) through the fixed logic and confirmed both
self-correct within the first post-boot decision cycle, the same way
`dead_band_owns_off_` already does — documented in place in
`restore_setpoints()` rather than coded around.

**Verification**: all 333+3 existing unit tests pass; new/updated tests
cover `dead_band_idle_setpoint()`, both `GO_IDLE` ownership branches, the
`STAY_IDLE`/`idling_on` interaction, and a full replay regression test
(`DeadBandReplay.GoIdleWithKnownDirectionStaysOnAndSurvives`) that
specifically asserts `ENTER_HEAT` fires correctly when the room drifts back
to the real low edge from an idling state — the exact case the bug above
would have broken. A full ESP32-C6 build against the real ESPHome/ESP-IDF
toolchain (local `external_components`, not the GitHub fork pin) compiles
clean.

## Hardware re-verification found a second, more serious bug (2026-10-23)

Re-ran the full sequence on `climate.minisplit_guest_rear` through the real
`HEAT_COOL` dead-band codepath (not the standalone plain-`HEAT` test used
to validate the base mechanism). `ENTER_HEAT` fired correctly
(`dead-band staged mode=HEAT power=ON setpoint=...`), and `GO_IDLE`
correctly idled via setpoint (`operating: NO, compressor freq: 0.0 Hz`,
`Power: ON` held). But a follow-up `set_temperature` call made while
idling caused the compressor to start running again — sustained, not a
blip — with **no** corresponding `dead-band staged` log line, meaning the
dead-band itself never decided to act (it was correctly dwell-gated).
Something else drove the wire.

Root cause, traced via the WRITE packet and `Temp °C` readback:
`control_temperature()`'s `HEAT_COOL` branch (`heatpump_profile.cpp:663-673`,
pre-existing, unrelated to this feature) unconditionally computes and
stages `hp_.wantedSettings.temperature` from the legacy single-value
`deadband_setting()` on *every* `ClimateCall` carrying a temperature
change, regardless of `dead_band_actuation`. This runs in the same packet
cycle as the dead-band's own `update_action()`, but is a **separate,
uncoordinated write** — if `update_action()` decides not to act (dwell,
hysteresis, whatever), this legacy write still goes out on its own. Under
the old power-off idle design this was inert: a temperature-only write has
no effect while `power=OFF`. The thermo-off redesign idles via `power=ON`
specifically so hardware's own thermostat can act on a pushed setpoint —
which means this same pre-existing, previously-dormant write now has real,
immediate effect, and it bypassed `min_dwell_s`, `band_hysteresis_c`, and
the cross-zone `allow_heat_`/`allow_cool_` veto entirely. On a
shared-compressor fleet where that veto exists specifically to prevent
cross-zone conflicts, this is a real regression, not a cosmetic one.

**Fix** (separate commit): `control_temperature()` now skips the legacy
write entirely whenever `dead_band_actuation` is enabled —
`stage_transition_()` is the sole intended writer of wire temperature in
that mode, and already overwrites this value correctly on any real
transition; skipping the legacy write only ever mattered for the no-op
case the dead-band itself decided not to act on.

**Re-verified on hardware after the fix**: the same `ClimateCall`
sequence now logs `HEAT_COOL dead-band active: skipping legacy
deadband_setting() write` on every call in this mode — including the
exact dwell-blocked moment that previously caused the unwanted
engagement — with no further unintended compressor starts. All
333+3 unit tests still pass; full ESP32-C6 build still compiles clean.

This is the second time this research session needed a live hardware
re-check to catch something code review and unit tests couldn't: the first
(`dead_band_idling_`) was found by tracing a test's own expected behavior
before touching hardware; this one only surfaced *because* of the hardware
re-verification step — the unit tests can't host `CN105Climate` and have
no way to exercise `control_temperature()`'s interaction with
`update_action()` across a real `ClimateCall`. Worth remembering next time
a change here looks "done" after the unit suite passes.

## Backward-compatibility check: does the fix touch the legacy, non-opt-in path? (2026-10-23)

Raised directly: does `control_temperature()`'s new guard
(`if (hp_.dead_band_enabled()) { ...; return; }`) change anything for an
existing upstream user who has `dual_setpoint: true` but not
`dead_band_actuation` (the only option before this fork's feature
existed, and still the default)? By construction, no — `dead_band_enabled()`
is `dead_band_actuation_ && cn105_traits_requires_two_point(...)`, so with
the flag at its default `false`, the guard is never entered. But that's a
read, not a test, and the unit suite provides zero coverage either way
(same `CN105Climate`-hosting limitation noted above) — reading the code
isn't the same as testing it, so this got verified on hardware, not just
asserted.

Temporarily set `dead_band_actuation: false` on `guest_rear` (reverting to
plain `dual_setpoint` + hardware `AUTO`, with debug logging) and ran the
same `set_temperature` sequence. Confirmed via raw log: entering
`HEAT_COOL` sent wire-level `mode=AUTO` (`WRITE: [01 03 00 01 08 ...]`,
`08` = `AUTO`) exactly as plain upstream always has; the legacy
`HEAT_COOL deadband: current=..., low=..., high=... => setting=...` log
line ran (not skipped); and the computed setpoint was written to the wire
(`WRITE: [...00 A2 00]`, `0xA2` = 17.0°C in half-degree encoding) exactly
as legacy behavior always did. No `"skipping legacy deadband_setting()
write"` line appeared anywhere — the new guard correctly never fired.
Reverted; `guest_rear` confirmed back on the original `config_hash`.

**The fix is confirmed, not just argued, to leave the non-opt-in path
byte-for-byte unchanged.**

## Bottom line (so far)

The structural mismatch (`dual_setpoint` UI affordance vs. single-setpoint
wire protocol) is real and well evidenced. But "hardware AUTO is broken, so
override it entirely" overstates the case: AUTO has real internal logic
(energy/comfort tuning, probable multi-zone coordination) that
`dead_band_actuation`'s on/off HEAT/COOL/power takeover does not have access
to and may be duplicating less well. Worth investigating whether a
setpoint-nudging approach, or exposing/using ISU 110, gets closer to the
comfort/energy-savings goal while keeping hardware AUTO (and its modulation
and multi-zone logic) in the loop.
