/// test_dead_band_actuation.cpp — Coverage for the HEAT_COOL dead-band
/// actuation, anti-short-cycle dwell, absolute safety clamp, and the
/// external cross-zone veto hook.
/// Deps: heatpump_profile_protocol.h (production, pure functions only)
#include <gtest/gtest.h>
#include <cmath>
#include "heatpump_profile_protocol.h"
using esphome::cn105::HeatPumpProtocol;
using DeadBandAction = HeatPumpProtocol::DeadBandAction;
using SafetyClampAction = HeatPumpProtocol::SafetyClampAction;

namespace {

DeadBandAction deadBand(float current, float low, float high, bool heating, bool cooling,
                         bool dwellOk = true, bool allowHeat = true, bool allowCool = true,
                         bool foreign = false, float hysteresis = 0.0f) {
    return HeatPumpProtocol::dead_band_decision(current, low, high, heating, cooling,
                                                 dwellOk, allowHeat, allowCool, foreign, hysteresis);
}

} // namespace

// --- compressor_running_as() -------------------------------------------

TEST(CompressorRunningAs, TrueOnlyWhenPowerOnAndModeMatches) {
    heatpumpSettings s;
    s.power = "ON";
    s.mode = "HEAT";
    EXPECT_TRUE(HeatPumpProtocol::compressor_running_as(s, "HEAT"));
    EXPECT_FALSE(HeatPumpProtocol::compressor_running_as(s, "COOL"));
}

TEST(CompressorRunningAs, FalseWhenPowerOff) {
    heatpumpSettings s;
    s.power = "OFF";
    s.mode = "HEAT";
    EXPECT_FALSE(HeatPumpProtocol::compressor_running_as(s, "HEAT"));
}

TEST(CompressorRunningAs, FalseWhenFieldsUnset) {
    heatpumpSettings s; // power/mode default nullptr
    EXPECT_FALSE(HeatPumpProtocol::compressor_running_as(s, "HEAT"));
    EXPECT_FALSE(HeatPumpProtocol::compressor_running_as(s, "COOL"));
}

// --- dead_band_decision(): entry ----------------------------------------

TEST(DeadBandDecision, EntersHeatBelowLowWhenAllowedAndDwellOk) {
    EXPECT_EQ(deadBand(/*current=*/17.0f, /*low=*/20.0f, /*high=*/24.0f,
                        /*heating=*/false, /*cooling=*/false),
              DeadBandAction::ENTER_HEAT);
}

TEST(DeadBandDecision, EntersCoolAboveHighWhenAllowedAndDwellOk) {
    EXPECT_EQ(deadBand(27.0f, 20.0f, 24.0f, false, false), DeadBandAction::ENTER_COOL);
}

TEST(DeadBandDecision, VetoBlocksEnteringHeat) {
    EXPECT_EQ(deadBand(17.0f, 20.0f, 24.0f, false, false,
                        /*dwellOk=*/true, /*allowHeat=*/false),
              DeadBandAction::STAY_IDLE);
}

TEST(DeadBandDecision, VetoBlocksEnteringCool) {
    EXPECT_EQ(deadBand(27.0f, 20.0f, 24.0f, false, false,
                        /*dwellOk=*/true, /*allowHeat=*/true, /*allowCool=*/false),
              DeadBandAction::STAY_IDLE);
}

TEST(DeadBandDecision, DwellBlocksEnteringHeat) {
    EXPECT_EQ(deadBand(17.0f, 20.0f, 24.0f, false, false, /*dwellOk=*/false),
              DeadBandAction::STAY_IDLE);
}

TEST(DeadBandDecision, DwellBlocksEnteringCool) {
    EXPECT_EQ(deadBand(27.0f, 20.0f, 24.0f, false, false, /*dwellOk=*/false),
              DeadBandAction::STAY_IDLE);
}

// --- dead_band_decision(): continuation is never vetoed/dwell-gated ----

TEST(DeadBandDecision, ContinuingHeatIgnoresVeto) {
    // Already heating, current still below low, but allow_heat is false --
    // a veto only blocks a NEW entry, never an in-progress direction.
    EXPECT_EQ(deadBand(17.0f, 20.0f, 24.0f, /*heating=*/true, false,
                        /*dwellOk=*/true, /*allowHeat=*/false),
              DeadBandAction::HOLD_HEATING);
}

TEST(DeadBandDecision, ContinuingCoolIgnoresDwell) {
    EXPECT_EQ(deadBand(27.0f, 20.0f, 24.0f, false, /*cooling=*/true, /*dwellOk=*/false),
              DeadBandAction::HOLD_COOLING);
}

// --- dead_band_decision(): exit / idle -----------------------------------

TEST(DeadBandDecision, GoesIdleInsideBandWhenDwellAllows) {
    EXPECT_EQ(deadBand(22.0f, 20.0f, 24.0f, /*heating=*/true, false, /*dwellOk=*/true),
              DeadBandAction::GO_IDLE);
}

TEST(DeadBandDecision, HeldPastEdgeByDwellReportsWhicheverIsRunning) {
    // Inside the band now, dwell still running -- must not claim an idle
    // that hasn't physically happened yet.
    EXPECT_EQ(deadBand(22.0f, 20.0f, 24.0f, /*heating=*/true, false, /*dwellOk=*/false),
              DeadBandAction::HOLD_HEATING);
    EXPECT_EQ(deadBand(22.0f, 20.0f, 24.0f, false, /*cooling=*/true, /*dwellOk=*/false),
              DeadBandAction::HOLD_COOLING);
}

TEST(DeadBandDecision, StaysIdleWhenNeitherRunningAndInsideBand) {
    EXPECT_EQ(deadBand(22.0f, 20.0f, 24.0f, false, false), DeadBandAction::STAY_IDLE);
}

// --- dead_band_decision(): NaN reading never commands anything ---------

TEST(DeadBandDecision, NanCurrentHoldsWhateverIsRunningWithoutCommanding) {
    EXPECT_EQ(deadBand(NAN, 20.0f, 24.0f, /*heating=*/true, false), DeadBandAction::HOLD_HEATING);
    EXPECT_EQ(deadBand(NAN, 20.0f, 24.0f, false, /*cooling=*/true), DeadBandAction::HOLD_COOLING);
    EXPECT_EQ(deadBand(NAN, 20.0f, 24.0f, false, false), DeadBandAction::STAY_IDLE);
}

// --- safety_clamp_decision() ---------------------------------------------

TEST(SafetyClampDecision, DisabledWhenBothThresholdsUnset) {
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(5.0f, NAN, NAN, false, false, true),
              SafetyClampAction::NONE);
}

TEST(SafetyClampDecision, EntersHeatBelowFloor) {
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(/*current=*/5.0f, /*floor=*/10.0f, /*ceiling=*/NAN,
                                                        false, false, /*dwellOk=*/true),
              SafetyClampAction::ENTER_HEAT);
}

TEST(SafetyClampDecision, EntersCoolAboveCeiling) {
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(35.0f, NAN, 32.0f, false, false, true),
              SafetyClampAction::ENTER_COOL);
}

TEST(SafetyClampDecision, DwellBlocksEntryButReportsWhicheverIsRunning) {
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(5.0f, 10.0f, NAN, false, /*already_cooling=*/true,
                                                        /*dwellOk=*/false),
              SafetyClampAction::HOLD_COOLING);
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(5.0f, 10.0f, NAN, false, false, false),
              SafetyClampAction::HOLD_IDLE);
}

TEST(SafetyClampDecision, NeverVetoed) {
    // The safety clamp has no allow_heat/allow_cool parameter at all --
    // nothing can veto it, by construction, not just by a passed-in flag.
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(5.0f, 10.0f, NAN, false, false, true),
              SafetyClampAction::ENTER_HEAT);
}

TEST(SafetyClampDecision, NanCurrentNeverActs) {
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(NAN, 10.0f, 32.0f, false, false, true),
              SafetyClampAction::NONE);
}

TEST(SafetyClampDecision, WithinBandIsNone) {
    EXPECT_EQ(HeatPumpProtocol::safety_clamp_decision(20.0f, 10.0f, 32.0f, false, false, true),
              SafetyClampAction::NONE);
}

// --- resolve_mode(): power-off hold for a dead-band-owned idle ----------

using esphome::climate::CLIMATE_MODE_HEAT_COOL;
using esphome::climate::CLIMATE_MODE_OFF;
using esphome::climate::CLIMATE_MODE_HEAT;

TEST(ResolveModePowerOffHold, DeadBandOwnedOffKeepsHeatCool) {
    EXPECT_EQ(HeatPumpProtocol::resolve_mode("OFF", "HEAT", CLIMATE_MODE_HEAT_COOL, /*dual=*/true,
                                             /*hold_power_off=*/true),
              CLIMATE_MODE_HEAT_COOL);
}

TEST(ResolveModePowerOffHold, UnownedOffStillDropsToOff) {
    // A user's power-off (remote or HA) must still be honoured.
    EXPECT_EQ(HeatPumpProtocol::resolve_mode("OFF", "HEAT", CLIMATE_MODE_HEAT_COOL, true, false),
              CLIMATE_MODE_OFF);
}

TEST(ResolveModePowerOffHold, HoldOnlyAppliesInDualHeatCool) {
    EXPECT_EQ(HeatPumpProtocol::resolve_mode("OFF", "HEAT", CLIMATE_MODE_HEAT_COOL, /*dual=*/false, true),
              CLIMATE_MODE_OFF);
    EXPECT_EQ(HeatPumpProtocol::resolve_mode("OFF", "HEAT", CLIMATE_MODE_HEAT, true, true),
              CLIMATE_MODE_OFF);
}

TEST(ResolveModePowerOffHold, ReconcilePassesHoldThrough) {
    heatpumpSettings incoming;
    incoming.power = "OFF";
    incoming.mode = "HEAT";
    heatpumpSettings current;
    current.power = "ON";
    current.mode = "HEAT";
    EXPECT_EQ(HeatPumpProtocol::reconcile_power_mode(incoming, current, CLIMATE_MODE_HEAT_COOL,
                                                     /*dual=*/true, /*update_current=*/false,
                                                     /*hold_power_off=*/true),
              CLIMATE_MODE_HEAT_COOL);
}

// --- dead_band_decision(): idling a foreign mode (e.g. hardware AUTO) ----

TEST(DeadBandDecision, ForeignRunningInsideBandIsIdled) {
    EXPECT_EQ(deadBand(22.0f, 20.0f, 24.0f, false, false, /*dwellOk=*/true, true, true,
                        /*foreign=*/true),
              DeadBandAction::GO_IDLE);
}

TEST(DeadBandDecision, ForeignRunningIdleWaitsForDwell) {
    EXPECT_EQ(deadBand(22.0f, 20.0f, 24.0f, false, false, /*dwellOk=*/false, true, true,
                        /*foreign=*/true),
              DeadBandAction::STAY_IDLE);
}

TEST(DeadBandDecision, ForeignRunningBelowLowEntersHeat) {
    EXPECT_EQ(deadBand(17.0f, 20.0f, 24.0f, false, false, true, true, true, /*foreign=*/true),
              DeadBandAction::ENTER_HEAT);
}

// --- hysteresis ------------------------------------------------------------

TEST(DeadBandHysteresis, ZeroReproducesNoHysteresis) {
    // Heating, room just back inside the band: idle immediately at h == 0.
    EXPECT_EQ(deadBand(20.5f, 20.0f, 24.0f, /*heating=*/true, false, true, true, true, false, 0.0f),
              DeadBandAction::GO_IDLE);
}

TEST(DeadBandHysteresis, HeatingCarriesOnWhileWithinHOfTheEdge) {
    // 0.5 °C readings, h = 0.5: heat at 20.0, keep going at 20.5, idle at 21.0.
    EXPECT_EQ(deadBand(20.5f, 20.0f, 24.0f, true, false, true, true, true, false, /*h=*/0.5f),
              DeadBandAction::HOLD_HEATING);
    EXPECT_EQ(deadBand(21.0f, 20.0f, 24.0f, true, false, true, true, true, false, 0.5f),
              DeadBandAction::GO_IDLE);
}

TEST(DeadBandHysteresis, CoolingCarriesOnWhileWithinHOfTheEdge) {
    EXPECT_EQ(deadBand(23.5f, 20.0f, 24.0f, false, /*cooling=*/true, true, true, true, false, 0.5f),
              DeadBandAction::HOLD_COOLING);
    EXPECT_EQ(deadBand(23.0f, 20.0f, 24.0f, false, true, true, true, true, false, 0.5f),
              DeadBandAction::GO_IDLE);
}

TEST(DeadBandHysteresis, HalfDegreeIsNotANoOpWithHalfDegreeReadings) {
    // The case that would silently do nothing with an exclusive comparison.
    EXPECT_EQ(deadBand(20.5f, 20.0f, 24.0f, true, false, true, true, true, false, 0.0f),
              DeadBandAction::GO_IDLE);
    EXPECT_EQ(deadBand(20.5f, 20.0f, 24.0f, true, false, true, true, true, false, 0.5f),
              DeadBandAction::HOLD_HEATING);
}

TEST(DeadBandHysteresis, DoesNotStartAnything) {
    // Idle inside the band stays idle regardless of h.
    EXPECT_EQ(deadBand(20.5f, 20.0f, 24.0f, false, false, true, true, true, false, 1.0f),
              DeadBandAction::STAY_IDLE);
}

TEST(DeadBandHysteresis, CappedAtHalfBand) {
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_hysteresis(20.0f, 21.0f, 5.0f), 0.5f);
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_hysteresis(20.0f, 24.0f, 0.5f), 0.5f);
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_hysteresis(20.0f, 24.0f, -1.0f), 0.0f);
}

TEST(DeadBandEntrySetpoint, OneStepPastTheExitPoint) {
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_entry_setpoint(/*heat=*/true, 20.0f, 24.0f, 0.0f), 20.5f);
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_entry_setpoint(true, 20.0f, 24.0f, 0.5f), 21.0f);
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_entry_setpoint(false, 20.0f, 24.0f, 0.5f), 23.0f);
}

TEST(DeadBandEntrySetpoint, StaysInsideTheBand) {
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_entry_setpoint(true, 20.0f, 20.5f, 0.0f), 20.5f);
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_entry_setpoint(false, 20.0f, 20.5f, 0.0f), 20.0f);
}

// --- ownership of an idle/off unit -------------------------------------------

TEST(DeadBandOwnership, GoIdleWithAKnownDirectionIsNotOwned) {
    // A known running direction idles via a satisfied setpoint, staying
    // powered on -- no power-off event of ours to protect.
    EXPECT_FALSE(HeatPumpProtocol::dead_band_owns_off_after(false, DeadBandAction::GO_IDLE,
                                                            /*heating=*/true, false, false, false));
    EXPECT_FALSE(HeatPumpProtocol::dead_band_owns_off_after(false, DeadBandAction::GO_IDLE,
                                                            false, /*cooling=*/true, false, false));
}

TEST(DeadBandOwnership, GoIdleWithNoKnownDirectionIsOwned) {
    // Foreign_running fallback (idling a leftover hardware-AUTO run): no
    // setpoint of ours to anchor to, still forced OFF, so still owned.
    EXPECT_TRUE(HeatPumpProtocol::dead_band_owns_off_after(false, DeadBandAction::GO_IDLE,
                                                           false, false, /*foreign=*/true, false));
}

TEST(DeadBandOwnership, ChosenIdleIsOwned) {
    EXPECT_TRUE(HeatPumpProtocol::dead_band_owns_off_after(false, DeadBandAction::STAY_IDLE,
                                                           false, false, false, false));
}

TEST(DeadBandOwnership, EnteringADirectionReleasesIt) {
    EXPECT_FALSE(HeatPumpProtocol::dead_band_owns_off_after(true, DeadBandAction::ENTER_HEAT,
                                                            false, false, false, false));
}

TEST(DeadBandOwnership, SomeoneElseTurningItOnReleasesIt) {
    // Running HEAT with nothing of ours staged: not ours any more, so a later
    // power-off (the remote) must be honoured.
    EXPECT_FALSE(HeatPumpProtocol::dead_band_owns_off_after(true, DeadBandAction::HOLD_HEATING,
                                                            /*heating=*/true, false, false,
                                                            /*power_staged=*/false));
}

TEST(DeadBandOwnership, OurOwnStagedOffIsNotReleasedByStaleRunningState) {
    // GO_IDLE staged, wire still reads heating until it's sent: keep ownership.
    EXPECT_TRUE(HeatPumpProtocol::dead_band_owns_off_after(true, DeadBandAction::HOLD_HEATING,
                                                           true, false, false, /*power_staged=*/true));
}

TEST(DeadBandOwnership, IdleWhileForeignModeStillRunningIsNotClaimed) {
    EXPECT_FALSE(HeatPumpProtocol::dead_band_owns_off_after(false, DeadBandAction::STAY_IDLE,
                                                            false, false, /*foreign=*/true, false));
}

// --- replay: the 2026-10-22 21:04 failure, through the real functions ------
// Kitchen, band 20.0-24.4 °C, wire heating, room drifts to 20.5 (inside).
// Before the fix: nothing marked the staged OFF for sending, and once some
// unrelated call carried it out, reconcile dropped HEAT_COOL to OFF for good.
// Since the thermo-off redesign (2026-10-23), a known running direction no
// longer powers off at all on GO_IDLE -- see
// DeadBandReplay.GoIdleWithKnownDirectionStaysOnAndSurvives below for that
// path. This replay now covers the one case that still powers off: idling a
// foreign (hardware-AUTO) leftover, no direction of ours to anchor to.

TEST(DeadBandReplay, GoIdleIsQueuedAndHeatCoolSurvivesThePowerOff) {
    auto mode = CLIMATE_MODE_HEAT_COOL;
    bool owns = false;
    heatpumpSettings current;
    current.power = "ON";
    current.mode = "AUTO";  // foreign: not a direction the dead-band drives
    wantedHeatpumpSettings wanted;

    // Room reaches 20.5: decide, update ownership, stage.
    const auto action = deadBand(20.5f, 20.0f, 24.4f, /*heating=*/false, /*cooling=*/false,
                                  /*dwellOk=*/true, /*allowHeat=*/true, /*allowCool=*/true,
                                  /*foreign=*/true);
    ASSERT_EQ(action, DeadBandAction::GO_IDLE);
    owns = HeatPumpProtocol::dead_band_owns_off_after(owns, action, false, false,
                                                      /*foreign=*/true, wanted.power != nullptr);
    wanted.power = "OFF";
    HeatPumpProtocol::mark_staged(wanted, 1000);
    EXPECT_TRUE(wanted.hasChanged);  // loop() would now send it
    EXPECT_FALSE(wanted.hasBeenSent);

    // Sent: apply_wanted_settings() reconciles against what was sent...
    mode = HeatPumpProtocol::reconcile_power_mode(wanted, current, mode, /*dual=*/true,
                                                  /*update_current=*/false, owns);
    EXPECT_EQ(mode, CLIMATE_MODE_HEAT_COOL);
    wanted.resetSettings();

    // ...then the unit reports power off.
    heatpumpSettings reported;
    reported.power = "OFF";
    reported.mode = "AUTO";
    mode = HeatPumpProtocol::reconcile_power_mode(reported, current, mode, true, true, owns);
    EXPECT_EQ(mode, CLIMATE_MODE_HEAT_COOL);
    EXPECT_STREQ(current.power, "OFF");

    // Later, after dwell, the room is back at the low edge: heat again.
    const auto next = deadBand(20.0f, 20.0f, 24.4f,
                               HeatPumpProtocol::compressor_running_as(current, "HEAT"),
                               HeatPumpProtocol::compressor_running_as(current, "COOL"));
    EXPECT_EQ(next, DeadBandAction::ENTER_HEAT);
    owns = HeatPumpProtocol::dead_band_owns_off_after(owns, next, false, false, false, false);
    EXPECT_FALSE(owns);
}

TEST(DeadBandReplay, UserPowerOffWhileRunningStillDropsToOff) {
    auto mode = CLIMATE_MODE_HEAT_COOL;
    heatpumpSettings current;
    current.power = "ON";
    current.mode = "HEAT";
    // Dead-band holding heat, nothing staged: ownership released.
    const bool owns = HeatPumpProtocol::dead_band_owns_off_after(true, DeadBandAction::HOLD_HEATING,
                                                                 true, false, false, false);
    heatpumpSettings reported;  // the remote's OFF button
    reported.power = "OFF";
    reported.mode = "HEAT";
    mode = HeatPumpProtocol::reconcile_power_mode(reported, current, mode, true, true, owns);
    EXPECT_EQ(mode, CLIMATE_MODE_OFF);
}

// --- thermo-off redesign (2026-10-23): GO_IDLE with a known direction ------
// stays powered on and idles via a satisfied setpoint instead of a
// power-off. Mechanism verified on real hardware against
// climate.minisplit_guest_rear; see docs/surveys/hardware-auto-mode.md.

TEST(DeadBandIdleSetpoint, PastCurrentInTheIdlingDirection) {
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_idle_setpoint(/*heat=*/true, 20.0f), 19.5f);
    EXPECT_FLOAT_EQ(HeatPumpProtocol::dead_band_idle_setpoint(/*heat=*/false, 20.0f), 20.5f);
}

// `idling` here stands in for cn105.h's `dead_band_idling_`: production
// feeds dead_band_decision()/dead_band_owns_off_after() effective, not raw
// wire, heating/cooling (compressor_running_as(...) && !idling). Without
// that distinction, a direction idled via a satisfied setpoint would read
// as permanently "already heating/cooling" at the wire level forever,
// and the band edges the room is later compared against would silently
// stop being re-evaluated -- a real regression caught while writing this
// test, fixed the same session (cn105.h's dead_band_idling_ comment).
TEST(DeadBandReplay, GoIdleWithKnownDirectionStaysOnAndSurvives) {
    auto mode = CLIMATE_MODE_HEAT_COOL;
    bool owns = false;
    bool idling = false;
    heatpumpSettings current;
    current.power = "ON";
    current.mode = "HEAT";
    wantedHeatpumpSettings wanted;

    // Room reaches 20.5 (inside the 20.0-24.4 band), already heating.
    const bool wireHeating = HeatPumpProtocol::compressor_running_as(current, "HEAT");
    const bool heating = wireHeating && !idling;
    const auto action = deadBand(20.5f, 20.0f, 24.4f, heating, false);
    ASSERT_EQ(action, DeadBandAction::GO_IDLE);
    owns = HeatPumpProtocol::dead_band_owns_off_after(owns, action, heating, false, false,
                                                      wanted.power != nullptr, idling);
    EXPECT_FALSE(owns);  // nothing of ours being powered off this time

    // update_action() stages a satisfied setpoint, power left ON, and marks
    // the direction as idling (not actively driven) from here on.
    wanted.power = "ON";
    wanted.temperature = HeatPumpProtocol::dead_band_idle_setpoint(heating, 20.5f);
    HeatPumpProtocol::mark_staged(wanted, 1000);
    idling = true;
    EXPECT_TRUE(wanted.hasChanged);

    // Sent, then the unit reports back still ON/HEAT (thermo-off is a wire-
    // level idle, not visible as a power/mode change) -- mode holds
    // regardless of `owns`, since resolve_mode()'s HEAT_COOL hold for a
    // power=ON report never consults hold_power_off at all.
    mode = HeatPumpProtocol::reconcile_power_mode(wanted, current, mode, /*dual=*/true,
                                                  /*update_current=*/true, owns);
    EXPECT_EQ(mode, CLIMATE_MODE_HEAT_COOL);
    EXPECT_STREQ(current.power, "ON");

    // Room drifts back to the low edge. Fed the *effective* heating (false,
    // since idling), this correctly fires a fresh ENTER_HEAT against the
    // real band edge -- not HOLD_HEATING, which is what the raw wire read
    // (still ON/HEAT the whole time) would have produced, and which would
    // have silently deferred to whatever fixed setpoint was staged back at
    // 20.5 instead of the user's actual configured low edge (20.0).
    const auto next = deadBand(20.0f, 20.0f, 24.4f,
                               HeatPumpProtocol::compressor_running_as(current, "HEAT") && !idling,
                               HeatPumpProtocol::compressor_running_as(current, "COOL") && !idling);
    EXPECT_EQ(next, DeadBandAction::ENTER_HEAT);
}

TEST(DeadBandOwnership, IdlingOnMeansNoPowerOffToOwnEvenOnStayIdle) {
    // While idling_on, a later power-off report is the user's doing (the
    // dead-band never commands OFF from this state) -- STAY_IDLE must not
    // claim ownership the way it does for a genuine power-off idle state.
    EXPECT_FALSE(HeatPumpProtocol::dead_band_owns_off_after(false, DeadBandAction::STAY_IDLE,
                                                            false, false, false, false,
                                                            /*idling_on=*/true));
    // Without idling_on (the true off-at-boot/old-design idle state),
    // STAY_IDLE's existing behaviour is unchanged.
    EXPECT_TRUE(HeatPumpProtocol::dead_band_owns_off_after(false, DeadBandAction::STAY_IDLE,
                                                           false, false, false, false,
                                                           /*idling_on=*/false));
}
