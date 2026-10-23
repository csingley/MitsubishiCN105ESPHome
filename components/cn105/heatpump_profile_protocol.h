#pragma once
#include "cn105_protocol.h"
#include "cn105_types.h"
#include "heatpumpFunctions.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "esphome/components/climate/climate_mode.h"

namespace esphome::cn105 {

// Production PAC state transitions; the standalone tests use these same helpers.
struct HeatPumpProtocol {
    struct SetpointState {
        uint8_t version;
        uint8_t mode;
        float target_low;
        float target_high;
    } __attribute__((packed));
    static SetpointState saved_setpoint_state(climate::ClimateMode mode, float low, float high) {
        return {1, static_cast<uint8_t>(mode), low, high};
    }
    static std::optional<SetpointState> restored_setpoint_state(bool enabled, bool loaded, const SetpointState& state, bool supports_dual) {
        if (!enabled || !loaded || state.version != 1 || state.mode != static_cast<uint8_t>(climate::CLIMATE_MODE_HEAT_COOL) ||
            std::isnan(state.target_low) || std::isnan(state.target_high) || !supports_dual) return std::nullopt;
        return state;
    }
    static float deadband_setting(float current, float low, float high) {
        if (current < low) return low;
        if (current > high) return high;
        return current;
    }

    // True if the polled wire state is actually running (power ON) in the
    // given mode ("HEAT"/"COOL") right now. Pure observation of
    // currentSettings -- never of this->mode, which can disagree with the
    // wire by design while the HEAT_COOL dead-band is driving it directly.
    static bool compressor_running_as(const heatpumpSettings& settings, const char* mode) {
        return settings.power != nullptr && std::strcmp(settings.power, "ON") == 0 &&
               settings.mode != nullptr && std::strcmp(settings.mode, mode) == 0;
    }

    // Decision math for the HEAT_COOL dead-band (update_action()'s
    // CLIMATE_MODE_HEAT_COOL case) -- pure function, no side effects, so
    // it's directly testable the same way deadband_setting() above is.
    // `dwell_ok`/`allow_heat`/`allow_cool` are already-resolved booleans;
    // this function doesn't know about millis(), min_dwell_s_, or where
    // the veto came from. `foreign_running`: powered on in a mode the
    // dead-band doesn't drive (e.g. hardware AUTO left over from before
    // HEAT_COOL was entered) -- idled like a running direction would be.
    enum class DeadBandAction { ENTER_HEAT, ENTER_COOL, HOLD_HEATING, HOLD_COOLING, GO_IDLE, STAY_IDLE };

    // Hysteresis actually applied: capped at half the band so the heat and
    // cool exit points can't cross.
    static float dead_band_hysteresis(float low, float high, float hysteresis) {
        if (std::isnan(hysteresis) || hysteresis <= 0.0f) return 0.0f;
        const float half_band = (high - low) / 2.0f;
        return half_band > 0.0f && hysteresis > half_band ? half_band : hysteresis;
    }

    // Setpoint sent with ENTER_HEAT/ENTER_COOL: the first 0.5 °C step at which
    // the dead-band idles again, so the unit's own thermostat doesn't satisfy
    // itself short of that point. Kept inside the band.
    static float dead_band_entry_setpoint(bool heat, float low, float high, float hysteresis) {
        const float h = dead_band_hysteresis(low, high, hysteresis);
        if (heat) return std::min(low + h + 0.5f, high);
        return std::max(high - h - 0.5f, low);
    }

    // Setpoint sent on GO_IDLE for a known running direction: lets the
    // unit's own thermostat settle into native thermo-off (compressor
    // idles, fan behavior per the unit's own ISU 125/127 settings) instead
    // of a power-off. Deliberately 0.5 C past the current reading, not
    // merely equal to it -- hardware's own comparison can read current
    // temperature slightly differently than ours (a different sensor
    // source, or coarser rounding), so an exactly-equal setpoint isn't
    // reliably "satisfied" to the unit itself. Verified on real hardware
    // (2026-10-23, docs/surveys/hardware-auto-mode.md): an equal setpoint
    // left the compressor running; -0.5/+0.5 reliably idled it.
    static float dead_band_idle_setpoint(bool heat, float current) {
        return heat ? current - 0.5f : current + 0.5f;
    }

    static DeadBandAction dead_band_decision(float current, float low, float high,
                                              bool heating, bool cooling,
                                              bool dwell_ok, bool allow_heat, bool allow_cool,
                                              bool foreign_running = false, float hysteresis = 0.0f) {
        if (std::isnan(current)) {
            // No reading yet -- never act blind. Report truthfully (whichever
            // direction is actually running, or idle), but never COMMAND
            // anything from here: unlike the comparisons below, a NaN
            // reading can't legitimately justify entering OR exiting.
            if (heating) return DeadBandAction::HOLD_HEATING;
            if (cooling) return DeadBandAction::HOLD_COOLING;
            return DeadBandAction::STAY_IDLE;
        }
        if (current >= high && (cooling || (dwell_ok && allow_cool))) {
            return cooling ? DeadBandAction::HOLD_COOLING : DeadBandAction::ENTER_COOL;
        }
        if (current <= low && (heating || (dwell_ok && allow_heat))) {
            return heating ? DeadBandAction::HOLD_HEATING : DeadBandAction::ENTER_HEAT;
        }
        // Hysteresis: a running direction carries on while the room is within
        // h of its edge, and idles once it's further in. Inclusive, so with
        // 0.5 °C readings h = 0.5 means one extra step. No effect at h == 0.
        const float h = dead_band_hysteresis(low, high, hysteresis);
        constexpr float EPS = 0.01f;
        if (h > 0.0f && heating && current <= low + h + EPS) return DeadBandAction::HOLD_HEATING;
        if (h > 0.0f && cooling && current >= high - h - EPS) return DeadBandAction::HOLD_COOLING;
        if (!heating && !cooling) {
            return (foreign_running && dwell_ok) ? DeadBandAction::GO_IDLE : DeadBandAction::STAY_IDLE;
        }
        if (dwell_ok) return DeadBandAction::GO_IDLE;
        // Held on past the dead-band edge by dwell -- still actually
        // heating/cooling, so say so rather than claiming an idle that
        // hasn't happened yet.
        return heating ? DeadBandAction::HOLD_HEATING : DeadBandAction::HOLD_COOLING;
    }

    // Whether the dead-band owns the unit being off/idle after this decision
    // (see resolve_mode()'s hold_power_off). `power_staged`: a power command
    // of ours was already waiting to be sent before this decision.
    // `idling_on`: dead_band_idling_'s value going into this decision --
    // true while the dead-band is holding a known direction idle via a
    // satisfied setpoint (power stays ON), as opposed to a genuine
    // power-off state. Needed because STAY_IDLE alone can mean either: a
    // real power-off report from this state would only ever be the user's
    // doing while idling_on is true (the dead-band never commands OFF from
    // there), so ownership must not be claimed in that case -- unlike the
    // true power-off idle state (e.g. at boot, before anything's ever been
    // commanded), where a power-off report is the dead-band's own normal
    // resting state and must still be held.
    static bool dead_band_owns_off_after(bool owns, DeadBandAction action, bool heating, bool cooling,
                                         bool foreign_running, bool power_staged, bool idling_on = false) {
        // Running with nothing of ours staged: someone else turned it on, so
        // a later power-off is theirs to make.
        if ((heating || cooling || foreign_running) && !power_staged) owns = false;
        switch (action) {
        case DeadBandAction::ENTER_HEAT:
        case DeadBandAction::ENTER_COOL: return false;
        // A known running direction idles via a satisfied setpoint, staying
        // powered on (see update_action()) -- no power-off event of ours to
        // protect. Only the foreign_running fallback (no known direction;
        // still forced OFF) keeps ownership of the resulting power-off
        // report, same as before.
        case DeadBandAction::GO_IDLE: return !(heating || cooling);
        case DeadBandAction::STAY_IDLE: return idling_on ? false : (foreign_running ? owns : true);
        default: return owns;
        }
    }

    // Queue staged wantedSettings for sending. loop() only sends when
    // hasChanged is set; a ClimateCall sets it in finalizeControlIfUpdated(),
    // anything staged outside one must set it here or it never goes out.
    static void mark_staged(wantedHeatpumpSettings& wanted, long now) {
        wanted.hasChanged = true;
        wanted.hasBeenSent = false;
        wanted.lastChange = now;
    }

    // Decision math for the absolute safety clamp (update_action(), checked
    // before everything else) -- pure function, same testability rationale
    // as dead_band_decision() above.
    enum class SafetyClampAction { NONE, ENTER_HEAT, HOLD_HEATING, ENTER_COOL, HOLD_COOLING, HOLD_IDLE };
    static SafetyClampAction safety_clamp_decision(float current, float floor_c, float ceiling_c,
                                                     bool already_heating, bool already_cooling,
                                                     bool dwell_ok) {
        if (std::isnan(current)) return SafetyClampAction::NONE;   // no reading yet, don't act blind
        if (!std::isnan(floor_c) && current < floor_c) {
            if (already_heating) return SafetyClampAction::HOLD_HEATING;
            if (dwell_ok) return SafetyClampAction::ENTER_HEAT;
            return already_cooling ? SafetyClampAction::HOLD_COOLING : SafetyClampAction::HOLD_IDLE;
        }
        if (!std::isnan(ceiling_c) && current > ceiling_c) {
            if (already_cooling) return SafetyClampAction::HOLD_COOLING;
            if (dwell_ok) return SafetyClampAction::ENTER_COOL;
            return already_heating ? SafetyClampAction::HOLD_HEATING : SafetyClampAction::HOLD_IDLE;
        }
        return SafetyClampAction::NONE;
    }
    template<typename Targets>
    static void reconcile_target_temperatures(Targets& targets, climate::ClimateMode mode, bool two_point, float temperature) {
        if (!two_point) {
            targets.setTargetTemperature(temperature);
            return;
        }
        if (mode == climate::CLIMATE_MODE_HEAT) {
            targets.setTargetTemperatureLow(temperature);
            if (std::isnan(targets.getTargetTemperatureHigh())) targets.setTargetTemperatureHigh(temperature);
        } else if (mode == climate::CLIMATE_MODE_COOL || mode == climate::CLIMATE_MODE_DRY) {
            targets.setTargetTemperatureHigh(temperature);
            if (std::isnan(targets.getTargetTemperatureLow())) targets.setTargetTemperatureLow(temperature);
        } else if (mode == climate::CLIMATE_MODE_AUTO || mode == climate::CLIMATE_MODE_HEAT_COOL) {
            // The hardware setpoint carries no information about an existing band.
            // Recentring it also ratchets Fahrenheit bands through the non-idempotent
            // conversion table (issue #673). Preserve both known bounds exactly.
            const bool low_defined = !std::isnan(targets.getTargetTemperatureLow());
            const bool high_defined = !std::isnan(targets.getTargetTemperatureHigh());
            if (low_defined && !high_defined) {
                targets.setTargetTemperatureHigh(targets.getTargetTemperatureLow() + 2.0f);
            } else if (!low_defined && high_defined) {
                targets.setTargetTemperatureLow(targets.getTargetTemperatureHigh() - 2.0f);
            } else if (!low_defined && !high_defined) {
                targets.setTargetTemperatureLow(temperature - 2.0f);
                targets.setTargetTemperatureHigh(temperature + 2.0f);
            }
        } else {
            if (std::isnan(targets.getTargetTemperatureLow())) targets.setTargetTemperatureLow(temperature);
            if (std::isnan(targets.getTargetTemperatureHigh())) targets.setTargetTemperatureHigh(temperature);
            const float theoretical = targets.calculateTemperatureSetting(
                (targets.getTargetTemperatureLow() + targets.getTargetTemperatureHigh()) / 2.0f);
            if (theoretical != temperature) {
                const float delta = (targets.getTargetTemperatureHigh() - targets.getTargetTemperatureLow()) / 2.0f;
                targets.setTargetTemperatureLow(theoretical - delta);
                targets.setTargetTemperatureHigh(theoretical + delta);
            }
        }
    }

    // The protocol cache and action refresh must not depend on a diagnostic sensor.
    template<typename RefreshAction>
    static bool apply_sub_mode(heatpumpSettings& current, const char* sub_mode, RefreshAction refresh_action) {
        if (sub_mode == nullptr || (current.sub_mode != nullptr && std::strcmp(current.sub_mode, sub_mode) == 0))
            return false;
        current.sub_mode = sub_mode;
        refresh_action();
        return true;
    }
    static bool accepts_remote_temperature(float setting) { return !std::isnan(setting); }
    static bool can_control_airflow(const heatpumpSettings& current) {
        return current.wideVane != nullptr && std::strcmp(current.wideVane, WIDEVANE_MAP[7]) == 0;
    }
    static bool decode_functions(heatpumpFunctions& functions, uint8_t code, const uint8_t* data, size_t length) {
        if (length < 16 || data[0] != code || (code != 0x20 && code != 0x22)) return false;
        // Codes with a zero value still announce support (e.g. SEZ outside installer mode).
        if (std::all_of(data + 1, data + length, [](uint8_t byte) { return byte == 0; })) return false;
        if (code == 0x20) functions.setData1(data + 1);
        else functions.setData2(data + 1);
        return true;
    }
    static bool encode_functions(uint8_t* first, uint8_t* second, const heatpumpFunctions& functions) {
        if (!functions.isValid()) return false;
        first[5] = 0x1F;
        second[5] = 0x21;
        functions.getData1(first + 6);
        functions.getData2(second + 6);
        return true;
    }
    static heatpumpRunStates optimistic_run_states(const wantedHeatpumpRunStates& wanted, const heatpumpRunStates& current) {
        heatpumpRunStates states = current;
        if (wanted.airflow_control != nullptr) states.airflow_control = wanted.airflow_control;
        if (wanted.air_purifier > -1) states.air_purifier = wanted.air_purifier;
        if (wanted.night_mode > -1) states.night_mode = wanted.night_mode;
        if (wanted.circulator > -1) states.circulator = wanted.circulator;
        return states;
    }
    static void encode_run_states(uint8_t* packet, const wantedHeatpumpRunStates& wanted, const heatpumpRunStates& current) {
        packet[5] = 0x08;
        if (wanted.airflow_control != nullptr) {
            const int index = cn105_protocol::lookup_index(AIRFLOW_CONTROL_MAP, 3, wanted.airflow_control);
            if (index >= 0) { packet[11] = AIRFLOW_CONTROL[index]; packet[6] |= RUN_STATE_PACKET_1[4]; }
        }
        if (wanted.air_purifier > -1 && static_cast<bool>(wanted.air_purifier) != current.air_purifier) {
            packet[17] = wanted.air_purifier ? 1 : 0; packet[7] |= RUN_STATE_PACKET_2[1];
        }
        if (wanted.night_mode > -1 && static_cast<bool>(wanted.night_mode) != current.night_mode) {
            packet[18] = wanted.night_mode ? 1 : 0; packet[7] |= RUN_STATE_PACKET_2[2];
        }
        if (wanted.circulator > -1 && static_cast<bool>(wanted.circulator) != current.circulator) {
            packet[19] = wanted.circulator ? 1 : 0; packet[7] |= RUN_STATE_PACKET_2[3];
        }
    }
    static float calculate_temperature_setting(float setting, bool encoding_b, bool special_table = false) {
        if (special_table) {
            setting = std::round(2.0f * setting) / 2.0f;
            return setting < 16.0f ? 16.0f : (setting > 31.0f ? 31.0f : setting);
        }
        if (!encoding_b)
            return cn105_protocol::lookup_index(TEMP_MAP, 16, static_cast<int>(setting + 0.5)) > -1 ? setting : TEMP_MAP[0];
        setting = std::round(2.0f * setting) / 2.0f;
        return setting < 10 ? 10 : (setting > 31 ? 31 : setting);
    }
    static void set_vane_setting(wantedHeatpumpSettings& wanted, const char* setting, uint32_t now) {
        const int index = cn105_protocol::lookup_index(VANE_MAP, 7, setting);
        wanted.vane = VANE_MAP[index >= 0 ? index : 0];
        if (index >= 0) {
            wanted.last_user_vane = wanted.vane;
            wanted.last_user_vane_ms = now;
        }
    }
    static const char* vane_for_packet(const wantedHeatpumpSettings& wanted) {
        return wanted.vane != nullptr ? wanted.vane : wanted.last_user_vane;
    }
    static const char* wide_vane_setting(wantedHeatpumpSettings& wanted, const heatpumpSettings& current) {
        if (wanted.wideVane == nullptr) return current.wideVane;
        // AIRFLOW CONTROL requires an active i-See sensor. Keep the current position otherwise,
        // including in the optimistic settings that will be published after this write.
        if (std::strcmp(wanted.wideVane, WIDEVANE_MAP[7]) == 0 && !current.iSee)
            wanted.wideVane = current.wideVane;
        return wanted.wideVane;
    }
    static void encode_control(uint8_t* packet, wantedHeatpumpSettings& wanted, const heatpumpSettings& current,
                               bool encoding_b, bool special_table, bool wide_adjust, bool split_horizontal) {
        const auto field = [&](const char* value, const char** names, const uint8_t* bytes,
                               int size, int offset, int mask_offset, uint8_t mask) {
            if (value == nullptr) return;
            const int index = cn105_protocol::lookup_index(names, size, value);
            if (index >= 0) { packet[offset] = bytes[index]; packet[mask_offset] |= mask; }
        };
        field(wanted.power, POWER_MAP, POWER, 2, 8, 6, CONTROL_PACKET_1[0]);
        field(wanted.mode, MODE_MAP, MODE, 5, 9, 6, CONTROL_PACKET_1[1]);
        if (wanted.temperature != -1.0f) {
            if (special_table) {
                packet[10] = cn105_protocol::encode_msz_a24na_setpoint(wanted.temperature);
                packet[6] |= CONTROL_PACKET_1[2];
            } else if (encoding_b) {
                packet[19] = static_cast<uint8_t>(wanted.temperature * 2 + 128);
                packet[6] |= CONTROL_PACKET_1[2];
            } else {
                const int index = cn105_protocol::lookup_index(TEMP_MAP, 16, static_cast<int>(wanted.temperature));
                if (index >= 0) { packet[10] = TEMP[index]; packet[6] |= CONTROL_PACKET_1[2]; }
            }
        }
        field(wanted.fan, FAN_MAP, FAN, 6, 11, 6, CONTROL_PACKET_1[3]);
        field(vane_for_packet(wanted), VANE_MAP, VANE, 7, 12, 6, CONTROL_PACKET_1[4]);
        if (wanted.wideVane != nullptr) {
            const char* setting = wide_vane_setting(wanted, current);
            const int index = setting != nullptr ? cn105_protocol::lookup_index(WIDEVANE_MAP, 8, setting) : -1;
            if (index >= 0) {
                packet[18] = WIDEVANE[index] | (wide_adjust ? 0x80 : 0x00);
                packet[7] |= CONTROL_PACKET_2[0];
                if (split_horizontal) packet[16] = WIDEVANE[index];
            }
        }
    }
    static void encode_remote_temperature(uint8_t* packet, float temperature) {
        packet[5] = 0x07;
        if (temperature > 0) {
            packet[6] = 0x01;
            cn105_protocol::encode_remote_temperature(temperature, packet[7], packet[8]);
        } else packet[8] = 0x80;
    }
    // hold_power_off: the HEAT_COOL dead-band itself chose to leave the unit
    // off/idle, so a power-off report is its own doing, not a user turning
    // the unit off -- keep HEAT_COOL instead of dropping to OFF.
    static climate::ClimateMode resolve_mode(const char* power, const char* reported,
                                             climate::ClimateMode mode, bool supports_dual,
                                             bool hold_power_off = false) {
        if (power == nullptr) return mode;
        if (std::strcmp(power, "ON") != 0) {
            const bool hold = hold_power_off && supports_dual && mode == climate::CLIMATE_MODE_HEAT_COOL;
            return hold ? mode : climate::CLIMATE_MODE_OFF;
        }
        if (reported == nullptr) return mode;
        const bool hold = supports_dual && mode == climate::CLIMATE_MODE_HEAT_COOL;
        if (std::strcmp(reported, "HEAT") == 0) return hold ? mode : climate::CLIMATE_MODE_HEAT;
        if (std::strcmp(reported, "COOL") == 0) return hold ? mode : climate::CLIMATE_MODE_COOL;
        if (std::strcmp(reported, "DRY") == 0) return climate::CLIMATE_MODE_DRY;
        if (std::strcmp(reported, "FAN") == 0) return climate::CLIMATE_MODE_FAN_ONLY;
        if (std::strcmp(reported, "AUTO") == 0)
            return mode == climate::CLIMATE_MODE_HEAT_COOL ? mode : climate::CLIMATE_MODE_AUTO;
        return mode;
    }
    static climate::ClimateMode reconcile_power_mode(const heatpumpSettings& settings, heatpumpSettings& current,
                                                      climate::ClimateMode mode, bool supports_dual, bool update_current,
                                                      bool hold_power_off = false) {
        const auto changed = [](const char* value, const char* previous) {
            return value != nullptr && (previous == nullptr || std::strcmp(value, previous) != 0);
        };
        const bool update = changed(settings.power, current.power) || changed(settings.mode, current.mode);
        if (update_current) {
            if (settings.power != nullptr) current.power = settings.power;
            if (settings.mode != nullptr) current.mode = settings.mode;
        }
        return update ? resolve_mode(settings.power, settings.mode, mode, supports_dual, hold_power_off) : mode;
    }
    static uint8_t compressor_frequency(const uint8_t* data, bool report_when_idle) {
        return report_when_idle || data[4] > 0 ? data[3] : 0;
    }
    static climate::ClimateAction operating_action(bool operating, bool use_stage, const char* stage,
                                                    climate::ClimateAction active_action) {
        const bool stage_active = use_stage && stage != nullptr && std::strcmp(stage, STAGE_MAP[0]) != 0;
        return operating || stage_active ? active_action : climate::CLIMATE_ACTION_IDLE;
    }
    static float decode_temperature(const uint8_t* data, float previous, bool special_table,
                                    bool& encoding_b, bool& encoding_b_latched) {
        if (special_table) return cn105_protocol::decode_msz_a24na_setpoint(data[5]);
        if (data[11] == 0x80) return previous;
        if (data[11] != 0x00) {
            encoding_b = true;
            encoding_b_latched = true;
            return static_cast<float>(data[11] - 128) / 2.0f;
        }
        const auto value = cn105_protocol::lookup_value_opt(TEMP_MAP, TEMP, 16, data[5]);
        return value ? static_cast<float>(*value) : previous;
    }
};

}  // namespace esphome::cn105
