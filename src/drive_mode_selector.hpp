#pragma once

#include <optional>

#include "types.hpp"

// Operator requests are separate from the effective FOLLOW/AUTOPILOT state.
// A held RC switch must not undo recovery or re-arm a failed startup scan.
class DriveModeSelector {
public:
    DriveMode requested_mode() const { return requested_; }

    std::optional<DriveMode> update_rc(bool fresh, bool activated, DriveMode selected) {
        if (!fresh) {
            if (requested_ == DriveMode::MANUAL) {
                manual_reentry_blocked_ = true;
                return set_requested(DriveMode::IDLE);
            }
            return std::nullopt;
        }
        if (!activated) {
            have_rc_selection_ = false;
            return std::nullopt;
        }
        if (selected != DriveMode::MANUAL) manual_reentry_blocked_ = false;
        if (have_rc_selection_ && selected == last_rc_selection_) return std::nullopt;
        have_rc_selection_ = true;
        last_rc_selection_ = selected;
        return set_requested(selected == DriveMode::MANUAL && manual_reentry_blocked_
            ? DriveMode::IDLE : selected);
    }

    std::optional<DriveMode> request_voice(DriveMode mode, bool rc_active, DriveMode rc_mode) {
        // STOP always cancels motion. Other voice selections respect an active
        // RC switch, but repeating its selection explicitly re-arms a session.
        if (mode != DriveMode::IDLE && rc_active && mode != rc_mode) return std::nullopt;
        return set_requested(mode);
    }

    void finish_startup() { requested_ = DriveMode::IDLE; }

private:
    DriveMode set_requested(DriveMode mode) {
        requested_ = mode;
        return mode;
    }

    DriveMode requested_{DriveMode::IDLE};
    DriveMode last_rc_selection_{DriveMode::IDLE};
    bool have_rc_selection_{false};
    bool manual_reentry_blocked_{false};
};
