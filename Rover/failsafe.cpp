/*
  failsafe support
  Andrew Tridgell, December 2011
 */

#include "Rover.h"

#include <stdio.h>

#define FAILSAFE_CHECK_MAX_PERIOD_MCS 500000

#define FAILSAFE_STATISTICS_PERIOD_MCS 60000000
void calcFailsafeCheckStatistics(const uint32_t tnow, const uint32_t last_timestamp) {
    
    static uint32_t max_delta_time = 0;
    static uint32_t min_delta_time = FAILSAFE_STATISTICS_PERIOD_MCS;
    static uint32_t check_count = 0;
    static uint32_t mean_delta_time = 0;

    static uint32_t delta_time_timestamp = 0;

    if (delta_time_timestamp != 0) {
         uint32_t delta_time = tnow - last_timestamp;
         check_count++;

         if (delta_time > max_delta_time) max_delta_time = delta_time;
         if (delta_time < min_delta_time) min_delta_time = delta_time;
         mean_delta_time += delta_time;

        if (tnow - delta_time_timestamp > FAILSAFE_STATISTICS_PERIOD_MCS) { // calc statistics every one minute
            mean_delta_time = mean_delta_time / check_count;
            gcs().send_text(MAV_SEVERITY_DEBUG, "Failsafe max-min-mean: %u - %u - %u", max_delta_time, min_delta_time, mean_delta_time);
            
            delta_time_timestamp = tnow;
            max_delta_time = 0;
            min_delta_time = FAILSAFE_STATISTICS_PERIOD_MCS;
            mean_delta_time = 0;
            check_count = 0;
        }
    }
    else {
        delta_time_timestamp = tnow;
    }

}
/*
  our failsafe strategy is to detect main loop lockup and disarm.
 */

/*
  this failsafe_check function is called from the core timer interrupt
  at 1kHz.
 */
void Rover::failsafe_check()
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    const uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // the main loop is running, all is OK
        last_ticks = ticks;
        last_timestamp = tnow;
        return;
    }

    // calcFailsafeCheckStatistics(tnow, last_timestamp);

    if (tnow - last_timestamp > FAILSAFE_CHECK_MAX_PERIOD_MCS) {
        // we have gone at least FAILSAFE_CHECK_MAX_PERIOD_MCS micro seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. disarm the motors
        // To-Do: log error
        if (arming.is_armed()) {
            // disarm motors
            arming.disarm(AP_Arming::Method::CPUFAILSAFE);
        }
    }
    
}

/*
  called to set/unset a failsafe event.
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, const char* type_str, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
    if (old_bits == 0 && failsafe.bits != 0) {
        // a failsafe event has started
        failsafe.start_time = millis();
    }
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // a failsafe event has ended
        gcs().send_text(MAV_SEVERITY_INFO, "%s Failsafe Cleared", type_str);
    }

    failsafe.triggered &= failsafe.bits;

    if ((failsafe.triggered == 0) &&
        (failsafe.bits != 0) &&
        (millis() - failsafe.start_time > g.fs_timeout * 1000) &&
        (control_mode != &mode_rtl) &&
        ((control_mode != &mode_hold || (g2.fs_options & (uint32_t)Failsafe_Options::Failsafe_Option_Active_In_Hold)))) {
        failsafe.triggered = failsafe.bits;
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe", type_str);

        // clear rc overrides
        RC_Channels::clear_overrides();

        if ((control_mode == &mode_auto) &&
            ((failsafe_type == FAILSAFE_EVENT_THROTTLE && g.fs_throttle_enabled == FS_THR_ENABLED_CONTINUE_MISSION) ||
             (failsafe_type == FAILSAFE_EVENT_GCS && g.fs_gcs_enabled == FS_GCS_ENABLED_CONTINUE_MISSION))) {
            // continue with mission in auto mode
        } else {
            switch (g.fs_action) {
            case Failsafe_Action_None:
                break;
            case Failsafe_Action_RTL:
                if (!set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::FAILSAFE);
                }
                break;
            case Failsafe_Action_Hold:
                set_mode(mode_hold, ModeReason::FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL:
                if (!set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    if (!set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                        set_mode(mode_hold, ModeReason::FAILSAFE);
                    }
                }
                break;
            case Failsafe_Action_SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::FAILSAFE);
                }
                break;
            }
        }
    }
}

void Rover::handle_battery_failsafe(const char* type_str, const int8_t action)
{
        switch ((Failsafe_Action)action) {
            case Failsafe_Action_None:
                break;
            case Failsafe_Action_SmartRTL:
                if (set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case Failsafe_Action_RTL:
                if (set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case Failsafe_Action_Hold:
                set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                }
                break;
            case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
                char battery_type_str[17];
                snprintf(battery_type_str, 17, "%s battery", type_str);
                g2.afs.gcs_terminate(true, battery_type_str);
#else
                arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
#endif // ADVANCED_FAILSAFE == ENABLED
                break;
        }
}

#if ADVANCED_FAILSAFE == ENABLED
/*
   check for AFS failsafe check
 */
void Rover::afs_fs_check(void)
{
    // perform AFS failsafe checks
    g2.afs.check(g2.fence.get_breaches() != 0, failsafe.last_valid_rc_ms);
}
#endif
