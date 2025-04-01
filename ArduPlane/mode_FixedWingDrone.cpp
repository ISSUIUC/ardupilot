#include "mode.h"
#include "Plane.h"
// Loiter Copy and Pasted: 
bool ModeFixedWingDrone::_enter()
{
    //targetAltitude; 
    //if(targetAlt)
    plane.do_loiter_at_location();
    plane.setup_terrain_target_alt(plane.next_WP_loc);

    // make sure the local target altitude is the same as the nav target used for loiter nav
    // this allows us to do FBWB style stick control
    /*IGNORE_RETURN(plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, plane.target_altitude.amsl_cm));*/
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        plane.set_target_altitude_current();
    }

    plane.loiter_angle_reset();

    return true;
}

void ModeFixedWingDrone::update()
{
    plane.calc_nav_roll();
    if (plane.stick_mixing_enabled() && plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        plane.update_fbwb_speed_height();
    } else {
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // while a trick is running we reset altitude
        plane.set_target_altitude_current();
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }
#endif
}

bool ModeFixedWingDrone::isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc)
{
    // Return true if current heading is aligned to vector to targetLoc.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.

    // Corrected radius for altitude
    const float loiter_radius = plane.nav_controller->loiter_radius(fabsf(plane.loiter.radius));
    if (!is_positive(loiter_radius)) {
        // Zero is invalid, protect against divide by zero for destination inside loiter radius case
        return true;
    }

    // Calculate relative position of the vehicle relative to loiter center projected onto the closest point of the loiter circle
    // This removes error due to radial position as the nav controller attempts to track the circle
    const Vector2f projected_pos = loiterCenterLoc.get_distance_NE(plane.current_loc).normalized() * loiter_radius;

    // Target position relative to loiter center
    const Vector2f target_pos = loiterCenterLoc.get_distance_NE(targetLoc);

    // Distance between loiter circle and target
    const float target_dist = target_pos.length();
    if (!is_positive(target_dist)) {
        // Target is coincident with loiter center, no heading will be closer than any other
        return true;
    }

    // Target bearing in centi-degrees
    int32_t target_bearing_cd;

    if (target_dist >= loiter_radius) {
        // Destination outside loiter radius, heading will always line up with destination

        // Vector from between projected vehicle position and target postion
        const Vector2f pos_to_target = target_pos - projected_pos;
        target_bearing_cd = degrees(pos_to_target.angle()) * 100;

    } else {
        // Destination is inside loiter, heading will never line up with destination

        // Advance turn point by the angle of a segment with chord "a"
        // This results in turning earlier as the target point approaches the center
        // If target is on radius angle of 0 and angle of 60 deg if target is on center 
        const float a = loiter_radius - target_dist;
        const float segment_angle = 2.0 * asinf(a / (2.0 * loiter_radius));

        // Pick the intersection point that will be hit first for the current loiter direction, add 90 deg to get the tangent angle
        target_bearing_cd = degrees(wrap_PI(target_pos.angle() + (M_PI_2 - segment_angle) * plane.loiter.direction)) * 100;

    }

    // Ideal heading in centi-degrees, +- 90 to get tangent to loiter circle at closest point
    const int32_t current_heading_cd = degrees(wrap_PI(projected_pos.angle() + M_PI_2 * plane.loiter.direction)) * 100;

    return isHeadingLinedUp_cd(target_bearing_cd, current_heading_cd);
}


bool ModeFixedWingDrone::isHeadingLinedUp_cd(const int32_t bearing_cd) {

    // get current heading.
    const int32_t heading_cd = (wrap_360(degrees(ahrs.groundspeed_vector().angle())))*100;

    return isHeadingLinedUp_cd(bearing_cd, heading_cd);
}


bool ModeFixedWingDrone::isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd)
{
    // Return true if current heading is aligned to bearing_cd.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.
    const int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw.

      After every full circle, extend acceptance criteria to ensure
      aircraft will not loop forever in case high winds are forcing
      it beyond 200 deg/sec when passing the desired exit course
    */

    // Use integer division to get discrete steps
    const int32_t expanded_acceptance = 1000 * (labs(plane.loiter.sum_cd) / 36000);

    if (labs(heading_err_cd) <= 1000 + expanded_acceptance) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        if (plane.next_WP_loc.loiter_xtrack) {
            plane.next_WP_loc = plane.current_loc;
        }
        return true;
    }
    return false;
}

void ModeFixedWingDrone::navigate()
{
    if (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        // update the WP alt from the global target adjusted by update_fbwb_speed_height
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // don't try to navigate while running trick
        return;
    }
#endif

    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

void ModeFixedWingDrone::update_target_altitude()
{
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        return;
    }
    Mode::update_target_altitude();
}

// Guided Copy and Pasted
"""

#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plane.current_loc};

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());
    }
#endif

    // set guided radius to WP_LOITER_RAD on mode change.
    active_radius_m = 0;

    plane.set_guided_WP(loc);
    return true;
}

void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif

    // Received an external msg that guides roll in the last 3 seconds?
    if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < 3000) {
        plane.nav_roll_cd = constrain_int32(plane.guided_state.forced_rpy_cd.x, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // guided_state.target_heading is radians at this point between -pi and pi ( defaults to -4 )
    // This function is used in Guided and AvoidADSB, check for guided
    } else if ((plane.control_mode == &plane.mode_guided) && (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
        uint32_t tnow = AP_HAL::millis();
        float delta = (tnow - plane.guided_state.target_heading_time_ms) * 1e-3f;
        plane.guided_state.target_heading_time_ms = tnow;

        float error = 0.0f;
        if (plane.guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
            error = wrap_PI(plane.guided_state.target_heading - AP::ahrs().get_yaw());
        } else {
            Vector2f groundspeed = AP::ahrs().groundspeed_vector();
            error = wrap_PI(plane.guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
        }

        float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
        bank_limit = MIN(bank_limit, plane.roll_limit_cd);

        // push error into AC_PID
        const float desired = plane.g2.guidedHeading.update_error(error, delta, plane.guided_state.target_heading_limit);

        // Check for output saturation
        plane.guided_state.target_heading_limit = fabsf(desired) >= bank_limit;

        plane.nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
        plane.update_load_factor();

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else {
        plane.calc_nav_roll();
    }

    if (plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < 3000) {
        plane.nav_pitch_cd = constrain_int32(plane.guided_state.forced_rpy_cd.y, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    } else {
        plane.calc_nav_pitch();
    }

    // Throttle output
    if (plane.guided_throttle_passthru) {
        // manual passthrough of throttle in fence breach
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));

    }  else if (plane.aparm.throttle_cruise > 1 &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < 3000) {
        // Received an external msg that guides throttle in the last 3 seconds?
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.guided_state.forced_throttle);

    } else {
        // TECS control
        plane.calc_throttle();

    }

}

void ModeGuided::navigate()
{
    plane.update_loiter(active_radius_m);
}

bool ModeGuided::handle_guided_request(Location target_loc)
{
    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    plane.set_guided_WP(target_loc);

    return true;
}

void ModeGuided::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    // constrain to (uint16_t) range for update_loiter()
    active_radius_m = constrain_int32(fabsf(radius), 0, UINT16_MAX);
    plane.loiter.direction = direction_is_ccw ? -1 : 1;
}

void ModeGuided::update_target_altitude()
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // target altitude can be negative (e.g. flying below home altitude from the top of a mountain)
    if (((plane.guided_state.target_alt_time_ms != 0) || plane.guided_state.target_location.alt != -1 )) { // target_alt now defaults to -1, and _time_ms defaults to zero.
        // offboard altitude demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - plane.guided_state.target_alt_time_ms);
        plane.guided_state.target_alt_time_ms = now;
        // determine delta accurately as a float
        float delta_amt_f = delta * plane.guided_state.target_alt_rate;
        // then scale x100 to match last_target_alt and convert to a signed int32_t as it may be negative
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        // To calculate the required velocity (up or down), we need to target and current altitudes in the target frame
        const Location::AltFrame target_frame = plane.guided_state.target_location.get_alt_frame();
        int32_t target_alt_previous_cm;
        if (plane.current_loc.initialised() && plane.guided_state.target_location.initialised() && 
            plane.current_loc.get_alt_cm(target_frame, target_alt_previous_cm)) {
            // create a new interim target location that that takes current_location and moves delta_amt_i in the right direction
            int32_t temp_alt_cm = constrain_int32(plane.guided_state.target_location.alt, target_alt_previous_cm - delta_amt_i,  target_alt_previous_cm + delta_amt_i);
            Location temp_location = plane.guided_state.target_location;
            temp_location.set_alt_cm(temp_alt_cm, target_frame);

            // incrementally step the altitude towards the target            
            plane.set_target_altitude_location(temp_location);
        }
    } else 
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        {
        Mode::update_target_altitude();
    }
}



"""