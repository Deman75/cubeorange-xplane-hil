#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_HITL.h"

#if HAL_EXTERNAL_AHRS_HITL_ENABLED
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Arming/AP_Arming.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_HITL::AP_ExternalAHRS_HITL(AP_ExternalAHRS *_frontend,
    AP_ExternalAHRS::state_t &_state) : AP_ExternalAHRS_backend(_frontend, _state) {

    // external ekf initial state
    state.have_location = false;
    state.have_origin = false;
    state.have_quaternion = false;
    state.have_velocity = false;

    // start serial communications
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "HITL: No serial port defined as type ExternalAHRS");
        return;
    }

    // thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_HITL::thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("HITL: Failed to allocate HITL driver thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HITL: Initialized");
}

int8_t AP_ExternalAHRS_HITL::get_port(void) const {
    auto &sm = AP::serialmanager();
    return sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
};

bool AP_ExternalAHRS_HITL::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    if (!initialised() || !healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "HIL2: no fresh X-Plane state");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_HITL::thread(void) {
    auto &sm = AP::serialmanager();
    uart->begin(sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0));

    while (true) {
        uint32_t now = AP_HAL::millis();
        // find efi
#if AP_EFI_SCRIPTING_ENABLED
        if (efi == nullptr) {
            efi = AP::EFI() != nullptr ? AP::EFI()->get_backend(0) : nullptr;
            if (efi != nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HITL: EFI Enabled");
            }
        }
#endif
        // keep track of how many state packets have been received
        // every second (1000 ms)
        // these are shown in the xplane plugin and it should match the fps
        if (now - ahrs_count_time > 1000) {
            state_msg.ahrs_count = ahrs_count;
            ahrs_count = 0;
            ahrs_count_time = now;
        }
        // read and process data from xplane plugin
        uint32_t last_ahrs_time = on_ahrs_time;
        read(now);
        if (!healthy()) {
            // not connected, spam sensor data to avoid ardupilot complaining
            // and send pings
            push_sensors();
            if (now - last_ping_time > ping_interval) {
                send_ping();
                last_ping_time = now;
            }
        } else {
            // connected, on new telemetry from xplane
            // respond with servo outputs
            if (on_ahrs_time > last_ahrs_time) {
                push_sensors();
                push_ekf();
#if AP_EFI_SCRIPTING_ENABLED
                push_efi();
#endif
                send_rc();
            }
        }
        hal.scheduler->delay(1);
    }
}

void AP_ExternalAHRS_HITL::read(uint32_t t) {
    uint32_t nbytes = uart->available();
    struct _header header;
    struct _footer footer;
    while (nbytes-- > 0) {
        // read a single byte into the buffer
        if (!uart->read(&buffer[pos], 1)) {
            continue;
        }
        // check if the first bytes of the message match the preamble
        if (pos < sizeof(header.preamble)) {
            if (buffer[pos] != header.preamble[pos]) {
                pos = 0;
                continue;
            }
        }
        // if we get to this point it means the first bytes have matched the
        // header so far
        pos++;
        if (pos == sizeof(header)) {
            memcpy(&header, buffer, sizeof(header));
            if (header.type == 1) {
                on_reset();
                pos = 0;
            }
        } else if (pos == sizeof(ahrs_msg) + sizeof(header) + sizeof(footer)) {
            pos = 0;
            memcpy(&header, buffer, sizeof(header));
            memcpy(&footer, &buffer[sizeof(header) + sizeof(ahrs_msg)], sizeof(footer));
            if (header.type != 0) { continue; }
            if (footer.len != sizeof(header) + sizeof(ahrs_msg)) { continue; }
            if (strncmp(footer.postamble, "END", 3) != 0) { continue; };
            SensorWire incoming;
            memcpy(&incoming, &buffer[sizeof(header)], sizeof(incoming));
            const float qnorm = sq(incoming.q1) + sq(incoming.q2) + sq(incoming.q3) + sq(incoming.q4);
            // Reject corrupt/non-finite state before refreshing link health or publishing sensors.
            if (!(incoming.baro.pressure_pa > 100.0f && incoming.baro.pressure_pa < 120000.0f) ||
                !(incoming.baro.temperature > -100.0f && incoming.baro.temperature < 150.0f) ||
                !(qnorm > 0.9f && qnorm < 1.1f) ||
                !(incoming.ins.accel.length_squared() < 1.0e8f) ||
                !(incoming.ins.gyro.length_squared() < 1.0e6f) ||
                !(incoming.mag.field.length_squared() < 1.0e8f) ||
                !(incoming.aspd.differential_pressure >= 0 && incoming.aspd.differential_pressure < 1.0e6f) ||
                incoming.gps.latitude < -900000000 || incoming.gps.latitude > 900000000 ||
                incoming.gps.longitude < -1800000000 || incoming.gps.longitude > 1800000000 ||
                !isfinite(incoming.gps.ned_vel_north) || !isfinite(incoming.gps.ned_vel_east) ||
                !isfinite(incoming.gps.ned_vel_down)) {
                continue;
            }
            ahrs_msg = incoming;
            on_ahrs(t);
        }
    }
}

float random_float();
float random_float() {
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

void AP_ExternalAHRS_HITL::on_ahrs(uint32_t t) {
    // baro readings are considered unhealthy if its the same everytime
    // this can happen if the plane is parked or in calibration
    ahrs_msg.baro.pressure_pa += random_float();
    ahrs_msg.baro.temperature += random_float();
    // scale compass reading magnitude to match gps location
    float intensity;
    float declination;
    float inclination;
    if (AP_Declination::get_mag_field_ef(
        ahrs_msg.gps.latitude * 1e-7f,
        ahrs_msg.gps.longitude * 1e-7f,
        intensity, declination, inclination)) {
        ahrs_msg.mag.field *= (intensity * 1000);
    }
    on_ahrs_time = t;
    ahrs_msg.efi.last_updated_ms = t;
    ahrs_count++;
}

void AP_ExternalAHRS_HITL::push_sensors() {
    AP::ins().handle_external(ahrs_msg.ins);
    AP::compass().handle_external(ahrs_msg.mag);
    AP::baro().handle_external(ahrs_msg.baro);
    AP::gps().handle_external(ahrs_msg.gps);
    if (AP::airspeed() != nullptr) {
        AP::airspeed()->handle_external(ahrs_msg.aspd);
    }
}

void AP_ExternalAHRS_HITL::push_ekf() {
    WITH_SEMAPHORE(state.sem);
    state.accel = ahrs_msg.ins.accel;
    state.gyro = ahrs_msg.ins.gyro;
    state.location = { ahrs_msg.gps.latitude, ahrs_msg.gps.longitude, ahrs_msg.gps.msl_altitude, Location::AltFrame::ABSOLUTE };
    state.have_location = true;
    state.velocity = { ahrs_msg.gps.ned_vel_north, ahrs_msg.gps.ned_vel_east, ahrs_msg.gps.ned_vel_down };
    state.have_velocity = true;
    state.quat = { ahrs_msg.q1, ahrs_msg.q2, ahrs_msg.q3, ahrs_msg.q4 };
    state.have_quaternion = true;
    if (!state.have_origin) {
        state.origin = state.location;
        state.have_origin = true;
    }
}

#if AP_EFI_SCRIPTING_ENABLED
void AP_ExternalAHRS_HITL::push_efi() {
    if (efi != nullptr) {
        efi->handle_scripting(ahrs_msg.efi);
    }
}
#endif

// restart ahrs

void AP_ExternalAHRS_HITL::on_reset() {
    WITH_SEMAPHORE(state.sem);
    state.have_origin = false;
    state.have_location = false;
    state.have_velocity = false;
    state.have_quaternion = false;
    on_ahrs_time = 0;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HIL2: simulator state reset");
}

// rc

void AP_ExternalAHRS_HITL::send_rc() {
    struct _header header;
    header.type = 1;
    ///bool safety_on = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
    bool safety_on = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;
    bool armed = AP::arming().is_armed();
    if (safety_on) {
        state_msg.state = 0;
    } else {
        state_msg.state = armed ? 2 : 1;
    }
    state_msg.starter = 0;
    SRV_Channels::get_output_pwm(SRV_Channel::k_starter, state_msg.starter);
    uart->write(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    uart->write(reinterpret_cast<uint8_t *>(&state_msg), sizeof(state_msg));
    struct _footer footer;
    footer.len = sizeof(header) + sizeof(state_msg);
    uart->write(reinterpret_cast<uint8_t *>(&footer), sizeof(footer));
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // HIL2 plane message: four PWM commands and flap deployment in 0.01%.
    // Type 4 deliberately differs from the upstream four-command packet.
    static_assert(sizeof(plane_msg) == 10, "Unexpected HIL2 plane wire layout");
    header.type = 4;
    plane_msg.roll = 1500;
    plane_msg.pitch = 1500;
    plane_msg.yaw = 1500;
    plane_msg.throttle = 1100;
    SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, plane_msg.roll);
    SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, plane_msg.pitch);
    SRV_Channels::get_output_pwm(SRV_Channel::k_throttle, plane_msg.throttle);
    SRV_Channels::get_output_pwm(SRV_Channel::k_rudder, plane_msg.yaw);
    plane_msg.flap_percent_x100 = uint16_t(constrain_float(
        SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto), 0.0f, 100.0f) * 100.0f);
    uart->write(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    uart->write(reinterpret_cast<uint8_t *>(&plane_msg), sizeof(plane_msg));
    footer.len = sizeof(header) + sizeof(plane_msg);
    uart->write(reinterpret_cast<uint8_t *>(&footer), sizeof(footer));

    // Optional type 5 adds landing gear without changing the existing type 4 wire layout.
    // Use the controller's output, not raw RC input, so RC6_OPTION/LGR settings apply.
    uint16_t gear_pwm = 0;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_landing_gear_control, gear_pwm) &&
        gear_pwm >= 900 && gear_pwm <= 2100) {
        struct {
            uint16_t deployed;
        } gear_msg { uint16_t(gear_pwm >= 1500) };
        static_assert(sizeof(gear_msg) == 2, "Unexpected HIL2 gear wire layout");
        header.type = 5;
        uart->write(reinterpret_cast<uint8_t *>(&header), sizeof(header));
        uart->write(reinterpret_cast<uint8_t *>(&gear_msg), sizeof(gear_msg));
        footer.len = sizeof(header) + sizeof(gear_msg);
        uart->write(reinterpret_cast<uint8_t *>(&footer), sizeof(footer));
    }
    // Optional HIL2 type 6: symmetric service brakes from calibrated RC channel 9.
    // No servo output assignment or RC9_OPTION is needed; do not consume RC8.
    const RC_Channel *brake_channel = rc().channel(8);
    const bool brake_input_valid = brake_channel != nullptr &&
        rc().has_valid_input() && RC_Channels::get_valid_channel_count() >= 9;
    struct { uint16_t ratio_x10000; } brake_msg {
        uint16_t(brake_input_valid ? constrain_float(
            (brake_channel->norm_input_ignore_trim()+1.0f)*5000.0f,0.0f,10000.0f) : 0)
    };
    static_assert(sizeof(brake_msg)==2, "Unexpected HIL2 brake wire layout");
    header.type=6;
    uart->write(reinterpret_cast<uint8_t *>(&header),sizeof(header));
    uart->write(reinterpret_cast<uint8_t *>(&brake_msg),sizeof(brake_msg));
    footer.len=sizeof(header)+sizeof(brake_msg);
    uart->write(reinterpret_cast<uint8_t *>(&footer),sizeof(footer));
#endif
#if APM_BUILD_TYPE(APM_BUILD_Heli)
    header.type = 3;
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor1, heli_msg.roll_cyclic);
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor2, heli_msg.pitch_cyclic);
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor3, heli_msg.collective);
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor4, heli_msg.tail);
    SRV_Channels::get_output_pwm(SRV_Channel::k_heli_rsc, heli_msg.throttle);
    uart->write(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    uart->write(reinterpret_cast<uint8_t *>(&heli_msg), sizeof(heli_msg));
    footer.len = sizeof(header) + sizeof(heli_msg);
    uart->write(reinterpret_cast<uint8_t *>(&footer), sizeof(footer));
#endif
}

void AP_ExternalAHRS_HITL::send_ping() {
    struct _header header;
    header.type = 0;
    uart->write(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    uart->write(reinterpret_cast<uint8_t *>(&ping_msg[0]), sizeof(ping_msg));
    struct _footer footer;
    footer.len = sizeof(header) + sizeof(ping_msg);
    uart->write(reinterpret_cast<uint8_t *>(&footer), sizeof(footer));
}

// external EKF
// not really an EKF, just grab attitude quaternion directly from X-Plane

bool AP_ExternalAHRS_HITL::healthy(void) const {
    return initialised() && (AP_HAL::millis() - on_ahrs_time < unhealthy_ahrs_ms);
}

bool AP_ExternalAHRS_HITL::initialised(void) const {
    return (on_ahrs_time > 0);
}

void AP_ExternalAHRS_HITL::get_filter_status(nav_filter_status &status) const {
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
    if (healthy()) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;
        status.flags.horiz_vel = 1;
        status.flags.horiz_pos_rel = 1;
        status.flags.horiz_pos_abs = 1;
        status.flags.pred_horiz_pos_rel = 1;
        status.flags.pred_horiz_pos_abs = 1;
        status.flags.using_gps = 1;
    }
}

void AP_ExternalAHRS_HITL::send_status_report(GCS_MAVLINK &link) const {
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags, 0, 0, 0, 0, 0, 0);
    return;
}

void AP_ExternalAHRS_HITL::update() {
    return;
}

#endif
