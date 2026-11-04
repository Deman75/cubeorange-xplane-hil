#pragma once

#include "AP_ExternalAHRS_backend.h"
#include <stddef.h>

#ifndef HAL_EXTERNAL_AHRS_HITL_ENABLED
#define HAL_EXTERNAL_AHRS_HITL_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#if HAL_EXTERNAL_AHRS_HITL_ENABLED
#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_EFI/AP_EFI.h>

class AP_ExternalAHRS_HITL : public AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_HITL(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);
    int8_t get_port(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    const char *get_name() const override { return "HITL"; };

    bool healthy(void) const override;
    bool initialised(void) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;
    void update() override;

private:
    struct _header {
        char preamble[4] = { 'H', 'I', 'L', '2' };
        int type;
    };

    struct _footer {
        int len;
        char postamble[3] = { 'E','N','D' };
    };

    struct {
        AP_ExternalAHRS::baro_data_message_t baro{
            .instance = 0,
            .pressure_pa = 101325,
            .temperature = 25
        };
        AP_ExternalAHRS::mag_data_message_t mag{
            .field = Vector3f(400, 0, 0)
        };
        AP_ExternalAHRS::gps_data_message_t gps{
            .fix_type = GPS_FIX_TYPE_NO_FIX
        };
        AP_ExternalAHRS::ins_data_message_t ins{
            .accel = Vector3f(0, 0, -9.80665f)
        };
        AP_ExternalAHRS::airspeed_data_message_t aspd{};
        float q1, q2, q3, q4;
        EFI_State efi{};
    } ahrs_msg;
    using SensorWire = decltype(ahrs_msg);

    // HIL2 uses native little-endian records: enforce the same layout on ARM and Windows.
    static_assert(sizeof(ahrs_msg) == 228, "HIL2 sensor body size");
    static_assert(offsetof(SensorWire, baro) == 0, "HIL2 baro offset");
    static_assert(offsetof(SensorWire, mag) == 12, "HIL2 mag offset");
    static_assert(offsetof(SensorWire, gps) == 24, "HIL2 GPS offset");
    static_assert(offsetof(SensorWire, ins) == 80, "HIL2 IMU offset");
    static_assert(offsetof(SensorWire, aspd) == 108, "HIL2 airspeed offset");
    static_assert(offsetof(SensorWire, q1) == 116, "HIL2 quaternion offset");
    static_assert(offsetof(SensorWire, efi) == 132, "HIL2 EFI offset");
    static_assert(sizeof(EFI_State) == 96, "HIL2 EFI size");
    static_assert(sizeof(_header) == 8 && sizeof(_footer) == 8, "HIL2 framing size");


    struct {
        uint8_t state;
        uint32_t ahrs_count;
        uint16_t starter;
    } state_msg;

    struct {
        uint16_t roll_cyclic;
        uint16_t pitch_cyclic;
        uint16_t collective;
        uint16_t tail;
        uint16_t throttle;
    } heli_msg;

    struct {
        uint16_t roll;
        uint16_t pitch;
        uint16_t yaw;
        uint16_t throttle;
        uint16_t flap_percent_x100;
    } plane_msg;

    char ping_msg[21] = "PINGHIL2PINGHIL2PING";

    AP_HAL::UARTDriver *uart;

    // serial rx buffer
    uint8_t buffer[600];
    long unsigned int pos = 0;

    // time of most recent telemetry packet
    uint32_t on_ahrs_time = 0;

    // keep track of how many ahrs packet have been received
    uint32_t ahrs_count_time = 0;
    uint32_t ahrs_count = 0;

    // time without updates before the ekf ahrs becomes unhealthy
    const uint32_t unhealthy_ahrs_ms = 100;

    // ping timer
    uint32_t last_ping_time = 0;
    const uint32_t ping_interval = 1000;

    // efi
    AP_EFI_Backend *efi;

    void thread();

    void read(uint32_t t);
    void on_ahrs(uint32_t t);
    void on_reset();
    void push_sensors();
    void push_ekf();
#if AP_EFI_SCRIPTING_ENABLED
    void push_efi();
#endif
    void send_rc();
    void send_ping();
};

#endif
