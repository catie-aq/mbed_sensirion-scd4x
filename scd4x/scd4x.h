/*
 * Copyright (c) 2021, Koncepto.io
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_SCD4X_H_
#define CATIE_SIXTRON_SCD4X_H_

#include "mbed.h"
#include <chrono>
#include <cstdint>

namespace sixtron {

using rtos::Kernel::Clock;
using std::chrono::operator""ms;

typedef struct {
    double temperature;
    double rh;
    uint16_t co2;
} scd4x_measurement_t;

/*!
 *  \class SCD4X
 *  SCD4X CO2 sensor driver
 */
class SCD4X {
public:
    enum class ErrorType {
        Ok = 0,
        I2cError,
        CrcError,
        DataNotReady,
        FrcError,
        SelfTestError,
    };

    SCD4X(I2C *bus);

    ErrorType start_periodic_measurement();
    ErrorType read_measurement(scd4x_measurement_t *data);
    ErrorType stop_periodic_measurement();

    ErrorType set_temperature_offset(float t);
    ErrorType get_temperature_offset(float *t);

    ErrorType set_sensor_altitude(uint16_t alt);
    ErrorType get_sensor_altitude(uint16_t *alt);
    ErrorType set_ambient_pressure(uint16_t hpa);

    ErrorType perform_forced_calibration(uint16_t target_co2, uint16_t *frc_correction);
    ErrorType set_automatic_calibration_enabled(bool enable);
    ErrorType get_automatic_calibration_enabled(bool *enable);

    ErrorType start_low_power_periodic_measurement();
    ErrorType get_data_ready_status();

    ErrorType persist_settings();
    ErrorType get_serial_number(uint16_t serial[3]);

    ErrorType perform_self_test();
    ErrorType perfom_factory_reset();
    ErrorType reinit();

    ErrorType measure_single_shot_rht_only();
    ErrorType measure_single_shot();

private:
    enum class Command : uint16_t {
        StartPeriodicMeasurement = 0x21b1,
        ReadMeasurement = 0xec05,
        StopPeriodicMeasurement = 0x3f86,
        SetTemperatureOffset = 0x241d,
        GetTemperatureOffset = 0x2318,
        SetSensorAltitude = 0x2427,
        GetSensorAltitude = 0x2322,
        SetAmbientPressure = 0xe000,
        PerformForcedRecalibration = 0x362f,
        SetAutomaticSelfCalibrationEnabled = 0x2416,
        GetAutomaticSelfCalibrationEnabled = 0x2313,
        StartLowPowerPeriodicMeasurement = 0x21ac,
        GetDataReadyStatus = 0xe4b8,
        PersistSettings = 0x3615,
        GetSerialNumber = 0x3682,
        PerformSelfTest = 0x3639,
        PerformFactoryReset = 0x3632,
        Reinit = 0x3646,
        MeasureSingleShot = 0x219d,
        MeasureSingleShotRhtOnly = 0x2196,
    };

    I2C *_bus;
    char compute_crc(char *data, uint8_t len);
    ErrorType send_command(Command cmd);
    ErrorType read(
            Command cmd, uint16_t len, uint16_t *val_out, Clock::duration_u32 exec_time = 1ms);
    ErrorType write(Command cmd, uint16_t len, uint16_t *val_in);
    ErrorType send_and_fetch(
            Command cmd, uint16_t *val_in, uint16_t *val_out, Clock::duration_u32 exec_time = 1ms);
};

} // namespace sixtron

#endif // CATIE_SIXTRON_SCD4X_H_
