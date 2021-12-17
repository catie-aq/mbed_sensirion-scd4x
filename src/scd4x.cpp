/*
 * Copyright (c) 2021, Koncepto.io
 * SPDX-License-Identifier: Apache-2.0
 */
#include "scd4x/scd4x.h"
#include "MbedCRC.h"
#include <cmath>

#define SCD4X_ADDR (0x62 << 1)

#define SCD4X_POLY (0x31)
#define SCD4X_WIDTH_POLY (8)
#define SCD4X_INIT_XOR (0xFF)
#define SCD4X_FINAL_XOR (0)
#define SCD4X_REFLECT_IN (false)
#define SCD4X_REFLECT_OUT (false)

#define SELF_TEST_WAIT_TIME (10s)
#define FORCED_CALIBRATION_WAIT_TIME (400ms)

#define MAX_READ_SIZE (3)

#define U16_TO_BYTE_ARRAY(u, ba)                                                                   \
    do {                                                                                           \
        ba[0] = (u >> 8) & 0xFF;                                                                   \
        ba[1] = u & 0xFF;                                                                          \
    } while (0)

#define BYTE_ARRAY_TO_U16(ba, u)                                                                   \
    do {                                                                                           \
        u = (ba[0] << 8) | (ba[1]);                                                                \
    } while (0)

/*!
 * brief Convert raw binary temperature value from sensor to a double
 *
 * \param raw raw value from the sensor
 * \param offset offset value to apply (depends on the command used)
 *
 * \return temperature in Celsius degrees
 */
static double raw_to_temperature(uint16_t raw, double offset)
{
    return (offset + (double)(raw)*175.0 / (1 << 16));
}

/*!
 * brief Convert temperature in Celsius degrees to sensor raw format
 *
 * \param t temperature in Celsius degrees
 *
 * \return raw binary value for the sensor
 */
static uint16_t temperature_to_raw(double t)
{
    return round(t * (1 << 16) / 175);
}

/*!
 * brief Convert raw binary relative humidity value from sensor to a double
 *
 * \param raw raw value from the sensor
 *
 * \return Relative humidity in %
 */
static double raw_to_rh(uint16_t raw)
{
    return ((double)(raw)*100.0 / (1 << 16));
}

namespace sixtron {

SCD4X::SCD4X(PinName i2c_sda, PinName i2c_scl): _bus(i2c_sda, i2c_scl)
{
}

SCD4X::ErrorType SCD4X::start_periodic_measurement()
{
    return this->send_command(Command::StartPeriodicMeasurement);
}

/** Measurement buffer */
static uint16_t measurement_buf[3];

SCD4X::ErrorType SCD4X::read_measurement(scd4x_measurement_t *data)
{
    ErrorType err;

    err = this->read(Command::ReadMeasurement, 3, measurement_buf);
    if (err != ErrorType::Ok) {
        goto read_measurement_end;
    }

    data->co2 = measurement_buf[0];
    data->temperature = raw_to_temperature(measurement_buf[1], -45.0);
    data->rh = raw_to_rh(measurement_buf[2]);

read_measurement_end:
    return err;
}

SCD4X::ErrorType SCD4X::stop_periodic_measurement()
{
    return this->send_command(Command::StopPeriodicMeasurement);
}

SCD4X::ErrorType SCD4X::set_temperature_offset(float t)
{
    uint16_t data = temperature_to_raw(t);
    return this->write(Command::SetTemperatureOffset, 1, &data);
}

SCD4X::ErrorType SCD4X::get_temperature_offset(float *t)
{
    uint16_t data;
    ErrorType err;

    err = this->read(Command::GetTemperatureOffset, 1, &data);
    if (err == ErrorType::Ok) {
        *t = raw_to_temperature(data, 0.0);
    }
    return err;
}

SCD4X::ErrorType SCD4X::set_sensor_altitude(uint16_t alt)
{
    return this->write(Command::SetSensorAltitude, 1, &alt);
}

SCD4X::ErrorType SCD4X::get_sensor_altitude(uint16_t *alt)
{
    return this->read(Command::GetSensorAltitude, 1, alt);
}

SCD4X::ErrorType SCD4X::set_ambient_pressure(uint16_t hpa)
{
    return this->write(Command::SetAmbientPressure, 1, &hpa);
}

SCD4X::ErrorType SCD4X::perform_forced_calibration(uint16_t target_co2, uint16_t *frc_correction)
{
    ErrorType err;
    uint16_t frc_result;

    err = this->send_and_fetch(Command::PerformForcedRecalibration,
            &target_co2,
            &frc_result,
            FORCED_CALIBRATION_WAIT_TIME);

    if (err == ErrorType::Ok) {
        if (frc_result == 0xFFFF) {
            err = ErrorType::FrcError;
        } else {
            *frc_correction = frc_result - 0x8000;
        }
    }

    return err;
}

SCD4X::ErrorType SCD4X::set_automatic_calibration_enabled(bool enable)
{
    uint16_t tmp = (enable) ? 1 : 0;
    return this->write(Command::SetAutomaticSelfCalibrationEnabled, 1, &tmp);
}

SCD4X::ErrorType SCD4X::get_automatic_calibration_enabled(bool *enable)
{
    uint16_t tmp;
    ErrorType err;

    err = this->read(Command::GetAutomaticSelfCalibrationEnabled, 1, &tmp);
    *enable = (tmp == 0) ? false : true;
    return err;
}

SCD4X::ErrorType SCD4X::start_low_power_periodic_measurement()
{
    return this->send_command(Command::StartLowPowerPeriodicMeasurement);
}

SCD4X::ErrorType SCD4X::get_data_ready_status()
{
    ErrorType err;
    uint16_t data;

    err = this->read(Command::GetDataReadyStatus, 1, &data);

    if ((err == ErrorType::Ok) && ((data & 0x3FF) == 0)) {
        err = ErrorType::DataNotReady;
    }

    return err;
}

SCD4X::ErrorType SCD4X::persist_settings()
{
    return this->send_command(Command::PersistSettings);
}

SCD4X::ErrorType SCD4X::get_serial_number(uint16_t serial[3])
{
    return this->read(Command::GetSerialNumber, 3, serial);
}

SCD4X::ErrorType SCD4X::perform_self_test()
{
    ErrorType err;
    uint16_t data;

    err = this->read(Command::PerformSelfTest, 1, &data, SELF_TEST_WAIT_TIME);

    if ((err == ErrorType::Ok) && (data != 0)) {
        err = ErrorType::SelfTestError;
    }

    return err;
}

SCD4X::ErrorType SCD4X::perfom_factory_reset()
{
    return this->send_command(Command::PerformFactoryReset);
}

SCD4X::ErrorType SCD4X::reinit()
{
    return this->send_command(Command::Reinit);
}

SCD4X::ErrorType SCD4X::measure_single_shot_rht_only()
{
    return this->send_command(Command::MeasureSingleShotRhtOnly);
}

SCD4X::ErrorType SCD4X::measure_single_shot()
{
    return this->send_command(Command::MeasureSingleShot);
}

char SCD4X::compute_crc(char *data, uint8_t len)
{
    uint32_t crc;
    MbedCRC<SCD4X_POLY, SCD4X_WIDTH_POLY> ct(
            SCD4X_INIT_XOR, SCD4X_FINAL_XOR, SCD4X_REFLECT_IN, SCD4X_REFLECT_OUT);

    ct.compute(data, len, &crc);
    return (crc & 0xFF);
}

SCD4X::ErrorType SCD4X::send_command(Command cmd)
{
    ErrorType retval = ErrorType::Ok;

    char bytes[2];
    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), bytes);

    if (this->_bus.write(SCD4X_ADDR, bytes, 2, false)) {
        retval = ErrorType::I2cError;
    }

    return retval;
}

/** Read buffer */
static char read_buf[3 * MAX_READ_SIZE];

SCD4X::ErrorType SCD4X::read(
        Command cmd, uint8_t len, uint16_t *val_out, Clock::duration_u32 exec_time)
{

    ErrorType retval = ErrorType::Ok;

    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), read_buf);

    this->_bus.lock();

    if (len > MAX_READ_SIZE) {
        retval = ErrorType::ReadSizeTooLarge;
        goto read_end;
    }
    if (this->_bus.write(SCD4X_ADDR, read_buf, 2, true)) {
        retval = ErrorType::I2cError;
        this->_bus.stop();
        goto read_end;
    }

    ThisThread::sleep_for(exec_time);

    if (this->_bus.read(SCD4X_ADDR, read_buf, 3 * len, false)) {
        retval = ErrorType::I2cError;
        this->_bus.stop();
        goto read_end;
    }

    for (int i = 0; i < len; i++) {
        BYTE_ARRAY_TO_U16((read_buf + (3 * i)), val_out[i]);

        if (compute_crc(read_buf + (3 * i), 2) != (read_buf + (3 * i))[2]) {
            retval = ErrorType::CrcError;
            break;
        }
    }

read_end:
    this->_bus.unlock();
    return retval;
}

SCD4X::ErrorType SCD4X::write(Command cmd, uint8_t len, uint16_t *val_in)
{
    ErrorType retval = ErrorType::Ok;

    char bytes[3];
    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), bytes);

    this->_bus.lock();

    if (this->_bus.write(SCD4X_ADDR, bytes, 2, true)) {
        retval = ErrorType::I2cError;
        this->_bus.stop();
        goto write_end;
    }

    for (int i = 1; i <= len; i++) {
        U16_TO_BYTE_ARRAY(*val_in, bytes);
        bytes[2] = compute_crc(bytes, 2);

        if (this->_bus.write(SCD4X_ADDR, bytes, 3, i == len)) {
            retval = ErrorType::I2cError;
            this->_bus.stop();
            goto write_end;
        }

        val_in++;
    }
write_end:
    this->_bus.unlock();
    return retval;
}

SCD4X::ErrorType SCD4X::send_and_fetch(
        Command cmd, uint16_t *val_in, uint16_t *val_out, Clock::duration_u32 exec_time)
{
    ErrorType retval = ErrorType::Ok;

    char bytes[3];
    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), bytes);

    this->_bus.lock();

    if (this->_bus.write(SCD4X_ADDR, bytes, 2, true)) {
        retval = ErrorType::I2cError;
        this->_bus.stop();
        goto send_fetch_end;
    }

    U16_TO_BYTE_ARRAY(*val_in, bytes);
    bytes[2] = compute_crc(bytes, 2);

    if (this->_bus.write(SCD4X_ADDR, bytes, 3, true)) {
        retval = ErrorType::I2cError;
        this->_bus.stop();
        goto send_fetch_end;
    }

    ThisThread::sleep_for(exec_time);

    if (this->_bus.read(SCD4X_ADDR, bytes, 3, false)) {
        retval = ErrorType::I2cError;
        this->_bus.stop();
        goto send_fetch_end;
    }

    BYTE_ARRAY_TO_U16(bytes, *val_out);

    if (compute_crc(bytes, 2) != bytes[2]) {
        retval = ErrorType::I2cError;
    }

send_fetch_end:
    this->_bus.unlock();
    return retval;
}

} // namespace sixtron
