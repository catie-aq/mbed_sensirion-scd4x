/*
 * Copyright (c) 2021, Koncepto.io
 * SPDX-License-Identifier: Apache-2.0
 */
#include "scd4x.h"
#include "MbedCRC.h"
#include <cmath>

#define SCD4X_ADDR (0x62 << 1)

#define SCD4X_POLY (0x31)
#define SCD4X_WIDTH_POLY (8)
#define SCD4X_INIT_XOR (0xFF)
#define SCD4X_FINAL_XOR (0)
#define SCD4X_REFLECT_IN (false)
#define SCD4X_REFLECT_OUT (false)

#define U16_TO_BYTE_ARRAY(u, ba)                                                                   \
    do {                                                                                           \
        ba[0] = (u >> 8) & 0xFF;                                                                   \
        ba[1] = u & 0xFF;                                                                          \
    } while (0)

#define BYTE_ARRAY_TO_U16(ba, u)                                                                   \
    do {                                                                                           \
        u = (ba[0] << 8) | (ba[1]);                                                                \
    } while (0)

namespace sixtron {

static double raw_to_temperature(uint16_t raw, double offset)
{
    return (offset + (double)(raw)*175.0 / (1 << 16));
}

static uint16_t temperature_to_raw(double t)
{
    return round(t * (1 << 16) / 175);
}

static double raw_to_rh(uint16_t raw)
{
    return ((double)(raw)*100.0 / (1 << 16));
}

Scd4x::Scd4x(I2C *bus): _bus(bus)
{
}

Scd4x::ErrorType Scd4x::start_periodic_measurement()
{
    return this->send_command(Command::StartPeriodicMeasurement);
}

Scd4x::ErrorType Scd4x::read_measurement(scd4x_measurement_t *data)
{
    ErrorType err;
    uint16_t buf[3];

    err = this->read(Command::ReadMeasurement, 3, buf);
    if (err != ErrorType::Ok) {
        goto read_measurement_end;
    }

    data->co2 = buf[0];
    data->temperature = raw_to_temperature(buf[1], -45.0);
    data->rh = raw_to_rh(buf[2]);

read_measurement_end:
    return err;
}

Scd4x::ErrorType Scd4x::stop_periodic_measurement()
{
    return this->send_command(Command::StopPeriodicMeasurement);
}

Scd4x::ErrorType Scd4x::set_temperature_offset(float t)
{
    uint16_t data = temperature_to_raw(t);
    return this->write(Command::SetTemperatureOffset, 1, &data);
}

Scd4x::ErrorType Scd4x::get_temperature_offset(float *t)
{
    uint16_t data;
    ErrorType err;

    err = this->read(Command::GetTemperatureOffset, 1, &data);
    if (err == ErrorType::Ok) {
        *t = raw_to_temperature(data, 0.0);
    }
    return err;
}

Scd4x::ErrorType Scd4x::set_sensor_altitude(uint16_t alt)
{
    return this->write(Command::SetSensorAltitude, 1, &alt);
}

Scd4x::ErrorType Scd4x::get_sensor_altitude(uint16_t *alt)
{
    return this->read(Command::GetSensorAltitude, 1, alt);
}

Scd4x::ErrorType Scd4x::set_ambient_pressure(uint16_t hpa)
{
    return this->write(Command::SetAmbientPressure, 1, &hpa);
}

Scd4x::ErrorType Scd4x::perform_forced_calibration(uint16_t target_co2, uint16_t *frc_correction)
{
    ErrorType err;
    uint16_t frc_result;

    err = this->send_and_fetch(
            Command::PerformForcedRecalibration, &target_co2, &frc_result, 400ms);

    if (err == ErrorType::Ok) {
        if (frc_result == 0xFFFF) {
            err = ErrorType::FrcError;
        } else {
            *frc_correction = frc_result - 0x8000;
        }
    }

    return err;
}

Scd4x::ErrorType Scd4x::set_automatic_calibration_enabled(bool enable)
{
    uint16_t tmp = (enable) ? 1 : 0;
    return this->write(Command::SetAutomaticSelfCalibrationEnabled, 1, &tmp);
}

Scd4x::ErrorType Scd4x::get_automatic_calibration_enabled(bool *enable)
{
    uint16_t tmp;
    ErrorType err;

    err = this->read(Command::GetAutomaticSelfCalibrationEnabled, 1, &tmp);
    *enable = (tmp == 0) ? false : true;
    return err;
}

Scd4x::ErrorType Scd4x::start_low_power_periodic_measurement()
{
    return this->send_command(Command::StartLowPowerPeriodicMeasurement);
}

Scd4x::ErrorType Scd4x::get_data_ready_status()
{
    ErrorType err;
    uint16_t data;

    err = this->read(Command::GetDataReadyStatus, 1, &data);

    if ((err == ErrorType::Ok) && ((data & 0x3FF) == 0)) {
        err = ErrorType::DataNotReady;
    }

    return err;
}

Scd4x::ErrorType Scd4x::persist_settings()
{
    return this->send_command(Command::PersistSettings);
}

Scd4x::ErrorType Scd4x::get_serial_number(uint16_t serial[3])
{
    return this->read(Command::GetSerialNumber, 3, serial);
}

Scd4x::ErrorType Scd4x::perform_self_test()
{
    ErrorType err;
    uint16_t data;

    err = this->read(Command::PerformSelfTest, 1, &data, 10s);

    if ((err == ErrorType::Ok) && (data != 0)) {
        err = ErrorType::SelfTestError;
    }

    return err;
}

Scd4x::ErrorType Scd4x::perfom_factory_reset()
{
    return this->send_command(Command::PerformFactoryReset);
}

Scd4x::ErrorType Scd4x::reinit()
{
    return this->send_command(Command::Reinit);
}

Scd4x::ErrorType Scd4x::measure_single_shot_rht_only()
{
    return this->send_command(Command::MeasureSingleShotRhtOnly);
}

Scd4x::ErrorType Scd4x::measure_single_shot()
{
    return this->send_command(Command::MeasureSingleShot);
}

char Scd4x::compute_crc(char *data, uint8_t len)
{
    uint32_t crc;
    MbedCRC<SCD4X_POLY, SCD4X_WIDTH_POLY> ct(
            SCD4X_INIT_XOR, SCD4X_FINAL_XOR, SCD4X_REFLECT_IN, SCD4X_REFLECT_OUT);

    ct.compute(data, len, &crc);
    return (crc & 0xFF);
}

Scd4x::ErrorType Scd4x::send_command(Command cmd)
{
    ErrorType retval = ErrorType::Ok;

    char bytes[2];
    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), bytes);

    if (this->_bus->write(SCD4X_ADDR, bytes, 2, false)) {
        retval = ErrorType::I2cError;
    }

    return retval;
}

Scd4x::ErrorType Scd4x::read(
        Command cmd, uint16_t len, uint16_t *val_out, Clock::duration_u32 exec_time)
{
    char bytes[len * 3];
    ErrorType retval = ErrorType::Ok;

    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), bytes);

    this->_bus->lock();

    if (this->_bus->write(SCD4X_ADDR, bytes, 2, true)) {
        retval = ErrorType::I2cError;
        this->_bus->stop();
        goto read_end;
    }

    ThisThread::sleep_for(exec_time);

    if (this->_bus->read(SCD4X_ADDR, bytes, 3 * len, false)) {
        retval = ErrorType::I2cError;
        this->_bus->stop();
        goto read_end;
    }

    for (int i = 0; i < len; i++) {
        BYTE_ARRAY_TO_U16((bytes + (3 * i)), val_out[i]);

        if (compute_crc(bytes + (3 * i), 2) != (bytes + (3 * i))[2]) {
            retval = ErrorType::CrcError;
            break;
        }
    }

read_end:
    this->_bus->unlock();
    return retval;
}

Scd4x::ErrorType Scd4x::write(Command cmd, uint16_t len, uint16_t *val_in)
{
    ErrorType retval = ErrorType::Ok;

    char bytes[3];
    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), bytes);

    this->_bus->lock();

    if (this->_bus->write(SCD4X_ADDR, bytes, 2, true)) {
        retval = ErrorType::I2cError;
        this->_bus->stop();
        goto write_end;
    }

    for (int i = 1; i <= len; i++) {
        U16_TO_BYTE_ARRAY(*val_in, bytes);
        bytes[2] = compute_crc(bytes, 2);

        if (this->_bus->write(SCD4X_ADDR, bytes, 3, i == len)) {
            retval = ErrorType::I2cError;
            this->_bus->stop();
            goto write_end;
        }

        val_in++;
    }
write_end:
    this->_bus->unlock();
    return retval;
}

Scd4x::ErrorType Scd4x::send_and_fetch(
        Command cmd, uint16_t *val_in, uint16_t *val_out, Clock::duration_u32 exec_time)
{
    ErrorType retval = ErrorType::Ok;

    char bytes[3];
    U16_TO_BYTE_ARRAY(static_cast<uint16_t>(cmd), bytes);

    this->_bus->lock();

    if (this->_bus->write(SCD4X_ADDR, bytes, 2, true)) {
        retval = ErrorType::I2cError;
        this->_bus->stop();
        goto send_fetch_end;
    }

    U16_TO_BYTE_ARRAY(*val_in, bytes);
    bytes[2] = compute_crc(bytes, 2);

    if (this->_bus->write(SCD4X_ADDR, bytes, 3, true)) {
        retval = ErrorType::I2cError;
        this->_bus->stop();
        goto send_fetch_end;
    }

    ThisThread::sleep_for(exec_time);

    if (this->_bus->read(SCD4X_ADDR, bytes, 3, false)) {
        retval = ErrorType::I2cError;
        this->_bus->stop();
        goto send_fetch_end;
    }

    BYTE_ARRAY_TO_U16(bytes, *val_out);

    if (compute_crc(bytes, 2) != bytes[2]) {
        retval = ErrorType::I2cError;
    }

send_fetch_end:
    this->_bus->unlock();
    return retval;
}

} // namespace sixtron
