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

/*!
 * \brief Sensor data
 */
typedef struct {
    /** Temperature in Celsius degrees */
    double temperature;
    /** Relative humidity in % */
    double rh;
    /** CO2 in ppm */
    uint16_t co2;
} scd4x_measurement_t;

/*!
 *  \class SCD4X
 *  SCD4X CO2 sensor driver
 */
class SCD4X {
public:
    /*!
     * \brief Commands error codes
     */
    enum class ErrorType {
        /** Command successful */
        Ok = 0,
        /** Error returned by I2C communication */
        I2cError,
        /** Requested read size is too large */
        ReadSizeTooLarge,
        /** CRC mismatch in received frame */
        CrcError,
        /** Sensor data is not ready to be read */
        DataNotReady,
        /** Forced calibration failure */
        FrcError,
        /** Self test failure */
        SelfTestError,
    };

    /*! Constructor
     *
     * \param i2c_sda I2C SDA pin
     * \param i2c_scl I2C SCL pin
     */
    SCD4X(PinName i2c_sda, PinName i2c_scl);

    /*!
     * \brief Send command to start periodic measurement
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType start_periodic_measurement();

    /*!
     * \brief Send command to start periodic measurement
     *
     * \param data pointer to data to store read values
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType read_measurement(scd4x_measurement_t *data);

    /*!
     * \brief Send command to stop periodic measurement
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType stop_periodic_measurement();

    /*!
     * \brief Set temperature offset
     *
     * \param t temperature in Celsius degrees to set
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType set_temperature_offset(float t);

    /*!
     * \brief Get temperature offset
     *
     * \param t pointer to data to store read temperature in Celsius degrees
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType get_temperature_offset(float *t);

    /*!
     * \brief Set sensor altitude
     *
     * \param alt altitude in meters to set
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType set_sensor_altitude(uint16_t alt);

    /*!
     * \brief Get sensor altitude
     *
     * \param alt pointer to data to store read altitude in meters
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType get_sensor_altitude(uint16_t *alt);

    /*!
     * \brief Set ambient pressure
     *
     * \param hpa pressure in hPa to set
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType set_ambient_pressure(uint16_t hpa);

    /*!
     * \brief Perform a forced calibration
     *
     * \param target_co2 CO2 reference for the calibration in ppm
     * \param frc_correction pointer to data to store read correction in ppm
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType perform_forced_calibration(uint16_t target_co2, uint16_t *frc_correction);

    /*!
     * \brief Enable or disable automatic calibration
     *
     * \param enable true to enable automatic calibration, false to disable it
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType set_automatic_calibration_enabled(bool enable);

    /*!
     * \brief Get status of automatic calibration
     *
     * \param enable pointer to data to store read status
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType get_automatic_calibration_enabled(bool *enable);

    /*!
     * \brief Start low power periodic measurement
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType start_low_power_periodic_measurement();

    /*!
     * \brief Read data ready status
     *
     * \return Ok if data ready, DataNotReady if data not ready, the reason of failure otherwise
     */
    ErrorType get_data_ready_status();

    /*!
     * \brief Save settings to internal EEPROM
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType persist_settings();

    /*!
     * \brief Get status of automatic calibration
     *
     * \param serial buffer to store read serial number
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType get_serial_number(uint16_t serial[3]);

    /*!
     * \brief Perform a self test
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType perform_self_test();

    /*!
     * \brief Perform a factory reset
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType perfom_factory_reset();

    /*!
     * \brief Reinitialize the sensor by realoading settings from the EEPROM
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType reinit();

    /*!
     * \brief Start a single shot measurement of temperature and humidity
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType measure_single_shot_rht_only();

    /*!
     * \brief Start a single shot measurement of CO2 level, temperature and humidity
     *
     * \return Ok on success, the reason of failure otherwise
     */
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

    I2C _bus;

    /*!
     * \brief Compute CRC of a given message
     *
     * \param data data buffer to compute the CRC on
     * \param len length of the data buffer
     *
     * \return computed value of CRC
     */
    char compute_crc(char *data, uint8_t len);

    /*!
     * \brief Send a command to the sensor
     *
     * \param cmd command to send
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType send_command(Command cmd);

    /*!
     * \brief Read data from the sensor
     *
     * \param cmd command to read data from
     * \param len number of 16bit words to read from the sensor
     * \param val_out pointer to store read data
     * \param exec_time time to wait between acknowledgement of the command and the start of the
     *                  read transaction
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType read(
            Command cmd, uint8_t len, uint16_t *val_out, Clock::duration_u32 exec_time = 1ms);

    /*!
     * \brief Write data to the sensor
     *
     * \param cmd command to write data to
     * \param len number of 16bit words to write
     * \param val_in pointer to the data to write
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType write(Command cmd, uint8_t len, uint16_t *val_in);

    /*!
     * \brief Read data from the sensor
     *
     * \param cmd command to send and fetch data
     * \param val_in pointer to the data to write
     * \param val_out pointer to store read data
     * \param exec_time time to wait between acknowledgement of the write command and the start of
     *                  the read transaction
     *
     * \return Ok on success, the reason of failure otherwise
     */
    ErrorType send_and_fetch(
            Command cmd, uint16_t *val_in, uint16_t *val_out, Clock::duration_u32 exec_time = 1ms);
};

} // namespace sixtron

#endif // CATIE_SIXTRON_SCD4X_H_
