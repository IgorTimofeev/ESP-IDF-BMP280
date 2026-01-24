#pragma once

#include <cstdint>
#include <utility>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include <busHAL.h>

namespace YOBA {
	enum class BMP280I2CAddress : uint8_t {
		// SDO pin is low
		primary = 0x76,
		// SDO pin is high
		secondary = 0x77
	};

	enum class BMP280Oversampling : uint8_t {
		none = 0x00,
		x1 = 0x01,
		x2 = 0x02,
		x4 = 0x03,
		x8 = 0x04,
		x16 = 0x05
	};

	enum class BMP280Mode : uint8_t {
		sleep = 0x00,
		forced = 0x01,
		normal = 0x03
	};

	enum class BMP280Filter : uint8_t {
		none = 0x00,
		x2 = 0x01,
		x4 = 0x02,
		x8 = 0x03,
		x16 = 0x04
	};

	enum class BMP280StandbyDuration : uint8_t {
		ms1 = 0x00,
		ms63 = 0x01,
		ms125 = 0x02,
		ms250 = 0x03,
		ms500 = 0x04,
		ms1000 = 0x05,
		ms2000 = 0x06,
		ms4000 = 0x07
	};

	enum class BMP280Register : uint8_t {
		digT1 = 0x88,
		digT2 = 0x8A,
		digT3 = 0x8C,

		digP1 = 0x8E,
		digP2 = 0x90,
		digP3 = 0x92,
		digP4 = 0x94,
		digP5 = 0x96,
		digP6 = 0x98,
		digP7 = 0x9A,
		digP8 = 0x9C,
		digP9 = 0x9E,

		chipID = 0xD0,
		version = 0xD1,
		softReset = 0xE0,
		calibration = 0xE1,
		status = 0xF3,
		control = 0xF4,
		config = 0xF5,
		pressureData = 0xF7,
		temperatureData = 0xFA
	};

	class BMP280 {
		public:
			bool setup(
				busHAL* bus,

				BMP280Mode mode,
				BMP280Oversampling pressureOversampling,
				BMP280Oversampling temperatureOversampling,
				BMP280Filter filter,
				BMP280StandbyDuration standbyDuration
			) {
				_bus = bus;

				// Soft reset
				reset();

				// Checking for valid chip ID & proper SPI wiring
				uint8_t chipID = 0;

				if (!readUint8(BMP280Register::chipID, chipID)) {
					ESP_LOGE(_logTag, "Unable to SPIReadRegister chip ID");
					return false;
				}

				if (chipID != BMP280ChipID) {
					ESP_LOGE(_logTag, "Invalid chip ID: %d", chipID);
					return false;
				}

				// Reading factory-fused calibration data
				if (!readCalibrationData()) {
					ESP_LOGE(_logTag, "Unable to SPIReadRegister calibration data");
					return false;
				}

				// Configuring sensor to initial state
				reconfigure(
					mode,
					pressureOversampling,
					temperatureOversampling,
					filter,
					standbyDuration
				);

				return true;
			}

			void reset() const {
				// Magical constant, only B6 will put to reset, who knows why
				writeUint8(BMP280Register::softReset, 0xB6);

				delayMs(200);
			}

			void reconfigure(
				const BMP280Mode mode,
				const BMP280Oversampling pressureOversampling,
				const BMP280Oversampling temperatureOversampling,
				const BMP280Filter filter,
				const BMP280StandbyDuration standbyDuration
			) const {
				// t_sb = standbyDuration
				// filter = filter
				// spi3w_en = 0
				writeUint8(
					BMP280Register::config,
					static_cast<uint8_t>(
						(std::to_underlying(standbyDuration) << 5)
						| (std::to_underlying(filter) << 2)
						| 0
					)
				);

				// osrs_t = temperatureOversampling
				// osrs_p = pressureOversampling
				// mode = mode
				writeUint8(
					BMP280Register::control,
					static_cast<uint8_t>(
						(std::to_underlying(temperatureOversampling) << 5)
						| (std::to_underlying(pressureOversampling) << 2)
						| std::to_underlying(mode)
					)
				);

				delayMs(50);
			}

			// These bitchy formulas has been taken directly from datasheet: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
			// Don't see any reason to touch them))0
			void readPressureAndTemperature(float& pressure, float& temperature) {
				// Seems like module allows to read both pressure & temp is one continuous operation
				uint8_t buffer[6];
				read(BMP280Register::pressureData, buffer, 6);

				const int32_t adc_P = buffer[0] << 12 | buffer[1] << 4 | buffer[2] >> 4;
				const int32_t adc_T = buffer[3] << 12 | buffer[4] << 4 | buffer[5] >> 4;

//				ESP_LOGI(_logTag, "adc_P: %d, adc_T: %d", adc_P, adc_T);

				// Temperature should be processed first for tFine
				{
					const int32_t var1 = ((((adc_T >> 3) - (static_cast<int32_t>(_calibrationDigT1) << 1))) * static_cast<int32_t>(
						_calibrationDigT2)) >> 11;
					const int32_t var2 = (((((adc_T >> 4) - static_cast<int32_t>(_calibrationDigT1)) * ((adc_T >> 4) - static_cast<int32_t>(
							_calibrationDigT1))) >> 12) *
						static_cast<int32_t>(_calibrationDigT3)) >> 14;
					_tFine = var1 + var2;
					const int32_t T = (_tFine * 5 + 128) >> 8;

					temperature = T / 100.f;
				}

				// Pressure
				{
					int64_t var1 = static_cast<int64_t>(_tFine) - 128000;
					int64_t var2 = var1 * var1 * static_cast<int64_t>(_calibrationDigP6);
					var2 = var2 + ((var1*static_cast<int64_t>(_calibrationDigP5))<<17);
					var2 = var2 + (static_cast<int64_t>(_calibrationDigP4)<<35);
					var1 = ((var1 * var1 * static_cast<int64_t>(_calibrationDigP3))>>8) + ((var1 * static_cast<int64_t>(_calibrationDigP2))<<12);
					var1 = (((static_cast<int64_t>(1)<<47)+var1))*static_cast<int64_t>(_calibrationDigP1)>>33;
					if (var1 == 0)
					{
						pressure = 0;
						return; // avoid exception caused by division by zero
					}
					int64_t p = 1048576 - adc_P;
					p = (((p<<31)-var2)*3125)/var1;
					var1 = (static_cast<int64_t>(_calibrationDigP9) * (p>>13) * (p>>13)) >> 25;
					var2 = (static_cast<int64_t>(_calibrationDigP8) * p) >> 19;
					p = ((p + var1 + var2) >> 8) + (static_cast<int64_t>(_calibrationDigP7)<<4);

					pressure = static_cast<uint32_t>(p) / 256.0f;
				}
			}

		private:
			constexpr static auto _logTag = "BMP280";
			constexpr static uint8_t BMP280ChipID = 0x58;
			

			// From datasheet:
			// The variable t_fine (signed 32 bit) carries a fine resolution temperature value over to the
			// pressure compensation formula and could be implemented as a global variable
			int32_t _tFine = -0xFFFF;

			// Bus
			busHAL* _bus = nullptr;

			// Calibration data
			uint16_t _calibrationDigT1 = 0;
			int16_t _calibrationDigT2 = 0;
			int16_t _calibrationDigT3 = 0;

			uint16_t _calibrationDigP1 = 0;
			int16_t _calibrationDigP2 = 0;
			int16_t _calibrationDigP3 = 0;
			int16_t _calibrationDigP4 = 0;
			int16_t _calibrationDigP5 = 0;
			int16_t _calibrationDigP6 = 0;
			int16_t _calibrationDigP7 = 0;
			int16_t _calibrationDigP8 = 0;
			int16_t _calibrationDigP9 = 0;

			// Writing
			bool writeUint8(const BMP280Register reg, const uint8_t value) const {
				return _bus->writeUint8(std::to_underlying(reg), value);
			}

			// Reading
			static uint8_t getRegisterValueForReading(const BMP280Register reg) {
				return static_cast<uint8_t>(std::to_underlying(reg) | 0x80);
			}

			bool read(const BMP280Register reg, uint8_t* buffer, const size_t length) const {
				return _bus->read(getRegisterValueForReading(reg), buffer, length);
			}

			bool readUint8(const BMP280Register reg, uint8_t& value) const {
				return _bus->readUint8(getRegisterValueForReading(reg), value);
			}

			bool readUint16LE(const BMP280Register reg, uint16_t& value) const {
				return _bus->readUint16LE(getRegisterValueForReading(reg), value);
			}

			bool readInt16LE(const BMP280Register reg, int16_t& value) const {
				return _bus->readInt16LE(getRegisterValueForReading(reg), value);
			}

			bool readCalibrationData() {
				// Temperature
				if (!readUint16LE(BMP280Register::digT1, _calibrationDigT1))
					return false;

				if (!readInt16LE(BMP280Register::digT2, _calibrationDigT2))
					return false;

				if (!readInt16LE(BMP280Register::digT3, _calibrationDigT3))
					return false;

				// Pressure
				if (!readUint16LE(BMP280Register::digP1, _calibrationDigP1))
					return false;

				if (!readInt16LE(BMP280Register::digP2, _calibrationDigP2))
					return false;

				if (!readInt16LE(BMP280Register::digP3, _calibrationDigP3))
					return false;

				if (!readInt16LE(BMP280Register::digP4, _calibrationDigP4))
					return false;

				if (!readInt16LE(BMP280Register::digP5, _calibrationDigP5))
					return false;

				if (!readInt16LE(BMP280Register::digP6, _calibrationDigP6))
					return false;

				if (!readInt16LE(BMP280Register::digP7, _calibrationDigP7))
					return false;

				if (!readInt16LE(BMP280Register::digP8, _calibrationDigP8))
					return false;

				if (!readInt16LE(BMP280Register::digP9, _calibrationDigP9))
					return false;

//				ESP_LOGI(_logTag, "_calibrationDigT1: %d", _calibrationDigT1);
//				ESP_LOGI(_logTag, "_calibrationDigT2: %d", _calibrationDigT2);
//				ESP_LOGI(_logTag, "_calibrationDigT3: %d", _calibrationDigT3);
//				ESP_LOGI(_logTag, "_calibrationDigP1: %d", _calibrationDigP1);
//				ESP_LOGI(_logTag, "_calibrationDigP2: %d", _calibrationDigP2);
//				ESP_LOGI(_logTag, "_calibrationDigP3: %d", _calibrationDigP3);
//				ESP_LOGI(_logTag, "_calibrationDigP4: %d", _calibrationDigP4);
//				ESP_LOGI(_logTag, "_calibrationDigP5: %d", _calibrationDigP5);
//				ESP_LOGI(_logTag, "_calibrationDigP6: %d", _calibrationDigP6);
//				ESP_LOGI(_logTag, "_calibrationDigP7: %d", _calibrationDigP7);
//				ESP_LOGI(_logTag, "_calibrationDigP8: %d", _calibrationDigP8);
//				ESP_LOGI(_logTag, "_calibrationDigP9: %d", _calibrationDigP9);

				return true;
			}

			static void delayMs(const uint32_t ms) {
				vTaskDelay(ms <= portTICK_PERIOD_MS ? portTICK_PERIOD_MS : pdMS_TO_TICKS(ms));
			}
	};
}