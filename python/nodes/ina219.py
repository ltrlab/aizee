"""
INA219 I2C Power Monitor Driver

Driver for Texas Instruments INA219 current/power monitor IC.
Used by Waveshare UPS Power Module (C) for battery monitoring.

Datasheet: https://www.ti.com/lit/ds/symlink/ina219.pdf
"""

import smbus
import time


# Config Register (R/W)
_REG_CONFIG = 0x00
# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE = 0x01
# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE = 0x02
# POWER REGISTER (R)
_REG_POWER = 0x03
# CURRENT REGISTER (R)
_REG_CURRENT = 0x04
# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION = 0x05


class BusVoltageRange:
    """Constants for bus_voltage_range"""
    RANGE_16V = 0x00  # set bus voltage range to 16V
    RANGE_32V = 0x01  # set bus voltage range to 32V (default)


class Gain:
    """Constants for gain"""
    DIV_1_40MV = 0x00   # shunt prog. gain set to  1, 40 mV range
    DIV_2_80MV = 0x01   # shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV = 0x02  # shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV = 0x03  # shunt prog. gain set to /8, 320 mV range


class ADCResolution:
    """ADC resolution and averaging constants"""
    ADCRES_9BIT_1S = 0x00    # 9-bit, 1 sample
    ADCRES_12BIT_32S = 0x0D  # 12-bit, 32 samples (averaged)


class Mode:
    """Operating mode constants"""
    SANDBVOLT_CONTINUOUS = 0x07  # Continuous shunt and bus voltage


class INA219:
    """INA219 I2C current/power monitor driver

    Configured for Waveshare UPS Power Module (C):
    - 16V bus voltage range
    - 0.01 ohm shunt resistor
    - Up to 5A continuous current measurement
    """

    def __init__(self, i2c_bus=7, addr=0x40):
        """Initialize INA219

        Args:
            i2c_bus: I2C bus number (7 for Jetson Orin Nano)
            addr: I2C device address (0x40 or 0x41)
        """
        self.bus = smbus.SMBus(i2c_bus)
        self.addr = addr

        # Set chip to known config values to start
        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0
        self.set_calibration_16V_5A()

    def read(self, address):
        """Read 16-bit register

        Args:
            address: Register address

        Returns:
            16-bit register value
        """
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return (data[0] << 8) + data[1]

    def write(self, address, data):
        """Write 16-bit register

        Args:
            address: Register address
            data: 16-bit value to write
        """
        self.bus.write_i2c_block_data(self.addr, address, [(data >> 8) & 0xFF, data & 0xFF])

    def set_calibration_16V_5A(self):
        """Configure INA219 for 16V/5A measurement range

        Calculations assume 0.01 ohm shunt resistor:
        - VBUS_MAX = 16V
        - VSHUNT_MAX = 0.08V (Gain 2, 80mV)
        - RSHUNT = 0.01 ohm
        - MaxPossible_I = 8.0A
        - MaxExpected_I = 5.0A
        - CurrentLSB = 0.1524 mA per bit
        - PowerLSB = 3.048 mW per bit
        """
        # Current LSB = 0.1524 mA per bit
        self._current_lsb = 0.1524

        # Calibration value
        # Cal = trunc(0.04096 / (Current_LSB * RSHUNT))
        self._cal_value = 26868

        # Power LSB = 20 * CurrentLSB
        self._power_lsb = 0.003048  # 3.048 mW per bit

        # Set Calibration register
        self.write(_REG_CALIBRATION, self._cal_value)

        # Set Config register
        # - 16V bus voltage range
        # - Gain /2 (80mV shunt voltage range)
        # - 12-bit ADC resolution with 32 sample averaging
        # - Continuous shunt and bus voltage measurement
        config = (BusVoltageRange.RANGE_16V << 13) | \
                 (Gain.DIV_2_80MV << 11) | \
                 (ADCResolution.ADCRES_12BIT_32S << 7) | \
                 (ADCResolution.ADCRES_12BIT_32S << 3) | \
                 Mode.SANDBVOLT_CONTINUOUS
        self.write(_REG_CONFIG, config)

    def getShuntVoltage_mV(self):
        """Get shunt voltage in millivolts

        Returns:
            Shunt voltage in mV (can be negative for charging)
        """
        value = self.read(_REG_SHUNTVOLTAGE)
        if value > 32767:
            value -= 65536
        return value * 0.01

    def getBusVoltage_V(self):
        """Get bus voltage in volts

        Returns:
            Bus voltage in V
        """
        value = self.read(_REG_BUSVOLTAGE)
        # Bus voltage is in bits 15-3, LSB = 4mV
        return (value >> 3) * 0.004

    def getCurrent_mA(self):
        """Get current in milliamperes

        Returns:
            Current in mA (positive = discharging, negative = charging)
        """
        value = self.read(_REG_CURRENT)
        if value > 32767:
            value -= 65536
        return value * self._current_lsb

    def getPower_W(self):
        """Get power in watts

        Returns:
            Power in W
        """
        value = self.read(_REG_POWER)
        if value > 32767:
            value -= 65536
        return value * self._power_lsb


if __name__ == '__main__':
    """Test script for INA219"""
    # Use address 0x41 for Waveshare UPS Module (C)
    ina219 = INA219(i2c_bus=7, addr=0x41)

    print("INA219 Test - Waveshare UPS Power Module (C)")
    print("=" * 50)

    while True:
        try:
            bus_voltage = ina219.getBusVoltage_V()
            shunt_voltage = ina219.getShuntVoltage_mV() / 1000
            current = ina219.getCurrent_mA()
            power = ina219.getPower_W()

            # Calculate percentage (9V = 0%, 12.6V = 100%)
            percentage = (bus_voltage - 9.0) / 3.6 * 100
            percentage = max(0, min(percentage, 100))

            print(f"Load Voltage:  {bus_voltage:6.3f} V")
            print(f"Current:       {current/1000:9.6f} A")
            print(f"Power:         {power:9.6f} W")
            print(f"Percentage:    {percentage:6.2f} %")
            print("-" * 50)

            time.sleep(2)

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            break
