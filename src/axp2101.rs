//! AXP2101 power management chip interface
//!
//! [`Axp2101::new`] is the starting-point interface for this crate.
//!
//! Some devices which include this IC:
//!
//!  * [M5Stack Core 2](https://docs.m5stack.com/en/core/core2) (including the [Core 2 for
//!    AWS](https://docs.m5stack.com/en/core/core2_for_aws) variant)
//!  * [M5Stack Tough](https://docs.m5stack.com/en/core/tough)
//!  * [M5StickC](https://docs.m5stack.com/en/core/m5stickc)
//!  * [M5StickC PLUS](https://docs.m5stack.com/en/core/m5stickc_plus)
//!
//! ***Warning!*** This chip probably controls power to the microcontroller you are running the
//! code on, and bricking the entire device is a possibility!
//!
//! This implementation does not yet completely cover the functionality of the IC. If there's a
//! feature you'd like to see implemented, either [open an
//! issue](https://github.com/rcloran/axp2101-rs/issues) or create a pull request :)
//!
//! Datasheet:
//! [https://github.com/m5stack/M5-Schematic/blob/master/Core/AXP2101%20Datasheet_v1.1_en_draft_2211.pdf
//! ](https://github.com/m5stack/M5-Schematic/blob/master/Core/AXP2101%20Datasheet_v1.1_en_draft_2211.pdf)

use embedded_hal::i2c;

const AXP2101_ADDRESS: u8 = 0x34;

/// The Axp2101 struct is the main interface for this crate
pub struct Axp2101<I2C> {
    i2c: I2C,
}

/// GPIO1 and GPIO2 function setting
///
/// Used with [`Axp2101::set_gpio1_mode`] and [`Axp2101::set_gpio2_mode`]
pub enum GpioMode12 {
    /// 000: NMOS Open drain output
    NmosOpenDrainOutput = 0b000,
    /// 001: Universal input function
    UniversalInput = 0b001,
    /// 010:PWM1 Output, high level is VINT ,Do not Can be less than 100K Pull-down resistor
    PWMOutput = 0b010,
    /// 011: Keep
    Keep = 0b011,
    /// 100: ADC enter
    AdcEnter = 0b100,
    /// 101: Low output
    LowOutput = 0b101,
    /// 11X: Floating
    Floating = 0b110,
}

/// GPIO3 and GPIO4 function setting
///
/// Used with [`Axp2101::set_gpio3_mode`] and [`Axp2101::set_gpio4_mode`]
pub enum GpioMode34 {
    /// 00: External charging control
    ExternalChargeControl = 0b00,
    /// 01: NMOS Open drain output
    NmosOpenDrainOutput = 0b01,
    /// 10: Universal input
    UniversalInput = 0b10,
    /// GPIO3: 11: ADC enter
    /// GPIO4: 11: Undefined
    AdcEnter = 0b11,
}

/// Boot time setting
///
/// Used with [`Axp2101::set_key_mode`]
pub enum BootTime {
    Boot128ms = 0b00,
    Boot512ms = 0b01,
    Boot1s = 0b10,
    Boot2s = 0b11,
}

/// Long press time setting
///
/// Used with [`Axp2101::set_key_mode`]
pub enum LongPress {
    Lp1000ms = 0b00,
    Lp1500ms = 0b01,
    Lp2000ms = 0b10,
    Lp2500ms = 0b11,
}

/// After the power is started PWROK Signal delay
///
/// Used with [`Axp2101::set_key_mode`]
pub enum PowerOkDelay {
    Delay32ms = 0b0,
    Delay64ms = 0b1,
}

/// Shutdown duration setting
///
/// Used with [`Axp2101::set_key_mode`]
pub enum ShutdownDuration {
    Sd4s = 0b00,
    Sd6s = 0b01,
    Sd8s = 0b10,
    Sd10s = 0b11,
}

impl<I2C, E> Axp2101<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Construct a new [`Axp2101`]
    ///
    /// `i2c` must be an object which implements the I2C trait from embedded-hal
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    // Utility internal functions
    fn get(&mut self, reg_addr: u8, buff: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(AXP2101_ADDRESS, &[reg_addr], buff)
    }

    fn get_8(&mut self, reg_addr: u8) -> Result<u8, E> {
        let mut buff = [0u8];
        self.get(reg_addr, &mut buff)?;
        Ok(buff[0])
    }

    fn get_flag(&mut self, reg_addr: u8, flag: u8) -> Result<bool, E> {
        Ok(self.get_8(reg_addr)? & flag != 0)
    }

    fn set_8(&mut self, addr: u8, v: u8) -> Result<(), E> {
        self.i2c.write(AXP2101_ADDRESS, &[addr, v])
    }

    fn set_flag(&mut self, addr: u8, bit: u8, state: bool) -> Result<(), E> {
        let mut v = self.get_8(addr)?;
        if state {
            v |= bit;
        } else {
            v &= !bit;
        }

        self.set_8(addr, v)
    }

    fn voltage_to_dcdc1_value(value: u16) -> u8 {
        let mut voltage = value;
        if voltage < 1500 {
            voltage = 1500;
        }

        // AXP2101 DCDC1 max voltage value is 3400mV
        if voltage > 3400 {
            voltage = 3400;
        }

        ((voltage - 1500) / 100) as u8
    }

    fn dcdc1_value_to_voltage(value: u8) -> u16 {
        (value as u16) * 100 + 1500
    }

    fn voltage_to_dcdc2_value(value: u16) -> u8 {
        let mut voltage = value;
        let data: u8;

        // AXP2101 DCDC2 min voltage value is 500mV
        if voltage < 500 {
            voltage = 500;
        } else if voltage > 1200 && voltage < 1220 {
            voltage = 1200;
        } else if voltage > 1540 {
            // AXP2101 DCDC2 max voltage value is 1540mV
            voltage = 3400;
        }

        if voltage <= 1200 {
            data = ((voltage - 500) / 10 + 0b00000000) as u8;
        } else {
            data = ((voltage - 1220) / 20 + 0b01000111) as u8;
        }

        data
    }

    fn dcdc2_value_to_voltage(value: u8) -> u16 {
        let result: u16;
        let val: u16 = value as u16;

        if val <= 71 {
            result = val * 10 + 500;
        } else {
            result = (val - 71) * 20 + 1220;
        }

        result as u16
    }

    fn voltage_to_dcdc3_value(value: u16) -> u8 {
        let mut voltage = value;
        let data: u8;

        // AXP2101 DCDC3 min voltage value is 500mV
        if voltage < 500 {
            voltage = 500;
        } else if voltage > 1200 && voltage < 1220 {
            voltage = 1200;
        } else if voltage > 1540 && voltage < 1600 {
            voltage = 1540;
        } else if voltage > 3400 {
            // AXP2101 DCDC3 max voltage value is 3400mV
            voltage = 3400;
        }

        if voltage <= 1200 {
            data = ((voltage - 500) / 10 + 0b00000000) as u8;
        } else if voltage <= 1540 {
            data = ((voltage - 1220) / 20 + 0b01000111) as u8;
        } else {
            data = ((voltage - 1600) / 100 + 0b01011000) as u8;
        }

        data
    }

    fn dcdc3_value_to_voltage(value: u8) -> u16 {
        let result: u16;
        let val: u16 = value as u16;

        if val <= 71 {
            result = val * 10 + 500;
        } else if val > 71 && val < 88 {
            result = (val - 71) * 20 + 1220;
        } else {
            result = (val - 88) * 100 + 1600;
        }

        result as u16
    }

    fn voltage_to_dcdc5_value(value: u16) -> u8 {
        let mut voltage = value;

        // AXP2101 DCDC5 min voltage value is 1400mV
        if voltage < 1400 {
            voltage = 1400;
        } else if voltage > 3700 {
            // AXP2101 DCDC5 max voltage value is 3700mV
            voltage = 3700;
        }

        ((voltage - 1400) / 100 + 0b00000000) as u8
    }

    fn dcdc5_value_to_voltage(value: u8) -> u16 {
        (value as u16) * 100 + 1400 as u16
    }

    fn voltage_to_ldo_value(value: u16) -> u8 {
        let mut voltage = value;
        if voltage < 500 {
            voltage = 500;
        }

        if voltage > 3400 {
            voltage = 3400;
        }

        ((voltage - 500) / 100) as u8
    }

    fn ldo_value_to_voltage(value: u8) -> u16 {
        (value as u16) * 100 + 500
    }

    // Public functions in register order

    /// Current limit state
    pub fn get_current_limit_status(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0000_001)
    }

    /// Thermal regulation status
    pub fn get_temp_regu_status(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0000_0010)
    }

    /// Battery in active mode
    pub fn get_batt_active(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0000_0100)
    }

    /// Battery present state
    pub fn get_batt_present(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0000_1000)
    }

    /// Instructions ACIN it's usable or not
    pub fn get_batfet_state(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0001_0000)
    }

    /// VBUS good indication
    /// 0: not good
    /// 1: good
    pub fn get_vbus_good(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0010_0000)
    }

    /// Sysled Switch control
    pub fn set_sysled_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x69, 0b00110000, state)
    }

    /// DC-DC1 Switch status
    pub fn get_dcdc1_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x80, 0b0000_0001)
    }

    /// PMU comm config
    /// Default 0x30
    /// 7:6 : reserved
    /// 5   : Interal off-discharge enabled for dcdc & LDO & Switch (Default 1)
    /// 4   : reserved
    /// 3   : PWROK pin pull low to restart system (default 0)
    /// 2   : PWRON 16s to shut the PMIC enable (default 0)
    /// 1   : Restart the SoC system POWOFF/ POWON and reset the related registers, 0 = normal, 1 = reset (default 0)
    /// 0   : Soft PWROFF, 0 = nomal, 1 = PWROFF config (default 0)
    pub fn set_pmu_conf(&mut self, config: u8) -> Result<(), E> {
        self.set_8(0x10, config)
    }

    /// DC-DC1 Switch control
    ///
    /// ***Warning!*** This output is often connected to the microcontroller that you are
    /// controlling the AXP2101 from!
    pub fn set_dcdc1_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x80, 0b0000_0001, state)
    }

    /// DC-DC2 Switch status
    pub fn get_dcdc2_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x80, 0b0000_0010)
    }

    /// DC-DC2 Switch control
    pub fn set_dcdc2_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x80, 0b0000_0010, state)
    }

    /// DC-DC3 Switch status
    pub fn get_dcdc3_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x80, 0b0000_0100)
    }

    /// DC-DC3 Switch control
    pub fn set_dcdc3_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x80, 0b0000_0100, state)
    }

    /// DC-DC4 Switch status
    pub fn get_dcdc4_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x80, 0b0000_1000)
    }

    /// DC-DC4 Switch control
    pub fn set_dcdc4_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x80, 0b0000_1000, state)
    }

    /// DC-DC5 Switch status
    pub fn get_dcdc5_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x80, 0b0001_0000)
    }

    /// DC-DC5 Switch control
    pub fn set_dcdc5_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x80, 0b0001_0000, state)
    }

    /// ALDO1 control
    pub fn set_aldo1_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b0000_0001, state)
    }

    /// ALDO1 status
    pub fn get_aldo1_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b0000_0001)
    }

    /// ALDO2 control
    pub fn set_aldo2_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b0000_0010, state)
    }

    /// ALDO2 status
    pub fn get_aldo2_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b0000_0010)
    }

    /// ALDO3 control
    pub fn set_aldo3_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b0000_0100, state)
    }

    /// ALDO3 status
    pub fn get_aldo3_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b0000_0100)
    }

    /// ALDO4 control
    pub fn set_aldo4_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b0000_1000, state)
    }

    /// ALDO4 status
    pub fn get_aldo4_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b0000_1000)
    }

    /// BLDO1 control
    pub fn set_bldo1_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b0001_0000, state)
    }

    /// BLDO1 Switch status
    pub fn get_bldo1_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b0001_0000)
    }

    /// BLDO2 control
    pub fn set_bldo2_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b0010_0000, state)
    }

    /// BLDO2 status
    pub fn get_bldo2_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b0010_0000)
    }

    /// BLDO2 control
    pub fn set_cpusldo_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b0100_0000, state)
    }

    /// BLDO2 status
    pub fn get_cpusldo_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b0100_0000)
    }

    /// DLDO1 control
    pub fn set_dldo1_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x90, 0b1000_0000, state)
    }

    /// DLDO1 status
    pub fn get_dldo1_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x90, 0b1000_0000)
    }

    /// DLDO2 control
    pub fn set_dldo2_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x91, 0b0000_0001, state)
    }

    /// DLDO2 status
    pub fn get_dldo2_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x91, 0b0000_0001)
    }

    ///
    /// Voltage getters/ setters
    ///

    /// DC-DC1 output voltage setting
    ///
    /// 1.5-3.4V, 100mV/step, 20 steps
    /// 00000: 1.5V
    /// 00001: 1.6V
    /// .....
    /// 10011: 3.4V
    ///
    /// ***Warning!*** This output is often connected to the microcontroller that you are
    /// controlling the AXP192 from!
    pub fn get_dcdc1_voltage(&mut self) -> Result<u16, E> {
        let voltage = self.get_8(0x82)?;
        Ok(Self::dcdc1_value_to_voltage(voltage))
    }

    /// DC-DC1 output voltage setting
    ///
    /// 1.5-3.4V, 100mV/step, 20 steps
    /// 00000: 1.5V
    /// 00001: 1.6V
    /// .....
    /// 10011: 3.4V
    ///
    /// ***Warning!*** This output is often connected to the microcontroller that you are
    /// controlling the AXP2101 from!
    pub fn set_dcdc1_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_dcdc1_value(voltage);
        self.set_8(0x82, data)
    }

    /// DC-DC2 output voltage setting
    ///
    /// 0.5-1.2V, 10mV/step, 71 steps
    /// 1.22-1.54V, 20mV/step, 17 steps
    pub fn get_dcdc2_voltage(&mut self) -> Result<u16, E> {
        let voltage = self.get_8(0x83)?;
        Ok(Self::dcdc2_value_to_voltage(voltage))
    }

    /// DC-DC2 output voltage setting
    ///
    /// 0.5-1.2V, 10mV/step, 71 steps
    /// 1.22-1.54V, 20mV/step, 17 steps
    pub fn set_dcdc2_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_dcdc2_value(voltage);
        self.set_8(0x83, data)
    }

    /// DC-DC3 output voltage setting
    ///
    /// 0.5-1.2V, 10mV/step, 71 steps
    /// 1.22-1.54V, 20mV/step, 17 steps
    /// 1.6-3.4V, 100mV/step, 19 steps
    /// 0000000: 0.50v
    /// 0000001: 0.51v
    /// .......
    /// 1000110: 1.20v
    /// 1000111: 1.22v
    /// 1001000: 1.24v
    /// 1011000: 1.60v
    /// 1011001: 1.70v
    /// .......
    /// 1101011: 3.40v
    pub fn get_dcdc3_voltage(&mut self) -> Result<u16, E> {
        let voltage = self.get_8(0x84)?;
        Ok(Self::dcdc3_value_to_voltage(voltage))
    }

    /// DC-DC3 output voltage setting
    ///
    /// 0.5-1.2V, 10mV/step, 71 steps
    /// 1.22-1.54V, 20mV/step, 17 steps
    /// 1.6-3.4V, 100mV/step, 19 steps
    /// 0000000: 0.50v
    /// 0000001: 0.51v
    /// .......
    /// 1000110: 1.20v
    /// 1000111: 1.22v
    /// 1001000: 1.24v
    /// .......
    /// 1010111: 1.54v
    /// 1011000: 1.60v
    /// 1011001: 1.70v
    /// .......
    /// 1101011: 3.40v
    pub fn set_dcdc3_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_dcdc3_value(voltage);
        self.set_8(0x84, data)
    }

    /// DC-DC4 output voltage setting
    ///
    /// 0.5-1.2V, 10mV/step, 71 steps
    /// 1.22-1.84V, 20mV/step, 32 steps
    /// 0000000: 0.50v
    /// 0000001: 0.51v
    /// .......
    /// 1000110: 1.20v
    /// 1000111: 1.22v
    /// 1001000: 1.24v
    /// .......
    /// 1100110: 1.84v
    pub fn get_dcdc4_voltage(&mut self) -> Result<u8, E> {
        unimplemented!("No use for now.");
        // self.get_8(0x85)
    }

    /// DC-DC4 output voltage setting
    ///
    /// 0.5-1.2V, 10mV/step, 71 steps
    /// 1.22-1.84V, 20mV/step, 32 steps
    /// 0000000: 0.50v
    /// 0000001: 0.51v
    /// .......
    /// 1000110: 1.20v
    /// 1000111: 1.22v
    /// 1001000: 1.24v
    /// .......
    /// 1100110: 1.84v
    pub fn set_dcdc4_voltage(&mut self, _voltage: u16) -> Result<(), E> {
        unimplemented!("No use for now.");
        // self.set_8(0x85, v)
    }

    /// DC-DC5 output voltage setting
    ///
    /// 1.4-3.7V, 100mV/step, 24 steps
    /// 00000: 1.4v
    /// 00001: 1.5v
    /// .......
    /// 10111: 3.7v
    pub fn get_dcdc5_voltage(&mut self) -> Result<u16, E> {
        let voltage = self.get_8(0x86)?;
        Ok(Self::dcdc5_value_to_voltage(voltage))
    }

    /// DC-DC5 output voltage setting
    ///
    /// 1.4-3.7V, 100mV/step, 24 steps
    /// 00000: 1.4v
    /// 00001: 1.5v
    /// .......
    /// 10111: 3.7v
    pub fn set_dcdc5_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_dcdc5_value(voltage);
        self.set_8(0x86, data)
    }

    /// ALDO1 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_aldo1_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x92, data)
    }

    /// ALDO2 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_aldo2_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x93, data)
    }

    /// ALDO3 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_aldo3_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x94, data)
    }

    /// ALDO4 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_aldo4_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x95, data)
    }

    /// DC-DC5 output voltage setting
    ///
    /// 1.4-3.7V, 100mV/step, 24 steps
    /// 00000: 1.4v
    /// 00001: 1.5v
    /// .......
    /// 10111: 3.7v
    pub fn get_bldo1_voltage(&mut self) -> Result<u16, E> {
        let voltage = self.get_8(0x96)?;
        Ok(Self::ldo_value_to_voltage(voltage))
    }

    /// BLDO1 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31 steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_bldo1_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x96, data)
    }

    /// BLDO2 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31 steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_bldo2_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x97, data)
    }

    /// CPUSLDO Output voltage setting
    ///
    /// 0.5-1.4V , 50mV/step , 20 steps
    /// 00000: 0.50v
    /// 00001: 0.55v
    /// .....
    /// 10011: 1.40v
    pub fn set_cpusldo_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x98, data)
    }

    /// DLDO1 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_dldo1_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x99, data)
    }

    /// DLDO2 Output voltage setting
    ///
    /// 0.5-3.5V , 100mV/step , 31steps
    /// 00000: 0.5v
    /// 00001: 0.6v
    /// .....
    /// 11110: 3.5v
    pub fn set_dldo2_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let data = Self::voltage_to_ldo_value(voltage);
        self.set_8(0x9a, data)
    }

    // 0xa4 - battery %
}
