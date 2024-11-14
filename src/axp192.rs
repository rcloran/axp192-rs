//! AXP192 power management chip interface
//!
//! [`Axp192::new`] is the starting-point interface for this crate.
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
//! issue](https://github.com/rcloran/axp192-rs/issues) or create a pull request :)
//!
//! Datasheet:
//! [https://github.com/m5stack/M5-Schematic/blob/master/Core/AXP192%20Datasheet_v1.1_en_draft_2211.pdf
//! ](https://github.com/m5stack/M5-Schematic/blob/master/Core/AXP192%20Datasheet_v1.1_en_draft_2211.pdf)

use embedded_hal::i2c;

const AXP192_ADDRESS: u8 = 0x34;

/// The Axp192 struct is the main interface for this crate
pub struct Axp192<I2C> {
    i2c: I2C,
}

/// GPIO0 function setting
///
/// Used with [`Axp192::set_gpio0_mode`]
pub enum GpioMode0 {
    /// 000: NMOS Open drain output
    NmosOpenDrainOutput = 0b000,
    /// 001: Universal input function
    UniversalInput = 0b001,
    /// 010:PWM1 Output, high level is VINT ,Do not Can be less than 100K Pull-down resistor
    LowNoiseLdo = 0b010,
    /// 011: Keep
    Keep = 0b011,
    /// 100: ADC enter
    AdcEnter = 0b100,
    /// 101: Low output
    LowOutput = 0b101,
    /// 11X: Floating
    Floating = 0b110,
}

/// GPIO1 and GPIO2 function setting
///
/// Used with [`Axp192::set_gpio1_mode`] and [`Axp192::set_gpio2_mode`]
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
/// Used with [`Axp192::set_gpio3_mode`] and [`Axp192::set_gpio4_mode`]
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
/// Used with [`Axp192::set_key_mode`]
pub enum BootTime {
    Boot128ms = 0b00,
    Boot512ms = 0b01,
    Boot1s = 0b10,
    Boot2s = 0b11,
}

/// Long press time setting
///
/// Used with [`Axp192::set_key_mode`]
pub enum LongPress {
    Lp1000ms = 0b00,
    Lp1500ms = 0b01,
    Lp2000ms = 0b10,
    Lp2500ms = 0b11,
}

/// After the power is started PWROK Signal delay
///
/// Used with [`Axp192::set_key_mode`]
pub enum PowerOkDelay {
    Delay32ms = 0b0,
    Delay64ms = 0b1,
}

/// Shutdown duration setting
///
/// Used with [`Axp192::set_key_mode`]
pub enum ShutdownDuration {
    Sd4s = 0b00,
    Sd6s = 0b01,
    Sd8s = 0b10,
    Sd10s = 0b11,
}

impl<I2C, E> Axp192<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    /// Construct a new [`Axp192`]
    ///
    /// `i2c` must be an object which implements the I2C trait from embedded-hal
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    // Utility internal functions
    fn get(&mut self, reg_addr: u8, buff: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(AXP192_ADDRESS, &[reg_addr], buff)
    }

    fn get_8(&mut self, reg_addr: u8) -> Result<u8, E> {
        let mut buff = [0u8];
        self.get(reg_addr, &mut buff)?;
        Ok(buff[0])
    }

    fn get_12(&mut self, reg_addr: u8) -> Result<u16, E> {
        let mut buff = [0; 2];
        self.get(reg_addr, &mut buff)?;
        Ok(((buff[0] as u16) << 4) + (buff[1] as u16))
    }

    fn get_13(&mut self, reg_addr: u8) -> Result<u16, E> {
        let mut buff = [0; 2];
        self.get(reg_addr, &mut buff)?;
        Ok(((buff[0] as u16) << 5) + (buff[1] as u16))
    }

    fn get_flag(&mut self, reg_addr: u8, flag: u8) -> Result<bool, E> {
        Ok(self.get_8(reg_addr)? & flag != 0)
    }

    fn set_8(&mut self, addr: u8, v: u8) -> Result<(), E> {
        self.i2c.write(AXP192_ADDRESS, &[addr, v])
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

    fn voltage_value(value: u16, min: u16, max: u16, step: u16) -> u8 {
        let value = value.clamp(min, max);
        ((value - min) / step) as u8
    }

    // Public functions in register order

    /// Get input power status register (00)
    pub fn get_power_status(&mut self) -> Result<u8, E> {
        self.get_8(0x00)
    }

    /// Indicates the direction of battery current
    pub fn get_charging(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0000_0100)
    }

    /// Instructions VBUS it's usable or not
    pub fn get_vbus_usable(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0001_0000)
    }

    /// VBUS Presence indication
    pub fn get_vbus_present(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0010_0000)
    }

    /// Instructions ACIN it's usable or not
    pub fn get_acin_usable(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b0100_0000)
    }

    /// ACIN Presence indication
    pub fn get_acin_present(&mut self) -> Result<bool, E> {
        self.get_flag(0x00, 0b1000_0000)
    }

    /// EXTEN Switch status
    pub fn get_exten_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x10, 0b0000_0100)
    }

    /// EXTEN Switch control
    pub fn set_exten_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x10, 0b0000_0100, state)
    }

    /// DC-DC1 Switch status
    pub fn get_dcdc1_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x12, 0b0000_0001)
    }

    /// DC-DC1 Switch control
    ///
    /// ***Warning!*** This output is often connected to the microcontroller that you are
    /// controlling the AXP192 from!
    pub fn set_dcdc1_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x12, 0b0000_0001, state)
    }

    /// DC-DC3 Switch status
    pub fn get_dcdc3_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x12, 0b0000_0010)
    }

    /// DC-DC3 Switch control
    pub fn set_dcdc3_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x12, 0b0000_0010, state)
    }

    /// LDO2 Switch status
    pub fn get_ldo2_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x12, 0b0000_0100)
    }

    /// LDO2 Switch control
    pub fn set_ldo2_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x12, 0b0000_0100, state)
    }

    /// LDO3 Switch status
    pub fn get_ldo3_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x12, 0b0000_1000)
    }

    /// LDO3 Switch control
    pub fn set_ldo3_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x12, 0b0000_1000, state)
    }

    /// DC-DC2 Switch status
    pub fn get_dcdc2_on(&mut self) -> Result<bool, E> {
        self.get_flag(0x12, 0b0001_0000)
    }

    /// DC-DC2 Switch control
    pub fn set_dcdc2_on(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x12, 0b0001_0000, state)
    }

    /// DC-DC1 output voltage setting
    ///
    /// 0.7-3.5V , 25mV/step
    ///
    /// ***Warning!*** This output is often connected to the microcontroller that you are
    /// controlling the AXP192 from!
    pub fn set_dcdc1_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let v = Self::voltage_value(voltage, 700, 3500, 25) & 0x7f;
        self.set_8(0x26, v)
    }

    /// DC-DC3 output voltage setting
    ///
    /// 0.7-3.5V , 25mV/step
    pub fn set_dcdc3_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let v = Self::voltage_value(voltage, 700, 3500, 25) & 0x7f;
        self.set_8(0x27, v)
    }

    /// LDO3 Output voltage setting
    ///
    /// 1.8-3.3V , 100mV/step
    pub fn set_ldo3_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let v = Self::voltage_value(voltage, 1800, 3300, 100) & 0x0f;
        let existing = self.get_8(0x28)?;
        self.set_8(0x28, (existing & 0xf0) | v)
    }

    /// LDO2 Output voltage setting
    ///
    /// 1.8-3.3V , 100mV/step
    pub fn set_ldo2_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let v = Self::voltage_value(voltage, 1800, 3300, 100) & 0x0f;
        let existing = self.get_8(0x28)?;
        self.set_8(0x28, (existing & 0x0f) | (v << 4))
    }

    /// VBUS-IPSOUT channel management
    /// VBUS When available VBUS-IPSOUT Path selection control signal
    /// false: by N_VBUSEN pin Decide whether to open this channel
    /// true: :VBUS-IPSOUT Access can be selected to open, regardless of N_VBUSEN status
    pub fn set_ipsout_always(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x30, 0b1000_0000, state)
    }

    /// PEK key parameter setting
    pub fn set_key_mode(
        &mut self,
        shutdown_duration: ShutdownDuration,
        powerok_delay: PowerOkDelay,
        automatic_shutdown: bool,
        longpress_duration: LongPress,
        boot_time: BootTime,
    ) -> Result<(), E> {
        let v = (shutdown_duration as u8)
            | ((powerok_delay as u8) << 2)
            | ((automatic_shutdown as u8) << 3)
            | ((longpress_duration as u8) << 4)
            | ((boot_time as u8) << 6);
        self.set_8(0x36, v)
    }

    /// ACIN Voltage ADC
    ///
    /// Return unit: volts
    pub fn get_acin_voltage(&mut self) -> Result<f32, E> {
        let v = self.get_12(0x56)?;
        Ok(v as f32 * 0.0017)
    }

    /// ACIN Current ADC
    ///
    /// Return unit: amps
    pub fn get_acin_current(&mut self) -> Result<f32, E> {
        let v = self.get_12(0x58)?;
        Ok(v as f32 * 0.000625)
    }

    /// VBUS Voltage ADC
    ///
    /// Return unit: volts
    pub fn get_vbus_voltage(&mut self) -> Result<f32, E> {
        let v = self.get_12(0x5a)?;
        Ok(v as f32 * 0.0017)
    }

    /// VBUS Current ADC
    ///
    /// Return unit: amps
    pub fn get_vbus_current(&mut self) -> Result<f32, E> {
        let v = self.get_12(0x5c)?;
        Ok(v as f32 * 0.000375)
    }

    /// AXP192 Internal temperature monitoring
    ///
    /// Return unit: °C
    pub fn get_internal_temperature(&mut self) -> Result<f32, E> {
        let v = self.get_12(0x5e)?;
        Ok((v as f32 * 0.1) - 144.7)
    }

    /// Battery voltage
    ///
    /// Return unit: volts
    pub fn get_battery_voltage(&mut self) -> Result<f32, E> {
        let v = self.get_12(0x78)?;
        Ok(v as f32 * 0.0011)
    }

    /// battery charging current
    ///
    /// Return unit: amps
    pub fn get_battery_charge_current(&mut self) -> Result<f32, E> {
        let v = self.get_13(0x7a)?;
        Ok(v as f32 * 0.0005)
    }

    /// battery discharge current
    ///
    /// Return unit: amps
    pub fn get_battery_discharge_current(&mut self) -> Result<f32, E> {
        let v = self.get_13(0x7c)?;
        Ok(v as f32 * 0.0005)
    }

    /// TS Pin ADC Function enable
    pub fn set_ts_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b0000_0001, state)
    }

    /// APS Voltage ADC Enable
    pub fn set_aps_voltage_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b0000_0010, state)
    }

    /// VBUS Current ADC Enable
    pub fn set_vbus_current_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b0000_0100, state)
    }

    /// VBUS Voltage ADC Enable
    pub fn set_vbus_voltage_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b0000_1000, state)
    }

    /// ACIN Current ADC Enable
    pub fn set_acin_current_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b0001_0000, state)
    }

    /// ACIN Voltage ADC Enable
    pub fn set_acin_voltage_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b0010_0000, state)
    }

    /// Battery current ADC Enable
    pub fn set_battery_current_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b0100_0000, state)
    }

    /// battery voltage ADC Enable
    pub fn set_battery_voltage_adc_enable(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x82, 0b1000_0000, state)
    }

    /// GPIO0 Pin function setting
    pub fn set_gpio0_mode(&mut self, mode: GpioMode0) -> Result<(), E> {
        self.set_8(0x90, mode as u8)
    }

    /// Output voltage setting when GPIO0 is in LDO mode
    ///
    /// The input is clamped to between 1800mV and 3300mV, in steps of 100mV
    pub fn set_gpio0_ldo_voltage(&mut self, voltage: u16) -> Result<(), E> {
        let v = Self::voltage_value(voltage, 1800, 3300, 100) << 4;
        self.set_8(0x91, v)
    }

    /// GPIO1 function setting
    pub fn set_gpio1_mode(&mut self, mode: GpioMode12) -> Result<(), E> {
        self.set_8(0x92, mode as u8)
    }

    /// GPIO2 function setting
    pub fn set_gpio2_mode(&mut self, mode: GpioMode12) -> Result<(), E> {
        self.set_8(0x93, mode as u8)
    }

    /// GPIO0 Output settings
    ///
    ///  - false: Output low level, ground NMOS turn on
    ///  - true: Output floating, grounded NMOS shut down
    pub fn set_gpio0_output(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x94, 0b0000_0001, state)
    }

    /// GPIO1 Output settings
    ///
    ///  - false: Output low level, ground NMOS turn on
    ///  - true: Output floating, grounded NMOS shut down
    pub fn set_gpio1_output(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x94, 0b0000_0010, state)
    }

    /// GPIO2 Output settings
    ///
    ///  - false: Output low level, ground NMOS turn on
    ///  - true: Output floating, grounded NMOS shut down
    pub fn set_gpio2_output(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x94, 0b0000_0100, state)
    }

    /// GPIO3 Pin function setting
    pub fn set_gpio3_mode(&mut self, mode: GpioMode34) -> Result<(), E> {
        let existing = self.get_8(0x95)?;
        self.set_8(0x95, (existing & 0b1111_1100) | 0x80 | (mode as u8))
    }

    /// GPIO4 Pin function setting
    pub fn set_gpio4_mode(&mut self, mode: GpioMode34) -> Result<(), E> {
        let existing = self.get_8(0x95)?;
        self.set_8(0x95, (existing & 0b1111_0011) | 0x80 | ((mode as u8) << 2))
    }

    /// GPIO3 Output settings
    ///
    ///  - false: Output low level, NMOS turn on
    ///  - true: Floating, NMOS shut down
    pub fn set_gpio3_output(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x96, 0b0000_0001, state)
    }

    /// GPIO4 Output settings
    ///
    ///  - false: Output low level, NMOS turn on
    ///  - true: Floating, NMOS shut down
    pub fn set_gpio4_output(&mut self, state: bool) -> Result<(), E> {
        self.set_flag(0x96, 0b0000_0010, state)
    }
}
