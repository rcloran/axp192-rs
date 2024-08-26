#![no_std]
#![no_main]

use axp192;
use embedded_hal as eh;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*,
    system::SystemControl,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        400u32.kHz(),
        &clocks,
        None,
    );

    let mut axp = axp192::Axp192::new(i2c);
    m5sc2_init(&mut axp, &mut delay).unwrap();

    loop {
        let bat_v = axp.get_battery_voltage().unwrap();
        let bat_ch_a = axp.get_battery_charge_current().unwrap();
        let bat_disch_a = axp.get_battery_discharge_current().unwrap();
        let usb_v = axp.get_acin_voltage().unwrap();
        let usb_a = axp.get_acin_current().unwrap();
        let temp = axp.get_internal_temperature().unwrap();
        println!("Bat {bat_v}v ch {bat_ch_a}A disch {bat_disch_a}A");
        println!("USB {usb_v}v @ {usb_a}A");
        println!("Internal temperature: {temp}°C");

        delay.delay_millis(1000u32);
    }
}

fn m5sc2_init<I2C, E>(axp: &mut axp192::Axp192<I2C>, delay: &mut Delay) -> Result<(), E>
where
    I2C: eh::i2c::I2c<Error = E>,
{
    // Default setup for M5Stack Core 2
    axp.set_dcdc1_voltage(3350)?; // Voltage to provide to the microcontroller (this one!)

    axp.set_ldo2_voltage(3300)?; // Peripherals (LCD, ...)
    axp.set_ldo2_on(true)?;

    axp.set_ldo3_voltage(2000)?; // Vibration motor
    axp.set_ldo3_on(false)?;

    axp.set_dcdc3_voltage(2800)?; // LCD backlight
    axp.set_dcdc3_on(true)?;

    axp.set_gpio1_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Power LED
    axp.set_gpio1_output(false)?; // In open drain modes, state is opposite to what you might
                                  // expect

    axp.set_gpio2_mode(axp192::GpioMode12::NmosOpenDrainOutput)?; // Speaker
    axp.set_gpio2_output(true)?;

    axp.set_key_mode(
        // Configure how the power button press will work
        axp192::ShutdownDuration::Sd4s,
        axp192::PowerOkDelay::Delay64ms,
        true,
        axp192::LongPress::Lp1000ms,
        axp192::BootTime::Boot512ms,
    )?;

    axp.set_gpio4_mode(axp192::GpioMode34::NmosOpenDrainOutput)?; // LCD reset control

    axp.set_battery_voltage_adc_enable(true)?;
    axp.set_battery_current_adc_enable(true)?;
    axp.set_acin_current_adc_enable(true)?;
    axp.set_acin_voltage_adc_enable(true)?;

    // Actually reset the LCD
    axp.set_gpio4_output(false)?;
    axp.set_ldo3_on(true)?; // Buzz the vibration motor while intializing ¯\_(ツ)_/¯
    delay.delay_millis(100u32);
    axp.set_gpio4_output(true)?;
    axp.set_ldo3_on(false)?;
    delay.delay_millis(100u32);

    Ok(())
}
