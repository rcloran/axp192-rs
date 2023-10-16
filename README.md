# AXP192 Power Management IC, Rust library

[![Build status](https://img.shields.io/github/actions/workflow/status/rcloran/axp192-rs/rust_ci.yml?logo=github)](https://github.com/rcloran/axp192-rs/actions)
[![docs.rs](https://img.shields.io/docsrs/axp192?logo=rust)](https://docs.rs/axp192)
[![Crates.io](https://img.shields.io/crates/v/axp192?logo=rust)](https://crates.io/crates/axp192)


## Example

`examples/m5stack-core2.rs` demonstrates configuring and reading from the
AXP192 on an [M5Stack Core2](https://docs.m5stack.com/en/core/core2).

This example uses the [Rust on ESP](https://esp-rs.github.io/book/) toolchain
and [esp32-hal crate](https://crates.io/crates/esp32-hal), which requires [some
setup](https://esp-rs.github.io/book/installation/).

You can run this example on your device with `cargo run --example m5stack-core2`.
