# Rust Embedded HAL for Apache NuttX RTOS

This crate provides Rust Embedded HAL interfaces (GPIO, I2C and SPI) for Apache NuttX RTOS.

For a sample NuttX Rust app, see [rust-i2c-nuttx](https://github.com/lupyuen/rust-i2c-nuttx)

If you find this crate useful, please support me on [GitHub Sponsors](https://github.com/sponsors/lupyuen)

# GPIO Output

```rust
//  Open GPIO Output
let mut gpio = nuttx_embedded_hal::OutputPin::new("/dev/gpio1")
    .expect("open gpio failed");

//  Set Chip Select to Low
gpio.set_low()
    .expect("cs failed");

//  Set Chip Select to High
gpio.set_high()
    .expect("cs failed");
```

# GPIO Input

```rust
//  Open GPIO Input
let gpio = nuttx_embedded_hal::InputPin::new("/dev/gpio0")
    .expect("open gpio failed");
```

# GPIO Interrupt

```rust
//  Open GPIO Interrupt
let gpio = nuttx_hal::InterruptPin::new("/dev/gpio2");
    .expect("open gpio failed");
```

# I2C

```rust
//  Open I2C Port
let mut i2c = nuttx_embedded_hal::I2c::new(
    "/dev/i2c0",  //  I2C Port
    400000,       //  I2C Frequency: 400 kHz
).expect("open failed");

//  Buffer for received I2C data
let mut buf = [0 ; 1];

//  Read register 0xD0 from I2C Address 0x77
i2c.write_read(
    0x77,     //  I2C Address
    &[0xD0],  //  Register ID
    &mut buf  //  Buffer to be received
).expect("read register failed");

//  Print the register value
println!("Register value is 0x{:02x}", buf[0]);
```

# SPI

The SPI interface requires the SPI Test Driver (/dev/spitest0) to be installed:

https://github.com/lupyuen/incubator-nuttx/tree/master/drivers/rf

```rust
//  Open GPIO Output for Chip Select
let mut cs = nuttx_embedded_hal::OutputPin::new("/dev/gpio1")
    .expect("open gpio failed");

//  Open SPI Bus
let mut spi = nuttx_embedded_hal::Spi::new("/dev/spitest0")
    .expect("open spi failed");

//  Set Chip Select to Low
cs.set_low()
    .expect("cs failed");

//  Transmit and receive SPI data
let mut data: [ u8; 5 ] = [ 0x1d, 0x00, 0x08, 0x00, 0x00 ];
spi.transfer(&mut data)
    .expect("spi failed");

//  Show the received SPI data
for i in 0..data.len() {
    println!("  {:02x}", data[i as usize]);
}

//  Set Chip Select to High
cs.set_high()
    .expect("cs failed");
```
