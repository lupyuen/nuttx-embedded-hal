# Rust Embedded HAL for Apache NuttX RTOS

This crate provides Rust Embedded HAL interfaces (GPIO, I2C, SPI and Delay) for Apache NuttX RTOS.

For sample NuttX Rust apps, see [rust-i2c-nuttx](https://github.com/lupyuen/rust-i2c-nuttx) and [rust_test](https://github.com/lupyuen/rust_test)

If you find this crate useful, please support me on [GitHub Sponsors](https://github.com/sponsors/lupyuen)

More about NuttX Embedded HAL...

-   ["Rust talks I2C on Apache NuttX RTOS"](https://lupyuen.github.io/articles/rusti2c)

-   ["Rust on Apache NuttX OS"](https://lupyuen.github.io/articles/rust2)

# GPIO Output

```rust
//  Import Output Pin Trait
use embedded_hal::digital::v2::OutputPin;

//  Open /dev/gpio1 for GPIO Output
let mut gpio = nuttx_embedded_hal::OutputPin
    ::new("/dev/gpio1")
    .expect("open gpio failed");

//  Set Chip Select to Low
gpio.set_low()
    .expect("set gpio failed");

//  Set Chip Select to High
gpio.set_high()
    .expect("set gpio failed");
```

[(Documentation)](https://docs.rs/nuttx-embedded-hal/latest/nuttx_embedded_hal/struct.OutputPin.html)

[(Implementation)](https://lupyuen.github.io/articles/rust2#gpio-hal)

# GPIO Input

```rust
//  Import Input Pin Trait
use embedded_hal::digital::v2::InputPin;

//  Open /dev/gpio0 for GPIO Input
let gpio = nuttx_embedded_hal::InputPin
    ::new("/dev/gpio0")
    .expect("open gpio failed");

//  True if GPIO is High
let is_high = gpio.is_high()
    .expect("read gpio failed");

//  True if GPIO is Low
let is_low = gpio.is_low()
    .expect("read gpio failed");
```

[(Documentation)](https://docs.rs/nuttx-embedded-hal/latest/nuttx_embedded_hal/struct.InputPin.html)

# GPIO Interrupt

Interrupt callbacks are not supported yet.

```rust
//  Import Input Pin Trait
use embedded_hal::digital::v2::InputPin;

//  Open /dev/gpio2 for GPIO Interrupt
let gpio = nuttx_hal::InterruptPin
    ::new("/dev/gpio2");
    .expect("open gpio failed");

//  True if GPIO is High
let is_high = gpio.is_high()
    .expect("read gpio failed");

//  True if GPIO is Low
let is_low = gpio.is_low()
    .expect("read gpio failed");
```

[(Documentation)](https://docs.rs/nuttx-embedded-hal/latest/nuttx_embedded_hal/struct.InterruptPin.html)

# I2C

```rust
//  Import I2C Trait
use embedded_hal::blocking::i2c;

//  Open I2C Port /dev/i2c0
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

//  Write 0xA0 to Register 0xF5
i2c.write(
    0x77,          //  I2C Address
    &[0xF5, 0xA0]  //  Register ID and value
).expect("write register failed");
```

[(Documentation)](https://docs.rs/nuttx-embedded-hal/latest/nuttx_embedded_hal/struct.I2c.html)

[(Implementation)](https://lupyuen.github.io/articles/rusti2c#nuttx-embedded-hal)

# SPI

The SPI interface requires the SPI Test Driver (/dev/spitest0) to be installed:

-   [SPI Test Driver](https://github.com/lupyuen/incubator-nuttx/tree/master/drivers/rf)

SPI settings are configured in the [SPI Test Driver](https://github.com/lupyuen/incubator-nuttx/blob/master/drivers/rf/spi_test_driver.c#L39-L58).

```rust
//  Import SPI Trait
use embedded_hal::blocking::spi;

//  Open SPI Bus /dev/spitest0
let mut spi = nuttx_embedded_hal::Spi
    ::new("/dev/spitest0")
    .expect("open spi failed");

//  Open GPIO Output /dev/gpio1 for Chip Select
let mut cs = nuttx_embedded_hal::OutputPin
    ::new("/dev/gpio1")
    .expect("open gpio failed");

//  Set Chip Select to Low
cs.set_low()
    .expect("cs failed");

//  Transmit and receive SPI data
let mut data: [ u8; 5 ] = [ 0x1d, 0x00, 0x08, 0x00, 0x00 ];
spi.transfer(&mut data)
    .expect("spi failed");

//  Show the received SPI data
for i in 0..data.len() {
    println!("{:02x}", data[i as usize]);
}

//  Set Chip Select to High
cs.set_high()
    .expect("cs failed");
```

[(Documentation)](https://docs.rs/nuttx-embedded-hal/latest/nuttx_embedded_hal/struct.Spi.html)

[(Implementation)](https://lupyuen.github.io/articles/rust2#spi-hal)

# Delay

```rust
//  Import Delay Trait (milliseconds)
use embedded_hal::blocking::delay::DelayMs;

//  Get a Delay Interface
let mut delay = nuttx_embedded_hal::Delay;

//  Wait 500 milliseconds
delay.delay_ms(500_u32);
```

[(Documentation)](https://docs.rs/nuttx-embedded-hal/latest/nuttx_embedded_hal/struct.Delay.html)