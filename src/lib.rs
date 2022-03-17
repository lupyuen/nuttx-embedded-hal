//! Rust Embedded HAL for Apache NuttX RTOS

#![no_std]  //  Use the Rust Core Library instead of the Rust Standard Library, which is not compatible with embedded systems

//  Import NuttX Macros
#[macro_use]
mod macros;

//  Import NuttX HAL
mod hal;
pub use hal::*;

//  Import Libraries
use core::{            //  Rust Core Library
    fmt,               //  String Formatting    
    str::FromStr,      //  For converting `str` to `String`
};

/// Print a message to the serial console.
/// TODO: Auto-generate this wrapper with `bindgen` from the C declaration
pub fn puts(s: &str) -> i32 {  //  `&str` is a reference to a string slice, similar to `const char *` in C

    extern "C" {  //  Import C Function
        /// Print a message to the serial console (from C stdio library)
        fn puts(s: *const u8) -> i32;
    }

    //  Convert `str` to `String`, which similar to `char [64]` in C
    //  TODO: Increase the buffer size if we're sure we won't overflow the stack
    let mut s_with_null = String::from_str(s)  //  `mut` because we will modify it
        .expect("puts conversion failed");     //  If it exceeds 64 chars, halt with an error
    
    //  Terminate the string with null, since we will be passing to C
    s_with_null.push('\0')
        .expect("puts overflow");  //  If we exceed 64 chars, halt with an error

    //  Convert the null-terminated string to a pointer
    let p = s_with_null.as_str().as_ptr();

    //  Call the C function
    unsafe {  //  Flag this code as unsafe because we're calling a C function
        puts(p)
    }

    //  No semicolon `;` here, so the value returned by the C function will be passed to our caller
}

/// Print a formatted message to the serial console. Called by println! macro.
pub fn puts_format(args: fmt::Arguments<'_>) {
    //  Allocate a 64-byte buffer.
    //  TODO: Increase the buffer size if we're sure we won't overflow the stack
    let mut buf = String::new();

    //  Format the message into the buffer
    fmt::write(&mut buf, args)
        .expect("puts_format overflow");

    //  Print the buffer
    puts(&buf);
}

/// Limit Strings to 64 chars, similar to `char[64]` in C
pub type String = heapless::String::<64>;

extern "C" {  //  Import POSIX Functions. TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/unistd.h
    pub fn open(path: *const u8, oflag: i32, ...) -> i32;
    pub fn read(fd: i32, buf: *mut u8, count: size_t)    -> ssize_t;
    pub fn write(fd: i32, buf: *const u8, count: size_t) -> ssize_t;
    pub fn close(fd: i32) -> i32;
    pub fn ioctl(fd: i32, request: i32, ...) -> i32;  //  On NuttX: request is i32, not u64 like Linux
    pub fn sleep(secs: u32)  -> u32;
    pub fn usleep(usec: u32) -> u32;
    pub fn exit(status: u32) -> !;
}

//  GPIO ioctl Commands. TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rust/include/nuttx/ioexpander/gpio.h
/// Set the value of an output GPIO
pub const GPIOC_WRITE:      i32 = _GPIOBASE | 1;  //  _GPIOC(1)
/// Read the value of an input or output GPIO
pub const GPIOC_READ:       i32 = _GPIOBASE | 2;  //  _GPIOC(2)
/// Return the GPIO pin type.
pub const GPIOC_PINTYPE:    i32 = _GPIOBASE | 3;  //  _GPIOC(3)
/// Register to receive a signal whenever there an interrupt
/// is received on an input gpio pin.  This feature, of course,
/// depends upon interrupt GPIO support from the platform.
pub const GPIOC_REGISTER:   i32 = _GPIOBASE | 4;  //  _GPIOC(4)
/// Stop receiving signals for pin interrupts.
pub const GPIOC_UNREGISTER: i32 = _GPIOBASE | 5;  //  _GPIOC(5)
/// Set the GPIO pin type.
pub const GPIOC_SETPINTYPE: i32 = _GPIOBASE | 6;  //  _GPIOC(6)

//  GPIO Constants. TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rust/include/fcntl.h
/// GPIO driver commands
pub const _GPIOBASE: i32 = 0x2300;
//  #define _GPIOC(nr)       _IOC(_GPIOBASE,nr)
//  #define _IOC(type,nr)    ((type)|(nr))

//  I2C ioctl Commands.  TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L105-L129
/// Perform an I2C transfer
pub const I2CIOC_TRANSFER: i32 = _I2CBASE | 0x0001;  //  _I2CIOC(0x0001)
/// Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
pub const I2CIOC_RESET:    i32 = _I2CBASE | 0x0002;  //  _I2CIOC(0x0002)
//  #define _I2CIOC(nr)       _IOC(_I2CBASE,nr)
//  #define _IOC(type,nr)     ((type)|(nr))

//  I2C Constants. TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L93-L96
/// Read data, from slave to master
pub const I2C_M_READ:    u16 = 0x0001;
/// Ten bit address
pub const I2C_M_TEN:     u16 = 0x0002;
/// Message should not end with a STOP
pub const I2C_M_NOSTOP:  u16 = 0x0040;
/// Message should not begin with a START
pub const I2C_M_NOSTART: u16 = 0x0080;
/// I2C driver commands
pub const _I2CBASE:      i32 = 0x2100; 

/// I2C Message Struct: I2C transaction segment beginning with a START. A number of these can
/// be transferred together to form an arbitrary sequence of write/read
/// transfer to an I2C device.
/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L208-L215
#[repr(C)]
pub struct i2c_msg_s {
    /// I2C Frequency
    pub frequency: u32,
    /// I2C Address
    pub addr: u16,
    /// I2C Flags (I2C_M_*)
    pub flags: u16,
    /// Buffer to be transferred
    pub buffer: *mut u8,
    /// Length of the buffer in bytes
    pub length: ssize_t,
}

/// I2C Transfer Struct: This structure is used to communicate with the I2C character driver in
/// order to perform IOCTL transfers.
/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L231-L235
#[repr(C)]
pub struct i2c_transfer_s {
    /// Array of I2C messages for the transfer
    pub msgv: *const i2c_msg_s,
    /// Number of messages in the array
    pub msgc: size_t,
}

//  Input / Output Constants. TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rust/include/fcntl.h
/// Open for read access (only)
pub const O_RDONLY: i32 = 1 << 0;       
/// Read access is permitted (non-standard)
pub const O_RDOK:   i32 = O_RDONLY;      
/// Open for write access (only)
pub const O_WRONLY: i32 = 1 << 1;        
/// Write access is permitted (non-standard)
pub const O_WROK:   i32 = O_WRONLY;      
/// Open for both read & write access
pub const O_RDWR:   i32 = O_RDOK|O_WROK; 

/// size_t for NuttX 32-bit. TODO: Support other architectures
#[allow(non_camel_case_types)]
pub type size_t = u32;

/// ssize_t for NuttX 32-bit. TODO: Support other architectures
#[allow(non_camel_case_types)]
pub type ssize_t = i32;
