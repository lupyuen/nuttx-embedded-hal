//! Embedded HAL for NuttX

use core::{
    str::FromStr,
};
use embedded_hal::{
    blocking::{
        delay::{DelayMs, DelayUs},
        i2c,
        spi,
    },
    digital::v2,
};
use crate::{
    close, ioctl, read, usleep, write,
    i2c_msg_s, i2c_transfer_s, size_t, ssize_t,
    GPIOC_WRITE, I2CIOC_TRANSFER, I2C_M_READ, O_RDWR,
    String,
};

/// NuttX I2C Read
impl i2c::Read for I2c {
    /// TODO: Error Type
    type Error = ();

    /// TODO: Read `buf` from I2C Port
    fn read(&mut self, _addr: u8, _buf: &mut [u8]) -> Result<(), Self::Error> {
        //  Not implemented, because BL602 needs the I2C Sub Address for read to work
        panic!();
    }
}

/// NuttX I2C Write
impl i2c::Write for I2c {
    /// TODO: Error Type
    type Error = ();

    /// Write `buf` to I2C Port.
    /// We assume this is a Write I2C Register operation, with Register ID at `buf[0]`.
    /// TODO: Handle other kinds of I2C operations
    fn write(&mut self, addr: u8, buf: &[u8]) -> Result<(), Self::Error> {
        //  Copy to local buffer because we need a mutable reference
        let mut buf2 = [0 ; 64];
        assert!(buf.len() <= buf2.len());
        buf2[..buf.len()].copy_from_slice(buf);

        //  Buffer for received I2C data
        let mut rbuf = [0 ; 1];

        //  Compose I2C Transfer
        let msg = [
            //  First I2C Message: Send Register ID and I2C Data as I2C Sub Address
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    buf2.as_mut_ptr(),     //  Buffer to be sent
                length:    buf.len() as ssize_t,  //  Number of bytes to send

                //  For BL602: Register ID must be passed as I2C Sub Address
                #[cfg(target_arch = "riscv32")]  //  If architecture is RISC-V 32-bit...
                flags:     crate::I2C_M_NOSTOP,  //  I2C Flags: Send I2C Sub Address
                
                //  Otherwise pass Register ID as I2C Data
                #[cfg(not(target_arch = "riscv32"))]  //  If architecture is not RISC-V 32-bit...
                flags:     0,  //  I2C Flags: None

                //  TODO: Check for BL602 specifically (by target_abi?), not just RISC-V 32-bit
            },
            //  Second I2C Message: Read I2C Data, because this forces BL602 to send the first message correctly
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    rbuf.as_mut_ptr(),      //  Buffer to be received
                length:    rbuf.len() as ssize_t,  //  Number of bytes to receive
                flags:     I2C_M_READ,  //  I2C Flags: Read I2C Data
            },
        ];
        
        //  Compose ioctl Argument to write I2C Registers
        let xfer = i2c_transfer_s {
            msgv: msg.as_ptr(),         //  Array of I2C messages for the transfer
            msgc: msg.len() as size_t,  //  Number of messages in the array
        };

        //  Execute I2C Transfer to write I2C Registers
        let ret = unsafe { 
            ioctl(
                self.fd,          //  I2C Port
                I2CIOC_TRANSFER,  //  I2C Transfer
                &xfer             //  I2C Messages for the transfer
            )
        };
        assert!(ret >= 0);   
        Ok(())
    }
}

/// NuttX I2C WriteRead
impl i2c::WriteRead for I2c {
    /// TODO: Error Type
    type Error = ();

    /// Write `wbuf` to I2C Port and read `rbuf` from I2C Port.
    /// We assume this is a Read I2C Register operation, with Register ID at `wbuf[0]`.
    /// TODO: Handle other kinds of I2C operations
    fn write_read(&mut self, addr: u8, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        //  We assume this is a Read I2C Register operation, with Register ID at wbuf[0]
        assert_eq!(wbuf.len(), 1);
        let reg_id = wbuf[0];

        //  Read I2C Registers, starting at Register ID
        let mut start = [reg_id ; 1];

        //  Compose I2C Transfer
        let msg = [
            //  First I2C Message: Send Register ID
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    start.as_mut_ptr(),      //  Buffer to be sent
                length:    start.len() as ssize_t,  //  Number of bytes to send

                //  For BL602: Register ID must be passed as I2C Sub Address
                #[cfg(target_arch = "riscv32")]  //  If architecture is RISC-V 32-bit...
                flags:     crate::I2C_M_NOSTOP,  //  I2C Flags: Send I2C Sub Address
                
                //  Otherwise pass Register ID as I2C Data
                #[cfg(not(target_arch = "riscv32"))]  //  If architecture is not RISC-V 32-bit...
                flags:     0,  //  I2C Flags: None

                //  TODO: Check for BL602 specifically (by target_abi?), not just RISC-V 32-bit
            },
            //  Second I2C Message: Receive Register Values
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    rbuf.as_mut_ptr(),      //  Buffer to be received
                length:    rbuf.len() as ssize_t,  //  Number of bytes to receive
                flags:     I2C_M_READ,  //  I2C Flags: Read I2C Data
            },
        ];

        //  Compose ioctl Argument
        let xfer = i2c_transfer_s {
            msgv: msg.as_ptr(),         //  Array of I2C messages for the transfer
            msgc: msg.len() as size_t,  //  Number of messages in the array
        };

        //  Execute I2C Transfer
        let ret = unsafe { 
            ioctl(
                self.fd,
                I2CIOC_TRANSFER,
                &xfer
            )
        };
        assert!(ret >= 0);   
        Ok(())
    }
}

/// NuttX SPI Transfer
impl spi::Transfer<u8> for Spi {
    /// TODO: Error Type
    type Error = ();

    /// Transfer SPI data
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        //  Transmit data
        let bytes_written = unsafe { 
            write(self.fd, words.as_ptr(), words.len() as u32) 
        };
        assert_eq!(bytes_written, words.len() as i32);

        //  Read response
        let bytes_read = unsafe { 
            read(self.fd, words.as_mut_ptr(), words.len() as u32) 
        };
        assert_eq!(bytes_read, words.len() as i32);

        //  Return response
        Ok(words)
    }
}

/// NuttX SPI Write
impl spi::Write<u8> for Spi{
    /// TODO: Error Type
    type Error = ();

    /// Write SPI data
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        //  Transmit data
        let bytes_written = unsafe { 
            write(self.fd, words.as_ptr(), words.len() as u32) 
        };
        assert_eq!(bytes_written, words.len() as i32);
        Ok(())
    }
}

/// Set NuttX Output Pin
impl v2::OutputPin for OutputPin {
    /// TODO: Error Type
    type Error = ();

    /// Set the GPIO Output to High
    fn set_high(&mut self) -> Result<(), Self::Error> {
        let ret = unsafe { 
            ioctl(self.fd, GPIOC_WRITE, 1) 
        };
        assert!(ret >= 0);
        Ok(())
    }

    /// Set the GPIO Output to low
    fn set_low(&mut self) -> Result<(), Self::Error> {
        let ret = unsafe { 
            ioctl(self.fd, GPIOC_WRITE, 0) 
        };
        assert!(ret >= 0);
        Ok(())
    }
}

/// Read NuttX Input Pin
impl v2::InputPin for InputPin {
    /// TODO: Error Type
    type Error = ();

    /// Return true if GPIO Input is high
    fn is_high(&self) -> Result<bool, Self::Error> {
        let mut invalue: i32 = 0;
        let addr: *mut i32 = &mut invalue;
        let ret = unsafe {
            ioctl(self.fd, crate::GPIOC_READ, addr)
        };
        assert!(ret >= 0);
        match invalue {
            0 => Ok(false),
            _ => Ok(true),
        }
    }

    /// Return true if GPIO Input is low
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

/// Read NuttX Interrupt Pin
impl v2::InputPin for InterruptPin {
    /// TODO: Error Type
    type Error = ();

    /// Return true if GPIO Input is high
    fn is_high(&self) -> Result<bool, Self::Error> {
        let mut invalue: i32 = 0;
        let addr: *mut i32 = &mut invalue;
        let ret = unsafe {
            ioctl(self.fd, crate::GPIOC_READ, addr)
        };
        assert!(ret >= 0);
        match invalue {
            0 => Ok(false),
            _ => Ok(true),
        }
    }

    /// Return true if GPIO Input is low
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

/// Set NuttX Unused Pin
impl v2::OutputPin for UnusedPin {
    /// TODO: Error Type
    type Error = ();

    /// Set the pin to high
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Set the pin to low
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// NuttX Delay in Microseconds
impl DelayUs<u8> for Delay {
    /// Sleep for us microseconds
    fn delay_us(&mut self, us: u8) {
        unsafe { usleep(us as u32); }
    }
}

/// NuttX Delay in Microseconds
impl DelayUs<u16> for Delay {
    /// Sleep for us microseconds
    fn delay_us(&mut self, us: u16) {
        unsafe { usleep(us as u32); }
    }
}

/// NuttX Delay in Microseconds
impl DelayUs<u32> for Delay {
    /// Sleep for us microseconds
    fn delay_us(&mut self, us: u32) {
        unsafe { usleep(us); }
    }
}

/// NuttX Delay in Milliseconds
impl DelayMs<u8> for Delay {
    /// Sleep for ms milliseconds
    fn delay_ms(&mut self, ms: u8) {
        unsafe { usleep(ms as u32 * 1000); }
    }
}

/// NuttX Delay in Milliseconds
impl DelayMs<u16> for Delay {
    /// Sleep for ms milliseconds
    fn delay_ms(&mut self, ms: u16) {
        unsafe { usleep(ms as u32 * 1000); }
    }
}

/// NuttX Delay in Milliseconds
impl DelayMs<u32> for Delay {
    /// Sleep for ms milliseconds
    fn delay_ms(&mut self, ms: u32) {
        unsafe { usleep(ms * 1000); }
    }
}

/// New NuttX I2C Bus
impl I2c {
    /// Create an I2C Bus from a Device Path (e.g. "/dev/i2c0")
    #[allow(dead_code)]
    pub fn new(path: &str, frequency: u32) -> Result<Self, i32> {
        //  Open the NuttX Device Path (e.g. "/dev/spitest0") for read-write
        let fd = open(path, O_RDWR);
        if fd < 0 { return Err(fd) }

        //  Return the I2C Bus
        Ok(Self { fd, frequency })
    }
}

/// New NuttX SPI Bus
impl Spi {
    /// Create an SPI Bus from a Device Path (e.g. "/dev/spitest0")
    #[allow(dead_code)]
    pub fn new(path: &str) -> Result<Self, i32> {
        //  Open the NuttX Device Path (e.g. "/dev/spitest0") for read-write
        let fd = open(path, O_RDWR);
        if fd < 0 { return Err(fd) }

        //  Return the SPI Bus
        Ok(Self { fd })
    }
}

/// New NuttX GPIO Input
impl InputPin {
    /// Create a GPIO Input Pin from a Device Path (e.g. "/dev/gpio0")
    #[allow(dead_code)]
    pub fn new(path: &str) -> Result<Self, i32> {
        //  Open the NuttX Device Path (e.g. "/dev/gpio0") for read-write
        let fd = open(path, O_RDWR);
        if fd < 0 { return Err(fd) }

        //  Return the pin
        Ok(Self { fd })
    }
}

/// New NuttX GPIO Output
impl OutputPin {
    /// Create a GPIO Output Pin from a Device Path (e.g. "/dev/gpio1")
    #[allow(dead_code)]
    pub fn new(path: &str) -> Result<Self, i32> {
        //  Open the NuttX Device Path (e.g. "/dev/gpio1") for read-write
        let fd = open(path, O_RDWR);
        if fd < 0 { return Err(fd) }

        //  Return the pin
        Ok(Self { fd })
    }
}

/// New NuttX GPIO Interrupt
impl InterruptPin {
    /// Create a GPIO Interrupt Pin from a Device Path (e.g. "/dev/gpio2")
    #[allow(dead_code)]
    pub fn new(path: &str) -> Result<Self, i32> {
        //  Open the NuttX Device Path (e.g. "/dev/gpio2") for read-write
        let fd = open(path, O_RDWR);
        if fd < 0 { return Err(fd) }

        //  Return the pin
        Ok(Self { fd })
    }
}

/// New NuttX GPIO Unused
impl UnusedPin {
    /// Create a GPIO Unused Pin
    #[allow(dead_code)]
    pub fn new() -> Result<Self, i32> {
        //  Return the pin
        Ok(Self {})
    }
}

/// Drop NuttX SPI Bus
impl Drop for I2c {
    /// Close the SPI Bus
    fn drop(&mut self) {
        unsafe { close(self.fd) };
    }
}

/// Drop NuttX SPI Bus
impl Drop for Spi {
    /// Close the SPI Bus
    fn drop(&mut self) {
        unsafe { close(self.fd) };
    }
}

/// Drop NuttX GPIO Input
impl Drop for InputPin {
    /// Close the GPIO Input
    fn drop(&mut self) {
        unsafe { close(self.fd) };
    }
}

/// Drop NuttX GPIO Output
impl Drop for OutputPin {
    /// Close the GPIO Output
    fn drop(&mut self) {
        unsafe { close(self.fd) };
    }
}

/// Drop NuttX GPIO Interrupt
impl Drop for InterruptPin {
    /// Close the GPIO Interrupt
    fn drop(&mut self) {
        unsafe { close(self.fd) };
    }
}

/// NuttX I2C Struct
pub struct I2c {
    /// NuttX File Descriptor
    fd: i32,
    /// I2C Frequency in Hz
    frequency: u32,
}

/// NuttX SPI Struct
pub struct Spi {
    /// NuttX File Descriptor
    fd: i32,
}

/// NuttX GPIO Input Struct
pub struct InputPin {
    /// NuttX File Descriptor
    fd: i32,
}

/// NuttX GPIO Output Struct
pub struct OutputPin {
    /// NuttX File Descriptor
    fd: i32,
}

/// NuttX GPIO Interrupt Struct
pub struct InterruptPin {
    /// NuttX File Descriptor
    fd: i32,
}

/// NuttX GPIO Unused Struct
pub struct UnusedPin {
}

/// NuttX Delay Struct
pub struct Delay;

/// Open a file and return the file descriptor.
/// TODO: Auto-generate this wrapper with `bindgen` from the C declaration
fn open(path: &str, oflag: i32) -> i32 {  //  `&str` is a reference to a string slice, similar to `const char *` in C

    use crate::open;

    //  Convert `str` to `String`, which similar to `char [64]` in C
    let mut s_with_null = String::from_str(path)  //  `mut` because we will modify it
        .expect("open conversion failed");        //  If it exceeds 64 chars, halt with an error
    
    //  Terminate the string with null, since we will be passing to C
    s_with_null.push('\0')
        .expect("open overflow");  //  If we exceed 64 chars, halt with an error

    //  Convert the null-terminated string to a pointer
    let p = s_with_null.as_str().as_ptr();

    //  Call the C function
    unsafe {  //  Flag this code as unsafe because we're calling a C function
        open(p, oflag)
    }

    //  No semicolon `;` here, so the value returned by the C function will be passed to our caller
}
