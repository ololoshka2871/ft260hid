use std::time::Duration;

use crate::hid::consts::*;
use crate::hid::reports;
use crate::io::gpio;
use crate::{device::Device, Ft260Error, Ft260Result};

/// Status flags of the last I2C transaction (FT260 status register bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cStatus(pub u8);

impl I2cStatus {
    /// Controller busy: all other status bits are invalid.
    pub fn controller_busy(self) -> bool {
        self.0 & 0x01 != 0
    }

    /// Error condition on the last operation.
    pub fn error(self) -> bool {
        self.0 & 0x02 != 0
    }

    /// Arbitration lost during the last operation.
    pub fn arbitration_lost(self) -> bool {
        self.0 & 0x10 != 0
    }

    /// No slave acknowledged its address during the last operation.
    pub fn address_nack(self) -> bool {
        self.0 & 0x04 != 0
    }

    /// The slave acknowledged its address but not a data byte.
    pub fn data_nack(self) -> bool {
        self.0 & 0x08 != 0
    }

    /// The controller is idle (the last transaction finished).
    pub fn controller_idle(self) -> bool {
        self.0 & 0x20 != 0
    }

    /// The bus is held by another controller.
    pub fn bus_busy(self) -> bool {
        self.0 & 0x40 != 0
    }
}

/// Interface type to use I2C function of the FT2260 device.
#[derive(Debug)]
pub struct I2c<'a> {
    device: &'a Device,
    inited: bool,
}

/// Flags to indicate I2C bus conditions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Flag {
    /// Normal condition
    None,
    /// START condition
    Start,
    /// Repeated START condition
    ReStart,
    /// STOP condition
    Stop,
    /// START and STOP
    StartAndStop,
    /// Repeated START and STOP
    ReStartAndStop,
}

/// Default I2C clock speed value
pub const KBPS_DEFAULT: u16 = 100;

/// Feature report that carries system settings (0xA1).
const CONTROL_REPORT_ID: u8 = 0xA1;
/// `SetI2cMode` system command: enables the I2C function.
const CMD_SET_I2C_MODE: u8 = 0x02;
/// How long to wait for the controller to become ready / finish, per step.
const WAIT_STEP: Duration = Duration::from_millis(1);
/// Poll budget of one wait step sequence.
const WAIT_STEPS: u32 = 50;

/// Feature-report buffer of the FT260 (64 bytes).
fn feat_rep_buf() -> [u8; 64] {
    [0u8; 64]
}

/// Default timeout duration
pub const DURATION_WAIT_DEFAULT: Duration = Duration::from_millis(5000);

impl<'a> I2c<'a> {
    /// create a new `I2c` instance
    pub(crate) fn new(device: &'a Device) -> Self {
        Self {
            device,
            inited: false,
        }
    }

    /// Handle for a chip whose I2C clock was already configured.
    ///
    /// `init` also disables the shared GPIO0/1 pins and writes the clock as a
    /// feature report; repeating that before every transaction is both slow and
    /// races with the first transaction, so owners init once per chip.
    pub fn initialized(device: &'a Device) -> Self {
        Self {
            device,
            inited: true,
        }
    }

    /// Initialize I2C function: enable the module (if it is off) and set the clock.
    ///
    /// The FT260 has a system setting that turns the I2C function on; without it
    /// every transaction fails. The clock value is two bytes, MSB first
    /// (`SetI2cClockSpeed`), as FTDI's own tooling sends it.
    pub fn init(&mut self, kbps: u16) -> Ft260Result<()> {
        let gpio = self.device.gpio();
        if let Err(e) = gpio.disable_pin(gpio::Group::Gpio_0_1) {
            return Err(e);
        }
        if !self.is_enabled()? {
            self.enable()?;
        }
        match reports::i2c::init(&self.device, kbps) {
            Ok(()) => {
                self.inited = true;
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    /// Is the I2C function enabled in the chip's system settings?
    pub fn is_enabled(&self) -> Ft260Result<bool> {
        let mut buf = feat_rep_buf();
        buf[0] = CONTROL_REPORT_ID;
        let size = self.device.get_feature(&mut buf)?;
        Ok(size > 5 && buf[5] == 1)
    }

    /// Turn the I2C function on (system setting `SetI2cMode`).
    pub fn enable(&self) -> Ft260Result<()> {
        let mut buf = feat_rep_buf();
        buf[0] = CONTROL_REPORT_ID;
        buf[1] = CMD_SET_I2C_MODE;
        buf[2] = 1;
        self.device.set_feature(&buf)
    }

    /// Wait until the controller is idle and the bus is free; reset on an error
    /// or a lost arbitration.
    pub fn wait_bus_ready(&self) -> Ft260Result<()> {
        for _ in 0..WAIT_STEPS {
            let status = self.status()?;
            if status.error() || status.arbitration_lost() {
                self.reset()?;
            } else if status.controller_idle() && !status.bus_busy() {
                return Ok(());
            }
            std::thread::sleep(WAIT_STEP);
        }
        Err(Ft260Error::I2cError {
            message: "bus did not become ready".to_string(),
        })
    }

    /// Wait until the controller finished the last transaction.
    pub fn wait_transaction_complete(&self) -> Ft260Result<I2cStatus> {
        for _ in 0..WAIT_STEPS {
            let status = self.status()?;
            if !status.controller_busy() {
                return Ok(status);
            }
            std::thread::sleep(WAIT_STEP);
        }
        Err(Ft260Error::I2cError {
            message: "transaction did not complete".to_string(),
        })
    }

    /// Presence probe: write one zero byte with START+STOP and read the ACK bit.
    ///
    /// This is the sequence FTDI's tooling uses for address scanning: the length
    /// byte says one byte, no payload follows, and the verdict is the
    /// AddressNack bit — a read request answers with a dummy report even when
    /// nobody is on the bus.
    pub fn scan(&self, addr: u8) -> Ft260Result<bool> {
        self.wait_bus_ready()?;
        let mut buf = feat_rep_buf();
        buf[0] = ReportId::InOutI2cReport04 as u8;
        buf[1] = addr;
        buf[2] = I2cCondition::StartAndStop.bits();
        buf[3] = 1;
        self.device.write_output(&buf)?;
        std::thread::sleep(WAIT_STEP);
        let status = self.wait_transaction_complete()?;
        Ok(!(status.address_nack() || status.error() || status.arbitration_lost()))
    }

    fn flag_to_cond(f: Flag) -> I2cCondition {
        match f {
            Flag::None => I2cCondition::None,
            Flag::Start => I2cCondition::Start,
            Flag::ReStart => I2cCondition::ReStart,
            Flag::Stop => I2cCondition::Stop,
            Flag::StartAndStop => I2cCondition::StartAndStop,
            Flag::ReStartAndStop => I2cCondition::ReStartAndStop,
        }
    }

    /// Read I2C data
    pub fn read(
        &self,
        addr: u8,
        flag: Flag,
        buf: &mut [u8],
        len: usize,
        duration_wait: Duration,
    ) -> Ft260Result<usize> {
        assert!(self.inited);
        reports::i2c::read(
            self.device,
            addr,
            Self::flag_to_cond(flag),
            buf,
            len,
            duration_wait,
        )
    }

    /// Write I2C data
    pub fn write(&self, addr: u8, flag: Flag, buf: &[u8], len: usize) -> Ft260Result<usize> {
        assert!(self.inited);
        reports::i2c::write(self.device, addr, Self::flag_to_cond(flag), buf, len)
    }

    /// Write and read I2C data
    pub fn write_read(
        &self,
        addr: u8,
        buf_write: &[u8],
        len_write: usize,
        buf_read: &mut [u8],
        len_read: usize,
        duration_wait: Duration,
    ) -> Ft260Result<()> {
        assert!(self.inited);
        match reports::i2c::get_status(self.device) {
            Ok(s) => {
                if s != I2cBusStatus::ControllerIdle {
                    return Err(Ft260Error::I2cError {
                        message: "Bus was not in idle".to_string(),
                    });
                }
            }
            Err(e) => {
                return Err(e);
            }
        };
        match self.write(addr, Flag::Start, buf_write, len_write) {
            Ok(bytes_written) => {
                if bytes_written != len_write {
                    return Err(Ft260Error::I2cError {
                        message: "Failed to write all data".to_string(),
                    });
                }
            }
            Err(e) => {
                return Err(e);
            }
        };
        match reports::i2c::get_status(self.device) {
            Ok(s) => {
                if s.intersects(I2cBusStatus::AddressNack) || s.intersects(I2cBusStatus::DataNack) {
                    return Err(Ft260Error::I2cError {
                        message: "Device returned NACK".to_string(),
                    });
                }
            }
            Err(e) => {
                return Err(e);
            }
        };
        match self.read(
            addr,
            Flag::ReStartAndStop,
            buf_read,
            len_read,
            duration_wait,
        ) {
            Ok(bytes_read) => {
                if bytes_read != len_read {
                    return Err(Ft260Error::I2cError {
                        message: "Failed to read all data".to_string(),
                    });
                }
            }
            Err(e) => {
                return Err(e);
            }
        }
        Ok(())
    }

    /// Status flags of the last I2C transaction.
    ///
    /// `read` cannot tell a missing device from a silent one by the amount of
    /// data alone: the chip still emits a report. The NACK bits are the only
    /// reliable "no device at this address" signal.
    pub fn status(&self) -> Ft260Result<I2cStatus> {
        Ok(I2cStatus(reports::i2c::get_status(self.device)?.bits()))
    }

    /// If I2C bus is idling or busy
    pub fn is_idle(&self) -> Option<bool> {
        match reports::i2c::get_status(self.device) {
            Ok(s) => Some(s.contains(I2cBusStatus::ControllerIdle)),
            Err(e) => None,
        }
    }

    /// Reset the I2C controller (`I2CReset`) — the only recovery from a wedged bus.
    pub fn reset(&self) -> Ft260Result<()> {
        reports::i2c::reset(self.device)
    }
}

impl<'a> Drop for I2c<'a> {
    fn drop(&mut self) {
        if self.inited {
            // Best effort: panicking in Drop aborts the process.
            let _ = self.reset();
        }
    }
}
