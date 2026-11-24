use std::sync::MutexGuard;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
/// FT260 Device
use std::thread;
use std::thread::JoinHandle;

// use hidapi crate only in this module
use hidapi::{DeviceInfo as HidDeviceInfo, HidError};

use crate::hid::rep_fifo::ReportFifo;
use crate::io::{gpio::Gpio, i2c::I2c, uart::Uart};
use crate::{Ft260Error, Ft260Result};

/// conversion of `HidError` into `Ft260Error`
impl From<HidError> for Ft260Error {
    fn from(err: HidError) -> Self {
        match err {
            HidError::HidApiError { message: _ }
            | HidError::HidApiErrorEmpty
            | HidError::FromWideCharError { wide_char: _ }
            | HidError::InitializationError
            | HidError::InvalidZeroSizeData
            | HidError::IncompleteSendError { sent: _, all: _ }
            | HidError::SetBlockingModeError { mode: _ }
            | HidError::OpenHidDeviceWithDeviceInfoError { device_info: _ }
            | HidError::IoError { error: _ } => Ft260Error::HidError {
                message: format!("{}", err),
            },
        }
    }
}

/// Data struct for FT260 device

#[derive(Debug)]
pub struct Device {
    hid: Arc<Mutex<hidapi::HidDevice>>,
    fifo: Arc<Mutex<ReportFifo>>,
    reading: Arc<AtomicBool>,
    /// `None` in sequential mode (no reader thread).
    handle: Option<JoinHandle<()>>,
    /// Reads are driven by the caller, not by a background thread.
    sequential: bool,
}

/// Default USB Vendor ID for FT260 device
pub const VID_DEFAULT: u16 = 0x0403;
/// Default USB Product ID for FT260 device
pub const PID_DEFAULT: u16 = 0x6030;

/// Open FT260 HID interface with its number.  
/// `interface` should take `0` or `1` only.  
/// If it could be opened, a new thread is spawned and it continues to read HID input report from it.
pub fn open(interface: i32) -> Option<Device> {
    open_by_vid_pid(VID_DEFAULT, PID_DEFAULT, interface)
}

/// Open FT260 HID by explicit Vendor ID and Product ID.  
/// `interface` should take `0` or `1` only.
pub fn open_by_vid_pid(vendor_id: u16, product_id: u16, interface: i32) -> Option<Device> {
    Device::try_new(vendor_id, product_id, interface, 0) // try find only the first one
}

/// One FT260 HID interface found on the host.
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    /// HID path, the argument of [`open_by_path`].
    pub path: String,
    /// HID interface number (0 = I2C/GPIO, 1 = UART).
    pub interface: i32,
    pub serial_number: Option<String>,
}

/// Every FT260 HID interface on the host (both interfaces of every chip).
pub fn enumerate() -> Vec<DeviceInfo> {
    let Ok(api) = hidapi::HidApi::new() else {
        return Vec::new();
    };
    api.device_list()
        .filter(|info| info.vendor_id() == VID_DEFAULT && info.product_id() == PID_DEFAULT)
        .map(|info| DeviceInfo {
            path: info.path().to_string_lossy().into_owned(),
            interface: info.interface_number(),
            serial_number: info.serial_number().map(str::to_string),
        })
        .collect()
}

/// Open FT260 by its HID device path (as enumerated by `hidapi`).
///
/// Several FT260 chips can be attached at once; the path picks one of them,
/// whereas [`open`] always takes the first matching device.
pub fn open_by_path(path: &str) -> Option<Device> {
    let api = hidapi::HidApi::new().ok()?;
    let info = api
        .device_list()
        .find(|info| info.path().to_string_lossy() == path)?;
    let hid = info.open_device(&api).ok()?;
    Some(Device::new(hid))
}

/// Open FT260 by HID path **without** the background reader thread.
///
/// On Windows a pending overlapped read on a vendor-defined HID interface can
/// hold back `hid_write` (it dies with `ERROR_IO_PENDING` after the write
/// timeout), so a write-then-read protocol deadlocks with a reader thread
/// running. In this mode every read is done synchronously by the caller.
pub fn open_by_path_sequential(path: &str) -> Option<Device> {
    let api = hidapi::HidApi::new().ok()?;
    let info = api
        .device_list()
        .find(|info| info.path().to_string_lossy() == path)?;
    let hid = info.open_device(&api).ok()?;
    Some(Device {
        hid: Arc::new(Mutex::new(hid)),
        fifo: Arc::new(Mutex::new(ReportFifo::new())),
        reading: Arc::new(AtomicBool::new(false)),
        handle: None,
        sequential: true,
    })
}

impl Device {
    /// Create new `Device` instance from `hidapi::HidDevice` instance.  
    /// When it is opened, a new thread is spawned and it continues to read HID input report from the device.
    fn new(hid: hidapi::HidDevice) -> Self {
        let mutex_hid = Arc::new(Mutex::new(hid));
        let mutex_fifo = Arc::new(Mutex::new(ReportFifo::new()));
        let reading = Arc::new(AtomicBool::new(true));

        let handle = thread::spawn({
            let mutex_hid = mutex_hid.clone();
            let mutex_fifo = mutex_fifo.clone();
            let reading = reading.clone();
            move || loop {
                let mut buf = [0u8; 256];
                let mut has_report = false;
                if let Ok(hid) = mutex_hid.lock() {
                    if let Ok(sz) = hid.read_timeout(&mut buf, 0) {
                        has_report = sz > 0;
                    }
                }
                thread::yield_now();
                if has_report {
                    if let Ok(mut fifo) = mutex_fifo.lock() {
                        fifo.push_report(buf.to_vec());
                    }
                }
                thread::yield_now();
                if !reading.load(Ordering::Relaxed) {
                    return;
                }
            }
        });

        Self {
            hid: mutex_hid,
            fifo: mutex_fifo,
            reading,
            handle: Some(handle),
            sequential: false,
        }
    }

    /// Enumerate HID interfaces with specified conditions (VID, PID, IF#)  
    /// If some were found, then create new `Device` instance from `hidapi::HidDevice` instance
    fn try_new(vendor_id: u16, product_id: u16, interface: i32, index: usize) -> Option<Self> {
        if interface < 0 || interface > 1 {
            return None;
        }

        let api = hidapi::HidApi::new().expect("Failed to create HID API context");
        let mut infs: Vec<&HidDeviceInfo> = Vec::new();
        for inf in api.device_list() {
            if (vendor_id, product_id, interface)
                == (inf.vendor_id(), inf.product_id(), inf.interface_number())
            {
                infs.push(inf);
            }
        }
        if index >= infs.len() {
            // out of index range
            None
        } else {
            if let Ok(hid) = infs[index].open_device(&api) {
                Some(Self::new(hid))
            } else {
                None
            }
        }
    }

    /// True when reads are done synchronously by the caller (no reader thread).
    pub(crate) fn is_sequential(&self) -> bool {
        self.sequential
    }

    /// Exclusive reference to FIFO instance for HID input report from FT260 device
    pub(crate) fn fifo<'a>(&'a self) -> MutexGuard<'a, ReportFifo> {
        self.fifo.lock().unwrap()
    }

    /// Create instance to control GPIO features
    pub fn gpio(&self) -> Gpio {
        Gpio::new(self)
    }

    /// Create instance to control I2C features
    pub fn i2c(&self) -> I2c {
        I2c::new(self)
    }

    /// Create instance to control UART features
    pub fn uart(&self) -> Uart {
        Uart::new(self)
    }

    /// Read input report manually
    pub(crate) fn read_input(&self, buf: &mut [u8], timeout: i32) -> Ft260Result<usize> {
        match self.hid.lock().unwrap().read_timeout(buf, timeout) {
            Ok(sz) => Ft260Result::Ok(sz),
            Err(e) => Ft260Result::Err(Ft260Error::from(e)),
        }
    }

    /// Write HID output report manually
    pub(crate) fn write_output(&self, data: &[u8]) -> Ft260Result<()> {
        match self.hid.lock().unwrap().write(data) {
            Ok(_) => Ft260Result::Ok(()),
            Err(e) => Ft260Result::Err(Ft260Error::from(e)),
        }
    }

    /// Read HID feature report
    pub(crate) fn get_feature(&self, buf: &mut [u8]) -> Ft260Result<usize> {
        match self.hid.lock().unwrap().get_feature_report(buf) {
            Ok(sz) => Ft260Result::Ok(sz),
            Err(e) => Ft260Result::Err(Ft260Error::from(e)),
        }
    }

    /// Write HID feature report
    pub(crate) fn set_feature(&self, data: &[u8]) -> Ft260Result<()> {
        match self.hid.lock().unwrap().send_feature_report(data) {
            Ok(_) => Ft260Result::Ok(()),
            Err(e) => Ft260Result::Err(Ft260Error::from(e)),
        }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        self.reading.store(false, Ordering::Relaxed);
        let Some(handle) = self.handle.as_ref() else {
            return;
        };
        loop {
            if handle.is_finished() {
                return;
            }
        }
    }
}
