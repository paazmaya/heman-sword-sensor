use core::{
    future::Future,
    pin::pin,
    task::{Context, Poll, Waker},
};
use xiao_nrf52840_sword::{
    storage::{
        clear_devices_async, decode_full_padded, store_device_async, AsyncBondedDeviceBackend,
        StorageError, FLASH_BONDED_DEVICE_SIZE, FLASH_BONDED_DEVICE_START, FULL_RECORD_PADDED_SIZE,
    },
    BondedDevice,
};

struct Flash {
    page: [u8; FLASH_BONDED_DEVICE_SIZE],
    calls: Vec<&'static str>,
    failure: Option<StorageError>,
}

impl Flash {
    fn new() -> Self {
        Self {
            page: [0xFF; FLASH_BONDED_DEVICE_SIZE],
            calls: Vec::new(),
            failure: None,
        }
    }

    fn record(&self) -> Option<(BondedDevice, [u8; 10])> {
        decode_full_padded(self.page[..FULL_RECORD_PADDED_SIZE].try_into().unwrap())
    }
}

struct YieldOnce(bool);

impl Future for YieldOnce {
    type Output = ();

    fn poll(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if self.0 {
            Poll::Ready(())
        } else {
            self.0 = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

impl AsyncBondedDeviceBackend for Flash {
    async fn erase_page(&mut self, address: u32) -> Result<(), StorageError> {
        assert_eq!(address, FLASH_BONDED_DEVICE_START);
        self.calls.push("erase-start");
        YieldOnce(false).await;
        if self.failure == Some(StorageError::Erase) {
            return Err(StorageError::Erase);
        }
        self.page.fill(0xFF);
        self.calls.push("erase-done");
        Ok(())
    }

    async fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), StorageError> {
        assert_eq!(address, FLASH_BONDED_DEVICE_START);
        assert_eq!(bytes.as_ptr() as usize % 4, 0, "S140 requires aligned RAM");
        assert_eq!(bytes.len(), FULL_RECORD_PADDED_SIZE);
        assert!(self.page.iter().all(|byte| *byte == 0xFF));
        self.calls.push("write-start");
        YieldOnce(false).await;
        if self.failure == Some(StorageError::Write) {
            return Err(StorageError::Write);
        }
        self.page[..bytes.len()].copy_from_slice(bytes);
        self.calls.push("write-done");
        Ok(())
    }
}

fn complete<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut context = Context::from_waker(Waker::noop());
    for _ in 0..10 {
        if let Poll::Ready(result) = future.as_mut().poll(&mut context) {
            return result;
        }
    }
    panic!("test future did not complete");
}

#[test]
fn async_production_persistence_waits_for_each_hardware_completion() {
    let mut flash = Flash::new();
    let mut device = BondedDevice::new([1, 2, 3, 4, 5, 6]);
    device.set_paired(true);
    device.timestamp = 1234;
    let uid = [4, 1, 2, 3, 0, 0, 0, 0, 0, 0];
    {
        let mut operation = pin!(store_device_async(&mut flash, &device, Some(&uid)));
        let mut context = Context::from_waker(Waker::noop());
        assert_eq!(operation.as_mut().poll(&mut context), Poll::Pending);
        assert_eq!(operation.as_mut().poll(&mut context), Poll::Pending);
        assert_eq!(operation.as_mut().poll(&mut context), Poll::Ready(Ok(())));
    }
    assert_eq!(
        flash.calls,
        ["erase-start", "erase-done", "write-start", "write-done"]
    );
    assert_eq!(flash.record(), Some((device, uid)));

    let restarted = Flash {
        page: flash.page,
        calls: Vec::new(),
        failure: None,
    };
    assert_eq!(restarted.record(), Some((device, uid)));
    device.mac = [6, 5, 4, 3, 2, 1];
    assert_eq!(
        complete(store_device_async(&mut flash, &device, Some(&uid))),
        Ok(())
    );
    assert_eq!(flash.record(), Some((device, uid)));
    assert_eq!(complete(clear_devices_async(&mut flash)), Ok(()));
    assert_eq!(flash.record(), None);
}

#[test]
fn erase_and_write_failures_are_not_success_shaped() {
    let device = BondedDevice::new([1; 6]);
    let mut flash = Flash::new();
    assert_eq!(
        complete(store_device_async(&mut flash, &device, None)),
        Ok(())
    );
    assert_eq!(flash.record(), Some((device, [0xFF; 10])));
    flash.failure = Some(StorageError::Erase);
    flash.calls.clear();
    assert_eq!(
        complete(store_device_async(&mut flash, &device, None)),
        Err(StorageError::Erase)
    );
    assert_eq!(flash.calls, ["erase-start"]);
    assert_eq!(flash.record(), Some((device, [0xFF; 10])));
    assert_eq!(
        complete(clear_devices_async(&mut flash)),
        Err(StorageError::Erase)
    );

    flash.failure = Some(StorageError::Write);
    flash.calls.clear();
    assert_eq!(
        complete(store_device_async(&mut flash, &device, None)),
        Err(StorageError::Write)
    );
    assert_eq!(flash.calls, ["erase-start", "erase-done", "write-start"]);
    assert_eq!(flash.record(), None, "erase/write is not power-loss atomic");
}
