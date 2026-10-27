use xiao_nrf52840_sword::storage::{BondedDeviceBackend, StorageError};

pub struct MemoryFlash {
    pub bytes: [u8; 4096],
    pub fail_read: bool,
    pub fail_erase: bool,
    pub fail_write: bool,
    pub erase_count: usize,
    pub write_count: usize,
}

impl Default for MemoryFlash {
    fn default() -> Self {
        Self {
            bytes: [0xFF; 4096],
            fail_read: false,
            fail_erase: false,
            fail_write: false,
            erase_count: 0,
            write_count: 0,
        }
    }
}

impl MemoryFlash {
    fn range(&self, address: u32, length: usize) -> Result<std::ops::Range<usize>, StorageError> {
        let start = address
            .checked_sub(xiao_nrf52840_sword::storage::FLASH_BONDED_DEVICE_START)
            .ok_or(StorageError::Read)? as usize;
        let end = start.checked_add(length).ok_or(StorageError::Read)?;
        (end <= self.bytes.len())
            .then_some(start..end)
            .ok_or(StorageError::Read)
    }
}

impl BondedDeviceBackend for MemoryFlash {
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), StorageError> {
        if self.fail_read {
            return Err(StorageError::Read);
        }
        let range = self.range(address, output.len())?;
        output.copy_from_slice(&self.bytes[range]);
        Ok(())
    }

    fn erase_page(&mut self, address: u32) -> Result<(), StorageError> {
        if self.fail_erase {
            return Err(StorageError::Erase);
        }
        let start = address
            .checked_sub(xiao_nrf52840_sword::storage::FLASH_BONDED_DEVICE_START)
            .ok_or(StorageError::Erase)? as usize;
        if start >= self.bytes.len() {
            return Err(StorageError::Erase);
        }
        self.bytes.fill(0xFF);
        self.erase_count += 1;
        Ok(())
    }

    fn write(&mut self, address: u32, bytes: &[u8]) -> Result<(), StorageError> {
        if self.fail_write {
            return Err(StorageError::Write);
        }
        let range = self.range(address, bytes.len())?;
        self.bytes[range].copy_from_slice(bytes);
        self.write_count += 1;
        Ok(())
    }
}
