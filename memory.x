/* nRF52840 memory map
 *
 * The nRF-Softdevice S140 v7.x occupies the first 0x26000 (152 KB) of flash.
 * Application code starts immediately after the SoftDevice.
 *
 * Flash bonded-device storage lives at 0x000E_F000, the last 4 KB page of the
 * 1 MB device. This is outside the application and SoftDevice regions.
 * When using the SoftDevice, bonded devices should eventually be persisted
 * via its flash API (sd_flash_write / sd_flash_page_erase); the current
 * development firmware still uses the NVMC driver directly.
 *
 * With SoftDevice:
 *   FLASH origin: 0x00026000  (after S140)
 *   RAM   origin: 0x20002000  (first 8 KB reserved by S140)
 */

MEMORY
{
  FLASH : ORIGIN = 0x00026000, LENGTH = 872K  /* 1024K - 152K SoftDevice */
  RAM   : ORIGIN = 0x20002000, LENGTH = 248K  /* 256K  -   8K SoftDevice */
}
