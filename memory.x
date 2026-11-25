/* nRF52840 memory map
 *
 * The Seeed XIAO nRF52840 Sense USB bootloader uses S140 7.3.0, whose
 * application starts at 0x27000. The locked FFI headers use S140 7.0.1
 * (the same S140 API major version).
 *
 * The Seeed linker reserves 0x000ED000..0x000EF000 beyond the app image.
 * Bond storage occupies 0x000EF000..0x000F0000. All writes use the
 * SoftDevice API.
 *
 * With SoftDevice:
 *   FLASH origin: 0x00027000  (after S140)
 *   RAM   origin: 0x20010000 (64 KB reserved conservatively for S140).
 * Softdevice::enable checks the actual configuration's requirement at boot;
 * a board smoke test must confirm it fits this reservation.
 */

MEMORY
{
  FLASH : ORIGIN = 0x00027000, LENGTH = 792K
  RAM   : ORIGIN = 0x20010000, LENGTH = 192K
}
