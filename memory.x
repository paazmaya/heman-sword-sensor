/* nRF52840 memory map
 *
 * This layout reserves 0x26000 (152 KB) for the S140 image. The locked FFI
 * headers identify S140 7.0.1; verify the installed binary's size/version
 * before flashing. Application code starts at 0x26000.
 *
 * Bond storage occupies 0x000EF000..0x000F0000. Retain that existing address
 * and end application flash before it. All writes use the SoftDevice API.
 *
 * With SoftDevice:
 *   FLASH origin: 0x00026000  (after S140)
 *   RAM   origin: 0x20010000 (64 KB reserved conservatively for S140).
 * Softdevice::enable checks the actual configuration's requirement at boot;
 * a board smoke test must confirm it fits this reservation.
 */

MEMORY
{
  FLASH : ORIGIN = 0x00026000, LENGTH = 804K
  RAM   : ORIGIN = 0x20010000, LENGTH = 192K
}
