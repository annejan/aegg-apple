MEMORY

# The bootloader has the first 64kb of the flash memory hence 0x10000  start address
{
  FLASH (rx)  : ORIGIN = 0x00010000, LENGTH = 960K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}
