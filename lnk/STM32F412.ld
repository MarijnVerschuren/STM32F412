MEMORY {
    FLASH(rx) : ORIGIN = 0x08000000, LENGTH = 1M
    SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

INCLUDE "lnk/flash.ld"
INCLUDE "lnk/sram.ld"