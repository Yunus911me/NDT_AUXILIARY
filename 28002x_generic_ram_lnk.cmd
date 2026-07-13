/*
 * 28002x_ndt_flash_lnk.cmd
 * ────────────────────────────────────────────────────────────────────────
 * FLASH linker command file for the F280025 NDT auxiliary board.
 *
 * Why flash (not the generic RAM linker):
 *   .text is ~0x24D0 words. The device has only ~12K words of SRAM total,
 *   and g_waveBuf alone is 0x1000 words. Code + waveform buffer + stack do
 *   not fit in RAM. Code therefore runs from flash; ONLY the timing-critical
 *   capture loop (.TI.ramfunc) is copied to RAM and executes there.
 *
 * Boot flow with this file:
 *   1. Boot ROM jumps to 0x080000 (codestart / BEGIN in flash).
 *   2. c_int00 → __TI_auto_init copies .cinit/.data init images from flash.
 *   3. main() copies .TI.ramfunc from flash (LOAD) to RAMM0 (RUN) — see the
 *      NDT_copyRamfuncs() snippet you must add (below this file's comments).
 *   4. Flash wait-states / prefetch configured (Flash_initModule) before any
 *      flash execution at full speed.
 *
 * Symbols exported for the ramfunc copy (EABI naming):
 *   RamfuncsLoadStart, RamfuncsLoadSize, RamfuncsRunStart
 * ──────────────────────────────────────────────────────────────────────── */

MEMORY
{
   /* ── Flash entry point (2 words the boot ROM branches to) ───────────── */
   BEGIN             : origin = 0x080000, length = 0x000002

   /* ── On-chip SRAM ───────────────────────────────────────────────────── */
   BOOT_RSVD         : origin = 0x00000002, length = 0x00000126
   RAMM0             : origin = 0x00000128, length = 0x000002D8  /* ramfunc RUN */
   RAMM1             : origin = 0x00000400, length = 0x000003F8  /* .stack      */
// RAMM1_RSVD        : origin = 0x000007F8, length = 0x00000008  /* errata      */

   RAMLS4567         : origin = 0x0000A000, length = 0x00002000  /* .bss/.data  */
   RAMGS0            : origin = 0x0000C000, length = 0x000007F8
// RAMGS0_RSVD       : origin = 0x0000C7F8, length = 0x00000008  /* errata      */

   RESET             : origin = 0x003FFFC0, length = 0x00000002

   /* ── Flash BANK 0 (SEC0 leaves 2 words at 0x080000 for BEGIN) ────────── */
   FLASH_BANK0_SEC0  : origin = 0x080002, length = 0x000FFE
   FLASH_BANK0_SEC1  : origin = 0x081000, length = 0x001000
   FLASH_BANK0_SEC2  : origin = 0x082000, length = 0x001000
   FLASH_BANK0_SEC3  : origin = 0x083000, length = 0x001000
   FLASH_BANK0_SEC4  : origin = 0x084000, length = 0x001000
   FLASH_BANK0_SEC5  : origin = 0x085000, length = 0x001000
   FLASH_BANK0_SEC6  : origin = 0x086000, length = 0x001000
   FLASH_BANK0_SEC7  : origin = 0x087000, length = 0x001000
   FLASH_BANK0_SEC8  : origin = 0x088000, length = 0x001000
   FLASH_BANK0_SEC9  : origin = 0x089000, length = 0x001000
   FLASH_BANK0_SEC10 : origin = 0x08A000, length = 0x001000
   FLASH_BANK0_SEC11 : origin = 0x08B000, length = 0x001000
   FLASH_BANK0_SEC12 : origin = 0x08C000, length = 0x001000
   FLASH_BANK0_SEC13 : origin = 0x08D000, length = 0x001000
   FLASH_BANK0_SEC14 : origin = 0x08E000, length = 0x001000
   FLASH_BANK0_SEC15 : origin = 0x08F000, length = 0x000FF8
// FLASH_BANK0_SEC15_RSVD : origin = 0x08FFF0, length = 0x000010 /* errata      */

   BOOTROM           : origin = 0x003F0000, length = 0x00008000
   BOOTROM_EXT       : origin = 0x003F8000, length = 0x00007FC0
}

SECTIONS
{
   /* ── Flash entry ──────────────────────────────────────────────────────── */
   codestart         : > BEGIN, ALIGN(4)

   /* ── Code in flash. '>>' lets .text span contiguous flash sectors. ────── */
   .text             : >> FLASH_BANK0_SEC0 | FLASH_BANK0_SEC1 | FLASH_BANK0_SEC2 |
                          FLASH_BANK0_SEC3 | FLASH_BANK0_SEC4 | FLASH_BANK0_SEC5 |
                          FLASH_BANK0_SEC6 | FLASH_BANK0_SEC7, ALIGN(8)

   .cinit            : > FLASH_BANK0_SEC8, ALIGN(8)
   .switch           : > FLASH_BANK0_SEC8, ALIGN(8)
   .reset            : > RESET, TYPE = DSECT   /* unused vector */

   /* ── Timing-critical capture loop: LOAD in flash, RUN from RAMM0. ─────── */
   .TI.ramfunc       : LOAD = FLASH_BANK0_SEC9,
                       RUN  = RAMM0,
                       LOAD_START(RamfuncsLoadStart),
                       LOAD_SIZE (RamfuncsLoadSize),
                       LOAD_END  (RamfuncsLoadEnd),
                       RUN_START (RamfuncsRunStart),
                       RUN_SIZE  (RamfuncsRunSize),
                       RUN_END   (RamfuncsRunEnd),
                       ALIGN(8)

   /* ── RAM: stack, uninitialised + initialised data ────────────────────── */
   .stack            : > RAMM1

#if defined(__TI_EABI__)
   .bss              : > RAMLS4567          /* g_waveBuf (0x1000 w) lives here */
   .bss:output       : > RAMLS4567
   .init_array       : > FLASH_BANK0_SEC8, ALIGN(8)
   .const            : > FLASH_BANK0_SEC10, ALIGN(8)
   .data             : > RAMLS4567 | RAMGS0
   .sysmem           : > RAMLS4567
#else
   .pinit            : > FLASH_BANK0_SEC8, ALIGN(8)
   .ebss             : > RAMLS4567
   .econst           : > FLASH_BANK0_SEC10, ALIGN(8)
   .esysmem          : > RAMLS4567
#endif

   .cio              : > RAMGS0
   ramgs0            : > RAMGS0

   /* ── IQmath (harmless if unused) ─────────────────────────────────────── */
   IQmath            : > FLASH_BANK0_SEC11, ALIGN(8)
   IQmathTables      : > FLASH_BANK0_SEC11, ALIGN(8)
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/