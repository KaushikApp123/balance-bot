/*
 * STM32F411CEU6: 512 KB flash, 128 KB RAM.
 *
 * Flash sector 1 (16 KB at 0x08004000) is kept free for saved PID gains.
 * A small sector erases in ~0.25 s instead of ~1 s for the 128 KB sectors,
 * which keeps a "save gains" request well inside the watchdog window.
 * Sector 0 holds the vector table, sectors 2..7 hold everything else.
 */
ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x200;   /* newlib only; FreeRTOS has its own heap */
_Min_Stack_Size = 0x800;   /* MSP: startup code and interrupts        */

MEMORY
{
  FLASH_VEC (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
  CONFIG    (r)   : ORIGIN = 0x08004000, LENGTH = 16K
  FLASH     (rx)  : ORIGIN = 0x08008000, LENGTH = 480K
  RAM       (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}

__config_start = ORIGIN(CONFIG);
__config_size  = LENGTH(CONFIG);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH_VEC

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*)
    *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata) *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array    : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array    : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    *(.RamFunc) *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
