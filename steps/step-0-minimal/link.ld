/*taken from https://github.com/cherry-k/bare-metal-programming-guide/blob/main/steps/step-0-minimal/link.ld
changed by Larissa Pauser according to reference manual for STM32F746ZGT6*/

ENTRY(_reset);
MEMORY {
  flash(rx)  : ORIGIN = 0x08000000, LENGTH = 1024k
  sram(rwx) : ORIGIN = 0x20010000, LENGTH = 240k  /* this is only SRAM1 */
}
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */

SECTIONS {
/*first put the vector table in flash, then text (firmware code) and then read-only data*/
  .vectors  : { KEEP(*(.vectors)) }   > flash
  .text     : { *(.text*) }           > flash
  .rodata   : { *(.rodata*) }         > flash
/*copy the data section in the _reset function to ram (so we can work with it), but keep the addresses still in the ram*/
  .data : {
    _sdata = .;   /* .data section start */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* .data section end */
  } > sram AT > flash
  _sidata = LOADADDR(.data);
/*copy the uninitialized data in the _reset function to ram*/
  .bss : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram

  . = ALIGN(8);
  _end = .;     /* for cmsis_gcc.h  */
}
