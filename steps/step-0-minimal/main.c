// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved
// Changed by Larissa Pauser for ARM-Cortex M7

int main(void) {
  return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void); //defined in link.ld

//16 standard and 98 specific interrupt handlers
//define an array of 16+98 pointers to functions which return nothing and take 2 arguments
//every such function is an interrupt request handler
//this array is the vector table
__attribute__((section(".vectors")))
void (*tab[16+98])(void) = {_estack,_reset};