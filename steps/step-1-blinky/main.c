// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved
// Changed by Larissa Pauser for STM32F746ZGT6
#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
// to address a specific pin at a specific bank, use a 16bit value, where the first byte is the bank and the second byte is the pin
#define PIN(bank, num) ((bank-'A') << 8 | (num))
// defines for both parts of the PIN 16 bit value
#define PINBANK(pin) (pin >> 8)
#define PINNO(pin) (pin & 255)

//each of these registers has 32 bit, respectively an offset of 4 byte, so we can map them as a 32bit struct

struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR, PLLSAICFGR, DCKCFGR1, DCKCFGR2;
};
#define RCC ((struct rcc *) 0x40023800)

struct gpio {
    volatile uint32_t MODER, OTYPER, OSPEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))

enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG};
//enum values 0,1,2,3 as per reference manual

static inline void gpio_set_mode (uint16_t pin, uint8_t mode) {
    struct gpio *gpio = GPIO(PINBANK(pin));
    int n = PINNO(pin);
    gpio->MODER &= ~(3U << (n * 2));
    // clear existing setting
    gpio->MODER |= (mode & 3) << (n * 2);
    // set new mode
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

int main (void){
	uint16_t led = PIN('B', 7);            // Blue LED
	RCC->AHB1ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED
  	gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
	for (;;) {
		gpio_write(led, true);
		spin(999999);
		gpio_write(led, false);
		spin(999999);
	}
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