// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved
// Changes by Larissa Pauser for STM32F746

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#define FREQ 16000000  // CPU frequency, 16 Mhz
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

struct systick {
	volatile uint32_t CSR, RVR, CVR, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)

struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR, PLLSAICFGR, DCKCFGR1, DCKCFGR2;
};
#define RCC ((struct rcc *) 0x40023800)

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->RVR = ticks - 1;
  SYSTICK->CVR = 0;	
  SYSTICK->CSR = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(14);                   // Enable SYSCFG
}

struct gpio {
	volatile uint32_t MODER, OTYPER, OSPEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))

// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode (uint16_t pin, uint8_t mode) {
    struct gpio *gpio = GPIO(PINBANK(pin));
    int n = PINNO(pin);
    RCC->AHB1ENR |= BIT(PINBANK(pin));       // Enable GPIO clock
    gpio->MODER &= ~(3U << (n * 2));
    // clear existing setting
    gpio->MODER |= (mode & 3) << (n * 2);
    // set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4)); //set the part of AFR that corresponds to pin to zeroes
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4); //write the number of the alternate function in it
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

struct uart {
	volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR;	
};
#define UART1 ((struct uart *) 0x40011000)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)


static inline void uart_init(struct uart *uart, unsigned long baud) {
  uint8_t af = 7;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins

  if (uart == UART1) RCC->APB2ENR |= BIT(4);
  if (uart == UART2) RCC->APB1ENR |= BIT(17);
  if (uart == UART3) RCC->APB1ENR |= BIT(18);

  if (uart == UART1) tx = PIN('A', 9), rx = PIN('A', 10);
  if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);
  if (uart == UART3) tx = PIN('D', 8), rx = PIN('D', 9);

  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  uart->CR1 = 0;                           // Disable this UART
  uart->BRR = FREQ / baud;                 // FREQ is a UART bus frequency
  uart->CR1 |= BIT(2) | BIT(3) | BIT(0);  // Set UE, RE, TE
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  uart->TDR = byte;
  while ((uart->ISR & BIT(7)) == 0) spin(1); //if TXE bit is set, data can be written to TDR again
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

static inline int uart_read_ready(struct uart *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready
}

static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t) (uart->RDR & 255);
}

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

int main (void){
	uint16_t led = PIN('B', 7);            // Blue LED
	RCC->AHB1ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED
  	systick_init(16000);         // Tick every 1 ms
  	gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
	uint32_t timer, period = 1312;          // Declare timer and 500ms period
	uart_init(UART3, 115200);
	for (;;) {
		if (timer_expired(&timer, period, s_ticks)) {
      		static bool on;       
      		gpio_write(led, on);  
      		on = !on;
		uart_write_buf(UART3, "ho\r\n", 4);  // Write message             
		}
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
//SysTick Interrupt has index 16
__attribute__((section(".vectors"))) void (*const tab[16 + 98])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};