#include "bbb_regs.h"
#include "hw_types.h"

bool flag_timer = true;

#define DMTimerWaitForWrite(reg)        \
  if (HWREG(DMTIMER_TSICR) & 0x4)       \
    while ((reg & HWREG(DMTIMER_TWPS))) \
      ;

void disableWdt(void) {
  HWREG(WDT_WSPR) = 0xAAAA;
  while ((HWREG(WDT_WWPS) & (1 << 4)))
    ;

  HWREG(WDT_WSPR) = 0x5555;
  while ((HWREG(WDT_WWPS) & (1 << 4)))
    ;
}

void putCh(char c) {
  while (!(HWREG(UART0_LSR) & (1 << 5)))
    ;

  HWREG(UART0_THR) = c;
}

char getCh() {
  while (!(HWREG(UART0_LSR) & (1 << 0)))
    ;

  return (HWREG(UART0_RHR));
}

int putString(char *str, unsigned int length) {
  for (int i = 0; i < length; i++) {
    putCh(str[i]);
  }
  return (length);
}

int getString(char *buf, unsigned int length) {
  for (int i = 0; i < length; i++) {
    buf[i] = getCh();
  }
  return (length);
}

void timerEnable() {
  /* Wait for previous write to complete in TCLR */
  DMTimerWaitForWrite(0x1);

  /* Start the timer */
  HWREG(DMTIMER_TCLR) |= 0x1;
} /* -----  end of function timerEnable  ----- */

void timerDisable() {
  /* Wait for previous write to complete in TCLR */
  DMTimerWaitForWrite(0x1);

  /* Start the timer */
  HWREG(DMTIMER_TCLR) &= ~(0x1);
} /* -----  end of function timerEnable  ----- */

void delay(unsigned int mSec) {
  unsigned int countVal = TIMER_OVERFLOW - (mSec * TIMER_1MS_COUNT);

  /* Wait for previous write to complete */
  DMTimerWaitForWrite(0x2);

  /* Load the register with the re-load value */
  HWREG(DMTIMER_TCRR) = countVal;

  flag_timer = false;

  /* Enable the DMTimer interrupts */
  HWREG(DMTIMER_IRQENABLE_SET) = 0x2;

  /* Start the DMTimer */
  timerEnable();

  while (flag_timer == false)
    ;

  /* Disable the DMTimer interrupts */
  HWREG(DMTIMER_IRQENABLE_CLR) = 0x2;
}

void timerSetup(void) {
  /*  Clock enable for DMTIMER7 TRM 8.1.12.1.25 */
  HWREG(CM_PER_TIMER7_CLKCTRL) |= 0x2;

  /*  Check clock enable for DMTIMER7 TRM 8.1.12.1.25 */
  while ((HWREG(CM_PER_TIMER7_CLKCTRL) & 0x3) != 0x2)
    ;

  /* Interrupt mask */
  HWREG(INTC_MIR_CLEAR2) |=
      (1 << 31);  //(95 --> Bit 31 do 3º registrador (MIR CLEAR2))
}

/*
 * ===  FUNCTION
 * ====================================================================== Name:
 * gpioSetup Description:
 * =====================================================================================
 */
void gpioSetup() {
  /* set clock for GPIO1, TRM 8.1.12.1.31 */
  HWREG(CM_PER_GPIO1_CLKCTRL) = 0x40002;

  // TODO: Mux of GPIO to use
}

void Timer_IRQHandler(void) {
  /* Clear the status of the interrupt flags */
  HWREG(DMTIMER_IRQSTATUS) = 0x2;

  flag_timer = true;

  /* Stop the DMTimer */
  timerDisable();
}

void ISR_Handler(void) {
  /* Verifica se é interrupção do DMTIMER7 */
  unsigned int irq_number = HWREG(INTC_SIR_IRQ) & 0x7f;

  if (irq_number == 95) Timer_IRQHandler();

  /* Reconhece a IRQ */
  HWREG(INTC_CONTROL) = 0x1;
}

int main(void) {
  /* Hardware setup */
  gpioSetup();
  timerSetup();
  disableWdt();

  putString("Hello World\n\r", 14);

  return (0);
}
