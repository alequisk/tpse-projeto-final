#include "bbb_regs.h"
#include "hw_types.h"

bool flag_timer = true;

#define CM_BASE 0x44E10000
#define GPMC_A12 0x830  // gpio1_12

#define INTC_MIR_CLEAR3 0xE8

#define GPIO1_BASE 0x4804C000
#define GPIO_DATAIN 0x138
#define GPIO_IRQSTATUS_0 0x2C
#define GPIO_IRQSTATUS_SET_0 0x34
#define GPIO_IRQSTATUS_CLEAR_0 0x3C
#define GPIO1_DATAIN (GPIO1_BASE + GPIO_DATAIN)

#define GPIO_RISINGDETECT 0x148

#define GPMC_AD14 0x838

#define SENSOR_1 12
#define SENSOR_2 14

unsigned int currentCount;

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

void delaySync(unsigned int msec) {
  while (msec != 0) {
    DMTimerWaitForWrite(0x2);
    HWREG(DMTIMER_TCRR) = 0;

    timerEnable();
    while (HWREG(DMTIMER_TCRR) < TIMER_1MS_COUNT)
      ;
    msec--;
    timerDisable();
  }
}

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
  HWREG(INTC_MIR_CLEAR2) |= (1 << 31);  //(95 --> Bit 31 do 3º registrador (MIR CLEAR2))
}

void gpioSetup() {
  /* set clock for GPIO1, TRM 8.1.12.1.31 */
  HWREG(CM_PER_GPIO1_CLKCTRL) = 0x40002;

  HWREG(CM_BASE + GPMC_A12) |= 0x7;
  HWREG(CM_BASE + GPMC_AD14) |= 0x7;
  HWREG(CM_BASE + GPMC_A12) &= ~(0b11 << 3);   // active pull up/down and pull down select
  HWREG(CM_BASE + GPMC_AD14) &= ~(0b11 << 3);  // active pull up/down and pull down select

  HWREG(GPIO1_OE) |= (1 << SENSOR_1);
  HWREG(GPIO1_OE) |= (1 << SENSOR_2);

  HWREG(GPIO1_OE) &= ~(1 << 21);  // led

  // setup interruption
  HWREG(GPIO1_BASE + GPIO_IRQSTATUS_SET_0) |= (1 << SENSOR_1) | (1 << SENSOR_2);
  HWREG(INTC_BASE + INTC_MIR_CLEAR3) |= (1 << 2);

  HWREG(GPIO1_BASE + GPIO_RISINGDETECT) |= (1 << SENSOR_1) | (1 << SENSOR_2);
}

void Timer_IRQHandler(void) {
  /* Clear the status of the interrupt flags */
  HWREG(DMTIMER_IRQSTATUS) = 0x2;

  flag_timer = true;

  /* Stop the DMTimer */
  timerDisable();
}

void Gpio1A_IRQHandler() {
  unsigned int data = HWREG(GPIO1_DATAIN);

  if (data & (1 << SENSOR_1)) {
    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_CLEAR_0) |= (1 << SENSOR_2);  // disable sensor 2

    data = HWREG(GPIO1_DATAIN);
    bool ok = false;

    while (data & (1 << SENSOR_1)) {
      data = HWREG(GPIO1_DATAIN);
      if (data & (1 << SENSOR_2)) {
        ok = true;
      }
    }

    if (ok) {
      putString("Saiu alguem\n\r", 14);

      if (currentCount > 0)
        currentCount--;

      while (data & (1 << SENSOR_2)) {
        data = HWREG(GPIO1_DATAIN);
      }
    }

    putString("SENSOR 1 foi acionado!\n\r", 25);

    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_SET_0) |= (1 << SENSOR_2);  // renable sensor 2
    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_0) |= (1 << SENSOR_1);

  } else {
    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_CLEAR_0) |= (1 << SENSOR_1);  // disable sensor 1

    data = HWREG(GPIO1_DATAIN);

    bool ok = false;

    while (data & (1 << SENSOR_2)) {
      data = HWREG(GPIO1_DATAIN);

      if ((data & (1 << SENSOR_2))) {
        ok = true;
      }
    }

    if (ok) {
      putString("Entrou alguem\n\r", 16);

      currentCount++;

      while (data & (1 << SENSOR_1)) {
        data = HWREG(GPIO1_DATAIN);
      }
    }

    putString("SENSOR 2 foi acionado!\n\r", 25);

    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_SET_0) |= (1 << SENSOR_1);  // renable sensor 1
    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_0) |= (1 << SENSOR_2);
  }
}

void ISR_Handler(void) {
  /* Verifica se é interrupção do DMTIMER7 */
  unsigned int irq_number = HWREG(INTC_SIR_IRQ) & 0x7f;

  if (irq_number == 95) Timer_IRQHandler();
  if (irq_number == 98) Gpio1A_IRQHandler();

  /* Reconhece a IRQ */
  HWREG(INTC_CONTROL) = 0x1;
}

void delay2(int s) {
  for (int k = 0; k < s; k++)
    for (int i = 0; i < 1258291; i++)
      ;
}

unsigned int toString(unsigned int value, char *buff) {
  unsigned int len = 0;
  while (value > 0u) {
    buff[len++] = '0' + (value % 10);
    value /= 10u;
  }
  if (len == 0) buff[len++] = '0';

  for (int i = 0; i < len / 2; i++) {
    char temp = buff[i];
    buff[i] = buff[len - i - 1];
    buff[len - i - 1] = temp;
  }
  return len;
}

void showTotal() {
  char buff[128];
  unsigned int len;
  putString("Current peoples: ", 18);
  len = toString(currentCount, buff);
  putString(buff, len);

  putString(" | Sensor 1: ", 13);
  if (HWREG(GPIO1_DATAIN) & (1 << SENSOR_1)) {
    putString("1 | Sensor 2: ", 15);
  } else {
    putString("0 | Sensor 2: ", 15);
  }

  if (HWREG(GPIO1_DATAIN) & (1 << SENSOR_2)) {
    putString("1\n\r", 4);
  } else {
    putString("0\n\r", 4);
  }
}

int main(void) {
  /* Hardware setup */

  gpioSetup();
  timerSetup();
  disableWdt();

  putString("Waiting 1 minute to setup sensor\n\r", 35);

  char buff[128];
  unsigned int len;

  currentCount = 0U;
  goto skip;

  for (int i = 0; i < 60; i++) {
    delaySync(1000);
    len = toString(i + 1, buff);
    putString(buff, len);
    putString("\n\r", 2);
  }

skip:

  putString("Sensor is ready!\n\r", 19);

  while (true) {
    if (currentCount > 0u) {
      HWREG(GPIO1_SETDATAOUT) |= 21;
    } else {
      HWREG(GPIO1_CLEARDATAOUT) |= 21;
    }

    showTotal();
  }

  return (0);
}
