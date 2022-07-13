#include "bbb_regs.h"
#include "hw_types.h"

//#define DEBUG 1

bool flag_timer = true;

// basic address
#define CM_BASE 0x44E10000
#define GPIO1_BASE 0x4804C000

// Gpio offset
#define GPIO_IRQSTATUS_RAW_0 0x24
#define GPIO_DATAIN 0x138
#define GPIO_IRQSTATUS_0 0x2C
#define GPIO_IRQSTATUS_SET_0 0x34
#define GPIO_IRQSTATUS_CLEAR_0 0x3C
#define GPIO_RISINGDETECT 0x148
#define GPIO_FALLINGDETECT 0x14C

#define GPIO1_DATAIN (GPIO1_BASE + GPIO_DATAIN)
#define INTC_MIR_CLEAR3 0xE8

// gpio defines pin out
#define GPMC_AD12 0x830  // gpio1_12
#define GPMC_AD14 0x838  // gpio1_14
#define GPMC_A5 0x854    // USR0

#define SENSOR_1 12  // pin sensor 1
#define SENSOR_2 14  // pin sensor 2

unsigned int currentCount;

// template function
void showTotal();

#define DMTimerWaitForWrite(reg)        \
  if (HWREG(DMTIMER_TSICR) & 0x4)       \
    while (((reg)&HWREG(DMTIMER_TWPS))) \
      ;

void disableWdt(void) {
  HWREG(WDT_WSPR) = 0xAAAA;
  while ((HWREG(WDT_WWPS) & (1 << 4)))
    ;
  HWREG(WDT_WSPR) = 0x5555;
  while ((HWREG(WDT_WWPS) & (1 << 4)))
    ;
}

/*======== start UART ========*/
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
  for (int i = 0; i < length; i++)
    putCh(str[i]);
  return (length);
}

int getString(char *buf, unsigned int length) {
  for (int i = 0; i < length; i++)
    buf[i] = getCh();
  return (length);
}
/*======== end UART ========*/

/*======== start TIMER ========*/
void timerEnable() {
  DMTimerWaitForWrite(0x1);    // Wait for previous write to complete in TCLR
  HWREG(DMTIMER_TCLR) |= 0x1;  // Start the timer
}

void timerDisable() {
  DMTimerWaitForWrite(0x1);       // Wait for previous write to complete in TCLR
  HWREG(DMTIMER_TCLR) &= ~(0x1);  // End the timer
}

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
  // TODO: verify if mSec generate a negative subtract (is up to overflow timer)
  unsigned int countVal = TIMER_OVERFLOW - (mSec * TIMER_1MS_COUNT);
  DMTimerWaitForWrite(0x2);        // Wait for previous write to complete
  HWREG(DMTIMER_TCRR) = countVal;  // Load the register with the re-load value

  flag_timer = false;  // restarting flag

  HWREG(DMTIMER_IRQENABLE_SET) = 0x2;  // Enable the DMTimer interrupts
  timerEnable();                       // Start the DMTimer
  HWREG(DMTIMER_IRQENABLE_CLR) = 0x2;  // Disable the DMTimer interrupts
}

void timerSetup(void) {
  HWREG(CM_PER_TIMER7_CLKCTRL) |= 0x2;                 // Clock enable for DMTIMER7
  while ((HWREG(CM_PER_TIMER7_CLKCTRL) & 0x3) != 0x2)  // Check clock enable for DMTIMER7
    ;
  HWREG(INTC_MIR_CLEAR2) |= (1 << 31);  // Interrupt mask (INT 95)
}
/*======== end TIMER ========*/

/*======== start GPIO ========*/
typedef enum {
  OUTPUT = 0,
  INPUT = 1
} pinOE_t;

void setGpioOrientation(int pin, pinOE_t orientation) {
  if (orientation == OUTPUT)
    HWREG(GPIO1_OE) &= ~(1 << pin);
  else
    HWREG(GPIO1_OE) |= (1 << pin);
}

typedef enum {
  ENABLE = 1,
  DISABLE = 0
} interruptStatus_t;

typedef enum {
  RISINGDETECT,
  FALLINGDETECT,
  RISING_AND_FALLINGDETECT
} interruptDetection_t;

void setGpioPinInterruption(int pin, interruptStatus_t status, interruptDetection_t detection) {
  if (status == ENABLE) {
    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_SET_0) |= (1 << pin);
    if (detection == RISINGDETECT) {
      HWREG(GPIO1_BASE + GPIO_RISINGDETECT) |= (1 << pin);
      HWREG(GPIO1_BASE + GPIO_FALLINGDETECT) &= ~(1 << pin);
    } else if (detection == FALLINGDETECT) {
      HWREG(GPIO1_BASE + GPIO_FALLINGDETECT) |= (1 << pin);
      HWREG(GPIO1_BASE + GPIO_RISINGDETECT) &= ~(1 << pin);
    } else if (detection == RISING_AND_FALLINGDETECT) {
      HWREG(GPIO1_BASE + GPIO_FALLINGDETECT) |= (1 << pin);
      HWREG(GPIO1_BASE + GPIO_RISINGDETECT) |= (1 << pin);
    }
  } else {
    HWREG(GPIO1_BASE + GPIO_IRQSTATUS_SET_0) &= ~(1 << pin);
  }
}

void gpioSetup() {
  HWREG(CM_PER_GPIO1_CLKCTRL) = 0x40002;  // set clock for GPIO1

  /* setting mux to gpio mode */
  HWREG(CM_BASE + GPMC_AD12) |= 0x7;
  HWREG(CM_BASE + GPMC_AD14) |= 0x7;
  HWREG(CM_BASE + GPMC_A5) |= 0x7;

  /** active pull up/down and pull down select **/
  HWREG(CM_BASE + GPMC_AD12) &= ~(0x3 << 3);
  HWREG(CM_BASE + GPMC_AD14) &= ~(0x3 << 3);

  setGpioOrientation(SENSOR_1, INPUT);
  setGpioOrientation(SENSOR_2, INPUT);
  setGpioOrientation(21, OUTPUT);

  // setup interruption
  setGpioPinInterruption(SENSOR_1, ENABLE, RISINGDETECT);
  setGpioPinInterruption(SENSOR_2, ENABLE, RISINGDETECT);
}

/*======== end GPIO ========*/

/*======== start INTERRUPTION ========*/
void Timer_IRQHandler(void) {
  HWREG(DMTIMER_IRQSTATUS) = 0x2;  // Clear the status of the interrupt flags
  flag_timer = true;
  timerDisable();  // Stop the DMTimer
}

bool getGpioPinValue(int pin) {
  bool value = (HWREG(GPIO1_DATAIN) & (1 << pin)) > 0 ? true : false;
  return value;
}

bool getGpioInterruptionPinValue(int pin) {
  bool value = (HWREG(GPIO1_BASE + GPIO_IRQSTATUS_RAW_0) & (1 << pin)) > 0 ? true : false;
  return value;
}

void clearInterruptionFlagGpio(int pin) { HWREG(GPIO1_BASE + GPIO_IRQSTATUS_0) |= (1 << pin); }

void Gpio1A_IRQHandler() {
#ifdef DEBUG
  putString("Calling Interrupt of GPIO1A\n\r", 30);
#endif
  if (getGpioInterruptionPinValue(SENSOR_1) == true) {
#ifdef DEBUG
    putString("Sensor 1 is active\n\r", 21);
#endif
    setGpioPinInterruption(SENSOR_2, DISABLE, RISINGDETECT);  // disable interruption to sensor 2
    bool ok = false;                                          // variable to check if is passing by
    while (getGpioPinValue(SENSOR_1) == true) {
      if (getGpioPinValue(SENSOR_2) == true) {
        ok = true;  // someone enter the room, changing state of ok
      }
    }
    if (ok) {
      if (currentCount > 0u) {  // verify if count is negative
        currentCount = currentCount - 1u;
      }
      while (getGpioPinValue(SENSOR_2) == true)  // wait both to disable
        ;
      delaySync(4000);
#ifdef DEBUG
      putString("Saiu alguem\n\r", 14);
#endif
      showTotal();
    }
    clearInterruptionFlagGpio(SENSOR_1);                     // ack of gpio
    clearInterruptionFlagGpio(SENSOR_2);                     // ack of gpio
    setGpioPinInterruption(SENSOR_2, ENABLE, RISINGDETECT);  // enable gpio of sensor 2

  } else {
#ifdef DEBUG
    putString("Sensor 2 is active\n\r", 21);
#endif
    setGpioPinInterruption(SENSOR_1, DISABLE, RISINGDETECT);  // disable sensor 1
    bool ok = false;
    while (getGpioPinValue(SENSOR_2) == true) {
      if (getGpioPinValue(SENSOR_1) == true) {
        ok = true;  // someone enter the room, changing state of ok
      }
    }
    if (ok) {
      currentCount = currentCount + 1u;          // increment number of person inside room
      while (getGpioPinValue(SENSOR_1) == true)  // wait both to disable
        ;
      delaySync(4000);
#ifdef DEBUG
      putString("Entrou alguem\n\r", 16);
#endif
      showTotal();
    }
    clearInterruptionFlagGpio(SENSOR_1);  // ack of gpio
    clearInterruptionFlagGpio(SENSOR_2);  // ack of gpio
    setGpioPinInterruption(SENSOR_1, ENABLE, RISINGDETECT);
  }
}

void ISR_Handler(void) {
  unsigned int irq_number = HWREG(INTC_SIR_IRQ) & 0x7f;
  if (irq_number == 95) Timer_IRQHandler();
  if (irq_number == 98) Gpio1A_IRQHandler();
  HWREG(INTC_CONTROL) = 0x1;  // Reconhece a IRQ
}

void enableGpioInterruption() {
  HWREG(INTC_BASE + INTC_MIR_CLEAR3) |= (1 << 2);
}

/*======== end INTERRUPTION ========*/

/*======== start MISC ========*/

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
#ifdef DEBUG
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

#else
  putString("\n\r", 2);
#endif
}

/*======== end MISC ========*/

int main(void) {
  /* Hardware setup */
  gpioSetup();
  timerSetup();
  disableWdt();

  putString("Waiting 1 minute to setup sensor\n\r", 35);

  currentCount = 0u;

#ifdef DEBUG
  goto skip;
#endif
  for (int i = 0; i < 60; i++) {
    delaySync(1000);
  }

#ifdef DEBUG
skip:
#endif

  enableGpioInterruption();
  putString("Sensor is ready!\n\r", 19);

  while (true) {
    if (currentCount > 0u) {
      HWREG(GPIO1_SETDATAOUT) |= (1 << 21);
    } else {
      HWREG(GPIO1_CLEARDATAOUT) |= (1 << 21);
    }
  }

  return (0);
}
