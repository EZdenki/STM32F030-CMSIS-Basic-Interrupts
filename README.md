# STM32F030-CMSIS-Basic-Interrupts
Examples of how to set up basic interrupt routines including triggers based on button presses,
timer overflow, or the SysTick (system clock).

## Interrupt Types
The following types of interrupts are defined. They can be enabled or disabled by commenting
out the relevant #define statements near the top of ```main.c```. Multiple interrupts may be set up simultaneously.

* __BUTTON_INTERRUPT:<br>
  Interrupt generated on the rising edge of PA0, PA1, and PA2. Pressing and releasing a
  button will result in a rising edge and trigger one of the following:
    - PA0: Will call EXTI0_1_IRQHandler while setting the PR0; Turn ON PA3 LED.
    - PA1: Will call EXTI0_1_IRQHandler while setting the PR1; Turn OFF PA3 LED.
    - PA2: Will call EXTI2_3_IRQHandler while setting the PR2: Toggle PA3 LED.
  Note that EXTI0 and EXTI1 both result in a call to EXTI0_1_IRQHandler. The code inside
  the handler must determine which line actually caused the interrupt by checking the
  EXTI->PR register for the PR0 bit (for PA0) or PR1 bit (for PA1). Likewise for EXTI2
  and EXTI3 calling EXTI2_3_IRQHandler.

* __TIMER_INTERRUPT<br>
  TIM14 is set up to overflow at a certain perdiod. Each time the timer overflows, an
  interrupt is generated which calls TIM14_IRQHandler. This handler toggles the PA4 LED.

* __SYSTICK_INTERRUPT
  Once the SysTick interrupt is initialized, an interrupt is generated each time that x
  clock cycles have occurred, where x is a 24-bit number initialized in the
  SysTick_Config( x ) procedure. On an 8 MHz clock, the slowest time between calls to the
  interrupt handler is approx. 2 seconds when using a value of (uint32_t)16E6 for x.

## Hardware Setup
```
                                                          ST-Link V2 
                                  STM32F030F4xx           ╭───────╮     
                                   ╭────╮╭────╮           │    GND├───────╮
                             BOOT0 │1       20│ SWCLK ────┤SWCLK  │       │
                               PF0 │2       19│ SWCLK ────┤SWDIO  │       │
                               PF1 │3       18│ PA10      │   3.3V├───╮   │
                              NRST │4       17│ PA9       ╰───────╯   │   │
                              VDDA │5 ───── 16│ VCC ──────── VCC ─────╯   │
          GND ── [Button 1] ── PA0 │6       15│ GND ──────── GND ─────────╯
          GND ── [Button 2] ── PA1 │7       14│ PB1
          GND ── [Button 2] ── PA2 │8       13│ PA7
       GND ── [1K] ── [LED] ── PA3 │9       12│ PA6
       GND ── [1K] ── [LED] ── PA4 │10      11│ PA5 ── [1K] ── [LED] ── GND 
                                   ╰──────────╯
```

### See ```main.c``` for additional details
