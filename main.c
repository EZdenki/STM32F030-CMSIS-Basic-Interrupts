//  ==========================================================================================
//  main.c for STM32F030-CMSIS-Basic-Interrupts
//  ------------------------------------------------------------------------------------------
//  Examples of how to set up basic interrupt routines including triggers based on button
//  presses, timer-overflow, and the SysTick system clock.
//  ------------------------------------------------------------------------------------------
//  https://github.com/EZdenki/STM32F030-CMSIS-Basic-Interrupts
//  Released under the MIT License
//  Copyright (c) 2023
//  Mike Shegedin, EZdenki.com
//  Version 1.0   26 Aug 2023    Started
//  ------------------------------------------------------------------------------------------
//  Target Microcontroller and Devices:
//    * STM32F030Fxxx
//    * Status LED and current limiting resistor on pin 2-PF0
//    * Tactile buttons on pin 6 (PA0) and pin 7 (PA1)
//    
//  ------------------------------------------------------------------------------------------
//  Hardware Setup:
//
//                                        STM32F030F4xx     
//                                         ╭────╮╭────╮
//                                   BOOT0 │1       20│ SWCLK -- [SWCLK│ST-Link V2]
//    GND -- [1K] -- [-Status LED+] -- PF0 │2       19│ SWCLK -- [SWDIO│          ]
//                                     PF1 │3       18│ PA10
//                                    NRST │4       17│ PA9
//                                    VDDA │5 ----- 16│ VCC -- VCC
//                GND -- [Button 1] -- PA0 │6       15│ GND -- GND
//                GND -- [Button 2] -- PA1 │7       14│ PB1
//                                     PA2 │8       13│ PA7
//                                     PA3 │9       12│ PA6
//                                     PA4 │10      11│ PA5
//                                         ╰──────────╯
//
//  ==========================================================================================

#include "stm32f030x6.h"

//  ==========================================================================================
//  Interrupt Defines
//  Comment out the define to set up and run the desired type of interrupt. Multiple
//  interrupts may be set up simultaneously.
//
//  __BUTTON_INTERRUPT:
//    Interrupt generated on the rising edge of PA0 and PA1. Pressing and releasing the button
//    on PA0 will cause EXTI0_1_IRQHandler to be called and will turn ON the LED attached to
//    PF0. Likewise, pressing and releasing the button on PA1 will cause EXTI0_1_IRQHandler
//    to be called and will turn OFF the LED.
//    Note that EXTI0 and EXTI1 both result in a call to the same interrupt handler. The code
//    inside the handler must determine which line actually caused the interrupt by checking
//    the EXTI->PR register for the PR0 bit (for PA0) or PR1 bit (for PA1).
//
//  __TIMER_INTERRUPT
//    Interrupt generated each time that TIM14 overflows.
//
//  __SYSTICK_INTERRUPT
//    Interrupt generated each time that x clock cycles have occurred, where x is a 24-bit
//    number initialized in the SysTick_Config( x ) procedure. On an 8 MHz clock, the
//    slowest time between calls to the interrupt handler is approx. 2 seconds when using a
//    value of (uint32_t)16E6 for x.
//  ==========================================================================================

 #define __BUTTON_INTERRUPT
// #define __TIMER_INTERRUPT
// #define __SYSTICK_INTERRUPT


#ifdef __BUTTON_INTERRUPT
//  ------------------------------------------------------------------------------------------
//  EXTI0_1_IRQHandler
//  ------------------------------------------------------------------------------------------
// void EXTI0_1_IRQHandler( void )
// This is the interrupt handler for external interrupt lines 0 and 1, so it is called when
// either of these lines generate an interrupt. You likely want to use the Pending Register
// to confirm which line generated the interrupt and act accordingly. Be sure to clear the
// interrupt by setting the bit in the Pending Register to effectively clear it.

void
EXTI0_1_IRQHandler( void )
{
  if( EXTI->PR & EXTI_PR_PR0 )      // If detected rising edge on PA0
  {
    EXTI->PR |= EXTI_PR_PR0;        // Clear the interrupt by *setting* the Pending Reg. bit
    GPIOF->ODR |= GPIO_ODR_0;       // Turn ON LED on PF0
  }
  else
    if( EXTI->PR & EXTI_PR_PR1 )    // If rising edge detected on PA1,
    {
      EXTI->PR |= EXTI_PR_PR1;      // Clear the interrupt by *setting* the Pending Reg. bit
      GPIOF->ODR &= ~GPIO_ODR_0;    // Turn OFF LED on PF0
    }
}
#endif // __BUTTON_INTERRUPT


#ifdef __TIMER_INTERRUPT
//  ------------------------------------------------------------------------------------------
//  TIM14_IRQHandler
//  ------------------------------------------------------------------------------------------
// void TIM14_IRQHandler( void )
// This function will be called when the TIM14 interrupt is generated. Note that the function
// must clear the TIM14 interrupt flag (TIM_SR_UIF) everytime so that the next interrupt will
// be recognized.
// The name of the TIM14_IRQHandler and other interrupt handler function names to use are
// defined in STM32CubeF0\Core_Startup\Startup_stm32f030f4px.s 

void
TIM14_IRQHandler( void )
{
  GPIOF->ODR ^= GPIO_ODR_0;
  TIM14->SR &= ~TIM_SR_UIF;
}
#endif // __TIMER_INTERRUPT


#ifdef __SYSTICK_INTERRUPT
//  ------------------------------------------------------------------------------------------
//  SysTick_Handler
//  ------------------------------------------------------------------------------------------
// void SysTick_Handler( void )
// This function will be called when the SysTick timer overflows. The SysTick timer must
// be initialized by calling the SysTick_Config( x ), where x is the number of clock ticks.
// Note that x is a 24-bit number, meaning that with the 8 MHz internal clock, the longest
// time that can be set is approx 16E6, or a time of 2 seconds.

void
SysTick_Handler( void )
{
  GPIOF->ODR ^= GPIO_ODR_0;
}
#endif // __SYSTICK_INTERRUPT



//  ==========================================================================================
//  main
//  ==========================================================================================

int
main( void )
{

//  ------------------------------------------------------------------------------------------
//  Set up GPIO pins as inputs and outputs as required
//  ------------------------------------------------------------------------------------------

  // Set up GPIO PA0 and PA1 as inputs with pullups
  RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;                 // Enable GPIO Port A
  GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPDR0_Pos) |   // Set pullups on PA0 and PA1
                  (0b01 << GPIO_PUPDR_PUPDR1_Pos);
  
  // Set up GPIO PF0 (pin 2) for output for LED
  RCC->AHBENR  |= RCC_AHBENR_GPIOFEN;                 // Enable GPIOF
  GPIOF->MODER |= ( 0b01 << GPIO_MODER_MODER0_Pos );  // Set PF0 as output

#ifdef __BUTTON_INTERRUPT
//  ------------------------------------------------------------------------------------------
//  Configure GPIO pins as interrupt triggers
//  ------------------------------------------------------------------------------------------

//  To configure a line as interrupt source, use the following procedure:
//  1 Unmask the desired interrupt by setting the bit in the EXTI_IMR register.
//  2 Configure the Trigger Selection bits of the Interrupt line (EXTI_RTSR and EXTI_FTSR)
//    so that the interrupt is triggered on the rising and/or falling edge of this line.
//  3 Configure the enable and mask bits that control the NVIC IRQ channel mapped to the
//    EXTI so that an interrupt coming from one of the EXTI line can be correctly
//    acknowledged.
//  4 Set the priority of this interrupt

  EXTI->IMR  |= EXTI_IMR_MR0 | EXTI_IMR_MR1;    // Unmask EXTI interrupts on lines 0 and 1
  EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1;  // Trigger on rising edge of lines 0 and 1
  NVIC_EnableIRQ( EXTI0_1_IRQn );               // Enable this interrupt
  NVIC_SetPriority( EXTI0_1_IRQn, 0 );          // Set the desired priority (0 = highest)
#endif // __BUTTON_INTERRUPT


#ifdef __TIMER_INTERRUPT
//  ------------------------------------------------------------------------------------------
//  Configure TIM14 as interrupt trigger
//  ------------------------------------------------------------------------------------------

//  To set a timer as an interrupt trigger, use the following procedure:
//  1 Set up the timer normally to overflow at the desired rate.
//  2 Enable the UIE bit in the DIER register of the timer to have an interrupt triggered when
//    the timer overflows.
//  3 Enable the NVIC TIMx_IRQn
//  4 Set the NVIC TIMx_IRQn priority as needed

  // Set up TIM14 for 1 ms clock pulse, and to turnover and toggle the interrupt
  // every 100 ms.
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;  // Enable TIM14
  TIM14->PSC    = 8000-1;               // Set prescaler to x8000 for 1 ms clock
  TIM14->ARR    = 500-1;                // Set Auto Reload Register to 500 (ms)
  TIM14->CR1   |= TIM_CR1_CEN;          // Start the timer
  TIM14->DIER  |= TIM_DIER_UIE;         // Have TIM14 generate interrupt when it overflows

  NVIC_EnableIRQ( TIM14_IRQn );         // Enable TIM14_IRQn
  NVIC_SetPriority( TIM14_IRQn, 3);     // Set priority for TIM14_IRQn
#endif // __TIMER_INTERRUPT


#ifdef __SYSTICK_INTERRUPT
//  ------------------------------------------------------------------------------------------
//  Configure SysTick as interrupt trigger
//  ------------------------------------------------------------------------------------------

//  To set the SysTick as an interrupt, use the following procedure:
//    Call SysTick_Config with a 24-bit value which is the number of clock ticks between
//    calls to the interrupt handler.
//    Note that a call to NVIC_EnableIRQ( SysTick_IRQn ) is not required.
  SysTick_Config( (uint32_t)16E6 );       // Configure the number of clock ticks between calls
                                          // to the SysTick interrupt handler.
  NVIC_SetPriority( SysTick_IRQn, 0 );    // Set the desired priority of the SysTick interrupt
                                          // though it defaults to 3, which is pretty high
#endif // __SYSTICK_INTERRUPT


  while( 1 ) ;                          // Do nothing!


} // End of main()
