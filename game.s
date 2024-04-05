# PASTE LINK TO TEAM VIDEO BELOW
#
#

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 250
  .equ    NOTE_LENGTH, notesEnd-notes
  .section .text

Main:
  PUSH  {R4-R5,LR}


  @
  @ Prepare GPIO Port E Pin 9 for output (LED LD3)
  @ We'll blink LED LD3 (the orange LED)
  @

  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ Configure LD3~10 for output
  @   by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @   (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                      @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD4_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD4_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD5_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD5_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD6_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD6_PIN*2))      @ write 01 to bits 8
  BIC     R5, #(0b11<<(LD7_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD7_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD8_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD8_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD9_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD9_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD10_PIN*2))     @ Modify ...
  ORR     R5, #(0b01<<(LD10_PIN*2))     @ write 01 to bits 
  STR     R5, [R4]                      @ Write 

  @ Initialise the first countdown

  LDR     R4, =blink_count
  LDR     R5, =0
  STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1
  @ initialize delayTime to be 400
  LDR     R4, =delayTime
  LDR     R5, =#400
  STR     R5, [R4]
  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt Line0
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set rising edge detection on Line0
  LDR     R4, =EXTI_RTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt #6 (external interrupt Line0)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
.LgameOver:
  LDR     R4, =GPIOE_ODR            @   clear LEDs;
  LDR     R5, [R4]
  BIC     R5, R5, 0x0000FF00     
  
  LDR     R4, =playerScore         @ show result;
  LDR     R6, [R4]
  CMP     R6, #1000                 @ change numbers here
  BHS     .Lphi
  CMP     R6, #800
  BHS     .Lv 
  CMP     R6, #400
  BHS     .Ls 
  CMP     R6, #200
  BHS     .La 
  B       .Lb
.Lphi:
  ORR     R5, R5, 0x0000FF00
  B       .LendResult
.Lv:
  ORR     R5, R5, 0x0000FE00
  B       .LendResult
.Ls:
  ORR     R5, R5, 0x0000FC00
  B       .LendResult
.La:
  ORR     R5, R5, 0x0000F800
  B       .LendResult
.Lb:
  ORR     R5, R5, 0x0000F000
  B       .LendResult

.LendResult:
  LDR     R4, = GPIOE_ODR 
  STR     R5, [R4]
End_Main:
  POP   {R4-R5,PC}



@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4-R7, LR}
  LDR   R4, = failureCount
  LDR   R5, [R4]
  CMP   R5, #3
  BHS   .LgameOver
  LDR   R4, = delayTime
  LDR   R6, [R4]              @delayTime in R6
  LDR   R4, =blink_count        @ if (timeCount < delayTime) {
  LDR   R5, [R4]                    @
  CMP   R5, R6                      @
  BHS   .LelseFire                  @

  ADD   R5, R5, #1                  @   timeCount = timeCount + 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

 @ LDR     R4, =GPIOE_ODR            @   
 @ LDR     R5, [R4]                  @
 @ EOR     R5, #(0b1<<(LD3_PIN))     @   
 @ STR     R5, [R4]                  @ 
  LDR     R4, =hasNote               @  if(hasNote) failureCount++;
  LDR     R5, [R4]
  CMP     R5, #0
  BEQ     .LnoMiss
  LDR     R4, =failureCount
  LDR     R5, [R4]
  ADD     R5, R5, #1
  STR     R5, [R4]
.LnoMiss:
  LDR     R4, =notes
  LDR     R5, =noteIndex
  LDR     R6, [R5]                  @   
  LDRB    R6, [R4, R6]               @   read the notes at address notes+index;
  MOV     R5, #0
  CMP     R6, 0x80                  @   if (LD6 is going to be turned on)   hasNote=true (1);
  BLO     .LnoNote                  @   else hasNote=false(0);
  MOV     R5, #1
.LnoNote:
  LDR     R4, = hasNote
  STR     R5, [R4]
  MOV     R6, R6, LSL#8
  LDR     R4, =GPIOE_ODR            @   clear LEDs;
  LDR     R5, [R4]
  BIC     R5, R5, 0x0000FF00     
  ORR     R5, R5, R6                @   set new LED status;
  STR     R5, [R4]


  LDR     R4, = noteIndex
  LDR     R5, [R4]
  LDR     R6, =NOTE_LENGTH
  CMP     R5, R6                    @ if (index<noteLength)
  BHS     .LrestartSong
  ADD     R5, R5, #1                @   index++;
  B       .LendIfRestart
.LrestartSong:
  MOV     R5, #0                    @ else index=0;
  LDR     R6, =delayTime
  LDR     R7, [R6]
  SUB     R7, R7, #10
  STR     R7, [R6]
.LendIfRestart:
  STR     R5, [R4]
  LDR     R4, =blink_count          @   timeCount=0;
  LDR     R5, =0                    @
  STR     R5, [R4]                  @

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4-R7, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:
@ When button pressed, if has note then read the time, add score, if not then add failure count
  PUSH  {R4,R5,LR}

  LDR   R4, =button_count           @ count = count + 1
  LDR   R5, [R4]                    @
  ADD   R5, R5, #1                  @
  STR   R5, [R4]                    @
  LDR     R4, =hasNote
  LDR     R5, [R4]
  CMP     R5, #0                    @  if(hasNote){
  BEQ     .LnoNoteWhenBtnPressed
  MOV     R5, #0
  STR     R5, [R4]                  @   hasNote=false;

  LDR     R4, =blink_count
  LDR     R5, [R4]                  @   get time difference;
  LDR     R4, =delayTime
  LDR     R4, [R4]
  MOV     R4, R4, LSR#1             @   get delay/2;
  CMP     R5, R4
  BHI     .Lgood                    @   if (timeDifference<delay/2)  score+=20;
  LDR     R4, =playerScore          @   else score+=10;
  LDR     R5, [R4]
  ADD     R5, R5, #20
  STR     R5, [R4]
  B       .LendJudgement
.Lgood:
  LDR     R4, =playerScore
  LDR     R5, [R4]
  ADD     R5, R5, #10
  STR     R5, [R4]
.LendJudgement:

  LDR     R4, =GPIOE_ODR            @   Invert LD6;
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD6_PIN))     @   
  STR     R5, [R4]                  @   }

  B       .LendBtnPressed
.LnoNoteWhenBtnPressed:              @ else
  LDR     R4, =failureCount          @ {
  LDR     R5, [R4]
  ADD     R5, R5, #1                 @  failureCount+=1;
  STR     R5, [R4] 
  LDR     R4, =GPIOE_ODR 
  LDR     R5, [R4]
  ORR     R5, 0x0000FF00
  STR     R5, [R4]                   @ }
.LendBtnPressed:
  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  @ Return from interrupt handler
  POP  {R4,R5,PC}


  .section .data
  
button_count:
  .space  4

blink_count:
  .space  4
delayTime:
  .space  4
noteIndex:
  .space  4
notes:
  .byte 0b00000101, 0b00001010, 0b00010101, 0b00101010
  .byte 0b01010100, 0b10101000, 0b01010000, 0b10100001
  .byte 0b01000010, 0b10000101, 0b00001010, 0b00010101 
  .byte 0b00101011, 0b01010111, 0b10101110, 0b01011100 
  .byte 0b10111000, 0b01110000, 0b11100000, 0b11000000
  .byte 0b10000000
notesEnd:
hasNote:
  .space  4
playerScore:
  .space  4
failureCount:
  .space  4
  .end