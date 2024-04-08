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

  .equ    BLINK_PERIOD, 300

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

  @ Configure LD3-LD10 for output
  @ by setting bits corresponding bits of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @ (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                      @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD4_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD4_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD5_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD5_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD6_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD6_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD7_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD7_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD8_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD8_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD9_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD9_PIN*2))      @ write 01 to bits 
  BIC     R5, #(0b11<<(LD10_PIN*2))     @ Modify ...
  ORR     R5, #(0b01<<(LD10_PIN*2))     @ write 01 to bits 
  STR     R5, [R4]                      @ Write 

  MOV     R6, #0b00000001               @ Mask for first LED

  @ Initialise the first countdown

  LDR     R4, =blink_countdown
  LDR     R5, =BLINK_PERIOD
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




  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  LDR   R4, =good_count
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]
  LDR   R4, =missedHit_count
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]
  LDR   R4, =isJudgmentPhase
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]
  LDR   R4, =perfect_count
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]
  LDR   R4, = sheetMusicReadAddress
  LDR   R5, = sheetMusic
  @LDR   R5, = sheetMusicSecondLevel
  STR   R5, [R4]
  LDR   R4, =nextLevelIndicator
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]

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

  @ Set falling edge detection on Line0
  LDR     R4, =EXTI_FTSR
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
  LDR     R8, =BLINK_PERIOD
  LDR     R9, [R8]
  B     Idle_Loop
  
End_Main:
  POP   {R4-R5,PC}



@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4-R5, LR}

  @Executing game over evaluation
  LDR   R4, = missedHit_count
  LDR   R5, [R4]
  CMP   R5, #5
  BGT   .LgameOver
  B     .Lgaming
.LgameOver:
  BL    GameOver

.Lgaming:

  LDR   R4, =blink_countdown        @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

  @MOV   R0, R6
  @BL    InvertNextLED                @   Invert next LED
  @MOV   R6, R0
@-------------------------------------------------------
  LDR     R4, =nextLevelIndicator
  LDR     R5, [R4]
  CMP     R5, #2
  BEQ     .LReadFromSheetMusic2   
  B     .LReadFromSheetMusic1    
.LReadFromSheetMusic2:   
  BL    level2SheetMusic 
  B     .LeReadFromSheetMusic     
.LReadFromSheetMusic1:
  BL    level1SheetMusic
.LeReadFromSheetMusic:

  @LDR     R4, =GPIOE_ODR            @   Invert LD3
  @LDR     R5, [R4]                  @
  @EOR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  @STR     R5, [R4]                  @ 

  LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD         @
  STR     R5, [R4]                  @

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4-R5, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4-R6,LR}

  LDR   R4, =button_count           @ count = count + 1
  LDR   R5, [R4]                    @
  ADD   R5, R5, #1                  @
  STR   R5, [R4]                    @

  BL    JudgementLevels

@Next level------------------------------------------
  LDR   R4, =button_count           @
  LDR   R5, [R4]                    @

  LDR   R4, =nextLevelIndicator     @
  LDR   R6, [R4]                    @

  CMP  R5, #2
  BNE  .LnotNextLevel
  CMP  R6, #1
  BNE  .LnotNextLevel

  LDR     R4, =nextLevelIndicator
  LDR     R5, =0x2                    
  STR     R5, [R4]

.LnotNextLevel:
@--------------------------------------------------

  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  @ Return from interrupt handler
  POP  {R4-R6,PC}


@ InvertNextLED subroutine
@ Invert next LED by inverting bit of next LED of GPIOE_ODR (GPIO Port E Output Data Register)
@ (by using EOR to invert bit of next LED, leaving other bits unchanged)
@ Paramaters:
@   R0: mask 
@
@ Return:
@   R0: updated mask
InvertNextLED:
  PUSH {R4-R6, LR}
  MOV R6, R0
  LDR R4, =GPIOE_ODR
  LDR R5, [R4]
  EOR R5, R5, R6, LSL #(LD4_PIN)
  STR R5, [R4]

  CMP R6, #0b10000000
  BEQ .LelseLastLED
  MOV R6, R6, LSL #1
  B   .LendIfNotLastLED

.LelseLastLED:
  MOV R6, #0b00000001
.LendIfNotLastLED:
  MOV R0, R6

  POP {R4-R6, PC}

@ isInJudgmentPhase subroutine
@ Checking whether note falls on judgment line by checking the output state of LED 3
@ Paramaters:
@ null
@
@ Return:
@ null
isInJudgmentPhase:
  PUSH {R4-R5, LR}
  LDR R4, =GPIOE_ODR
  LDR R5, [R4]
  BIC R5, R5, #0xFFFF7FFF
  CMP R5,#0x8000
  BEQ .LisJudgmentPhase
  B   .LnotJudgmentPhase
.LisJudgmentPhase:
  MOV R5, #1
  LDR R4, =isJudgmentPhase
  STR R5, [R4]
  B   .LeJudgment
.LnotJudgmentPhase:
  MOV R5, #0
  LDR R4, =isJudgmentPhase
  STR R5, [R4]
.LeJudgment:
  POP {R4-R5, PC}

@ JudgementLevels subroutine
@ Judgement types base on timing of keyy pressed
@ Paramaters:
@ null
@
@ Return:
@ null
JudgementLevels:
  PUSH {R4-R5, LR}

  BL    isInJudgmentPhase
  LDR R4, =isJudgmentPhase          @ if(isJudgmentPhase)
  LDR R5, [R4]
  CMP R5, #1
  BEQ   .LisSuccessfulHit
  B     .LmissedHit

.LisSuccessfulHit:
  LDR   R4, =good_count;
  LDR   R5, [R4]
  ADD   R5, R5, #1
  STR   R5, [R4]
  B     .LeHitCount
.LmissedHit:
  LDR   R4, =missedHit_count
  LDR   R5, [R4]
  ADD   R5, R5, #1
  STR   R5, [R4]
  B     .LeHitCount
.LeHitCount:

  POP {R4-R5, PC}
  

@ GameOver subroutine
@ clear all the lights and show resulits by display LEDs
@ Paramaters:
@ null
@
@ Return:
@ null
GameOver:
  PUSH {R4-R5, LR}

  LDR     R4, =GPIOE_ODR            @   clear LEDs;
  LDR     R5, [R4]
  BIC     R5, R5, 0x0000FF00

  LDR     R4, =gameScore           @ show result;
  LDR     R6, [R4]
  CMP     R6, #30                 @ change numbers here
  BGE     .Lv
  CMP     R6, #10
  BGE     .Ls 
  CMP     R6, #0
  BGE     .La
  B       .Lb

  LDR   R4, =button_count           @
  LDR   R5, [R4]                    @
  MOV   R5, #0
  STR   R5, [R4]                    @

  LDR   R4, =nextLevelIndicator     @
  LDR   R5, [R4]                    @
  MOV   R5, #1
  STR   R5, [R4]                    @
  


.Lv:
  ORR     R5, R5, 0x0000FF00
  B       .LendResult
.Ls:
  ORR     R5, R5, 0x0000FE00
  B       .LendResult
.La:
  ORR     R5, R5, 0x0000FC00
  B       .LendResult
.Lb:
  ORR     R5, R5, 0x0000F800
  B       .LendResult

.LendResult:
  LDR     R4, = GPIOE_ODR 
  STR     R5, [R4]

  BL  ResetForLevel2SheetMusic

  POP {R4-R5, PC}


@ GameScoreCalculation subroutine
@ calculate the game score
@ Paramaters:
@ null
@
@ Return:
@ null
GameScoreCalculation:
  PUSH {R4-R6, LR}

  MOV   R6, #0
  LDR   R4, =perfect_count
  LDR   R4, [R4]
  MOV   R5, #5
  MUL   R4, R4, R5
  ADD   R6, R6, R4

  LDR   R4, =good_count
  LDR   R4, [R4]
  MOV   R5, #3
  MUL   R4, R4, R5
  ADD   R6, R6, R4

  LDR   R4, =missedHit_count
  LDR   R4, [R4]
  MOV   R5, #-1
  MUL   R4, R4, R5
  ADD   R6, R6, R4

  LDR   R4, =missedHit_count
  LDR   R4, [R4]
  MOV   R5, #-1
  MUL   R4, R4, R5
  ADD   R6, R6, R4

  LDR   R4, =gameScore
  STR   R6, [R4]

  POP {R4-R5, PC}


@ level1SheetMusic subroutine
@ Read memory to display LED
@ Paramaters:
@ null
@
@ Return:
@ null
level1SheetMusic:
  PUSH {R4-R6, LR}

  LDR     R4, = sheetMusicReadAddress
  LDR     R5, [R4]
  LDR     R6, [R5]
  CMP     R6, #0
  BEQ     .Lreset
  ADD     R5, R5, #1
  STR     R5, [R4]
  B       .LememoryRead
.Lreset:
  LDR   R5, = sheetMusic
  @LDR   R5, = sheetMusicSecondLevel
  STR   R5, [R4]
.LememoryRead:

  LDR     R4, = sheetMusicReadAddress
  LDR     R5, [R4]
  LDRB    R6, [R5]
  MOV     R6, R6, LSL#8

  LDR     R4, =GPIOE_ODR            @   clear LEDs;
  LDR     R5, [R4]
  BIC     R5, R5, 0x0000FF00     
  ORR     R5, R5, R6                @   set new LED status;
  STR     R5, [R4]

    POP {R4-R6, PC}

@ level2SheetMusic subroutine
@ Read memory to display LED
@ Paramaters:
@ null
@
@ Return:
@ null
level2SheetMusic:
  PUSH {R4-R6, LR}

  LDR     R4, = sheetMusicReadAddress
  LDR     R5, [R4]
  LDR     R6, [R5]
  CMP     R6, #0
  BEQ     .LresetLevel2
  ADD     R5, R5, #1
  STR     R5, [R4]
  B       .LememoryReadLevel2
.LresetLevel2:
  @LDR   R5, = sheetMusic
  LDR   R5, = sheetMusicSecondLevel
  STR   R5, [R4]
.LememoryReadLevel2:

  LDR     R4, = sheetMusicReadAddress
  LDR     R5, [R4]
  LDRB    R6, [R5]
  MOV     R6, R6, LSL#8

  LDR     R4, =GPIOE_ODR            @   clear LEDs;
  LDR     R5, [R4]
  BIC     R5, R5, 0x0000FF00     
  ORR     R5, R5, R6                @   set new LED status;
  STR     R5, [R4]

    POP {R4-R6, PC}

@ ResetForLevel2SheetMusic subroutine
@ reset systick timer and set the memory read address for next level sheet music
@ Paramaters:
@ null
@
@ Return:
@ null
ResetForLevel2SheetMusic:
  PUSH {R4-R6, LR}
  

  @.equ    BLINK_PERIOD, 150
  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 
@___________________________________________________________
  LDR     R4, =blink_countdown
  MOV     R5, 150
  STR     R5, [R4]  
@--------------------------------------------------------------

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1
  
  LDR     R4, =nextLevelIndicator
  LDR     R5, =0x2                    
  STR     R5, [R4]

  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  LDR   R4, =good_count
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]
  LDR   R4, =missedHit_count
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]
  LDR   R4, =isJudgmentPhase
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]
  LDR   R4, =perfect_count
  LDR   R5, [R4]
  MOV   R5, #0
  STR   R5, [R4]


  POP {R4-R6, PC}
  .section .data
  
button_count:
  .space  4

blink_countdown:
  .space  4

missedHit_count:
  .space  4

isJudgmentPhase:
  .space  1

bonusTime:
  .space  4
  
good_count:
  .space  4

perfect_count:
  .space  4

gameScore:
  .space  4

sheetMusicReadAddress:
  .space  4

sheetMusic:

.byte 0b00001010, 0b00010101, 0b00101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010001, 0b10100010, 0b01000100

.byte 0b10001000, 0b00010001, 0b00100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010001, 0b10100010, 0b01000100

.byte 0b10001000, 0b00010001, 0b00100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000011, 0b00000110
.byte 0b00001100, 0b00011001, 0b00110010, 0b01100101

.byte 0b11001010, 0b10010101, 0b00101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101011, 0b01010111, 0b10101110, 0b01011100

.byte 0b10111000, 0b01110001, 0b11100010, 0b11000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010100, 0b10101000, 0b01010000

.byte 0b10100000, 0b01000000, 0b10000000, 0b00000000

blank:
  .space  4

nextLevelIndicator:
  .space  4

sheetMusicSecondLevel:
.byte 0b00000001, 0b00000010, 0b00000100, 0b00001000 @
.byte 0b00000100, 0b00000010, 0b00000001, 0b00000001 @

.byte 0b00001010, 0b00010101, 0b00101010, 0b01010100
.byte 0b01010100, 0b01010100, 0b01010100, 0b01010100  @

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10100010, 0b01010001, 0b10101000, 0b10101000  @ 

@.byte 0b10001010, 0b00010101, 0b00101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010001, 0b10100010, 0b01000100

.byte 0b10001000, 0b00010001, 0b00100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010001, 0b10100010, 0b01000100

.byte 0b10001000, 0b00010001, 0b00100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000011, 0b00000110
.byte 0b00001100, 0b00011001, 0b00110010, 0b01100101

.byte 0b11001010, 0b10010101, 0b00101010, 0b01010101
.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100

.byte 0b10101000, 0b01010001, 0b10100010, 0b01000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010100
.byte 0b10101000, 0b01010000, 0b10100000, 0b01000000

.byte 0b10000000, 0b00000001, 0b00000010, 0b00000101
.byte 0b00001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101011, 0b01010111, 0b10101110, 0b01011100

.byte 0b10111000, 0b01110001, 0b11100010, 0b11000101
.byte 0b10001010, 0b00010101, 0b00101010, 0b01010101

.byte 0b10101010, 0b01010101, 0b10101010, 0b01010101
.byte 0b10101010, 0b01010100, 0b10101000, 0b01010000

.byte 0b10100000, 0b01000000, 0b10000000, 0b00000000
  .end