        MODULE  ?cstartup
        SECTION CSTACK:DATA:NOROOT(3)
        SECTION .intvec:CODE:NOROOT(2)
        EXTERN  __iar_program_start      
        PUBLIC  __vector_table
        DATA
__vector_table
        DCD     sfe(CSTACK)                     ; 0x0000 STACK
        DCD     Reset_Handler                   ; 0x0004 Reset Handler
        DCD     NMI_Handler                     ; 0x0008 NMI Handler
        DCD     HardFault_Handler               ; 0x000C Hard Fault Handler
        DCD     0                               ; 0x0010 Reserved
        DCD     0                               ; 0x0014 Reserved
        DCD     0                               ; 0x0018 Reserved
        DCD     0                               ; 0x001C Reserved
        DCD     0                               ; 0x0020 Reserved
        DCD     0                               ; 0x0024 Reserved
        DCD     0                               ; 0x0028 Reserved
        DCD     SVC_Handler                     ; 0x002C SVCall Handler
        DCD     0                               ; 0x0030 Reserved
        DCD     0                               ; 0x0034 Reserved
        DCD     PendSV_Handler                  ; 0x0038 PendSV Handler
        DCD     SysTick_Handler                 ; 0x003C SysTick Handler
        DCD     WWDG_Handler                    ; 0x0040 Window Watchdog
        DCD     PVD_Handler                     ; 0x0044 PVD through EXTI Line detect
        DCD     RTC_Handler                     ; 0x0048 RTC
        DCD     FLASH_Handler                   ; 0x004C Flash
        DCD     RCC_Handler                     ; 0x0050 RCC
        DCD     EXTI01_Handler                 ; 0x0054 EXTI Line 0..1
        DCD     EXTI23_Handler                 ; 0x0058 EXTI Line 2..3
        DCD     EXTI415_Handler                ; 0x005C EXTI Line 4..15
        DCD     TSC_Handler                     ; 0x0060 Touch sensing
        DCD     DMA1_CH1_Handler                ; 0x0064 DMA1 Channel 1
        DCD     DMA1_CH23_DMA2_CH12_Handler     ; 0x0068 DMA1 Channel 2..3, DMA2 Channel 1..2
        DCD     DMA1_CH47_DMA2_CH35_Handler     ; 0x006C DMA1 Channel 4..7, DMA2 Channel 3..5
        DCD     ADC_COMP_Handler                ; 0x0070 ADC and COMP (ADC interrupt combined with EXTI lines 21 and 22)
        DCD     TIM1_Handler                    ; 0x0074 TIM1 break, update, trigger and commutation
        DCD     TIM1_CC_Handler                 ; 0x0078 TIM1 capture compare
        DCD     TIM2_Handler                    ; 0x007C TIM2
        DCD     TIM3_Handler                    ; 0x0080 TIM3
        DCD     TIM6_DAC_Handler                ; 0x0084 TIM6 and DAC underrun 
        DCD     TIM7_Handler                    ; 0x0088 TIM7
        DCD     TIM14_Handler                   ; 0x008C TIM14
        DCD     TIM15_Handler                   ; 0x0090 TIM15
        DCD     TIM16_Handler                   ; 0x0094 TIM16
        DCD     TIM17_Handler                   ; 0x0098 TIM17
        DCD     I2C1_Handler                    ; 0x009C I2C1 (combined with EXTI line 23)
        DCD     I2C2_Handler                    ; 0x00A0 I2C2
        DCD     SPI1_Handler                    ; 0x00A4 SPI1
        DCD     SPI2_Handler                    ; 0x00A8 SPI2
        DCD     UART1_Handler                   ; 0x00AC USART1
        DCD     UART2_Handler                   ; 0x00B0 USART2
        DCD     UART38_Handler                  ; 0x00B4 USART3..8 (combined with EXTI line 28)
        DCD     CEC_CAN_Handler                 ; 0x00B8 CEC and CAN (combined with EXTI line 27)
        DCD     USB_Handler                     ; 0x00BC USB (combined with EXTI line 18)
        
        THUMB
        
        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK WWDG_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
WWDG_Handler
        B WWDG_Handler

        PUBWEAK PVD_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PVD_Handler
        B PVD_Handler

        PUBWEAK RTC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
RTC_Handler
        B RTC_Handler

        PUBWEAK FLASH_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
FLASH_Handler
        B FLASH_Handler

        PUBWEAK RCC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
RCC_Handler
        B RCC_Handler

        PUBWEAK EXTI01_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI01_Handler
        B EXTI01_Handler

        PUBWEAK EXTI23_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI23_Handler
        B EXTI23_Handler

        PUBWEAK EXTI415_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXTI415_Handler
        B EXTI415_Handler

        PUBWEAK TSC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TSC_Handler
        B TSC_Handler

        PUBWEAK DMA1_CH1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_CH1_Handler
        B DMA1_CH1_Handler

        PUBWEAK DMA1_CH23_DMA2_CH12_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_CH23_DMA2_CH12_Handler
        B DMA1_CH23_DMA2_CH12_Handler

        PUBWEAK DMA1_CH47_DMA2_CH35_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DMA1_CH47_DMA2_CH35_Handler
        B DMA1_CH47_DMA2_CH35_Handler

        PUBWEAK ADC_COMP_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
ADC_COMP_Handler
        B ADC_COMP_Handler

        PUBWEAK TIM1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM1_Handler
        B TIM1_Handler

        PUBWEAK TIM1_CC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM1_CC_Handler
        B TIM1_CC_Handler

        PUBWEAK TIM2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM2_Handler
        B TIM2_Handler

        PUBWEAK TIM3_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM3_Handler
        B TIM3_Handler

        PUBWEAK TIM6_DAC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM6_DAC_Handler
        B TIM6_DAC_Handler

        PUBWEAK TIM7_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM7_Handler
        B TIM7_Handler

        PUBWEAK TIM14_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM14_Handler
        B TIM14_Handler

        PUBWEAK TIM15_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM15_Handler
        B TIM15_Handler

        PUBWEAK TIM16_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM16_Handler
        B TIM16_Handler

        PUBWEAK TIM17_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIM17_Handler
        B TIM17_Handler

        PUBWEAK I2C1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C1_Handler
        B I2C1_Handler

        PUBWEAK I2C2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2C2_Handler
        B I2C2_Handler

        PUBWEAK SPI1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI1_Handler
        B SPI1_Handler

        PUBWEAK SPI2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPI2_Handler
        B SPI2_Handler

        PUBWEAK UART1_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART1_Handler
        B UART1_Handler

        PUBWEAK UART2_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART2_Handler
        B UART2_Handler

        PUBWEAK UART38_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UART38_Handler
        B UART38_Handler

        PUBWEAK CEC_CAN_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
CEC_CAN_Handler
        B CEC_CAN_Handler

        PUBWEAK USB_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
USB_Handler
        B USB_Handler

        END
