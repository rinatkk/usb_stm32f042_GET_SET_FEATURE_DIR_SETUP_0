#ifdef __cplusplus
  extern "C" {
#endif
#define  M0                                                     //Core M0
#define  u1     unsigned                                        //0...1
#define  u8     unsigned char                                   //0...255
#define  s8     signed char                                     //-128...127
#define  u16    unsigned short                                  //0...65535
#define  s16    signed short                                    //-32768...2767
#define  u32    unsigned int                                    //0...4294967295
#define  s32    signed int                                      //-2147483647...2147483647
#define  u64    unsigned long long                              //0...18446744073709551615
#define  s64    signed long long                                //-9223372036854775807...9223372036854775807
#define  f32    float                                           //±1.18e-38 to ±3.40e38 (exp 8bit (-126...127), mant 23bit)
#define  f64    double                                          //±2.23e-308 to ±1.8e308 (exp 11bit (-1022...1023), mant 52bit)

#define int8_t          unsigned char
#define uint8_t         unsigned char
#define uint16_t        unsigned short
#define uint32_t        unsigned int                                    //0...4294967295
    
/**************************************************************************************************/
#define ON                      ((u8)0x01)
#define OFF                     ((u8)0x00)
#define ONE                     ((u8)0x01)
#define NULL                    ((u8)0x00)
//Dec to Bin
#define TWO                     ((u16)0b0010)
#define N_03                    ((u16)0b0011)
#define N_04                    ((u16)0b0100)
#define N_05                    ((u16)0b0101)
#define N_06                    ((u16)0b0110)
#define N_07                    ((u16)0b0111)
#define N_08                    ((u16)0b1000)
#define N_09                    ((u16)0b1001)
#define N_10                    ((u16)0b1010)
#define N_11                    ((u16)0b1011)
#define N_12                    ((u16)0b1100)
#define N_13                    ((u16)0b1101)
#define N_14                    ((u16)0b1110)
#define N_15                    ((u16)0b1111)
#define N_16                    ((u32)0b00010000)
#define N_17                    ((u32)0b00010001)
#define N_18                    ((u32)0b00010010)
#define N_19                    ((u32)0b00010011)
#define N_20                    ((u32)0b00010100)
#define N_21                    ((u32)0b00010101)
#define N_22                    ((u32)0b00010110)
#define N_23                    ((u32)0b00010111)
#define N_24                    ((u32)0b00011000)
#define N_25                    ((u32)0b00011001)
#define N_26                    ((u32)0b00011010)
#define N_27                    ((u32)0b00011011)
#define N_28                    ((u32)0b00011100)
#define N_29                    ((u32)0b00011101)
#define N_30                    ((u32)0b00011110)
#define N_31                    ((u32)0b00011111)
#define N_32                    ((u32)0b00100000)
    
//*************************************************************************************************
#define N00                     ((u16) 0x0001)
#define N01                     ((u16) 0x0002)
#define N02                     ((u16) 0x0004)
#define N03                     ((u16) 0x0008)
#define N04                     ((u16) 0x0010)
#define N05                     ((u16) 0x0020)
#define N06                     ((u16) 0x0040)
#define N07                     ((u16) 0x0080)
#define N08                     ((u16) 0x0100)
#define N09                     ((u16) 0x0200)
#define N10                     ((u16) 0x0400)
#define N11                     ((u16) 0x0800)
#define N12                     ((u16) 0x1000)
#define N13                     ((u16) 0x2000)
#define N14                     ((u16) 0x4000)
#define N15                     ((u16) 0x8000)
    
/************************************* Core M0 memory map *****************************************/
#define FLASH_BASE              ((u32) 0x08000000)              //FLASH base address
#define UID_BASE                ((u32) 0x1FFFF7AC)              //CPU UID Base Address
#define OB_BASE                 ((u32) 0x1FFFF800)              //Option byte base address
#define SRAM_BASE               ((u32) 0x20000000)              //SRAM base address
#define APB_BASE                ((u32) 0x40000000)              //APB base address 
#define AHB1_BASE               ((u32) 0x40020000)              //AHB1 base address
#define AHB2_BASE               ((u32) 0x48000000)              //AHB2 base address
#define DWT_BASE                ((u32) 0xE0001000)              //Data Watchpoint and Trace unit Space Base Address
#define STK_BASE                ((u32) 0xE000E010)              //SysTick Base Address
#define NVIC_BASE               ((u32) 0xE000E100)              //NVIC Base Address
#define SCB_BASE                ((u32) 0xE000ED00)              //SCB Base Address
#define SYS_RESET *(u32*)(SCB_BASE + 12) = 0x05FA0004           //System reset
#define FLASH_SIZE (*(u16*)0x1FFFF7CC)                          //FLASH size
/*********************************** Peripheral memory map ****************************************/
#define TIM02_BASE              (APB_BASE + 0x00000000)
#define TIM03_BASE              (APB_BASE + 0x00000400)
#define TIM06_BASE              (APB_BASE + 0x00001000)
#define TIM07_BASE              (APB_BASE + 0x00001400)
#define TIM14_BASE              (APB_BASE + 0x00002000)
#define RTC_BASE                (APB_BASE + 0x00002800)
#define WWDG_BASE               (APB_BASE + 0x00002C00)
#define IWDG_BASE               (APB_BASE + 0x00003000)
#define SPI2_BASE               (APB_BASE + 0x00003800)
#define UART2_BASE              (APB_BASE + 0x00004400)
#define UART3_BASE              (APB_BASE + 0x00004800)
#define UART4_BASE              (APB_BASE + 0x00004C00)
#define UART5_BASE              (APB_BASE + 0x00005000)
#define I2C1_BASE               (APB_BASE + 0x00005400)
#define I2C2_BASE               (APB_BASE + 0x00005800)
#define USB_BASE                (APB_BASE + 0x00005C00)
#define USB_RAM_BASE            (APB_BASE + 0x00006000)
#define CAN_BASE                (APB_BASE + 0x00006400)
#define CRS_BASE                (APB_BASE + 0x00006C00)
#define PWR_BASE                (APB_BASE + 0x00007000)
#define DAC_BASE                (APB_BASE + 0x00007400)
#define CEC_BASE                (APB_BASE + 0x00007800)
#define SYSCFG_BASE             (APB_BASE + 0x00010000)
#define EXTI_BASE               (APB_BASE + 0x00010400)
#define UART6_BASE              (APB_BASE + 0x00011400)
#define UART7_BASE              (APB_BASE + 0x00011800)
#define UART8_BASE              (APB_BASE + 0x00011C00)
#define ADC_BASE                (APB_BASE + 0x00012400)
#define TIM01_BASE              (APB_BASE + 0x00012C00)
#define SPI1_BASE               (APB_BASE + 0x00013000)
#define UART1_BASE              (APB_BASE + 0x00013800)
#define TIM15_BASE              (APB_BASE + 0x00014000)
#define TIM16_BASE              (APB_BASE + 0x00014400)
#define TIM17_BASE              (APB_BASE + 0x00014800)
#define DBGMCU_BASE             (APB_BASE + 0x00015800)   
#define DMA1_BASE               (AHB1_BASE + 0x00000000)
#define DMA2_BASE               (AHB1_BASE + 0x00000400)
#define RCC_BASE                (AHB1_BASE + 0x00001000)
#define FLASH_R_BASE            (AHB1_BASE + 0x00002000)
#define CRC_BASE                (AHB1_BASE + 0x00003000)
#define TSC_BASE                (AHB1_BASE + 0x00004000)
#define GPIOA_BASE              (AHB2_BASE + 0x00000000)
#define GPIOB_BASE              (AHB2_BASE + 0x00000400)
#define GPIOC_BASE              (AHB2_BASE + 0x00000800)
#define GPIOD_BASE              (AHB2_BASE + 0x00000C00)
#define GPIOE_BASE              (AHB2_BASE + 0x00001000)
#define GPIOF_BASE              (AHB2_BASE + 0x00001400)
/****************************************** IRQ ***************************************************/
typedef enum
{
  WWDG_IRQ                      = 0,  //Window WatchDog Interrupt
  PVD_IRQ                       = 1,  //PVD through EXTI Line detection Interrupt
  RTC_IRQ                       = 2,  //RTC global Interrupt
  FLASH_IRQ                     = 3,  //FLASH global Interrupt
  RCC_IRQ                       = 4,  //RCC global Interrupt
  EXTI01_IRQ                    = 5,  //EXTI Line0..1 Interrupt
  EXTI23_IRQ                    = 6,  //EXTI Line2..3 Interrupt
  EXTI415_IRQ                   = 7,  //EXTI Line4..15 Interrupt
  TSC_IRQ                       = 8,  //Touch sensing Interrupt
  DMA1_CH1_IRQ                  = 9,  //DMA1 Channel 1 Interrupt
  DMA1_CH23_DMA2_CH12_IRQ       = 10, //DMA1 Channel 2..3, DMA2 Channel 1..2 Interrupt
  DMA1_CH47_DMA2_CH35_IRQ       = 11, //DMA1 Channel 4..7, DMA2 Channel 3..5 Interrupt
  ADC1_COMP_IRQ                 = 12, //ADC and COMP Interrupt (ADC interrupt combined with EXTI lines 21 and 22)
  TIM1_IRQ                      = 13, //TIM1 break, update, trigger and commutation Interrupt
  TIM1_CC_IRQ                   = 14, //TIM1 capture compare Interrupt
  TIM2_IRQ                      = 15, //TIM2 Interrupt
  TIM3_IRQ                      = 16, //TIM3 Interrupt
  TIM6_DAC_IRQ                  = 17, //TIM6 and DAC underrun  Interrupt
  TIM7_IRQ                      = 18, //TIM7 Interrupt
  TIM14_IRQ                     = 19, //TIM14 Interrupt
  TIM15_IRQ                     = 20, //TIM15 Interrupt
  TIM16_HIRQ                    = 21, //TIM16 Interrupt
  TIM17_IRQ                     = 22, //TIM17 Interrupt
  I2C1_IRQ                      = 23, //I2C1 Interrupt (combined with EXTI line 23)
  I2C2_IRQ                      = 24, //I2C2 Interrupt
  SPI1_IRQ                      = 25, //SPI1 Interrupt
  SPI2_IRQ                      = 26, //SPI2 Interrupt
  UART1_IRQ                     = 27, //USART1 Interrupt
  UART2_IRQ                     = 28, //USART2 Interrupt
  UART38_IRQ                    = 29, //USART3..8 Interrupt (combined with EXTI line 28)
  CEC_CAN_IRQ                   = 30, //CEC and CAN Interrupt (combined with EXTI line 27)
  USB_IRQ                       = 31, //USB Interrupt (combined with EXTI line 18)
}IRQ_Type;
/***************************************** SysTick ************************************************/
typedef struct
{
  struct
  {
    u1 ENABLE     : 1;  //Counter enable
    u1 TICKINT    : 1;  //SysTick exception request enable
    u1 CLKSOURCE  : 1;  //Clock source selection
    u1 RSVD1      : 13; //Reserved
    u1 COUNTFLAG  : 1;  //Returns 1 if timer counted to 0 since last time this was read
    u1 RSVD0      : 15; //Reserved
  }volatile CSR;              //SysTick control and status register (STK_CSR)
  struct
  {
    u1 RELOAD     : 24; //RELOAD value
    u1 RSVD0      : 8;  //Reserved
  }volatile RVR;              //SysTick reload value register (STK_RVR)
  struct
  {
    u1 CURRENT    : 24; //Current counter value
    u1 RSVD0      : 8;  //Reserved
  }volatile CVR;              //SysTick current value register (STK_CVR)
  struct
  {
    u1 TENMS      : 24; //Calibration value
    u1 RSVD0      : 6;  //Reserved
    u1 SKEW       : 1;  //SKEW flag
    u1 NOREF      : 1;  //NOREF flag
  }volatile CALIB;            //SysTick calibration value register (STK_CALIB)
}STK_Type;
#define STK ((STK_Type*) STK_BASE)
#define STK_EN                  STK->CSR.ENABLE                 //Counter enable
#define STK_IRQ                 STK->CSR.TICKINT                //SysTick exception request enable
#define STK_CLK                 STK->CSR.CLKSOURCE              //Clock source selection
#define STK_FLG                 STK->CSR.COUNTFLAG              //Returns 1 if timer counted to 0 since last time this was read
#define STK_REL                 STK->RVR.RELOAD                 //RELOAD value
#define STK_CUR                 STK->CVR.CURRENT                //Current counter value
#define STK_TENMS               STK->CALIB.TENMS                //Calibration value
#define STK_SKEW                STK->CALIB.SKEW                 //SKEW flag
#define STK_NOREF               STK->CALIB.NOREF                //NOREF flag
/******************************************* SCB **************************************************/
typedef struct
{
  struct
  {
    u1 REV        : 4;  //Indicates patch release: 0x0 = Patch 0
    u1 PART       : 12; //Indicates part number: 0xC20 = Cortex-M0
    u1 CONST      : 4;  //Reads as 0xF
    u1 VAR        : 4;  //Indicates processor revision: 0x2 = Revision 2
    u1 IMP        : 8;  //Indicates implementor: 0x41 = ARM
  }volatile CPUID;      //CPUID base register (CPUID)
  struct
  {
    u1 VECTACTIVE : 6;  //Active ISR number field
    u1 RSVD0      : 6;  //Reserved
    u1 VECTPENDING: 6;  //Pending ISR number field
    u1 RSVD1      : 4;  //Reserved
    u1 ISRPENDING : 1;  //Interrupt pending flag
    u1 RSVD2      : 2;  //Reserved
    u1 PENDSTCLR  : 1;  //Clear pending SysTick bit
    u1 PENDSTSET  : 1;  //Set a pending SysTick bit
    u1 PENDSVCLR  : 1;  //Clear pending pendSV bit
    u1 PENDSVSET  : 1;  //Set pending pendSV bit
    u1 RSVD3      : 2;  //Reserved
    u1 NMIPENDSET : 1;  //Set pending NMI bit
  }volatile ICSR;       //Interrupt control and state register (ICSR)
  volatile u32 RSVD0;   //Reserved
  struct
  {
    u1 RSVD0      : 1;  //Reserved
    u1 VECTCLRACT : 1;  //Clear active vector bit
    u1 SYSRESETREQ: 1;  //Causes a signal to be asserted to the outer system that indicates a reset is requested
    u1 RSVD1      : 12; //Reserved
    u1 ENDIANESS  : 1;  //Data endianness bit (1 = big endian, 0 = little endian)
    u1 VECTKEY    : 16; //Register key. Writing to this register requires 0x05FA in the VECTKEY field. Otherwise the write value is ignored. Reads as 0xFA05
  }volatile AIRCR;      //Application interrupt and reset control register (AIRCR)
  struct
  {
    u1 RSVD0      : 1;  //Reserved
    u1 SLPONEXIT  : 1;  //Sleep on exit when returning from Handler mode to Thread mode
    u1 SLEEPDEEP  : 1;  //Sleep deep bit
    u1 RSVD1      : 1;  //Reserved
    u1 SEVONPEND  : 1;  //When enabled, this causes WFE to wake up when an interrupt moves from inactive to pended
    u1 RSVD2      : 27; //Reserved
  }volatile SCR;        //System control register (SCR)
  struct
  {
    u1 RSVD0      : 3;  //Reserved
    u1 UNALIGNTRP : 1;  //Trap for unaligned access
    u1 RSVD1      : 5;  //Reserved
    u1 STKALIGN   : 1;  //1 = 8-byte, 1 = 4-byte
    u1 RSVD2      : 22; //Reserved
  }volatile CCR;        //Configuration and control register (CCR)
  volatile u32 RSVD1;   //Reserved
  struct
  {
    u1 RSVD0      : 24; //Reserved
    u1 PRI_11     : 8;  //Priority of system handler 11, SVCall
  }volatile SHPR2;      //System handler priority registers (SHPR2)
  struct
  {
    u1 RSVD0      : 16; //Reserved
    u1 PRI_14     : 8;  //Priority of system handler 14, PendSV
    u1 PRI_15     : 8;  //Priority of system handler 15, SysTick exception
  }volatile SHPR3;      //System handler priority registers (SHPR3)
}SCB_Type;
#define SCB ((SCB_Type*) SCB_BASE)
#define SCB_REV                 SCB->CPUID.REV                  //Indicates patch release: 0x0 = Patch 0
#define SCB_PART                SCB->CPUID.PART                 //Indicates part number: 0xC23 = Cortex-M3
#define SCB_CONST               SCB->CPUID.CONST                //Reads as 0xF
#define SCB_VAR                 SCB->CPUID.VAR                  //Indicates processor revision: 0x2 = Revision 2
#define SCB_IMP                 SCB->CPUID.IMP                  //Indicates implementor: 0x41 = ARM
#define SCB_VECTACTIVE          SCB->ICSR.VECTACTIVE            //Active ISR number field
#define SCB_VECTPENDING         SCB->ICSR.VECTPENDING           //Pending ISR number field
#define SCB_ISRPENDING          SCB->ICSR.ISRPENDING            //Interrupt pending flag
#define SCB_PENDSTCLR           SCB->ICSR.PENDSTCLR             //Clear pending SysTick bit
#define SCB_PENDSTSET           SCB->ICSR.PENDSTSET             //Set a pending SysTick bit
#define SCB_PENDSVCLR           SCB->ICSR.PENDSVCLR             //Clear pending pendSV bit
#define SCB_PENDSVSET           SCB->ICSR.PENDSVSET             //Set pending pendSV bit
#define SCB_NMIPENDSET          SCB->ICSR.NMIPENDSET            //Set pending NMI bit
#define SCB_VECTCLRACT          SCB->AIRCR.VECTCLRACT           //Clear active vector bit
#define SCB_SYSRESET            SCB->AIRCR.SYSRESETREQ          //Causes a signal to be asserted to the outer system that indicates a reset is requested
#define SCB_ENDIANESS           SCB->AIRCR.ENDIANESS            //Data endianness bit (1 = big endian, 0 = little endian)
#define SCB_VECTKEY             SCB->AIRCR.VECTKEY              //Register key. Writing to this register requires 0x5FA in the VECTKEY field. Otherwise the write value is ignored. Reads as 0xFA05
#define SCB_SLPONEXIT           SCB->SCR.SLPONEXIT              //Sleep on exit when returning from Handler mode to Thread mode
#define SCB_SLEEPDEEP           SCB->SCR.SLEEPDEEP              //Sleep deep bit
#define SCB_SEVONPEND           SCB->SCR.SEVONPEND              //When enabled, this causes WFE to wake up when an interrupt moves from inactive to pended
#define SCB_UNALIGNTRP          SCB->CCR.UNALIGNTRP             //Trap for unaligned access
#define SCB_STKALIGN            SCB->CCR.STKALIGN               //1 = 8-byte, 1 = 4-byte
#define SCB_PRI_11              SCB->SHPR2.PRI_11               //Priority of system handler 11, SVCall
#define SCB_PRI_14              SCB->SHPR3.PRI_14               //Priority of system handler 14, PendSV
#define SCB_PRI_15              SCB->SHPR3.PRI_15               //Priority of system handler 15, SysTick exception
/******************************************* NVIC *************************************************/
typedef struct
{
  volatile u32 ISER[1];         //Interrupt set-enable registers (NVIC_ISERx)
  volatile u32 RSVD0[31];       //Reserved
  volatile u32 ICER[1];         //Interrupt clear-enable registers (NVIC_ICERx)
  volatile u32 RSVD1[31];       //Reserved
  volatile u32 ISPR[1];         //Interrupt set-pending registers (NVIC_ISPRx)
  volatile u32 RSVD2[31];       //Reserved
  volatile u32 ICPR[1];         //Interrupt clear-pending registers (NVIC_ICPRx)
  volatile u32 RSVD3[31];       //Reserved
  volatile u32 RSVD4[64];       //Reserved
  volatile u32 IPR[8];          //Interrupt priority register (IPR0-IPR7)
}NVIC_Type;
#define NVIC ((NVIC_Type*) NVIC_BASE)
#define NVIC_INT_SET_EN(n, IRQn)        NVIC->ISER[n] = (1 <<((u32)(IRQn)& 0x1F))
#define NVIC_INT_CLR_EN(n, IRQn)        NVIC->ICER[n] = (1 <<((u32)(IRQn))) //?
#define NVIC_INT_SET_PEN(n, IRQn)       NVIC->ISER[n] = (1 <<((u32)(IRQn))) //?
#define NVIC_INT_CLR_PEN(n, IRQn)       NVIC->ISER[n] = (1 <<((u32)(IRQn))) //?
//#define NVIC_INT_PRIOR(n, IRQn)         NVIC->ISER[N] = (1 <<((u32)(IRQn))) //?
/******************************************* RCC **************************************************/
typedef struct
{
  struct
  {
    u1 HSION      : 1;  //Internal high-speed clock enable  
    u1 HSIRDY     : 1;  //Internal high-speed clock ready flag
    u1 RSVD0      : 1;  //Reserved
    u1 HSITRIM    : 5;  //Internal high-speed clock trimming
    u1 HSICAL     : 8;  //Internal high-speed clock calibration
    u1 HSEON      : 1;  //External high-speed clock enable
    u1 HSERDY     : 1;  //External high-speed clock ready flag
    u1 HSEBYP     : 1;  //External high-speed clock bypass
    u1 CSSON      : 1;  //Clock security system enable
    u1 RSVD1      : 4;  //Reserved
    u1 PLLON      : 1;  //PLL enable
    u1 PLLRDY     : 1;  //PLL clock ready flag
    u1 RSVD2      : 6;  //Reserved
  }volatile CR;         //Clock control register (RCC_CR)
  struct
  {
    u1 SW         : 2;  //System clock switch
    u1 SWS        : 2;  //System clock switch status
    u1 HPRE       : 4;  //AHB prescaler
    u1 PPRE       : 3;  //APB prescaler
    u1 RSVD0      : 3;  //Reserved
    u1 ADCPRE     : 1;  //ADC prescaler
    u1 PLLSRC     : 2;  //PLL entry clock source
    u1 PLLXTPRE   : 1;  //HSE divider for PLL entry
    u1 PLLMUL     : 4;  //PLL multiplication factor
    u1 RSVD1      : 2;  //Reserved
    u1 MCO        : 4;  //Microcontroller clock output
    u1 MCOPRE     : 3;  //Microcontroller Clock Output Prescaler
    u1 PLLNODIV   : 1;  //PLL clock not divided for MCO
  }volatile CFGR;       //Clock configuration register (RCC_CFGR)
  struct
  {
    u1 LSIRDYF    : 1;  //LSI ready interrupt flag
    u1 LSERDYF    : 1;  //LSE ready interrupt flag
    u1 HSIRDYF    : 1;  //HSI ready interrupt flag
    u1 HSERDYF    : 1;  //HSE ready interrupt flag
    u1 PLLRDYF    : 1;  //PLL ready interrupt flag
    u1 HSI14RDYF  : 1;  //HSI14 ready interrupt flag
    u1 HSI48RDYF  : 1;  //HSI48 ready interrupt flag
    u1 CSSF       : 1;  //Clock security system interrupt flag
    u1 LSIRDYIE   : 1;  //LSI ready interrupt enable
    u1 LSERDYIE   : 1;  //LSE ready interrupt enable
    u1 HSIRDYIE   : 1;  //HSI ready interrupt enable
    u1 HSERDYIE   : 1;  //HSE ready interrupt enable
    u1 PLLRDYIE   : 1;  //PLL ready interrupt enable
    u1 HSI14RDYIE : 1;  //HSI14 ready interrupt enable
    u1 HSI48RDYIE : 1;  //HSI48 ready interrupt enable
    u1 RSVD0      : 1;  //Reserved
    u1 LSIRDYC    : 1;  //LSI ready interrupt clear
    u1 LSERDYC    : 1;  //LSE ready interrupt clear
    u1 HSIRDYC    : 1;  //HSI ready interrupt clear
    u1 HSERDYC    : 1;  //HSE ready interrupt clear
    u1 PLLRDYC    : 1;  //PLL ready interrupt clear
    u1 HSI14RDYC  : 1;  //HSI14 ready interrupt clear
    u1 HSI48RDYC  : 1;  //HSI48 ready interrupt clear
    u1 CSSC       : 1;  //Clock security system interrupt clear
    u1 RSVD1      : 8;  //Reserved
  }volatile CIR;        //Clock interrupt register (RCC_CIR)
  struct
  {
    u1 SYSCFGRST  : 1;  //SYSCFG reset
    u1 RSVD0      : 4;  //Reserved
    u1 USART6RST  : 1;  //USART6 reset
    u1 USART7RST  : 1;  //USART7 reset
    u1 USART8RST  : 1;  //USART8 reset
    u1 RSVD1      : 1;  //Reserved
    u1 ADCRST     : 1;  //ADC interface reset
    u1 RSVD2      : 1;  //Reserved
    u1 TIM1RST    : 1;  //TIM1 timer reset
    u1 SPI1RST    : 1;  //SPI1 reset
    u1 RSVD3      : 1;  //Reserved
    u1 USART1RST  : 1;  //USART1 reset
    u1 RSVD4      : 1;  //Reserved
    u1 TIM15RST   : 1;  //TIM15 timer reset
    u1 TIM16RST   : 1;  //TIM16 timer reset
    u1 TIM17RST   : 1;  //TIM17 timer reset
    u1 RSVD5      : 3;  //Reserved
    u1 DBGMCURST  : 1;  //Debug MCU reset
    u1 RSVD6      : 9;  //Reserved
  }volatile APB2RSTR;   //APB2 peripheral reset register (RCC_APB2RSTR)
  struct
  {
    u1 TIM2RST    : 1;  //TIM2 timer reset
    u1 TIM3RST    : 1;  //TIM3 timer reset
    u1 RSVD0      : 2;  //Reserved
    u1 TIM6RST    : 1;  //TIM6 timer reset
    u1 TIM7RST    : 1;  //TIM7 timer reset
    u1 RSVD1      : 2;  //Reserved
    u1 TIM14RST   : 1;  //TIM14 timer reset
    u1 RSVD2      : 2;  //Reserved
    u1 WWDGRST    : 1;  //Window watchdog reset
    u1 RSVD3      : 2;  //Reserved
    u1 SPI2RST    : 1;  //SPI2 reset
    u1 RSVD4      : 2;  //Reserved
    u1 UART2RST   : 1;  //USART2 reset
    u1 UART3RST   : 1;  //USART3 reset
    u1 UART4RST   : 1;  //UART4 reset
    u1 UART5RST   : 1;  //UART5 reset
    u1 I2C1RST    : 1;  //I2C1 reset
    u1 I2C2RST    : 1;  //I2C2 reset
    u1 USBRST     : 1;  //USB reset
    u1 RSVD5      : 1;  //Reserved
    u1 CANRST     : 1;  //CAN reset
    u1 RSVD6      : 1;  //Reserved
    u1 CRSRST     : 1;  //Clock Recovery System interface reset
    u1 PWRRST     : 1;  //Power interface reset
    u1 DACRST     : 1;  //DAC interface reset
    u1 CECRST     : 1;  //HDMI CEC reset
    u1 RSVD7      : 1;  //Reserved
  }volatile APB1RSTR;   //APB1 peripheral reset register (RCC_APB1RSTR)
  struct
  {
    u1 DMA1EN     : 1;  //DMA1 clock enable
    u1 DMA2EN     : 1;  //DMA2 clock enable
    u1 SRAMEN     : 1;  //SRAM interface clock enable
    u1 RSVD0      : 1;  //Reserved
    u1 FLITFEN    : 1;  //FLITF clock enable
    u1 RSVD1      : 1;  //Reserved
    u1 CRCEN      : 1;  //CRC clock enable
    u1 RSVD2      : 10; //Reserved
    u1 IOPAEN     : 1;  //IO port A clock enable
    u1 IOPBEN     : 1;  //IO port B clock enable
    u1 IOPCEN     : 1;  //IO port C clock enable
    u1 IOPDEN     : 1;  //IO port D clock enable
    u1 IOPEEN     : 1;  //IO port E clock enable
    u1 IOPFEN     : 1;  //IO port F clock enable
    u1 RSVD3      : 1;  //Reserved
    u1 TSCEN      : 1;  //Touch sensing controller clock enable
    u1 RSVD4      : 7;  //Reserved
  }volatile AHBENR;     //AHB peripheral clock enable register (RCC_AHBENR)
  struct
  {
    u1 SYSCFGEN   : 1;  //SYSCFG enable
    u1 RSVD0      : 4;  //Reserved
    u1 USART6EN   : 1;  //USART6 enable
    u1 USART7EN   : 1;  //USART7 enable
    u1 USART8EN   : 1;  //USART8 enable
    u1 RSVD1      : 1;  //Reserved
    u1 ADCEN      : 1;  //ADC interface enable
    u1 RSVD2      : 1;  //Reserved
    u1 TIM1EN     : 1;  //TIM1 timer enable
    u1 SPI1EN     : 1;  //SPI1 enable
    u1 RSVD3      : 1;  //Reserved
    u1 USART1EN   : 1;  //USART1 enable
    u1 RSVD4      : 1;  //Reserved
    u1 TIM15EN    : 1;  //TIM15 timer enable
    u1 TIM16EN    : 1;  //TIM16 timer enable
    u1 TIM17EN    : 1;  //TIM17 timer enable
    u1 RSVD5      : 3;  //Reserved
    u1 DBGMCUEN   : 1;  //Debug MCU enable
    u1 RSVD6      : 9;  //Reserved
  }volatile APB2ENR;    //APB2 peripheral clock enable register (RCC_APB2ENR)
  struct
  {
    u1 TIM2EN     : 1;  //TIM2 timer enable
    u1 TIM3EN     : 1;  //TIM3 timer enable
    u1 RSVD0      : 2;  //Reserved
    u1 TIM6EN     : 1;  //TIM6 timer enable
    u1 TIM7EN     : 1;  //TIM7 timer enable
    u1 RSVD1      : 2;  //Reserved
    u1 TIM14EN    : 1;  //TIM14 timer enable
    u1 RSVD2      : 2;  //Reserved
    u1 WWDGEN     : 1;  //Window watchdog enable
    u1 RSVD3      : 2;  //Reserved
    u1 SPI2EN     : 1;  //SPI2 enable
    u1 RSVD4      : 2;  //Reserved
    u1 UART2EN    : 1;  //USART2 enable
    u1 UART3EN    : 1;  //USART3 enable
    u1 UART4EN    : 1;  //UART4 enable
    u1 UART5EN    : 1;  //UART5 enable
    u1 I2C1EN     : 1;  //I2C1 enable
    u1 I2C2EN     : 1;  //I2C2 enable
    u1 USBEN      : 1;  //USB enable
    u1 RSVD5      : 1;  //Reserved
    u1 CANEN      : 1;  //CAN enable
    u1 RSVD6      : 1;  //Reserved
    u1 CRSEN      : 1;  //Clock Recovery System interface enable
    u1 PWREN      : 1;  //Power interface enable
    u1 DACEN      : 1;  //DAC interface enable
    u1 CECEN      : 1;  //HDMI CEC enable
    u1 RSVD7      : 1;  //Reserved
  }volatile APB1ENR;    //APB1 peripheral clock enable register (RCC_APB1ENR)
  struct
  {
    u1 LSEON      : 1;  //External low-speed oscillator enable
    u1 LSERDY     : 1;  //External low-speed oscillator ready
    u1 LSEBYP     : 1;  //External low-speed oscillator bypass
    u1 LSEDRV     : 2;  //LSE oscillator drive capability
    u1 RSVD0      : 3;  //Reserved
    u1 RTCSEL     : 2;  //RTC clock source selection
    u1 RSVD1      : 5;  //Reserved
    u1 RTCEN      : 1;  //RTC clock enable
    u1 BDRST      : 1;  //Backup domain software reset
    u1 RSVD2      : 15; //Reserved
  }volatile BDCR;       //Backup domain control register (RCC_BDCR)
  struct
  {
    u1 LSION      : 1;  //Internal low-speed oscillator enable
    u1 LSIRDY     : 1;  //Internal low-speed oscillator ready
    u1 RSVD0      : 21; //Reserved
    u1 V18PWRRSTF : 1;  //Reset flag of the 1.8 V domain.
    u1 RMVF       : 1;  //Remove reset flag
    u1 OBLRSTF    : 1;  //Option byte loader reset flag
    u1 PINRSTF    : 1;  //PIN reset flag
    u1 PORRSTF    : 1;  //POR/PDR reset flag
    u1 SFTRSTF    : 1;  //Software reset flag
    u1 IWDGRSTF   : 1;  //Independent watchdog reset flag
    u1 WWDGRSTF   : 1;  //Window watchdog reset flag
    u1 LPWRRSTF   : 1;  //Low-power reset flag
  }volatile CSR;        //Control/status register (RCC_CSR)
  struct
  {
    u1 RSVD0      : 17; //Reserved
    u1 IOPARST    : 1;  //I/O port A reset
    u1 IOPBRST    : 1;  //I/O port B reset
    u1 IOPCRST    : 1;  //I/O port C reset
    u1 IOPDRST    : 1;  //I/O port D reset
    u1 IOPERST    : 1;  //I/O port E reset
    u1 IOPFRST    : 1;  //I/O port F reset
    u1 RSVD1      : 1;  //Reserved
    u1 TSCRST     : 1;  //Touch sensing controller reset
    u1 RSVD2      : 7;  //Reserved
  }volatile AHBRSTR;    //AHB peripheral reset register (RCC_AHBRSTR)
  struct
  {
    u1 PREDIV     : 4;  //PREDIV division factor
    u1 RSVD0      : 28; //Reserved
  }volatile CFGR2;      //Clock configuration register 2 (RCC_CFGR2)
  struct
  {
    u1 USART1SW   : 2;  //USART1 clock source selection
    u1 RSVD0      : 2;  //Reserved
    u1 I2C1SW     : 1;  //I2C1 clock source selection
    u1 RSVD1      : 1;  //Reserved
    u1 CECSW      : 1;  //HDMI CEC clock source selection
    u1 USBSW      : 1;  //USB clock source selection
    u1 ADCSW      : 1;  //ADC clock source selection
    u1 RSVD2      : 7;  //Reserved
    u1 USART2SW   : 2;  //USART2 clock source selection (available only on STM32F07x and STM32F09x devices)
    u1 USART3SW   : 2;  //USART3 clock source selection (available only on STM32F09x devices)
    u1 RSVD3      : 12; //Reserved
  }volatile CFGR3;      //Clock configuration register 3 (RCC_CFGR3)
  struct
  {
    u1 HSI14ON    : 1;  //HSI14 clock enable
    u1 HSI14RDY   : 1;  //HSI14 clock ready flag
    u1 HSI14DIS   : 1;  //HSI14 clock request from ADC disable
    u1 HSI14TRIM  : 5;  //HSI14 clock trimming
    u1 HSI14CAL   : 8;  //HSI14 clock calibration
    u1 HSI48ON    : 1;  //HSI48 clock enable
    u1 HSI48RDY   : 1;  //HSI48 clock ready flag
    u1 RSVD0      : 6;  //Reserved
    u1 HSI48CAL   : 8;  //HSI48 factory clock calibration
  }volatile CR2;        //Clock control register 2 (RCC_CR2)
}RCC_Type;
#define RCC ((RCC_Type*) RCC_BASE)
#define RCC_HSI_EN              RCC->CR.HSION                   //Internal high-speed clock enable
#define RCC_HSI_RDY             RCC->CR.HSIRDY                  //Internal high-speed clock ready flag
#define RCC_HSI_CLK             RCC->CR.HSITRIM                 //Internal high-speed clock trimming
#define RCC_HSI_CAL             RCC->CR.HSICAL                  //Internal high-speed clock calibration
#define RCC_HSE_EN              RCC->CR.HSEON                   //External high-speed clock enable
#define RCC_HSE_RDY             RCC->CR.HSERDY                  //External high-speed clock ready flag
#define RCC_HSE_BYP             RCC->CR.HSEBYP                  //External high-speed clock bypass
#define RCC_CSS_EN              RCC->CR.CSSON                   //Clock security system enable
#define RCC_PLL_EN              RCC->CR.PLLON                   //PLL enable
#define RCC_PLL_RDY             RCC->CR.PLLRDY                  //PLL clock ready flag
#define RCC_SYSCLK              RCC->CFGR.SW                    //System clock switch
#define RCC_SYSCLK_FLG          RCC->CFGR.SWS                   //System clock switch status
#define RCC_AHB_DIV             RCC->CFGR.HPRE                  //AHB prescaler
#define RCC_APB_DIV             RCC->CFGR.PPRE                  //APB prescaler
#define RCC_ADC_DIV             RCC->CFGR.ADCPRE                //ADC prescaler
#define RCC_PLL_CLK             RCC->CFGR.PLLSRC                //PLL entry clock source
#define RCC_PLL_MUX             RCC->CFGR.PLLMUL                //PLL multiplication factor
#define RCC_MCO_CLK             RCC->CFGR.MCO                   //Microcontroller clock output
#define RCC_MCO_DIV             RCC->CFGR.MCOPRE                //Microcontroller Clock Output Prescaler
#define RCC_MCO_PLL_DIV         RCC->CFGR.PLLNODIV              //PLL clock not divided for MCO
#define RCC_LSI_IRQ_FLG         RCC->CIR.LSIRDYF                //LSI ready interrupt flag
#define RCC_LSE_IRQ_FLG         RCC->CIR.LSERDYF                //LSE ready interrupt flag
#define RCC_HSI_IRQ_FLG         RCC->CIR.HSIRDYF                //HSI ready interrupt flag
#define RCC_HSE_IRQ_FLG         RCC->CIR.HSERDYF                //HSE ready interrupt flag
#define RCC_PLL_IRQ_FLG         RCC->CIR.PLLRDYF                //PLL ready interrupt flag
#define RCC_HSI14_IRQ_FLG       RCC->CIR.HSI14RDYF              //HSI14 ready interrupt flag
#define RCC_HSI48_IRQ_FLG       RCC->CIR.HSI48RDYF              //HSI48 ready interrupt flag
#define RCC_CSS_IRQ_FLG         RCC->CIR.CSSF                   //Clock security system interrupt flag
#define RCC_LSI_IRQ             RCC->CIR.LSIRDYIE               //LSI ready interrupt enable
#define RCC_LSE_IRQ             RCC->CIR.LSERDYIE               //LSE ready interrupt enable
#define RCC_HSI_IRQ             RCC->CIR.HSIRDYIE               //HSI ready interrupt enable
#define RCC_HSE_IRQ             RCC->CIR.HSERDYIE               //HSE ready interrupt enable
#define RCC_PLL_IRQ             RCC->CIR.PLLRDYIE               //PLL ready interrupt enable
#define RCC_HSI14_IRQ           RCC->CIR.HSI14RDYIE             //HSI14 ready interrupt enable
#define RCC_HSI48_IRQ           RCC->CIR.HSI48RDYIE             //HSI48 ready interrupt enable
#define RCC_LSI_IRQ_CLR         RCC->CIR.LSIRDYC = ON           //LSI ready interrupt clear
#define RCC_LSE_IRQ_CLR         RCC->CIR.LSERDYC = ON           //LSE ready interrupt clear
#define RCC_HSI_IRQ_CLR         RCC->CIR.HSIRDYC = ON           //HSI ready interrupt clear
#define RCC_HSE_IRQ_CLR         RCC->CIR.HSERDYC = ON           //HSE ready interrupt clear
#define RCC_PLL_IRQ_CLR         RCC->CIR.PLLRDYC = ON           //PLL ready interrupt clear
#define RCC_HSI14_IRQ_CLR       RCC->CIR.HSI14RDYC = ON         //HSI14 ready interrupt clear
#define RCC_HSI48_IRQ_CLR       RCC->CIR.HSI48RDYC = ON         //HSI48 ready interrupt clear
#define RCC_CSS_IRQ_CLR         RCC->CIR.CSSC = ON              //Clock security system interrupt clear
#define RCC_SYS_RST             RCC->APB2RSTR.SYSCFGRST = ON    //SYSCFG reset
#define RCC_USART6_RST          RCC->APB2RSTR.USART6RST = ON    //USART6 reset
#define RCC_USART7_RST          RCC->APB2RSTR.USART7RST = ON    //USART7 reset
#define RCC_USART8_RST          RCC->APB2RSTR.USART8RST = ON    //USART8 reset
#define RCC_ADC_RST             RCC->APB2RSTR.ADCRST = ON       //ADC interface reset
#define RCC_TIM1_RST            RCC->APB2RSTR.TIM1RST = ON      //TIM1 timer reset
#define RCC_SPI1_RST            RCC->APB2RSTR.SPI1RST = ON      //SPI1 reset
#define RCC_USART1_RST          RCC->APB2RSTR.USART1RST = ON    //USART1 reset
#define RCC_TIM15_RST           RCC->APB2RSTR.TIM15RST = ON     //TIM15 timer reset
#define RCC_TIM16_RST           RCC->APB2RSTR.TIM16RST = ON     //TIM16 timer reset
#define RCC_TIM17_RST           RCC->APB2RSTR.TIM17RST = ON     //TIM17 timer reset
#define RCC_DBGMCU_RST          RCC->APB2RSTR.DBGMCURST = ON    //Debug MCU reset
#define RCC_TIM2_RST            RCC->APB1RSTR.TIM2RST = ON      //TIM2 timer reset
#define RCC_TIM3_RST            RCC->APB1RSTR.TIM3RST = ON      //TIM3 timer reset
#define RCC_TIM6_RST            RCC->APB1RSTR.TIM6RST = ON      //TIM6 timer reset
#define RCC_TIM7_RST            RCC->APB1RSTR.TIM7RST = ON      //TIM7 timer reset
#define RCC_TIM14_RST           RCC->APB1RSTR.TIM14RST = ON     //TIM14 timer reset
#define RCC_WWDG_RST            RCC->APB1RSTR.WWDGRST = ON      //Window watchdog reset
#define RCC_SPI2_RST            RCC->APB1RSTR.SPI2RST = ON      //SPI2 reset
#define RCC_UART2_RST           RCC->APB1RSTR.UART2RST = ON     //USART2 reset
#define RCC_UART3_RST           RCC->APB1RSTR.UART3RST = ON     //USART3 reset
#define RCC_UART4_RST           RCC->APB1RSTR.UART4RST = ON     //UART4 reset
#define RCC_UART5_RST           RCC->APB1RSTR.UART5RST = ON     //UART5 reset
#define RCC_I2C1_RST            RCC->APB1RSTR.I2C1RST = ON      //I2C1 reset
#define RCC_I2C2_RST            RCC->APB1RSTR.I2C2RST = ON      //I2C2 reset
#define RCC_USB_RST             RCC->APB1RSTR.USBRST = ON       //USB reset
#define RCC_CAN_RST             RCC->APB1RSTR.CANRST = ON       //CAN reset
#define RCC_CRS_RST             RCC->APB1RSTR.CRSRST = ON       //Clock Recovery System interface reset
#define RCC_PWR_RST             RCC->APB1RSTR.PWRRST = ON       //Power interface reset
#define RCC_DAC_RST             RCC->APB1RSTR.DACRST = ON       //DAC interface reset
#define RCC_CEC_RST             RCC->APB1RSTR.CECRST = ON       //HDMI CEC reset
#define RCC_DMA1_EN             RCC->AHBENR.DMA1EN              //DMA1 clock enable
#define RCC_DMA2_EN             RCC->AHBENR.DMA2EN              //DMA2 clock enable
#define RCC_SRAM_EN             RCC->AHBENR.SRAMEN              //SRAM interface clock enable
#define RCC_FLITF_EN            RCC->AHBENR.FLITFEN             //FLITF clock enable
#define RCC_CRC_EN              RCC->AHBENR.CRCEN               //CRC clock enable
#define RCC_GPIOA_EN            RCC->AHBENR.IOPAEN              //IO port A clock enable
#define RCC_GPIOB_EN            RCC->AHBENR.IOPBEN              //IO port B clock enable
#define RCC_GPIOC_EN            RCC->AHBENR.IOPCEN              //IO port C clock enable
#define RCC_GPIOD_EN            RCC->AHBENR.IOPDEN              //IO port D clock enable
#define RCC_GPIOE_EN            RCC->AHBENR.IOPEEN              //IO port E clock enable
#define RCC_GPIOF_EN            RCC->AHBENR.IOPFEN              //IO port F clock enable
#define RCC_TSC_EN              RCC->AHBENR.TSCEN               //Touch sensing controller clock enable
#define RCC_SYS_EN              RCC->APB2ENR.SYSCFGEN           //SYSCFG enable
#define RCC_UART6_EN            RCC->APB2ENR.USART6EN           //USART6 enable
#define RCC_UART7_EN            RCC->APB2ENR.USART7EN           //USART7 enable
#define RCC_UART8_EN            RCC->APB2ENR.USART8EN           //USART8 enable
#define RCC_ADC_EN              RCC->APB2ENR.ADCEN              //ADC interface enable
#define RCC_TIM1_EN             RCC->APB2ENR.TIM1EN             //TIM1 timer enable
#define RCC_SPI1_EN             RCC->APB2ENR.SPI1EN             //SPI1 enable
#define RCC_UART1_EN            RCC->APB2ENR.USART1EN           //USART1 enable
#define RCC_TIM15_EN            RCC->APB2ENR.TIM15EN            //TIM15 timer enable
#define RCC_TIM16_EN            RCC->APB2ENR.TIM16EN            //TIM16 timer enable
#define RCC_TIM17_EN            RCC->APB2ENR.TIM17EN            //TIM17 timer enable
#define RCC_DBGMCU_EN           RCC->APB2ENR.DBGMCUEN           //Debug MCU enable
#define RCC_TIM2_EN             RCC->APB1ENR.TIM2EN             //TIM2 timer enable
#define RCC_TIM3_EN             RCC->APB1ENR.TIM3EN             //TIM3 timer enable
#define RCC_TIM6_EN             RCC->APB1ENR.TIM6EN             //TIM6 timer enable
#define RCC_TIM7_EN             RCC->APB1ENR.TIM7REN            //TIM7 timer enable
#define RCC_TIM14_EN            RCC->APB1ENR.TIM14EN            //TIM14 timer enable
#define RCC_WWDG_EN             RCC->APB1ENR.WWDGEN             //Window watchdog enable
#define RCC_SPI2_EN             RCC->APB1ENR.SPI2EN             //SPI2 enable
#define RCC_UART2_EN            RCC->APB1ENR.UART2EN            //USART2 enable
#define RCC_UART3_EN            RCC->APB1ENR.UART3EN            //USART3 enable
#define RCC_UART4_EN            RCC->APB1ENR.UART4EN            //UART4 enable
#define RCC_UART5_EN            RCC->APB1ENR.UART5EN            //UART5 enable
#define RCC_I2C1_EN             RCC->APB1ENR.I2C1EN             //I2C1 enable
#define RCC_I2C2_EN             RCC->APB1ENR.I2C2EN             //I2C2 enable
#define RCC_USB_EN              RCC->APB1ENR.USBEN              //USB enable
#define RCC_CAN_EN              RCC->APB1ENR.CANEN              //CAN reset
#define RCC_CRS_EN              RCC->APB1ENR.CRSEN              //Clock Recovery System interface enable
#define RCC_PWR_EN              RCC->APB1ENR.PWREN              //Power interface enable
#define RCC_DAC_EN              RCC->APB1ENR.DACEN              //DAC interface enable
#define RCC_CEC_EN              RCC->APB1ENR.CECEN              //HDMI CEC enable
#define RCC_LSE_EN              RCC->BDCR.LSEON                 //External low-speed oscillator enable
#define RCC_LSE_RDY             RCC->BDCR.LSERDY                //External low-speed oscillator ready
#define RCC_LSE_BYP             RCC->BDCR.LSEBYP                //External low-speed oscillator bypass
#define RCC_LSE_DRV             RCC->BDCR.LSEDRV                //LSE oscillator drive capability
#define RCC_RTC_CLK             RCC->BDCR.RTCSEL                //RTC clock source selection
#define RCC_RTC_EN              RCC->BDCR.RTCEN                 //RTC clock enable
#define RCC_RTC_RST             RCC->BDCR.BDRST = ON            //Backup domain software reset
#define RCC_LSI_EN              RCC->CSR.LSION                  //Internal low-speed oscillator enable
#define RCC_LSI_RDY             RCC->CSR.LSIRDY                 //Internal low-speed oscillator ready
#define RCC_V18RST_FLG          RCC->CSR.V18PWRRSTF             //Reset flag of the 1.8 V domain.
#define RCC_RMVRST_FLG          RCC->CSR.RMVF                   //Remove reset flag
#define RCC_OBLRST_FLG          RCC->CSR.OBLRSTF                //Option byte loader reset flag
#define RCC_PINRST_FLG          RCC->CSR.PINRSTF                //PIN reset flag
#define RCC_PORRST_FLG          RCC->CSR.PORRSTF                //POR/PDR reset flag
#define RCC_SFTRST_FLG          RCC->CSR.SFTRSTF                //Software reset flag
#define RCC_IWDGRST_FLG         RCC->CSR.IWDGRSTF               //Independent watchdog reset flag
#define RCC_WWDGRST_FLG         RCC->CSR.WWDGRSTF               //Window watchdog reset flag
#define RCC_LPWRRST_FLG         RCC->CSR.LPWRRSTF               //Low-power reset flag
#define RCC_GPIOA_RST           RCC->AHBRSTR.IOPARST = ON       //I/O port A reset
#define RCC_GPIOB_RST           RCC->AHBRSTR.IOPBRST = ON       //I/O port B reset
#define RCC_GPIOC_RST           RCC->AHBRSTR.IOPCRST = ON       //I/O port C reset
#define RCC_GPIOD_RST           RCC->AHBRSTR.IOPDRST = ON       //I/O port D reset
#define RCC_GPIOE_RST           RCC->AHBRSTR.IOPERST = ON       //I/O port E reset
#define RCC_GPIOF_RST           RCC->AHBRSTR.IOPFRST = ON       //I/O port F reset
#define RCC_TSC_RST             RCC->AHBRSTR.TSCRST = ON        //Touch sensing controller reset
#define RCC_HSE_DIV             RCC->CFGR2.PREDIV               //PREDIV division factor
#define RCC_USART1_SW           RCC->CFGR3.USART1SW             //USART1 clock source selection
#define RCC_I2C1_SW             RCC->CFGR3.I2C1SW               //I2C1 clock source selection
#define RCC_CEC_SW              RCC->CFGR3.CECSW                //HDMI CEC clock source selection
#define RCC_USB_SW              RCC->CFGR3.USBSW                //USB clock source selection
#define RCC_ADC_SW              RCC->CFGR3.ADCSW                //ADC clock source selection
#define RCC_USART2_SW           RCC->CFGR3.USART2SW             //USART2 clock source selection (available only on STM32F07x and STM32F09x devices)
#define RCC_USART3_SW           RCC->CFGR3.USART3SW             //USART3 clock source selection (available only on STM32F09x devices)
#define RCC_HSI14_EN            RCC->CR2.HSI14ON                //HSI14 clock enable
#define RCC_HSI14_RDY           RCC->CR2.HSI14RDY               //HSI14 clock ready flag
#define RCC_HSI14_DIS           RCC->CR2.HSI14DIS               //HSI14 clock request from ADC disable
#define RCC_HSI14_TRIM          RCC->CR2.HSI14TRIM              //HSI14 clock trimming
#define RCC_HSI14_CAL           RCC->CR2.HSI14CAL               //HSI14 clock calibration
#define RCC_HSI48_EN            RCC->CR2.HSI48ON                //HSI48 clock enable
#define RCC_HSI48_RDY           RCC->CR2.HSI48RDY               //HSI48 clock ready flag
#define RCC_HSI48_CAL           RCC->CR2.HSI48CAL               //HSI48 factory clock calibration
#define RCC_HSI_CLK_7360000     ((u8) 0x00)
#define RCC_HSI_CLK_7400000     ((u8) 0x01)
#define RCC_HSI_CLK_7440000     ((u8) 0x02)
#define RCC_HSI_CLK_7480000     ((u8) 0x03)
#define RCC_HSI_CLK_7520000     ((u8) 0x04)
#define RCC_HSI_CLK_7560000     ((u8) 0x05)
#define RCC_HSI_CLK_7600000     ((u8) 0x06)
#define RCC_HSI_CLK_7640000     ((u8) 0x07)
#define RCC_HSI_CLK_7680000     ((u8) 0x08)
#define RCC_HSI_CLK_7720000     ((u8) 0x09)
#define RCC_HSI_CLK_7760000     ((u8) 0x0A)
#define RCC_HSI_CLK_7800000     ((u8) 0x0B)
#define RCC_HSI_CLK_7840000     ((u8) 0x0C)
#define RCC_HSI_CLK_7880000     ((u8) 0x0D)
#define RCC_HSI_CLK_7920000     ((u8) 0x0E)
#define RCC_HSI_CLK_7960000     ((u8) 0x0F)
#define RCC_HSI_CLK_8000000     ((u8) 0x10)
#define RCC_HSI_CLK_8040000     ((u8) 0x11)
#define RCC_HSI_CLK_8080000     ((u8) 0x12)
#define RCC_HSI_CLK_8120000     ((u8) 0x13)
#define RCC_HSI_CLK_8160000     ((u8) 0x14)
#define RCC_HSI_CLK_8200000     ((u8) 0x15)
#define RCC_HSI_CLK_8240000     ((u8) 0x16)
#define RCC_HSI_CLK_8280000     ((u8) 0x17)
#define RCC_HSI_CLK_8320000     ((u8) 0x18)
#define RCC_HSI_CLK_8360000     ((u8) 0x19)
#define RCC_HSI_CLK_8400000     ((u8) 0x1A)
#define RCC_HSI_CLK_8440000     ((u8) 0x1B)
#define RCC_HSI_CLK_8480000     ((u8) 0x1C)
#define RCC_HSI_CLK_8520000     ((u8) 0x1D)
#define RCC_HSI_CLK_8560000     ((u8) 0x1E)
#define RCC_HSI_CLK_8600000     ((u8) 0x1F)
#define RCC_SYS_HSI             ((u8) 0x00)                     //HSI selected as system clock
#define RCC_SYS_HSE             ((u8) 0x01)                     //HSE selected as system clock
#define RCC_SYS_PLL             ((u8) 0x02)                     //PLL selected as system clock
#define RCC_SYS_HSI48           ((u8) 0x03)                     //HSI48 selected as system clock (when available)
#define RCC_AHB_DIV001          ((u8) 0x00)                     //SYSCLK not divided
#define RCC_AHB_DIV002          ((u8) 0x08)                     //SYSCLK divided by 2
#define RCC_AHB_DIV004          ((u8) 0x09)                     //SYSCLK divided by 4
#define RCC_AHB_DIV008          ((u8) 0x0A)                     //SYSCLK divided by 8
#define RCC_AHB_DIV016          ((u8) 0x0B)                     //SYSCLK divided by 16
#define RCC_AHB_DIV064          ((u8) 0x0C)                     //SYSCLK divided by 64
#define RCC_AHB_DIV128          ((u8) 0x0D)                     //SYSCLK divided by 128
#define RCC_AHB_DIV256          ((u8) 0x0E)                     //SYSCLK divided by 256
#define RCC_AHB_DIV512          ((u8) 0x0F)                     //SYSCLK divided by 512
#define RCC_APB_DIV01           ((u8) 0x00)                     //HCLK not divided
#define RCC_APB_DIV02           ((u8) 0x04)                     //HCLK divided by 2
#define RCC_APB_DIV04           ((u8) 0x05)                     //HCLK divided by 4
#define RCC_APB_DIV08           ((u8) 0x06)                     //HCLK divided by 8
#define RCC_APB_DIV16           ((u8) 0x07)                     //HCLK divided by 16
#define RCC_ADC_DIV2            ((u8) 0x00)                     //PCLK2 divided by 2
#define RCC_ADC_DIV4            ((u8) 0x01)                     //PCLK2 divided by 4
#define RCC_ADC_DIV6            ((u8) 0x02)                     //PCLK2 divided by 6
#define RCC_ADC_DIV8            ((u8) 0x03)                     //PCLK2 divided by 8
#define RCC_PLLSRC_HSI          ((u8) 0x00)                     //HSI/2 selected as PLL input clock
#define RCC_PLLSRC_HSIDIV       ((u8) 0x01)                     //HSI/PREDIV selected as PLL input clock
#define RCC_PLLSRC_HSE          ((u8) 0x02)                     //HSE/PREDIV selected as PLL input clock
#define RCC_PLLSRC_HSI48        ((u8) 0x03)                     //HSI48/PREDIV selected as PLL input clock
#define RCC_DIV01               ((u8) 0x00)                     //HSE clock not divided
#define RCC_DIV02               ((u8) 0x01)                     //HSE clock divided by 2
#define RCC_DIV03               ((u8) 0x02)                     //HSE clock divided by 3
#define RCC_DIV04               ((u8) 0x03)                     //HSE clock divided by 4
#define RCC_DIV05               ((u8) 0x04)                     //HSE clock divided by 5
#define RCC_DIV06               ((u8) 0x05)                     //HSE clock divided by 6
#define RCC_DIV07               ((u8) 0x06)                     //HSE clock divided by 7
#define RCC_DIV08               ((u8) 0x07)                     //HSE clock divided by 8
#define RCC_DIV09               ((u8) 0x08)                     //HSE clock divided by 9
#define RCC_DIV10               ((u8) 0x09)                     //HSE clock divided by 10
#define RCC_DIV11               ((u8) 0x0A)                     //HSE clock divided by 11
#define RCC_DIV12               ((u8) 0x0B)                     //HSE clock divided by 12
#define RCC_DIV13               ((u8) 0x0C)                     //HSE clock divided by 13
#define RCC_DIV14               ((u8) 0x0D)                     //HSE clock divided by 14
#define RCC_DIV15               ((u8) 0x0E)                     //HSE clock divided by 15
#define RCC_DIV16               ((u8) 0x0F)                     //HSE clock divided by 16
#define RCC_PLL_MUX02           ((u8) 0x00)                     //PLL input clock x 2
#define RCC_PLL_MUX03           ((u8) 0x01)                     //PLL input clock x 3
#define RCC_PLL_MUX04           ((u8) 0x02)                     //PLL input clock x 4
#define RCC_PLL_MUX05           ((u8) 0x03)                     //PLL input clock x 5
#define RCC_PLL_MUX06           ((u8) 0x04)                     //PLL input clock x 6
#define RCC_PLL_MUX07           ((u8) 0x05)                     //PLL input clock x 7
#define RCC_PLL_MUX08           ((u8) 0x06)                     //PLL input clock x 8
#define RCC_PLL_MUX09           ((u8) 0x07)                     //PLL input clock x 9
#define RCC_PLL_MUX10           ((u8) 0x08)                     //PLL input clock x 10
#define RCC_PLL_MUX11           ((u8) 0x09)                     //PLL input clock x 11
#define RCC_PLL_MUX12           ((u8) 0x0A)                     //PLL input clock x 12
#define RCC_PLL_MUX13           ((u8) 0x0B)                     //PLL input clock x 13
#define RCC_PLL_MUX14           ((u8) 0x0C)                     //PLL input clock x 14
#define RCC_PLL_MUX15           ((u8) 0x0D)                     //PLL input clock x 15
#define RCC_PLL_MUX16           ((u8) 0x0E)                     //PLL input clock x 16
#define RCC_USB_HSI48           ((u8) 0x00)                     //HSI48 clock selected as USB clock source (default)
#define RCC_USB_PLLCLK          ((u8) 0x01)                     //PLL clock (PLLCLK) selected as USB clock
#define RCC_MCO_NOCLK           ((u8) 0x00)                     //MCO output disabled, no clock on MCO
#define RCC_MCO_HSI14           ((u8) 0x01)                     //Internal RC 14 MHz (HSI14) oscillator clock selected
#define RCC_MCO_LSI             ((u8) 0x02)                     //Internal low speed (LSI) oscillator clock selected
#define RCC_MCO_LSE             ((u8) 0x03)                     //External low speed (LSE) oscillator clock selected
#define RCC_MCO_SYS             ((u8) 0x04)                     //System clock (SYSCLK) selected
#define RCC_MCO_HSI             ((u8) 0x05)                     //HSI clock selected
#define RCC_MCO_HSE             ((u8) 0x06)                     //HSE clock selected
#define RCC_MCO_PLL             ((u8) 0x07)                     //PLL clock selected (divided by 1 or 2, depending on PLLNODIV)
#define RCC_MCO_HSI48           ((u8) 0x08)                     //Internal RC 48 MHz (HSI48) oscillator clock selected
#define RCC_MCO_DIV001          ((u8) 0x00)                     //MCO is divided by 1
#define RCC_MCO_DIV002          ((u8) 0x01)                     //MCO is divided by 2
#define RCC_MCO_DIV004          ((u8) 0x02)                     //MCO is divided by 4
#define RCC_MCO_DIV008          ((u8) 0x03)                     //MCO is divided by 8
#define RCC_MCO_DIV016          ((u8) 0x04)                     //MCO is divided by 16
#define RCC_MCO_DIV032          ((u8) 0x05)                     //MCO is divided by 32
#define RCC_MCO_DIV064          ((u8) 0x06)                     //MCO is divided by 64
#define RCC_MCO_DIV128          ((u8) 0x07)                     //MCO is divided by 128
#define RCC_LSE_CAP1            ((u8) 0x00)                     //lower driving capability
#define RCC_LSE_CAP2            ((u8) 0x01)                     //medium low driving capability
#define RCC_LSE_CAP3            ((u8) 0x02)                     //medium high driving capability
#define RCC_LSE_CAP4            ((u8) 0x03)                     //higher driving capability (reset value)
#define RCC_USART_PCLK          ((u8) 0x00)                     //PCLK selected as USART3 clock source (default)
#define RCC_USART_SYSCLK        ((u8) 0x01)                     //System clock (SYSCLK) selected as USART3 clock
#define RCC_USART_LSE           ((u8) 0x02)                     //LSE clock selected as USART3 clock
#define RCC_USART_HSI           ((u8) 0x03)                     //HSI clock selected as USART3 clock
/****************************************** GPIO **************************************************/
typedef struct
{
  struct
  {
    u1 MD00       : 2;  //Port x configuration bits 0
    u1 MD01       : 2;  //Port x configuration bits 1
    u1 MD02       : 2;  //Port x configuration bits 2
    u1 MD03       : 2;  //Port x configuration bits 3
    u1 MD04       : 2;  //Port x configuration bits 4
    u1 MD05       : 2;  //Port x configuration bits 5
    u1 MD06       : 2;  //Port x configuration bits 6
    u1 MD07       : 2;  //Port x configuration bits 7
    u1 MD08       : 2;  //Port x configuration bits 8
    u1 MD09       : 2;  //Port x configuration bits 9
    u1 MD10       : 2;  //Port x configuration bits 10
    u1 MD11       : 2;  //Port x configuration bits 11
    u1 MD12       : 2;  //Port x configuration bits 12
    u1 MD13       : 2;  //Port x configuration bits 13
    u1 MD14       : 2;  //Port x configuration bits 14
    u1 MD15       : 2;  //Port x configuration bits 15
  }volatile MODER;            //GPIO port mode register (GPIOx_MODER) (x=A..F)
  struct
  {
    u1 OT00       : 1;  //Port x configuration bits 0
    u1 OT01       : 1;  //Port x configuration bits 1
    u1 OT02       : 1;  //Port x configuration bits 2
    u1 OT03       : 1;  //Port x configuration bits 3
    u1 OT04       : 1;  //Port x configuration bits 4
    u1 OT05       : 1;  //Port x configuration bits 5
    u1 OT06       : 1;  //Port x configuration bits 6
    u1 OT07       : 1;  //Port x configuration bits 7
    u1 OT08       : 1;  //Port x configuration bits 8
    u1 OT09       : 1;  //Port x configuration bits 9
    u1 OT10       : 1;  //Port x configuration bits 10
    u1 OT11       : 1;  //Port x configuration bits 11
    u1 OT12       : 1;  //Port x configuration bits 12
    u1 OT13       : 1;  //Port x configuration bits 13
    u1 OT14       : 1;  //Port x configuration bits 14
    u1 OT15       : 1;  //Port x configuration bits 15
    u1 RSVD0      : 16; //Reserved
  }volatile OTYPER;           //GPIO port output type register (GPIOx_OTYPER) (x=A..F)
  struct
  {
    u1 SP00       : 2;  //Port x configuration bits 0
    u1 SP01       : 2;  //Port x configuration bits 1
    u1 SP02       : 2;  //Port x configuration bits 2
    u1 SP03       : 2;  //Port x configuration bits 3
    u1 SP04       : 2;  //Port x configuration bits 4
    u1 SP05       : 2;  //Port x configuration bits 5
    u1 SP06       : 2;  //Port x configuration bits 6
    u1 SP07       : 2;  //Port x configuration bits 7
    u1 SP08       : 2;  //Port x configuration bits 8
    u1 SP09       : 2;  //Port x configuration bits 9
    u1 SP10       : 2;  //Port x configuration bits 10
    u1 SP11       : 2;  //Port x configuration bits 11
    u1 SP12       : 2;  //Port x configuration bits 12
    u1 SP13       : 2;  //Port x configuration bits 13
    u1 SP14       : 2;  //Port x configuration bits 14
    u1 SP15       : 2;  //Port x configuration bits 15
  }volatile OSPEEDR;          //GPIO port output speed register (GPIOx_OSPEEDR) (x=A..F)
  struct
  {
    u1 PP00       : 2;  //Port x configuration bits 0
    u1 PP01       : 2;  //Port x configuration bits 1
    u1 PP02       : 2;  //Port x configuration bits 2
    u1 PP03       : 2;  //Port x configuration bits 3
    u1 PP04       : 2;  //Port x configuration bits 4
    u1 PP05       : 2;  //Port x configuration bits 5
    u1 PP06       : 2;  //Port x configuration bits 6
    u1 PP07       : 2;  //Port x configuration bits 7
    u1 PP08       : 2;  //Port x configuration bits 8
    u1 PP09       : 2;  //Port x configuration bits 9
    u1 PP10       : 2;  //Port x configuration bits 10
    u1 PP11       : 2;  //Port x configuration bits 11
    u1 PP12       : 2;  //Port x configuration bits 12
    u1 PP13       : 2;  //Port x configuration bits 13
    u1 PP14       : 2;  //Port x configuration bits 14
    u1 PP15       : 2;  //Port x configuration bits 15
  }volatile PUPDR;            //GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x=A..F)
  struct
  {
    u1 IR         : 16; //Port input data register
    u1 RSVD0      : 16; //Reserved
  }volatile IDR;              //Port input data register (GPIO_IDR) (x=A..G)
  struct
  {
    u1 OR         : 16; //Port output data register
    u1 RSVD0      : 16; //Reserved
  }volatile ODR;              //Port output data register (GPIO_ODR) (x=A..G)
  struct
  {
    u1 BS         : 16; //Port x Set bit y (y= 0 .. 15)
    u1 BR         : 16; //Port x Reset bit y (y= 0 .. 15)
  }volatile BSRR;             //Port bit set/reset register (GPIO_BSRR) (x=A..G)
  struct
  {
    u1 LCK        : 16; //Port x Lock bit y (y= 0 .. 15)
    u1 LCKK       : 1;  //Lock key
    u1 RSVD0      : 15; //Reserved
  }volatile LCKR;             //Port configuration lock register (GPIO_LCKR) (x=A..G)
  struct
  {
    u1 AF00       : 4;  // Alternate function selection for port x pin y
    u1 AF01       : 4;  // Alternate function selection for port x pin y
    u1 AF02       : 4;  // Alternate function selection for port x pin y
    u1 AF03       : 4;  // Alternate function selection for port x pin y
    u1 AF04       : 4;  // Alternate function selection for port x pin y
    u1 AF05       : 4;  // Alternate function selection for port x pin y
    u1 AF06       : 4;  // Alternate function selection for port x pin y
    u1 AF07       : 4;  // Alternate function selection for port x pin y
  }volatile AFRL;             //GPIO alternate function low register (GPIOx_AFRL) (x=A..G)
  struct
  {
    u1 AF08       : 4;  // Alternate function selection for port x pin y
    u1 AF09       : 4;  // Alternate function selection for port x pin y
    u1 AF10       : 4;  // Alternate function selection for port x pin y
    u1 AF11       : 4;  // Alternate function selection for port x pin y
    u1 AF12       : 4;  // Alternate function selection for port x pin y
    u1 AF13       : 4;  // Alternate function selection for port x pin y
    u1 AF14       : 4;  // Alternate function selection for port x pin y
    u1 AF15       : 4;  // Alternate function selection for port x pin y
  }volatile AFRH;             //GPIO alternate function high register (GPIOx_AFRH) (x=A..G)
  struct
  {
    u1 BR         : 16; //Port x Reset bit y
    u1 RSVD0      : 16; //Reserved
  }volatile BRR;              //GPIO port bit reset register (GPIOx_BRR) (x=A..G)
}GPIO_Type;
#define GPIOA ((GPIO_Type*) GPIOA_BASE)
#define GPIOB ((GPIO_Type*) GPIOB_BASE)
#define GPIOC ((GPIO_Type*) GPIOC_BASE)
#define GPIOD ((GPIO_Type*) GPIOD_BASE)
#define GPIOE ((GPIO_Type*) GPIOE_BASE)
#define GPIOF ((GPIO_Type*) GPIOF_BASE)
#define GPIO_CFG_PIN00(PORT, MODE) PORT->MODER.MD00 = (MODE & 3); PORT->PUPDR.PP00 = ((MODE >> 2) & 3); PORT->OTYPER.OT00 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP00 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x0); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x0) //Port bit 00 configuration
#define GPIO_CFG_PIN01(PORT, MODE) PORT->MODER.MD01 = (MODE & 3); PORT->PUPDR.PP01 = ((MODE >> 2) & 3); PORT->OTYPER.OT01 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP01 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x1); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x1) //Port bit 01 configuration
#define GPIO_CFG_PIN02(PORT, MODE) PORT->MODER.MD02 = (MODE & 3); PORT->PUPDR.PP02 = ((MODE >> 2) & 3); PORT->OTYPER.OT02 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP02 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x2); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x2) //Port bit 02 configuration
#define GPIO_CFG_PIN03(PORT, MODE) PORT->MODER.MD03 = (MODE & 3); PORT->PUPDR.PP03 = ((MODE >> 2) & 3); PORT->OTYPER.OT03 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP03 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x3); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x3) //Port bit 03 configuration
#define GPIO_CFG_PIN04(PORT, MODE) PORT->MODER.MD04 = (MODE & 3); PORT->PUPDR.PP04 = ((MODE >> 2) & 3); PORT->OTYPER.OT04 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP04 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x4); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x4) //Port bit 04 configuration
#define GPIO_CFG_PIN05(PORT, MODE) PORT->MODER.MD05 = (MODE & 3); PORT->PUPDR.PP05 = ((MODE >> 2) & 3); PORT->OTYPER.OT05 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP05 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x5); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x5) //Port bit 05 configuration
#define GPIO_CFG_PIN06(PORT, MODE) PORT->MODER.MD06 = (MODE & 3); PORT->PUPDR.PP06 = ((MODE >> 2) & 3); PORT->OTYPER.OT06 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP06 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x6); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x6) //Port bit 06 configuration
#define GPIO_CFG_PIN07(PORT, MODE) PORT->MODER.MD07 = (MODE & 3); PORT->PUPDR.PP07 = ((MODE >> 2) & 3); PORT->OTYPER.OT07 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP07 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x7); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x7) //Port bit 07 configuration
#define GPIO_CFG_PIN08(PORT, MODE) PORT->MODER.MD08 = (MODE & 3); PORT->PUPDR.PP08 = ((MODE >> 2) & 3); PORT->OTYPER.OT08 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP08 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x8); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x8) //Port bit 08 configuration
#define GPIO_CFG_PIN09(PORT, MODE) PORT->MODER.MD09 = (MODE & 3); PORT->PUPDR.PP09 = ((MODE >> 2) & 3); PORT->OTYPER.OT09 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP09 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0x9); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0x9) //Port bit 09 configuration
#define GPIO_CFG_PIN10(PORT, MODE) PORT->MODER.MD10 = (MODE & 3); PORT->PUPDR.PP10 = ((MODE >> 2) & 3); PORT->OTYPER.OT10 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP10 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xA); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xA) //Port bit 10 configuration
#define GPIO_CFG_PIN11(PORT, MODE) PORT->MODER.MD11 = (MODE & 3); PORT->PUPDR.PP11 = ((MODE >> 2) & 3); PORT->OTYPER.OT11 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP11 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xB); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xB) //Port bit 11 configuration
#define GPIO_CFG_PIN12(PORT, MODE) PORT->MODER.MD12 = (MODE & 3); PORT->PUPDR.PP12 = ((MODE >> 2) & 3); PORT->OTYPER.OT12 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP12 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xC); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xC) //Port bit 12 configuration
#define GPIO_CFG_PIN13(PORT, MODE) PORT->MODER.MD13 = (MODE & 3); PORT->PUPDR.PP13 = ((MODE >> 2) & 3); PORT->OTYPER.OT13 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP13 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xD); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xD) //Port bit 13 configuration
#define GPIO_CFG_PIN14(PORT, MODE) PORT->MODER.MD14 = (MODE & 3); PORT->PUPDR.PP14 = ((MODE >> 2) & 3); PORT->OTYPER.OT14 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP14 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xE); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xE) //Port bit 14 configuration
#define GPIO_CFG_PIN15(PORT, MODE) PORT->MODER.MD15 = (MODE & 3); PORT->PUPDR.PP15 = ((MODE >> 2) & 3); PORT->OTYPER.OT15 = ((MODE >> 4) & 3); PORT->OSPEEDR.SP15 = ((MODE >> 6) & 3); PORT->BSRR.BS = (((MODE >> 2) & 1) << 0xF); PORT->BSRR.BR = (((MODE >> 3) & 1) << 0xF) //Port bit 15 configuration
#define GPIO_IN(PORT)           PORT->IDR.IR                    //Port input data register
#define GPIO_OUT(PORT)          PORT->ODR.OR                    //Port output data register
#define GPIO_BIT_SET(PORT)      PORT->BSRR.BS                   //Port Set bit
#define GPIO_BIT_RST(PORT)      PORT->BSRR.BR                   //Port Reset bit
#define GPIO_LCK(PORT)          PORT->LCKR.LCK                  //Port Lock bit
#define GPIO_LCK_KEY(PORT)      PORT->LCKR.LCKK                 //Port Lock key
#define GPIO_AF_PIN00(PORT)     PORT->AFRL.AF00                 // Alternate function selection for port x pin 0
#define GPIO_AF_PIN01(PORT)     PORT->AFRL.AF01                 // Alternate function selection for port x pin 1
#define GPIO_AF_PIN02(PORT)     PORT->AFRL.AF02                 // Alternate function selection for port x pin 2
#define GPIO_AF_PIN03(PORT)     PORT->AFRL.AF03                 // Alternate function selection for port x pin 3
#define GPIO_AF_PIN04(PORT)     PORT->AFRL.AF04                 // Alternate function selection for port x pin 4
#define GPIO_AF_PIN05(PORT)     PORT->AFRL.AF05                 // Alternate function selection for port x pin 5
#define GPIO_AF_PIN06(PORT)     PORT->AFRL.AF06                 // Alternate function selection for port x pin 6
#define GPIO_AF_PIN07(PORT)     PORT->AFRL.AF07                 // Alternate function selection for port x pin 7
#define GPIO_AF_PIN08(PORT)     PORT->AFRH.AF08                 // Alternate function selection for port x pin 8
#define GPIO_AF_PIN09(PORT)     PORT->AFRH.AF09                 // Alternate function selection for port x pin 9
#define GPIO_AF_PIN10(PORT)     PORT->AFRH.AF10                 // Alternate function selection for port x pin 10
#define GPIO_AF_PIN11(PORT)     PORT->AFRH.AF11                 // Alternate function selection for port x pin 11
#define GPIO_AF_PIN12(PORT)     PORT->AFRH.AF12                 // Alternate function selection for port x pin 12
#define GPIO_AF_PIN13(PORT)     PORT->AFRH.AF13                 // Alternate function selection for port x pin 13
#define GPIO_AF_PIN14(PORT)     PORT->AFRH.AF14                 // Alternate function selection for port x pin 14
#define GPIO_AF_PIN15(PORT)     PORT->AFRH.AF15                 // Alternate function selection for port x pin 15
#define GPIO_ANALOG             ((u8) 0x03)                     //Input/output Analog
#define GPIO_IN_FLOAT           ((u8) 0x00)                     //Input Floating
#define GPIO_IN_PU              ((u8) 0x04)                     //Input UP
#define GPIO_IN_PD              ((u8) 0x08)                     //Input PD
#define GPIO_OUT_PP_NO_L        ((u8) 0x01)                     //GP output PP
#define GPIO_OUT_PP_NO_M        ((u8) 0x41)                     //GP output PP
#define GPIO_OUT_PP_NO_H        ((u8) 0xC1)                     //GP output PP
#define GPIO_OUT_PP_PU_L        ((u8) 0x05)                     //GP output PP + PU
#define GPIO_OUT_PP_PU_M        ((u8) 0x45)                     //GP output PP + PU
#define GPIO_OUT_PP_PU_H        ((u8) 0xC5)                     //GP output PP + PU
#define GPIO_OUT_PP_PD_L        ((u8) 0x09)                     //GP output PP + PD
#define GPIO_OUT_PP_PD_M        ((u8) 0x49)                     //GP output PP + PD
#define GPIO_OUT_PP_PD_H        ((u8) 0xC9)                     //GP output PP + PD
#define GPIO_OUT_OD_NO_L        ((u8) 0x11)                     //GP output OD
#define GPIO_OUT_OD_NO_M        ((u8) 0x51)                     //GP output OD
#define GPIO_OUT_OD_NO_H        ((u8) 0xD1)                     //GP output OD
#define GPIO_OUT_OD_PU_L        ((u8) 0x15)                     //GP output OD + PU
#define GPIO_OUT_OD_PU_M        ((u8) 0x55)                     //GP output OD + PU
#define GPIO_OUT_OD_PU_H        ((u8) 0xD5)                     //GP output OD + PU
#define GPIO_OUT_OD_PD_L        ((u8) 0x19)                     //GP output OD + PD
#define GPIO_OUT_OD_PD_M        ((u8) 0x59)                     //GP output OD + PD
#define GPIO_OUT_OD_PD_H        ((u8) 0xD9)                     //GP output OD + PD
#define GPIO_AF_PP_NO_L         ((u8) 0x02)                     //AF PP
#define GPIO_AF_PP_NO_M         ((u8) 0x42)                     //AF PP
#define GPIO_AF_PP_NO_H         ((u8) 0xC2)                     //AF PP
#define GPIO_AF_PP_PU_L         ((u8) 0x06)                     //AF PP + PU
#define GPIO_AF_PP_PU_M         ((u8) 0x46)                     //AF PP + PU
#define GPIO_AF_PP_PU_H         ((u8) 0xC6)                     //AF PP + PU
#define GPIO_AF_PP_PD_L         ((u8) 0x0A)                     //AF PP + PD
#define GPIO_AF_PP_PD_M         ((u8) 0x4A)                     //AF PP + PD
#define GPIO_AF_PP_PD_H         ((u8) 0xCA)                     //AF PP + PD
#define GPIO_AF_OD_NO_L         ((u8) 0x12)                     //AF OD
#define GPIO_AF_OD_NO_M         ((u8) 0x52)                     //AF OD
#define GPIO_AF_OD_NO_H         ((u8) 0xD2)                     //AF OD
#define GPIO_AF_OD_PU_L         ((u8) 0x16)                     //AF OD + PU
#define GPIO_AF_OD_PU_M         ((u8) 0x56)                     //AF OD + PU
#define GPIO_AF_OD_PU_H         ((u8) 0xD6)                     //AF OD + PU
#define GPIO_AF_OD_PD_L         ((u8) 0x1A)                     //AF OD + PD
#define GPIO_AF_OD_PD_M         ((u8) 0x5A)                     //AF OD + PD
#define GPIO_AF_OD_PD_H         ((u8) 0xDA)                     //AF OD + PD
#define GPIO_PIN00              ((u16) 0x0001)
#define GPIO_PIN01              ((u16) 0x0002)
#define GPIO_PIN02              ((u16) 0x0004)
#define GPIO_PIN03              ((u16) 0x0008)
#define GPIO_PIN04              ((u16) 0x0010)
#define GPIO_PIN05              ((u16) 0x0020)
#define GPIO_PIN06              ((u16) 0x0040)
#define GPIO_PIN07              ((u16) 0x0080)
#define GPIO_PIN08              ((u16) 0x0100)
#define GPIO_PIN09              ((u16) 0x0200)
#define GPIO_PIN10              ((u16) 0x0400)
#define GPIO_PIN11              ((u16) 0x0800)
#define GPIO_PIN12              ((u16) 0x1000)
#define GPIO_PIN13              ((u16) 0x2000)
#define GPIO_PIN14              ((u16) 0x4000)
#define GPIO_PIN15              ((u16) 0x8000)
#define GPIO_PINAll             ((u16) 0xFFFF)
#define GPIO_AF0                ((u8) 0x00)
#define GPIO_AF1                ((u8) 0x01)
#define GPIO_AF2                ((u8) 0x02)
#define GPIO_AF3                ((u8) 0x03)
#define GPIO_AF4                ((u8) 0x04)
#define GPIO_AF5                ((u8) 0x05)
#define GPIO_AF6                ((u8) 0x06)
#define GPIO_AF7                ((u8) 0x07)
/******************************************* FLASH **************************************************/
typedef struct
{
  struct
  {
    u1 LATENCY          : 3;  //Latency
    u1 RSVD0            : 1; //Reserved
    u1 PRFTBE           : 1;  //Prefetch buffer enable
    u1 PRFTBS           : 1;  //Prefetch buffer status
    u1 RSVD1            : 26; //Reserved
  }volatile ACR;        //Flash access control register (FLASH_ACR)
struct
  {
    u32 FKEY;           //Flash key
  }volatile KEYR;        //Flash key register (FLASH_KEYR)
struct
  {
    u32 OPTKEY;         //Option byte key
  }volatile OPTKEYR;        //Flash option key register (FLASH_OPTKEYR)
struct
  {
    u1 BSY              : 1;  //Busy
    u1 RSVD0            : 1;  //Reserved
    u1 PGERR            : 1;  //Programming error
    u1 RSVD1            : 1;  //Reserved
    u1 WRPRTERR         : 1;  //Write protection error
    u1 EOP              : 1;  //End of operation
    u1 RSVD2            : 26;  //Reserved
  }volatile SR;      //Flash status register (FLASH_SR)
struct
  {
    u1 PG               : 1;  //Busy
    u1 PER              : 1;  //Reserved
    u1 MER              : 1;  //Programming error
    u1 RSVD0            : 1;  //Reserved
    u1 OPTPG            : 1;  //Reserved
    u1 OPTER            : 1;  //Write protection error
    u1 STRT             : 1;  //End of operation
    u1 LOCK             : 1;  //Busy
    u1 RSVD1            : 1;  //Reserved
    u1 OPTWRE           : 1;  //Programming error
    u1 ERRIE            : 1;  //Reserved
    u1 RSVD2            : 1;  //Reserved
    u1 EOPIE            : 1;  //Write protection error
    u1 OBL_LAUNCH       : 1;  //End of operation
    u1 RSVD3            : 18; //Reserved
  }volatile CR;      //Flash control register (FLASH_CR)
struct
  {
    u32 FAR;            //Flash Address
  }volatile AR;       //Flash address register (FLASH_AR)
struct
  {
    u1 OPTERR           : 1; //Analog watchdog lower threshold
    u1 RDPRT            : 2;  //Reserved
    u1 RSVD0            : 5;  //Reserved
    u1 WDG_SW           : 1; //Analog watchdog higher threshold
    u1 nRST_STOP        : 1;  //Reserved
    u1 nRST_STDBY       : 1;  //Busy
    u1 nBOOT0           : 1;  //Reserved
    u1 nBOOT1           : 1;  //Programming error
    u1 VDDA_MONITOR     : 1;  //Reserved
    u1 RAM_PARITY_CHECK : 1;  //Reserved
    u1 BOOT_SEL         : 1;  //Write protection error
    u1 DATA0            : 8;  //End of operation
    u1 DATA1            : 8;  //Busy
  }volatile OBR;         //Flash Option byte register (FLASH_OBR)
struct
  {
    u32 WRP;            //Converted data
  }volatile WRPR;         //Write protection register (FLASH_WRPR)
}FLASH_Type;
#define FLASH ((FLASH_Type*) FLASH_R_BASE)


//Flash access control register (FLASH_ACR)
#define FLASH_LATENCY        FLASH->ACR.LATENCY         //ADC ready
#define FLASH_PRFTBE         FLASH->ACR.PRFTBE          //ADC ready
#define FLASH_PRFTBS         FLASH->ACR.PRFTBS          //ADC ready

//Flash key register (FLASH_KEYR)
#define FLASH_FKEY           FLASH->KEYR.FKEY           //ADC ready
//Flash option key register (FLASH_OPTKEYR)
#define FLASH_BSY            FLASH->SR.BSY              //ADC ready
#define FLASH_PGERR          FLASH->SR.PGERR            //ADC ready
#define FLASH_WRPRTERR       FLASH->SR.WRPRTERR         //ADC ready
#define FLASH_EOP            FLASH->SR.EOP              //ADC ready
//Flash control register (FLASH_CR)
#define FLASH_PG            FLASH->CR.PG                //ADC ready
#define FLASH_PER           FLASH->CR.PER               //ADC ready
#define FLASH_MER           FLASH->CR.MER               //ADC ready
#define FLASH_OPTPG         FLASH->CR.OPTPG             //ADC ready
#define FLASH_OPTER         FLASH->CR.OPTER             //ADC ready
#define FLASH_STRT          FLASH->CR.STRT              //ADC ready
#define FLASH_LOCK          FLASH->CR.LOCK              //ADC ready
#define FLASH_OPTWRE        FLASH->CR.OPTWRE            //ADC ready
#define FLASH_ERRIE         FLASH->CR.ERRIE             //ADC ready
#define FLASH_EOPIE         FLASH->CR.EOPIE             //ADC ready
#define FLASH_OBL_LAUNCH    FLASH->CR.OBL_LAUNCH        //ADC ready
//Flash address register (FLASH_AR)
#define FLASH_FAR           FLASH->AR.FAR               //ADC ready
//Flash Option byte register (FLASH_OBR)
#define FLASH_OPTERR            FLASH->OBR.OPTERR       //ADC ready
#define FLASH_RDPRT             FLASH->OBR.RDPRT        //ADC ready
#define FLASH_WDG_SW            FLASH->OBR.WDG_SW       //ADC ready
#define FLASH_nRST_STOP         FLASH->OBR.nRST_STOP    //ADC ready
#define FLASH_nRST_STDBY        FLASH->OBR.nRST_STDBY   //ADC ready
#define FLASH_nBOOT0            FLASH->OBR.nBOOT0       //ADC ready
#define FLASH_nBOOT1            FLASH->OBR.nBOOT1       //ADC ready
#define FLASH_VDDA_MONITOR      FLASH->OBR.VDDA_MONITOR         //ADC ready
#define FLASH_RAM_PARITY_CHECK  FLASH->OBR.RAM_PARITY_CHECK     //ADC ready
#define FLASH_BOOT_SEL          FLASH->OBR.BOOT_SEL             //ADC ready
#define FLASH_DATA0             FLASH->OBR.DATA0                //ADC ready
#define FLASH_DATA1             FLASH->OBR.DATA1                //ADC ready
//Write protection register (FLASH_WRPR)
#define FLASH_WRP           FLASH->WRPR.WRP             //ADC ready

/******************************************* CRS **************************************************/
typedef struct
{
  struct
  {
    u1 SYNCOKIE         : 1;  //SYNC event OK interrupt enable
    u1 SYNCWARNIE       : 1;  //SYNC warning interrupt enable
    u1 ERRIE            : 1;  //Synchronization or trimming error interrupt enable
    u1 ESYNCIE          : 1;  //Expected SYNC interrupt enable
    u1 RSVD0            : 1;  //Reserved
    u1 CEN              : 1;  //Frequency error counter enable
    u1 AUTOTRIMEN       : 1;  //Automatic trimming enable
    u1 SWSYNC           : 1;  //Generate software SYNC event
    u1 TRIM             : 6;  //HSI48 oscillator smooth trimming
    u1 RSVD1            : 18; //Reserved
  }volatile CRS_CR;           //CRS control register (CRS_CR)
struct
  {
    u16 RELOAD;               //Counter reload value
    u1 FELIM            : 8;  //Frequency error limit
    u1 SYNCDIV          : 3;  //SYNC divider
    u1 RSVD0            : 1;  //Reserved
    u1 SYNCSRC          : 2;  //SYNC signal source selection
    u1 RSVD1            : 1;  //Reserved
    u1 SYNCPOL          : 1;  //SYNC polarity selection
  }volatile CRS_CFGR;        //CRS configuration register (CRS_CFGR)
struct
  {
    u1 SYNCOKF          : 1;  //SYNC event OK flag
    u1 SYNCWARNF        : 1;  //SYNC warning flag
    u1 ERRF             : 1;  //Error flag
    u1 ESYNCF           : 1;  //Expected SYNC flag
    u1 RSVD0            : 4;  //Reserved
    u1 SYNCERR          : 1;  //SYNC error
    u1 SYNCMISS         : 1;  //SYNC missed
    u1 TRIMOVF          : 1;  //Trimming overflow or underflow
    u1 RSVD1            : 4;  //Reserved
    u1 FEDIR            : 1;  //Frequency error direction
    u16 FECAP;                //Frequency error capture
  }volatile CRS_ISR;        //CRS interrupt and status register (CRS_ISR)
struct
  {
    u1 SYNCOKC          : 1;  //SYNC event OK clear flag
    u1 SYNCWARNC        : 1;  //SYNC warning clear flag
    u1 ERRC             : 1;  //Error clear flag
    u1 ESYNCC           : 1;  //Expected SYNC clear flag
    u1 RSVD0            : 18; //Reserved
  }volatile CRS_ICR;           //CRS interrupt flag clear register (CRS_ICR)
}CRS_Type;

#define CRS ((CRS_Type*) CRS_BASE)
//CRS control register (CRS_CR)
#define CRS_SYNCOKIE            CRS->CRS_CR.SYNCOKIE
#define CRS_SYNCWARNIE          CRS->CRS_CR.SYNCWARNIE
#define CRS_ERRIE               CRS->CRS_CR.ERRIE
#define CRS_ESYNCIE             CRS->CRS_CR.ESYNCI
#define CRS_CEN                 CRS->CRS_CR.CEN
#define CRS_AUTOTRIMEN          CRS->CRS_CR.AUTOTRIMEN
#define CRS_SWSYNC              CRS->CRS_CR.SWSYNC
#define CRS_TRIM                CRS->CRS_CR.TRIM

#define CRS_RELOAD              CRS->CRS_CFGR.RELOAD
#define CRS_FELIM               CRS->CRS_CFGR.FELIM
#define CRS_SYNCDIV             CRS->CRS_CFGR.SYNCDIV
#define CRS_SYNCSRC             CRS->CRS_CFGR.SYNCSRC
#define CRS_SYNCPOL             CRS->CRS_CFGR.SYNCPOL
/******************************************* ADC **************************************************/
typedef struct
{
  struct
  {
    u1 ADRDY      : 1;  //ADC ready
    u1 EOSMP      : 1;  //End of sampling flag
    u1 EOC        : 1;  //End of conversion flag
    u1 EOSEQ      : 1;  //End of sequence flag
    u1 OVR        : 1;  //ADC overrun
    u1 RSVD0      : 2;  //Reserved
    u1 AWD        : 1;  //Analog watchdog flag
    u1 RSVD1      : 24; //Reserved
  }volatile ISR;        //ADC interrupt and status register (ADC_ISR)
struct
  {
    u1 ADRDYIE    : 1;  //ADC ready interrupt enable
    u1 EOSMPIE    : 1;  //End of sampling flag interrupt enable
    u1 EOCIE      : 1;  //End of conversion interrupt enable
    u1 EOSEQIE    : 1;  //End of conversion sequence interrupt enable
    u1 OVRIE      : 1;  //Overrun interrupt enable
    u1 RSVD0      : 2;  //Reserved
    u1 AWDIE      : 1;  //Analog watchdog interrupt enable
    u1 RSVD1      : 24; //Reserved
  }volatile IER;        //ADC interrupt enable register
struct
  {
    u1 ADEN       : 1;  //ADC enable command
    u1 ADDIS      : 1;  //ADC disable command
    u1 ADSTART    : 1;  //ADC start conversion command
    u1 RSVD0      : 1;  //Reserved
    u1 ADSTP      : 1;  //ADC stop conversion command
    u1 RSVD1      : 26; //Reserved
    u1 ADCAL      : 1;  //ADC calibration
  }volatile CR;         //ADC control register
struct
  {
    u1 DMAEN      : 1;  //Direct memory access enable
    u1 DMACFG     : 1;  //Direct memory access configuration
    u1 SCANDIR    : 1;  //Scan sequence direction
    u1 RES        : 2;  //Data resolution
    u1 ALIGN      : 1;  //Data alignment
    u1 EXTSEL     : 3;  //External trigger selection
    u1 RSVD0      : 1;  //Reserved
    u1 EXTEN      : 2;  //External trigger enable and polarity selection
    u1 OVRMOD     : 1;  //Overrun management mode
    u1 CONT       : 1;  //Single / continuous conversion mode
    u1 WAIT       : 1;  //Wait conversion mode
    u1 AUTOFF     : 1;  //Auto-off mode
    u1 DISCEN     : 1;  //Discontinuous mode
    u1 RSVD1      : 5;  //Reserved
    u1 AWDSGL     : 1;  //Enable the watchdog on a single channel or on all channels
    u1 AWDEN      : 1;  //Analog watchdog enable
    u1 RSVD2      : 2;  //Reserved
    u1 AWDCH      : 5;  //Analog watchdog channel selection
    u1 RSVD3      : 1;  //Reserved
  }volatile CFGR1;      //ADC configuration register 1
struct
  {
    u1 RSVD0      : 30; //Reserved
    u1 CKMODE     : 2;  //ADC clock mode 
  }volatile CFGR2;      //ADC configuration register 2
struct
  {
    u1 SMP        : 3;  //Sampling time selection
    u1 RSVD0      : 29; //Reserved
  }volatile SMPR;       //ADC sampling time register
   volatile u32 RSVD0[2];   //Reserved
struct
  {
    u1 LT         : 12; //Analog watchdog lower threshold
    u1 RSVD0      : 4;  //Reserved
    u1 HT         : 12; //Analog watchdog higher threshold
    u1 RSVD1      : 4;  //Reserved
  }volatile TR;         //ADC watchdog threshold register
   volatile u32 RSVD1;   //Reserved
/*struct
  {
    u1 CHSEL00    : 1;  //Channel-00 selection
    u1 CHSEL01    : 1;  //Channel-01 selection
    u1 CHSEL02    : 1;  //Channel-02 selection
    u1 CHSEL03    : 1;  //Channel-03 selection
    u1 CHSEL04    : 1;  //Channel-04 selection
    u1 CHSEL05    : 1;  //Channel-05 selection
    u1 CHSEL06    : 1;  //Channel-06 selection
    u1 CHSEL07    : 1;  //Channel-07 selection
    u1 CHSEL08    : 1;  //Channel-08 selection
    u1 CHSEL09    : 1;  //Channel-09 selection
    u1 CHSEL10    : 1;  //Channel-10 selection
    u1 CHSEL11    : 1;  //Channel-11 selection
    u1 CHSEL12    : 1;  //Channel-12 selection
    u1 CHSEL13    : 1;  //Channel-13 selection
    u1 CHSEL14    : 1;  //Channel-14 selection
    u1 CHSEL15    : 1;  //Channel-15 selection
    u1 CHSEL16    : 1;  //Channel-16 selection
    u1 CHSEL17    : 1;  //Channel-17 selection
    u1 CHSEL18    : 1;  //Channel-18 selection
    u1 RSVD0      : 13; //Reserved    
  }volatile CHSELR;     //ADC channel selection register*/
   volatile u32 CHSELR;
   volatile u32 RSVD2[5];   //Reserved
struct
  {
    u1 DATA       : 16; //Converted data
    u1 RSVD0      : 16; //Reserved
  }volatile DR;         //ADC data register
   volatile u32 RSVD3[177];   //Reserved
struct
  {
    u1 RSVD0      : 22; //Reserved
    u1 VREFEN     : 1;  //Vrefint enable
    u1 TSEN       : 1;  //Temperature sensor enable
    u1 VBATEN     : 1;  //Vbat enable
    u1 RSVD1      : 7;  //Converted data
  }volatile CCR;              //ADC common configuration register
}ADC_Type;

#define ADC ((ADC_Type*) ADC_BASE)

//ADC interrupt and status control (ADC_ISR)
#define ADC_RDY              ADC->ISR.ADRDY                 //ADC ready
#define ADC_EOSMP            ADC->ISR.EOSMP                 //End of sampling flag
#define ADC_EOC              ADC->ISR.EOC                   //End of conversion flag
#define ADC_EOSEQ            ADC->ISR.EOSEQ                 //End of sequence flag
#define ADC_OVR              ADC->ISR.OVR                   //ADC overrun
#define ADC_WD               ADC->ISR.AWD                   //Analog watchdog flag
//ADC interrupt enable control (ADC_IER)
#define ADC_RDYIEN           ADC->IER.ADRDYIE               //ADC ready interrupt enable
#define ADC_EOSMPIE          ADC->IER.EOSMPIE               //End of sampling flag interrupt enable
#define ADC_EOCIE            ADC->IER.EOCIE                 //End of conversion interrupt enable
#define ADC_EOSEQIE          ADC->IER.EOSEQIE               //End of conversion sequence interrupt enable
#define ADC_OVRIE            ADC->IER.OVRIE                 //Overrun interrupt enable
#define ADC_AWDIE            ADC->IER.AWDIE                 //Analog watchdog interrupt enable
//ADC control (ADC_CR)
#define ADC_EN               ADC->CR.ADEN                   //ADC enable command
#define ADC_DIS              ADC->CR.ADDIS                  //ADC disable command
#define ADC_START            ADC->CR.ADSTART = ON           //ADC start conversion command
#define ADC_STP              ADC->CR.ADSTP                  //ADC stop conversion command
#define ADC_CAL              ADC->CR.ADCAL                  //ADC calibration
//ADC configuration 1 control(ADC_CFGR1)
#define ADC_DMAEN            ADC->CFGR1.DMAEN                   //Direct memory access enable
#define ADC_DMACFG           ADC->CFGR1.DMACFG                  //Direct memory access configuration
#define ADC_SCANDIR          ADC->CFGR1.SCANDIR                 //Scan sequence direction
#define ADC_RES              ADC->CFGR1.RES                     //Data resolution
#define ADC_ALIGN            ADC->CFGR1.ALIGN                   //Data alignment
#define ADC_EXTSEL           ADC->CFGR1.EXTSEL                  //External trigger selection
#define ADC_EXTEN            ADC->CFGR1.EXTEN                   //External trigger enable and polarity selection
#define ADC_OVRMOD           ADC->CFGR1.OVRMOD                  //Overrun management mode
#define ADC_CONT             ADC->CFGR1.CONT                    //Single / continuous conversion mode
#define ADC_WAIT             ADC->CFGR1.WAIT                    //Wait conversion mode
#define ADC_AUTOFF           ADC->CFGR1.AUTOFF                  //Auto-off mode
#define ADC_DISCEN           ADC->CFGR1.DISCEN                  //Discontinuous mode
#define ADC_AWDSGL           ADC->CFGR1.AWDSGL                  //Enable the watchdog on a single channel or on all channels
#define ADC_AWDEN            ADC->CFGR1.AWDEN                   //Analog watchdog enable
#define ADC_AWDCH            ADC->CFGR1.AWDCH                   //Analog watchdog channel selection
//ADC configuration 2 control(ADC_CFGR2)
#define ADC_CKMODE           ADC->CFGR2.CKMODE                  //ADC clock mode
//ADC sampling time control (ADC_SMPR)   
#define ADC_SMP              ADC->SMPR.SMP                      //Sampling time selection
//ADC watchdog threshold control (ADC_TR)
#define ADC_LT               ADC->TR.LT                         //Analog watchdog lower threshold
#define ADC_HT               ADC->TR.HT                         //Analog watchdog higher threshold
//ADC channel selection control (ADC_CHSELR)
#define ADC_CH00_ON         ADC->CHSELR = ADC_CH00              //Channel-00 selection
#define ADC_CH01_ON         ADC->CHSELR = ADC_CH01              //Channel-00 selection
#define ADC_CH02_ON         ADC->CHSELR = ADC_CH02              //Channel-00 selection
#define ADC_CH03_ON         ADC->CHSELR = ADC_CH03              //Channel-00 selection
#define ADC_CH04_ON         ADC->CHSELR = ADC_CH04              //Channel-00 selection
#define ADC_CH05_ON         ADC->CHSELR = ADC_CH05              //Channel-00 selection
#define ADC_CH06_ON         ADC->CHSELR = ADC_CH06              //Channel-00 selection
#define ADC_CH07_ON         ADC->CHSELR = ADC_CH07              //Channel-00 selection
#define ADC_CH08_ON         ADC->CHSELR = ADC_CH08              //Channel-00 selection
#define ADC_CH09_ON         ADC->CHSELR = ADC_CH09              //Channel-00 selection
#define ADC_CH10_ON         ADC->CHSELR = ADC_CH10              //Channel-00 selection
#define ADC_CH11_ON         ADC->CHSELR = ADC_CH11              //Channel-00 selection
#define ADC_CH12_ON         ADC->CHSELR = ADC_CH12              //Channel-00 selection
#define ADC_CH13_ON         ADC->CHSELR = ADC_CH13              //Channel-00 selection
#define ADC_CH14_ON         ADC->CHSELR = ADC_CH14              //Channel-00 selection
#define ADC_CH15_ON         ADC->CHSELR = ADC_CH15              //Channel-00 selection
#define ADC_TEMP_ON         ADC->CHSELR = ADC_CH16              //Channel-00 selection
#define ADC_VREF_ON         ADC->CHSELR = ADC_CH17              //Channel-00 selection
#define ADC_VBAT_ON         ADC->CHSELR = ADC_CH18              //Channel-00 selection
#define ADC_CH_NONE         ADC->CHSELR = ((u32) 0x00000000)    //Channel-00 selection


/*
#define ADC_CHSEL00          ADC->CHSELR.CHSEL00            //Channel-00 selection
#define ADC_CHSEL01          ADC->CHSELR.CHSEL01            //Channel-01 selection
#define ADC_CHSEL02          ADC->CHSELR.CHSEL02            //Channel-02 selection
#define ADC_CHSEL03          ADC->CHSELR.CHSEL03            //Channel-03 selection
#define ADC_CHSEL04          ADC->CHSELR.CHSEL04            //Channel-04 selection
#define ADC_CHSEL05          ADC->CHSELR.CHSEL05            //Channel-05 selection
#define ADC_CHSEL06          ADC->CHSELR.CHSEL06            //Channel-06 selection
#define ADC_CHSEL07          ADC->CHSELR.CHSEL07            //Channel-07 selection
#define ADC_CHSEL08          ADC->CHSELR.CHSEL08            //Channel-08 selection
#define ADC_CHSEL09          ADC->CHSELR.CHSEL09            //Channel-09 selection
#define ADC_CHSEL10          ADC->CHSELR.CHSEL10            //Channel-10 selection
#define ADC_CHSEL11          ADC->CHSELR.CHSEL11            //Channel-11 selection
#define ADC_CHSEL12          ADC->CHSELR.CHSEL12            //Channel-12 selection
#define ADC_CHSEL13          ADC->CHSELR.CHSEL13            //Channel-13 selection
#define ADC_CHSEL14          ADC->CHSELR.CHSEL14            //Channel-14 selection
#define ADC_CHSEL15          ADC->CHSELR.CHSEL15            //Channel-15 selection
#define ADC_CHSEL16          ADC->CHSELR.CHSEL16            //Channel-16 selection
#define ADC_CHSEL17          ADC->CHSELR.CHSEL17            //Channel-17 selection
#define ADC_CHSEL18          ADC->CHSELR.CHSEL18            //Channel-18 selection
*/
//ADC data control (ADC_DR)
#define ADC_DATA             ADC->DR.DATA                   //Converted data
//ADC common configuration control (ADC_CCR)
#define ADC_VREFEN           ADC->CCR.VREFEN                //Vrefint enable
#define ADC_TSEN             ADC->CCR.TSEN                  //Temperature sensor enable
#define ADC_VBATEN           ADC->CCR.VBATEN                //Vbat enable

#define ADC_CH00              ((u32) 0x00000001)
#define ADC_CH01              ((u32) 0x00000002)
#define ADC_CH02              ((u32) 0x00000004)
#define ADC_CH03              ((u32) 0x00000008)
#define ADC_CH04              ((u32) 0x00000010)
#define ADC_CH05              ((u32) 0x00000020)
#define ADC_CH06              ((u32) 0x00000040)
#define ADC_CH07              ((u32) 0x00000080)
#define ADC_CH08              ((u32) 0x00000100)
#define ADC_CH09              ((u32) 0x00000200)
#define ADC_CH10              ((u32) 0x00000400)
#define ADC_CH11              ((u32) 0x00000800)
#define ADC_CH12              ((u32) 0x00001000)
#define ADC_CH13              ((u32) 0x00002000)
#define ADC_CH14              ((u32) 0x00004000)
#define ADC_CH15              ((u32) 0x00008000)
#define ADC_CH16              ((u32) 0x00010000)
#define ADC_CH17              ((u32) 0x00020000)
#define ADC_CH18              ((u32) 0x00040000)

/******************************************* TIMER **************************************************/
typedef struct
{
struct
  {
    u1 CEN        : 1;  //Counter enable
    u1 UDIS       : 1;  //Update disable
    u1 URS        : 1;  //Update request source
    u1 OPM        : 1;  //One-pulse mode
    u1 DIR        : 1;  //Direction
    u1 CMS        : 2;  //Center-aligned mode selection
    u1 ARPE       : 1;  //Auto-reload preload enable
    u1 CKD        : 2;  //Clock division
    u1 RSVD0      : 22; //Reserved
  }volatile TIMx_CR1;   //TIMx control register 1 (TIMx_CR1)
struct
  {
    u1 CCPS       : 1;  //Capture/compare preloaded control
    u1 RSVD0      : 1;  //Reserved
    u1 CCUS       : 1;  //Capture/compare control update selection
    u1 CCDS       : 1;  //Capture/compare DMA selection
    u1 MMS        : 3;  //Master mode selection
    u1 TI1S       : 1;  //TI1 selection
    u1 OIS1       : 1;  //Output idle state 1 (OC1 output)
    u1 OIS1N      : 1;  //Output idle state 1 (OC1N output)
    u1 OIS2       : 1;  //Output idle state 2 (OC2 output)
    u1 OIS2N      : 1;  //Output idle state 2 (OC2N output)
    u1 OIS3       : 1;  //Output idle state 3 (OC3 output)
    u1 OIS3N      : 1;  //Output idle state 3 (OC3N output)
    u1 OIS4       : 1;  //Output idle state 4 (OC4 output)
    u1 RSVD1      : 17; //Reserved
  }volatile TIMx_CR2;   //TIMx control register 2 (TIMx_CR2)
struct
  {
    u1 SMS        : 3;  //Slave mode selection
    u1 OCCS       : 1;  //OCREF clear selection
    u1 TS         : 3;  //Trigger selection
    u1 MSM        : 1;  //Master/Slave mode
    u1 ETF        : 4;  //External trigger filter
    u1 ETPS       : 2;  //External trigger prescaler
    u1 ECE        : 1;  //External clock enable
    u1 ETP        : 1;  //External trigger polarity
    u1 RSVD0      : 16; //Reserved
  }volatile TIMx_SMCR;  //TIMx slave mode control register (TIMx_SMCR)
struct
  {
    u1 UIE        : 1;  //Update interrupt enable
    u1 CC1IE      : 1;  //Capture/Compare 1 interrupt enable
    u1 CC2IE      : 1;  //Capture/Compare 2 interrupt enable
    u1 CC3IE      : 1;  //Capture/Compare 3 interrupt enable
    u1 CC4IE      : 1;  //Capture/Compare 4 interrupt enable
    u1 COMIE      : 1;  //COM interrupt enable
    u1 TIE        : 1;  //Trigger interrupt enable
    u1 BIE        : 1;  //Break interrupt enable
    u1 UDE        : 1;  //Update DMA request enable
    u1 CC1DE      : 1;  //Capture/Compare 1 DMA enable
    u1 CC2DE      : 1;  //Capture/Compare 2 DMA enable
    u1 CC3DE      : 1;  //Capture/Compare 3 DMA enable
    u1 CC4DE      : 1;  //Capture/Compare 4 DMA enable
    u1 COMDE      : 1;  //COM DMA request enable
    u1 TDE        : 1;  //Trigger DMA requeest enable
    u1 RSVD0      : 17; //Reserved
  }volatile TIMx_DIER;  //TIMx DMA/Interrupt enable register (TIMx_DIER)
struct
  {
    u1 UIF        : 1;  //Update interrupt flag
    u1 CC1IF      : 1;  //Capture/Compare 1 interrupt flag
    u1 CC2IF      : 1;  //Capture/Compare 2 interrupt flag
    u1 CC3IF      : 1;  //Capture/Compare 3 interrupt flag
    u1 CC4IF      : 1;  //Capture/Compare 4 interrupt flag
    u1 COMIF      : 1;  //COM interrupt flag
    u1 TIF        : 1;  //Trigger interrupt flag
    u1 BIF        : 1;  //Break interrupt flag
    u1 RSVD0      : 1;  //Reserved
    u1 CC1OF      : 1;  //Capture/Compare 1 overcapture flag
    u1 CC2OF      : 1;  //Capture/Compare 2 overcapture flag
    u1 CC3OF      : 1;  //Capture/Compare 3 overcapture flag
    u1 CC4OF      : 1;  //Capture/Compare 4 overcapture flag
    u1 RSVD1      : 19; //Reserved
  }volatile TIMx_SR;  //TIMx status register (TIMx_SR)  
struct
  {
    u1 UG         : 1;  //Update generation
    u1 CC1G       : 1;  //Capture/Compare 1 generation
    u1 CC2G       : 1;  //Capture/Compare 2 generation
    u1 CC3G       : 1;  //Capture/Compare 3 generation
    u1 CC4G       : 1;  //Capture/Compare 4 generation
    u1 COMG       : 1;  //Capture/Compare control update generation
    u1 TG         : 1;  //Trigger generation
    u1 BG         : 1;  //Break generation
    u1 RSVD0      : 19; //Reserved
  }volatile TIMx_EGR;   //TIMx event generation register (TIMx_EGR)  
struct
  {
    u1 CC1S       : 8;  //Capture/Compare 1 selection
    u1 CC2S       : 8;  //Capture/Compare 2 selection
    u1 RSVD0      : 16; //Reserved
  }volatile TIMx_CCMR1; //TIMx capture/compare mode register 1 (TIMx_CCMR1)  
struct
  {
    u1 CC3S       : 8;  //Capture/Compare 3 selection
    u1 CC4S       : 8;  //Capture/Compare 4 selection
    u1 RSVD0      : 16; //Reserved
  }volatile TIMx_CCMR2; //TIMx capture/compare mode register 2 (TIMx_CCMR2)  
struct
  {
    u1 CC1OE      : 1;  //Capture/Compare 1 output enable
    u1 CC1OP      : 1;  //Capture/Compare 1 output Polarity
    u1 CC1NOE     : 1;  //Capture/Compare 1 complementary output enable
    u1 CC1NOP     : 1;  //Capture/Compare 1 complementary output Polarity
    u1 CC2OE      : 1;  //Capture/Compare 2 output enable
    u1 CC2OP      : 1;  //Capture/Compare 2 output Polarity
    u1 CC2NOE     : 1;  //Capture/Compare 2 complementary output enable
    u1 CC2NOP     : 1;  //Capture/Compare 2 complementary output Polarity
    u1 CC3OE      : 1;  //Capture/Compare 3 output enable
    u1 CC3OP      : 1;  //Capture/Compare 3 output Polarity
    u1 CC3NOE     : 1;  //Capture/Compare 3 complementary output enable
    u1 CC3NOP     : 1;  //Capture/Compare 3 complementary output Polarity
    u1 CC4OE      : 1;  //Capture/Compare 4 output enable
    u1 CC4OP      : 1;  //Capture/Compare 4 output Polarity
    u1 CC4NOE     : 1;  //Capture/Compare 4 complementary output enable
    u1 CC4NOP     : 1;  //Capture/Compare 4 complementary output Polarity
    u1 RSVD0      : 16; //Reserved
  }volatile TIMx_CCER;  //TIMx capture/compare enable register (TIMx_CCER)
struct
  {
    u1 CNT        : 16; //Low counter value
    u1 RSVD0      : 16; //High counter value (on TIM2)
  }volatile TIMx_CNT;   //TIMx counter (TIMx_CNT)  
struct
  {
    u1 PSC        : 16; //Prescaler value
    u1 RSVD0      : 16; //Reserved
  }volatile TIMx_PSC;   //TIMx prescaler (TIMx_PSC)  
struct
  {
    u1 LARR       : 16; //Low auto-reload value
    u1 HARR       : 16; //High auto-reload value (on TIM2)
  }volatile TIMx_ARR;   //TIMx auto-reload register (TIMx_ARR)
struct
  {
    u1 REP        : 8;  //Repetition counter value
    u1 RSVD0      : 24; //Reserved
  }volatile TIMx_RCR;   //TIMx repetition counter register (TIMx_RCR)
struct
  {
    u1 LCCR1      : 16; //Low Capture/Compare value
    u1 HCCR1      : 16; //High Capture/Compare value (on TIM2)
  }volatile TIMx_CCR1;  //TIMx capture/compare register 1 (TIMx_CCR1)
struct
  {
    u1 LCCR2      : 16; //Low Capture/Compare value
    u1 HCCR2      : 16; //High Capture/Compare value (on TIM2)
  }volatile TIMx_CCR2;  //TIMx capture/compare register 2 (TIMx_CCR2)
struct
  {
    u1 LCCR3      : 16; //Low Capture/Compare value
    u1 HCCR3      : 16; //High Capture/Compare value (on TIM2)
  }volatile TIMx_CCR3;  //TIMx capture/compare register 3 (TIMx_CCR3)
struct
  {
    u1 LCCR4      : 16; //Low Capture/Compare value
    u1 HCCR4      : 16; //High Capture/Compare value (on TIM2)
  }volatile TIMx_CCR4;  //TIMx capture/compare register 4 (TIMx_CCR4)
struct
  {
    u1 DTG        : 8;  //Dead-time generator setup
    u1 LOCK       : 2;  //Lock configuration
    u1 OSSI       : 1;  //Off-state selection for idle mode
    u1 OSSR       : 1;  //Off-state selection for Run mode
    u1 BKE        : 1;  //Break enable
    u1 BKP        : 1;  //Break polarity
    u1 AOE        : 1;  //Automatic output enable
    u1 MOE        : 1;  //Main output enable
    u1 RSVD0      : 16; //Reserved
  }volatile TIMx_BDTR;  //TIMx break and dead-time register (TIMx_BDTR)
struct
  {
    u1 DBA        : 5;  //DMA base address
    u1 RSVD0      : 3;  //Reserved
    u1 DBL        : 5;  //DMA burst length
    u1 RSVD1      : 19; //Reserved
  }volatile TIMx_DCR;   //TIMx DMA control register (TIMx_DCR)
struct
  {
    u1 DMAB       : 16; //DMA register for burst accesses
    u1 RSVD0      : 16; //Reserved
  }volatile TIMx_DMAR;  //TIMx DMA address for full transfer (TIMx_DMAR)

}TIMx_Type;

//TIM3 control (TIM3_CR1)
#define TIM01 ((TIMx_Type*) TIM01_BASE)
#define TIM02 ((TIMx_Type*) TIM02_BASE)
#define TIM03 ((TIMx_Type*) TIM03_BASE)
#define TIM06 ((TIMx_Type*) TIM06_BASE)
#define TIM07 ((TIMx_Type*) TIM07_BASE)
#define TIM14 ((TIMx_Type*) TIM14_BASE)
#define TIM15 ((TIMx_Type*) TIM15_BASE)
#define TIM16 ((TIMx_Type*) TIM16_BASE)
#define TIM17 ((TIMx_Type*) TIM17_BASE)

//TIMx control register 1
#define TIMx_COUNTER_EN(TIMx)           TIMx->TIMx_CR1.CEN  = ON                
#define TIMx_UPDATE_DIS(TIMx)           TIMx->TIMx_CR1.UDIS = ON
#define TIMx_UPDATEREQ_SRC(TIMx, MODE)  TIMx->TIMx_CR1.URS  = MODE              //1: Only counter overflow/underflow generates an update interrupt or DMA request if enabled
#define TIMx_ONEPULSE_ON(TIMx)          TIMx->TIMx_CR1.OPM  = ON
#define TIMx_DIR(TIMx, MODE)            TIMx->TIMx_CR1.DIR  = MODE              //0: Counter used as upcounter, 1: Counter used as downcounter
#define TIMx_CENTERALIGN(TIMx,MODE)     TIMx->TIMx_CR1.CMS  = MODE              //
#define TIMx_AUTO_RE_PRE_LOAD_EN(TIMx)  TIMx->TIMx_CR1.ARPE = ON
#define TIMx_CLKDIVISION(TIMx, MODE)    TIMx->TIMx_CR1.CKD  = MODE
//TIMx control register 2
#define TIMx_CAPCOMP_PRLD_CNTRL(TIMx)   TIMx->TIMx_CR2.CCPS
#define TIMx_CAPCOMP_C_UPD_SEL(TIMx)    TIMx->TIMx_CR2.CCUS
#define TIMx_CAPCOMP_DMA_SEL(TIMx)      TIMx->TIMx_CR2.CCDS
#define TIMx_MASTER_MODE_SEL(TIMx)      TIMx->TIMx_CR2.MMS
#define TIMx_TI_1_SEL(TIMx)             TIMx->TIMx_CR2.TI2S
#define TIMx_OUT_IDLE_STATE_1(TIMx)     TIMx->TIMx_CR2.OIS1
#define TIMx_OUT_IDLE_STATE_1N(TIMx)    TIMx->TIMx_CR2.OIS1N
#define TIMx_OUT_IDLE_STATE_2(TIMx)     TIMx->TIMx_CR2.OIS2
#define TIMx_OUT_IDLE_STATE_2N(TIMx)    TIMx->TIMx_CR2.OIS2N
#define TIMx_OUT_IDLE_STATE_3(TIMx)     TIMx->TIMx_CR2.OIS3
#define TIMx_OUT_IDLE_STATE_3N(TIMx)    TIMx->TIMx_CR2.OIS3N
#define TIMx_OUT_IDLE_STATE_4(TIMx)     TIMx->TIMx_CR2.OIS4
#define TIMx_OUT_IDLE_STATE_4N(TIMx)    TIMx->TIMx_CR2.OIS4N
//TIMx slave mode control register
#define TIMx_SLAVE_MODE_SEL(TIMx, MODE) TIMx->TIMx_SMCR.SMS  = MODE
#define TIMx_OCREF_CLEAR_SEL(TIMx)      TIMx->TIMx_SMCR.OCCS
#define TIMx_TRIG_SEL(TIMx, MODE)       TIMx->TIMx_SMCR.TS   = MODE
#define TIMx_MAST_SL_MODE(TIMx)         TIMx->TIMx_SMCR.MSM
#define TIMx_EXT_TRIG_FILT(TIMx)        TIMx->TIMx_SMCR.ETF1
#define TIMx_EXT_TRIG_PRESC(TIMx)       TIMx->TIMx_SMCR.ETPS
#define TIMx_EXT_CLK_EN(TIMx)           TIMx->TIMx_SMCR.ECE
#define TIMx_EXT_TRIG_POLAR(TIMx)       TIMx->TIMx_SMCR.ETP
//TIMx DMA/Interrupt enable register
#define TIMx_UPDATE_INT_EN(TIMx)        TIMx->TIMx_DIER.UIE
#define TIMx_CAP_COMP_1_INT_EN(TIMx)    TIMx->TIMx_DIER.CC1IE
#define TIMx_CAP_COMP_2_INT_EN(TIMx)    TIMx->TIMx_DIER.CC2IE
#define TIMx_CAP_COMP_3_INT_EN(TIMx)    TIMx->TIMx_DIER.CC3IE
#define TIMx_CAP_COMP_4_INT_EN(TIMx)    TIMx->TIMx_DIER.CC4IE
#define TIMx_COM_INT_EN(TIMx)           TIMx->TIMx_DIER.COMIE
#define TIMx_TRIG_INT_EN(TIMx)          TIMx->TIMx_DIER.TIE
#define TIMx_BREAK_INT_EN(TIMx)         TIMx->TIMx_DIER.BIE
#define TIMx_UPDATE_DMA_REQ_EN(TIMx)    TIMx->TIMx_DIER.UDE
#define TIMx_CAP_COMP_1_DMA_EN(TIMx)    TIMx->TIMx_DIER.CC1DE
#define TIMx_CAP_COMP_2_DMA_EN(TIMx)    TIMx->TIMx_DIER.CC2DE
#define TIMx_CAP_COMP_3_DMA_EN(TIMx)    TIMx->TIMx_DIER.CC3DE
#define TIMx_CAP_COMP_4_DMA_EN(TIMx)    TIMx->TIMx_DIER.CC4DE
#define TIMx_COM_DMA_REQ_EN(TIMx)       TIMx->TIMx_DIER.COMDE
#define TIMx_TRIG_DMA_REQ_EN(TIMx)      TIMx->TIMx_DIER.TDE
//TIMx status register
#define TIMx_UP_IF(TIMx)                TIMx->TIMx_SR.UIF //&= ~(0x01)
#define TIMx_CC1_IF(TIMx)               TIMx->TIMx_SR.CC1IF
#define TIMx_CC2_IF(TIMx)               TIMx->TIMx_SR.CC2IF
#define TIMx_CC3_IF(TIMx)               TIMx->TIMx_SR.CC3IF
#define TIMx_CC4_IF(TIMx)               TIMx->TIMx_SR.CC4IF
#define TIMx_COM_IF(TIMx)               TIMx->TIMx_SR.COMIF
#define TIMx_TRIG_IF(TIMx)              TIMx->TIMx_SR.TIF
#define TIMx_BREAK_IF(TIMx)             TIMx->TIMx_SR.BIF
#define TIMx_CC1_OCF(TIMx)              TIMx->TIMx_SR.CC1OF
#define TIMx_CC2_OCF(TIMx)              TIMx->TIMx_SR.CC2OF
#define TIMx_CC3_OCF(TIMx)              TIMx->TIMx_SR.CC3OF
#define TIMx_CC4_OCF(TIMx)              TIMx->TIMx_SR.CC4OF
//TIMx event generation register
#define TIMx_UPDATE_GEN(TIMx)           TIMx->TIMx_EGR.UG
#define TIMx_CAP_COMP_1_GEN(TIMx)       TIMx->TIMx_EGR.CC1G
#define TIMx_CAP_COMP_2_GEN(TIMx)       TIMx->TIMx_EGR.CC2G
#define TIMx_CAP_COMP_3_GEN(TIMx)       TIMx->TIMx_EGR.CC3G
#define TIMx_CAP_COMP_4_GEN(TIMx)       TIMx->TIMx_EGR.CC4G
#define TIMx_CAP_COMP_CTRL_UG(TIMx)     TIMx->TIMx_EGR.COMG
#define TIMx_TRIG_GEN(TIMx)             TIMx->TIMx_EGR.TG
#define TIMx_BREAK_GEN(TIMx)            TIMx->TIMx_EGR.BG
//TIMx capture/compare mode register 1
#define TIMx_CAP_COMP_1_SEL(TIMx)       TIMx->TIMx_CCMR1.CC1S
#define TIMx_CAP_COMP_2_SEL(TIMx)       TIMx->TIMx_CCMR1.CC2S
//TIMx capture/compare mode register 2
#define TIMx_CAP_COMP_3_SEL(TIMx)       TIMx->TIMx_CCMR2.CC3S
#define TIMx_CAP_COMP_4_SEL(TIMx)       TIMx->TIMx_CCMR2.CC4S
//TIMx capture/compare enable register
#define TIMx_CAP_COMP_1_OUT_EN(TIMx)    TIMx->TIMx_CCER.CC1OE
#define TIMx_CAP_COMP_1_OUT_POL(TIMx)   TIMx->TIMx_CCER.CC1OP
#define TIMx_CAP_COMP_1N_OUT_EN(TIMx)   TIMx->TIMx_CCER.CC1NOE
#define TIMx_CAP_COMP_1N_OUT_POL(TIMx)  TIMx->TIMx_CCER.CC1NOP
#define TIMx_CAP_COMP_2_OUT_EN(TIMx)    TIMx->TIMx_CCER.CC2OE
#define TIMx_CAP_COMP_2_OUT_POL(TIMx)   TIMx->TIMx_CCER.CC2OP
#define TIMx_CAP_COMP_2N_OUT_EN(TIMx)   TIMx->TIMx_CCER.CC2NOE
#define TIMx_CAP_COMP_2N_OUT_POL(TIMx)  TIMx->TIMx_CCER.CC2NOP
#define TIMx_CAP_COMP_3_OUT_EN(TIMx)    TIMx->TIMx_CCER.CC3OE
#define TIMx_CAP_COMP_3_OUT_POL(TIMx)   TIMx->TIMx_CCER.CC3OP
#define TIMx_CAP_COMP_3N_OUT_EN(TIMx)   TIMx->TIMx_CCER.CC3NOE
#define TIMx_CAP_COMP_3N_OUT_POL(TIMx)  TIMx->TIMx_CCER.CC3NOP
#define TIMx_CAP_COMP_4_OUT_EN(TIMx)    TIMx->TIMx_CCER.CC4OE
#define TIMx_CAP_COMP_4_OUT_POL(TIMx)   TIMx->TIMx_CCER.CC4OP
#define TIMx_CAP_COMP_4N_OUT_EN(TIMx)   TIMx->TIMx_CCER.CC4NOE
#define TIMx_CAP_COMP_4N_OUT_POL(TIMx)  TIMx->TIMx_CCER.CC4NOP
//TIMx counter
#define TIMx_COUNTER_VALUE(TIMx)        TIMx->TIMx_CNT.CNT
//TIMx prescaler
#define TIMx_PSC_VALUE(TIMx)            TIMx->TIMx_PSC.PSC
//TIMx auto-reload register
#define TIMx_LOW_ARR_VALUE(TIMx)        TIMx->TIMx_ARR.LARR
#define TIMx_HIGH_ARR_VALUE(TIMx)       TIMx->TIMx_ARR.HARR
//TIMx repetition counter register
#define TIMx_REP_CNT_VALUE(TIMx)        TIMx->TIMx_RCP.REP
//TIMx capture/compare register 1
#define TIMx_CAP_COMP_1_LVAL(TIMx)      TIMx->TIMx_CCR1.LCCR1
#define TIMx_CAP_COMP_1_HVAL(TIMx)      TIMx->TIMx_CCR1.HCCR1
//TIMx capture/compare register 2
#define TIMx_CAP_COMP_2_LVAL(TIMx)      TIMx->TIMx_CCR2.LCCR2
#define TIMx_CAP_COMP_2_HVAL(TIMx)      TIMx->TIMx_CCR2.HCCR2
//TIMx capture/compare register 3
#define TIMx_CAP_COMP_3_LVAL(TIMx)      TIMx->TIMx_CCR3.LCCR3
#define TIMx_CAP_COMP_3_HVAL(TIMx)      TIMx->TIMx_CCR3.HCCR3
//TIMx capture/compare register 4
#define TIMx_CAP_COMP_4_LVAL(TIMx)      TIMx->TIMx_CCR4.LCCR4
#define TIMx_CAP_COMP_4_HVAL(TIMx)      TIMx->TIMx_CCR4.HCCR4
//TIMx break and dead-time register
#define TIMx_DTG_SET(TIMx, MODE)        TIMx->TIMx_BDTR.DTG = MODE
#define TIMx_LOCK_CNF(TIMx)             TIMx->TIMx_BDTR.LOCK
#define TIMx_OFF_ST_SEL_IDLE(TIMx)      TIMx->TIMx_BDTR.OSSI
#define TIMx_OFF_ST_SEL_RUN(TIMx)       TIMx->TIMx_BDTR.OSSR
#define TIMx_BREAK_EN(TIMx)             TIMx->TIMx_BDTR.BKE
#define TIMx_BREAK_POL(TIMx)            TIMx->TIMx_BDTR.BKP
#define TIMx_AUTO_OUT_EN(TIMx)          TIMx->TIMx_BDTR.AOE
#define TIMx_MAIN_OUT_EN(TIMx)          TIMx->TIMx_BDTR.MOE
//TIMx DMA control register
#define TIMx_DMA_BASE_ADD(TIMx)         TIMx->TIMx_DCR.DBA
#define TIMx_DMA_BURST_LEN(TIMx)        TIMx->TIMx_DCR.DBL
//TIMx DMA address for full transfer
#define TIMx_DMA_BURST_ACCESS(TIMx)     TIMx->TIMx_DMAR.DMAB



/*#if (NUMBER_OF_PROCESSES > 1) && (LOCKING == TRUE)
 lock_process();
#endif
#define GPIO_ANALOG             ((u8) 0x03)                     //Input/output Analog
#define EXT_CLK_MODE            ((u8) 0x07)                     //External Clock Mode
//ADC interrupt and status control (ADC_ISR)
#define ADC_RDY              ADC->ISR.ADRDY                 //ADC ready
#define ADC_EOSMP            ADC->ISR.EOSMP                 //End of sampling flag
#define ADC_EOC              ADC->ISR.EOC                   //End of conversion flag
#define ADC_EOSEQ            ADC->ISR.EOSEQ                 //End of sequence flag
#define ADC_OVR              ADC->ISR.OVR                   //ADC overrun
#define ADC_WD               ADC->ISR.AWD                   //Analog watchdog flag
*/
/******************************************* SYSCFG **************************************************/
typedef struct
{
struct
  {
    u1 MEM_MODE          : 2;  //Memory mapping selection bits
    u1 RSVD1             : 2;  //Reserved
    u1 PA11_PA12_RMP     : 1;  //PA11 and PA12 remapping bit for small packages (28 and 20 pins). Available on STM32F04x devices only.
    u1 RSVD2             : 1;  //Reserved
    u1 IR_MOD            : 2;  //IR Modulation Envelope signal selection. Available on STM32F09x devices only.
    u1 ADC_DMA_RMP       : 1;  //ADC_DMA request remapping bit
    u1 USART1_TX_DMA_RMP : 1;  //USART1_TX_DMA_RMP request remapping bit
    u1 USART1_RX_DMA_RMP : 1;  //USART1_RX_DMA_RMP request remapping bit
    u1 TIM16_DMA_RMP     : 1;  //TIM16 DMA request remapping bit
    u1 TIM17_DMA_RMP     : 1;  //TIM17 DMA request remapping bit
    u1 TIM16_DMA_RMP2    : 1;  //TIM16 alternate DMA request remapping bit (on STM32F07x only)
    u1 TIM17_DMA_RMP2    : 1;  //TIM17 alternate DMA request remapping bit (on STM32F07x only)
    u1 RSVD3             : 1;  //Reserved
    u1 I2C_PBx_FMplus    : 4;  //Fast Mode Plus (FM+) driving capability activation bits
    u1 I2C1_FMpusl       : 1;  //FM+ driving capability activation for I2C1
    u1 I2C2_FMpusl       : 1;  //FM+ driving capability activation for I2C2 (on STM32F07x only)
    u1 I2C_PAx_FMplus    : 2;  //Fast Mode Plus (FM+) driving capability activation bits (on STM32F07x only)
    u1 SPI2_DMA_RMP      : 1;  //SPI2 DMA request remapping bit (on STM32F07x only)
    u1 USART2_DMA_RMP    : 1;  //USART2 DMA request remapping bit (on STM32F07x only)
    u1 USART3_DMA_RMP    : 1;  //USART3 DMA request remapping bit (on STM32F07x only)
    u1 I2C1_DMA_RMP      : 1;  //I2C1 DMA request remapping bit (on STM32F07x only)
    u1 TIM1_DMA_RMP      : 1;  //TIM1 DMA request remapping bit (on STM32F07x only)
    u1 TIM2_DMA_RMP      : 1;  //TIM2 DMA request remapping bit (on STM32F07x only)
    u1 TIM3_DMA_RMP      : 1;  //TIM3 DMA request remapping bit (on STM32F07x only)
    u1 RSVD4             : 1;  //Reserved
  }volatile SYSCFG_CFGR1;      //SYSCFG configuration register 1 (SYSCFG_CFGR1)
   volatile u32 RSVD0;         //Reserved

//EXTI means for STM32f030c6
/*
x000: PA[x] pin 
x001: PB[x] pin 
x010: PC[x] pin 
x011: PD[x] pin
x100: PE[x] pin
x101: PF[x] pin
   example: the expression "SYSCFG->SYSCFG_EXTICR1.EXTI00 = 0b001" relates to external interrupt on pin GPIOB00
*/   
struct
  {
    u1 EXTI00            : 4;  //EXTI00 configuration bits
    u1 EXTI01            : 4;  //EXTI01 configuration bits
    u1 EXTI02            : 4;  //EXTI02 configuration bits
    u1 EXTI03            : 4;  //EXTI03 configuration bits
    u1 RSVD0             : 16; //Reserved
  }volatile SYSCFG_EXTICR1;   //SYSCFG external interrupt configuration register 1 (SYSCFG_EXTICR1)
struct
  {
    u1 EXTI04            : 4;  //EXTI04 configuration bits
    u1 EXTI05            : 4;  //EXTI05 configuration bits
    u1 EXTI06            : 4;  //EXTI06 configuration bits
    u1 EXTI07            : 4;  //EXTI07 configuration bits
    u1 RSVD0             : 16; //Reserved
  }volatile SYSCFG_EXTICR2;   //SYSCFG external interrupt configuration register 2 (SYSCFG_EXTICR1)
struct
  {
    u1 EXTI08            : 4;  //EXTI08 configuration bits
    u1 EXTI09            : 4;  //EXTI09 configuration bits
    u1 EXTI10            : 4;  //EXTI10 configuration bits
    u1 EXTI11            : 4;  //EXTI11 configuration bits
    u1 RSVD0             : 16; //Reserved
  }volatile SYSCFG_EXTICR3;   //SYSCFG external interrupt configuration register 3 (SYSCFG_EXTICR1)
struct
  {
    u1 EXTI12            : 4;  //EXTI12 configuration bits
    u1 EXTI13            : 4;  //EXTI13 configuration bits
    u1 EXTI14            : 4;  //EXTI14 configuration bits
    u1 EXTI15            : 4;  //EXTI15 configuration bits
    u1 RSVD0             : 16; //Reserved
  }volatile SYSCFG_EXTICR4;   //SYSCFG external interrupt configuration register 4 (SYSCFG_EXTICR1)
struct
  {
    u1 LOCKUP_LOCK       : 1;  //Cortex-M0 LOCKUP bit enable bit
    u1 SRAM_PARITY_LOCK  : 1;  //SRAM parity lock bit
    u1 PVD_LOCK          : 1;  //PVD lock enable bit
    u1 RSVD0             : 5;  //Reserved
    u1 SRAM_PEF          : 1;  //SRAM parity error flag
    u1 RSVD1             : 16; //Reserved
  }volatile SYSCFG_CFGR2;  //SYSCFG configuration register 2 (SYSCFG_CFGR2)
}SYSCFG_Type;

#define SYSCFG ((SYSCFG_Type*) SYSCFG_BASE)

//SYSCFG configuration register 1
#define SYSCFG_MEM_MAPPING              SYSCFG->SYSCFG_CFGR1.MEM_MODE
#define SYSCFG_PA11_PA12_RMP            SYSCFG->SYSCFG_CFGR1.PA11_PA12_RMP
#define SYSCFG_IR_MOD                   SYSCFG->SYSCFG_CFGR1.IR_MOD
#define SYSCFG_ADC_DMA_REMAPP           SYSCFG->SYSCFG_CFGR1.ADC_DMA_RMP
#define SYSCFG_USART1_TX_DMA_REMAP      SYSCFG->SYSCFG_CFGR1.USART1_TX_DMA_RMP
#define SYSCFG_USART1_RX_DMA_REMAP      SYSCFG->SYSCFG_CFGR1.USART1_RX_DMA_RMP
#define SYSCFG_TIM16_DMA_REMAP          SYSCFG->SYSCFG_CFGR1.TIM16_DMA_RMP
#define SYSCFG_TIM17_DMA_REMAP          SYSCFG->SYSCFG_CFGR1.TIM17_DMA_RMP
#define SYSCFG_TIM16_DMA_REMAP_2        SYSCFG->SYSCFG_CFGR1.TIM16_DMA_RMP2
#define SYSCFG_TIM17_DMA_REMAP_2        SYSCFG->SYSCFG_CFGR1.TIM17_DMA_RMP2
#define SYSCFG_FMplus_ACTIV_BITS_B      SYSCFG->SYSCFG_CFGR1.I2C_PBx_FMplus
#define SYSCFG_FMpusl_ACTIV_I2C1        SYSCFG->SYSCFG_CFGR1.I2C1_FMpusl
#define SYSCFG_FMpusl_ACTIV_I2C2        SYSCFG->SYSCFG_CFGR1.I2C2_FMpusl
#define SYSCFG_FMplus_ACTIV_BITS_A      SYSCFG->SYSCFG_CFGR1.I2C_PAx_FMplus
#define SYSCFG_SPI2_DMA_REQ_REMAP       SYSCFG->SYSCFG_CFGR1.SPI2_DMA_RMP
#define SYSCFG_USART2_DMA_REQ_REMAP     SYSCFG->SYSCFG_CFGR1.USART2_DMA_RMP
#define SYSCFG_USART3_DMA_REQ_REMAP     SYSCFG->SYSCFG_CFGR1.USART3_DMA_RMP
#define SYSCFG_I2C1_DMA_REQ_REMAP       SYSCFG->SYSCFG_CFGR1.I2C1_DMA_RMP
#define SYSCFG_TIM1_DMA_REQ_REMAP       SYSCFG->SYSCFG_CFGR1.TIM1_DMA_RMP
#define SYSCFG_TIM2_DMA_REQ_REMAP       SYSCFG->SYSCFG_CFGR1.TIM2_DMA_RMP
#define SYSCFG_TIM3_DMA_REQ_REMAP       SYSCFG->SYSCFG_CFGR1.TIM3_DMA_RMP
//SYSCFG external interrupt configuration register 1
#define SYSCFG_EXTI00_CONF              SYSCFG->SYSCFG_EXTICR1.EXTI00
#define SYSCFG_EXTI01_CONF              SYSCFG->SYSCFG_EXTICR1.EXTI01
#define SYSCFG_EXTI02_CONF              SYSCFG->SYSCFG_EXTICR1.EXTI02
#define SYSCFG_EXTI03_CONF              SYSCFG->SYSCFG_EXTICR1.EXTI03
//SYSCFG external interrupt configuration register 2
#define SYSCFG_EXTI04_CONF              SYSCFG->SYSCFG_EXTICR2.EXTI04
#define SYSCFG_EXTI05_CONF              SYSCFG->SYSCFG_EXTICR2.EXTI05
#define SYSCFG_EXTI06_CONF              SYSCFG->SYSCFG_EXTICR2.EXTI06
#define SYSCFG_EXTI07_CONF              SYSCFG->SYSCFG_EXTICR2.EXTI07
//SYSCFG external interrupt configuration register 3
#define SYSCFG_EXTI08_CONF              SYSCFG->SYSCFG_EXTICR3.EXTI08
#define SYSCFG_EXTI09_CONF              SYSCFG->SYSCFG_EXTICR3.EXTI09
#define SYSCFG_EXTI10_CONF              SYSCFG->SYSCFG_EXTICR3.EXTI10
#define SYSCFG_EXTI11_CONF              SYSCFG->SYSCFG_EXTICR3.EXTI11
//SYSCFG external interrupt configuration register 4
#define SYSCFG_EXTI12_CONF              SYSCFG->SYSCFG_EXTICR4.EXTI12
#define SYSCFG_EXTI13_CONF              SYSCFG->SYSCFG_EXTICR4.EXTI13
#define SYSCFG_EXTI14_CONF              SYSCFG->SYSCFG_EXTICR4.EXTI14
#define SYSCFG_EXTI15_CONF              SYSCFG->SYSCFG_EXTICR4.EXTI15
//SYSCFG configuration register 2
#define SYSCFG_M0_LOCKUP_EN             SYSCFG->SYSCFG_CFGR2.LOCKUP_LOCK
#define SYSCFG_SRAM_PAR_LOCK_EN         SYSCFG->SYSCFG_CFGR2.SRAM_PARITY_LOCK
#define SYSCFG_PVD_LOCK_EN              SYSCFG->SYSCFG_CFGR2.PVD_LOCK
#define SYSCFG_SRAM_PAR_EF              SYSCFG->SYSCFG_CFGR2.SRAM_PEF

/******************************************* EXTI **************************************************/
typedef struct
{
//example for stm32f030c6
/*
  Interrupt mask register (EXTI_IMR) is 32 bit. 
  example: To set externall intrerrupt on pin GPIOB09 need to write 0b10.0000.0000 or 0x200 on hex
  EXTI->EXTI_IMR.MRx0 = 0x200;
*/
struct
  {
    u1 MRx0       : 8;  //Interrupt mask on external/internal line 0-7
    u1 MRx1       : 8;  //Interrupt mask on external/internal line 8-15
    u1 MRx2       : 8;  //Interrupt mask on external/internal line 16-23
    u1 MRx4       : 8;  //Interrupt mask on external/internal line 24-31
  }volatile EXTI_IMR;   //Interrupt mask register (EXTI_IMR)
struct
  {
    u1 MRx0       : 8;  //Event mask on external/internal line 0-7
    u1 MRx1       : 8;  //Event mask on external/internal line 8-15
    u1 MRx2       : 8;  //Event mask on external/internal line 16-23
    u1 MRx4       : 8;  //Event mask on external/internal line 24-31
  }volatile EXTI_EMR;   //Event mask register (EXTI_EMR)
   
struct
  {
    u1 TR0        : 8;  //EXTI0 configuration bits
    u1 TR1        : 8;  //EXTI1 configuration bits
    u1 TR2        : 2;  //EXTI2 configuration bits
    u1 RSVD0      : 1;  //Reserved
    u1 TR3        : 4;  //EXTI3 configuration bits
    u1 RSVD1      : 9;  //Reserved
  }volatile EXTI_RTSR;  //Rising trigger selection register (EXTI_RTSR)
struct
  {
    u1 TR0        : 8;  //EXTI0 configuration bits
    u1 TR1        : 8;  //EXTI1 configuration bits
    u1 TR2        : 2;  //EXTI2 configuration bits
    u1 RSVD0      : 1;  //Reserved
    u1 TR3        : 4;  //EXTI3 configuration bits
    u1 RSVD1      : 9;  //Reserved
  }volatile EXTI_FTSR;  //Falling trigger selection register (EXTI_FTSR)
struct
{
    u1 SWIER0     : 8;  //EXTI0 configuration bits
    u1 SWIER1     : 8;  //EXTI1 configuration bits
    u1 SWIER2     : 2;  //EXTI2 configuration bits
    u1 RSVD0      : 1;  //Reserved
    u1 SWIER3     : 4;  //EXTI3 configuration bits
    u1 RSVD1      : 9;  //Reserved
  }volatile EXTI_SWIER; //Software interrupt event register (EXTI_SWIER)
struct
{
    u1 PR0        : 8;  //EXTI0 configuration bits
    u1 PR1        : 8;  //EXTI1 configuration bits
    u1 PR2        : 2;  //EXTI2 configuration bits
    u1 RSVD0      : 1;  //Reserved
    u1 PR3        : 4;  //EXTI3 configuration bits
    u1 RSVD1      : 9;  //Reserved
  }volatile EXTI_PR;    //Pending register (EXTI_PR)
  
//struct
//  {
//    u1 MRx0       : 16; //Interrupt mask on external/internal line 0-15
//    u1 MRx1       : 16; //Interrupt mask on external/internal line 16-31
//  }volatile EXTI_IMR;   //Interrupt mask register (EXTI_IMR)
//struct
//  {
//    u1 MRx0       : 16; //Event mask on external/internal line 0-15
//    u1 MRx1       : 16; //Event mask on external/internal line 16-31
//  }volatile EXTI_EMR;   //Event mask register (EXTI_EMR)
//   
//struct
//  {
//    u1 TR0        : 16; //TR0 configuration bits
//    u1 TR1        : 7;  //TR1 configuration bits
//    u1 RSVD1      : 9;  //Reserved
//  }volatile EXTI_RTSR;  //Rising trigger selection register (EXTI_RTSR)
//struct
//  {
//    u1 TR0        : 16; //TR0 configuration bits
//    u1 TR1        : 7;  //TR1 configuration bits
//    u1 RSVD1      : 9;  //Reserved
//  }volatile EXTI_FTSR;  //Falling trigger selection register (EXTI_FTSR)
//struct
//{
//    u1 SWIER0     : 16; //SWIER0 configuration bits
//    u1 SWIER1     : 7;  //SWIER1 configuration bits
//    u1 RSVD1      : 9;  //Reserved
//  }volatile EXTI_SWIER; //Software interrupt event register (EXTI_SWIER)
//struct
//{
//    u1 PR0        : 16; //Pending bit on line x
//    u1 PR1        : 7;  //Pending bit on line x
//    u1 RSVD1      : 9;  //Reserved
//  }volatile EXTI_PR;    //Pending register (EXTI_PR)
}EXTI_Type;

#define EXTI ((EXTI_Type*) EXTI_BASE)   
//Interrupt mask register
#define EXTI_INT_MASK_00_07             EXTI->EXTI_IMR.MRx0
#define EXTI_INT_MASK_08_15             EXTI->EXTI_IMR.MRx1
#define EXTI_INT_MASK_16_23             EXTI->EXTI_IMR.MRx2
#define EXTI_INT_MASK_24_31             EXTI->EXTI_IMR.MRx4
//Event mask register
#define EXTI_EVENT_MASK_00_07           EXTI->EXTI_EMR.MRx0
#define EXTI_EVENT_MASK_08_15           EXTI->EXTI_EMR.MRx1
#define EXTI_EVENT_MASK_16_23           EXTI->EXTI_EMR.MRx2
#define EXTI_EVENT_MASK_24_31           EXTI->EXTI_EMR.MRx4
//Rising trigger selection register
#define EXTI_RISING_CONF_BITS0          EXTI->EXTI_RTSR.TR0
#define EXTI_RISING_CONF_BITS1          EXTI->EXTI_RTSR.TR1
#define EXTI_RISING_CONF_BITS2          EXTI->EXTI_RTSR.TR2
#define EXTI_RISING_CONF_BITS3          EXTI->EXTI_RTSR.TR3
//Falling trigger selection register
#define EXTI_FALLING_CONF_BITS0         EXTI->EXTI_FTSR.TR0
#define EXTI_FALLING_CONF_BITS1         EXTI->EXTI_FTSR.TR1
#define EXTI_FALLING_CONF_BITS2         EXTI->EXTI_FTSR.TR2
#define EXTI_FALLING_CONF_BITS3         EXTI->EXTI_FTSR.TR3
//Software interrupt event register
#define EXTI_SOFT_CONF_BITS0            EXTI->EXTI_FTSR.TR0
#define EXTI_SOFT_CONF_BITS1            EXTI->EXTI_FTSR.TR1
#define EXTI_SOFT_CONF_BITS2            EXTI->EXTI_FTSR.TR2
#define EXTI_SOFT_CONF_BITS3            EXTI->EXTI_FTSR.TR3
//Pending register
#define EXTI_PENDING_BIT_00_07          EXTI->EXTI_PR.PR0
#define EXTI_PENDING_BIT_08_15          EXTI->EXTI_PR.PR0
#define EXTI_PENDING_BIT_16_17          EXTI->EXTI_PR.PR0
#define EXTI_PENDING_BIT_19_22          EXTI->EXTI_PR.PR0
/******************************************* SPI **************************************************/
typedef struct
{
struct
  {
    u1 CPHA       : 1;  //Clock phase
    u1 CPOL       : 1;  //Clock polarity
    u1 MSTR       : 1;  //Master selection
    u1 BR         : 3;  //Baud rate control
    u1 SPE        : 1;  //SPI enable
    u1 LSBFIRST   : 1;  //Frame format
    u1 SSI        : 1;  //Internal slave select
    u1 SSM        : 1;  //Software slave management
    u1 RXONLY     : 1;  //Receive only mode enabled
    u1 CRCL       : 1;  //CRC length
    u1 CRCNEXT    : 1;  //Transmit CRC next
    u1 CRCEN      : 1;  //Hardware CRC calculation enable
    u1 BIDIOE     : 1;  //Output enable in bidirectional mode
    u1 BIDIMODE   : 1;  //Bidirectional data mode enable
    u1 RSVD0      : 16; //Reserved
  }volatile SPIx_CR1;   //SPI control register 1 (SPIx_CR1) 
struct
  {
    u1 RXDMAEN    : 1;  //Rx buffer DMA enable
    u1 TXDMAEN    : 1;  //Tx buffer DMA enable
    u1 SSOE       : 1;  //SS output enable
    u1 NSSP       : 1;  //NSS pulse management
    u1 FRF        : 1;  //Frame format
    u1 ERRIE      : 1;  //Error interrupt enable
    u1 RXNEIE     : 1;  //RX buffer not empty interrupt enable
    u1 TXEIE      : 1;  //Tx buffer empty interrupt enable
    u1 DS         : 4;  //Data size
    u1 FRXTH      : 1;  //FIFO reception threshold
    u1 LDMA_RX    : 1;  //Last DMA transfer for reception
    u1 LDMA_TX    : 1;  //Last DMA transfer for transmission
    u1 RSVD0      : 17; //Reserved
  }volatile SPIx_CR2;   //SPI control register 2 (SPIx_CR2)
struct
  {
    u1 RXNE       : 1;          //Receive buffer not empty
    u1 TXE        : 1;          //Transmit buffer empty
    u1 RSVD0      : 2;          //Channel side
    u1 CRCERR     : 1;          //CRC error flag
    u1 MODF       : 1;          //Mode fault
    u1 OVR        : 1;          //Overrun flag
    u1 BSY        : 1;          //Busy flag
    u1 FRE        : 1;          //Frame format error
    u1 FRLVL      : 2;          //FIFO reception level
    u1 FTLVL      : 2;          //FIFO transmission level
    u1 RSVD1      : 19;         //Reserved
  }volatile SPIx_SR;            //SPI status register (SPIx_SR)
/*struct
  {
    u1 DR0        : 8;       //Data register
    u1 DR1        : 8;       //Data register
    u16 RSVD0     : 16;       //Reserved
  }volatile SPIx_DR;          //SPI data register (SPIx_DR)
*/
  volatile u32 SPIx_DR;         //SPI data register (SPIx_DR)
 // volatile u16 RSVD0;           //Reserved

struct
{
    u1 CRCPOLY    : 16; //CRC polynomial register
    u1 RSVD0      : 16; //Reserved
  }volatile SPIx_CRCPR; //SPI CRC polynomial register (SPIx_CRCPR)
struct
{
    u1 RXCRC      : 16; //Rx CRC register
    u1 RSVD0      : 16; //Reserved
  }volatile SPIx_RXCRCR; //SPI Rx CRC register (SPIx_RXCRCR)
struct
{
    u1 TXCRC      : 16; //Tx CRC register
    u1 RSVD0      : 16; //Reserved
  }volatile SPIx_TXCRCR; //SPI Tx CRC register (SPIx_TXCRCR)
struct
{
    u1 CHLEN      : 1;  //Channel length (number of bits per audio channel)
    u1 DATLEN     : 2;  //Data length to be transferred
    u1 CKPOL      : 1;  //Steady state clock polarity
    u1 I2SSTD     : 2;  //I2S standard selection
    u1 RSVD0      : 1;  //Reserved
    u1 PCMSYNC    : 1;  //PCM frame synchronization
    u1 I2SCFG     : 2;  //I2S configuration mode
    u1 I2SE       : 1;  //I2S enable
    u1 I2SMOD     : 1;  //I2S mode selection
    u1 RSVD1      : 20; //Reserved
  }volatile SPIx_I2SCFGR;    //SPIx_I2S configuration register (SPIx_I2SCFGR)
struct
{
    u1 I2SDIV     : 8;  //I2S linear prescaler
    u1 ODD        : 1;  //Odd factor for the prescaler
    u1 MCKOE      : 1;  //Master clock output enable
    u1 RSVD0      : 22;  //Reserved
  }volatile SPIx_I2SPR; //SPIx_I2S prescaler register (SPIx_I2SPR)

}SPIx_Type;

#define SPI1 ((SPIx_Type*) SPI1_BASE)
#define SPI2 ((SPIx_Type*) SPI2_BASE)

//SPI control register 1
#define SPIx_CLK_PHASE(SPIx)            SPIx->SPIx_CR1.CPHA
#define SPIx_CLK_POL(SPIx)              SPIx->SPIx_CR1.CPOL
#define SPIx_MSTR_SEL(SPIx)             SPIx->SPIx_CR1.MSTR = 0x1
#define SPIx_SLAV_SEL(SPIx)             SPIx->SPIx_CR1.MSTR = 0x0 //??????
#define SPIx_BAUDRATE(SPIx, B_RATE)     SPIx->SPIx_CR1.BR = B_RATE
#define SPIx_SPI_EN(SPIx)               SPIx->SPIx_CR1.SPE
#define SPIx_LSBMSB(SPIx)               SPIx->SPIx_CR1.LSBFIRST
#define SPIx_IN_SLV_SEL(SPIx)           SPIx->SPIx_CR1.SSI
#define SPIx_SOFT_SLV_MNG(SPIx)         SPIx->SPIx_CR1.SSM
#define SPIx_RXONLY_MODE(SPIx)          SPIx->SPIx_CR1.RXONLY
#define SPIx_CRC_LEN(SPIx)              SPIx->SPIx_CR1.CRCL
#define SPIx_TR_CRC_NEXT(SPIx)          SPIx->SPIx_CR1.CRCNEXT
#define SPIx_HARD_CRC_CALC_EN(SPIx)     SPIx->SPIx_CR1.CRCEN
#define SPIx_OUT_BID_MODE(SPIx)         SPIx->SPIx_CR1.BIDIOE
#define SPIx_BIDIR_DATA_EN(SPIx)        SPIx->SPIx_CR1.BIDIMODE
//SPI control register 2
#define SPIx_RX_BUFF_DMA_EN(SPIx)       SPIx->SPIx_CR2.RXDMAEN
#define SPIx_TX_BUFF_DMA_EN(SPIx)       SPIx->SPIx_CR2.TXDMAEN
#define SPIx_SS_OUT_EN(SPIx)            SPIx->SPIx_CR2.SSOE
#define SPIx_NSS_PULSE_MNG(SPIx)        SPIx->SPIx_CR2.NSSP
#define SPIx_ERR_INT_EN(SPIx)           SPIx->SPIx_CR2.ERRIE
#define SPIx_RX_BUFF_NEMPTY_EN(SPIx)    SPIx->SPIx_CR2.RXNEIE
#define SPIx_TX_BUFF_EMPTY_EN(SPIx)     SPIx->SPIx_CR2.TXEIE
#define SPIx_DATASIZE(SPIx, D_SIZE)     SPIx->SPIx_CR2.DS = D_SIZE
#define SPIx_FIFO_BIT16(SPIx)           SPIx->SPIx_CR2.FRXTH = NULL
#define SPIx_FIFO_BIT08(SPIx)           SPIx->SPIx_CR2.FRXTH = ONE
#define SPIx_LAST_DMA_REC(SPIx)         SPIx->SPIx_CR2.LDMA_RX
#define SPIx_LAST_DMA_TRS(SPIx)         SPIx->SPIx_CR2.LDMA_TX

#define SPIx_RXNE(SPIx)                 SPIx->SPIx_SR.RXNE              //SPI status register
#define SPIx_TXE(SPIx)                  SPIx->SPIx_SR.TXE
#define SPIx_CRC_ERR_FLAG(SPIx)         SPIx->SPIx_SR.CRCERR
#define SPIx_MODE_FAULT(SPIx)           SPIx->SPIx_SR.MODF
#define SPIx_OCR_FLAG(SPIx)             SPIx->SPIx_SR.OVR
#define SPIx_BSY(SPIx)                  SPIx->SPIx_SR.BSY
#define SPIx_FRAMEFORM_ERR(SPIx)        SPIx->SPIx_SR.FRE
#define SPIx_FIFOREC_LEVEL(SPIx)        SPIx->SPIx_SR.FRLVL
#define SPIx_FIFOTRS_LEVEL(SPIx)        SPIx->SPIx_SR.FTLVL

#define SPIx_DATA_REG(SPIx)             SPIx->SPIx_DR                   //SPI data register
#define SPIx_CRC_POL(SPIx)              SPIx->SPIx_CRCPR.CRCPOLY        //SPI CRC polynomial register
#define SPIx_RX_CRC(SPIx)               SPIx->SPIx_RXCRCR.RXCRC         //SPI Rx CRC register
#define SPIx_TX_CRC(SPIx)               SPIx->SPIx_TXCRCR.TXCRC         //SPI Tx CRC register (SPIx_TXCRCR)

#define SPIx_CH_LEN(SPIx)               SPIx->SPIx_I2SCFGR.CHLEN        //SPIx_I2S configuration register
#define SPIx_DATA_LEN(SPIx)             SPIx->SPIx_I2SCFGR.DATLEN
#define SPIx_I2S_CLKPOL(SPIx)           SPIx->SPIx_I2SCFGR.CKPOL
#define SPIx_I2S_SEL(SPIx)              SPIx->SPIx_I2SCFGR.I2SSTD
#define SPIx_PCMFRAME_SYNCH(SPIx)       SPIx->SPIx_I2SCFGR.PCMSYNC
#define SPIx_I2S_CNF(SPIx)              SPIx->SPIx_I2SCFGR.I2SCFG
#define SPIx_I2S_EN(SPIx)               SPIx->SPIx_I2SCFGR.I2SE
#define SPIx_I2S_MOD(SPIx)              SPIx->SPIx_I2SCFGR.I2SOD

#define SPIx_I2S_DIV(SPIx)              SPIx->SPIx_I2SPR.I2SDIV         //SPIx_I2S prescaler register
#define SPIx_I2S_ODDDIV(SPIx)           SPIx->SPIx_I2SPR.ODD
#define SPIx_I2S_MCKOE(SPIx)            SPIx->SPIx_I2SPR.MCKOE

#define F_PLCKDIV_h02                   0                               //SPI baud rates
#define F_PLCKDIV_h04                   1
#define F_PLCKDIV_h08                   2
#define F_PLCKDIV_h10                   3
#define F_PLCKDIV_h20                   4
#define F_PLCKDIV_h40                   5
#define F_PLCKDIV_h80                   6
#define F_PLCKDIV_h100                  7

#define BIT04                           3                               //SPI data size
#define BIT05                           4
#define BIT06                           5
#define BIT07                           6
#define BIT08                           7
#define BIT09                           8
#define BIT10                           9
#define BIT11                           10
#define BIT12                           11
#define BIT13                           12
#define BIT14                           13
#define BIT15                           14
#define BIT16                           15
/******************************************** USB ***************************************************/
typedef struct
{
struct
{
  u1 EA                 :4;  //Endpoint address
  u1 STAT_TX            :2;  //Status bits, for transmission transfers
  u1 DTOG_TX            :1;  //Data Toggle, for transmission transfers
  u1 CTR_TX             :1;  //Correct Transfer for transmission
  u1 EP_KIND            :1;  //Endpoint kind
  u1 EP_TYPE            :2;  //Endpoint type
  u1 SETUP              :1;  //Setup transaction completed
  u1 STAT_RX            :2;  //Status bits, for reception transfers
  u1 DTOG_RX            :1;  //Data Toggle, for reception transfers
  u1 CTR_RX             :1;  //Correct Transfer for reception
  u16 RSVD1;                 //Reserved
}volatile USB_EPnR[8];       //USB endpoint n register (USB_EPnR)
volatile u8 RSVD0[0x20];   //Reserved
//volatile u32 RSVD1[0x40];   //Reserved
struct
{
  u1 FRES             :1;  //Force USB Reset
  u1 PDWN             :1;  //Power down
  u1 LP_MODE          :1;  //Low-power mode 
  u1 FSUSP            :1;  //Force suspend
  u1 RESUME           :1;  //Resume request
  u1 L1RESUME         :1;  //LPM L1 Resume request
  u1 RSVD0            :1;  //Reserved
  u1 L1REQM           :1;  //LPM L1 state request interrupt mask
  u1 ESOFM            :1;  //Expected start of frame interrupt mask
  u1 SOFM             :1;  //Start of frame interrupt mask
  u1 RESETM           :1;  //USB reset interrupt mask
  u1 SUSPM            :1;  //Suspend mode interrupt mask
  u1 WKUPM            :1;  //Wakeup interrupt mask
  u1 ERRM             :1;  //Error interrupt mask
  u1 PMAOVRM          :1;  //Packet memory area over / underrun interrupt mask
  u1 CTRM             :1;  //Correct transfer interrupt mask
  u16 RSVD1;               //Reserved
}volatile USB_CNTR;        //USB control register (USB_CNTR)
struct
{
  u1 EP_ID              :4;  //Endpoint Identifier
  u1 DIR                :1;  //Direction of transaction
  u1 RSVD0              :2;  //Reserved
  u1 L1REQ              :1;  //LPM L1 state request
  u1 ESOF               :1;  //Expected start of frame
  u1 SOF                :1;  //Start of frame
  u1 RESET              :1;  //USB reset request
  u1 SUSP               :1;  //Suspend mode request
  u1 WKUP               :1;  //Wakeup
  u1 ERR                :1;  //Error
  u1 PMAOVR             :1;  //Packet memory area over / underrun
  u1 CTR                :1;  //Correct transfer
  u16 RSVD1;                 //Reserved
}volatile USB_ISTR;          //USB interrupt status register (USB_ISTR)
struct
{
  u1 FN                 :11; //Frame number
  u1 LSOF               :2;  //Lost SOF
  u1 LCK                :1;  //Locked
  u1 RXDM               :1;  //Receive data - line status
  u1 RXDP               :1;  //Receive data + line status
  u16 RSVD1;                 //Reserved
}volatile USB_FNR;           //USB frame number register (USB_FNR)
struct
{
  u1 ADD                :7;  //Device address
  u1 EF                 :1;  //Enable function
  u1 RSVD0              :24; //Reserved
}volatile USB_DADDR;         //USB device address (USB_DADDR)
struct
{
  u1 RSVD0              :3;  //Reserved
  u1 BTABLE             :13; //Buffer table
  u16 RSVD1;                 //Reserved
}volatile USB_BTABLE;        //Buffer table address (USB_BTABLE)
struct
{
  u1 LPMEN              :1;  //LPM support enable
  u1 LPMACK             :1;  //LPM Token acknowledge enable
  u1 RSVD0              :1;  //Reserved
  u1 REMWAKE            :1;  //bRemoteWake value
  u1 BESL               :4;  //BESL value
  u1 RSVD1              :24; //Reserved
}volatile USB_LPMCSR;        //LPM control and status register (USB_LPMCSR)
struct
{
  u1 BCDEN              :1;  //Battery charging detector (BCD) enable
  u1 DCDEN              :1;  //Data contact detection (DCD) mode enable
  u1 PDEN               :1;  //Primary detection (PD) mode enable
  u1 SDEN               :1;  //Secondary detection (SD) mode enable
  u1 DCDET              :1;  //Data contact detection (DCD) status
  u1 PDET               :1;  //Primary detection (PD) status
  u1 SDET               :1;  //Secondary detection (SD) status
  u1 PS2DET             :1;  //DM pull-up detection status
  u1 RSVD0              :7;  //Reserved
  u1 DPPU               :1;  //DP pull-up control
  u16 RSVD1;                 //Reserved
}volatile USB_BCDR;          //Battery charging detector (USB_BCDR)
}USB_Type;

#define USB ((USB_Type*) USB_BASE)

#define USBEPnR_EA(n)           USB->USB_EPnR[n].EA
#define USBEPnR_STAT_TX(n)      USB->USB_EPnR[n].STAT_TX
#define USBEPnR_DTOG_TX(n)      USB->USB_EPnR[n].DTOG_TX
#define USBEPnR_CTR_TX(n)       USB->USB_EPnR[n].CTR_TX
#define USBEPnR_EP_KIND(n)      USB->USB_EPnR[n].EP_KIND
#define USBEPnR_EP_TYPE(n)      USB->USB_EPnR[n].EP_TYPE
#define USBEPnR_SETUP(n)        USB->USB_EPnR[n].SETUP
#define USBEPnR_STAT_RX(n)      USB->USB_EPnR[n].STAT_RX
#define USBEPnR_DTOG_RX(n)      USB->USB_EPnR[n].DTOG_RX
#define USBEPnR_CTR_RX(n)       USB->USB_EPnR[n].CTR_RX



#define USBCNTR_FRES           USB->USB_CNTR.FRES
#define USBCNTR_PWDN           USB->USB_CNTR.PDWN
#define USBCNTR_LP_MODE        USB->USB_CNTR.LP_MODE
#define USBCNTR_FSUSP          USB->USB_CNTR.FSUSP
#define USBCNTR_RESUME         USB->USB_CNTR.RESUME
#define USBCNTR_EL1RESUME      USB->USB_CNTR.L1RESUME
#define USBCNTR_L1REQM         USB->USB_CNTR.L1REQM
#define USBCNTR_ESOFM          USB->USB_CNTR.ESOFM
#define USBCNTR_SOFM           USB->USB_CNTR.SOFM
#define USBCNTR_RESETM         USB->USB_CNTR.RESETM
#define USBCNTR_SUSPM          USB->USB_CNTR.SUSPM
#define USBCNTR_WKUPM          USB->USB_CNTR.WKUPM
#define USBCNTR_ERRM           USB->USB_CNTR.ERRM
#define USBCNTR_PMAOVRM        USB->USB_CNTR.PMAOVRM
#define USBCNTR_CTRM           USB->USB_CNTR.CTRM

#define USB_ISTR_EP_ID          USB->USB_ISTR.EP_ID
#define USB_ISTR_DIR            USB->USB_ISTR.DIR
#define USB_ISTR_L1REQ          USB->USB_ISTR.L1REQ
#define USB_ISTR_ESOF           USB->USB_ISTR.ESOF
#define USB_ISTR_SOF            USB->USB_ISTR.SOF
#define USB_ISTR_RESET          USB->USB_ISTR.RESET
#define USB_ISTR_SUSP           USB->USB_ISTR.SUSP
#define USB_ISTR_WKUP           USB->USB_ISTR.WKUP
#define USB_ISTR_ERR            USB->USB_ISTR.ERR
#define USB_ISTR_PMAOVR         USB->USB_ISTR.PMAOVR
#define USB_ISTR_CTR            USB->USB_ISTR.CTR

#define USB_DADDR_ADD           USB->USB_DADDR.ADD
#define USB_DADDR_EF            USB->USB_DADDR.EF

#define USB_DADDR_ADD_FULL      ((u8)0x7FU)                  /*!< USB device address */
#define USB_DADDR_EF_FULL       ((u8)0x01U)                  /*!< USB device address Enable Function */


#define USB_BCDR_BCDEN          USB->USB_BCDR.BCDEN
#define USB_BCDR_DCDEN          USB->USB_BCDR.DCDEN
#define USB_BCDR_PDEN           USB->USB_BCDR.PDEN
#define USB_BCDR_SDEN           USB->USB_BCDR.SDEN
#define USB_BCDR_DCDET          USB->USB_BCDR.DCDET
#define USB_BCDR_SDET           USB->USB_BCDR.SDET
#define USB_BCDR_PS2DET         USB->USB_BCDR.PS2DET
#define USB_BCDR_DPPU           USB->USB_BCDR.DPPU


#define USB_CLR_REG(REG)        *(u16*)&(USB->REG) = 0;
/******************************************* USART **************************************************/
typedef struct
{
struct
  {
    u1 UE         : 1;  //USART enable
    u1 UESM       : 1;  //USART enable in Stop mode
    u1 RE         : 1;  //Receiver enable
    u1 TE         : 1;  //Transmitter enable
    u1 IDLEIE     : 1;  //IDLE interrupt enable
    u1 RXNEIE     : 1;  //RXNE interrupt enable
    u1 TCIE       : 1;  //Transmission complete interrupt enable
    u1 TXEIE      : 1;  //TXE interrupt enable
    u1 PEIE       : 1;  //PE interrupt enable
    u1 PS         : 1;  //Parity selection
    u1 PCE        : 1;  //Parity control enable
    u1 WAKE       : 1;  //Receiver wakeup method
    u1 M0         : 1;  //Word length
    u1 MME        : 1;  //Mute mode enable
    u1 CMIE       : 1;  //Character match interrupt enable
    u1 OVER8      : 1;  //Oversampling mode
    u1 DEDT       : 5;  //Driver Enable de-assertion time
    u1 DEAT       : 5;  //Driver Enable assertion time
    u1 RTOIE      : 1;  //Receiver timeout interrupt enable
    u1 EOBIE      : 1;  //End of Block interrupt enable
    u1 M1         : 1;  //Word length
    u1 RSVD0      : 3;  //Reserved
  }volatile USARTx_CR1;   //Control register 1 (USARTx_CR1) 
struct
  {
    u1 RSVD0      : 4;  //Reserved
    u1 ADDM7      : 1;  //7-bit Address Detection/4-bit Address Detection
    u1 LBDL       : 1;  //LIN break detection length
    u1 LBDIE      : 1;  //LIN break detection interrupt enable
    u1 RSVD1      : 1;  //Reserved
    u1 LBCL       : 1;  //Last bit clock pulse
    u1 CPHA       : 1;  //Clock phase
    u1 CPOL       : 1;  //Clock polarity
    u1 CLKEN      : 1;  //Clock enable
    u1 STOP       : 2;  //STOP bits
    u1 LINEN      : 1;  //LIN mode enable
    u1 SWAP       : 1;  //Swap TX/RX pins
    u1 RXINV      : 1;  //RX pin active level inversion
    u1 TXINV      : 1;  //TX pin active level inversion
    u1 DATAINV    : 1;  //Binary data inversion
    u1 MSBFIRST   : 1;  //Most significant bit first
    u1 ABREN      : 1;  //Auto baud rate enable
    u1 ABRMOD     : 2;  //Auto baud rate mode
    u1 RTOEN      : 1;  //Receiver timeout enable
    u1 ADDL       : 4;  //Address of the USART node
    u1 ADDH       : 4;  //Address of the USART node
  }volatile USARTx_CR2;   //Control register 2 (USARTx_CR2)
struct
  {
    u1 EIE        : 1;  //Error interrupt enable
    u1 IREN       : 1;  //IrDA mode enable
    u1 IRLP       : 1;  //IrDA low-power
    u1 HDSEL      : 1;  //Half-duplex selection
    u1 NACK       : 1;  //Smartcard NACK enable
    u1 SCEN       : 1;  //Smartcard mode enable
    u1 DMAR       : 1;  //DMA enable receiver
    u1 DMAT       : 1;  //DMA enable transmitter
    u1 RTSE       : 1;  //RTS enable
    u1 CTSE       : 1;  //CTS enable
    u1 CTSIE      : 1;  //CTS interrupt enable
    u1 ONEBIT     : 1;  //One sample bit method enable
    u1 OVRDIS     : 1;  //Overrun Disable
    u1 DDRE       : 1;  //DMA Disable on Reception Error
    u1 DEM        : 1;  //Driver enable mode
    u1 DEP        : 1;  //Driver enable polarity selection
    u1 RSVD0      : 1;  //Reserved
    u1 SCARCNT    : 3;  //Smartcard auto-retry count
    u1 WUS        : 2;  //Wakeup from Stop mode interrupt flag selection
    u1 WUFIE      : 1;  //Wakeup from Stop mode interrupt enable
    u1 RSVD1      : 9;  //Reserved
  }volatile USARTx_CR3; //Control register 3 (USARTx_CR3)
struct
{
    u1 BRRFRACT   : 4;  //
    u1 BRRMANT    : 12; //
    u1 RSVD0      : 16; //Reserved
  }volatile USARTx_BRR; //Baud rate register (USARTx_BRR)
struct
{
    u1 PSC        : 8;  //Prescaler value
    u1 GT         : 8;  //Guard time value
    u1 RSVD0      : 16; //Reserved
  }volatile USARTx_GTPR; //Guard time and prescaler register (USARTx_GTPR)
struct
{
    u1 RTO        : 24; //Receiver timeout value
    u1 BLEN       : 8;  //Block Length
  }volatile USARTx_RTOR; //Receiver timeout register (USARTx_RTOR)
struct
{
    u1 ABRRQ      : 1;  //Auto baud rate request
    u1 SBKRQ      : 1;  //Send break request
    u1 MMRQ       : 1;  //Mute mode request
    u1 RXFRQ      : 1;  //Receive data flush request
    u1 TXFRQ      : 1;  //Transmit data flush request
    u1 RSVD1      : 27; //Reserved
  }volatile USARTx_RQR;    //Request register (USARTx_RQR)
struct
{
    u1 PE         : 1;  //Parity error
    u1 FE         : 1;  //Framing error
    u1 NF         : 1;  //START bit Noise detection flag
    u1 ORE        : 1;  //Overrun error
    u1 IDLE       : 1;  //Idle line detected
    u1 RXNE       : 1;  //Read data register not empty
    u1 TC         : 1;  //Transmission complete
    u1 TXE        : 1;  //Transmit data register empty
    u1 LBDF       : 1;  //LIN break detection flag
    u1 CTSIF      : 1;  //CTS interrupt flag
    u1 CTS        : 1;  //CTS flag
    u1 RTOF       : 1;  //Receiver timeout
    u1 EOBF       : 1;  //End of block flag
    u1 RSVD0      : 1;  //Reserved
    u1 ABRE       : 1;  //Auto baud rate error
    u1 ABRF       : 1;  //Auto baud rate flag
    u1 BUSY       : 1;  //Busy flag
    u1 CMF        : 1;  //Character match flag
    u1 SBKF       : 1;  //Send break flag
    u1 RWU        : 1;  //Receiver wakeup from Mute mode
    u1 WUF        : 1;  //Wakeup from Stop mode flag
    u1 TEACK      : 1;  //Transmit enable acknowledge flag
    u1 REACK      : 1;  //Receive enable acknowledge flag
    u1 RSVD1      : 9; //Reserved
  }volatile USARTx_ISR; //Interrupt & status register (USARTx_ISR)
struct
{
    u1 PECF       : 1;  //Parity error clear flag
    u1 FECF       : 1;  //Framing error clear flag
    u1 NCF        : 1;  //Noise detected clear flag
    u1 ORECF      : 1;  //Overrun error clear flag
    u1 IDLECF     : 1;  //Idle line detected clear flag
    u1 RSVD0      : 1;  //Reserved
    u1 TCCF       : 1;  //Transmission complete clear flag
    u1 RSVD1      : 1;  //Reserved
    u1 LBDCF      : 1;  //LIN break detection clear flag
    u1 CTSCF      : 1;  //CTS clear flag
    u1 RSVD2      : 1;  //Reserved
    u1 RTOCF      : 1;  //Receiver timeout clear flag
    u1 EOBCF      : 4;  //End of block clear flag
    u1 RSVD3      : 1;  //Reserved
    u1 CMCF       : 1;  //Character match clear flag
    u1 RSVD4      : 2;  //Reserved
    u1 WUCF       : 1;  //Wakeup from Stop mode clear flag
    u1 RSVD5      : 11; //Reserved
  }volatile USARTx_ICR; //Interrupt flag clear register (USARTx_ICR)
struct
{
    u1 RDR        : 9;  //Receive data value
    u1 RSVD0      : 23; //Reserved
  }volatile USARTx_RDR; //Receive data register (USARTx_RDR)
struct
{
    u1 TDR        : 9;  //Transmit data value
    u1 RSVD0      : 23; //Reserved
  }volatile USARTx_TDR; //Transmit data register (USARTx_TDR)

}USARTx_Type;

#define USART1 ((USARTx_Type*) UART1_BASE)

//Control register 1
#define USARTx_EN(SPIx)                 SPIx->USARTx_CR1.UE
#define USARTx_EN_STOPMODE(SPIx)        SPIx->USARTx_CR1.UESM
#define USARTx_REC_EN(SPIx)             SPIx->USARTx_CR1.RE
#define USARTx_TRS_EN(SPIx)             SPIx->USARTx_CR1.TE
#define USARTx_IDLEINT_EN(SPIx)         SPIx->USARTx_CR1.IDLEIE
#define USARTx_RXNEINT_EN(SPIx)         SPIx->USARTx_CR1.RXNEIE
#define USARTx_TRSCOMPINT_EN(SPIx)      SPIx->USARTx_CR1.TCIE
#define USARTx_TXEINT_EN(SPIx)          SPIx->USARTx_CR1.TXEIE
#define USARTx_PEINT_EN(SPIx)           SPIx->USARTx_CR1.PEIE
#define USARTx_PARITY_SEL(SPIx)         SPIx->USARTx_CR1.PS
#define USARTx_PARCTRL_EN(SPIx)         SPIx->USARTx_CR1.PCE
#define USARTx_RECW_METHOD(SPIx)        SPIx->USARTx_CR1.WAKE
#define USARTx_W_LEN0(SPIx)             SPIx->USARTx_CR1.M0
#define USARTx_MUTEMODE_EN(SPIx)        SPIx->USARTx_CR1.MME
#define USARTx_CHARINT_EN(SPIx)         SPIx->USARTx_CR1.CMIE
#define USARTx_OVERS_MODE(SPIx)         SPIx->USARTx_CR1.OVER8
#define USARTx_DEDT(SPIx)               SPIx->USARTx_CR1.DEDT
#define USARTx_DEAT(SPIx)               SPIx->USARTx_CR1.DEAT
#define USARTx_RECTIMOUTINT_EN(SPIx)    SPIx->USARTx_CR1.RTOIE
#define USARTx_ENDOFBLOCKINT_EN(SPIx)   SPIx->USARTx_CR1.EOBIE
#define USARTx_W_LEN1(SPIx)             SPIx->USARTx_CR1.M1

/******************************************* END **************************************************/
#ifdef __cplusplus
}
#endif