#include "M0_Core.h"
#include "inc.h"
#include "desc.h"
#include "src.c"

void NVIC_IRQ_ON (u32 IRQ) {NVIC->ISER[IRQ >> 5] = (1 << (IRQ & 0x1F));}
void NVIC_IRQ_OFF(u32 IRQ) {NVIC->ICER[IRQ >> 5] = (1 << (IRQ & 0x1F));}
void NVIC_IRQ_SET(u32 IRQ) {NVIC->ISPR[IRQ >> 5] = (1 << (IRQ & 0x1F));}
void NVIC_IRQ_RST(u32 IRQ) {NVIC->ICPR[IRQ >> 5] = (1 << (IRQ & 0x1F));}
void RCC_init(void)
{
  
  RCC_CRS_EN = 1;
  RCC_USB_SW = 0;
  RCC_HSI48_EN = 1;
  while(!RCC_HSI48_RDY);
  FLASH_PRFTBE = 1;
  FLASH_LATENCY = 1;
//  CRS_SYNCSRC = 0;
  CRS_SYNCSRC = 2;
  CRS_AUTOTRIMEN = 1;
  CRS_CEN = 1;
  RCC_SYSCLK = RCC_SYS_HSI48;
  
//  RCC_GPIOA_EN = ON;
//  RCC_HSE_EN = 1;
//  while(!RCC_HSE_RDY);
//  RCC_PLL_CLK = RCC_PLLSRC_HSE;
//  RCC_PLL_MUX = RCC_PLL_MUX06;
//  RCC_PLL_EN = 1;
//  while(!RCC_PLL_RDY);
//  RCC_AHB_DIV = RCC_AHB_DIV001;
//  RCC_USB_SW = RCC_USB_PLLCLK;
//  FLASH->ACR.PRFTBE = 1;
//  FLASH->ACR.LATENCY = 1;
}

void USB_Init(void)
{
  
  RCC_USB_EN = 1;
//  NVIC_IRQ_ON(USB_IRQ);
  SYSCFG_PA11_PA12_RMP = 1;
  USBCNTR_FRES = 0;
  USBCNTR_PWDN = 0;
  USBCNTR_CTRM = 1;
  USBCNTR_RESETM = 1;
  *(u16*)&(USB->USB_ISTR) = 0;
  USB_BCDR_DPPU = 1;
  NVIC->ISER[0] = (1 << ((u32)(USB_IRQ) & 0x1F));
}
int main()
{
  RCC_init();
  USB_Init();
  while(1)
  {
    Reply_To_Setup();
//    Read_PBM8(12, (u16*)&EP_Data, USB_BTABLE_START+EP0_set[0].rx_buf_add);
    
  }
}