#define USB_BTABLE_BASE                 0x40006000
#define USB_BTABLE_ST			((USB_BtableDef *)(USB_BTABLE_BASE))
#define USB_BTABLE_START                0x0000
#define USB0_TX_BUFF_ADDR               0x0040
#define USB0_TX_BUFF_SIZE               0x0040
#define USB0_RX_BUFF_ADDR               0x0080

//Типы конечных точек
#define EP_TYPE_BULK                    0x00
#define EP_TYPE_CONTROL			0x01
#define EP_TYPE_ISO			0x02
#define EP_TYPE_INTERRUPT		0x03
//Стандартные запросы
#define GET_DESCRIPTOR                  0x06
#define SET_DESCRIPTOR                  0x07
#define SET_ADDRESS                     0x05
#define SET_FEATURE                     0x03
#define CLEAR_FEATURE                   0x01
#define GET_STATUS                      0x00
#define GET_CONFIGURATION               0x08
#define SET_CONFIGURATION               0x09
#define GET_INTERFACE                   0x0a
#define SET_INTERFCE                    0x0b
#define SYNC_FRAME                      0x0c
//Параметры запроса
#define DEVICE_DESC                     0x100
#define CONFIGURATION_DESC              0x200
#define HID_REPORT_DESC                 0x2200
#define STRING_LANG_DESC                0x300
#define STRING_MANUFAC_DESC             0x301
#define STRING_PROD_DESC                0x302
#define STRING_SN_DESC                  0x303
#define DEVICE_QUALIFIER_DESC           0x600
//Длина дескрипторов
#define DEVICE_DESC_SIZE                18
#define CONFIG_DESC_SIZE                41
#define DEVICE_QUALIFIER_SIZE           10
#define STRING_LANG_DESC_SIZE           4
#define STRING_MANUFAC_DESC_SIZE        12
#define STRING_PROD_DESC_SIZE           20
#define STRING_SN_DESC_SIZE             18
#define REPORT_DESC_SIZE                51


#define USB_CNTR_LP_MODE		0x00000004
#define USB_CNTR_PWDN			0x00000002
#define USB_ISTR_EPID			0x0000000F
#define USB_FNR_LSOF_0			0x00000800
#define USB_FNR_lSOF_1			0x00001000
#define USB_LPMCSR_BESL_0		0x00000010
#define USB_LPMCSR_BESL_1		0x00000020
#define USB_LPMCSR_BESL_2		0x00000040
#define USB_LPMCSR_BESL_3		0x00000080
#define USB_LPMCSR_LPMEN		0x00000001
#define USB_EPnR_CTR_RX			0x00008000
#define USB_EPnR_DTOG_RX		0x00004000
#define USB_EPnR_STAT_RX		0x00003000
#define USB_EPnR_STAT_RX_0		0x00001000
#define USB_EPnR_STAT_RX_1		0x00002000
#define USB_EPnR_SETUP			0x00000800
#define USB_EPnR_EP_TYPE		0x00000600
#define USB_EPnR_EP_TYPE_0		0x00000200
#define USB_EPnR_EP_TYPE_1		0x00000400
#define USB_EPnR_EP_KIND		0x00000100
#define USB_EPnR_CTR_TX			0x00000080
#define USB_EPnR_DTOG_TX		0x00000040
#define USB_EPnR_STAT_TX		0x00000030
#define USB_EPnR_STAT_TX_0		0x00000010
#define USB_EPnR_STAT_TX_1		0x00000020
#define USB_EPnR_EA			0x0000000F
#define USB_COUNTn_RX_BLSIZE	        0x00008000
#define USB_COUNTn_NUM_BLOCK	        0x00007C00
#define USB_COUNTn_RX			0x0000003F

//Состояния соединения USB
#define USB_DEFAULT_STATE				0
#define USB_ADRESSED_STATE				1
#define USB_CONFIGURE_STATE				2


//#define CLEAR_DTOG_RX(bit)              (bit) ? bit : 0 //bit = USBEPnR_DTOG_RX(n)
//#define SET_DTOG_RX(bit)                (bit) ? 0 : 1
//#define TOGGLE_DTOG_RX(bit)             bit = 1
//#define KEEP_DTOG_RX(bit)               bit = 0
//#define CLEAR_DTOG_TX(R)                (bit) ? bit : 0 //bit = USBEPnR_DTOG_TX(n)
//#define SET_DTOG_TX(R)                  (bit) ? 0 : 1
//#define TOGGLE_DTOG_TX(R)               bit = 1
//#define KEEP_DTOG_TX(R)                 bit = 0
//
//#define SET_VALID_RX(bit)               bit = (~bit)^bit
//#define SET_NAK_RX(bit)                 bit = (bit&0x10) ? (bit|0x10) : (bit|0x00)
//#define SET_STALL_RX(bit)               bit = (bit&0x01) ? (bit|0x01) : (bit|0x00)
//#define KEEP_STAT_RX(bit)               bit = 0x00
//#define SET_VALID_TX(bit)               bit = (~bit)^bit
//#define SET_NAK_TX(bit)                 bit = (bit&0x10) ? (bit|0x10) : (bit|0x00)
//#define SET_STALL_TX(bit)               bit = (bit&0x01) ? (bit|0x01) : (bit|0x00)
//#define KEEP_STAT_TX(bit)               bit = 0x00
#define SET_VALID_TX(R)                 ((R & USB_EPnR_STAT_TX) ^ USB_EPnR_STAT_TX)   | (R & (~USB_EPnR_STAT_TX))
#define KEEP_STAT_RX(R)                 (R&(~USB_EPnR_STAT_RX))
#define SET_DTOG_TX(R)                  (R&USB_EPnR_DTOG_TX) ?     (R & (~USB_EPnR_DTOG_TX)) : (R | USB_EPnR_DTOG_TX)
#define KEEP_DTOG_RX(R)                 (R&(~USB_EPnR_DTOG_RX))
#define SET_VALID_RX(R)                 ((R&USB_EPnR_STAT_RX) ^ USB_EPnR_STAT_RX)   | (R & (~USB_EPnR_STAT_RX))
#define SET_NAK_TX(R)                   ((R&USB_EPnR_STAT_TX) ^ USB_EPnR_STAT_TX_1) | (R & (~USB_EPnR_STAT_TX))
#define CLEAR_DTOG_TX(R)                (R&USB_EPnR_DTOG_TX) ? R : (R & (~USB_EPnR_DTOG_TX))
#define CLEAR_DTOG_RX(R)                (R&USB_EPnR_DTOG_RX) ? R : (R&(~USB_EPnR_DTOG_RX))
#define CLEAR_CTR_RX_TX(R)              (R&(~(USB_EPnR_CTR_TX | USB_EPnR_CTR_RX | USB_EPnR_STAT_RX | USB_EPnR_STAT_TX | USB_EPnR_DTOG_RX | USB_EPnR_DTOG_RX)))
#define KEEP_DTOG_TX(R)                 (R&(~USB_EPnR_DTOG_TX))


#define getptr(strc)            (u16*)&strc
#define ptr_                    (u8*)
#define ptr_pbm(addr)           *(u16*)(addr)
#define reg(R)                  *(u32*)&(USB->R)
u16 read_ep = 0;

struct
{
  u16 tx_buf_add;
  u16 tx_buf_dat;
  u16 rx_buf_add;
  u16 rx_buf_dat;
}EP_Read;
struct
{
  u16 tx_buf_add;
  u16 tx_buf_dat;
  u16 rx_buf_add;
  u16 rx_buf_dat;
}EP0_set[8] = {USB0_TX_BUFF_ADDR, USB0_TX_BUFF_SIZE, USB0_RX_BUFF_ADDR, 0x8400};

typedef struct {
  u8  bmRequestType;
  u8  bRequest;
  u16 wValue;
  u16 wIndex;
  u16 wLength;
//  u16 wdata[4];
} Conf_Pack_Typedef;

Conf_Pack_Typedef EP_Setup;
Conf_Pack_Typedef *EP0_rx = ((Conf_Pack_Typedef*) (USB_BTABLE_BASE + (u16)(USB0_RX_BUFF_ADDR)));
struct{
  u8  bmRequestType;
  u8  bRequest;
  u16 wValue;
  u16 wIndex;
  u16 wLength;
//  u16 wdata[2];
}a_[64];
//Bind_type *Bind = ((Bind_type*) F_PAGE31_STR);
struct
{
  u8  bmRequestType;
  u8  bRequest;
  u16 wValue;
  u16 wIndex;
  u16 wLength;
  u8 wdata[4];
}EP_Data;
u8 EP_Data_1[64];
u16 read_buff[24];
u8 r = 0x55;
u8 bm_req=0;
u8 desc_len  = 0;
u32 addr32 = 1;
u16 cn_rst = 0;
struct {
  u8 dir;
  u8 setup;
  u8 cn_istr_dir0;
  u8 cn_istr_dir1;
  u8 cn_ep_setup0;
  u8 cn_ep_setup1;
  
}ISTR_Dir;

typedef struct
{
  u16 tx_buf_add;
  u16 tx_buf_dat;
  u16 rx_buf_add;
  u16 rx_buf_dat;
}EP_Write[2];

typedef struct
{
  u16 tx_buf_add;
  u16 tx_buf_dat;
  u16 rx_buf_add;
  u16 rx_buf_dat;
}EP_Read_Req;

struct{
  u1 ep_addr    :4;
  u1 stat_tx    :2;
  u1 dtog_tx    :1;
  u1 ctr_tx     :1;
  u1 ep_kind    :1;
  u1 ep_type    :2;
  u1 ep_setup   :1;
  u1 stat_rx    :2;
  u1 dtog_rx    :1;
  u1 ctr_rx     :1;
  u1 rsv        :16;
}Ep_Stat;
  
typedef struct
{
  u16 *tx_buf;
  u8  *rx_buf;
  u16 status;
  u1 rx_cnt             :10;
  u1 tx_flag            :1;
  u1 rx_flag            :1;
  u1 setup_flag         :1;
}ep_t;
ep_t endpoints[2];
struct
{
  u8 USB_Status;
  u8 USB_Addr;
}USB_SetStatus;

  //    #define USBEPnR_EA(n)           USB->USB_EPnR[n].EA
//#define USBEPnR_STAT_TX(n)      USB->USB_EPnR[n].STAT_TX
//#define USBEPnR_DTOG_TX(n)      USB->USB_EPnR[n].DTOG_TX
//#define USBEPnR_CTR_TX(n)       USB->USB_EPnR[n].CTR_TX
//#define USBEPnR_EP_KIND(n)      USB->USB_EPnR[n].EP_KIND
//#define USBEPnR_EP_TYPE(n)      USB->USB_EPnR[n].EP_TYPE
//#define USBEPnR_SETUP(n)        USB->USB_EPnR[n].SETUP
//#define USBEPnR_STAT_RX(n)      USB->USB_EPnR[n].STAT_RX
//#define USBEPnR_DTOG_RX(n)      USB->USB_EPnR[n].DTOG_RX
//#define USBEPnR_CTR_RX(n)       USB->USB_EPnR[n].CTR_RX


#define USB_EP_WRITE            ((EP_Write*) USB_BTABLE_BASE)
#define USB_EP_READ_REQ         ((EP_Read_Req*)  USB_BTABLE_BASE + 16)
#define EP0_CONF_PACK           ((Conf_Pack_Typedef*) USB_BTABLE_BASE + (u16)(USB0_RX_BUFF_ADDR>>3))//EP0_set[8].rx_buf_add)

void Read_PBM(u8 size,  u16 *buf, u16 addr);
void Read_PBM8(u8 size, u16 *buf, u16 addr);
void Write_PBM(u8 size, u16 *buf, u16 addr);
void EP_Read_Func_1(void/*u8 *buf*/);
//void EP0_Init(void);
