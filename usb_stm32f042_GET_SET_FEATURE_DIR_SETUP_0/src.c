
void Write_To_EP(u8 num, u8 *buf, u8 len)
{
  USB_EP_WRITE[0]->tx_buf_add = 0x0040;
  USB_EP_WRITE[0]->tx_buf_dat = 0x0040;
  USB_EP_WRITE[0]->rx_buf_add = 0x0080;
  USB_EP_WRITE[0]->rx_buf_dat = 0x8400;

  EP_Read.tx_buf_add = USB_EP_READ_REQ->tx_buf_add;
  EP_Read.tx_buf_dat = USB_EP_READ_REQ->tx_buf_dat;
  EP_Read.rx_buf_add = USB_EP_READ_REQ->rx_buf_add;
  EP_Read.rx_buf_dat = USB_EP_READ_REQ->rx_buf_dat;
}
//PBM - packet buffer memory
//size - размер принимаемого масива в байтах (8 бит)
//перед отправкой следует учесть эту размерность, т.е.
//если отправляемые массив или структура длиной 8 элементов 
//по 4 байта, то size = 32
//addr - смещение относительно 0x40006000
void Write_PBM(u8 size, u16 *buf, u16 addr)
{
  size = (size&0x01) ? (size+1)/2 : size/2;
  for(u8 cnt=0; cnt<=size; cnt++) {*(u16*)(USB_BTABLE_BASE + addr + cnt*2) = *buf++;}
}
void Read_PBM(u8 size, u16 *buf, u16 addr)
{
  if(size&0x01 && addr32 && r) {*(u8*)(buf+(u8)(size>>1)) = *(u16*)(USB_BTABLE_BASE + addr + (size-1));}
  for(u8 cnt=0; cnt<size>>1; cnt++){*buf++ = *(u16*)(USB_BTABLE_BASE + addr + cnt*2);}
}
void EP_Write_Func(void)
{
  Write_PBM(desc_len, (u16*)USB_DeviceDesc, USB_BTABLE_START+EP0_set[0].tx_buf_add);
  u32 timeout = 1000000;
  u16 status = *(u16*)&(USB->USB_EPnR[0]);
//  USB_BTABLE_ST->EP[number].USB_COUNT_TX = size;
  USB_EP_WRITE[0]->tx_buf_dat = desc_len;
  status = KEEP_STAT_RX(status);        //RX в NAK
  status = SET_VALID_TX(status);        //TX в VALID
  status = KEEP_DTOG_TX(status);
  status = KEEP_DTOG_RX(status);
  *(u16*)&(USB->USB_EPnR[0]) = status;
  
  endpoints[0].tx_flag = 0;
  while(!endpoints[0].tx_flag)
  {
    if(timeout) timeout--;
    else break;
  }

}        
void EP_Write_Func_1(u8 len, u16 *buf)
{
  Write_PBM(len, buf, USB_BTABLE_START+EP0_set[0].tx_buf_add);
  u32 timeout = 1000000;
  u16 status = *(u16*)&(USB->USB_EPnR[0]);
//  USB_BTABLE_ST->EP[number].USB_COUNT_TX = size;
  USB_EP_WRITE[0]->tx_buf_dat = len;
  status = KEEP_STAT_RX(status);        //RX в NAK
  status = SET_VALID_TX(status);        //TX в VALID
  status = KEEP_DTOG_TX(status);
  status = KEEP_DTOG_RX(status);
  *(u16*)&(USB->USB_EPnR[0]) = status;
  
  endpoints[0].tx_flag = 0;
  while(!endpoints[0].tx_flag)
  {
    if(timeout) timeout--;
    else break;
  }

}
void EP_Read_Func_1(void/*u8 *buf*/)
{
	u32 timeout = 10000000;
	u16 status;
	status = *(u16*)&(USB->USB_EPnR[0]);
	status = SET_VALID_RX(status);
	status = SET_NAK_TX(status);
	status = KEEP_DTOG_TX(status);
	status = KEEP_DTOG_RX(status);
//        status = SET_VALID_TX(status);
//        status = KEEP_STAT_RX(status);
//        status = SET_DTOG_TX(status);
//        status = KEEP_DTOG_RX(status);
	*(u16*)&(USB->USB_EPnR[0]) = status;
	endpoints[0].rx_flag = 0;
	while (!endpoints[0].rx_flag)
        {
          if (!(ISTR_Dir.dir)&&!(ISTR_Dir.setup))
          {
            break;
          }
//		if (timeout) timeout--;
//		else break;
	}
//	Read_PBM8(8, getptr(EP_Read), USB_BTABLE_START+EP0_set[0].rx_buf_add);
        Read_PBM8(32, (u16*)&EP_Data_1, USB_BTABLE_START+EP0_set[0].rx_buf_add);
        Read_PBM8(32, (u16*)&EP_Data_1, USB_BTABLE_START);
}
void EP_Send_Null(void)
{
  u32 timeout = 100000;
  u16 status = *(u16*)&(USB->USB_EPnR[0]);//USB->EPnR[number];
  USB_EP_WRITE[0]->tx_buf_dat = 0;
  status = KEEP_STAT_RX(status);
  status = SET_VALID_TX(status);
  status = KEEP_DTOG_RX(status);
  status = SET_DTOG_TX(status);
  *(u16*)&(USB->USB_EPnR[0]) = status;
//  USB->EPnR[number]= status;
  while(!endpoints[0].tx_flag)
  {
    if(timeout) timeout--;
    else break;
  }
  endpoints[0].tx_flag = 0;
}

void EP_Wait_Null(void)
{
//  *(u16*)&(Ep_Stat) = *(u16*)&(USB->USB_EPnR[0]);
  u32 timeout = 100000;
  u16 status = *(u16*)&(USB->USB_EPnR[0]);
  status = SET_VALID_TX(status);
  status = KEEP_STAT_RX(status);
  status = SET_DTOG_TX(status);
  status = KEEP_DTOG_RX(status);
  *(u16*)&(USB->USB_EPnR[0]) = status;
  
  while(!endpoints[0].rx_flag)
  {
    if(timeout) timeout--;
    else break;
  }
  endpoints[0].rx_flag = 0;
}

void EP0_Init(void)
{
  USBEPnR_EA(0) = 0;
  USBEPnR_EP_TYPE(0) = EP_TYPE_CONTROL;
  *(u32*)&(USB->USB_EPnR[0]) ^= USB_EPnR_STAT_RX|USB_EPnR_STAT_TX_1;
  Write_PBM(8, getptr(EP0_set[0]), USB_BTABLE_START);
  USB_DADDR_EF = 1;
  endpoints[0].tx_buf = (u16 *)(USB_BTABLE_BASE + EP0_set[0].tx_buf_add);
  endpoints[0].rx_buf = (u8  *)(USB_BTABLE_BASE + EP0_set[0].rx_buf_add);
}


void Reply_To_Setup(void)
{
  u16 status;
  //-------    *********   **      **    *********   **      **         --------
  //-------    *********   **      **   **********   **    **           --------
  //-------    **          **      **   **           **  **             --------
  //-------    **          **      **   **           ***                --------
  //-------    *********   **      **   **           ****               --------
  //-------    *********   **      **   **           ** **              --------
  //-------    **          **      **   **           **  **             --------
  //-------    **          **********   **********   **    **           --------
  //-------    **           ********     *********   **      **         --------
  //Временно, для отладки, чтобы не искать и не тыкаться в поисках чему эти 
  //переменные все равны. 
  //После, ДЛЯ КОНЕЧНОГО ПРОЕКТА УБРАТЬ НАХЕР!!! чтобы не мусолить
  //и того малый объем памяти. Ну и защита от вопросов типа, "НАХЕРА?"
  EP_Setup.bmRequestType = EP0_CONF_PACK->bmRequestType;
  EP_Setup.bRequest = EP0_CONF_PACK->bRequest;
  EP_Setup.wValue = EP0_CONF_PACK->wValue;
  EP_Setup.wIndex = EP0_CONF_PACK->wIndex;
  EP_Setup.wLength = EP0_CONF_PACK->wLength;
//  EP_Setup.wdata[0] = EP0_CONF_PACK->wdata[0];
//  EP_Setup.wdata[1] = EP0_CONF_PACK->wdata[1];
//  EP_Setup.wdata[2] = EP0_CONF_PACK->wdata[2];
//  EP_Setup.wdata[3] = EP0_CONF_PACK->wdata[3];
  
  if(EP0_rx->bmRequestType) 
  {
    desc_len = EP0_rx->bmRequestType;
  }
  desc_len = EP0_CONF_PACK->wLength;
  a_[cn_rst].bmRequestType = EP0_CONF_PACK->bmRequestType;
  a_[cn_rst].bRequest      = EP0_CONF_PACK->bRequest;
  a_[cn_rst].wValue        = EP0_CONF_PACK->wValue;
  a_[cn_rst].wIndex        = EP0_CONF_PACK->wIndex;
  a_[cn_rst].wLength       = EP0_CONF_PACK->wLength;
//  a_[cn_rst].wdata[0]       = EP0_CONF_PACK->wdata[0];
//  a_[cn_rst].wdata[1]       = EP0_CONF_PACK->wdata[1];
//  a_[cn_rst].wdata[2]       = EP0_CONF_PACK->wdata[2];
//  a_[cn_rst].wdata[3]       = EP0_CONF_PACK->wdata[3];
  if(cn_rst >= 63) cn_rst = 0;
  if(desc_len){desc_len = EP0_CONF_PACK->wLength;}
  if(endpoints[0].rx_flag)
  {
    if(endpoints[0].setup_flag)
    {
      switch(EP0_CONF_PACK->bmRequestType)
      {
      case 0x80:
        switch(EP0_CONF_PACK->bRequest)
        {
        case GET_DESCRIPTOR:
          {
            switch(EP0_CONF_PACK->wValue)
            {
            case DEVICE_DESC:
              if(USB_SetStatus.USB_Addr)
                USB_SetStatus.USB_Addr = USB_SetStatus.USB_Addr;
              desc_len = (desc_len < DEVICE_DESC_SIZE) ? EP0_CONF_PACK->wLength : DEVICE_DESC_SIZE;
    //          Write_PBM(desc_len, (u16*)USB_DeviceDescriptor, USB_BTABLE_START+EP0_set[0].tx_buf_add);
              EP_Write_Func_1(desc_len, (u16*)USB_DeviceDesc);
    //          EP_Write_Func();
              EP_Wait_Null();
              break;
            case CONFIGURATION_DESC:
              desc_len = (desc_len < CONFIG_DESC_SIZE) ? EP0_CONF_PACK->wLength : CONFIG_DESC_SIZE;
              EP_Write_Func_1(desc_len, (u16*)USB_ConfigDesc);
    //          EP_Write_Func();
              EP_Wait_Null();
              break;
            case STRING_LANG_DESC:
              desc_len = (desc_len < STRING_LANG_DESC_SIZE) ? EP0_CONF_PACK->wLength : STRING_LANG_DESC_SIZE;
              EP_Write_Func_1(desc_len, (u16*)USB_StringLangDesc);
              EP_Wait_Null();
              break; 
            case STRING_MANUFAC_DESC:
              desc_len = (desc_len < STRING_MANUFAC_DESC_SIZE) ? EP0_CONF_PACK->wLength : STRING_MANUFAC_DESC_SIZE;
              EP_Write_Func_1(desc_len, (u16*)USB_StringManufacDesc);
              EP_Wait_Null();
              break; 
            case STRING_PROD_DESC:
              desc_len = (desc_len < STRING_PROD_DESC_SIZE) ? EP0_CONF_PACK->wLength : STRING_PROD_DESC_SIZE;
              EP_Write_Func_1(desc_len, (u16*)USB_StringProdDesc);
              EP_Wait_Null();
              break; 
            case STRING_SN_DESC:
              desc_len = (desc_len < STRING_SN_DESC_SIZE) ? EP0_CONF_PACK->wLength : STRING_SN_DESC_SIZE;
              EP_Write_Func_1(desc_len, (u16*)USB_StringSNDesc);
              EP_Wait_Null();
              break; 

            default:
              break;
            }
          }
        default:
          break;
        }
        break;
      case 0x00:
        switch(EP0_CONF_PACK->bRequest)
        {
        case SET_ADDRESS:
          USB_SetStatus.USB_Addr = EP0_CONF_PACK->wValue;
          EP_Send_Null();
          USB->USB_DADDR.ADD = USB_SetStatus.USB_Addr;
          USB->USB_DADDR.EF = 1;
          USB_SetStatus.USB_Status = USB_ADRESSED_STATE;
          break;
        case SET_CONFIGURATION:
          USB_SetStatus.USB_Addr = EP0_CONF_PACK->wValue;
          EP_Send_Null();
          break;
        default:
          break;
        }
        break;
      case 0xa1: //GET_FEATURE
        switch(EP0_CONF_PACK->bRequest)
        {
        case 0x01:
          status = EP0_CONF_PACK->wLength;
          status = status;
          status = 0x5645;
    //      EP_Write(0, (u8*)&status, 1);
          Read_PBM8(32, (u16*)&EP_Data_1, USB_BTABLE_START+EP0_set[0].rx_buf_add);
          EP_Write_Func_1(2, (u16*)&status);
          EP_Wait_Null();
          break;
        default:
          break;
        }
        break;
      case 0x21: ////SET_FEATURE
        switch(EP0_CONF_PACK->bRequest)
        {
        case 0x09:
          status = 0x00;//0x5645;
    //      EP_Write(0, (u8*)&status, 1);
          
  //        Read_PBM8(32, (u16*)&EP_Data_1, USB_BTABLE_START);
          EP_Read_Func_1();
          //EP_Write_Func_1(1, (u16*)&status);
//          EP_Wait_Null();
          EP_Send_Null();
          Read_PBM8(32, (u16*)&EP_Data_1, USB_BTABLE_START+EP0_set[0].rx_buf_add);
          break;
        default:
          break;
        }
        break;
      case 0x81:
        switch(EP0_CONF_PACK->bRequest)
        {
        case GET_DESCRIPTOR:
          switch(EP0_CONF_PACK->wValue)
          {
          case HID_REPORT_DESC:
            desc_len = (desc_len < REPORT_DESC_SIZE) ? EP0_CONF_PACK->wLength : REPORT_DESC_SIZE;
            EP_Write_Func_1(desc_len, (u16*)USB_ReportDesc);
            EP_Wait_Null();
            break;
          }
          break;
        }
        break;
      }
    }
    else
    {
      Read_PBM8(32, (u16*)&EP_Data_1, USB_BTABLE_START+EP0_set[0].rx_buf_add);
//      EP_Read_Func_1();
    }
    
    
    status = *(u16*)&(USB->USB_EPnR[0]);
    status = SET_VALID_RX(status);
    status = SET_NAK_TX(status);
    status = CLEAR_DTOG_TX(status);
    status = CLEAR_DTOG_RX(status);
    *(u16*)&(USB->USB_EPnR[0]) = status;
    
    endpoints[0].rx_flag = 0;
    endpoints[0].tx_flag = 0;

  //  *(u16*)&(Ep_Stat) = *(u16*)&(USB->USB_EPnR[0]);
  }
}

//void USB_OUT_EP(u8 EP)//<-- (Data)
//{
//  switch(EP)
//  {
//    case 0:
//      if((USB_CTR.REP) && (CTR.STAT != BUSY))
//      {
//        switch(USB_CTR.Value&0xFF)
//        {
//          case 1: USB_GET_BUF(EP, (u8*)&CTR, USB_GET_CNT_RX(EP), OFF); USB_CTR.REP = OFF; CTR.STAT = BUSY; break;
//          case 2: USB_GET_BUF(EP, (u8*)&CTR, USB_GET_CNT_RX(EP), OFF); USB_CTR.REP = OFF; CTR.STAT = BUSY; break;
//          case 3: USB_GET_BUF(EP, (u8*)&CTR + USB_CTR.OFS, USB_GET_CNT_RX(EP), OFF); USB_CTR.OFS += USB_GET_CNT_RX(EP);
//                  if(USB_CTR.OFS > 0x123) {USB_CTR.OFS = 0; USB_CTR.REP = OFF; CTR.STAT = BUSY;} break;
//          default: break;
//        }
//      } else {USB_CTR.REP = OFF;} break;
//    default: break;
//  }
//  USB_SET_SRX(EP, USB_RX_VALID);
//}


void USB_Handler(void)
{
  u8 ep_cnt = 0;
  if (USB_ISTR_RESET)
  {
//    Write_To_EP(0,0,0 );
    USB_ISTR_RESET = 0;
    EP0_Init();
    cn_rst++;
  }
  if (USB_ISTR_CTR)
  {
    ISTR_Dir.dir = USB_ISTR_DIR;
    if(USB_ISTR_DIR)
    {
      USB_ISTR_CTR = 0;
      ep_cnt = USB->USB_ISTR.EP_ID;
      endpoints[ep_cnt].rx_cnt     =    USB_EP_WRITE[ep_cnt]->rx_buf_dat; 
      endpoints[ep_cnt].status     =    *(u16*)&(USB->USB_EPnR[ep_cnt]);
      endpoints[ep_cnt].rx_flag    =    (endpoints[ep_cnt].status&USB_EPnR_CTR_RX) ? 1 : 0;
      endpoints[ep_cnt].setup_flag =    (endpoints[ep_cnt].status&USB_EPnR_SETUP)  ? 1 : 0;
      endpoints[ep_cnt].tx_flag    =    (endpoints[ep_cnt].status&USB_EPnR_CTR_TX) ? 1 : 0;
      endpoints[ep_cnt].status     =    CLEAR_CTR_RX_TX(endpoints[ep_cnt].status);
      *(u16*)&(USB->USB_EPnR[ep_cnt]) = endpoints[ep_cnt].status;
      //Read_PBM8(8, getptr(EP_Read), USB_BTABLE_START+EP0_set[0].rx_buf_add);
  //    Read_PBM(8, read_buff, USB_BTABLE_START+EP0_set[0].rx_buf_add);
      ISTR_Dir.setup = endpoints[ep_cnt].setup_flag;
      ISTR_Dir.cn_istr_dir1++;
      if(endpoints[ep_cnt].setup_flag)
      {
        ISTR_Dir.cn_ep_setup1 ++;
      }
      else
      {
        ISTR_Dir.cn_ep_setup0 ++;
      } 
    }
    else
    {
      USB_ISTR_CTR = 0;
      ep_cnt = USB->USB_ISTR.EP_ID;
      endpoints[ep_cnt].rx_cnt     =    USB_EP_WRITE[ep_cnt]->rx_buf_dat; 
      endpoints[ep_cnt].status     =    *(u16*)&(USB->USB_EPnR[ep_cnt]);
      endpoints[ep_cnt].rx_flag    =    (endpoints[ep_cnt].status&USB_EPnR_CTR_RX) ? 1 : 0;
      endpoints[ep_cnt].setup_flag =    (endpoints[ep_cnt].status&USB_EPnR_SETUP)  ? 1 : 0;
      endpoints[ep_cnt].tx_flag    =    (endpoints[ep_cnt].status&USB_EPnR_CTR_TX) ? 1 : 0;
      endpoints[ep_cnt].status     =    CLEAR_CTR_RX_TX(endpoints[ep_cnt].status);
      *(u16*)&(USB->USB_EPnR[ep_cnt]) = endpoints[ep_cnt].status;
      ISTR_Dir.setup = endpoints[ep_cnt].setup_flag;
      ISTR_Dir.cn_istr_dir0++;
      if(endpoints[ep_cnt].setup_flag)
      {
        ISTR_Dir.cn_ep_setup1 ++;
      }
      else
      {
        ISTR_Dir.cn_ep_setup0 ++;
        Read_PBM8(32, (u16*)&EP_Data_1, USB_BTABLE_START+EP0_set[0].rx_buf_add);
      } 
    }
    if(ep_cnt)
    {
      ep_cnt = cn_rst;
    }
    cn_rst++;
  }
}
//исправить косяк с 2байтной систеой копиррования из памяти
void Read_PBM_(u8 size, u16 *buf, u16 addr)
{
  size = (size&0x01) ? (size+1)/2 : size/2;
  for(u8 cnt=0; cnt<=size; cnt++)
  {
    *buf++ = *(u8*)(USB_BTABLE_BASE + addr + cnt*2);
  }
}

void Read_PBM8(u8 size, u16 *buf, u16 addr)
{
  
//  size = (size&0x01) ? (size+1)/2 : size/2;
  addr32 = USB_BTABLE_BASE + addr + size-1;
  if(size&0x01 && addr32 && r)
  {
    *(u8*)(buf+(u8)(size>>1)) = *(u16*)(USB_BTABLE_BASE + addr + (size-1));
  }
  for(u8 cnt=0; cnt<size>>1; cnt++)
  {
    addr32 = USB_BTABLE_BASE + addr + cnt*2;
    *buf++ = *(u16*)(USB_BTABLE_BASE + addr + cnt*2);
//    *buf++ = (*(u16*)(USB_BTABLE_BASE + addr + cnt*2))>>8;
  }
  
//  size=size>>1;
//  while(size--)
//  {
//    addr32 = USB_BTABLE_BASE + addr + cnt*2;
//    *buf++ = *(u16*)(USB_BTABLE_BASE + addr + cnt*2);
////    *buf++ = (*(u16*)(USB_BTABLE_BASE + addr + cnt*2))>>8;
//  }

}