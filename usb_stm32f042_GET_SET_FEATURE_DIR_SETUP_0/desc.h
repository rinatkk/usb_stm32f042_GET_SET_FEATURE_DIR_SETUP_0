// desc.h - файл с массивами дескрипторов
#define DEVICE_DESCRIPTOR_SIZE_BYTE                     18
#define CONFIG_DESCRIPTOR_SIZE_BYTE                     41
#define DEVICE_QALIFIER_SIZE_BYTE                       10
#define STRING_LANG_DESCRIPTOR_SIZE_BYTE                4
#define STRING_MANUFACTURING_DESCRIPTOR_SIZE_BYTE       10
#define STRING_PRODUCT_DESCRIPTOR_SIZE_BYTE             20
#define STRING_SN_DESCRIPTOR_SIZE_BYTE                  10
#define REPORT_DESCRIPTOR_SIZE_BYTE                     23
u8 USB_DeviceDesc[] = 
{
  0x12,                 //Length: 18
  0x01,                 //DescriptorType: DeviceDescriptor
  0x10, 0x01,           //USB Ver: USB 2.0 (0x0002), USB 1.1 (0x1001), USB 1.0 (0x0001).
  0x00,                 //DeviceClass: 0
  0x00,                 //DeviceSub Class: 0
  0x00,                 //DeviceProtocol: 0
  0x40,                 //MaxPacketSize: 64
  0x45, 0x56,           //Vendor ID : 0x5645 "VE"
  0x53, 0x52,           //Product ID : 0x5253 "RS"
  0x00, 0x01,           //Device Release Number: 1.0
  0x01,                 //Index of Manufacturer String Descriptor
  0x02,                 //Index of Product String Descriptor
  0x03,                 //Index of Serial Number String Descriptor
  0x01                  //Number of Possible Configurations: 1 Configuration
};


u8 USB_ConfigDesc[] = {
  0x09,                 //Length: Configuration Descriptor size
  0x02,                 //Descriptor Type: Configuration Descriptor
  0x29, 0x00,           //Total Length: 41
  0x01,                 //Number of Interfaces: 1
  0x01,                 //Configuration Value: 1
  0x00,                 //Configuration: Index of string descriptor describing the configuration
  0xC0,                 //Attributes: Self powered D7 Reserved, set to 1. (USB 1.0 Bus Powered), D6 Self Powered, D5 Remote Wakeup, D4..0 Reserved, set to 0.
  0x32,                 //MaxPower 100 mA: Maximum Power Consumption in 2mA units
/********************************Descriptor of Custom HID interface********************************/
  0x09,                 //Length: Interface Descriptor size
  0x04,                 //DescriptorType: Interface descriptor type
  0x00,                 //InterfaceNumber: Number of Interface
  0x00,                 //AlternateSetting: Alternate setting
  0x01,                 //NumEndpoints: 1
  0x03,                 //InterfaceClass: HID 3
  0x00,                 //InterfaceSubClass : 1=BOOT, 0=No boot
  0x00,                 //InterfaceProtocol : 0=None, 1=Keyboard, 2=Mouse
  0x00,                 //Interface: Index of string descriptor
/**********************************Descriptor of Custom HID device*********************************/
  0x09,                 //Length: HID Descriptor size
  0x21,                 //DescriptorType: HID Descriptor
  0x11, 0x01,           //bcdHID: HID Class Spec release number
  0x00,                 //CountryCode: Hardware target country
  0x01,                 //NumDescriptors: Number of HID class descriptors to follow
  0x22,                 //DescriptorType
  REPORT_DESC_SIZE, 0x00,           //ItemLength: Total length of Report descriptor
/********************************Descriptor of Custom HID endpoints********************************/
  0x07,                 //Length: Endpoint Descriptor size
  0x05,                 //DescriptorType: Endpoint Descriptor
  0x80,                 //EndpointAddress: Endpoint Address (IN)
  0x00,                 //Attributes: Control endpoint
  0x40, 0x00,           //MaxPacketSize: 64 Bytes max
  0x01,                 //Interval: Polling Interval (1 ms)
  0x07,                 //Length: Endpoint Descriptor size
  0x05,	                //DescriptorType: Endpoint descriptor type
  0x00,	                //EndpointAddress: Endpoint Address (OUT)
  0x00,	                //Attributes: Control endpoint
  0x40, 0x00,           //MaxPacketSize: 64 Bytes max
  0x01,	                //Interval: Polling Interval (1 ms)
};
u8 USB_StringLangDesc[] = 
{
  STRING_LANG_DESC_SIZE,	//bLength
  0x03,	//bDescriptorType
  0x09,	//wLANGID_L
  0x0d	//wLANGID_H
};
u8 USB_StringManufacDesc[] = {
//	STRING_MANUFACTURING_DESCRIPTOR_SIZE_BYTE,		//bLength
//	0x03,											//bDescriptorType
//	'S', 0x00,										//bString...
//	'O', 0x00,
//	'B', 0x00,
//	'S', 0x00
  STRING_MANUFAC_DESC_SIZE, 3, 'M', 0, 'a', 0, 'n', 0, 'u', 0, 'f', 0
};
u8 USB_StringProdDesc[] = {
	STRING_PROD_DESC_SIZE,		//bLength
	0x03,										//bDescriptorType
	'H', 0x00,									//bString...
	'I', 0x00,
	'D', 0x00,
	' ', 0x00,
	'T', 0x00,
	'E', 0x00,
	'S', 0x00,
	'T', 0x00
//  10, 3, 50, 0, 50, 0, 50, 0, 50, 0
};
u8 USB_StringSNDesc[STRING_SN_DESC_SIZE] =
{
  STRING_SN_DESC_SIZE, 3, 'S', 0, 'N', 0, 'u', 0, 'm', 0
};

//Дескриптор репорта из книги Агурова
u8 USB_ReportDesc[REPORT_DESC_SIZE] =
{
  0x06, 0x00, 0xFF,     // Usage Page(Vendor Defined Page 1)
  0x09, 0x01,           // Usage(Vendor Usage 1)
  0xA1, 0x01,           // Collection(Logical)
  
  0x85, 0x01,           // REPORT_ID (1)
  0x09, 0x01,           // USAGE (CMD)
  0x15, 0x00,           // LOGICAL_MINIMUM (0)
  0x25, 0xFF,           // LOGICAL_MAXIMUM (255)
  0x75, 0x08,           // REPORT_SIZE (8)
  0x95, 0x03,           // REPORT_COUNT (3)
  0xB1, 0x02,           // FEATURE (Data,Var,Abs,Vol)
  
  0x85, 0x02,           // REPORT_ID (2)
  0x09, 0x01,           // USAGE (CMD)
  0x15, 0x00,           // LOGICAL_MINIMUM (0)
  0x25, 0xFF,           // LOGICAL_MAXIMUM (255)
  0x75, 0x08,           // REPORT_SIZE (8)
  0x95, 0x23,           // REPORT_COUNT (35)
  0xB1, 0x82,           // FEATURE (Data,Var,Abs,Vol)
    
  0x85, 0x03,           // REPORT_ID (3)
  0x09, 0x03,           // USAGE (Data)
  0x15, 0x00,           // LOGICAL_MINIMUM (0)
  0x25, 0xFF,           // LOGICAL_MAXIMUM (255)
  0x75, 0x08,           // REPORT_SIZE (8)
  0x96, 0x23, 0x01,     // REPORT_COUNT (291)
  0xB1, 0x82,           // FEATURE (Data,Var,Abs,Vol)
  
  0xC0                  // END_COLLECTION
};