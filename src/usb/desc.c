//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"


#if (USBD_LPM_ENABLED == 1)
uint8_t * USBD_FS_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#endif /* (USBD_LPM_ENABLED == 1) */


USBD_DescriptorsTypeDef FS_Desc = {
	USBD_FS_DeviceDescriptor,
	USBD_FS_LangIDStrDescriptor,
	USBD_FS_ManufacturerStrDescriptor,
	USBD_FS_ProductStrDescriptor,
	USBD_FS_SerialStrDescriptor,
	USBD_FS_ConfigStrDescriptor,
	USBD_FS_InterfaceStrDescriptor,
#if (USBD_LPM_ENABLED == 1)
	USBD_FS_USR_BOSDescriptor
#endif /* (USBD_LPM_ENABLED == 1) */
};



uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
#if (USBD_LPM_ENABLED == 1)
  0x01,                       /*bcdUSB */ /* changed to USB version 2.01
                                             in order to support LPM L1 suspend
                                             resume test of USBCV3.0*/
#else
  0x00,                       /*bcdUSB */
#endif /* (USBD_LPM_ENABLED == 1) */
  0x02,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

/* USB_DeviceDescriptor */
/** BOS descriptor. */
#if (USBD_LPM_ENABLED == 1)
#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
__ALIGN_BEGIN uint8_t USBD_FS_BOSDesc[USB_SIZ_BOS_DESC] __ALIGN_END =
{
  0x5,
  USB_DESC_TYPE_BOS,
  0xC,
  0x0,
  0x1,  /* 1 device capability*/
        /* device capability*/
  0x7,
  USB_DEVICE_CAPABITY_TYPE,
  0x2,
  0x2,  /* LPM capability bit set*/
  0x0,
  0x0,
  0x0
};
#endif /* (USBD_LPM_ENABLED == 1) */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Variables USBD_DESC_Private_Variables
  * @brief Private variables.
  * @{
  */

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */

/** USB lang identifier descriptor. */
uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
/* Internal string descriptor. */
uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};


uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

/**
  * @brief  Return the LangID string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
  * @brief  Return the product string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the manufacturer string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
  * @brief  Return the serial number string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;

  /* Update the serial number string descriptor with the data from the unique
   * ID */
  Get_SerialNum();
  /* USER CODE BEGIN USBD_FS_SerialStrDescriptor */

  /* USER CODE END USBD_FS_SerialStrDescriptor */
  return (uint8_t *) USBD_StringSerial;
}

/**
  * @brief  Return the configuration string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the interface string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

#if (USBD_LPM_ENABLED == 1)
/**
  * @brief  Return the BOS descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_BOSDesc);
  return (uint8_t*)USBD_FS_BOSDesc;
}
#endif /* (USBD_LPM_ENABLED == 1) */

/**
  * @brief  Create the serial number string descriptor
  * @param  None
  * @retval None
  */
static void Get_SerialNum(void)
{
  uint32_t deviceserial0;
  uint32_t deviceserial1;
  uint32_t deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0U;
  uint8_t *pdesc;

  if (desc == NULL)
  {
    return;
  }

  pdesc = desc;
  *len = MIN(USBD_MAX_STR_DESC_SIZ, ((uint16_t)USBD_GetLen(pdesc) * 2U) + 2U);

  unicode[idx] = *(uint8_t *)len;
  idx++;
  unicode[idx] = USB_DESC_TYPE_STRING;
  idx++;

  while (*pdesc != (uint8_t)'\0')
  {
    unicode[idx] = *pdesc;
    pdesc++;
    idx++;

    unicode[idx] = 0U;
    idx++;
  }
}

static uint8_t USBD_GetLen(uint8_t *buf)
{
  uint8_t  len = 0U;
  uint8_t *pbuff = buf;

  while (*pbuff != (uint8_t)'\0')
  {
    len++;
    pbuff++;
  }

  return len;
}

static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++)
  {
    if (((value >> 28)) < 0xA)
    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}

///*!<
// * definitions
// * */
//#define SERIAL_STRING_DESCRIPTOR_SIZE	0x1AU
//#define DEVICE_DESCRIPTOR_SIZE			0x12U
//#define LANG_ID_STRING_DESCRIPTOR_SIZE	0x04U
//
//#define  USBD_IDX_LANGID_STR                            0x00U
//#define  USBD_IDX_MFC_STR                               0x01U
//#define  USBD_IDX_PRODUCT_STR                           0x02U
//#define  USBD_IDX_SERIAL_STR                            0x03U
//#define  USBD_IDX_CONFIG_STR                            0x04U
//#define  USBD_IDX_INTERFACE_STR                         0x05U
//
//
///*!<
// * descriptors TODO: elsewhere?
// * */
// uint8_t device_descriptor[DEVICE_DESCRIPTOR_SIZE]  = {
//	0x12,                       /*bLength */
//	USB_DEVICE_DESCRIPTOR,       /*bDescriptorType*/
//	0x00,                       /*bcdUSB */
//	0x02,
//	0x00,                       /*bDeviceClass*/
//	0x00,                       /*bDeviceSubClass*/
//	0x00,                       /*bDeviceProtocol*/
//	EP0_MPS,           /*bMaxPacketSize*/
//	0x00,        			   	/*idVendor low*/
//	0x00,         				/*idVendor hi*/
//	0x00,        				/*idProduct low*/
//	0x00,        				/*idProduct hi*/
//	0x00,                       /*bcdDevice rel. 2.00*/
//	0x02,
//	USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
//	USBD_IDX_PRODUCT_STR,       /*Index of product string*/
//	USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
//	MAX_CONFIGURATION_COUNT  /*bNumConfigurations*/
//};
// uint8_t lang_ID_descriptor[LANG_ID_STRING_DESCRIPTOR_SIZE]  = {
//	LANG_ID_STRING_DESCRIPTOR_SIZE,
//	USB_STRING_DESCRIPTOR,
//	0x09U,
//	0x04U
//};
// uint8_t manufacturer_string_descriptor[0xE]  = {
//		0xE,
//		USB_STRING_DESCRIPTOR,
//		'M', 0x00,
//		'A', 0x00,
//		'R', 0x00,
//		'I', 0x00,
//		'J', 0x00,
//		'N', 0x00,
//};
// uint8_t product_string_descriptor[0x16]  = {
//		0x16,
//		USB_STRING_DESCRIPTOR,
//		'M', 0x00,
//		'A', 0x00,
//		'R', 0x00,
//		'I', 0x00,
//		'J', 0x00,
//		'N', 0x00,
//		' ', 0x00,
//		'H', 0x00,
//		'I', 0x00,
//		'D', 0x00,
//};
// uint8_t serial_string_descriptor[SERIAL_STRING_DESCRIPTOR_SIZE]  = {
//		SERIAL_STRING_DESCRIPTOR_SIZE,
//		USB_STRING_DESCRIPTOR,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00,
//		'?', 0x00
//};
// uint8_t configuration_string_descriptor[0x16]  = {
//		0x16,
//		USB_STRING_DESCRIPTOR,
//		'H', 0x00,
//		'I', 0x00,
//		'D', 0x00,
//		' ', 0x00,
//		'C', 0x00,
//		'O', 0x00,
//		'N', 0x00,
//		'F', 0x00,
//		'I', 0x00,
//		'G', 0x00,
//};
// uint8_t interface_string_descriptor[0x1C]  = {
//		0x1C,
//		USB_STRING_DESCRIPTOR,
//		'H', 0x00,
//		'I', 0x00,
//		'D', 0x00,
//		' ', 0x00,
//		'I', 0x00,
//		'N', 0x00,
//		'T', 0x00,
//		'E', 0x00,
//		'R', 0x00,
//		'F', 0x00,
//		'A', 0x00,
//		'C', 0x00,
//		'E', 0x00,
//};
//
//
//descriptor_handle_t FS_Desc = {
//	device_descriptor,
//	DEVICE_DESCRIPTOR_SIZE,
//	lang_ID_descriptor,
//	LANG_ID_STRING_DESCRIPTOR_SIZE,
//	manufacturer_string_descriptor,
//	sizeof(manufacturer_string_descriptor),
//	product_string_descriptor,
//	sizeof(product_string_descriptor),
//	serial_string_descriptor,
//	SERIAL_STRING_DESCRIPTOR_SIZE,
//	configuration_string_descriptor,
//	sizeof(configuration_string_descriptor),
//	interface_string_descriptor,
//	sizeof(interface_string_descriptor),
//};