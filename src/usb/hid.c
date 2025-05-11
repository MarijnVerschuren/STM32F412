//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"
#include "usb/hid.h"


static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */


USBD_ClassTypeDef USBD_HID = {
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL,              /* EP0_TxSent */
  NULL,              /* EP0_RxReady */
  USBD_HID_DataIn,   /* DataIn */
  NULL,              /* DataOut */
  NULL,              /* SOF */
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_HID_GetHSCfgDesc,
  USBD_HID_GetFSCfgDesc,
  USBD_HID_GetOtherSpeedCfgDesc,
  USBD_HID_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB HID device FS Configuration Descriptor */
static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] =
{
  0x09,                                               /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                        /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,                            /* wTotalLength: Bytes returned */
  0x00,
  0x01,                                               /* bNumInterfaces: 1 interface */
  0x01,                                               /* bConfigurationValue: Configuration value */
  0x00,                                               /* iConfiguration: Index of string descriptor
                                                         describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xE0,                                               /* bmAttributes: Bus Powered according to user configuration */
#else
  0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                                     /* MaxPower (mA) */

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,                                               /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: Interface descriptor type */
  0x00,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x01,                                               /* bNumEndpoints */
  0x03,                                               /* bInterfaceClass: HID */
  0x01,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x02,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0,                                                  /* iInterface: Index of string descriptor */
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/

  HID_EPIN_ADDR,                                      /* bEndpointAddress: Endpoint Address (IN) */
  0x03,                                               /* bmAttributes: Interrupt endpoint */
  HID_EPIN_SIZE,                                      /* wMaxPacketSize: 4 Bytes max */
  0x00,
  HID_FS_BINTERVAL,                                   /* bInterval: Polling Interval */
  /* 34 */
};
#endif /* USE_USBD_COMPOSITE  */

/* USB HID device Configuration Descriptor */
static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ] =
{
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
};

#ifndef USE_USBD_COMPOSITE
/* USB Standard Device Descriptor */
static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */

static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] =
{
  0x05, 0x01,        /* Usage Page (Generic Desktop Ctrls)     */
  0x09, 0x02,        /* Usage (Mouse)                          */
  0xA1, 0x01,        /* Collection (Application)               */
  0x09, 0x01,        /*   Usage (Pointer)                      */
  0xA1, 0x00,        /*   Collection (Physical)                */
  0x05, 0x09,        /*     Usage Page (Button)                */
  0x19, 0x01,        /*     Usage Minimum (0x01)               */
  0x29, 0x03,        /*     Usage Maximum (0x03)               */
  0x15, 0x00,        /*     Logical Minimum (0)                */
  0x25, 0x01,        /*     Logical Maximum (1)                */
  0x95, 0x03,        /*     Report Count (3)                   */
  0x75, 0x01,        /*     Report Size (1)                    */
  0x81, 0x02,        /*     Input (Data,Var,Abs)               */
  0x95, 0x01,        /*     Report Count (1)                   */
  0x75, 0x05,        /*     Report Size (5)                    */
  0x81, 0x01,        /*     Input (Const,Array,Abs)            */
  0x05, 0x01,        /*     Usage Page (Generic Desktop Ctrls) */
  0x09, 0x30,        /*     Usage (X)                          */
  0x09, 0x31,        /*     Usage (Y)                          */
  0x09, 0x38,        /*     Usage (Wheel)                      */
  0x15, 0x81,        /*     Logical Minimum (-127)             */
  0x25, 0x7F,        /*     Logical Maximum (127)              */
  0x75, 0x08,        /*     Report Size (8)                    */
  0x95, 0x03,        /*     Report Count (3)                   */
  0x81, 0x06,        /*     Input (Data,Var,Rel)               */
  0xC0,              /*   End Collection                       */
  0x09, 0x3C,        /*   Usage (Motion Wakeup)                */
  0x05, 0xFF,        /*   Usage Page (Reserved 0xFF)           */
  0x09, 0x01,        /*   Usage (0x01)                         */
  0x15, 0x00,        /*   Logical Minimum (0)                  */
  0x25, 0x01,        /*   Logical Maximum (1)                  */
  0x75, 0x01,        /*   Report Size (1)                      */
  0x95, 0x02,        /*   Report Count (2)                     */
  0xB1, 0x22,        /*   Feature (Data,Var,Abs,NoWrp)         */
  0x75, 0x06,        /*   Report Size (6)                      */
  0x95, 0x01,        /*   Report Count (1)                     */
  0xB1, 0x01,        /*   Feature (Const,Array,Abs,NoWrp)      */
  0xC0               /* End Collection                         */
};

static uint8_t HIDInEpAdd = HID_EPIN_ADDR;

void *USBD_static_malloc(uint32_t size) {
  static uint32_t mem[(sizeof(USBD_HID_HandleTypeDef)/4)+1];/* On 32-bit boundary */
  return mem;
}

void USBD_static_free(void *p) {}

static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_HID_HandleTypeDef *hhid;

  hhid = (USBD_HID_HandleTypeDef *)USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (hhid == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)hhid;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = HID_HS_BINTERVAL;
  }
  else   /* LOW and FULL-speed endpoints */
  {
    pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = HID_FS_BINTERVAL;
  }

  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, HIDInEpAdd, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
  pdev->ep_in[HIDInEpAdd & 0xFU].is_used = 1U;

  hhid->state = USBD_HID_IDLE;

  return (uint8_t)USBD_OK;
}


static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Close HID EPs */
  (void)USBD_LL_CloseEP(pdev, HIDInEpAdd);
  pdev->ep_in[HIDInEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[HIDInEpAdd & 0xFU].bInterval = 0U;

  /* Free allocated memory */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case USBD_HID_REQ_SET_PROTOCOL:
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case USBD_HID_REQ_GET_PROTOCOL:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
          break;

        case USBD_HID_REQ_SET_IDLE:
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case USBD_HID_REQ_GET_IDLE:
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == HID_REPORT_DESC)
          {
            len = MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
            pbuf = HID_MOUSE_ReportDesc;
          }
          else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_HID_Desc;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          (void)USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

USBD_DescHeaderTypeDef *USBD_GetNextDesc(uint8_t *pbuf, uint16_t *ptr) {
  USBD_DescHeaderTypeDef *pnext = (USBD_DescHeaderTypeDef *)(void *)pbuf;

  *ptr += pnext->bLength;
  pnext = (USBD_DescHeaderTypeDef *)(void *)(pbuf + pnext->bLength);

  return (pnext);
}

void *USBD_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr)
{
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_EpDescTypeDef *pEpDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDescriptorType == USB_DESC_TYPE_ENDPOINT)
      {
        pEpDesc = (USBD_EpDescTypeDef *)(void *)pdesc;

        if (pEpDesc->bEndpointAddress == EpAddr)
        {
          break;
        }
        else
        {
          pEpDesc = NULL;
        }
      }
    }
  }

  return (void *)pEpDesc;
}

#ifdef USE_USBD_COMPOSITE
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len, uint8_t ClassId)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[ClassId];
#else
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
#endif /* USE_USBD_COMPOSITE */

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, ClassId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hhid->state == USBD_HID_IDLE)
    {
      hhid->state = USBD_HID_BUSY;
      (void)USBD_LL_Transmit(pdev, HIDInEpAdd, report, len);
    }
  }

  return (uint8_t)USBD_OK;
}

uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev)
{
  uint32_t polling_interval;

  /* HIGH-speed endpoints */
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Sets the data transfer polling interval for high speed transfers.
     Values between 1..16 are allowed. Values correspond to interval
     of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
    polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
  }
  else   /* LOW and FULL-speed endpoints */
  {
    /* Sets the data transfer polling interval for low and full
    speed transfers */
    polling_interval =  HID_FS_BINTERVAL;
  }

  return ((uint32_t)(polling_interval));
}

#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_FS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_HS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_FS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */

static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);
  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId])->state = USBD_HID_IDLE;

  return (uint8_t)USBD_OK;
}

#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_DeviceQualifierDesc);

  return USBD_HID_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */



///*!<
// * defines
// * */
//#define HID_IEP							0x01U
//#define HID_MPS							0x04U
//#define HID_FS_INTERVAL					0xAU
//
//#define HID_CONFIG_DESCRIPTOR_SIZE		34U
//#define HID_DESCRIPTOR_SIZE				9U
//#define HID_DESCRIPTOR_TYPE				0x21U
//#define HID_REPORT_DESCRIPTOR_TYPE		0x22U
//#define HID_REPORT_DESCRIPTOR_SIZE		63U
//
//
///*!<
// * types
// * */
//typedef enum {
//	HID_IDLE = 0x0U,
//	HID_BUSY = 0x1U
//} HID_state_t;
//
//typedef enum {
//	HID_SET_PROTOCOL =	0x0BU,
//	HID_GET_PROTOCOL =	0x03U,
//	HID_SET_IDLE =		0x0AU,
//	HID_GET_IDLE =		0x02U
//} HID_command_t;
//
//typedef __PACKED_STRUCT {
//	uint8_t		protocol;
//	uint8_t		idle;
//	uint8_t		alt_setting;
//	HID_state_t	state;
//} HID_handle_t;
//
//
///*!<
// * descriptors TODO: elsewhere?
// * */
// uint8_t HID_config_descriptor[HID_CONFIG_DESCRIPTOR_SIZE]  = {
//	0x09,                                               /* bLength: Configuration Descriptor size */
//	USB_CONFIG_DESCRIPTOR,                        /* bDescriptorType: Configuration */
//	HID_CONFIG_DESCRIPTOR_SIZE,                         /* wTotalLength: Bytes returned */
//	0x00,
//	0x01,                                               /* bNumInterfaces: 1 interface */
//	0x01,                                               /* bConfigurationValue: Configuration value */
//	0x00,                                               /* iConfiguration: Index of string descriptor describing the configuration */
//	0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
//	MAX_POWER,                                     /* MaxPower (mA) */
//	/************** Descriptor of Joystick Mouse interface ****************/
//	0x09,                                               /* bLength: Interface Descriptor size */
//	USB_INTERFACE_DESCRIPTOR,                            /* bDescriptorType: Interface descriptor type */
//	0x00,                                               /* bInterfaceNumber: Number of Interface */
//	0x00,                                               /* bAlternateSetting: Alternate setting */
//	0x01,                                               /* bNumEndpoints */
//	0x03,                                               /* bInterfaceClass: HID */
//	0x01,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
//	0x01,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
//	0,                                                  /* iInterface: Index of string descriptor */
//	/******************** Descriptor of Joystick Mouse HID ********************/
//	0x09,                                               /* bLength: HID Descriptor size */
//	HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
//	0x11,                                               /* bcdHID: HID Class Spec release number */
//	0x01,
//	0x00,                                               /* bCountryCode: Hardware target country */
//	0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
//	0x22,                                               /* bDescriptorType */
//	HID_REPORT_DESCRIPTOR_SIZE,                         /* wItemLength: Total length of Report descriptor */
//	0x00,
//	/******************** Descriptor of Mouse endpoint ********************/
//	0x07,                                               /* bLength: Endpoint Descriptor size */
//	USB_ENDPOINT_DESCRIPTOR,                             /* bDescriptorType:*/
//	HID_IEP | 0x80,                       		        /* bEndpointAddress: Endpoint Address (IN) */
//	0x03,                                               /* bmAttributes: Interrupt endpoint */
//	HID_MPS,                               		        /* wMaxPacketSize: 4 Bytes max */
//	0x00,
//		HID_FS_INTERVAL,                                  /* bInterval: Polling Interval */
//};
// static uint8_t HID_keyboard_report_descriptor[HID_REPORT_DESCRIPTOR_SIZE]  = {
//	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
//	0x09, 0x06,                    // USAGE (Keyboard)
//	0xa1, 0x01,                    // COLLECTION (Application)
//	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
//	0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
//	0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
//	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//	0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
//	0x75, 0x01,                    //   REPORT_SIZE (1)
//	0x95, 0x08,                    //   REPORT_COUNT (8)
//	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
//	0x95, 0x01,                    //   REPORT_COUNT (1)
//	0x75, 0x08,                    //   REPORT_SIZE (8)
//	0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
//	0x95, 0x05,                    //   REPORT_COUNT (5)
//	0x75, 0x01,                    //   REPORT_SIZE (1)
//	0x05, 0x08,                    //   USAGE_PAGE (LEDs)
//	0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
//	0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
//	0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
//	0x95, 0x01,                    //   REPORT_COUNT (1)
//	0x75, 0x03,                    //   REPORT_SIZE (3)
//	0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
//	0x95, 0x06,                    //   REPORT_COUNT (6)
//	0x75, 0x08,                    //   REPORT_SIZE (8)
//	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
//	0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
//	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
//	0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event in322222dicated))
//	0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
//	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
//	0xc0                           // END_COLLECTION
//};
// static uint8_t HID_descriptor[HID_DESCRIPTOR_SIZE]  = {
//	0x09,                                               /* bLength: HID Descriptor size */
//	HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
//	0x11,                                               /* bcdHID: HID Class Spec release number */
//	0x01,
//	0x00,                                               /* bCountryCode: Hardware target country */
//	0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
//	0x22,                                               /* bDescriptorType */
//	HID_REPORT_DESCRIPTOR_SIZE,                         /* wItemLength: Total length of Report descriptor */
//	0x00,
//};
//
//
///*!<
// * handle and class definition
// * */
//HID_handle_t HID_handle;
//static uint8_t HID_init(void* handle, uint8_t cfgidx);
//static void HID_deinit(void* handle, uint8_t cfgidx);
//static void HID_setup(void* handle, setup_header_t* req);
//static void HID_in_transfer(void* handle, uint8_t epnum);
//class_handle_t HID_class = {
//		HID_init,
//		HID_deinit,
//		HID_setup,
//		NULL,
//		NULL,
//		HID_in_transfer,
//		NULL,
//		NULL,
//		NULL,
//		NULL,
//		HID_config_descriptor,
//		HID_CONFIG_DESCRIPTOR_SIZE
//};
//
//
///*!<
// * imported functions
// * */
//extern void IN_transfer(USB_handle_t *handle, uint8_t ep_num, void* buffer, uint32_t size);
//extern void open_IEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type);
//extern void open_OEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type);
//extern void close_IEP(USB_handle_t *handle, uint8_t ep_num);
//extern void stall_IEP(USB_handle_t *handle, uint8_t ep_num);
//extern void stall_OEP(USB_handle_t *handle, uint8_t ep_num);
//extern void stall_EP(USB_handle_t* handle, uint8_t ep_num);
//
//
///*!<
// * class functions
// * */
//static uint8_t HID_init(void* handle, uint8_t config_index) {
//	(void)config_index;
//	open_IEP(handle, HID_IEP, HID_MPS, EP_TYPE_INTR);
//	((USB_handle_t*)handle)->IN_ep[HID_IEP].is_used = 1U;
//	HID_handle.state = HID_IDLE;
//	return 0;
//}
//static void HID_deinit(void* handle, uint8_t config_index) {
//	(void)config_index;
//	close_IEP(handle, HID_IEP);
//	((USB_handle_t*)handle)->IN_ep[HID_IEP].is_used = 0U;
//}
//static void HID_setup(void* handle, setup_header_t* request) {
//	uint16_t	size;
//	uint8_t*	buffer;
//	uint16_t	status = 0U;
//
//	switch (request->type) {
//	case CLASS_REQUEST :
//		switch ((HID_command_t)request->command) {
//		case HID_SET_PROTOCOL:
//			HID_handle.protocol = request->value & 0xFFU;
//			return;
//		case HID_GET_PROTOCOL:
//			((USB_handle_t*)handle)->ep0_state = EP0_DATA_IN;
//			return IN_transfer(handle, 0x00U, (uint8_t*)&HID_handle.protocol, 1U);
//		case HID_SET_IDLE:
//			HID_handle.idle = (request->value >> 8) & 0xFFU;
//			return;
//		case HID_GET_IDLE:
//			((USB_handle_t*)handle)->ep0_state = EP0_DATA_IN;
//			return IN_transfer(handle, 0x00U, (uint8_t*)&HID_handle.idle, 1U);
//		default: break;
//		} break;
//	case STANDARD_REQUEST:
//		switch (request->command) {
//		case GET_STATUS:
//			if (((USB_handle_t*)handle)->dev_state == DEV_STATE_CONFIGURED) {
//				((USB_handle_t*)handle)->ep0_state = EP0_DATA_IN;
//				return IN_transfer(handle, 0x00U, (uint8_t*)&status, 2U);
//			} break;
//		case GET_DESCRIPTOR:
//			if (((request->value >> 8) & 0xFFU) == HID_REPORT_DESCRIPTOR_TYPE) {
//				size = HID_REPORT_DESCRIPTOR_SIZE > request->length? request->length : HID_REPORT_DESCRIPTOR_SIZE;
//				buffer = HID_keyboard_report_descriptor;
//			} else if (((request->value >> 8) & 0xFFU) == HID_DESCRIPTOR_TYPE) {
//				size = HID_DESCRIPTOR_SIZE > request->length? request->length : HID_DESCRIPTOR_SIZE;
//				buffer = HID_descriptor;
//			} else { break; }
//			((USB_handle_t*)handle)->ep0_state = EP0_DATA_IN;
//			return IN_transfer(handle, 0x00U, buffer, size);
//		case GET_INTERFACE:
//			if (((USB_handle_t*)handle)->dev_state == DEV_STATE_CONFIGURED) {
//				((USB_handle_t*)handle)->ep0_state = EP0_DATA_IN;
//				return IN_transfer(handle, 0x00U, (uint8_t*)&HID_handle.alt_setting, 1U);
//			} break;
//		case SET_INTERFACE:
//			if (((USB_handle_t*)handle)->dev_state == DEV_STATE_CONFIGURED) {
//				HID_handle.alt_setting = request->value & 0xFFU;
//				return;
//			} break;
//		case CLEAR_FEATURE:	return;
//		default: break;
//		} break;
//	default: break;
//	}
//	stall_EP(handle, 0x0U);
//}
//static void HID_in_transfer(void* handle, uint8_t epnum) {
//	(void)handle; (void)epnum; HID_handle.state = HID_IDLE;
//}
//
//
///*!<
// * usage
// * */
//void send_HID_report(USB_handle_t* handle, uint8_t* report, uint16_t len) {
//	if (handle->dev_state != DEV_STATE_CONFIGURED || HID_handle.state != HID_IDLE) { return; }
//	HID_handle.state = HID_BUSY; IN_transfer(handle, HID_IEP, report, len);
//}