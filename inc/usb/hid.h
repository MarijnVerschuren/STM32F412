//
// Created by marijn on 4/2/24.
//

#ifndef STM32F_CMSIS_USB_HAL_CPY_HID_H
#define STM32F_CMSIS_USB_HAL_CPY_HID_H
#include "usb.h"


#define HID_EPIN_ADDR                              0x81U
#define HID_EPIN_SIZE                              0x04U

#define USB_HID_CONFIG_DESC_SIZ                    34U
#define USB_HID_DESC_SIZ                           9U
#define HID_MOUSE_REPORT_DESC_SIZE                 74U

#define HID_DESCRIPTOR_TYPE                        0x21U
#define HID_REPORT_DESC                            0x22U

#ifndef HID_HS_BINTERVAL
#define HID_HS_BINTERVAL                           0x07U
#endif /* HID_HS_BINTERVAL */

#ifndef HID_FS_BINTERVAL
#define HID_FS_BINTERVAL                           0x0AU
#endif /* HID_FS_BINTERVAL */

#define USBD_HID_REQ_SET_PROTOCOL                       0x0BU
#define USBD_HID_REQ_GET_PROTOCOL                       0x03U

#define USBD_HID_REQ_SET_IDLE                           0x0AU
#define USBD_HID_REQ_GET_IDLE                           0x02U

#define USBD_HID_REQ_SET_REPORT                         0x09U
#define USBD_HID_REQ_GET_REPORT                         0x01U

#define USBD_malloc         (void *)USBD_static_malloc
#define USBD_free           USBD_static_free

typedef enum{
  USBD_HID_IDLE = 0,
  USBD_HID_BUSY,
} USBD_HID_StateTypeDef;


typedef struct
{
  uint32_t Protocol;
  uint32_t IdleState;
  uint32_t AltSetting;
  USBD_HID_StateTypeDef state;
} USBD_HID_HandleTypeDef;


typedef struct
{
  uint8_t           bLength;
  uint8_t           bDescriptorType;
  uint16_t          bcdHID;
  uint8_t           bCountryCode;
  uint8_t           bNumDescriptors;
  uint8_t           bHIDDescriptorType;
  uint16_t          wItemLength;
} __PACKED USBD_HIDDescTypeDef;

extern USBD_ClassTypeDef USBD_HID;
#define USBD_HID_CLASS &USBD_HID

#ifdef USE_USBD_COMPOSITE
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len, uint8_t ClassId);
#else
uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);
#endif /* USE_USBD_COMPOSITE */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev);


//extern class_handle_t HID_class;
//
//void send_HID_report(USB_handle_t *handle, uint8_t *report, uint16_t len);


#endif // STM32F_CMSIS_USB_HAL_CPY_HID_H
