//
// Created by marijn on 4/2/24.
//
#include "usb/usb.h"

volatile uint8_t USB_IRQ_log[256];
volatile uint8_t USB_IRQ_cnt = 0;
/*!<
 * defines
 * */
#define inline __attribute__((always_inline))


/*!<
 * functions
 * */

 HAL_StatusTypeDef USB_WritePacket(const USB_OTG_GlobalTypeDef *USBx, uint8_t *src,
                                  uint8_t ch_ep_num, uint16_t len, uint8_t dma)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint8_t *pSrc = src;
  uint32_t count32b;
  uint32_t i;

  if (dma == 0U)
  {
    count32b = ((uint32_t)len + 3U) / 4U;
    for (i = 0U; i < count32b; i++)
    {
      USBx_DFIFO((uint32_t)ch_ep_num) = __UNALIGNED_UINT32_READ(pSrc);
      pSrc++;
      pSrc++;
      pSrc++;
      pSrc++;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep, uint8_t dma)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;
  uint16_t pktcnt;

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /* Zero Length Packet? */
    if (ep->xfer_len == 0U)
    {
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
    }
    else
    {
      /* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
      */
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
      USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);

      if (epnum == 0U)
      {
        if (ep->xfer_len > ep->maxpacket)
        {
          ep->xfer_len = ep->maxpacket;
        }

        USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
      }
      else
      {
        pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
        USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (pktcnt << 19));

        if (ep->type == EP_TYPE_ISOC)
        {
          USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
          USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (pktcnt << 29));
        }
      }

      USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);
    }

    if (dma == 1U)
    {
      if ((uint32_t)ep->dma_addr != 0U)
      {
        USBx_INEP(epnum)->DIEPDMA = (uint32_t)(ep->dma_addr);
      }

      if (ep->type == EP_TYPE_ISOC)
      {
        if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
        {
          USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
        }
        else
        {
          USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }
      }

      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
    }
    else
    {
      /* EP enable, IN data in FIFO */
      USBx_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

      if (ep->type != EP_TYPE_ISOC)
      {
        /* Enable the Tx FIFO Empty Interrupt for this EP */
        if (ep->xfer_len > 0U)
        {
          USBx_DEVICE->DIEPEMPMSK |= 1UL << (ep->num & EP_ADDR_MSK);
        }
      }
      else
      {
        if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
        {
          USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
        }
        else
        {
          USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }

        (void)USB_WritePacket(USBx, ep->xfer_buff, ep->num, (uint16_t)ep->xfer_len, dma);
      }
    }
  }
  else /* OUT endpoint */
  {
    /* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
    */
    USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
    USBx_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

    if (epnum == 0U)
    {
      if (ep->xfer_len > 0U)
      {
        ep->xfer_len = ep->maxpacket;
      }

      /* Store transfer size, for EP0 this is equal to endpoint max packet size */
      ep->xfer_size = ep->maxpacket;

      USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size);
      USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
    }
    else
    {
      if (ep->xfer_len == 0U)
      {
        USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->maxpacket);
        USBx_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
      }
      else
      {
        pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
        ep->xfer_size = ep->maxpacket * pktcnt;

        USBx_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_PKTCNT & ((uint32_t)pktcnt << 19);
        USBx_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size;
      }
    }

    if (dma == 1U)
    {
      if ((uint32_t)ep->xfer_buff != 0U)
      {
        USBx_OUTEP(epnum)->DOEPDMA = (uint32_t)(ep->xfer_buff);
      }
    }

    if (ep->type == EP_TYPE_ISOC)
    {
      if ((USBx_DEVICE->DSTS & (1U << 8)) == 0U)
      {
        USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
      }
      else
      {
        USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
      }
    }
    /* EP enable */
    USBx_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
  }

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len) {
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if (hpcd->Init.dma_enable == 1U)
  {
    ep->dma_addr = (uint32_t)pBuf;
  }

  (void)USB_EPStartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);

  return HAL_OK;
}


USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

 HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if (hpcd->Init.dma_enable == 1U)
  {
    ep->dma_addr = (uint32_t)pBuf;
  }

  (void)USB_EPStartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);

  return HAL_OK;
}


USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef *pdev,
                                      uint8_t *pbuf, uint32_t len)
{
  (void)USBD_LL_PrepareReceive(pdev, 0U, pbuf, len);

  return USBD_OK;
}

 USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev){
  /* Set EP0 State */
  pdev->ep0_state = USBD_EP0_STATUS_IN;

  /* Start the transfer */
  (void)USBD_LL_Transmit(pdev, 0x00U, NULL, 0U);

  return USBD_OK;
}

 uint32_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef const *hpcd, uint8_t ep_addr)
{
  return hpcd->OUT_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}

 uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}

static inline uint16_t SWAPBYTE(uint8_t *addr)
{
  uint16_t _SwapVal;
  uint16_t _Byte1;
  uint16_t _Byte2;
  uint8_t *_pbuff = addr;

  _Byte1 = *(uint8_t *)_pbuff;
  _pbuff++;
  _Byte2 = *(uint8_t *)_pbuff;

  _SwapVal = (_Byte2 << 8) | _Byte1;

  return _SwapVal;
}

void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata)
{
  uint8_t *pbuff = pdata;

  req->bmRequest = *(uint8_t *)(pbuff);

  pbuff++;
  req->bRequest = *(uint8_t *)(pbuff);

  pbuff++;
  req->wValue = SWAPBYTE(pbuff);

  pbuff++;
  pbuff++;
  req->wIndex = SWAPBYTE(pbuff);

  pbuff++;
  pbuff++;
  req->wLength = SWAPBYTE(pbuff);
}

void USBD_CtlError(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  UNUSED(req);

  (void)USBD_LL_StallEP(pdev, 0x80U);
  (void)USBD_LL_StallEP(pdev, 0U);
}

USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev,
                                    uint8_t *pbuf, uint32_t len)
{
  /* Set EP0 State */
  pdev->ep0_state = USBD_EP0_DATA_IN;
  pdev->ep_in[0].total_length = len;

#ifdef USBD_AVOID_PACKET_SPLIT_MPS
  pdev->ep_in[0].rem_length = 0U;
#else
  pdev->ep_in[0].rem_length = len;
#endif /* USBD_AVOID_PACKET_SPLIT_MPS */

  /* Start the transfer */
  (void)USBD_LL_Transmit(pdev, 0x00U, pbuf, len);

  return USBD_OK;
}

static void USBD_GetDescriptor(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint8_t err = 0U;

  switch (req->wValue >> 8)
  {
#if ((USBD_LPM_ENABLED == 1U) || (USBD_CLASS_BOS_ENABLED == 1U))
    case USB_DESC_TYPE_BOS:
      if (pdev->pDesc->GetBOSDescriptor != NULL)
      {
        pbuf = pdev->pDesc->GetBOSDescriptor(pdev->dev_speed, &len);
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;
#endif /* (USBD_LPM_ENABLED == 1U) || (USBD_CLASS_BOS_ENABLED == 1U) */
    case USB_DESC_TYPE_DEVICE:
      pbuf = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &len);
      break;

    case USB_DESC_TYPE_CONFIGURATION:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
#ifdef USE_USBD_COMPOSITE
        if ((uint8_t)(pdev->NumClasses) > 0U)
        {
          pbuf = (uint8_t *)USBD_CMPSIT.GetHSConfigDescriptor(&len);
        }
        else
#endif /* USE_USBD_COMPOSITE */
        {
          pbuf = (uint8_t *)pdev->pClass[0]->GetHSConfigDescriptor(&len);
        }
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      else
      {
#ifdef USE_USBD_COMPOSITE
        if ((uint8_t)(pdev->NumClasses) > 0U)
        {
          pbuf = (uint8_t *)USBD_CMPSIT.GetFSConfigDescriptor(&len);
        }
        else
#endif /* USE_USBD_COMPOSITE */
        {
          pbuf = (uint8_t *)pdev->pClass[0]->GetFSConfigDescriptor(&len);
        }
        pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
      }
      break;

    case USB_DESC_TYPE_STRING:
      switch ((uint8_t)(req->wValue))
      {
        case USBD_IDX_LANGID_STR:
          if (pdev->pDesc->GetLangIDStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_MFC_STR:
          if (pdev->pDesc->GetManufacturerStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_PRODUCT_STR:
          if (pdev->pDesc->GetProductStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_SERIAL_STR:
          if (pdev->pDesc->GetSerialStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_CONFIG_STR:
          if (pdev->pDesc->GetConfigurationStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        case USBD_IDX_INTERFACE_STR:
          if (pdev->pDesc->GetInterfaceStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
          break;

        default:
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
          pbuf = NULL;

          for (uint32_t idx = 0U; (idx < pdev->NumClasses); idx++)
          {
            if (pdev->pClass[idx]->GetUsrStrDescriptor != NULL)
            {
              pdev->classId = idx;
              pbuf = pdev->pClass[idx]->GetUsrStrDescriptor(pdev, LOBYTE(req->wValue), &len);

              if (pbuf == NULL) /* This means that no class recognized the string index */
              {
                continue;
              }
              else
              {
                break;
              }
            }
          }
#endif /* USBD_SUPPORT_USER_STRING_DESC  */

#if (USBD_CLASS_USER_STRING_DESC == 1U)
          if (pdev->pDesc->GetUserStrDescriptor != NULL)
          {
            pbuf = pdev->pDesc->GetUserStrDescriptor(pdev->dev_speed, LOBYTE(req->wValue), &len);
          }
          else
          {
            USBD_CtlError(pdev, req);
            err++;
          }
#endif /* USBD_SUPPORT_USER_STRING_DESC  */

#if ((USBD_CLASS_USER_STRING_DESC == 0U) && (USBD_SUPPORT_USBD_GetDescriptorUSER_STRING_DESC == 0U))
          USBD_CtlError(pdev, req);
          err++;
#endif /* (USBD_CLASS_USER_STRING_DESC == 0U) && (USBD_SUPPORT_USER_STRING_DESC == 0U) */
          break;
      }
      break;

    case USB_DESC_TYPE_DEVICE_QUALIFIER:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
#ifdef USE_USBD_COMPOSITE
        if ((uint8_t)(pdev->NumClasses) > 0U)
        {
          pbuf = (uint8_t *)USBD_CMPSIT.GetDeviceQualifierDescriptor(&len);
        }
        else
#endif /* USE_USBD_COMPOSITE */
        {
          pbuf = (uint8_t *)pdev->pClass[0]->GetDeviceQualifierDescriptor(&len);
        }
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;

    case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
#ifdef USE_USBD_COMPOSITE
        if ((uint8_t)(pdev->NumClasses) > 0U)
        {
          pbuf = (uint8_t *)USBD_CMPSIT.GetOtherSpeedConfigDescriptor(&len);
        }
        else
#endif /* USE_USBD_COMPOSITE */
        {
          pbuf = (uint8_t *)pdev->pClass[0]->GetOtherSpeedConfigDescriptor(&len);
        }
        pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
      }
      else
      {
        USBD_CtlError(pdev, req);
        err++;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      err++;
      break;
  }

  if (err != 0U)
  {
    return;
  }

  if (req->wLength != 0U)
  {
    if (len != 0U)
    {
      len = MIN(len, req->wLength);
      (void)USBD_CtlSendData(pdev, pbuf, len);
    }
    else
    {
      USBD_CtlError(pdev, req);
    }
  }
  else
  {
    (void)USBD_CtlSendStatus(pdev);
  }
}


/**
  * @brief  USBD_SetAddress
  *         Set device address
  * @param  pdev: device instance
  * @param  req: usb request
  * @retval None
  */
static void USBD_SetAddress(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint8_t  dev_addr;

  if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U))
  {
    dev_addr = (uint8_t)(req->wValue) & 0x7FU;

    if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CtlError(pdev, req);
    }
    else
    {
      pdev->dev_address = dev_addr;
      (void)USBD_LL_SetUSBAddress(pdev, dev_addr);
      (void)USBD_CtlSendStatus(pdev);

      if (dev_addr != 0U)
      {
        pdev->dev_state = USBD_STATE_ADDRESSED;
      }
      else
      {
        pdev->dev_state = USBD_STATE_DEFAULT;
      }
    }
  }
  else
  {
    USBD_CtlError(pdev, req);
  }
}

USBD_StatusTypeDef USBD_StdDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      ret = (USBD_StatusTypeDef)pdev->pClass[pdev->classId]->Setup(pdev, req);
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_DESCRIPTOR:
          USBD_GetDescriptor(pdev, req);
          break;

        case USB_REQ_SET_ADDRESS:
          USBD_SetAddress(pdev, req);
          break;

        case USB_REQ_SET_CONFIGURATION:
          ret = USBD_SetConfig(pdev, req);
          break;

        case USB_REQ_GET_CONFIGURATION:
          USBD_GetConfig(pdev, req);
          break;

        case USB_REQ_GET_STATUS:
          USBD_GetStatus(pdev, req);
          break;

        case USB_REQ_SET_FEATURE:
          USBD_SetFeature(pdev, req);
          break;

        case USB_REQ_CLEAR_FEATURE:
          USBD_ClrFeature(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }

  return ret;
}

USBD_StatusTypeDef USBD_StdItfReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t idx;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
    case USB_REQ_TYPE_STANDARD:
      switch (pdev->dev_state)
      {
        case USBD_STATE_DEFAULT:
        case USBD_STATE_ADDRESSED:
        case USBD_STATE_CONFIGURED:

          if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES)
          {
            /* Get the class index relative to this interface */
            idx = USBD_CoreFindIF(pdev, LOBYTE(req->wIndex));
            if (((uint8_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS))
            {
              /* Call the class data out function to manage the request */
              if (pdev->pClass[idx]->Setup != NULL)
              {
                pdev->classId = idx;
                ret = (USBD_StatusTypeDef)(pdev->pClass[idx]->Setup(pdev, req));
              }
              else
              {
                /* should never reach this condition */
                ret = USBD_FAIL;
              }
            }
            else
            {
              /* No relative interface found */
              ret = USBD_FAIL;
            }

            if ((req->wLength == 0U) && (ret == USBD_OK))
            {
              (void)USBD_CtlSendStatus(pdev);
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }

  return ret;
}

USBD_StatusTypeDef USBD_StdEPReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_EndpointTypeDef *pep;
  uint8_t ep_addr;
  uint8_t idx;
  USBD_StatusTypeDef ret = USBD_OK;

  ep_addr = LOBYTE(req->wIndex);

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
      /* Get the class index relative to this endpoint */
      idx = USBD_CoreFindEP(pdev, ep_addr);
      if (((uint8_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS))
      {
        pdev->classId = idx;
        /* Call the class data out function to manage the request */
        if (pdev->pClass[idx]->Setup != NULL)
        {
          ret = (USBD_StatusTypeDef)pdev->pClass[idx]->Setup(pdev, req);
        }
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_SET_FEATURE:
          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                (void)USBD_LL_StallEP(pdev, ep_addr);
                (void)USBD_LL_StallEP(pdev, 0x80U);
              }
              else
              {
                USBD_CtlError(pdev, req);
              }
              break;

            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr != 0x00U) && (ep_addr != 0x80U) && (req->wLength == 0x00U))
                {
                  (void)USBD_LL_StallEP(pdev, ep_addr);
                }
              }
              (void)USBD_CtlSendStatus(pdev);

              break;

            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:

          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                (void)USBD_LL_StallEP(pdev, ep_addr);
                (void)USBD_LL_StallEP(pdev, 0x80U);
              }
              else
              {
                USBD_CtlError(pdev, req);
              }
              break;

            case USBD_STATE_CONFIGURED:
              if (req->wValue == USB_FEATURE_EP_HALT)
              {
                if ((ep_addr & 0x7FU) != 0x00U)
                {
                  (void)USBD_LL_ClearStallEP(pdev, ep_addr);
                }
                (void)USBD_CtlSendStatus(pdev);

                /* Get the class index relative to this interface */
                idx = USBD_CoreFindEP(pdev, ep_addr);
                if (((uint8_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS))
                {
                  pdev->classId = idx;
                  /* Call the class data out function to manage the request */
                  if (pdev->pClass[idx]->Setup != NULL)
                  {
                    ret = (USBD_StatusTypeDef)(pdev->pClass[idx]->Setup(pdev, req));
                  }
                }
              }
              break;

            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;

        case USB_REQ_GET_STATUS:
          switch (pdev->dev_state)
          {
            case USBD_STATE_ADDRESSED:
              if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
              {
                USBD_CtlError(pdev, req);
                break;
              }
              pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
                    &pdev->ep_out[ep_addr & 0x7FU];

              pep->status = 0x0000U;

              (void)USBD_CtlSendData(pdev, (uint8_t *)&pep->status, 2U);
              break;

            case USBD_STATE_CONFIGURED:
              if ((ep_addr & 0x80U) == 0x80U)
              {
                if (pdev->ep_in[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(pdev, req);
                  break;
                }
              }
              else
              {
                if (pdev->ep_out[ep_addr & 0xFU].is_used == 0U)
                {
                  USBD_CtlError(pdev, req);
                  break;
                }
              }

              pep = ((ep_addr & 0x80U) == 0x80U) ? &pdev->ep_in[ep_addr & 0x7FU] : \
                    &pdev->ep_out[ep_addr & 0x7FU];

              if ((ep_addr == 0x00U) || (ep_addr == 0x80U))
              {
                pep->status = 0x0000U;
              }
              else if (USBD_LL_IsStallEP(pdev, ep_addr) != 0U)
              {
                pep->status = 0x0001U;
              }
              else
              {
                pep->status = 0x0000U;
              }

              (void)USBD_CtlSendData(pdev, (uint8_t *)&pep->status, 2U);
              break;

            default:
              USBD_CtlError(pdev, req);
              break;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }

  return ret;
}

 USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup)
{
  USBD_StatusTypeDef ret;

  USBD_ParseSetupRequest(&pdev->request, psetup);

  pdev->ep0_state = USBD_EP0_SETUP;

  pdev->ep0_data_len = pdev->request.wLength;

  switch (pdev->request.bmRequest & 0x1FU)
  {
    case USB_REQ_RECIPIENT_DEVICE:
      ret = USBD_StdDevReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_INTERFACE:
      ret = USBD_StdItfReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_ENDPOINT:
      ret = USBD_StdEPReq(pdev, &pdev->request);
      break;

    default:
      ret = USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
      break;
  }

  return ret;
}

uint8_t USBD_CoreFindIF(USBD_HandleTypeDef *pdev, uint8_t index)
{
#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      /* Parse all interfaces listed in the current class */
      for (uint32_t j = 0U; j < pdev->tclasslist[i].NumIf; j++)
      {
        /* Check if requested Interface matches the current class interface */
        if (pdev->tclasslist[i].Ifs[j] == index)
        {
          if (pdev->pClass[i]->Setup != NULL)
          {
            return (uint8_t)i;
          }
        }
      }
    }
  }

  return 0xFFU;
#else
  UNUSED(pdev);
  UNUSED(index);

  return 0x00U;
#endif /* USE_USBD_COMPOSITE */
}

 uint8_t USBD_CoreFindEP(USBD_HandleTypeDef *pdev, uint8_t index)
{
#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      /* Parse all endpoints listed in the current class */
      for (uint32_t j = 0U; j < pdev->tclasslist[i].NumEps; j++)
      {
        /* Check if requested endpoint matches the current class endpoint */
        if (pdev->tclasslist[i].Eps[j].add == index)
        {
          if (pdev->pClass[i]->Setup != NULL)
          {
            return (uint8_t)i;
          }
        }
      }
    }
  }

  return 0xFFU;
#else
  UNUSED(pdev);
  UNUSED(index);

  return 0x00U;
#endif /* USE_USBD_COMPOSITE */
}


USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef *pdev) {
  if (pdev->dev_state == USBD_STATE_SUSPENDED){
    pdev->dev_state = pdev->dev_old_state;
  }

  return USBD_OK;
}



 void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

 USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev,
                                        uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t idx;

  if (epnum == 0U)
  {
    pep = &pdev->ep_out[0];

    if (pdev->ep0_state == USBD_EP0_DATA_OUT)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        (void)USBD_CtlContinueRx(pdev, pdata, MIN(pep->rem_length, pep->maxpacket));
      }
      else
      {
        /* Find the class ID relative to the current request */
        switch (pdev->request.bmRequest & 0x1FU)
        {
          case USB_REQ_RECIPIENT_DEVICE:
            /* Device requests must be managed by the first instantiated class
               (or duplicated by all classes for simplicity) */
            idx = 0U;
            break;

          case USB_REQ_RECIPIENT_INTERFACE:
            idx = USBD_CoreFindIF(pdev, LOBYTE(pdev->request.wIndex));
            break;

          case USB_REQ_RECIPIENT_ENDPOINT:
            idx = USBD_CoreFindEP(pdev, LOBYTE(pdev->request.wIndex));
            break;

          default:
            /* Back to the first class in case of doubt */
            idx = 0U;
            break;
        }

        if (idx < USBD_MAX_SUPPORTED_CLASS)
        {
          /* Setup the class ID and route the request to the relative class function */
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (pdev->pClass[idx]->EP0_RxReady != NULL)
            {
              pdev->classId = idx;
              pdev->pClass[idx]->EP0_RxReady(pdev);
            }
          }
        }

        (void)USBD_CtlSendStatus(pdev);
      }
    }
  }
  else
  {
    /* Get the class index relative to this interface */
    idx = USBD_CoreFindEP(pdev, (epnum & 0x7FU));

    if (((uint16_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS))
    {
      /* Call the class data out function to manage the request */
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        if (pdev->pClass[idx]->DataOut != NULL)
        {
          pdev->classId = idx;
          ret = (USBD_StatusTypeDef)pdev->pClass[idx]->DataOut(pdev, epnum);
        }
      }
      if (ret != USBD_OK)
      {
        return ret;
      }
    }
  }

  return USBD_OK;
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef *pdev,
                                            uint8_t *pbuf, uint32_t len) {
  /* Start the next transfer */
  (void)USBD_LL_Transmit(pdev, 0x00U, pbuf, len);

  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;

  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev){
  /* Set EP0 State */
  pdev->ep0_state = USBD_EP0_STATUS_OUT;

  /* Start the transfer */
  (void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);

  return USBD_OK;
}

uint32_t USBD_GetRxCount(USBD_HandleTypeDef *pdev, uint8_t ep_addr){
  return USBD_LL_GetRxDataSize(pdev, ep_addr);
}

USBD_StatusTypeDef USBD_LL_DevConnected(USBD_HandleTypeDef *pdev)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);

  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef *pdev)
{
  USBD_StatusTypeDef   ret = USBD_OK;

  /* Free Class Resources */
  pdev->dev_state = USBD_STATE_DEFAULT;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Clear configuration  and De-initialize the Class process*/
        if (pdev->pClass[i]->DeInit(pdev, (uint8_t)pdev->dev_config) != 0U)
        {
          ret = USBD_FAIL;
        }
      }
    }
  }
#else
  if (pdev->pClass[0] != NULL)
  {
    if (pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config) != 0U)
    {
      ret = USBD_FAIL;
    }
  }
#endif /* USE_USBD_COMPOSITE */

  return ret;
}


void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd){
  USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd){
  USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

uint32_t HAL_RCC_GetHCLKFreq(void) {
  return SYS_clock_frequency;
}

HAL_StatusTypeDef USB_DeactivateEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  /* Read DEPCTLn register */
  if (ep->is_in == 1U)
  {
    if ((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
    {
      USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
      USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
    }

    USBx_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
    USBx_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
    USBx_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_USBAEP |
                                   USB_OTG_DIEPCTL_MPSIZ |
                                   USB_OTG_DIEPCTL_TXFNUM |
                                   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                   USB_OTG_DIEPCTL_EPTYP);
  }
  else
  {
    if ((USBx_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
      USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
    }

    USBx_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
    USBx_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
    USBx_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_USBAEP |
                                    USB_OTG_DOEPCTL_MPSIZ |
                                    USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
                                    USB_OTG_DOEPCTL_EPTYP);
  }

  return HAL_OK;
}

HAL_StatusTypeDef USB_ActivateEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK));

    if ((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U)
    {
      USBx_INEP(epnum)->DIEPCTL |= (ep->maxpacket & USB_OTG_DIEPCTL_MPSIZ) |
                                   ((uint32_t)ep->type << 18) | (epnum << 22) |
                                   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                   USB_OTG_DIEPCTL_USBAEP;
    }
  }
  else
  {
    USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16);

    if (((USBx_OUTEP(epnum)->DOEPCTL) & USB_OTG_DOEPCTL_USBAEP) == 0U)
    {
      USBx_OUTEP(epnum)->DOEPCTL |= (ep->maxpacket & USB_OTG_DOEPCTL_MPSIZ) |
                                    ((uint32_t)ep->type << 18) |
                                    USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                                    USB_OTG_DOEPCTL_USBAEP;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr,
                                  uint16_t ep_mps, uint8_t ep_type)
{
  HAL_StatusTypeDef ret = HAL_OK;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->num = ep_addr & EP_ADDR_MSK;
  ep->maxpacket = (uint32_t)ep_mps & 0x7FFU;
  ep->type = ep_type;

  if (ep->is_in != 0U)
  {
    /* Assign a Tx FIFO */
    ep->tx_fifo_num = ep->num;
  }

  /* Set initial data PID. */
  if (ep_type == EP_TYPE_BULK)
  {
    ep->data_pid_start = 0U;
  }

  __HAL_LOCK(hpcd);
  (void)USB_ActivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return ret;
}


static USBD_StatusTypeDef USBD_SetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  static uint8_t cfgidx;

  cfgidx = (uint8_t)(req->wValue);

  if (cfgidx > USBD_MAX_NUM_CONFIGURATION)
  {
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
  }

  switch (pdev->dev_state)
  {
    case USBD_STATE_ADDRESSED:
      if (cfgidx != 0U)
      {
        pdev->dev_config = cfgidx;

        ret = USBD_SetClassConfig(pdev, cfgidx);

        if (ret != USBD_OK)
        {
          USBD_CtlError(pdev, req);
          pdev->dev_state = USBD_STATE_ADDRESSED;
        }
        else
        {
          (void)USBD_CtlSendStatus(pdev);
          pdev->dev_state = USBD_STATE_CONFIGURED;

#if (USBD_USER_REGISTER_CALLBACK == 1U)
          if (pdev->DevStateCallback != NULL)
          {
            pdev->DevStateCallback(USBD_STATE_CONFIGURED, cfgidx);
          }
#endif /* USBD_USER_REGISTER_CALLBACK */
        }
      }
      else
      {
        (void)USBD_CtlSendStatus(pdev);
      }
      break;

    case USBD_STATE_CONFIGURED:
      if (cfgidx == 0U)
      {
        pdev->dev_state = USBD_STATE_ADDRESSED;
        pdev->dev_config = cfgidx;
        (void)USBD_ClrClassConfig(pdev, cfgidx);
        (void)USBD_CtlSendStatus(pdev);
      }
      else if (cfgidx != pdev->dev_config)
      {
        /* Clear old configuration */
        (void)USBD_ClrClassConfig(pdev, (uint8_t)pdev->dev_config);

        /* set new configuration */
        pdev->dev_config = cfgidx;

        ret = USBD_SetClassConfig(pdev, cfgidx);

        if (ret != USBD_OK)
        {
          USBD_CtlError(pdev, req);
          (void)USBD_ClrClassConfig(pdev, (uint8_t)pdev->dev_config);
          pdev->dev_state = USBD_STATE_ADDRESSED;
        }
        else
        {
          (void)USBD_CtlSendStatus(pdev);
        }
      }
      else
      {
        (void)USBD_CtlSendStatus(pdev);
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      (void)USBD_ClrClassConfig(pdev, cfgidx);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

static void USBD_GetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  if (req->wLength != 1U)
  {
    USBD_CtlError(pdev, req);
  }
  else
  {
    switch (pdev->dev_state)
    {
      case USBD_STATE_DEFAULT:
      case USBD_STATE_ADDRESSED:
        pdev->dev_default_config = 0U;
        (void)USBD_CtlSendData(pdev, (uint8_t *)&pdev->dev_default_config, 1U);
        break;

      case USBD_STATE_CONFIGURED:
        (void)USBD_CtlSendData(pdev, (uint8_t *)&pdev->dev_config, 1U);
        break;

      default:
        USBD_CtlError(pdev, req);
        break;
    }
  }
}

static void USBD_GetStatus(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (pdev->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wLength != 0x2U)
      {
        USBD_CtlError(pdev, req);
        break;
      }

#if (USBD_SELF_POWERED == 1U)
      pdev->dev_config_status = USB_CONFIG_SELF_POWERED;
#else
      pdev->dev_config_status = 0U;
#endif /* USBD_SELF_POWERED */

      if (pdev->dev_remote_wakeup != 0U)
      {
        pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
      }

      (void)USBD_CtlSendData(pdev, (uint8_t *)&pdev->dev_config_status, 2U);
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }
}


static void USBD_SetFeature(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
  {
    pdev->dev_remote_wakeup = 1U;
    (void)USBD_CtlSendStatus(pdev);
  }
  else if (req->wValue == USB_FEATURE_TEST_MODE)
  {
    pdev->dev_test_mode = (uint8_t)(req->wIndex >> 8);
    (void)USBD_CtlSendStatus(pdev);
  }
  else
  {
    USBD_CtlError(pdev, req);
  }
}

HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  __HAL_LOCK(hpcd);

  if ((ep_addr & 0x80U) == 0x80U)
  {
    (void)USB_FlushTxFifo(hpcd->Instance, (uint32_t)ep_addr & EP_ADDR_MSK);
  }
  else
  {
    (void)USB_FlushRxFifo(hpcd->Instance);
  }

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
    ep->is_in = 0U;
  }

  ep->is_stall = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);

  (void)USB_EPSetStall(hpcd->Instance, ep);

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0_OutStart(hpcd->Instance, (uint8_t)hpcd->Init.dma_enable, (uint8_t *)hpcd->Setup);
  }

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
  __HAL_LOCK(hpcd);
  hpcd->USB_Address = address;
  (void)USB_SetDevAddress(hpcd->Instance, address);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

HAL_StatusTypeDef USB_EPSetStall(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    if (((USBx_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U) && (epnum != 0U))
    {
      USBx_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);
    }
    USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
  }
  else
  {
    if (((USBx_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U) && (epnum != 0U))
    {
      USBx_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);
    }
    USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
  }

  return HAL_OK;
}

HAL_StatusTypeDef USB_SetDevAddress(const USB_OTG_GlobalTypeDef *USBx, uint8_t address)
{
  uint32_t USBx_BASE = (uint32_t)USBx;

  USBx_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
  USBx_DEVICE->DCFG |= ((uint32_t)address << 4) & USB_OTG_DCFG_DAD;

  return HAL_OK;
}

HAL_StatusTypeDef USB_EPClearStall(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep)
{
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t epnum = (uint32_t)ep->num;

  if (ep->is_in == 1U)
  {
    USBx_INEP(epnum)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
    {
      USBx_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM; /* DATA0 */
    }
  }
  else
  {
    USBx_OUTEP(epnum)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
    if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK))
    {
      USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM; /* DATA0 */
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->is_stall = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_EPClearStall(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_StatusTypeDef ret = USBD_OK;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Set configuration  and Start the Class*/
        if (pdev->pClass[i]->Init(pdev, cfgidx) != 0U)
        {
          ret = USBD_FAIL;
        }
      }
    }
  }
#else
  if (pdev->pClass[0] != NULL)
  {
    /* Set configuration and Start the Class */
    ret = (USBD_StatusTypeDef)pdev->pClass[0]->Init(pdev, cfgidx);
  }
#endif /* USE_USBD_COMPOSITE */

  return ret;
}

USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_StatusTypeDef ret = USBD_OK;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Clear configuration  and De-initialize the Class process */
        if (pdev->pClass[i]->DeInit(pdev, cfgidx) != 0U)
        {
          ret = USBD_FAIL;
        }
      }
    }
  }
#else
  /* Clear configuration  and De-initialize the Class process */
  if (pdev->pClass[0]->DeInit(pdev, cfgidx) != 0U)
  {
    ret = USBD_FAIL;
  }
#endif /* USE_USBD_COMPOSITE */

  return ret;
}

static void USBD_ClrFeature(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (pdev->dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
      {
        pdev->dev_remote_wakeup = 0U;
        (void)USBD_CtlSendStatus(pdev);
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }
}

HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_DeactivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}

HAL_StatusTypeDef USB_EPStopXfer(const USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep)
{
  __IO uint32_t count = 0U;
  HAL_StatusTypeDef ret = HAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* IN endpoint */
  if (ep->is_in == 1U)
  {
    /* EP enable, IN data in FIFO */
    if (((USBx_INEP(ep->num)->DIEPCTL) & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
    {
      USBx_INEP(ep->num)->DIEPCTL |= (USB_OTG_DIEPCTL_SNAK);
      USBx_INEP(ep->num)->DIEPCTL |= (USB_OTG_DIEPCTL_EPDIS);

      do
      {
        count++;

        if (count > 10000U)
        {
          ret = HAL_ERROR;
          break;
        }
      } while (((USBx_INEP(ep->num)->DIEPCTL) & USB_OTG_DIEPCTL_EPENA) ==  USB_OTG_DIEPCTL_EPENA);
    }
  }
  else /* OUT endpoint */
  {
    if (((USBx_OUTEP(ep->num)->DOEPCTL) & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      USBx_OUTEP(ep->num)->DOEPCTL |= (USB_OTG_DOEPCTL_SNAK);
      USBx_OUTEP(ep->num)->DOEPCTL |= (USB_OTG_DOEPCTL_EPDIS);

      do
      {
        count++;

        if (count > 10000U)
        {
          ret = HAL_ERROR;
          break;
        }
      } while (((USBx_OUTEP(ep->num)->DOEPCTL) & USB_OTG_DOEPCTL_EPENA) ==  USB_OTG_DOEPCTL_EPENA);
    }
  }

  return ret;
}

HAL_StatusTypeDef HAL_PCD_EP_Abort(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  HAL_StatusTypeDef ret;
  PCD_EPTypeDef *ep;

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
  }

  /* Stop Xfer */
  ret = USB_EPStopXfer(hpcd->Instance, ep);

  return ret;
}

USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef *pdev) {
  if (pdev->dev_state != USBD_STATE_SUSPENDED)
  {
    pdev->dev_old_state = pdev->dev_state;
  }

  pdev->dev_state = USBD_STATE_SUSPENDED;

  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef *pdev,
                                    USBD_SpeedTypeDef speed)
{
  pdev->dev_speed = speed;

  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);

  return usb_status;
}


USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef *pdev)
{
  USBD_StatusTypeDef ret = USBD_OK;

  /* Upon Reset call user call back */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->ep0_state = USBD_EP0_IDLE;
  pdev->dev_config = 0U;
  pdev->dev_remote_wakeup = 0U;
  pdev->dev_test_mode = 0U;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Clear configuration  and De-initialize the Class process*/

        if (pdev->pClass[i]->DeInit != NULL)
        {
          if (pdev->pClass[i]->DeInit(pdev, (uint8_t)pdev->dev_config) != USBD_OK)
          {
            ret = USBD_FAIL;
          }
        }
      }
    }
  }
#else

  if (pdev->pClass[0] != NULL)
  {
    if (pdev->pClass[0]->DeInit != NULL)
    {
      if (pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config) != USBD_OK)
      {
        ret = USBD_FAIL;
      }
    }
  }
#endif /* USE_USBD_COMPOSITE */

  /* Open EP0 OUT */
  (void)USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_out[0x00U & 0xFU].is_used = 1U;

  pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  (void)USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_in[0x80U & 0xFU].is_used = 1U;

  pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

  return ret;
}

uint8_t USB_GetDevSpeed(const USB_OTG_GlobalTypeDef *USBx){
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint8_t speed;
  uint32_t DevEnumSpeed = USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD;

  if (DevEnumSpeed == DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ)
  {
    speed = USBD_HS_SPEED;
  }
  else if ((DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ) ||
           (DevEnumSpeed == DSTS_ENUMSPD_FS_PHY_48MHZ))
  {
    speed = USBD_FS_SPEED;
  }
  else
  {
    speed = 0xFU;
  }
  return speed;
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
   USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

  if ( hpcd->Init.speed != PCD_SPEED_FULL)
  {
    for(;;);
  }
    /* Set Speed. */
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);

  /* Reset Device. */
  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

HAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef *USBx,uint32_t hclk, uint8_t speed){
  uint32_t UsbTrd;

  /* The USBTRD is configured according to the tables below, depending on AHB frequency
  used by application. In the low AHB frequency range it is used to stretch enough the USB response
  time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access
  latency to the Data FIFO */
  if (speed == USBD_FS_SPEED)
  {
    if ((hclk >= 14200000U) && (hclk < 15000000U))
    {
      /* hclk Clock Range between 14.2-15 MHz */
      UsbTrd = 0xFU;
    }
    else if ((hclk >= 15000000U) && (hclk < 16000000U))
    {
      /* hclk Clock Range between 15-16 MHz */
      UsbTrd = 0xEU;
    }
    else if ((hclk >= 16000000U) && (hclk < 17200000U))
    {
      /* hclk Clock Range between 16-17.2 MHz */
      UsbTrd = 0xDU;
    }
    else if ((hclk >= 17200000U) && (hclk < 18500000U))
    {
      /* hclk Clock Range between 17.2-18.5 MHz */
      UsbTrd = 0xCU;
    }
    else if ((hclk >= 18500000U) && (hclk < 20000000U))
    {
      /* hclk Clock Range between 18.5-20 MHz */
      UsbTrd = 0xBU;
    }
    else if ((hclk >= 20000000U) && (hclk < 21800000U))
    {
      /* hclk Clock Range between 20-21.8 MHz */
      UsbTrd = 0xAU;
    }
    else if ((hclk >= 21800000U) && (hclk < 24000000U))
    {
      /* hclk Clock Range between 21.8-24 MHz */
      UsbTrd = 0x9U;
    }
    else if ((hclk >= 24000000U) && (hclk < 27700000U))
    {
      /* hclk Clock Range between 24-27.7 MHz */
      UsbTrd = 0x8U;
    }
    else if ((hclk >= 27700000U) && (hclk < 32000000U))
    {
      /* hclk Clock Range between 27.7-32 MHz */
      UsbTrd = 0x7U;
    }
    else /* if(hclk >= 32000000) */
    {
      /* hclk Clock Range between 32-200 MHz */
      UsbTrd = 0x6U;
    }
  }
  else if (speed == USBD_HS_SPEED)
  {
    UsbTrd = USBD_HS_TRDT_VALUE;
  }
  else
  {
    UsbTrd = USBD_DEFAULT_TRDT_VALUE;
  }

  USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
  USBx->GUSBCFG |= (uint32_t)((UsbTrd << 10) & USB_OTG_GUSBCFG_TRDT);

  return HAL_OK;
}

HAL_StatusTypeDef USB_ActivateSetup(const USB_OTG_GlobalTypeDef *USBx) {
  uint32_t USBx_BASE = (uint32_t)USBx;

  /* Set the MPS of the IN EP0 to 64 bytes */
  USBx_INEP(0U)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;

  USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;

  return HAL_OK;
}

USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_HandleTypeDef *pdev,uint8_t epnum){
  if (pdev->pClass[pdev->classId] == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass[pdev->classId]->IsoINIncomplete != NULL)
    {
      (void)pdev->pClass[pdev->classId]->IsoINIncomplete(pdev, epnum);
    }
  }

  return USBD_OK;
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef *pdev)
{
  /* The SOF event can be distributed for all classes that support it */
  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
#ifdef USE_USBD_COMPOSITE
    /* Parse the table of classes in use */
    for (uint32_t i = 0; i < USBD_MAX_SUPPORTED_CLASS; i++)
    {
      /* Check if current class is in use */
      if ((pdev->tclasslist[i].Active) == 1U)
      {
        if (pdev->pClass[i] != NULL)
        {
          if (pdev->pClass[i]->SOF != NULL)
          {
            pdev->classId = i;
            (void)pdev->pClass[i]->SOF(pdev);
          }
        }
      }
    }
#else
    if (pdev->pClass[0] != NULL)
    {
      if (pdev->pClass[0]->SOF != NULL)
      {
        (void)pdev->pClass[0]->SOF(pdev);
      }
    }
#endif /* USE_USBD_COMPOSITE */
  }

  return USBD_OK;
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd){
  USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg){
  switch (msg)
  {
  case PCD_LPM_L0_ACTIVE:
    if (hpcd->Init.low_power_enable)
    {
      sys_init();

      /* Reset SLEEPDEEP bit of Cortex System Control Register. */
      SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
    __HAL_PCD_UNGATE_PHYCLOCK(hpcd);
    USBD_LL_Resume(hpcd->pData);
    break;

  case PCD_LPM_L1_ACTIVE:
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
    USBD_LL_Suspend(hpcd->pData);

    /* Enter in STOP mode. */
    if (hpcd->Init.low_power_enable)
    {
      /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
      SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
    break;
  }
}

HAL_StatusTypeDef USB_EP0_OutStart(const USB_OTG_GlobalTypeDef *USBx, uint8_t dma, const uint8_t *psetup) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO const uint32_t *)(&USBx->CID + 0x1U);

  if (gSNPSiD > USB_OTG_CORE_ID_300A)
  {
    if ((USBx_OUTEP(0U)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      return HAL_OK;
    }
  }

  USBx_OUTEP(0U)->DOEPTSIZ = 0U;
  USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
  USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
  USBx_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;

  if (dma == 1U)
  {
    USBx_OUTEP(0U)->DOEPDMA = (uint32_t)psetup;
    /* EP enable */
    USBx_OUTEP(0U)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
  }

  return HAL_OK;
}

uint32_t USB_ReadDevAllOutEpInterrupt(const USB_OTG_GlobalTypeDef *USBx) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_DEVICE->DAINT;
  tmpreg &= USBx_DEVICE->DAINTMSK;

  return ((tmpreg & 0xffff0000U) >> 16);
}

uint32_t USB_ReadDevOutEPInterrupt(const USB_OTG_GlobalTypeDef *USBx, uint8_t epnum) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_OUTEP((uint32_t)epnum)->DOEPINT;
  tmpreg &= USBx_DEVICE->DOEPMSK;

  return tmpreg;
}


void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}


void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
  /* Inform USB library that core enters in suspend Mode. */
  USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
  __HAL_PCD_GATE_PHYCLOCK(hpcd);
  /* Enter in STOP mode. */
  /* USER CODE BEGIN 2 */
  if (hpcd->Init.low_power_enable){
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
  /* USER CODE END 2 */
}

static HAL_StatusTypeDef PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum) {
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USB_OTG_EPTypeDef *ep;
  uint32_t len;
  uint32_t len32b;
  uint32_t fifoemptymsk;

  ep = &hpcd->IN_ep[epnum];

  if (ep->xfer_count > ep->xfer_len)
  {
    return HAL_ERROR;
  }

  len = ep->xfer_len - ep->xfer_count;

  if (len > ep->maxpacket)
  {
    len = ep->maxpacket;
  }

  len32b = (len + 3U) / 4U;

  while (((USBx_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= len32b) &&
         (ep->xfer_count < ep->xfer_len) && (ep->xfer_len != 0U))
  {
    /* Write the FIFO */
    len = ep->xfer_len - ep->xfer_count;

    if (len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }
    len32b = (len + 3U) / 4U;

    (void)USB_WritePacket(USBx, ep->xfer_buff, (uint8_t)epnum, (uint16_t)len,
                          (uint8_t)hpcd->Init.dma_enable);

    ep->xfer_buff  += len;
    ep->xfer_count += len;
  }

  if (ep->xfer_len <= ep->xfer_count)
  {
    fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
    USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef PCD_EP_OutSetupPacket_int(PCD_HandleTypeDef *hpcd, uint32_t epnum){
  const USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO const uint32_t *)(&USBx->CID + 0x1U);
  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

  if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
      ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
  {
    CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
  }

  /* Inform the upper layer that a setup packet is available */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  hpcd->SetupStageCallback(hpcd);
#else
  HAL_PCD_SetupStageCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

  if ((gSNPSiD > USB_OTG_CORE_ID_300A) && (hpcd->Init.dma_enable == 1U))
  {
    (void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
  }

  return HAL_OK;
}

static HAL_StatusTypeDef PCD_EP_OutXfrComplete_int(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
  USB_OTG_EPTypeDef *ep;
  const USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO const uint32_t *)(&USBx->CID + 0x1U);
  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

  if (hpcd->Init.dma_enable == 1U)
  {
    if ((DoepintReg & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) /* Class C */
    {
      /* StupPktRcvd = 1 this is a setup packet */
      if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
          ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
      }
    }
    else if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) /* Class E */
    {
      CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
    }
    else if ((DoepintReg & (USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_OTEPSPR)) == 0U)
    {
      /* StupPktRcvd = 1 this is a setup packet */
      if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
          ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
      }
      else
      {
        ep = &hpcd->OUT_ep[epnum];

        /* out data packet received over EP */
        ep->xfer_count = ep->xfer_size - (USBx_OUTEP(epnum)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ);

        if (epnum == 0U)
        {
          if (ep->xfer_len == 0U)
          {
            /* this is ZLP, so prepare EP0 for next setup */
            (void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
          }
          else
          {
            ep->xfer_buff += ep->xfer_count;
          }
        }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
        HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
    }
    else
    {
      /* ... */
    }
  }
  else
  {
    if (gSNPSiD == USB_OTG_CORE_ID_310A)
    {
      /* StupPktRcvd = 1 this is a setup packet */
      if ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
      }
      else
      {
        if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
        {
          CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
        }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
        HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
    }
    else
    {
      if ((epnum == 0U) && (hpcd->OUT_ep[epnum].xfer_len == 0U))
      {
        /* this is ZLP, so prepare EP0 for next setup */
        (void)USB_EP0_OutStart(hpcd->Instance, 0U, (uint8_t *)hpcd->Setup);
      }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
      HAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }
  }

  return HAL_OK;
}

USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev,uint8_t epnum, uint8_t *pdata) {
  USBD_EndpointTypeDef *pep;
  USBD_StatusTypeDef ret;
  uint8_t idx;

  if (epnum == 0U)
  {
    pep = &pdev->ep_in[0];

    if (pdev->ep0_state == USBD_EP0_DATA_IN)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        (void)USBD_CtlContinueSendData(pdev, pdata, pep->rem_length);

        /* Prepare endpoint for premature end of transfer */
        (void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
      }
      else
      {
        /* last packet is MPS multiple, so send ZLP packet */
        if ((pep->maxpacket == pep->rem_length) &&
            (pep->total_length >= pep->maxpacket) &&
            (pep->total_length < pdev->ep0_data_len))
        {
          (void)USBD_CtlContinueSendData(pdev, NULL, 0U);
          pdev->ep0_data_len = 0U;

          /* Prepare endpoint for premature end of transfer */
          (void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
        }
        else
        {
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (pdev->pClass[0]->EP0_TxSent != NULL)
            {
              pdev->classId = 0U;
              pdev->pClass[0]->EP0_TxSent(pdev);
            }
          }
          (void)USBD_LL_StallEP(pdev, 0x80U);
          (void)USBD_CtlReceiveStatus(pdev);
        }
      }
    }

  }
  else
  {
    /* Get the class index relative to this interface */
    idx = USBD_CoreFindEP(pdev, ((uint8_t)epnum | 0x80U));

    if (((uint16_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS))
    {
      /* Call the class data out function to manage the request */
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        if (pdev->pClass[idx]->DataIn != NULL)
        {
          pdev->classId = idx;
          ret = (USBD_StatusTypeDef)pdev->pClass[idx]->DataIn(pdev, epnum);

          if (ret != USBD_OK)
          {
            return ret;
          }
        }
      }
    }
  }

  return USBD_OK;
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(USBD_HandleTypeDef *pdev,uint8_t epnum){
  if (pdev->pClass[pdev->classId] == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass[pdev->classId]->IsoOUTIncomplete != NULL)
    {
      (void)pdev->pClass[pdev->classId]->IsoOUTIncomplete(pdev, epnum);
    }
  }
  return USBD_OK;
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

uint32_t USB_ReadDevAllInEpInterrupt(const USB_OTG_GlobalTypeDef *USBx) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;

  tmpreg  = USBx_DEVICE->DAINT;
  tmpreg &= USBx_DEVICE->DAINTMSK;

  return ((tmpreg & 0xFFFFU));
}


uint32_t USB_ReadDevInEPInterrupt(const USB_OTG_GlobalTypeDef *USBx, uint8_t epnum) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t tmpreg;
  uint32_t msk;
  uint32_t emp;

  msk = USBx_DEVICE->DIEPMSK;
  emp = USBx_DEVICE->DIEPEMPMSK;
  msk |= ((emp >> (epnum & EP_ADDR_MSK)) & 0x1U) << 7;
  tmpreg = USBx_INEP((uint32_t)epnum)->DIEPINT & msk;

  return tmpreg;
}

void *USB_ReadPacket(const USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint8_t *pDest = dest;
  uint32_t pData;
  uint32_t i;
  uint32_t count32b = (uint32_t)len >> 2U;
  uint16_t remaining_bytes = len % 4U;

  for (i = 0U; i < count32b; i++)
  {
    __UNALIGNED_UINT32_WRITE(pDest, USBx_DFIFO(0U));
    pDest++;
    pDest++;
    pDest++;
    pDest++;
  }

  /* When Number of data is not word aligned, read the remaining byte */
  if (remaining_bytes != 0U)
  {
    i = 0U;
    __UNALIGNED_UINT32_WRITE(&pData, USBx_DFIFO(0U));

    do
    {
      *(uint8_t *)pDest = (uint8_t)(pData >> (8U * (uint8_t)(i)));
      i++;
      pDest++;
      remaining_bytes--;
    } while (remaining_bytes != 0U);
  }

  return ((void *)pDest);
}

uint32_t USB_ReadInterrupts(USB_OTG_GlobalTypeDef const *USBx){
  uint32_t tmpreg;

  tmpreg = USBx->GINTSTS;
  tmpreg &= USBx->GINTMSK;

  return tmpreg;
}

void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd) {
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USB_OTG_EPTypeDef *ep;
  uint32_t i;
  uint32_t ep_intr;
  uint32_t epint;
  uint32_t epnum;
  uint32_t fifoemptymsk;
  uint32_t RegVal;

  /* ensure that we are in device mode */
  if (USB_GetMode(hpcd->Instance) == USB_OTG_MODE_DEVICE)
  {
    /* avoid spurious interrupt */
    if (__HAL_PCD_IS_INVALID_INTERRUPT(hpcd))
    {
      return;
    }

    /* store current frame number */
    hpcd->FrameNumber = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;

    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_MMIS))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'M';
      /* incorrect mode, acknowledge the interrupt */
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_MMIS);
    }

    /* Handle RxQLevel Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_RXFLVL))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'R';
      USB_MASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);

      RegVal = USBx->GRXSTSP;

      ep = &hpcd->OUT_ep[RegVal & USB_OTG_GRXSTSP_EPNUM];

      if (((RegVal & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_DATA_UPDT)
      {
        if ((RegVal & USB_OTG_GRXSTSP_BCNT) != 0U)
        {
          (void)USB_ReadPacket(USBx, ep->xfer_buff,
                               (uint16_t)((RegVal & USB_OTG_GRXSTSP_BCNT) >> 4));

          ep->xfer_buff += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
          ep->xfer_count += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
        }
      }
      else if (((RegVal & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_SETUP_UPDT)
      {
        (void)USB_ReadPacket(USBx, (uint8_t *)hpcd->Setup, 8U);
        ep->xfer_count += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
      }
      else
      {
        /* ... */
      }

      USB_UNMASK_INTERRUPT(hpcd->Instance, USB_OTG_GINTSTS_RXFLVL);
    }

    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_OEPINT))
    {
      epnum = 0U;

      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllOutEpInterrupt(hpcd->Instance);

      while (ep_intr != 0U)
      {
        if ((ep_intr & 0x1U) != 0U)
        {
        	USB_IRQ_log[USB_IRQ_cnt++] = 0x10 + epnum;
          epint = USB_ReadDevOutEPInterrupt(hpcd->Instance, (uint8_t)epnum);

          if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);
            (void)PCD_EP_OutXfrComplete_int(hpcd, epnum);
          }

          if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
            /* Class B setup phase done for previous decoded setup */
            (void)PCD_EP_OutSetupPacket_int(hpcd, epnum);
          }

          if ((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPDIS);
          }

          /* Clear OUT Endpoint disable interrupt */
          if ((epint & USB_OTG_DOEPINT_EPDISD) == USB_OTG_DOEPINT_EPDISD)
          {
            if ((USBx->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == USB_OTG_GINTSTS_BOUTNAKEFF)
            {
              USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;
            }

            ep = &hpcd->OUT_ep[epnum];

            if (ep->is_iso_incomplete == 1U)
            {
              ep->is_iso_incomplete = 0U;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
#else
              HAL_PCD_ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
            }

            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_EPDISD);
          }

          /* Clear Status Phase Received interrupt */
          if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
          }

          /* Clear OUT NAK interrupt */
          if ((epint & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_NAK);
          }
        }
        epnum++;
        ep_intr >>= 1U;
      }
    }

    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_IEPINT))
    {
      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllInEpInterrupt(hpcd->Instance);

      epnum = 0U;

      while (ep_intr != 0U)
      {
        if ((ep_intr & 0x1U) != 0U) /* In ITR */
        {
        	USB_IRQ_log[USB_IRQ_cnt++] = 0x10 + epnum;
          epint = USB_ReadDevInEPInterrupt(hpcd->Instance, (uint8_t)epnum);

          if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC)
          {
            fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
            USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);

            if (hpcd->Init.dma_enable == 1U)
            {
              hpcd->IN_ep[epnum].xfer_buff += hpcd->IN_ep[epnum].maxpacket;

              /* this is ZLP, so prepare EP0 for next setup */
              if ((epnum == 0U) && (hpcd->IN_ep[epnum].xfer_len == 0U))
              {
                /* prepare to rx more setup packets */
                (void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
              }
            }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
            hpcd->DataInStageCallback(hpcd, (uint8_t)epnum);
#else
            HAL_PCD_DataInStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
          }
          if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TOC);
          }
          if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITTXFE);
          }
          if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_INEPNE);
          }
          if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD)
          {
            (void)USB_FlushTxFifo(USBx, epnum);

            ep = &hpcd->IN_ep[epnum];

            if (ep->is_iso_incomplete == 1U)
            {
              ep->is_iso_incomplete = 0U;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
#else
              HAL_PCD_ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
            }

            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDISD);
          }
          if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
          {
            (void)PCD_WriteEmptyTxFifo(hpcd, epnum);
          }
        }
        epnum++;
        ep_intr >>= 1U;
      }
    }

    /* Handle Resume Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'W';
      /* Clear the Remote Wake-up Signaling */
      USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

      if (hpcd->LPM_State == LPM_L1)
      {
        hpcd->LPM_State = LPM_L0;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->LPMCallback(hpcd, PCD_LPM_L0_ACTIVE);
#else
        HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      else
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->ResumeCallback(hpcd);
#else
        HAL_PCD_ResumeCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_WKUINT);
    }

    /* Handle Suspend Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 's';
      if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS)
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->SuspendCallback(hpcd);
#else
        HAL_PCD_SuspendCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_USBSUSP);
    }
#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) \
 || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) \
 || defined(STM32F423xx)
    /* Handle LPM Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'L';
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT);

      if (hpcd->LPM_State == LPM_L0)
      {
        hpcd->LPM_State = LPM_L1;
        hpcd->BESL = (hpcd->Instance->GLPMCFG & USB_OTG_GLPMCFG_BESL) >> 2U;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->LPMCallback(hpcd, PCD_LPM_L1_ACTIVE);
#else
        HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      else
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->SuspendCallback(hpcd);
#else
        HAL_PCD_SuspendCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
    }
#endif /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) ||
          defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) ||
          defined(STM32F423xx) */
    /* Handle Reset Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_USBRST))
    {

    	USB_IRQ_log[USB_IRQ_cnt++] = 'r';
      USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
      (void)USB_FlushTxFifo(hpcd->Instance, 0x10U);

      for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
      {
        USBx_INEP(i)->DIEPINT = 0xFB7FU;
        USBx_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
        USBx_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        USBx_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
      }
      USBx_DEVICE->DAINTMSK |= 0x10001U;

      if (hpcd->Init.use_dedicated_ep1 != 0U)
      {
        USBx_DEVICE->DOUTEP1MSK |= USB_OTG_DOEPMSK_STUPM |
                                   USB_OTG_DOEPMSK_XFRCM |
                                   USB_OTG_DOEPMSK_EPDM;

        USBx_DEVICE->DINEP1MSK |= USB_OTG_DIEPMSK_TOM |
                                  USB_OTG_DIEPMSK_XFRCM |
                                  USB_OTG_DIEPMSK_EPDM;
      }
      else
      {
        USBx_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |
                                USB_OTG_DOEPMSK_XFRCM |
                                USB_OTG_DOEPMSK_EPDM |
                                USB_OTG_DOEPMSK_OTEPSPRM |
                                USB_OTG_DOEPMSK_NAKM;

        USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM |
                                USB_OTG_DIEPMSK_XFRCM |
                                USB_OTG_DIEPMSK_EPDM;
      }

      /* Set Default Address to 0 */
      USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

      /* setup EP0 to receive SETUP packets */
      (void)USB_EP0_OutStart(hpcd->Instance, (uint8_t)hpcd->Init.dma_enable,
                             (uint8_t *)hpcd->Setup);

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_USBRST);
    }

    /* Handle Enumeration done Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'E';
      (void)USB_ActivateSetup(hpcd->Instance);
      hpcd->Init.speed = USB_GetDevSpeed(hpcd->Instance);

      /* Set USB Turnaround time */
      (void)USB_SetTurnaroundTime(hpcd->Instance,
                                  HAL_RCC_GetHCLKFreq(),
                                  (uint8_t)hpcd->Init.speed);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->ResetCallback(hpcd);
#else
      HAL_PCD_ResetCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_ENUMDNE);
    }

    /* Handle SOF Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_SOF))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'S';
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->SOFCallback(hpcd);
#else
      HAL_PCD_SOFCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_SOF);
    }

    /* Handle Global OUT NAK effective Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_BOUTNAKEFF))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'N';
      USBx->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;

      for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
      {
        if (hpcd->OUT_ep[epnum].is_iso_incomplete == 1U)
        {
          /* Abort current transaction and disable the EP */
          (void)HAL_PCD_EP_Abort(hpcd, (uint8_t)epnum);
        }
      }
    }

    /* Handle Incomplete ISO IN Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'I';
      for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
      {
        RegVal = USBx_INEP(epnum)->DIEPCTL;

        if ((hpcd->IN_ep[epnum].type == EP_TYPE_ISOC) &&
            ((RegVal & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA))
        {
          hpcd->IN_ep[epnum].is_iso_incomplete = 1U;

          /* Abort current transaction and disable the EP */
          (void)HAL_PCD_EP_Abort(hpcd, (uint8_t)(epnum | 0x80U));
        }
      }

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR);
    }

    /* Handle Incomplete ISO OUT Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'O';
      for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
      {
        RegVal = USBx_OUTEP(epnum)->DOEPCTL;

        if ((hpcd->OUT_ep[epnum].type == EP_TYPE_ISOC) &&
            ((RegVal & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) &&
            ((RegVal & (0x1U << 16)) == (hpcd->FrameNumber & 0x1U)))
        {
          hpcd->OUT_ep[epnum].is_iso_incomplete = 1U;

          USBx->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;

          if ((USBx->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U)
          {
            USBx_DEVICE->DCTL |= USB_OTG_DCTL_SGONAK;
            break;
          }
        }
      }

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
    }

    /* Handle Connection event Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'C';
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->ConnectCallback(hpcd);
#else
      HAL_PCD_ConnectCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_SRQINT);
    }

    /* Handle Disconnection event Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_OTGINT))
    {
    	USB_IRQ_log[USB_IRQ_cnt++] = 'D';
      RegVal = hpcd->Instance->GOTGINT;

      if ((RegVal & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET)
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DisconnectCallback(hpcd);
#else
        HAL_PCD_DisconnectCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      hpcd->Instance->GOTGINT |= RegVal;
    }
  }
}


/*!<
 * interrupts
 * */
void USB_handler(void)		{ HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); }
//void USB_WKUP_handler(void)	{ USB_wakeup_handler(&USB_handle); }



///*!<
// * external functions
// * */
//extern void flush_RX_FIFO(USB_global_t* usb);
//extern void flush_TX_FIFO(USB_global_t* usb, uint8_t ep);
//extern void flush_TX_FIFOS(USB_global_t* usb);
//
//
///*!<
// * IO
// * */
//static inline void USB_write_packet(const USB_global_t *usb, uint8_t *src, uint8_t ep_num, uint16_t len) {
//	uint32_t*	FIFO = (uint32_t*)((uint32_t)usb + (0x1000UL * (ep_num + 1U)));
//	uint32_t	word_count;
//
//	word_count = ((uint32_t)len + 3U) / 4U;
//	for (uint32_t i = 0U; i < word_count; i++) {
//		*FIFO = __UNALIGNED_UINT32_READ(src);
//		src++; src++; src++; src++;
//	}
//}
//void IN_transfer(USB_handle_t* handle, uint8_t ep_num, void* buffer, uint32_t size) {
//	ep_num &= 0xFU;
//	USB_global_t* 	usb =		handle->instance;
//	USB_device_t*	device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*		in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*	ep =		&handle->IN_ep[ep_num];
//
//	ep->buffer =	buffer;
//	ep->size =		size;
//	ep->count =		0U;
//
//	if (!ep->size) {
//		in->TSIZ &= ~(USB_OTG_TSIZ_PKTCNT | USB_OTG_TSIZ_XFRSIZ);
//		in->TSIZ |= 0x1UL << USB_OTG_TSIZ_PKTCNT_Pos;
//	} else {
//		in->TSIZ &= ~(USB_OTG_TSIZ_XFRSIZ);
//		in->TSIZ &= ~(USB_OTG_TSIZ_PKTCNT);
//		if (!ep_num) {
//			if (ep->size > ep->mps) { ep->size = ep->mps; }
//			in->TSIZ |= (USB_OTG_TSIZ_PKTCNT & (1U << 19));
//		} else {
//			in->TSIZ |= (USB_OTG_TSIZ_PKTCNT & (((ep->size + ep->mps - 1U) / ep->mps) << 19));
//		}
//		in->TSIZ |= (USB_OTG_TSIZ_XFRSIZ & ep->size);
//		if (ep->type == EP_TYPE_ISOC) {
//			in->TSIZ &= ~(USB_OTG_TSIZ_MULCNT);
//			in->TSIZ |= (USB_OTG_TSIZ_MULCNT & (1U << 29));
//		}
//	}
//
//	in->CTL |= (USB_OTG_CTL_CNAK | USB_OTG_CTL_EPENA);
//	if (ep->type != EP_TYPE_ISOC && ep->size) {
//		device->DIEPEMPMSK |= (0x01UL << ep_num);
//	}
//	else {
//		if ((device->DSTS & (1U << 8)) == 0U)	{ in->CTL |= USB_OTG_CTL_SODDFRM; }
//		else									{ in->CTL |= USB_OTG_CTL_SD0PID_SEVNFRM; }
//		USB_write_packet(usb, ep->buffer, ep_num, (uint16_t)ep->size);
//	}
//
//}
//void OUT_transfer(USB_handle_t* handle, uint8_t ep_num, void* buffer, uint32_t size) {
//	ep_num &= 0xFU;
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*			out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*		ep =		&handle->OUT_ep[ep_num];
//
//	ep->buffer =		buffer;
//	ep->size =		size;
//	ep->count =	0U;
//
//	out->TSIZ &= ~(USB_OTG_TSIZ_XFRSIZ | USB_OTG_TSIZ_PKTCNT);
//	if (!ep_num) {
//		if (ep->size) { ep->size = ep->mps; }
//		out->TSIZ |= (
//			(USB_OTG_TSIZ_XFRSIZ & ep->mps)	|
//			(USB_OTG_TSIZ_PKTCNT & (1U << 19))
//		);
//	} else {
//		if (!ep->size) {
//			out->TSIZ |= (
//				(USB_OTG_TSIZ_XFRSIZ & ep->mps)	|
//				(USB_OTG_TSIZ_PKTCNT & (1U << 19))
//			);
//		} else {
//			uint16_t packet_count = (uint16_t)((ep->size + ep->mps - 1U) / ep->mps);
//			out->TSIZ |= USB_OTG_TSIZ_PKTCNT & ((uint32_t)packet_count << 19);
//			out->TSIZ |= USB_OTG_TSIZ_XFRSIZ & ep->mps * packet_count;
//		}
//	}
//	if (ep->type == EP_TYPE_ISOC) {
//		if ((device->DSTS & (1U << 8)) == 0U)	{ out->CTL |= USB_OTG_CTL_SODDFRM; }
//		else									{ out->CTL |= USB_OTG_CTL_SD0PID_SEVNFRM; }
//	}
//	out->CTL |= (USB_OTG_CTL_CNAK | USB_OTG_CTL_EPENA);
//}
//void start_OEP0(USB_global_t* usb) {
//	USB_EP_t*	out = (void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE);
//	out->TSIZ = (
//		(USB_OTG_TSIZ_PKTCNT & (1U << 19)) |
//		(3U * 8U) |
//		USB_OTG_TSIZ_STUPCNT
//	);
//}
//
//
///*!<
// * ep init
// * */
//void open_IEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type) {
//	ep_num &= 0xF;
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*			in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*		ep =		&handle->IN_ep[ep_num];
//
//	ep->mps = ep_mps;
//	ep->type = ep_type;
//
//	device->DAINTMSK |= (0x01UL << ep_num);
//	if (in->CTL & USB_OTG_CTL_USBAEP) { return; }
//	in->CTL |= (
//		(ep->mps & USB_OTG_CTL_MPSIZ)		|
//		((uint32_t)ep->type << 18) | (ep_num << 22)	|
//		USB_OTG_CTL_SD0PID_SEVNFRM				|
//		USB_OTG_CTL_USBAEP
//	);
//}
//void open_OEP(USB_handle_t *handle, uint8_t ep_num, uint16_t ep_mps, uint8_t ep_type) {
//	ep_num &= 0xF;
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->OUT_ep[ep_num];
//
//	ep->mps = ep_mps;
//	ep->type = ep_type;
//
//	device->DAINTMSK |= (0x10UL << ep_num);
//	if (out->CTL & USB_OTG_CTL_USBAEP) { return; }
//	out->CTL |= (
//		(ep->mps & USB_OTG_CTL_MPSIZ)	|
//		((uint32_t)ep->type << 18)				|
//		USB_OTG_CTL_SD0PID_SEVNFRM			|
//		USB_OTG_CTL_USBAEP
//	);
//}
//void close_IEP(USB_handle_t *handle, uint8_t ep_num) {
//	ep_num &= 0xF;
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->IN_ep[ep_num];
//
//	if (in->CTL & USB_OTG_CTL_EPENA) { in->CTL |= (USB_OTG_CTL_SNAK | USB_OTG_CTL_EPDIS); }
//	device->DEACHMSK &= ~(0x01UL << ep_num);
//	device->DAINTMSK &= ~(0x01UL << ep_num);
//	in->CTL &= ~(
//		USB_OTG_CTL_USBAEP |
//		USB_OTG_CTL_MPSIZ |
//		USB_OTG_CTL_TXFNUM |
//		USB_OTG_CTL_SD0PID_SEVNFRM |
//		USB_OTG_CTL_EPTYP
//	);
//}
//void close_OEP(USB_handle_t *handle, uint8_t ep_num) {
//	ep_num &= 0xF;
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->OUT_ep[ep_num];
//
//	if (out->CTL & USB_OTG_CTL_EPENA) { out->CTL |= (USB_OTG_CTL_SNAK | USB_OTG_CTL_EPDIS); }
//	device->DEACHMSK &= ~(0x10UL << ep_num);
//	device->DAINTMSK &= ~(0x10UL << ep_num);
//	out->CTL &= ~(
//		USB_OTG_CTL_USBAEP |
//		USB_OTG_CTL_MPSIZ |
//		USB_OTG_CTL_SD0PID_SEVNFRM |
//		USB_OTG_CTL_EPTYP
//	);
//}
//
//
///*!<
// * stall/un-stall
// * */
//void stall_IEP(USB_handle_t* handle, uint8_t ep_num) {
//	ep_num &= 0xFU;  // TODO: needed?
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->IN_ep[ep_num];
//
//	ep->is_stall =	1U;
//
//	if (!(in->CTL & USB_OTG_CTL_EPENA) && ep_num) {
//		in->CTL &= ~(USB_OTG_CTL_EPDIS);
//	} in->CTL |= USB_OTG_CTL_STALL;
//
//	if (!ep_num) { start_OEP0(usb); }
//}
//void stall_OEP(USB_handle_t* handle, uint8_t ep_num) {
//	ep_num &= 0xFU;  // TODO: needed?
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->OUT_ep[ep_num];
//
//	ep->is_stall =	1U;
//
//	if (!(out->CTL & USB_OTG_CTL_EPENA) && ep_num) {
//		out->CTL &= ~(USB_OTG_CTL_EPDIS);
//	} out->CTL |= USB_OTG_CTL_STALL;
//
//	if (!ep_num) { start_OEP0(usb); }
//}
//void stall_EP(USB_handle_t* handle, uint8_t ep_num) {
//	stall_IEP(handle, ep_num); stall_OEP(handle, ep_num);
//}
//void unstall_IEP(USB_handle_t* handle, uint8_t ep_num) {
//	ep_num &= 0xFU;  // TODO: needed?
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->IN_ep[ep_num];
//
//	ep->is_stall = 0U;
//
//	in->CTL &= ~USB_OTG_CTL_STALL;
//	if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
//		in->CTL |= USB_OTG_CTL_SD0PID_SEVNFRM;
//	}
//}
//void unstall_OEP(USB_handle_t* handle, uint8_t ep_num) {
//	ep_num &= 0xFU;  // TODO: needed?
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->OUT_ep[ep_num];
//
//	ep->is_stall = 0U;
//
//	out->CTL &= ~USB_OTG_CTL_STALL;
//	if ((ep->type == EP_TYPE_INTR) || (ep->type == EP_TYPE_BULK)) {
//		out->CTL |= USB_OTG_CTL_SD0PID_SEVNFRM;
//	}
//}
//
//
///*!<
// * request handling
// * */
//static inline void get_device_status(USB_handle_t* handle) {
//	if (
//		handle->dev_state == DEV_STATE_SUSPENDED ||
//		handle->header.length != 0x2U
//	) { return stall_EP(handle, 0x0U); }
//	handle->dev_config_status = 0U;
//	if (handle->remote_wakeup) { handle->dev_config_status |= 0x02U; }  // enable remote wakeup
//	handle->ep0_state = EP0_DATA_IN;
//	IN_transfer(handle, 0x00U, &handle->dev_config_status, 2U);
//}
//static inline void clear_device_feature(USB_handle_t* handle) {
//	if (
//		handle->dev_state == DEV_STATE_SUSPENDED
//	) { return stall_EP(handle, 0x0U); }
//	if (handle->header.value == USB_FEATURE_REMOTE_WAKEUP) {
//		handle->remote_wakeup = 0U;
//		handle->ep0_state = EP0_STATUS_IN;
//		IN_transfer(handle, 0x00U, NULL, 0U);
//	}
//}
//static inline void set_device_feature(USB_handle_t* handle) {
//	if (handle->header.value == USB_FEATURE_REMOTE_WAKEUP) {
//		handle->remote_wakeup = 1U;
//		handle->ep0_state = EP0_STATUS_IN;
//		return IN_transfer(handle, 0x00U, NULL, 0U);
//	}
//	if (handle->header.value == USB_FEATURE_TEST_MODE) {
//		handle->ep0_state = EP0_STATUS_IN;
//		return IN_transfer(handle, 0x00U, NULL, 0U);
//	}
//	stall_EP(handle, 0x0U);
//}
//static inline void set_device_address(USB_handle_t* handle) {
//	USB_global_t* 	usb =		handle->instance;
//	USB_device_t*	device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	uint8_t					device_address;
//	if (
//		!handle->header.index &&
//		!handle->header.length &&
//		(handle->header.value < 128U) &&
//		handle->dev_state != DEV_STATE_CONFIGURED
//	) {
//		device_address = (uint8_t)(handle->header.value) & 0x7FU;
//		handle->address = device_address;
//		device->DCFG &= ~(USB_OTG_DCFG_DAD);
//		device->DCFG |= device_address << USB_OTG_DCFG_DAD_Pos;
//
//		handle->ep0_state = EP0_STATUS_IN;
//		IN_transfer(handle, 0x00U, NULL, 0U);
//		if (device_address)	{ handle->dev_state = DEV_STATE_ADDRESSED; }
//		else				{ handle->dev_state = DEV_STATE_DEFAULT; }
//		return;
//	}
//	stall_EP(handle, 0x0U);
//}
//static inline void get_device_descriptor(USB_handle_t* handle) {
//	uint16_t	size;
//	uint8_t*	buffer;
//
//	switch (handle->header.value >> 8) {
//	case USB_DEVICE_DESCRIPTOR:
//		buffer = handle->desc->device_descriptor;
//		size = handle->desc->device_descriptor_size;					break;
//	case USB_CONFIG_DESCRIPTOR:
//		buffer = handle->class->configuration_descriptor;
//		size = handle->class->configuration_descriptor_size;			break;
//	case USB_STRING_DESCRIPTOR:
//		switch (handle->header.value & 0xFFU) {
//		case USB_LANGUAGE_ID_STRING_DESCRIPTOR:
//			buffer = handle->desc->language_ID_string_descriptor;
//			size = handle->desc->language_ID_string_descriptor_size;	break;
//		case USB_MANUFACTURER_STRING_DESCRIPTOR:
//			buffer = handle->desc->manufacturer_string_descriptor;
//			size = handle->desc->manufacturer_string_descriptor_size;	break;
//		case USB_PRODUCT_STRING_DESCRIPTOR:
//			buffer = handle->desc->product_string_descriptor;
//			size = handle->desc->product_string_descriptor_size;		break;
//		case USB_SERIAL_STRING_DESCRIPTOR:
//			buffer = handle->desc->serial_string_descriptor;
//			size = handle->desc->serial_string_descriptor_size;			break;
//		case USB_CONFIG_STRING_DESCRIPTOR:
//			buffer = handle->desc->configuration_string_descriptor;
//			size = handle->desc->configuration_string_descriptor_size;	break;
//		case USB_INTERFACE_STRING_DESCRIPTOR:
//			buffer = handle->desc->interface_string_descriptor;
//			size = handle->desc->interface_string_descriptor_size;		break;
//		default:									return stall_EP(handle, 0x0U);
//		} break;
//	case USB_QUALIFIER_DESCRIPTOR:					return stall_EP(handle, 0x0U);
//	case USB_OTHER_SPEED_DESCRIPTOR:				return stall_EP(handle, 0x0U);
//	default:										return stall_EP(handle, 0x0U);
//	}
//
//	if (handle->header.length) {
//		if (size) {
//			if (size > handle->header.length) { size = handle->header.length; }
//			handle->ep0_state = EP0_DATA_IN;
//			return IN_transfer(handle, 0x00U, buffer, size);
//		} return stall_EP(handle, 0x0U);
//	}
//	handle->ep0_state = EP0_STATUS_IN;
//	IN_transfer(handle, 0x00U, NULL, 0U);
//}
//static inline void get_device_configuration(USB_handle_t* handle) {
//	if (
//		handle->header.length != 0xFFFFU ||
//		handle->dev_state == DEV_STATE_SUSPENDED
//	) { return stall_EP(handle, 0x0U); }
//
//	switch (handle->dev_state) {
//	case DEV_STATE_DEFAULT:
//	case DEV_STATE_ADDRESSED:
//		handle->dev_default_config = 0U;
//		handle->ep0_state = EP0_DATA_IN;
//		return IN_transfer(handle, 0x00U, &handle->dev_default_config, 1U);
//	case DEV_STATE_CONFIGURED:
//		handle->ep0_state = EP0_DATA_IN;
//		return IN_transfer(handle, 0x00U, &handle->dev_config, 1U);
//	default:
//		break;
//	}
//}
//static inline void set_device_configuration(USB_handle_t* handle) {
//	uint8_t config = handle->header.value & 0xFFU;
//	if (config > MAX_CONFIGURATION_COUNT) { return stall_EP(handle, 0x0U); }
//
//	switch (handle->dev_state) {
//	case DEV_STATE_ADDRESSED:
//		if (!config) {
//			handle->ep0_state = EP0_STATUS_IN;
//			return IN_transfer(handle, 0x00U, NULL, 0U);
//		}
//		handle->dev_config = config;
//		if (handle->class->init(handle, config)) {
//			handle->dev_state = DEV_STATE_ADDRESSED;
//			break;
//		}
//		handle->ep0_state = EP0_STATUS_IN;
//		handle->dev_state = DEV_STATE_CONFIGURED;
//		return IN_transfer(handle, 0x00U, NULL, 0U);
//	case DEV_STATE_CONFIGURED:
//		if (!config) {
//			handle->dev_state = DEV_STATE_ADDRESSED;
//			handle->dev_config = config;
//			handle->class->deinit(handle, config);
//			handle->ep0_state = EP0_STATUS_IN;
//			return IN_transfer(handle, 0x00U, NULL, 0U);
//		} else if (config != handle->dev_config) {
//			handle->class->deinit(handle, (uint8_t)handle->dev_config);
//			handle->dev_config = config;
//			if (handle->class->init(handle, config)){
//				handle->class->deinit(handle, (uint8_t)handle->dev_config);
//				handle->dev_state = DEV_STATE_ADDRESSED;
//				break;
//			}
//		}
//		handle->ep0_state = EP0_STATUS_IN;
//		return IN_transfer(handle, 0x00U, NULL, 0U);
//	default: handle->class->deinit(handle, config); break;
//	}
//	stall_EP(handle, 0x0U);
//}
//static inline void get_dendpoint_status(USB_handle_t* handle, uint8_t ep_num) {
//	EP_handle_t*	ep;
//
//	switch (handle->dev_state) {
//	case DEV_STATE_ADDRESSED:
//		if ((ep_num != 0x00U) && (ep_num != 0x80U)) { break; }
//		ep = ((ep_num & 0x80U) == 0x80U) ? &handle->IN_ep[ep_num & 0x7FU] : &handle->OUT_ep[ep_num & 0x7FU];
//		ep->status = 0x0000U;
//		handle->ep0_state = EP0_DATA_IN;
//		return IN_transfer(handle, 0x00U, &ep->status, 2U);
//	case DEV_STATE_CONFIGURED:
//		if (
//			(ep_num & 0x80U && !handle->IN_ep[ep_num & 0xFU].is_used) ||
//			!handle->OUT_ep[ep_num & 0xFU].is_used
//		) { break; }
//		ep = (ep_num & 0x80U) ? &handle->IN_ep[ep_num & 0x7FU] : &handle->OUT_ep[ep_num & 0x7FU];
//		ep->status = 0x0000U;
//		if ((ep_num != 0x00U) && (ep_num != 0x80U) &&
//			((ep_num & 0x80U && handle->IN_ep[ep_num & 0x7FU].is_stall) ||
//			handle->OUT_ep[ep_num & 0x7FU].is_stall)
//		) { ep->status = 0x0001U; }
//		handle->ep0_state = EP0_DATA_IN;
//		return IN_transfer(handle, 0x00U, &ep->status, 2U);
//	default: break;
//	}
//	stall_EP(handle, 0x0U);
//}
//static inline void clear_endpoint_feature(USB_handle_t* handle, uint8_t ep_num) {
//	switch (handle->dev_state) {
//	case DEV_STATE_ADDRESSED:
//		if ((ep_num != 0x00U) && (ep_num != 0x80U)) {
//			if (ep_num & 0x80)	{ stall_IEP(handle, ep_num & 0xFU); }
//			else				{ stall_OEP(handle, ep_num & 0xFU); }
//		} return stall_EP(handle, 0x0U);
//	case DEV_STATE_CONFIGURED:
//		if (handle->header.value == USB_FEATURE_EP_HALT) {
//			if (ep_num == 0x80U)		{ unstall_IEP(handle, 0U); }
//			else if (ep_num == 0x00U)	{ unstall_OEP(handle, 0U); }
//			handle->ep0_state = EP0_STATUS_IN;
//			IN_transfer(handle, 0x00U, NULL, 0U);
//			if (handle->class->setup != NULL) {
//				(void)(handle->class->setup(handle, (void*)&handle->header));
//			}
//		}
//		return;
//	default: break;
//	}
//	stall_EP(handle, 0x0U);
//}
//static inline void set_endpoin_feature(USB_handle_t* handle, uint8_t ep_num) {
//	switch (handle->dev_state) {
//	case DEV_STATE_ADDRESSED:
//		if ((ep_num != 0x00U) && (ep_num != 0x80U)) {
//			if (ep_num & 0x80)	{ stall_IEP(handle, ep_num & 0xFU); }
//			else				{ stall_OEP(handle, ep_num & 0xFU); }
//			stall_IEP(handle, 0x0U);
//		}
//		else {
//			stall_EP(handle, 0x0U);
//		}
//		return;
//	case DEV_STATE_CONFIGURED:
//		if (handle->header.value == USB_FEATURE_EP_HALT) {
//			if ((ep_num != 0x00U) && (ep_num != 0x80U) && !handle->header.length) {
//				if (ep_num & 0x80)	{ stall_IEP(handle, ep_num & 0xFU); }
//				else				{ stall_OEP(handle, ep_num & 0xFU); }
//			}
//		}
//		handle->ep0_state = EP0_STATUS_IN;
//		IN_transfer(handle, 0x00U, NULL, 0U);
//		return;
//	default: break;
//	}
//	stall_EP(handle, 0x0U);
//}
//
//static inline void device_setup_request(USB_handle_t* handle) {
//	switch (handle->header.type) {
//	case STANDARD_REQUEST:
//		switch (handle->header.command) {
//		case GET_STATUS:		return get_device_status(handle);
//		case CLEAR_FEATURE:		return clear_device_feature(handle);
//		case SET_FEATURE:		return set_device_feature(handle);
//		case SET_ADDRESS:		return set_device_address(handle);
//		case GET_DESCRIPTOR:	return get_device_descriptor(handle);
//		case SET_DESCRIPTOR:	break;
//		case GET_CONFIGURATION:	return get_device_configuration(handle);
//		case SET_CONFIGURATION:	return set_device_configuration(handle);
//		default:				break;
//		} break;
//	default: break;
//	}
//	stall_EP(handle, 0x0U);
//}
//static inline void interface_setup_request(USB_handle_t* handle) {
//	if (
//		handle->dev_state == DEV_STATE_SUSPENDED ||
//		!handle->dev_state || handle->header.type > VENDOR_REQUEST ||
//		(handle->header.index & 0xFFU) > MAX_INTERFACE_COUNT
//	) { return stall_EP(handle, 0x0U); }
//	if (handle->class->setup != NULL) {
//		(void)(handle->class->setup(handle, (void*)&handle->header));
//	}
//	if (!handle->header.length) {
//		handle->ep0_state = EP0_STATUS_IN;
//		IN_transfer(handle, 0x00U, NULL, 0U);
//	}
//}
//static inline void endpoint_setup_request(USB_handle_t* handle) {
//	uint8_t ep_num = handle->header.index & 0xFFU;
//
//	switch (handle->header.type) {
//	case STANDARD_REQUEST:
//		switch (handle->header.command) {
//		case SET_FEATURE:	return set_endpoin_feature(handle, ep_num);
//		case CLEAR_FEATURE:	return clear_endpoint_feature(handle, ep_num);
//		case GET_STATUS:	return get_dendpoint_status(handle, ep_num);
//		default: break;
//		} break;
//	case CLASS_REQUEST:
//	case VENDOR_REQUEST:
//		if (handle->class->setup) {
//			(void)handle->class->setup(handle, (void*)&handle->header);
//		}
//		return;
//	default: break;
//	}
//	stall_EP(handle, 0x0U);
//}
//
//static inline void USB_OTG_IRQ(USB_handle_t* handle, USB_global_t* usb) {
//	// TODO: redo handle and make it possible to pass usb ptr
//	uint32_t tmp = usb->GOTGINT;
//	if (tmp & USB_OTG_GOTGINT_SEDET) {
//		handle->dev_state = DEV_STATE_DEFAULT;
//		if (handle->class == NULL) { return; }
//		handle->class->deinit(handle, (uint8_t)handle->dev_config);
//	}
//	usb->GOTGINT |= tmp;
//}
//static inline void USB_SOF_IRQ(USB_handle_t* handle, USB_global_t* usb ) {
//	// TODO: redo handle and make it possible to pass usb ptr
//	if (handle->dev_state == DEV_STATE_CONFIGURED) {
//		if (handle->class == NULL)		{ return; }
//		if (handle->class->SOF == NULL)	{ return; }
//		(void)handle->class->SOF(handle);
//	}
//	usb->GINTSTS |= USB_OTG_GINTSTS_SOF;
//}
//
//static inline void USB_receive_packet_IRQ(USB_handle_t* handle) {
//	// TODO: redo handle and make it possible to pass usb ptr
//	USB_global_t*	usb = handle->instance;
//	usb->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
//	uint32_t				tmp = usb->GRXSTSP;
//	GRXSTS_t				status = *((GRXSTS_t*)&tmp);
//	EP_handle_t*			ep = &handle->OUT_ep[status.EPNUM];  // TODO: redo handle
//	volatile uint32_t*		FIFO = (void*)(((uint32_t)usb) + USB_FIFO_REL_BASE);
//
//	switch (status.PKTSTS) {
//	case 0b0010:  // DATA (UPDT) packet
//		if (!status.BCNT) { break; }
//		const uint32_t	words = status.BCNT >> 2U;
//		const uint8_t	bytes = status.BCNT & 0b11UL;
//		for (uint32_t i = 0UL; i < words; i++) {
//			__UNALIGNED_UINT32_WRITE(ep->buffer, *FIFO);
//			// 4x inc is faster than an iadd due to pipelining
//			ep->buffer++; ep->buffer++;
//			ep->buffer++; ep->buffer++;
//		} if (bytes) {
//			__UNALIGNED_UINT32_WRITE(&tmp, *FIFO);
//			for (uint8_t i = 0; i < bytes; i++) {
//				*(uint8_t*)ep->buffer++ = (uint8_t)(tmp >> (8U * i));
//			}
//		} ep->count += status.BCNT;
//		break;
//	case 0b0110:  // SETUP packet
//		__UNALIGNED_UINT32_WRITE(&handle->setup[0], *FIFO);
//		__UNALIGNED_UINT32_WRITE(&handle->setup[1], *FIFO);
//		ep->count += 8;
//		break;
//	default: break;
//	}
//
//	usb->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
//}
//static inline void USB_global_NAK_OUT_IRQ(USB_handle_t* handle) {
//	// TODO: redo handle and make it possible to pass usb ptr
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE);
//	EP_handle_t*				ep;
//
//	usb->GINTMSK &= ~USB_OTG_GINTMSK_GONAKEFFM;
//	for (uint8_t ep_num = 1U; ep_num < handle->config.dev_endpoints; ep_num++) {
//		ep = &handle->OUT_ep[ep_num];
//		if (ep->is_iso_incomplete == 1U) { continue; }
//		if (out[ep_num].CTL & USB_OTG_CTL_EPENA) {
//			out[ep_num].CTL |= (USB_OTG_CTL_SNAK | USB_OTG_CTL_EPDIS);
//			while (out[ep_num].CTL & USB_OTG_CTL_EPENA);
//		}
//	}
//}
//static inline void USB_suspend_IRQ(USB_handle_t* handle) {
//	// TODO: redo handle and make it possible to pass usb ptr
//	USB_global_t* 	usb =		handle->instance;
//	USB_device_t*	device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	_IO uint32_t*			PCGCCTL =	(void*)(((uint32_t)usb) + USB_PCGCCTL_REL_BASE);
//
//	if ((device->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS) {
//		if (handle->dev_state != DEV_STATE_SUSPENDED) {
//			handle->dev_old_state = handle->dev_state;
//		}
//		handle->dev_state = DEV_STATE_SUSPENDED;
//		*PCGCCTL |= USB_OTG_PCGCCTL_STOPCLK;
//		if (handle->config.low_power_enable) {
//			SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
//		}
//	}
//	usb->GINTSTS &= USB_OTG_GINTSTS_USBSUSP;
//}
//static inline void USB_reset_IRQ(USB_handle_t* handle) {
//	// TODO: redo handle and make it possible to pass usb ptr
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*			in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE);
//	USB_EP_t*			out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE);
//
//	device->DCTL &= ~USB_OTG_DCTL_RWUSIG;
//	flush_TX_FIFOS(handle->instance);
//	for (uint8_t i = 0U; i < handle->config.dev_endpoints; i++) {
//		in[i].INT =		0xFB7FU;
//		in[i].CTL &=	~USB_OTG_CTL_STALL;
//		out[i].INT =	0xFB7FU;
//		out[i].CTL &=	~USB_OTG_CTL_STALL;
//		out[i].CTL |=	USB_OTG_CTL_SNAK;
//	}
//	device->DAINTMSK |= 0x10001U;
//	device->DOEPMSK |= (
//		USB_OTG_DOEPMSK_STUPM		|
//		USB_OTG_DOEPMSK_XFRCM		|
//		USB_OTG_DOEPMSK_EPDM		|
//		USB_OTG_DOEPMSK_OTEPSPRM	|
//		USB_OTG_DOEPMSK_NAKM
//   );
//	device->DIEPMSK |= (
//		USB_OTG_DIEPMSK_TOM			|
//		USB_OTG_DIEPMSK_XFRCM		|
//		USB_OTG_DIEPMSK_EPDM
//	);
//
//	device->DCFG &= ~USB_OTG_DCFG_DAD;
//	start_OEP0(usb);
//	usb->GINTSTS &= USB_OTG_GINTSTS_USBRST;
//}
//static inline void USB_enumeration_done_IRQ(USB_handle_t* handle) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE);
//
//	in[0].CTL &= ~USB_OTG_CTL_MPSIZ;
//	device->DCTL |= USB_OTG_DCTL_CGINAK;
//
//	USB->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;  // reset turnaround time
//	if		(PLL_Q_clock_frequency < 15000000UL)	{ USB->GUSBCFG |= 0xFU << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 16000000UL)	{ USB->GUSBCFG |= 0xEU << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 17200000UL)	{ USB->GUSBCFG |= 0xDU << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 18500000UL)	{ USB->GUSBCFG |= 0xCU << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 20000000UL)	{ USB->GUSBCFG |= 0xBU << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 21800000UL)	{ USB->GUSBCFG |= 0xAU << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 24000000UL)	{ USB->GUSBCFG |= 0x9U << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 27700000UL)	{ USB->GUSBCFG |= 0x8U << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else if	(PLL_Q_clock_frequency < 32000000UL)	{ USB->GUSBCFG |= 0x7U << USB_OTG_GUSBCFG_TRDT_Pos; }
//	else											{ USB->GUSBCFG |= 0x6U << USB_OTG_GUSBCFG_TRDT_Pos; }
//
//	handle->dev_state = DEV_STATE_DEFAULT;
//	handle->ep0_state = EP0_IDLE;
//	handle->dev_config = 0U;
//	handle->remote_wakeup = 0U;
//	if (handle->class != NULL) {
//		if (handle->class->deinit != NULL) {
//			handle->class->deinit(handle, (uint8_t)handle->dev_config);
//		}
//	}
//
//	open_OEP(handle, 0x00U, EP0_MPS, EP_TYPE_CTRL);
//	handle->OUT_ep[0].is_used = 1U;
//	handle->OUT_ep[0].mps = EP0_MPS;
//
//	open_IEP(handle, 0x00U, EP0_MPS, EP_TYPE_CTRL);
//	handle->IN_ep[0].is_used = 1U;
//	handle->IN_ep[0].mps = EP0_MPS;
//
//	usb->GINTSTS &= USB_OTG_GINTSTS_ENUMDNE;
//}
//static inline void IEP_transfer_complete_IRQ(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*		ep;
//	uint8_t*					data = handle->IN_ep[ep_num].buffer;
//
//	device->DIEPEMPMSK &= ~(0x1UL << ep_num);
//	in->INT |= USB_OTG_INT_XFRC;
//
//	if (!ep_num) {
//		ep = &handle->IN_ep[0];
//		if (handle->ep0_state != EP0_DATA_IN) { return; }
//		if ((ep->size - ep->count) > ep->mps) {
//			IN_transfer(handle, 0x00U, data, (ep->size - ep->count));
//			OUT_transfer(handle, 0x00U, NULL, 0U);
//		} else if (
//			(ep->mps == (ep->size - ep->count)) &&
//			(ep->size >= ep->mps) &&
//			(ep->size < handle->header.length)
//		) {
//			IN_transfer(handle, 0x00U, NULL, 0U);
//			OUT_transfer(handle, 0x00U, NULL, 0U);
//		} else {
//			if (handle->dev_state == DEV_STATE_CONFIGURED && handle->class->IEP0_complete != NULL) {
//				handle->class->IEP0_complete(handle);
//			}
//			stall_IEP(handle, 0x0U);
//			handle->ep0_state = EP0_STATUS_OUT;
//			OUT_transfer(handle, 0x00U, NULL, 0U);
//		}
//		return;
//	}
//	if (handle->dev_state == DEV_STATE_CONFIGURED && handle->class->data_IN != NULL) {
//		(void)handle->class->data_IN(handle, ep_num);
//	}
//}
//static inline void IEP_disabled_IRQ(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->IN_ep[ep_num];
//
//	flush_TX_FIFO(handle->instance, ep_num);
//	if (ep->is_iso_incomplete == 1U) {
//		ep->is_iso_incomplete = 0U;
//		if (
//			handle->class &&
//			handle->class->ISO_IN_incomplete &&
//			handle->dev_state == DEV_STATE_CONFIGURED
//		) {
//			(void)handle->class->ISO_IN_incomplete(handle, ep_num);
//		}
//	}
//	in->INT |= USB_OTG_INT_EPDISD;
//}
//static inline void IEP_FIFO_empty_IRQ(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->IN_ep[ep_num];
//	uint32_t len;
//
//	while (ep->count < ep->size && ep->size) {
//		len = ep->size - ep->count;
//		if (len > ep->mps) { len = ep->mps; }
//		if ((in->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < ((len + 3U) >> 2U)) { break; }
//		USB_write_packet(usb, ep->buffer, (uint8_t)ep_num, (uint16_t)len);
//		ep->buffer  += len;
//		ep->count += len;
//	}
//	// disable interrupt if transfer done
//	if (ep->size <= ep->count) { device->DIEPEMPMSK &= ~((uint32_t)(0x1UL << ep_num)); }
//}
//static inline void OEP_transfer_complete(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->OUT_ep[ep_num];
//	out->INT |= USB_OTG_INT_XFRC;
//
//	if (!(ep_num | ep->size)) { start_OEP0(usb); }
//
//	if (!ep_num && handle->ep0_state == EP0_DATA_OUT) {
//		if ((ep->size - ep->count) > ep->mps) {
//			OUT_transfer(handle, 0x00U, ep->buffer, (ep->size - ep->count));
//			return;
//		}
//		if (handle->dev_state == DEV_STATE_CONFIGURED && handle->class->OEP0_ready) {
//			handle->class->OEP0_ready(handle);
//		}
//		handle->ep0_state = EP0_STATUS_IN;
//		IN_transfer(handle, 0x00U, NULL, 0U);
//	} else if (handle->dev_state == DEV_STATE_CONFIGURED && handle->class->data_OUT) {
//		(void)handle->class->data_OUT(handle, ep_num);
//	}
//}
//static inline void OEP_disabled(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	EP_handle_t*				ep =		&handle->OUT_ep[ep_num];
//
//	if (usb->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) { device->DCTL |= USB_OTG_DCTL_CGONAK; }
//	if (ep->is_iso_incomplete == 1U) {
//		ep->is_iso_incomplete = 0U;
//		if (
//			handle->class &&
//			handle->dev_state == DEV_STATE_CONFIGURED &&
//			handle->class->ISO_OUT_incomplete
//		) {
//			(void)handle->class->ISO_OUT_incomplete(handle, ep_num);
//		}
//	}
//	out->INT |= USB_OTG_INT_EPDISD;
//}
//static inline void OEP_setup_done(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//
//	out->INT |= USB_OTG_INT_STUP;
//
//	uint8_t* data =		(uint8_t*)handle->setup;
//	uint8_t* header =	(uint8_t*)&handle->header;
//	*header++ =	*data++;
//	*header++ =	*data++;
//	*header++ =	*data++;
//	*header++ =	*data++;
//	*header++ =	*data++;
//	*header++ =	*data++;
//	*header++ =	*data++;
//	*header =	*data;
//
//	handle->ep0_state = EP0_SETUP;
//	switch (handle->header.recipiant) {
//	case RECIPIANT_DEVICE:		return device_setup_request(handle);
//	case RECIPIANT_INTERFACE:	return interface_setup_request(handle);
//	case RECIPIANT_ENDPOINT:	return endpoint_setup_request(handle);
//	default: break;
//	}
//	if (handle->header.direction)	{ stall_IEP(handle, 0x0U); }
//	else							{ stall_OEP(handle, 0x0U); }
//}
//static inline void USB_incomplete_ISO_IN_IRQ(USB_handle_t* handle) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE);
//	EP_handle_t*				ep;
//
//	for (uint8_t ep_num = 1U; ep_num < handle->config.dev_endpoints; ep_num++) {
//		ep = &handle->IN_ep[ep_num];
//		if (ep->type != EP_TYPE_ISOC || !(in[ep_num].CTL & USB_OTG_CTL_EPENA)) { continue; }
//		ep->is_iso_incomplete = 1U;
//		if (in[ep_num].CTL & USB_OTG_CTL_EPENA) {
//			in[ep_num].CTL |= (USB_OTG_CTL_SNAK | USB_OTG_CTL_EPDIS);
//			while (in[ep_num].CTL & USB_OTG_CTL_EPENA);
//		}
//	}
//	usb->GINTSTS |= USB_OTG_GINTSTS_IISOIXFR;
//}
//static inline void USB_incomplete_ISO_OUT_IRQ(USB_handle_t* handle) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE);
//	uint32_t					tmp;
//
//	for (uint8_t ep_num = 1U; ep_num < handle->config.dev_endpoints; ep_num++) {
//		tmp = out[ep_num].CTL;
//		if ((handle->OUT_ep[ep_num].type == EP_TYPE_ISOC) && tmp & USB_OTG_CTL_EPENA &&
//			(tmp & (0x1U << 16)) == (handle->frame_number & 0x1U)) {
//			handle->OUT_ep[ep_num].is_iso_incomplete = 1U;
//			usb->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;
//			if ((usb->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U) {
//				device->DCTL |= USB_OTG_DCTL_SGONAK;
//				break;
//			}
//		}
//	}
//	usb->GINTSTS |= USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
//}
//static inline void USB_connection_IRQ(USB_handle_t* handle) {
//	USB_global_t*	usb =		handle->instance;
//	// TODO!!!
//	usb->GINTSTS |= USB_OTG_GINTSTS_SRQINT;
//}
//static inline void USB_wake_up_IRQ(USB_handle_t* handle) {
//	USB_global_t*	usb =		handle->instance;
//	USB_device_t*	device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	device->DCTL &= ~USB_OTG_DCTL_RWUSIG;
//
//	if (handle->dev_state == DEV_STATE_SUSPENDED) {
//		handle->dev_state = handle->dev_old_state;
//	}
//	usb->GINTSTS &= USB_OTG_GINTSTS_WKUINT;
//}
//
//
///*!<
// * handlers
// * */
//static inline void IEP_common_handler(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//		uint32_t					ep_int =	(in->INT & device->DIEPMSK) | ((device->DIEPEMPMSK >> ep_num) & 0x1U) << 7;
//
//	/* transfer complete interrupt */
//	if (ep_int & USB_OTG_INT_XFRC)			{ IEP_transfer_complete_IRQ(handle, ep_num); }
//	/* endpoint disabled interrupt */
//	if (ep_int & USB_OTG_INT_EPDISD)		{ IEP_disabled_IRQ(handle, ep_num); }
//	/* AHB error interrupt */
//	if (ep_int & USB_OTG_INT_AHBERR)		{ in->INT |= USB_OTG_INT_AHBERR;}
//	/* timeout condition interrupt */
//	if (ep_int & USB_OTG_INT_TOC)			{ in->INT |= USB_OTG_INT_TOC; }
//	/* IN token received when TX FIFO is empty interrupt */
//	if (ep_int & USB_OTG_INT_ITTXFE)		{ in->INT |= USB_OTG_INT_ITTXFE; }
//	/* IN token recieved with EP mismatch interrupt */
//	if (ep_int & USB_OTG_INT_INEPNM)		{ in->INT |= USB_OTG_INT_INEPNM; }
//	/* IN enpoint NAK effective interrupt */
//	if (ep_int & USB_OTG_INT_INEPNE)		{ in->INT |= USB_OTG_INT_INEPNE; }
//	/* TX FIFO empty interrupt */
//	if (ep_int & USB_OTG_INT_TXFE)			{ IEP_FIFO_empty_IRQ(handle, ep_num); }
//	/* TX FIFO underrun interrupt */
//	if (ep_int & USB_OTG_INT_TXFIFOUDRN)	{ in->INT |= USB_OTG_INT_TXFIFOUDRN; }
//	/* buffer not available interrupt */
//	if (ep_int & USB_OTG_INT_BNA)			{ in->INT |= USB_OTG_INT_BNA; }
//	/* packet dropped interrupt */
//	if (ep_int & USB_OTG_INT_PKTDRPSTS)		{ in->INT |= USB_OTG_INT_PKTDRPSTS; }
//	/* NAK interrupt */
//	if (ep_int & USB_OTG_INT_PKTDRPSTS)		{ in->INT |= USB_OTG_INT_NAK; }
//}
//static inline void OEP_common_handler(USB_handle_t* handle, uint8_t ep_num) {
//	USB_global_t* 		usb =		handle->instance;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*	out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE + (ep_num * USB_OTG_EP_REG_SIZE));
//	uint32_t					ep_int =	out->INT & device->DIEPEMPMSK;
//
//	/* transfer complete interrupt */
//	if (ep_int & USB_OTG_INT_XFRC)					{ OEP_transfer_complete(handle, ep_num); }
//	/* endpoint disabled interrupt */
//	if (ep_int & USB_OTG_INT_EPDISD)				{ OEP_disabled(handle, ep_num); }
//	/* AHB error interrupt */
//	if (ep_int & USB_OTG_INT_AHBERR)				{ out->INT |= USB_OTG_INT_AHBERR; }
//	/* SETUP phase done */
//	if (ep_int & USB_OTG_INT_STUP)					{ OEP_setup_done(handle, ep_num); }
//	/* OUT token received when endpoint disabled interrupt */
//	if (ep_int & USB_OTG_INT_OTEPDIS)				{ out->INT |= USB_OTG_INT_OTEPDIS; }
//	/* status phase received for control write */
//	if (ep_int & USB_OTG_INT_OTEPSPR)				{ out->INT |= USB_OTG_INT_OTEPSPR; }
//	/* back to back setup packet recived interrupt */
//	if (ep_int & USB_OTG_INT_B2BSTUP)				{ out->INT |= USB_OTG_INT_B2BSTUP; }
//	/* OUT packet error interrupt */
//	if (ep_int & USB_OTG_INT_OUTPKTERR)				{ out->INT |= USB_OTG_INT_OUTPKTERR; }
//	/* buffer not available interrupt */
//	if (ep_int & USB_OTG_INT_BNA)					{ out->INT |= USB_OTG_INT_BNA; }
//	/* NAK interrupt */
//	if (ep_int & USB_OTG_INT_NAK)					{ out->INT |= USB_OTG_INT_NAK; }
//	/* NYET interrupt */
//	if (ep_int & USB_OTG_INT_NYET)					{ out->INT |= USB_OTG_INT_NYET;	}
//	/* setup packet received interrupt */
//	if (ep_int & USB_OTG_INT_STPKTRX)				{ out->INT |= USB_OTG_INT_STPKTRX;	}
//}
//
//
//volatile uint8_t USB_IRQ_log[256];
//volatile uint8_t USB_IRQ_cnt = 0;
//static inline void USB_common_handler(USB_handle_t* handle) {
//	USB_global_t*	usb = handle->instance;
//	if ((usb->GINTSTS) & 0x1U) { return; }
//	const uint32_t			irqs = usb->GINTSTS & usb->GINTMSK;
//	if (!irqs)				{ return; }
//	USB_device_t*	device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	uint8_t					ep_num;
//	uint16_t				ep_gint;
//
//	/* store current frame number */
//	handle->frame_number = (device->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos;
//
//	/* mode mismatch interrupt */
//	if (irqs & USB_OTG_GINTSTS_MMIS)					{ usb->GINTSTS |= USB_OTG_GINTSTS_MMIS;	USB_IRQ_log[USB_IRQ_cnt++] = 'M'; }
//	/* OTG interrupt */
//	if (irqs & USB_OTG_GINTSTS_OTGINT)					{ USB_OTG_IRQ(handle, usb); 			USB_IRQ_log[USB_IRQ_cnt++] = 'O'; }
//	/* start of frame interrupt */
//	if (irqs & USB_OTG_GINTSTS_SOF)						{ USB_SOF_IRQ(handle, usb); 			USB_IRQ_log[USB_IRQ_cnt++] = 'S'; }
//	/* receive packet interrupt */
//	if (irqs & USB_OTG_GINTSTS_RXFLVL)					{ USB_IRQ_log[USB_IRQ_cnt++] = 'R'; USB_receive_packet_IRQ(handle); }
//	/* global OUT NAK effective interrupt */
//	if (irqs & USB_OTG_GINTSTS_BOUTNAKEFF)				{ USB_global_NAK_OUT_IRQ(handle);		USB_IRQ_log[USB_IRQ_cnt++] = 'N'; }
//	/* suspend interrupt */
//	if (irqs & USB_OTG_GINTSTS_USBSUSP)					{ USB_suspend_IRQ(handle);				USB_IRQ_log[USB_IRQ_cnt++] = 's'; }
//	/* reset interrupt */
//	if (irqs & USB_OTG_GINTSTS_USBRST)					{ USB_reset_IRQ(handle); 				USB_IRQ_log[USB_IRQ_cnt++] = 'r'; }
//	/* enumeration done interrupt */
//	if (irqs & USB_OTG_GINTSTS_ENUMDNE)					{ USB_enumeration_done_IRQ(handle);		USB_IRQ_log[USB_IRQ_cnt++] = 'E'; }
//	/* IN endpoint interrupts */
//	if (irqs & USB_OTG_GINTSTS_IEPINT) {
//		ep_gint = device->DAINT & device->DAINTMSK & 0xFFFFU;
//		ep_num = 0U;
//		while (ep_gint) {
//			if (ep_gint & 0b1UL)						{ IEP_common_handler(handle, ep_num);	USB_IRQ_log[USB_IRQ_cnt++] = ep_num; }
//			ep_num++; ep_gint >>= 1U;
//		}
//	}
//	/* OUT endpoint interrupts */
//	if (irqs & USB_OTG_GINTSTS_OEPINT) {
//		ep_gint = (device->DAINT & device->DAINTMSK) >> 0x10UL;
//		ep_num = 0U;
//		while (ep_gint) {
//			if (ep_gint & 0x1U)							{ OEP_common_handler(handle, ep_num); 	USB_IRQ_log[USB_IRQ_cnt++] = 0x10 + ep_num; }
//			ep_num++; ep_gint >>= 1U;
//		}
//	}
//	/* incomplete isochronous IN interrupt */
//	if (irqs & USB_OTG_GINTSTS_IISOIXFR)				{ USB_incomplete_ISO_IN_IRQ(handle);	USB_IRQ_log[USB_IRQ_cnt++] = 'I'; }
//	/* incomplete isochronous OUT interrupt */
//	if (irqs & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)		{ USB_incomplete_ISO_OUT_IRQ(handle); 	USB_IRQ_log[USB_IRQ_cnt++] = 'O'; }
//	/* connection event interrupt */
//	if (irqs & USB_OTG_GINTSTS_SRQINT)					{ USB_connection_IRQ(handle); 			USB_IRQ_log[USB_IRQ_cnt++] = 'C'; }
//	/* wake-up interrupt */
//	if (irqs & USB_OTG_GINTSTS_WKUINT)					{ USB_wake_up_IRQ(handle); 				USB_IRQ_log[USB_IRQ_cnt++] = 'W'; }
//}
//static inline void USB_wakeup_handler(USB_handle_t* handle) {
//	USB_global_t*	usb =		handle->instance;
//	_IO uint32_t*			PCGCCTL =	(void*)(((uint32_t)usb) + USB_PCGCCTL_REL_BASE);
//	if (handle->config.low_power_enable) {
//		SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
//		sys_init();
//	}
//	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK);
//	EXTI->PR |= 0x1UL << 18U;  // USB FS EXTI line TODO: modular
//}
//
//
///*!<
// * interrupts
// * */
//void USB_handler(void)		{ USB_common_handler(&USB_handle); }
//void USB_WKUP_handler(void)	{ USB_wakeup_handler(&USB_handle); }
