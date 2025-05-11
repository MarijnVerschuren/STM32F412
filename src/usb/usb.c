//
// Created by marijn on 7/19/23.
//

#include "usb/usb.h"

/*!<
 * defines
 * */
#define USB_OTG_FS_WAKEUP_EXTI_LINE	(0x1U << 18)  /*!< USB FS EXTI Line WakeUp Interrupt */


/*!<
 * variables
 * */
#if (USBD_LPM_ENABLED == 1)
uint8_t* USBD_FS_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#endif /* (USBD_LPM_ENABLED == 1) */
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


PCD_HandleTypeDef hpcd_USB_OTG_FS;
USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;
extern USBD_ClassTypeDef USBD_HID;


/*!<
 * functions
 * */
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx){
  USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
  return HAL_OK;
}

HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx){
  USBx->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
  return HAL_OK;
}

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle){
    __HAL_RCC_CLK48_CONFIG(RCC_CLK48CLKSOURCE_PLLQ);

	fconfig_GPIO(GPIOA, 11, GPIO_alt_func, 10);
	fconfig_GPIO(GPIOA, 12, GPIO_alt_func, 10);

	__HAL_RCC_USB_OTG_FS_CLK_ENABLE();

	NVIC_set_IRQ_priority(USB_IRQn, 0);
	NVIC_enable_IRQ(USB_IRQn);
}

static HAL_StatusTypeDef USB_CoreReset(USB_OTG_GlobalTypeDef *USBx) {
  _IO uint32_t count = 0U;

  do{
    count++;
    if (count > HAL_USB_TIMEOUT) {
      return HAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  count = 0U;
  USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  do {
    count++;
    if (count > HAL_USB_TIMEOUT) {
      return HAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

  return HAL_OK;
}

uint32_t USB_GetMode(const USB_OTG_GlobalTypeDef *USBx) {
  return ((USBx->GINTSTS) & 0x1U);
}

HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_OTG_ModeTypeDef mode) {
  uint32_t ms = 0U;
  USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
  if (mode == USB_HOST_MODE) {
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;
    do {
      delay_ms(10U);
      ms += 10U;
    } while ((USB_GetMode(USBx) != (uint32_t)USB_HOST_MODE) && (ms < HAL_USB_CURRENT_MODE_MAX_DELAY_MS));
  }
  else if (mode == USB_DEVICE_MODE) {
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    do {
      delay_ms(10U);
      ms += 10U;
    } while ((USB_GetMode(USBx) != (uint32_t)USB_DEVICE_MODE) && (ms < HAL_USB_CURRENT_MODE_MAX_DELAY_MS));
  }
  else {
    return HAL_ERROR;
  }

  if (ms == HAL_USB_CURRENT_MODE_MAX_DELAY_MS){
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef USB_SetDevSpeed(const USB_OTG_GlobalTypeDef *USBx, uint8_t speed){
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_DEVICE->DCFG |= speed;
  return HAL_OK;
}

HAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num) {
  __IO uint32_t count = 0U;
  do{
    count++;
    if (count > HAL_USB_TIMEOUT){
      return HAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  count = 0U;
  USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << 6));

  do{
    count++;
    if (count > HAL_USB_TIMEOUT){
      return HAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

  return HAL_OK;
}

HAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx) {
  __IO uint32_t count = 0U;

  do {
    count++;
    if (count > HAL_USB_TIMEOUT) {
      return HAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

  count = 0U;
  USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

  do {
    count++;
    if (count > HAL_USB_TIMEOUT) {
      return HAL_TIMEOUT;
    }
  } while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

  return HAL_OK;
}

HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg){
  HAL_StatusTypeDef ret = HAL_OK;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t i;

  for (i = 0U; i < 15U; i++){
    USBx->DIEPTXF[i] = 0U;
  }

  /* VBUS Sensing setup */
  if (cfg.vbus_sensing_enable == 0U){
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;

    /* Deactivate VBUS Sensing B */
    USBx->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

    /* B-peripheral session valid override enable */
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  }
  else{
    /* Enable HW VBUS sensing */
    USBx->GCCFG |= USB_OTG_GCCFG_VBDEN;
  }

  /* Restart the Phy Clock */
  USBx_PCGCCTL = 0U;
  if (cfg.phy_itface == USB_OTG_ULPI_PHY){
    if (cfg.speed == USBD_HS_SPEED){
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH);
    }
    else{
      (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_HIGH_IN_FULL);
    }
  }
  else{
    (void)USB_SetDevSpeed(USBx, USB_OTG_SPEED_FULL);
  }

  /* Flush the FIFOs */
  if (USB_FlushTxFifo(USBx, 0x10U) != HAL_OK) /* all Tx FIFOs */{
    ret = HAL_ERROR;
  }
  if (USB_FlushRxFifo(USBx) != HAL_OK){
    ret = HAL_ERROR;
  }

  /* Clear all pending Device Interrupts */
  USBx_DEVICE->DIEPMSK = 0U;
  USBx_DEVICE->DOEPMSK = 0U;
  USBx_DEVICE->DAINTMSK = 0U;

  for (i = 0U; i < cfg.dev_endpoints; i++){
    if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA){
      if (i == 0U){
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
      }
      else{
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
      }
    }
    else{
      USBx_INEP(i)->DIEPCTL = 0U;
    }

    USBx_INEP(i)->DIEPTSIZ = 0U;
    USBx_INEP(i)->DIEPINT  = 0xFB7FU;
  }

  for (i = 0U; i < cfg.dev_endpoints; i++){
    if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA){
      if (i == 0U){
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
      }
      else{
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
      }
    }
    else{
      USBx_OUTEP(i)->DOEPCTL = 0U;
    }

    USBx_OUTEP(i)->DOEPTSIZ = 0U;
    USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
  }

  USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

  /* Disable all interrupts. */
  USBx->GINTMSK = 0U;
  /* Clear any pending interrupts */
  USBx->GINTSTS = 0xBFFFFFFFU;
  /* Enable the common interrupts */
  if (cfg.dma_enable == 0U){
    USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
  }

  /* Enable interrupts matching to the Device mode ONLY */
  USBx->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
                   USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
                   USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
                   USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;

  if (cfg.Sof_enable != 0U) {
    USBx->GINTMSK |= USB_OTG_GINTMSK_SOFM;
  }
  if (cfg.vbus_sensing_enable == 1U){
    USBx->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
  }
  return ret;
}

HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg){
  HAL_StatusTypeDef ret;
  if (cfg.phy_itface == USB_OTG_ULPI_PHY){
    USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
    if (cfg.use_external_vbus == 1U){
      USBx->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;
    }
    ret = USB_CoreReset(USBx);
  }
  else /* FS interface (embedded Phy) */{
    /* Select FS Embedded PHY */
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
    /* Reset after a PHY select */
    ret = USB_CoreReset(USBx);
    if (cfg.battery_charging_enable == 0U){
      USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;
    }
    else{
      USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
    }
  }

  if (cfg.dma_enable == 1U){
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_2;
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
  }

  return ret;
}

HAL_StatusTypeDef HAL_PCDEx_ActivateLPM(PCD_HandleTypeDef *hpcd){
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;

  hpcd->lpm_active = 1U;
  hpcd->LPM_State = LPM_L0;
  USBx->GINTMSK |= USB_OTG_GINTMSK_LPMINTM;
  USBx->GLPMCFG |= (USB_OTG_GLPMCFG_LPMEN | USB_OTG_GLPMCFG_LPMACK | USB_OTG_GLPMCFG_ENBESL);
  return HAL_OK;
}

HAL_StatusTypeDef USB_DevDisconnect(const USB_OTG_GlobalTypeDef *USBx) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
  USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size)
{
  uint8_t i;
  uint32_t Tx_Offset;

  /*  TXn min size = 16 words. (n  : Transmit FIFO index)
      When a TxFIFO is not used, the Configuration should be as follows:
          case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txm can use the space allocated for Txn.
         case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txn should be configured with the minimum space of 16 words
     The FIFO is used optimally when used TxFIFOs are allocated in the top
         of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
     When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */

  Tx_Offset = hpcd->Instance->GRXFSIZ;

  if (fifo == 0U)
  {
    hpcd->Instance->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;
  }
  else
  {
    Tx_Offset += (hpcd->Instance->DIEPTXF0_HNPTXFSIZ) >> 16;
    for (i = 0U; i < (fifo - 1U); i++)
    {
      Tx_Offset += (hpcd->Instance->DIEPTXF[i] >> 16);
    }

    /* Multiply Tx_Size by 2 to get higher performance */
    hpcd->Instance->DIEPTXF[fifo - 1U] = ((uint32_t)size << 16) | Tx_Offset;
  }

  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size){
  hpcd->Instance->GRXFSIZ = size;
  return HAL_OK;
}


HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd){
  const USB_OTG_GlobalTypeDef *USBx;
  uint8_t i;

  USBx = hpcd->Instance;

  if (hpcd->State == HAL_PCD_STATE_RESET){
    hpcd->Lock = HAL_UNLOCKED;
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SOFCallback = HAL_PCD_SOFCallback;
    hpcd->SetupStageCallback = HAL_PCD_SetupStageCallback;
    hpcd->ResetCallback = HAL_PCD_ResetCallback;
    hpcd->SuspendCallback = HAL_PCD_SuspendCallback;
    hpcd->ResumeCallback = HAL_PCD_ResumeCallback;
    hpcd->ConnectCallback = HAL_PCD_ConnectCallback;
    hpcd->DisconnectCallback = HAL_PCD_DisconnectCallback;
    hpcd->DataOutStageCallback = HAL_PCD_DataOutStageCallback;
    hpcd->DataInStageCallback = HAL_PCD_DataInStageCallback;
    hpcd->ISOOUTIncompleteCallback = HAL_PCD_ISOOUTIncompleteCallback;
    hpcd->ISOINIncompleteCallback = HAL_PCD_ISOINIncompleteCallback;
    hpcd->LPMCallback = HAL_PCDEx_LPM_Callback;
    hpcd->BCDCallback = HAL_PCDEx_BCD_Callback;
    if (hpcd->MspInitCallback == NULL){
      hpcd->MspInitCallback = HAL_PCD_MspInit;
    }
    /* Init the low level hardware */
    hpcd->MspInitCallback(hpcd);
#else
    HAL_PCD_MspInit(hpcd);
#endif /* (USE_HAL_PCD_REGISTER_CALLBACKS) */
  }
  hpcd->State = HAL_PCD_STATE_BUSY;
  __HAL_PCD_DISABLE(hpcd);

  if (USB_CoreInit(hpcd->Instance, hpcd->Init) != HAL_OK){
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  if (USB_SetCurrentMode(hpcd->Instance, USB_DEVICE_MODE) != HAL_OK){
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++){
    /* Init ep structure */
    hpcd->IN_ep[i].is_in = 1U;
    hpcd->IN_ep[i].num = i;
    hpcd->IN_ep[i].tx_fifo_num = i;
    /* Control until ep is activated */
    hpcd->IN_ep[i].type = EP_TYPE_CTRL;
    hpcd->IN_ep[i].maxpacket = 0U;
    hpcd->IN_ep[i].xfer_buff = 0U;
    hpcd->IN_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++){
    hpcd->OUT_ep[i].is_in = 0U;
    hpcd->OUT_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
    hpcd->OUT_ep[i].maxpacket = 0U;
    hpcd->OUT_ep[i].xfer_buff = 0U;
    hpcd->OUT_ep[i].xfer_len = 0U;
  }
  /* Init Device */
  if (USB_DevInit(hpcd->Instance, hpcd->Init) != HAL_OK){
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  hpcd->USB_Address = 0U;
  hpcd->State = HAL_PCD_STATE_READY;

  /* Activate LPM */
  if (hpcd->Init.lpm_enable == 1U){
    (void)HAL_PCDEx_ActivateLPM(hpcd);
  }
  (void)USB_DevDisconnect(hpcd->Instance);
  return HAL_OK;
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
  /* Link the driver to the stack. */
  hpcd_USB_OTG_FS.pData = pdev;
  pdev->pData = &hpcd_USB_OTG_FS;

  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = 0;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = 0;
  hpcd_USB_OTG_FS.Init.low_power_enable = 0;
  hpcd_USB_OTG_FS.Init.lpm_enable = 0;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = 0;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = 0;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = 0;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    for(;;);
  }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  /* Register USB PCD CallBacks */
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SOF_CB_ID, PCD_SOFCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SETUPSTAGE_CB_ID, PCD_SetupStageCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_RESET_CB_ID, PCD_ResetCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SUSPEND_CB_ID, PCD_SuspendCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_RESUME_CB_ID, PCD_ResumeCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_CONNECT_CB_ID, PCD_ConnectCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_DISCONNECT_CB_ID, PCD_DisconnectCallback);

  HAL_PCD_RegisterDataOutStageCallback(&hpcd_USB_OTG_FS, PCD_DataOutStageCallback);
  HAL_PCD_RegisterDataInStageCallback(&hpcd_USB_OTG_FS, PCD_DataInStageCallback);
  HAL_PCD_RegisterIsoOutIncpltCallback(&hpcd_USB_OTG_FS, PCD_ISOOUTIncompleteCallback);
  HAL_PCD_RegisterIsoInIncpltCallback(&hpcd_USB_OTG_FS, PCD_ISOINIncompleteCallback);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);
  return USBD_OK;
}


USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass) {
  uint16_t len = 0U;
  if (pclass == NULL) {
    return USBD_FAIL;
  }

  pdev->pClass[0] = pclass;
  if (pdev->pClass[pdev->classId]->GetFSConfigDescriptor != NULL){
    pdev->pConfDesc = (void *)pdev->pClass[pdev->classId]->GetFSConfigDescriptor(&len);
  }

  pdev->NumClasses++;
  return USBD_OK;
}

HAL_StatusTypeDef USB_DevConnect(const USB_OTG_GlobalTypeDef *USBx) {
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
  USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd) {
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  __HAL_LOCK(hpcd);
//  if (((USBx->GUSBCFG & USB_OTG_GUSBCFG_PHYSEL) != 0U) &&
//      (hpcd->Init.battery_charging_enable == 1U)) {
//    /* Enable USB Transceiver */
    //USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;
  __HAL_PCD_ENABLE(hpcd);
  (void)USB_DevConnect(hpcd->Instance);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status) {
  USBD_StatusTypeDef usb_status = USBD_OK;
  switch (hal_status) {
    case HAL_OK: usb_status = USBD_OK; break;
    case HAL_ERROR : usb_status = USBD_FAIL; break;
    case HAL_BUSY : usb_status = USBD_BUSY;  break;
    case HAL_TIMEOUT :  usb_status = USBD_FAIL; break;
    default :  usb_status = USBD_FAIL; break;
  }
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev){
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  hal_status = HAL_PCD_Start(pdev->pData);
  usb_status =  USBD_Get_USB_Status(hal_status);
  return usb_status;
}

USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *pdev){
#ifdef USE_USBD_COMPOSITE
  pdev->classId = 0U;
#endif /* USE_USBD_COMPOSITE */
  return USBD_LL_Start(pdev);
}

USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev, USBD_DescriptorsTypeDef *pdesc, uint8_t id) {
  USBD_StatusTypeDef ret;

  /* Check whether the USB Host handle is valid */
  if (pdev == NULL) {
    return USBD_FAIL;
  }

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Unlink previous class*/
    pdev->pClass[i] = NULL;
    pdev->pUserData[i] = NULL;

    /* Set class as inactive */
    pdev->tclasslist[i].Active = 0;
    pdev->NumClasses = 0;
    pdev->classId = 0;
  }
#else
  /* Unlink previous class*/
  pdev->pClass[0] = NULL;
  pdev->pUserData[0] = NULL;
#endif /* USE_USBD_COMPOSITE */
  pdev->pConfDesc = NULL;

  if (pdesc != NULL) {
    pdev->pDesc = pdesc;
  }

  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->id = id;

  ret = USBD_LL_Init(pdev);
  return ret;
}

void MX_USB_DEVICE_Init(void) {
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS)) {
	  for(;;);
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID)) {
	  for(;;);
  }
  if (USBD_Start(&hUsbDeviceFS)) {
	  for(;;);
  }
}




void flush_RX_FIFO(USB_OTG_GlobalTypeDef* usb) {
	_IO uint32_t why = 0;  // NOTE: creating a variable IS mandatory????
	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));	// wait for AHB master IDLE state
	usb->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;					// flush RX FIFO
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);			// wait until reset is processed
}
void flush_TX_FIFO(USB_OTG_GlobalTypeDef* usb, uint8_t ep) {
	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	usb->GRSTCTL = (
			ep << USB_OTG_GRSTCTL_TXFNUM_Pos		|			// select ep TX FIFO
			0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFO
	);
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
}
void flush_TX_FIFOS(USB_OTG_GlobalTypeDef* usb) {
	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	usb->GRSTCTL = (
			0x10UL << USB_OTG_GRSTCTL_TXFNUM_Pos		|		// select all TX FIFOs
			0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFOs
	);
	// TODO: hanging here
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
}


/*!<
 * init
 * */

//	if(USB_handle.config.low_power_enable == 1) {
//		EXTI->PR =		USB_OTG_FS_WAKEUP_EXTI_LINE;
//		EXTI->FTSR &=	~(USB_OTG_FS_WAKEUP_EXTI_LINE);
//		EXTI->RTSR |=	USB_OTG_FS_WAKEUP_EXTI_LINE;
//		EXTI->IMR |=	USB_OTG_FS_WAKEUP_EXTI_LINE;
//		NVIC_set_IRQ_priority(USB_WKUP_IRQn, 0);
//		NVIC_enable_IRQ(USB_WKUP_IRQn);
//	}

/*
  hpcd->lpm_active = 1U;
  hpcd->LPM_State = LPM_L0;
  USBx->GINTMSK |= USB_OTG_GINTMSK_LPMINTM;
  USBx->GLPMCFG |= (USB_OTG_GLPMCFG_LPMEN | USB_OTG_GLPMCFG_LPMACK | USB_OTG_GLPMCFG_ENBESL);
*/
void config_USB(
	USB_OTG_GlobalTypeDef* usb,
	uint8_t enable_SOF, uint8_t enable_low_power
) {
	uint8_t i;
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
	USB_OTG_INEndpointTypeDef*			in =		(void*)(((uint32_t)usb) + USB_OTG_IN_ENDPOINT_BASE);
	USB_OTG_OUTEndpointTypeDef*			out =		(void*)(((uint32_t)usb) + USB_OTG_OUT_ENDPOINT_BASE);
	_IO uint32_t*		PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);

	// enable CK48M and gpio
	RCC->DCKCFGR2 &= ~(0b1UL << 27);
	fconfig_GPIO(GPIOA, 11, GPIO_alt_func, 10);
	fconfig_GPIO(GPIOA, 12, GPIO_alt_func, 10);

	RCC->AHB2ENR |= 0b1UL << 7U;
	RCC->APB2ENR |= 0b1UL << 14U;

	NVIC_set_IRQ_priority(USB_IRQn, 0);
	NVIC_enable_IRQ(USB_IRQn);

	usb->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	usb->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
	usb->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;					// reset the core
	while (usb->GRSTCTL & USB_OTG_GRSTCTL_CSRST);			// wait until reset is processed

	usb->GCCFG |= USB_OTG_GCCFG_PWRDWN;
	usb->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);	// clear mode
	usb->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;								// set device mode
	while ((usb->GINTSTS) & 0b1U);  // wait until in device mode

	for (i = 0U; i < 15U; i++) { usb->DIEPTXF[i] = 0U; }

	device->DCTL |= USB_OTG_DCTL_SDIS;	// disconnect
    usb->GCCFG &= ~(0b1UL << 21U);		// disable VBUS sensing B
    // B-peripheral session valid override enable
	usb->GOTGCTL |= (0b1UL << 6U);
    usb->GOTGCTL |= (0b1UL << 7U);
	*PCGCCTL = 0U;						// restart phy clock

	device->DCFG |= USB_OTG_DCFG_DSPD;  // set full speed

	flush_TX_FIFOS(usb);// TODO <
	flush_RX_FIFO(usb);// TODO <

	device->DOEPMSK = 0U;
	device->DIEPMSK = 0U;
	device->DAINTMSK = 0U;
	for (i = 0U; i < 4U; i++) {
		if (in[i].DIEPCTL & USB_OTG_CTL_EPENA) {
			if (!i)	{ in[i].DIEPCTL = USB_OTG_CTL_SNAK; }
			else	{ in[i].DIEPCTL = USB_OTG_CTL_EPDIS | USB_OTG_CTL_SNAK; }
		} else		{ in[i].DIEPCTL = 0U; }
		if (out[i].DOEPCTL & USB_OTG_CTL_EPENA) {
			if (!i)	{ out[i].DOEPCTL = USB_OTG_CTL_SNAK; }
			else	{ out[i].DOEPCTL = USB_OTG_CTL_EPDIS | USB_OTG_CTL_SNAK; }
		} else		{ out[i].DOEPCTL = 0U; }
		in[i].DIEPTSIZ = out[i].DOEPTSIZ = 0U;
		in[i].DIEPINT  = out[i].DOEPINT  = 0xFB7FU;
	}

	device->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);
	usb->GINTMSK = 0U;
	usb->GINTSTS = 0xBFFFFFFFU;
	usb->GINTMSK |= (
		USB_OTG_GINTMSK_USBSUSPM		|
		USB_OTG_GINTMSK_USBRST			|
		USB_OTG_GINTMSK_RXFLVLM			|
		USB_OTG_GINTMSK_ENUMDNEM		|
		USB_OTG_GINTMSK_IEPINT			|
		USB_OTG_GINTMSK_OEPINT			|
		USB_OTG_GINTMSK_IISOIXFRM		|
		USB_OTG_GINTMSK_PXFRM_IISOOXFRM	|
		USB_OTG_GINTMSK_WUIM
	);

	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	device->DCTL |= USB_OTG_DCTL_SDIS;

	// TODO <
	// TODO!!!
	usb->GRXFSIZ = 0x80;											// TODO: argument
	usb->DIEPTXF0_HNPTXFSIZ = ((uint32_t)0x40 << 16) | 0x80;		// TODO: argument
	usb->DIEPTXF[0] = ((uint32_t)0x80 << 16) | 0xC0;				// TODO: argument + logic to select endpoints
}

void start_USB(USB_OTG_GlobalTypeDef* usb) {
	USB_OTG_DeviceTypeDef*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
	_IO uint32_t*				PCGCCTL =	(void*)(((uint32_t)usb) + USB_OTG_PCGCCTL_BASE);

	usb->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
	device->DCTL &= ~USB_OTG_DCTL_SDIS;
}




//void flush_RX_FIFO(USB_global_t* usb) {
//	_IO uint32_t why = 0;  // NOTE: creating a variable IS mandatory????
//	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));	// wait for AHB master IDLE state
//	usb->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;					// flush RX FIFO
//	while (usb->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH);			// wait until reset is processed
//}
//void flush_TX_FIFO(USB_global_t* usb, uint8_t ep) {
//	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
//	usb->GRSTCTL = (
//			ep << USB_OTG_GRSTCTL_TXFNUM_Pos		|			// select ep TX FIFO
//			0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFO
//	);
//	while (usb->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
//}
//void flush_TX_FIFOS(USB_global_t* usb) {
//	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
//	usb->GRSTCTL = (
//			0x10UL << USB_OTG_GRSTCTL_TXFNUM_Pos		|		// select all TX FIFOs
//			0b1UL << USB_OTG_GRSTCTL_TXFFLSH_Pos				// flush TX FIFOs
//	);
//	// TODO: hanging here
//	while (usb->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH);			// wait until reset is processed
//}
//
//
///*!<
// * init
// * */
//
////	if(USB_handle.config.low_power_enable == 1) {
////		EXTI->PR =		USB_OTG_FS_WAKEUP_EXTI_LINE;
////		EXTI->FTSR &=	~(USB_OTG_FS_WAKEUP_EXTI_LINE);
////		EXTI->RTSR |=	USB_OTG_FS_WAKEUP_EXTI_LINE;
////		EXTI->IMR |=	USB_OTG_FS_WAKEUP_EXTI_LINE;
////		NVIC_set_IRQ_priority(USB_WKUP_IRQn, 0);
////		NVIC_enable_IRQ(USB_WKUP_IRQn);
////	}
//
///*
//  hpcd->lpm_active = 1U;
//  hpcd->LPM_State = LPM_L0;
//  USBx->GINTMSK |= USB_OTG_GINTMSK_LPMINTM;
//  USBx->GLPMCFG |= (USB_OTG_GLPMCFG_LPMEN | USB_OTG_GLPMCFG_LPMACK | USB_OTG_GLPMCFG_ENBESL);
//*/
//void config_USB(
//	USB_global_t* usb, class_handle_t* class, descriptor_handle_t* desc,
//	uint8_t enable_SOF, uint8_t enable_low_power
//) {
//	uint8_t i;
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	USB_EP_t*			in =		(void*)(((uint32_t)usb) + USB_IN_ENDPOINT_REL_BASE);
//	USB_EP_t*			out =		(void*)(((uint32_t)usb) + USB_OUT_ENDPOINT_REL_BASE);
//	_IO uint32_t*		PCGCCTL =	(void*)(((uint32_t)usb) + USB_PCGCCTL_REL_BASE);
//
//	USB_handle.instance =					usb;
//	USB_handle.desc =						desc;
//	USB_handle.class =						class;
//	USB_handle.dev_state =					DEV_STATE_DEFAULT;
//	USB_handle.address =					0U;
//	USB_handle.config.dev_endpoints =		4U;
//	USB_handle.config.SOF_enable =			enable_SOF;
//	USB_handle.config.low_power_enable =	enable_low_power;
//
//	// enable CK48M and gpio
//	RCC->DCKCFGR2 &= ~(0b1UL << 27);
//	fconfig_GPIO(GPIOA, 11, GPIO_alt_func, 10);
//	fconfig_GPIO(GPIOA, 12, GPIO_alt_func, 10);
//
//	RCC->AHB2ENR |= 0b1UL << 7U;
//	RCC->APB2ENR |= 0b1UL << 14U;
//
//	NVIC_set_IRQ_priority(USB_IRQn, 0);
//	NVIC_enable_IRQ(USB_IRQn);
//
//	usb->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
//	usb->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
//	while (!(usb->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));		// wait for AHB master IDLE state
//	usb->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;					// reset the core
//	while (usb->GRSTCTL & USB_OTG_GRSTCTL_CSRST);			// wait until reset is processed
//
//	usb->GCCFG |= USB_OTG_GCCFG_PWRDWN;
//	usb->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);	// clear mode
//	usb->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;								// set device mode
//	while ((usb->GINTSTS) & 0b1U);  // wait until in device mode
//
//	for (i = 0U; i < USB_handle.config.dev_endpoints; i++) {
//		USB_handle.IN_ep[i].type =		USB_handle.OUT_ep[i].type =		EP_TYPE_CTRL;
//		USB_handle.IN_ep[i].mps =		USB_handle.OUT_ep[i].mps =		0U;
//		USB_handle.IN_ep[i].buffer =	USB_handle.OUT_ep[i].buffer =	NULL;
//		USB_handle.IN_ep[i].size =		USB_handle.OUT_ep[i].size =		0U;
//	}
//
//	for (i = 0U; i < 15U; i++) { usb->TXF[i] = 0U; }
//
//	device->DCTL |= USB_OTG_DCTL_SDIS;	// disconnect
//    usb->GCCFG &= ~(0b1UL << 21U);		// disable VBUS sensing B
//    // B-peripheral session valid override enable
//	usb->GOTGCTL |= (0b1UL << 6U);
//    usb->GOTGCTL |= (0b1UL << 7U);
//	*PCGCCTL = 0U;						// restart phy clock
//
//	device->DCFG |= USB_OTG_DCFG_DSPD;  // set full speed
//
//	flush_TX_FIFOS(usb);// TODO <
//	flush_RX_FIFO(usb);// TODO <
//
//	device->DOEPMSK = 0U;
//	device->DIEPMSK = 0U;
//	device->DAINTMSK = 0U;
//	for (i = 0U; i < USB_handle.config.dev_endpoints; i++) {
//		if (in[i].CTL & USB_OTG_CTL_EPENA) {
//			if (!i)	{ in[i].CTL = USB_OTG_CTL_SNAK; }
//			else	{ in[i].CTL = USB_OTG_CTL_EPDIS | USB_OTG_CTL_SNAK; }
//		} else		{ in[i].CTL = 0U; }
//		if (out[i].CTL & USB_OTG_CTL_EPENA) {
//			if (!i)	{ out[i].CTL = USB_OTG_CTL_SNAK; }
//			else	{ out[i].CTL = USB_OTG_CTL_EPDIS | USB_OTG_CTL_SNAK; }
//		} else		{ out[i].CTL = 0U; }
//		in[i].TSIZ = out[i].TSIZ = 0U;
//		in[i].INT  = out[i].INT  = 0xFB7FU;
//	}
//
//	device->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);
//	usb->GINTMSK = 0U;
//	usb->GINTSTS = 0xBFFFFFFFU;
//	usb->GINTMSK |= (
//		USB_OTG_GINTMSK_USBSUSPM		|
//		USB_OTG_GINTMSK_USBRST			|
//		USB_OTG_GINTMSK_RXFLVLM			|
//		USB_OTG_GINTMSK_ENUMDNEM		|
//		USB_OTG_GINTMSK_IEPINT			|
//		USB_OTG_GINTMSK_OEPINT			|
//		USB_OTG_GINTMSK_IISOIXFRM		|
//		USB_OTG_GINTMSK_PXFRM_IISOOXFRM	|
//		USB_OTG_GINTMSK_WUIM
//	);
//
//	if (USB_handle.config.SOF_enable != 0U) { usb->GINTMSK |= USB_OTG_GINTMSK_SOFM; }
//	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
//	device->DCTL |= USB_OTG_DCTL_SDIS;
//
//	// TODO <
//	// TODO!!!
//	usb->GRXFSIZ = 0x80;											// TODO: argument
//	usb->TXF0_HNPTXFSIZ = ((uint32_t)0x40 << 16) | 0x80;		// TODO: argument
//	usb->TXF[0] = ((uint32_t)0x80 << 16) | 0xC0;				// TODO: argument + logic to select endpoints
//}
//
//void start_USB(USB_global_t* usb) {
//	USB_device_t*		device =	(void*)(((uint32_t)usb) + USB_DEVICE_REL_BASE);
//	_IO uint32_t*				PCGCCTL =	(void*)(((uint32_t)usb) + USB_PCGCCTL_REL_BASE);
//
//	usb->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
//	*PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
//	device->DCTL &= ~USB_OTG_DCTL_SDIS;
//}