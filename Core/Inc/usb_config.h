/*
 * Copyright (c) 2024, CherryUSB Community
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef STM32F4_FS_CONFIG_H
#define STM32F4_FS_CONFIG_H

/* ================ USB common Configuration ================ */

// ????RTOS????printf?rt_kprintf
#ifdef __RTTHREAD__
#include <rtthread.h>
#define CONFIG_USB_PRINTF(...) rt_kprintf(__VA_ARGS__)
#else
#include <stdio.h> // ????stdio.h???printf
#define CONFIG_USB_PRINTF(...) printf(__VA_ARGS__)
#endif

// USB????,???????
#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_INFO
#endif

// ????????,????
//#define CONFIG_USB_PRINTF_COLOR_ENABLE

// ????DCACHE,STM32F4????OS???OS????,???????
//#define CONFIG_USB_DCACHE_ENABLE

/* ??????,???DMA?DCACHE????????,FS??4???? */
#ifdef CONFIG_USB_DCACHE_ENABLE
#define CONFIG_USB_ALIGN_SIZE 32 // 32 ? 64
#else
#define CONFIG_USB_ALIGN_SIZE 4
#endif

/* ???????????RAM?,??DMA?? */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* ????usb_memcpy??????,??????Flash???
 * ??ARM libc?memcpy?????4??????????????
 */
// #define CONFIG_USB_MEMCPY_DISABLE

/* ================= USB Device Stack Configuration ================ */

/* Ep0 ???????????? */
#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512
#endif

/* ????????ep0????,??????ep0 reqdata
 * ???,????????CONFIG_USB_ALIGN_SIZE??
 */
// #define CONFIG_USBDEV_EP0_INDATA_NO_COPY

/* ???????????? */
// #define CONFIG_USBDEV_DESC_CHECK

/* ?????? */
// #define CONFIG_USBDEV_TEST_MODE

/* ?????????API */
#define CONFIG_USBDEV_ADVANCE_DESC

/* ?ep0???????ISR?????,???RTOS??? */
#define CONFIG_USBDEV_EP0_THREAD

#ifndef CONFIG_USBDEV_EP0_PRIO
#define CONFIG_USBDEV_EP0_PRIO 4 // EP0?????
#endif

#ifndef CONFIG_USBDEV_EP0_STACKSIZE
#define CONFIG_USBDEV_EP0_STACKSIZE 2048 // EP0??????
#endif

/* ================= USB Device Class Configuration ================ */

// MSC (??????) ?? - ???
// #ifndef CONFIG_USBDEV_MSC_MAX_LUN
// #define CONFIG_USBDEV_MSC_MAX_LUN 1 // ???????
// #endif

// #ifndef CONFIG_USBDEV_MSC_MAX_BUFSIZE
// #define CONFIG_USBDEV_MSC_MAX_BUFSIZE 512 // MSC???????
// #endif

// #ifndef CONFIG_USBDEV_MSC_MANUFACTURER_STRING
// #define CONFIG_USBDEV_MSC_MANUFACTURER_STRING "CherryUSB" // ??????
// #endif

// #ifndef CONFIG_USBDEV_MSC_PRODUCT_STRING
// #define CONFIG_USBDEV_MSC_PRODUCT_STRING "Mass Storage" // ?????
// #endif

// #ifndef CONFIG_USBDEV_MSC_VERSION_STRING
// #define CONFIG_USBDEV_MSC_VERSION_STRING "0.01" // ?????
// #endif

/* ?MSC?????ISR?????,???RTOS??? - ??? */
// #define CONFIG_USBDEV_MSC_THREAD

// #ifndef CONFIG_USBDEV_MSC_PRIO
// #define CONFIG_USBDEV_MSC_PRIO 4 // MSC?????
// #endif

// #ifndef CONFIG_USBDEV_MSC_STACKSIZE
// #define CONFIG_USBDEV_MSC_STACKSIZE 2048 // MSC??????
// #endif

// MTP (??????) ?? - ???
// #ifndef CONFIG_USBDEV_MTP_MAX_BUFSIZE
// #define CONFIG_USBDEV_MTP_MAX_BUFSIZE 2048
// #endif

// #ifndef CONFIG_USBDEV_MTP_MAX_OBJECTS
// #define CONFIG_USBDEV_MTP_MAX_OBJECTS 256
// #endif

// #ifndef CONFIG_USBDEV_MTP_MAX_PATHNAME
// #define CONFIG_USBDEV_MTP_MAX_PATHNAME 256
// #endif

// #define CONFIG_USBDEV_MTP_THREAD // MTP???? - ???

// #ifndef CONFIG_USBDEV_MTP_PRIO
// #define CONFIG_USBDEV_MTP_PRIO 4
// #endif

// #ifndef CONFIG_USBDEV_MTP_STACKSIZE
// #define CONFIG_USBDEV_MTP_STACKSIZE 4096
// #endif

// RNDIS (????????) ?? - ???
// #ifndef CONFIG_USBDEV_RNDIS_RESP_BUFFER_SIZE
// #define CONFIG_USBDEV_RNDIS_RESP_BUFFER_SIZE 156
// #endif

/* rndis???????,???(1536 + 44)??? - ??? */
// #ifndef CONFIG_USBDEV_RNDIS_ETH_MAX_FRAME_SIZE
// #define CONFIG_USBDEV_RNDIS_ETH_MAX_FRAME_SIZE 1580
// #endif

// #ifndef CONFIG_USBDEV_RNDIS_VENDOR_ID
// #define CONFIG_USBDEV_RNDIS_VENDOR_ID 0x0000ffff
// #endif

// #ifndef CONFIG_USBDEV_RNDIS_VENDOR_DESC
// #define CONFIG_USBDEV_RNDIS_VENDOR_DESC "CherryUSB"
// #endif

// #define CONFIG_USBDEV_RNDIS_USING_LWIP // RNDIS??LWIP - ???
// #define CONFIG_USBDEV_CDC_ECM_USING_LWIP // CDC ECM??LWIP - ???

/* ================ USB HOST Stack Configuration ================== */
// ????STM32F4??USB IP???????,?????????
// ??,?????????,?????????????????

#define CONFIG_USBHOST_MAX_RHPORTS            1
#define CONFIG_USBHOST_MAX_EXTHUBS            1
#define CONFIG_USBHOST_MAX_EHPORTS            4
#define CONFIG_USBHOST_MAX_INTERFACES         8
#define CONFIG_USBHOST_MAX_INTF_ALTSETTINGS   8
#define CONFIG_USBHOST_MAX_ENDPOINTS          4

// ?????????,??????????,?????
// #define CONFIG_USBHOST_MAX_CDC_ACM_CLASS 4
// #define CONFIG_USBHOST_MAX_HID_CLASS     4
// #define CONFIG_USBHOST_MAX_MSC_CLASS     2
// #define CONFIG_USBHOST_MAX_AUDIO_CLASS   1
// #define CONFIG_USBHOST_MAX_VIDEO_CLASS   1

#define CONFIG_USBHOST_DEV_NAMELEN 16

#ifndef CONFIG_USBHOST_PSC_PRIO
#define CONFIG_USBHOST_PSC_PRIO 0
#endif
#ifndef CONFIG_USBHOST_PSC_STACKSIZE
#define CONFIG_USBHOST_PSC_STACKSIZE 2048
#endif

// #define CONFIG_USBHOST_GET_STRING_DESC

// #define CONFIG_USBHOST_MSOS_ENABLE
#ifndef CONFIG_USBHOST_MSOS_VENDOR_CODE
#define CONFIG_USBHOST_MSOS_VENDOR_CODE 0x00
#endif

/* Ep0 ??????? */
#ifndef CONFIG_USBHOST_REQUEST_BUFFER_LEN
#define CONFIG_USBHOST_REQUEST_BUFFER_LEN 512
#endif

#ifndef CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT
#define CONFIG_USBHOST_CONTROL_TRANSFER_TIMEOUT 500
#endif

#ifndef CONFIG_USBHOST_MSC_TIMEOUT
#define CONFIG_USBHOST_MSC_TIMEOUT 5000
#endif

// RNDIS/CDC NCM/ASIX/RTL8152 ????????? - ????
// #ifndef CONFIG_USBHOST_RNDIS_ETH_MAX_RX_SIZE
// #define CONFIG_USBHOST_RNDIS_ETH_MAX_RX_SIZE (2048)
// #endif
// #ifndef CONFIG_USBHOST_RNDIS_ETH_MAX_TX_SIZE
// #define CONFIG_USBHOST_RNDIS_ETH_MAX_TX_SIZE (2048)
// #endif

// #ifndef CONFIG_USBHOST_CDC_NCM_ETH_MAX_RX_SIZE
// #define CONFIG_USBHOST_CDC_NCM_ETH_MAX_RX_SIZE (2048)
// #endif
// #ifndef CONFIG_USBHOST_CDC_NCM_ETH_MAX_TX_SIZE
// #define CONFIG_USBHOST_CDC_NCM_ETH_MAX_TX_SIZE (2048)
// #endif

// #ifndef CONFIG_USBHOST_ASIX_ETH_MAX_RX_SIZE
// #define CONFIG_USBHOST_ASIX_ETH_MAX_RX_SIZE (2048)
// #endif
// #ifndef CONFIG_USBHOST_ASIX_ETH_MAX_TX_SIZE
// #define CONFIG_USBHOST_ASIX_ETH_MAX_TX_SIZE (2048)
// #endif

// #ifndef CONFIG_USBHOST_RTL8152_ETH_MAX_RX_SIZE
// #define CONFIG_USBHOST_RTL8152_ETH_MAX_RX_SIZE (2048)
// #endif
// #ifndef CONFIG_USBHOST_RTL8152_ETH_MAX_TX_SIZE
// #define CONFIG_USBHOST_RTL8152_ETH_MAX_TX_SIZE (2048)
// #endif

// ??HCI???? - ????
// #define CONFIG_USBHOST_BLUETOOTH_HCI_H4
// #define CONFIG_USBHOST_BLUETOOTH_HCI_LOG

// #ifndef CONFIG_USBHOST_BLUETOOTH_TX_SIZE
// #define CONFIG_USBHOST_BLUETOOTH_TX_SIZE 2048
// #endif
// #ifndef CONFIG_USBHOST_BLUETOOTH_RX_SIZE
// #define CONFIG_USBHOST_BLUETOOTH_RX_SIZE 2048
// #endif

/* ================ USB Device Port Configuration ================*/

#ifndef CONFIG_USBDEV_MAX_BUS
#define CONFIG_USBDEV_MAX_BUS 1 // ????????1,HPM IP??
#endif

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 8 // USB??????
#endif

// #define CONFIG_USBDEV_SOF_ENABLE // ????SOF??

/* ???????????????????????,
 * ??IP???CONFIG_USB_HS?????????PHY?
 *
 * ?STM32???,??PB14/PB15??HS??,PA11/PA12???(????,???????)?
 * ???FS??,???????CONFIG_USB_HS?
 */
// #define CONFIG_USB_HS

/* ---------------- FSDEV Configuration ---------------- */
// ??STM32F4??USB,????PMA??
#define CONFIG_USBDEV_FSDEV_PMA_ACCESS 2 // ???1?2,??????????,??2????

/* ---------------- DWC2 Configuration ---------------- */
/* ??dwc2???????DMA??
 * ?STM32???,??PB14/PB15??DMA??,PA11/PA12???(????,???????)
 * ???FS??,???????CONFIG_USB_DWC2_DMA_ENABLE?
 */
// #define CONFIG_USB_DWC2_DMA_ENABLE

/* ---------------- MUSB Configuration ---------------- */
// #define CONFIG_USB_MUSB_SUNXI // ??MUSB IP???,STM32F4?????

/* ================ USB Host Port Configuration ==================*/
#ifndef CONFIG_USBHOST_MAX_BUS
#define CONFIG_USBHOST_MAX_BUS 1
#endif

#ifndef CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USBHOST_PIPE_NUM 10
#endif

/* ---------------- EHCI Configuration ---------------- */
// STM32F4??USB IP???????EHCI/OHCI/XHCI,???????????USB????
// ????F4???OTG_HS?????????,??????????????
/*
#define CONFIG_USB_EHCI_HCCR_OFFSET       (0x0)
#define CONFIG_USB_EHCI_FRAME_LIST_SIZE 1024
#define CONFIG_USB_EHCI_QH_NUM            CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USB_EHCI_QTD_NUM           (CONFIG_USB_EHCI_QH_NUM * 3)
#define CONFIG_USB_EHCI_ITD_NUM           4
// #define CONFIG_USB_EHCI_HCOR_RESERVED_DISABLE
// #define CONFIG_USB_EHCI_CONFIGFLAG
// #define CONFIG_USB_EHCI_ISO
// #define CONFIG_USB_EHCI_WITH_OHCI
// #define CONFIG_USB_EHCI_DESC_DCACHE_ENABLE
*/

/* ---------------- OHCI Configuration ---------------- */
/*
#define CONFIG_USB_OHCI_HCOR_OFFSET (0x0)
#define CONFIG_USB_OHCI_ED_NUM CONFIG_USBHOST_PIPE_NUM
#define CONFIG_USB_OHCI_TD_NUM 3
// #define CONFIG_USB_OHCI_DESC_DCACHE_ENABLE
*/

/* ---------------- XHCI Configuration ---------------- */
/*
#define CONFIG_USB_XHCI_HCCR_OFFSET (0x0)
*/

/* ---------------- MUSB Configuration ---------------- */
// #define CONFIG_USB_MUSB_SUNXI

// ?????RAM??????,??????
#ifndef usb_phyaddr2ramaddr
#define usb_phyaddr2ramaddr(addr) (addr)
#endif

// RAM???????????,??????
#ifndef usb_ramaddr2phyaddr
#define usb_ramaddr2phyaddr(addr) (addr)
#endif

#endif // STM32F4_FS_CONFIG_H
