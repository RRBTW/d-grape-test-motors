#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#ifndef USB_SIZ_STRING_SERIAL
#define USB_SIZ_STRING_SERIAL  0x1AU
#endif

#define USBD_VID              0x0483U
#define USBD_PID_FS           0x5740U
#define USBD_LANGID_STRING    0x0409U
#define USBD_MANUFACTURER_STRING  "D-Grape Robot"
#define USBD_PRODUCT_STRING   "D-Grape Test Motors"
#define USBD_SERIALNUMBER_STRING "00000000001A"

static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
    USBD_FS_DeviceDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor,
};

#if defined(__ICCARM__)
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12, USB_DESC_TYPE_DEVICE, 0x00, 0x02,
    0x02, 0x00, 0x00, USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS),
    0x00, 0x02,
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR, USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
    USB_SIZ_STRING_SERIAL, USB_DESC_TYPE_STRING,
};

uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
    { (void)speed; *length = sizeof(USBD_FS_DeviceDesc); return USBD_FS_DeviceDesc; }

uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
    { (void)speed; *length = sizeof(USBD_LangIDDesc); return USBD_LangIDDesc; }

uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
    { (void)speed; USBD_GetString((uint8_t *)USBD_PRODUCT_STRING, USBD_StrDesc, length); return USBD_StrDesc; }

uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
    { (void)speed; USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length); return USBD_StrDesc; }

uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
    { (void)speed; *length = USB_SIZ_STRING_SERIAL; Get_SerialNum(); return (uint8_t *)USBD_StringSerial; }

uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
    { (void)speed; USBD_GetString((uint8_t *)"CDC Config", USBD_StrDesc, length); return USBD_StrDesc; }

uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
    { (void)speed; USBD_GetString((uint8_t *)"CDC Interface", USBD_StrDesc, length); return USBD_StrDesc; }

static void Get_SerialNum(void)
{
    uint32_t s0 = *(uint32_t *)0x1FFF7A10U;
    uint32_t s1 = *(uint32_t *)0x1FFF7A14U;
    uint32_t s2 = *(uint32_t *)0x1FFF7A18U;
    s0 += s2;
    if (s0 != 0U) {
        IntToUnicode(s0, &USBD_StringSerial[2], 8);
        IntToUnicode(s1, &USBD_StringSerial[18], 4);
    }
}

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
    for (uint8_t idx = 0U; idx < len; idx++) {
        uint8_t nibble = (uint8_t)((value >> 28U) & 0x0FU);
        pbuf[2U * idx]         = (nibble < 0x0AU) ? (nibble + '0') : (nibble - 0x0AU + 'A');
        pbuf[(2U * idx) + 1U]  = 0U;
        value <<= 4U;
    }
}
