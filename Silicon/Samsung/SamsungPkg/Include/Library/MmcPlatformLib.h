#ifndef _MMC_PLATFORM_LIB_H_
#define _MMC_PLATFORM_LIB_H_

#include <Uefi.h>
#include <Protocol/DevicePath.h>

//
// MMC channel identifiers.
// Channel 2 is the SD card slot (DW_MMC2) on Exynos platforms.
//
#define MMC_CHANNEL_SD    2

//
// Platform host configuration filled in by MmcPlatformInit().
//
typedef struct {
  UINTN    IoBase;
  UINT32   SdrClksel;
  UINT32   DdrClksel;
  UINT32   Sdr50Clksel;
  UINT32   Sdr104Clksel;
  UINT32   Hs200Clksel;
  UINT32   Hs400Clksel;
  UINT32   BusHz;
  UINT32   PhaseDivide;
  UINT32   Ciudiv;
  UINT32   FifoDepth;
  BOOLEAN  Secure;
  UINT32   MpsSecurity;
} MMC_PLATFORM_HOST;

/**
  Initialise MMC platform hardware for the specified channel.

  Handles:
    - PMIC regulator enable
    - GPIO pinmux configuration for the channel's pins
    - CMU clock setup (MUX, DIV, GATE)

  @param[in]  Channel   MMC channel (MMC_CHANNEL_SD).
  @param[out] Host      Platform host data filled by this function.

  @retval EFI_SUCCESS             Platform initialised.
  @retval EFI_INVALID_PARAMETER   Host is NULL or unsupported channel.
**/
EFI_STATUS
MmcPlatformInit (
  IN  UINT32              Channel,
  OUT MMC_PLATFORM_HOST  *Host
  );

/**
  Switch SD card I/O voltage from 3.3V to 1.8V (UHS-I).

  Called during the CMD11 voltage switch sequence by the DW MMC driver.
**/
EFI_STATUS
SdVoltageSwitch (
  VOID
  );

//
// SD Card Device Path
//
typedef struct {
  VENDOR_DEVICE_PATH        VendorDp;
  UINT8                     Slot;
  EFI_DEVICE_PATH_PROTOCOL  End;
} EXYNOS_SD_DEVICE_PATH;

#define EXYNOS_SD_CARD_GUID \
  { 0x7A2C3B4D, 0xE5F6, 0x4A8B, \
    { 0x9C, 0x0D, 0x1E, 0x2F, 0x3A, 0x4B, 0x5C, 0x6D } \
  }

extern EFI_GUID  gExynosSdCardGuid;

#endif // _MMC_PLATFORM_LIB_H_
