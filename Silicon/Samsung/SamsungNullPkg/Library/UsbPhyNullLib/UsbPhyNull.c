#include <Library/UsbPhyLib.h>

EFI_STATUS
GetUsbPhyConfigs (
  OUT USB_PHY_CONFIG  ***Configs,
  OUT UINT8            *Count
  )
{
  *Configs = NULL;
  *Count   = 0;
  return EFI_UNSUPPORTED;
}

EFI_STATUS
UsbPhyInit (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  return EFI_UNSUPPORTED;
}

EFI_STATUS
UsbPhyExit (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  return EFI_UNSUPPORTED;
}

VOID
UsbPhyConnect (VOID)
{
}

VOID
UsbPhyDisconnect (VOID)
{
}
