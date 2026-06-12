#include <Library/UartLib.h>

VOID
GetUartBusData (
  OUT EFI_UART_BUS_DATA **Data,
  OUT UINT8              *Count)
{
  *Data  = NULL;
  *Count = 0;
}

UINT32
GetUartClock (
  VOID
  )
{
  return 0;
}
