#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryMapHelperLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/MmcPlatformLib.h>
#include <Library/DwMmcRegs.h>
#include <Protocol/EfiGpio.h>
#include <Protocol/EfiSpeedy.h>
#include <Drivers/PmicDxe/S2mpu09.h>

//
// DW MMC base (SD card = dwmmc2)
//
#define DW_MMC2_BASE            0x13550000

//
// CMU TOP — MMC_CARD clock
//
#define CLK_CON_MUX_MMC_CARD    0x12101040UL
#define CLK_CON_DIV_MMC_CARD    0x12101848UL
#define CLK_CON_GAT_MMC_CARD    0x12102048UL

#define CMU_MUX_SEL_PLL          0x1
#define CMU_BUSY_BIT             (1 << 16)
#define CMU_GATE_MANUAL          (1 << 20)
#define CMU_GATE_CG_VAL          (1 << 21)
#define CMU_DIV_MASK             0x1FF
#define CMU_TIMEOUT              100

#define EXYNOS9610_SDR_CLKSEL    EXYNOS_CLKSEL_TIMING (3, 0, 2, 0)
#define EXYNOS9610_DDR_CLKSEL    EXYNOS_CLKSEL_TIMING (3, 0, 2, 1)
#define EXYNOS9610_SDR50_CLKSEL  EXYNOS_CLKSEL_TIMING (3, 0, 4, 2)
#define EXYNOS9610_SDR104_CLKSEL EXYNOS_CLKSEL_TIMING (3, 0, 3, 0)

//
// Controller parameters
//
#define MMC_BUS_HZ              800000000
#define MMC_CIU_DIV             4
#define MMC_CIUDIV              3
#define MMC_FIFO_DEPTH          0x40

STATIC
EFI_STATUS
MmcPowerSet (
  VOID
  )
{
  EFI_STATUS                    Status;
  EFI_SPEEDY_PROTOCOL          *Speedy;
  EFI_MEMORY_REGION_DESCRIPTOR  SpeedyMem;
  UINT8                         Value;

  Status = gBS->LocateProtocol (&gEfiSpeedyProtocolGuid, NULL, (VOID *)&Speedy);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: Speedy protocol not found: %r\n", Status));
    return Status;
  }

  Status = LocateMemoryRegionByName ("Speedy", &SpeedyMem);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: Speedy memory region not found: %r\n", Status));
    return Status;
  }

  // LDO2: VQMMC (I/O voltage)
  Status = Speedy->Read (SpeedyMem.Address, S2MPU09_PM_ADDR,
                         S2MPU09_PM_LDO2_CTRL, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_CTRL read failed: %r\n", Status));
    return Status;
  }
  Value |= S2MPU09_OUTPUT_ON_NORMAL;
  Status = Speedy->Write (SpeedyMem.Address, S2MPU09_PM_ADDR,
                          S2MPU09_PM_LDO2_CTRL, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_CTRL write failed: %r\n", Status));
    return Status;
  }

  // LDO35: VMMC (core voltage)
  Status = Speedy->Read (SpeedyMem.Address, S2MPU09_PM_ADDR,
                         S2MPU09_PM_LDO35_CTRL, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO35_CTRL read failed: %r\n", Status));
    return Status;
  }
  Value |= S2MPU09_OUTPUT_ON_NORMAL;
  Status = Speedy->Write (SpeedyMem.Address, S2MPU09_PM_ADDR,
                          S2MPU09_PM_LDO35_CTRL, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO35_CTRL write failed: %r\n", Status));
    return Status;
  }

  MicroSecondDelay (1200);
  return EFI_SUCCESS;
}

EFI_STATUS
SdVoltageSwitch (
  VOID
  )
{
  EFI_STATUS                    Status;
  EFI_SPEEDY_PROTOCOL          *Speedy;
  EFI_MEMORY_REGION_DESCRIPTOR  SpeedyMem;
  UINT8                         Value;

  Status = gBS->LocateProtocol (&gEfiSpeedyProtocolGuid, NULL, (VOID *)&Speedy);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: Speedy not found for voltage switch: %r\n",
      Status));
    return Status;
  }

  Status = LocateMemoryRegionByName ("Speedy", &SpeedyMem);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: Speedy region not found: %r\n", Status));
    return Status;
  }

  Status = Speedy->Read (SpeedyMem.Address, S2MPU09_PM_ADDR,
                         S2MPU09_PM_LDO2_VOLT, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_VOLT read failed: %r\n", Status));
    return Status;
  }

  Value &= ~S2MPU09_LDO_VSEL_MASK;
  Value |= S2MPU09_LDO_VSEL_1V8;

  Status = Speedy->Write (SpeedyMem.Address, S2MPU09_PM_ADDR,
                          S2MPU09_PM_LDO2_VOLT, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_VOLT write (1.8V) failed: %r\n", Status));
    return Status;
  }

  MicroSecondDelay (500);
  DEBUG ((EFI_D_WARN, "MMC: LDO2 VQMMC → 1.8V\n"));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MmcGpioInitSd (
  VOID
  )
{
  EFI_STATUS                 Status;
  EFI_EXYNOS_GPIO_PROTOCOL  *Gpio;
  UINT8                      Pin;

  Status = gBS->LocateProtocol (&gEfiExynosGpioProtocolGuid, NULL,
                                (VOID *)&Gpio);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPIO protocol not found: %r\n", Status));
    return Status;
  }

  // GPF2-0: SD_CLK (func 2, no pull)
  Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_F, 0, 2);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPF2-0 CLK config failed: %r\n", Status));
    return Status;
  }
  Gpio->SetPull (2, GPIO_BANK_ID_F, 0, GPIO_PULL_NONE);
  Gpio->SetDrv  (2, GPIO_BANK_ID_F, 0, 6);

  // GPF2-1: SD_CMD (func 2, pull-up)
  Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_F, 1, 2);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPF2-1 CMD config failed: %r\n", Status));
    return Status;
  }
  Gpio->SetPull (2, GPIO_BANK_ID_F, 1, GPIO_PULL_UP);
  Gpio->SetDrv  (2, GPIO_BANK_ID_F, 1, 4);

  // GPF2-2..5: SD_DATA[0..3] (func 2, pull-up)
  for (Pin = 2; Pin <= 5; Pin++) {
    Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_F, Pin, 2);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "MMC: GPF2-%d DATA config skip: %r\n", Pin, Status));
      continue;
    }
    Gpio->SetPull (2, GPIO_BANK_ID_F, Pin, GPIO_PULL_UP);
    Gpio->SetDrv  (2, GPIO_BANK_ID_F, Pin, 4);
  }

  Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_A, 6, 0);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPA2-6 CD config failed: %r\n", Status));
  } else {
    Gpio->SetPull (2, GPIO_BANK_ID_A, 6, GPIO_PULL_UP);
  }

  return EFI_SUCCESS;
}

STATIC
VOID
MmcClockInitSd (
  VOID
  )
{
  UINT32  Reg;
  UINT32  Timeout;

  // Clear divider
  Reg = MmioRead32 (CLK_CON_DIV_MMC_CARD);
  Reg &= ~CMU_DIV_MASK;
  MmioWrite32 (CLK_CON_DIV_MMC_CARD, Reg);
  for (Timeout = 0; Timeout < CMU_TIMEOUT; Timeout++) {
    if (!(MmioRead32 (CLK_CON_DIV_MMC_CARD) & CMU_BUSY_BIT)) {
      break;
    }
  }

  // Select PLL
  Reg = MmioRead32 (CLK_CON_MUX_MMC_CARD);
  Reg |= CMU_MUX_SEL_PLL;
  MmioWrite32 (CLK_CON_MUX_MMC_CARD, Reg);
  for (Timeout = 0; Timeout < CMU_TIMEOUT; Timeout++) {
    if (!(MmioRead32 (CLK_CON_MUX_MMC_CARD) & CMU_BUSY_BIT)) {
      break;
    }
  }

  // Gate on (manual)
  MmioWrite32 (CLK_CON_GAT_MMC_CARD, CMU_GATE_MANUAL | CMU_GATE_CG_VAL);
}

EFI_STATUS
MmcPlatformInit (
  IN  UINT32              Channel,
  OUT MMC_PLATFORM_HOST  *Host
  )
{
  EFI_STATUS  Status;

  if (!Host) {
    return EFI_INVALID_PARAMETER;
  }

  Status = MmcPowerSet ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC%d: PMIC power failed: %r\n", Channel, Status));
    return Status;
  }

  if (Channel == MMC_CHANNEL_SD) {
    Status = MmcGpioInitSd ();
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "MMC%d: GPIO init failed: %r\n", Channel, Status));
      return Status;
    }

    MmcClockInitSd ();

    Host->IoBase       = DW_MMC2_BASE;
    Host->SdrClksel    = EXYNOS9610_SDR_CLKSEL;
    Host->DdrClksel    = EXYNOS9610_DDR_CLKSEL;
    Host->Sdr50Clksel  = EXYNOS9610_SDR50_CLKSEL;
    Host->Sdr104Clksel = EXYNOS9610_SDR104_CLKSEL;
    Host->Hs200Clksel  = 0;
    Host->Hs400Clksel  = 0;
    Host->BusHz        = MMC_BUS_HZ;
    Host->PhaseDivide  = MMC_CIU_DIV;
    Host->Ciudiv       = MMC_CIUDIV;
    Host->FifoDepth    = MMC_FIFO_DEPTH;
    Host->Secure       = FALSE;
    Host->MpsSecurity  = 0;

  } else {
    DEBUG ((EFI_D_ERROR, "MMC: Unsupported channel: %d\n", Channel));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}
