#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DwMmcRegs.h>
#include <Library/MmcPlatformLib.h>
#include <Protocol/DevicePath.h>
#include "SdMmcDxeInternal.h"

STATIC CONST UINT8 mTuningPattern4Bit[TUNING_BLOCK_SIZE] = {
  0xFF, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
  0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
  0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
  0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
  0xFF, 0xF0, 0xFF, 0xF0, 0x0F, 0xFC, 0xCC, 0x3C,
  0xCC, 0x33, 0xCC, 0xCF, 0xFF, 0xEF, 0xFF, 0xEE,
  0xFF, 0xFD, 0xFF, 0xFD, 0xDF, 0xFF, 0xBF, 0xFF,
  0xBB, 0xFF, 0xF7, 0xFF, 0xF7, 0x7F, 0x7B, 0xDE,
};

//
// Single global host state (one DW MMC controller instance)
//
STATIC DW_MMC_HOST  mHost;
STATIC UINT32       mChannel = MMC_CHANNEL_SD;

//
// Register accessor macros (operate on the active channel's IO base)
//
#define RD(R)   MmioRead32 (mHost.IoBase + (R))
#define WR(R,V) MmioWrite32 (mHost.IoBase + (R), (V))
#define SET(R,V) WR (R, RD (R) | (V))
#define CLR(R,V) WR (R, RD (R) & ~(V))

/**
  Reset controller blocks and wait for reset completion.

  @param[in] Mask   CTRL register reset bits (CTRL_RESET, FIFO_RESET, DMA_RESET).

  @return TRUE if reset completed, FALSE on timeout.
**/
STATIC
BOOLEAN
DwMmcResetCtrl (
  UINT32  Mask
  )
{
  UINT32  Timeout = 1000;

  SET (DWMCI_CTRL, Mask);
  while (Timeout--) {
    if (!(RD (DWMCI_CTRL) & Mask)) {
      return TRUE;
    }
    MicroSecondDelay (10);
  }
  DEBUG ((EFI_D_ERROR, "DWMMC: reset_ctrl timeout mask=0x%x\n", Mask));
  return FALSE;
}

/**
  Reset controller once, preserving CLKSEL across the reset.

  @param[in] Mask   CTRL register reset bits.

  @return TRUE if reset completed, FALSE on timeout.
**/
STATIC
BOOLEAN
DwMmcResetCtrlOnce (
  UINT32  Mask
  )
{
  UINT32  Timeout  = 1000;
  UINT32  ClkselReg;

  ClkselReg = RD (DWMCI_CLKSEL);
  CLR (DWMCI_CLKSEL, CLKSEL_SAMPLE_CLK_ALL);
  SET (DWMCI_CTRL, Mask);
  WR (DWMCI_RINTSTS, INT_ALL);
  while (Timeout--) {
    if (!(RD (DWMCI_CTRL) & Mask)) {
      break;
    }
    MicroSecondDelay (10);
  }
  WR (DWMCI_CLKSEL, ClkselReg);
  return (Timeout > 0);
}

/**
  Initialise FIFO threshold watermark values.
**/
STATIC
VOID
DwMmcFifoInit (
  VOID
  )
{
  UINT32  Val;
  UINT32  RxWmark;
  UINT32  TxWmark;
  UINT32  Threshold;

  if (!mHost.FifoDepth) {
    mHost.FifoDepth = 0x20;
  }

  Val       = RD (DWMCI_FIFOTH);
  Threshold = mHost.FifoDepth / 2;
  RxWmark   = ((Threshold - 1) << 16) & RX_WMARK;
  TxWmark   = Threshold & TX_WMARK;

  Val &= ~FIFOTH_ALL;
  Val |= (RxWmark | TxWmark | MSIZE_8);
  WR (DWMCI_FIFOTH, Val);
}

/**
  Wait for clock change to propagate through the controller.

  Issues a CMD_ONLY_CLK and polls until the command start bit clears.
**/
STATIC
EFI_STATUS
DwMmcCheckClockChange (
  VOID
  )
{
  UINT32  LoopCount;
  UINT32  Retry;

  WR (DWMCI_CMD, 0);
  WR (DWMCI_CMD, CMD_ONLY_CLK);
  for (Retry = 10; Retry > 0; Retry--) {
    for (LoopCount = 1000; LoopCount > 0; LoopCount--) {
      if (!(RD (DWMCI_CMD) & CMD_STRT_BIT)) {
        return EFI_SUCCESS;
      }
    }
    DwMmcResetCtrl (CTRL_RESET);
    WR (DWMCI_CMD, CMD_ONLY_CLK);
  }
  return EFI_TIMEOUT;
}

/**
  Enable or disable the card clock.

  @param[in] Val   CLK_ENABLE or CLK_DISABLE.
**/
STATIC
EFI_STATUS
DwMmcControlClken (
  UINT32  Val
  )
{
  WR (DWMCI_CLKENA, Val);
  return DwMmcCheckClockChange ();
}

/**
  Compute the clock divider for a target frequency.

  The DW MMC clock divider produces: f_card = f_board / (2 * div)
  where div >= 1.  div=0 bypasses the divider.

  @param[in] BoardClk    Input CIU clock frequency in Hz.
  @param[in] TargetClk   Desired output clock frequency in Hz.

  @return Divider value (0..255).
**/
STATIC
UINT32
DwMmcGetClockDiv (
  UINT32  BoardClk,
  UINT32  TargetClk
  )
{
  UINT32  i;

  if (TargetClk >= BoardClk) {
    return 0;
  }
  for (i = 1; i <= 0xFF; i++) {
    if (TargetClk >= (BoardClk / (2 * i))) {
      return i;
    }
  }
  return 0xFF;
}

/**
  Change the card clock frequency.

  @param[in] TargetClk   Desired clock frequency in Hz.

  @retval EFI_SUCCESS   Clock changed successfully.
  @retval EFI_TIMEOUT   Clock change timed out.
**/
STATIC
EFI_STATUS
DwMmcChangeClock (
  UINT32  TargetClk
  )
{
  UINT32  BoardHz;
  UINT32  Div;

  //
  // PhaseDivide is the CIU clock divider applied before the DWMMC block.
  //
  if (mHost.PhaseDivide > 0) {
    BoardHz = mHost.BusHz / mHost.PhaseDivide;
  } else {
    BoardHz = mHost.BusHz;
  }

  if (!BoardHz) {
    return EFI_DEVICE_ERROR;
  }

  DwMmcControlClken (CLK_DISABLE);

  Div = DwMmcGetClockDiv (BoardHz, TargetClk);
  WR (DWMCI_CLKDIV, Div);
  DwMmcCheckClockChange ();

  if (EFI_ERROR (DwMmcControlClken (CLK_ENABLE))) {
    return EFI_TIMEOUT;
  }

  mHost.CardClock = (Div == 0) ? BoardHz : (BoardHz / (2 * Div));
  return EFI_SUCCESS;
}

/**
  Change CLKSEL sample phase for tuning.

  @param[in] PassIndex   0..15 tuning pass index.
**/
STATIC
VOID
DwMmcChangeClksel (
  UINT32  PassIndex
  )
{
  UINT32  ClkselVal;
  UINT32  Reg;

  if (PassIndex > 15) {
    PassIndex = 0;
  }

  ClkselVal  = RD (DWMCI_CLKSEL);
  ClkselVal &= ~0x7FU;          // Clear sample[2:0] and tuning bit
  ClkselVal |= (PassIndex / 2); // Sample phase = pass_index / 2

  //
  // Phases 6-7 (sample >= 6) require the AXI sampling path select bit.
  //
  Reg = RD (DWMCI_AXI_BURST_LEN);
  if ((PassIndex / 2) >= 6) {
    Reg |= AXI_SAMPLING_PATH_SEL;
  } else {
    Reg &= ~AXI_SAMPLING_PATH_SEL;
  }
  WR (DWMCI_AXI_BURST_LEN, Reg);

  //
  // Fine-tune: odd pass indices set CLKSEL_TUNING_1.
  //
  if (PassIndex % 2 == 0) {
    ClkselVal &= ~CLKSEL_SAMPLE_CLK_TUNING_1;
  } else {
    ClkselVal |= CLKSEL_SAMPLE_CLK_TUNING_1;
  }

  WR (DWMCI_CLKSEL, ClkselVal);
}

/**
  Set controller I/O settings: clock, bus width, bus mode, CLKSEL.

  @param[in] Clock       Target clock frequency in Hz.
  @param[in] ClkselVal   CLKSEL register value (0 to use mHost.SdrClksel).

  @retval EFI_SUCCESS   IOS applied successfully.
**/
STATIC
EFI_STATUS
DwMmcSetIos (
  UINT32  Clock,
  UINT32  ClkselVal
  )
{
  UINT32  Reg;

  if (Clock < mHost.MinClock) {
    Clock = mHost.MinClock;
  }
  if (Clock > mHost.MaxClock) {
    Clock = mHost.MaxClock;
  }

  DwMmcChangeClock (Clock);

  //
  // Bus width
  //
  if (mHost.BusWidth == 4) {
    WR (DWMCI_CTYPE, CTYPE_4BIT);
  } else {
    WR (DWMCI_CTYPE, CTYPE_1BIT);
  }

  //
  // Bus mode: DDR or SDR.
  // SD cards normally use SDR. DDR50 would set UHS_REG_DDR here.
  //
  if (mHost.BusMode == 1) {
    SET (DWMCI_UHS_REG, UHS_REG_DDR);
  } else {
    CLR (DWMCI_UHS_REG, UHS_REG_DDR);
  }

  //
  // CLKSEL: timing parameters (drive strength, sample phase, divider).
  //
  Reg = ClkselVal ? ClkselVal : mHost.SdrClksel;

  //
  // At identification speeds (<= 400 kHz), drive CLK hard.
  //
  if (mHost.CardClock <= 400000) {
    Reg |= EXYNOS_CLKSEL_CCLK_DRIVE (7);
  }

  //
  // Apply tuned sample phase if tuning has been performed.
  //
  if (mHost.IsTuned) {
    Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
    if ((mHost.TunedSamplePhase & 0x7) >= 6) {
      SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
    } else {
      CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
    }
    if (mHost.IsFineTuned) {
      Reg |= CLKSEL_SAMPLE_CLK_TUNING_1;
    } else {
      Reg &= ~CLKSEL_SAMPLE_CLK_TUNING_1;
    }
  }

  WR (DWMCI_CLKSEL, Reg);

  if (!mHost.IsTuned) {
    CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
  }

  return EFI_SUCCESS;
}

/**
  Set IOS using the mode-appropriate CLKSEL value from host configuration.

  @param[in] Clock   Target clock frequency in Hz.
**/
STATIC
EFI_STATUS
DwMmcSetIosSimple (
  UINT32  Clock
  )
{
  UINT32  ClkselVal;

  if (mHost.BusMode == 1) {
    ClkselVal = mHost.DdrClksel;
  } else {
    ClkselVal = mHost.SdrClksel;
  }

  return DwMmcSetIos (Clock, ClkselVal);
}

/**
  Poll until the DATA_BUSY status bit clears.

  @param[in] CmdIdx   Command index being checked (CMD13 is excluded).

  @retval EFI_SUCCESS   Data line is idle.
  @retval EFI_TIMEOUT   Data line stayed busy.
**/
STATIC
EFI_STATUS
DwMmcCheckDataBusy (
  UINT32  CmdIdx
  )
{
  UINT32  Timeout;

  if (CmdIdx == CMD13_SEND_STATUS) {
    return EFI_SUCCESS;
  }

  for (Timeout = 1000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_STATUS) & DATA_BUSY)) {
      WR (DWMCI_RINTSTS, INT_ALL);
      return EFI_SUCCESS;
    }
    MicroSecondDelay (1000);
  }
  DEBUG ((EFI_D_ERROR, "DWMMC: data busy timeout\n"));
  return EFI_TIMEOUT;
}

/**
  Populate one IDMAC descriptor entry.

  @param[out] Desc         Descriptor to fill.
  @param[in]  Control      DES0 control flags.
  @param[in]  BufferSize   Transfer size in bytes for this descriptor.
  @param[in]  BufferAddr   Physical buffer address.
**/
STATIC
VOID
DwMmcSetIdmaDesc (
  DWMMC_IDMAC_DESC  *Desc,
  UINT64             Control,
  UINT64             BufferSize,
  UINT64             BufferAddr
  )
{
  Desc->Des0  = (UINT32)Control;
  Desc->Des1  = 0;
  Desc->Des2  = (UINT32)BufferSize;
  Desc->Des3  = 0;
  Desc->Des4  = (UINT32)BufferAddr;
  Desc->Des5  = (UINT32)(BufferAddr >> 32);
  Desc->Des6  = (UINT32)((UINTN)(Desc + 1));
  Desc->Des7  = (UINT32)(((UINTN)(Desc + 1)) >> 32);
  Desc->Des8  = 0;
  Desc->Des9  = 0;
  Desc->Des10 = 0;
  Desc->Des11 = 0;
  Desc->Des12[0] = 0;
  Desc->Des12[1] = 0;
  Desc->Des12[2] = 0;
  Desc->Des12[3] = 0;
}

/**
  Prepare the IDMAC descriptor chain and configure DMA for a data transfer.

  Each descriptor carries at most 8 blocks (4 KB).  The descriptor chain
  is flushed from the data cache before the DMA engine reads it.

  @param[in] BlockSize   Bytes per block.
  @param[in] BlockCnt    Number of blocks.
  @param[in] IsWrite     TRUE = write to card, FALSE = read from card.
  @param[in] Buf         Data buffer (virtual address).
**/
STATIC
VOID
DwMmcPrepareData (
  UINT32   BlockSize,
  UINT32   BlockCnt,
  BOOLEAN  IsWrite,
  VOID    *Buf
  )
{
  DWMMC_IDMAC_DESC  *Desc;
  UINT32             Flags;
  UINT32             i;
  UINT32             DataCnt;
  UINT32             SendBytes;
  UINT32             Reg;

  //
  // Set MSIZE based on block size.  RPMB (block_size=8) uses MSIZE_1.
  //
  Reg = RD (DWMCI_FIFOTH);
  Reg &= ~MSIZE_MASK;
  if (BlockSize == 8) {
    Reg |= MSIZE_1;
  } else {
    Reg |= MSIZE_8;
  }
  WR (DWMCI_FIFOTH, Reg);

  DwMmcResetCtrl (FIFO_RESET);
  SET (DWMCI_BMOD, BMOD_IDMAC_RESET);

  ZeroMem (mDmaDesc, sizeof (mDmaDesc));

  mDmaBlockSize = BlockSize;
  mDmaBlockCnt  = BlockCnt;
  mDmaIsRead    = !IsWrite;
  mDmaBuffer    = Buf;

  Desc    = mDmaDesc;
  DataCnt = BlockCnt;

  for (i = 0; ; i++) {
    Flags = DWMCI_IDMAC_OWN | DWMCI_IDMAC_CH;
    if (i == 0) {
      Flags |= DWMCI_IDMAC_FS;
    }

    if (DataCnt <= 8) {
      Flags     |= DWMCI_IDMAC_LD;
      SendBytes  = BlockSize * DataCnt;
      DwMmcSetIdmaDesc (Desc, Flags, SendBytes,
                        (UINT64)(UINTN)Buf + (UINT64)(i * 0x1000));
      break;
    }

    SendBytes = BlockSize * 8;
    DwMmcSetIdmaDesc (Desc, Flags, SendBytes,
                      (UINT64)(UINTN)Buf + (UINT64)(i * 0x1000));
    DataCnt -= 8;
    Desc++;
  }

  //
  // Flush descriptor table so DMA sees the new entries.
  //
  WriteBackDataCacheRange (mDmaDesc, sizeof (mDmaDesc));

  //
  // For writes: clean data cache so DMA reads correct data.
  // For reads:  invalidate cache so DMA writes into clean lines.
  //
  if (IsWrite) {
    WriteBackDataCacheRange (Buf, BlockSize * BlockCnt);
  } else {
    InvalidateDataCacheRange (Buf, BlockSize * BlockCnt);
  }

  WR (DWMCI_DBADDRL, (UINT32)(UINTN)mDmaDesc);
  WR (DWMCI_DBADDRU, (UINT32)((UINTN)mDmaDesc >> 32));

  CLR (DWMCI_CTRL, SEND_AS_CCSD);
  SET (DWMCI_CTRL, ENABLE_IDMAC | DMA_ENABLE);
  SET (DWMCI_BMOD, BMOD_IDMAC_ENABLE | BMOD_IDMAC_FB);

  WR (DWMCI_BLKSIZ, BlockSize);
  WR (DWMCI_BYTCNT, BlockSize * BlockCnt);
}

/**
  Build the CMD register flag value for a given command.

  @param[in]  CmdIdx     Command index (0..63).
  @param[in]  Arg        Command argument.
  @param[in]  RespType   Response type flags (MMC_RSP_*).
  @param[in]  HasData    TRUE if data transfer follows.
  @param[in]  IsWrite    TRUE for write transfers.

  @return CMD register flags.
**/
STATIC
UINT32
DwMmcReadyCmd (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  BOOLEAN  HasData,
  BOOLEAN  IsWrite
  )
{
  UINT32  Flag = 0;

  WR (DWMCI_CMDARG, Arg);

  if (RespType & MMC_RSP_CRC) {
    Flag |= CMD_CHECK_CRC_BIT;
  }
  if (RespType & MMC_RSP_PRESENT) {
    Flag |= CMD_RESP_EXP_BIT;
    if (RespType & MMC_RSP_136) {
      Flag |= CMD_RESP_LENGTH_BIT;
    }
  }

  Flag |= (CmdIdx | CMD_STRT_BIT | CMD_USE_HOLD_REG | CMD_WAIT_PRV_DAT_BIT);

  if (HasData) {
    Flag |= CMD_DATA_EXP_BIT;
    if (IsWrite) {
      Flag |= CMD_RW_BIT;
    }
  }

  return Flag;
}

/**
  Start a command on the controller and wait for completion.

  Checks for response timeout, response error, and CRC error.

  @param[in] Flag   CMD register flags (from DwMmcReadyCmd).

  @retval EFI_SUCCESS        Command completed.
  @retval EFI_TIMEOUT        No response or CMD_DONE timeout.
  @retval EFI_DEVICE_ERROR   Response error (RE).
  @retval EFI_CRC_ERROR      Response CRC error (RCRC).
**/
STATIC
EFI_STATUS
DwMmcStartCmd (
  UINT32  Flag
  )
{
  UINT32  Mask;
  UINT32  Timeout;

  //
  // Wait for previous command to finish.
  //
  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_CMD) & CMD_STRT_BIT) &&
        !(RD (DWMCI_RINTSTS) & INT_CDONE)) {
      break;
    }
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: prev cmd not cleared\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  WR (DWMCI_RINTSTS, CMD_STATUS);
  WR (DWMCI_CMD, Flag);

  //
  // Wait for CMD_STRT_BIT to clear.
  //
  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_CMD) & CMD_STRT_BIT)) {
      break;
    }
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: cmd start bit stuck\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  //
  // Wait for FSM to leave busy state.
  //
  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_STATUS) & CMD_FSMSTAT)) {
      break;
    }
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: FSM stuck\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  //
  // Wait for CMD_DONE interrupt.
  //
  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    Mask = RD (DWMCI_RINTSTS);
    if (Mask & INT_CDONE) {
      break;
    }
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD_DONE timeout\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  if (Mask & INT_RTO) {
    DEBUG ((EFI_D_ERROR, "DWMMC: response timeout\n"));
    return EFI_TIMEOUT;
  }
  if (Mask & INT_RE) {
    DEBUG ((EFI_D_ERROR, "DWMMC: response error\n"));
    return EFI_DEVICE_ERROR;
  }
  if (Mask & INT_RCRC) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CRC error\n"));
    return EFI_CRC_ERROR;
  }

  WR (DWMCI_RINTSTS, Mask & CMD_STATUS);

  return EFI_SUCCESS;
}

/**
  Parse controller response registers into OutResp array.

  @param[in]  RespType   Response type (MMC_RSP_136 for 136-bit).
  @param[out] OutResp    Buffer for response (4 x UINT32 for R2, 1 x UINT32 for R1/R3/R6/R7).
**/
STATIC
VOID
DwMmcResponseParse (
  UINT32   RespType,
  UINT32  *OutResp
  )
{
  if (!(RespType & MMC_RSP_PRESENT)) {
    return;
  }
  if (!OutResp) {
    return;
  }

  if (RespType & MMC_RSP_136) {
    OutResp[0] = RD (DWMCI_RESP3);
    OutResp[1] = RD (DWMCI_RESP2);
    OutResp[2] = RD (DWMCI_RESP1);
    OutResp[3] = RD (DWMCI_RESP0);
  } else {
    OutResp[0] = RD (DWMCI_RESP0);
  }
}

/**
  Disable DMA and reset FIFO/DMA after a transfer completes.
**/
STATIC
VOID
DwMmcEndData (
  VOID
  )
{
  UINT32  Reg;

  CLR (DWMCI_CTRL, (DMA_ENABLE | ENABLE_IDMAC));
  DwMmcResetCtrl (DMA_RESET);
  DwMmcResetCtrl (FIFO_RESET);

  Reg  = RD (DWMCI_BMOD);
  Reg &= ~(BMOD_IDMAC_FB | BMOD_IDMAC_ENABLE);
  Reg |= BMOD_IDMAC_RESET;
  WR (DWMCI_BMOD, Reg);

  WR (DWMCI_IDINTEN, 0);
}

/**
  Simple error recovery: reset controller, clear interrupts, restart clock.
**/
STATIC
VOID
DwMmcSimpleRecovery (
  VOID
  )
{
  UINT32  Timeout;

  DwMmcResetCtrl (CTRL_RESET);
  DwMmcResetCtrl (FIFO_RESET);

  WR (DWMCI_RINTSTS, INT_ALL);
  WR (DWMCI_INTMSK,  0);

  DwMmcControlClken (CLK_DISABLE);

  WR (DWMCI_CMD, 0);
  WR (DWMCI_CMD, CMD_ONLY_CLK);
  for (Timeout = 1000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_CMD) & CMD_STRT_BIT)) {
      break;
    }
  }

  WR (DWMCI_CMD, RD (DWMCI_CMD) & ~CMD_SEND_CLK_ONLY);
  DwMmcControlClken (CLK_ENABLE);

  //
  // Abort any in-progress data transfer.
  //
  if (mHost.CardRca) {
    DwMmcSendCommand (CMD12_STOP_TRAN, 0,
      MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY, NULL);
  }
}

/**
  Poll the DMA engine until the data transfer completes.

  @retval EFI_SUCCESS        Transfer completed successfully.
  @retval EFI_DEVICE_ERROR   Data error detected.
  @retval EFI_TIMEOUT        Transfer timed out.
**/
STATIC
EFI_STATUS
DwMmcDataTransfer (
  VOID
  )
{
  UINT32  Mask;
  UINT32  Timeout = 2000000;

  while (Timeout--) {
    Mask = RD (DWMCI_RINTSTS);

    if (Mask & (DATA_ERR | DATA_TOUT)) {
      DwMmcSimpleRecovery ();
      DwMmcEndData ();
      DEBUG ((EFI_D_ERROR, "DWMMC: data error RINTSTS=0x%08x\n", Mask));
      return EFI_DEVICE_ERROR;
    }

    if (Mask & INT_DTO) {
      UINT32  DmaTimeout = 50000;
      while (!(RD (DWMCI_IDSTS) & IDSTS_NIS) && DmaTimeout--) {
        MicroSecondDelay (1);
      }

      DwMmcEndData ();
      //
      // For reads: invalidate cache AFTER DMA has written to memory.
      //
      if (mDmaIsRead) {
        InvalidateDataCacheRange (mDmaBuffer, mDmaBlockSize * mDmaBlockCnt);
      }
      return EFI_SUCCESS;
    }

    MicroSecondDelay (1);
  }

  DEBUG ((EFI_D_ERROR, "DWMMC: data transfer sw timeout\n"));
  DwMmcEndData ();
  return EFI_TIMEOUT;
}

/**
  Send a command with optional data transfer.

  @param[in]  CmdIdx     Command index.
  @param[in]  Arg        Command argument.
  @param[in]  RespType   Response type flags.
  @param[out] OutResp    Response buffer (may be NULL).
  @param[in]  IsWrite    TRUE for data-out (card write).
  @param[in]  BlockSize  Data block size in bytes (0 if no data).
  @param[in]  BlockCnt   Number of blocks (0 if no data).
  @param[in]  Buf        Data buffer (may be NULL if no data).

  @retval EFI_SUCCESS   Command and optional data transfer succeeded.
**/
STATIC
EFI_STATUS
DwMmcSendCommandData (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp,
  BOOLEAN  IsWrite,
  UINT32   BlockSize,
  UINT32   BlockCnt,
  VOID    *Buf
  )
{
  EFI_STATUS  Status;
  UINT32      Flag;

  Status = DwMmcCheckDataBusy (CmdIdx);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (BlockCnt) {
    DwMmcPrepareData (BlockSize, BlockCnt, IsWrite, Buf);
  }

  Flag   = DwMmcReadyCmd (CmdIdx, Arg, RespType, BlockCnt > 0, IsWrite);
  Status = DwMmcStartCmd (Flag);
  if (EFI_ERROR (Status)) {
    if (BlockCnt) {
      DwMmcEndData ();
    }
    return Status;
  }

  DwMmcResponseParse (RespType, OutResp);

  if (BlockCnt) {
    Status = DwMmcDataTransfer ();
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  //
  // For SD cards, issue CMD12 (STOP_TRANSMISSION) after multi-block
  // transfers when CMD23 (SET_BLOCK_COUNT) was not used.
  //
  if ((BlockCnt > 1) && (mHost.CardType != CARD_TYPE_MMC)) {
    UINT32  StopFlag;
    StopFlag = DwMmcReadyCmd (CMD12_STOP_TRAN, 0,
                 MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY,
                 FALSE, FALSE);
    DwMmcStartCmd (StopFlag);
  }

  return EFI_SUCCESS;
}

/**
  Send a command without data transfer.

  @param[in]  CmdIdx     Command index.
  @param[in]  Arg        Command argument.
  @param[in]  RespType   Response type flags.
  @param[out] OutResp    Response buffer (may be NULL).

  @retval EFI_SUCCESS   Command succeeded.
**/
STATIC
EFI_STATUS
DwMmcSendCommand (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp
  )
{
  return DwMmcSendCommandData (CmdIdx, Arg, RespType, OutResp,
                               FALSE, 0, 0, NULL);
}

/**
  Send CMD55 (APP_CMD) prefix for SD application commands.

  @param[in] Rca   Relative Card Address (shifted left 16 bits in argument).
**/
STATIC
EFI_STATUS
DwMmcSendAppCmd (
  UINT32  Rca
  )
{
  UINT32  Resp[4];
  return DwMmcSendCommand (CMD55_APP_CMD, Rca << 16,
           MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
}

/**
  Poll CMD13 (SEND_STATUS) until the card is ready.

  @param[in]  TimeoutMs   Maximum time to wait in milliseconds.
  @param[out] OutState    Card state (0=idle, 3=stby, 4=tran, ...).

  @retval EFI_SUCCESS        Card is ready.
  @retval EFI_TIMEOUT        Card did not become ready.
  @retval EFI_DEVICE_ERROR   Card reported an error.
**/
STATIC
EFI_STATUS
DwMmcGetCardStatus (
  UINT32   TimeoutMs,
  UINT32  *OutState
  )
{
  EFI_STATUS  Status;
  UINT32      Resp[4];

  do {
    Status = DwMmcSendCommand (CMD13_SEND_STATUS, mHost.CardRca << 16,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (!EFI_ERROR (Status)) {
      if (Resp[0] & R1_STATUS_MASK) {
        DEBUG ((EFI_D_ERROR, "DWMMC: CMD13 err 0x%08x\n", Resp[0]));
        return EFI_DEVICE_ERROR;
      }
      if ((Resp[0] & R1_READY_FOR_DATA) &&
          ((Resp[0] & R1_CURRENT_STATE_MASK) != R1_STATE_PRG)) {
        *OutState = (Resp[0] & R1_CURRENT_STATE_MASK) >> 9;
        return EFI_SUCCESS;
      }
    }
    MicroSecondDelay (1000);
  } while (TimeoutMs--);

  return EFI_TIMEOUT;
}

/**
  Invert byte order (endian swap) of a 32-bit value.

  Used for SCR register parsing (SCR is stored big-endian in little-endian memory).
**/
STATIC
UINT32
ChangeEndian (
  UINT32  Val
  )
{
  return ((Val & 0xFF) << 24) | ((Val & 0xFF00) << 8) |
         ((Val >> 8) & 0xFF00) | ((Val >> 24) & 0xFF);
}

/**
  Extract a bitfield from an array of 32-bit words.

  @param[in] Value   Pointer to array of UINT32.
  @param[in] Start   Starting bit position (inclusive).
  @param[in] End     Ending bit position (inclusive).

  @return Extracted bitfield value.
**/
STATIC
UINT32
DwMmcExtractBits (
  UINT32  *Value,
  UINT32   Start,
  UINT32   End
  )
{
  UINT32  Idx1 = Start / 32;
  UINT32  Off  = Start % 32;
  UINT32  Ret  = Value[Idx1] >> Off;
  UINT32  Idx2 = End / 32;
  UINT32  Mask = (1 << (End - Start + 1)) - 1;

  if (Idx2 > Idx1) {
    Ret |= (Value[Idx2] << (32 - Off));
  }
  return Ret & Mask;
}

/**
  Send SD_SWITCH_FUNC (CMD6) to the SD card.

  Used to query supported modes (mode=CHECK, group=0) or switch
  access mode (mode=SWITCH, group=1).

  @param[in]  Mode   0 = check, 1 = switch.
  @param[in]  Group  Function group (0=all, 1=access_mode, 2=cmd_system,
                     3=driver_strength, 4=current_limit).
  @param[in]  Value  Function value for the group (0xF = keep current).
  @param[out] Resp   64-byte switch status buffer.

  @retval EFI_SUCCESS   Switch command executed.
**/
STATIC
EFI_STATUS
DwMmcSdSwitchCmd (
  UINT32  Mode,
  UINT32  Group,
  UINT8   Value,
  UINT8  *Resp
  )
{
  UINT32  Arg;

  Arg  = (Mode << 31) | 0x00FFFFFF;
  Arg &= ~(0xF << (Group * 4));
  Arg |= (Value << (Group * 4));

  return DwMmcSendCommandData (CMD6_SWITCH_FUNC, Arg,
           MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE | MMC_RSP_BUSY,
           NULL, FALSE, 64, 1, Resp);
}

/**
  Execute the sampling clock tuning procedure.

  Sends CMD19 (SD) repeatedly, adjusting the sample phase each time
  (16-phase sweep), and picks the optimal phase from the longest run
  of consecutive passing phases.

  @param[in] CmdIdx   CMD19_SEND_TUNING (for SD).

  @retval EFI_SUCCESS        Tuning succeeded.
  @retval EFI_DEVICE_ERROR   No phase passed tuning.
**/
STATIC
EFI_STATUS
DwMmcExecuteTuning (
  UINT32  CmdIdx
  )
{
  EFI_STATUS    Status;
  UINT32        ClkselBase;
  UINT16        GoodBitmap;
  UINT32        GoodCount;
  UINT32        BestStart;
  UINT32        BestLen;
  UINT32        Start;
  UINT32        Len;
  UINT32        Median;
  UINT32        PassIndex;

  ClkselBase = RD (DWMCI_CLKSEL);
  GoodBitmap = 0;

  //
  // Clear sample timing for tuning start.
  //
  WR (DWMCI_CLKSEL, ClkselBase & ~0xFFU);
  CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
  WR (DWMCI_CARDTHRCTL, (TUNING_BLOCK_SIZE << 16) | 1);

  DEBUG ((EFI_D_WARN, "DWMMC: Tuning start (CMD%d), clksel=0x%08x\n",
    CmdIdx, ClkselBase));

  //
  // 16-phase sweep (pass_index 0..15).
  //
  for (PassIndex = 0; PassIndex <= 15; PassIndex++) {
    DwMmcChangeClksel (PassIndex);

    ZeroMem (mTuningBuf, sizeof (mTuningBuf));
    Status = DwMmcSendCommandData (CmdIdx, 0,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, FALSE, TUNING_BLOCK_SIZE, 1, mTuningBuf);

    if (!EFI_ERROR (Status)) {
      if (CompareMem (mTuningBuf, (VOID *)mTuningPattern4Bit,
                      TUNING_BLOCK_SIZE) == 0) {
        GoodBitmap |= (1 << PassIndex);
      }
    }
  }

  WR (DWMCI_CARDTHRCTL, 0);
  CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);

  //
  // Find the longest run of consecutive good phases.
  //
  if (GoodBitmap == 0xFFFF) {
    BestStart = 0;
    BestLen   = 16;
    DEBUG ((EFI_D_WARN, "DWMMC: all 16 phases pass\n"));
    goto TuningCalcMedian;
  }

  {
    UINT32  GoodBitmap32 = GoodBitmap | ((UINT32)GoodBitmap << 16);
    UINT32  Phase32;

    BestStart = 0;
    BestLen   = 0;
    Start     = 0;
    Len       = 0;

    for (Phase32 = 0; Phase32 < 32; Phase32++) {
      if (GoodBitmap32 & (1U << Phase32)) {
        if (Len == 0) {
          Start = Phase32;
        }
        Len++;
      } else {
        if (Len > BestLen) {
          BestLen   = Len;
          BestStart = Start;
        }
        Len = 0;
      }
    }
    if (Len > BestLen) {
      BestLen   = Len;
      BestStart = Start;
    }
  }

TuningCalcMedian:
  GoodCount = BestLen;
  {
    INT32  MaskNum[8] = { 13, 11, 9, 7, 5, 4, 3, 0 };
    INT32  MaskBit[8] = {  9,  7, 6, 5, 3, 2, 1, 0 };
    INT32  i;
    INT32  Offset = 0;

    for (i = 0; i < 8; i++) {
      if ((INT32)GoodCount >= MaskNum[i]) {
        Offset = MaskBit[i];
        break;
      }
    }
    Median = (BestStart + (UINT32)Offset) % 16;
  }

  DEBUG ((EFI_D_WARN,
    "DWMMC: Tuning result: bitmap=0x%04x, best=%d-%d (%d phases), median=%d\n",
    GoodBitmap, BestStart, BestStart + GoodCount - 1, GoodCount, Median));

  if (GoodCount == 0) {
    DEBUG ((EFI_D_ERROR, "DWMMC: Tuning failed -- no good phases\n"));
    return EFI_DEVICE_ERROR;
  }

  //
  // Decode 16-phase result to 8-bit sample phase + fine-tune flag.
  //
  mHost.TunedSamplePhase = Median / 2;
  mHost.IsFineTuned      = (Median % 2) ? TRUE : FALSE;
  mHost.IsTuned          = TRUE;

  DEBUG ((EFI_D_WARN, "DWMMC: Tuning selected phase=%d fine_tune=%d\n",
    mHost.TunedSamplePhase, mHost.IsFineTuned));

  return EFI_SUCCESS;
}

/**
  Execute the voltage switch sequence for UHS-I support.

  Stops the clock, switches the regulator to 1.8V via the platform
  library, updates the UHS_REG, and restarts the clock.

  @retval EFI_SUCCESS   Voltage switch completed.
**/
STATIC
EFI_STATUS
DwMmcVoltageSwitch (
  VOID
  )
{
  EFI_STATUS  Status;
  UINT32      Timeout;

  //
  // Wait for DATA_BUSY to assert (CMD11 completed).
  //
  for (Timeout = 20; Timeout > 0; Timeout--) {
    if (RD (DWMCI_STATUS) & DATA_BUSY) {
      break;
    }
    MicroSecondDelay (1000);
  }
  if (!Timeout) {
    DEBUG ((EFI_D_WARN,
      "DWMMC: Voltage switch: DATA_BUSY not set, continuing\n"));
  }

  //
  // Stop clock before voltage change.
  //
  WR (DWMCI_CLKENA, CLK_DISABLE);
  MicroSecondDelay (10000);

  //
  // Switch LDO to 1.8V.
  //
  Status = SdVoltageSwitch ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: SdVoltageSwitch failed: %r\n", Status));
    WR (DWMCI_CLKENA, CLK_ENABLE);
    return Status;
  }

  SET (DWMCI_UHS_REG, UHS_REG_V18);

  //
  // Update MaxClock for 1.8V signalling.
  //
  {
    UINT32  BoardHz;
    BoardHz = (mHost.PhaseDivide > 0) ? (mHost.BusHz / mHost.PhaseDivide)
                                      : mHost.BusHz;
    mHost.MaxClock = (BoardHz < 200000000) ? BoardHz : 200000000;
  }

  MicroSecondDelay (10000); // LDO stabilisation

  WR (DWMCI_CLKENA, CLK_ENABLE);
  MicroSecondDelay (1000);

  //
  // Wait for DAT[3:0] to be driven high (DAT_BUSY de-asserted).
  //
  for (Timeout = 100; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_STATUS) & DATA_BUSY)) {
      break;
    }
    MicroSecondDelay (100);
  }
  if (!Timeout) {
    DEBUG ((EFI_D_WARN,
      "DWMMC: Voltage switch -- DAT0 still busy after 1.8V switch\n"));
  }

  DEBUG ((EFI_D_WARN, "DWMMC: Voltage switch to 1.8V complete, MaxClock=%d MHz\n",
    mHost.MaxClock / 1000000));
  return EFI_SUCCESS;
}

/**
  Initialise the DW MMC host controller registers.

  Configures SMU (if secure), reset, FIFO, interrupts, CLKSEL,
  clock, debounce, card type, bus mode, and timeouts.
**/
STATIC
EFI_STATUS
DwMmcHostInit (
  VOID
  )
{
  //
  // SMU / FMP configuration (eMMC only; SD channel is not secured).
  //
  if (mHost.Secure) {
    WR (DWMCI_FMPSBEGIN0, 0);
    WR (DWMCI_FMPSEND0,   0xFFFFFFFF);
    WR (DWMCI_FMPSCTRL0,
      MPSCTRL_SECURE_READ_BIT | MPSCTRL_SECURE_WRITE_BIT |
      MPSCTRL_NON_SECURE_READ_BIT | MPSCTRL_NON_SECURE_WRITE_BIT |
      MPSCTRL_VALID);
    if (mHost.MpsSecurity) {
      WR (DWMCI_FMPSECURITY, mHost.MpsSecurity);
    }
    DEBUG ((EFI_D_INFO, "DWMMC: SMU configured (mps_security=0x%08x)\n",
      mHost.MpsSecurity));
  }

  mHost.Version = RD (DWMCI_VERID) & 0xFFFF;

  WR (DWMCI_PWREN, POWER_ENABLE);
  DwMmcCheckDataBusy (0);

  MicroSecondDelay (10000);

  DwMmcResetCtrlOnce (RESET_ALL);
  DwMmcFifoInit ();

  WR (DWMCI_RINTSTS, INT_ALL);
  WR (DWMCI_INTMSK,  0);

  WR (DWMCI_CLKSEL, mHost.SdrClksel);
  DwMmcSetIosSimple (400000);

  mHost.MinClock = 400000;
  mHost.MaxClock = 50000000;

  SET (DWMCI_CTRL, SEND_AS_CCSD);

  WR (DWMCI_DEBNCE, 0xFFFFF);
  WR (DWMCI_CTYPE,  CTYPE_1BIT);
  WR (DWMCI_BMOD,   BMOD_IDMAC_RESET);
  WR (DWMCI_IDINTEN, 0);
  WR (DWMCI_TMOUT,  0xFFFFFFFF);

  return EFI_SUCCESS;
}

/**
  Reset card state and host controller to identification mode.
**/
STATIC
EFI_STATUS
DwMmcSetInitState (
  VOID
  )
{
  mHost.CardRca          = 0;
  mHost.BlockSize        = MMC_MAX_BLOCK_LEN;
  mHost.BusWidth         = 1;
  mHost.BusMode          = 0;   // SDR
  mHost.CardClock        = 400000;
  mHost.CardPresent      = FALSE;
  mHost.IsSdhc           = FALSE;
  mHost.IsHighSpeed      = FALSE;
  mHost.InitDone         = FALSE;
  mHost.TunedSamplePhase = 0;
  mHost.IsTuned          = FALSE;
  mHost.IsFineTuned      = FALSE;

  DwMmcHostInit ();
  return DwMmcSetIosSimple (400000);
}

/**
  Initialise an SD card: CMD0 -> CMD8 -> ACMD41 -> (CMD11) -> CMD2 -> CMD3.

  @retval EFI_SUCCESS        SD card initialised and in identification state.
  @retval EFI_DEVICE_ERROR   Card did not respond correctly.
  @retval EFI_TIMEOUT        Card did not become ready.
**/
STATIC
EFI_STATUS
DwMmcSdInitCard (
  VOID
  )
{
  EFI_STATUS  Status;
  UINT32      Resp[4];
  UINT32      OcrArg;
  UINT32      i;

  //
  // CMD0: GO_IDLE_STATE (reset all cards).
  //
  Status = DwMmcSendCommand (CMD0_GO_IDLE, 0, 0, NULL);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  MicroSecondDelay (2000);

  //
  // CMD8: SEND_IF_COND (verify voltage range and check for SDHC/SDXC).
  //
  for (i = 0; i < 3; i++) {
    Status = DwMmcSendCommand (CMD8_SEND_IF_COND, SD_SEND_IF_COND_ARG,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (Status == EFI_SUCCESS) {
      if ((Resp[0] & 0xFFF) != 0x1AA) {
        return EFI_DEVICE_ERROR;
      }
      OcrArg = SD_HC_OCR;   // SDHC/SDXC capable
      break;
    }
    MicroSecondDelay (1000);
  }

  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_WARN, "DWMMC: CMD8 failed, trying SDSC (v1.x card)\n"));
    OcrArg = SD_OCR;
  }

  //
  // ACMD41: SD_SEND_OP_COND (wait for card to finish power-up).
  //
  for (i = 0; i < 20; i++) {
    Status = DwMmcSendAppCmd (0);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    Status = DwMmcSendCommand (ACMD41_SD_SEND_OP, OcrArg,
               MMC_RSP_PRESENT, Resp);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    if (Resp[0] & OCR_BUSY) {
      mHost.OcR      = Resp[0];
      mHost.CardType = (Resp[0] & OCR_HCS) ? CARD_TYPE_SDHC : CARD_TYPE_SD;
      mHost.IsSdhc   = (mHost.CardType == CARD_TYPE_SDHC);
      mHost.IsUhsSupported = (Resp[0] & OCR_S18R) ? TRUE : (OcrArg == SD_HC_OCR);
      break;
    }
    MicroSecondDelay (1000);
  }

  if (i >= 20) {
    return EFI_TIMEOUT;
  }

  //
  // CMD11: VOLTAGE_SWITCH (request 1.8V signalling for UHS-I).
  //
  if (mHost.IsUhsSupported) {
    DEBUG ((EFI_D_WARN, "DWMMC: CMD11 voltage switch...\n"));
    Status = DwMmcSendCommand (CMD11_SWITCH_VOLTAGE, 0,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN,
        "DWMMC: CMD11 timed out (%r), forcing voltage switch (Exynos quirk)\n",
        Status));
    } else if ((Resp[0] & R1_ERROR)) {
      DEBUG ((EFI_D_WARN,
        "DWMMC: CMD11 R1 error (0x%08x), UHS modes unavailable\n", Resp[0]));
      mHost.IsUhsSupported = FALSE;
    }

    if (mHost.IsUhsSupported) {
      DEBUG ((EFI_D_WARN, "DWMMC: Executing voltage switch sequence...\n"));
      Status = DwMmcVoltageSwitch ();
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_WARN,
          "DWMMC: Voltage switch failed (%r), UHS modes unavailable\n", Status));
        mHost.IsUhsSupported = FALSE;
      }
    }
  }

  DEBUG ((EFI_D_WARN,
    "DWMMC: SD card ready, OCR=0x%08x, SDHC=%d, UHS=%d\n",
    mHost.OcR, mHost.IsSdhc, mHost.IsUhsSupported));
  return EFI_SUCCESS;
}

/**
  Complete card identification: CMD2 (CID) -> CMD3 (RCA) -> CMD9 (CSD) -> CMD7 (select).

  @retval EFI_SUCCESS   Card identified and selected.
**/
STATIC
EFI_STATUS
DwMmcIdentifyCard (
  VOID
  )
{
  EFI_STATUS  Status;
  UINT32      Resp[4];
  UINT32      Csd[4];

  //
  // CMD2: ALL_SEND_CID.
  //
  Status = DwMmcSendCommand (CMD2_ALL_SEND_CID, 0,
             MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC, Resp);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD2 failed\n"));
    return Status;
  }

  //
  // CMD3: SEND_RELATIVE_ADDR (get RCA).
  //
  Status = DwMmcSendCommand (CMD3_SEND_REL_ADDR, 0,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD3 failed\n"));
    return Status;
  }

  mHost.CardRca = (Resp[0] >> 16) & 0xFFFF;
  DEBUG ((EFI_D_WARN, "DWMMC: RCA=0x%04x\n", mHost.CardRca));

  //
  // CMD9: SEND_CSD.
  //
  Status = DwMmcSendCommand (CMD9_SEND_CSD, mHost.CardRca << 16,
             MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC, Csd);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD9 failed\n"));
    return Status;
  }

  //
  // CSD is transferred MSB-first; reverse the 128-bit word order.
  //
  mHost.Csd[0] = Csd[3];
  mHost.Csd[1] = Csd[2];
  mHost.Csd[2] = Csd[1];
  mHost.Csd[3] = Csd[0];

  //
  // CID parsing.
  //
  {
    UINT32  CidVal[4];
    CidVal[0] = Resp[3];
    CidVal[1] = Resp[2];
    CidVal[2] = Resp[1];
    CidVal[3] = Resp[0];

    {
      UINT8   Mmid  = DwMmcExtractBits (CidVal, 120, 127);
      UINT32  Psn   = DwMmcExtractBits (CidVal, 8, 31);
      UINT16  Oid   = DwMmcExtractBits (CidVal, 104, 119);
      UINT8   Prv   = DwMmcExtractBits (CidVal, 96, 103);
      UINT8   Month = DwMmcExtractBits (CidVal, 0, 3);
      UINT8   Year  = DwMmcExtractBits (CidVal, 4, 7) + 2000;

      DEBUG ((EFI_D_WARN,
        "DWMMC: SD CID -- MID=0x%02x OID=%c%c PRV=%d.%d PSN=0x%08x %04d-%02d\n",
        Mmid, (Oid >> 8) & 0xFF, Oid & 0xFF,
        (Prv >> 4) & 0xF, Prv & 0xF, Psn, Year, Month));
    }
  }

  //
  // CMD7: SELECT_CARD (move to TRAN state).
  //
  Status = DwMmcSendCommand (CMD7_SELECT_CARD, mHost.CardRca << 16,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY, Resp);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD7 failed\n"));
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Decode SD card information: capacity from CSD, SCR, switch capabilities.

  Fills in mHost.Capacity, mHost.NumBlocks, mHost.Scr, mHost.CardCaps.
**/
STATIC
EFI_STATUS
DwMmcDecodeSdInfo (
  VOID
  )
{
  EFI_STATUS  Status;
  UINT64      CSize;
  UINT64      CMult;
  UINT32      FuncGroup1;
  UINT32      CsdVer;
  UINT8       SwitchStatus[64];
  UINT8       ScrData[8];

  mHost.Capacity  = 0;
  mHost.BlockSize = MMC_MAX_BLOCK_LEN;

  CsdVer = mHost.Csd[3] >> 30;

  if (CsdVer >= 1) {
    //
    // SDHC/SDXC (CSD v2.0): 22-bit C_SIZE.
    // MemoryCapacity = (C_SIZE + 1) * 512 KB.
    //
    CSize = DwMmcExtractBits (mHost.Csd, 48, 69);
    mHost.Capacity = (CSize + 1) * 512 * 1024ULL;
  } else {
    //
    // SDSC (CSD v1.0).
    //
    UINT32  Rbl = 1 << DwMmcExtractBits (mHost.Csd, 80, 83);
    CSize       = DwMmcExtractBits (mHost.Csd, 62, 73);
    CMult       = DwMmcExtractBits (mHost.Csd, 47, 49);
    mHost.Capacity = (CSize + 1) * (1ULL << (CMult + 2)) * Rbl;
  }
  mHost.NumBlocks = (UINT32)(mHost.Capacity / MMC_MAX_BLOCK_LEN);

  //
  // ACMD51: SEND_SCR (SD Card Configuration Register).
  //
  Status = DwMmcSendAppCmd (mHost.CardRca);
  if (!EFI_ERROR (Status)) {
    Status = DwMmcSendCommandData (ACMD51_SEND_SCR, 0,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, FALSE, 8, 1, ScrData);
    if (!EFI_ERROR (Status)) {
      mHost.Scr[0] = ChangeEndian (
                       (ScrData[0] << 24) | (ScrData[1] << 16) |
                       (ScrData[2] << 8)  |  ScrData[3]);
      mHost.Scr[1] = ChangeEndian (
                       (ScrData[4] << 24) | (ScrData[5] << 16) |
                       (ScrData[6] << 8)  |  ScrData[7]);
    }
  }

  //
  // CMD6: SD_SWITCH_FUNC (check function group 1 = access mode).
  //
  Status = DwMmcSdSwitchCmd (SD_SWITCH_MODE_CHECK, 0, 1, SwitchStatus);
  if (!EFI_ERROR (Status)) {
    FuncGroup1 = SwitchStatus[13];

    DEBUG ((EFI_D_WARN, "DWMMC: SD caps=0x%04x (b13=0x%02x)\n",
      FuncGroup1, SwitchStatus[13]));

    if (FuncGroup1 & 0x02) {
      mHost.IsHighSpeed = TRUE;
      mHost.CardCaps   |= SD_HS_SDR25;
      DEBUG ((EFI_D_WARN, "DWMMC: SD High Speed supported\n"));
    }
    if (FuncGroup1 & 0x1C) {
      if (mHost.IsUhsSupported) {
        mHost.CardCaps |= SD_UHS_SDR50 | SD_UHS_SDR104;
        DEBUG ((EFI_D_WARN,
          "DWMMC: SD UHS-I modes supported (caps=0x%04x)\n", FuncGroup1));
      } else {
        DEBUG ((EFI_D_WARN,
          "DWMMC: SD UHS-I modes reported but 1.8V not available\n"));
      }
    }
  }

  DEBUG ((EFI_D_WARN, "DWMMC: SD capacity=%lld MB, SCR=0x%08x, HS=%d\n",
    mHost.Capacity / (1024 * 1024), mHost.Scr[0], mHost.IsHighSpeed));
  return EFI_SUCCESS;
}

/**
  Negotiate the best bus speed and width with the SD card.

  DwMmcSetIos() applies CLKSEL, clock, bus width and bus mode in one
  operation.  After tuning, mHost.IsTuned is set and DwMmcSetIos()
  automatically picks up the tuned sample phase — no manual CLKSEL
  writes needed.

  @retval EFI_SUCCESS   Speed and bus width configured.
**/
STATIC
EFI_STATUS
DwMmcSdAdjustSpeed (
  VOID
  )
{
  EFI_STATUS  Status;
  UINT32      Resp[4];
  UINT32      Sel;
  UINT32      TargetClk;
  UINT32      ClkselVal;
  UINT8       SwSt[64];
  UINT32      i;

  //
  // switch order is driver strength (group 2),
  // current limit (group 3), access mode (group 0).
  //
  UINT32  SwitchGroup[3] = {2, 3, 0};

  //
  // Speed grade: 0=DS, 1=HS, 2=SDR50, 3=SDR104.
  // [sel] = {access_mode_val, clock_hz}
  //
  UINT32  SwitchVal[4]   = { 0x0, 0x1, 0x2, 0x3 };
  UINT32  SwitchClk[4]   = { SD_CLK_25MHZ, SD_CLK_50MHZ,
                              SD_CLK_100MHZ, SD_CLK_208MHZ };

  DEBUG ((EFI_D_WARN, "DWMMC: Scr[0]=0x%08x BW4=%d UHS=%d caps=0x%02x\n",
    mHost.Scr[0],
    (mHost.Scr[0] & SCR_BUS_WIDTH_4) ? 1 : 0,
    mHost.IsUhsSupported, mHost.CardCaps));

  mHost.BusMode = 0;   // SDR

  //
  // Set bus width via ACMD6, then apply IOS.
  //
  if (mHost.Scr[0] & SCR_BUS_WIDTH_4) {
    Status = DwMmcSendAppCmd (mHost.CardRca);
    if (!EFI_ERROR (Status)) {
      Status = DwMmcSendCommand (ACMD6_SET_BUS_WIDTH, 2,
                 MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
      if (!EFI_ERROR (Status)) {
        mHost.BusWidth = 4;
      } else {
        DEBUG ((EFI_D_WARN, "DWMMC: ACMD6 4-bit failed: %r\n", Status));
      }
    }
  }
  DwMmcSetIos (mHost.CardClock, mHost.SdrClksel);

  //
  // Pick highest supported speed grade.
  //
  if (mHost.IsUhsSupported && (mHost.CardCaps & SD_UHS_SDR104)) {
    Sel = 3;
  } else if (mHost.IsUhsSupported && (mHost.CardCaps & SD_UHS_SDR50)) {
    Sel = 2;
  } else if (mHost.CardCaps & SD_HS_SDR25) {
    Sel = 1;
  } else {
    Sel = 0;
  }

  //
  // For UHS: send the 3 switch commands.
  // If switch is rejected, fall back one grade.
  //
  if (mHost.IsUhsSupported) {
    for (i = 0; i < 3; i++) {
      Status = DwMmcSdSwitchCmd (SD_SWITCH_MODE_SWITCH,
                 SwitchGroup[i], SwitchVal[Sel], SwSt);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_WARN, "DWMMC: SD switch group %u failed: %r\n",
          SwitchGroup[i], Status));
        if (Sel > 1) {
          Sel--;
          i = (UINT32)-1;   // restart loop with lower grade
        }
      }
    }

    //
    // Verify access mode was accepted (group 0 result in sts[16] bits 3:0).
    //
    if ((SwSt[16] & 0xF) != SwitchVal[Sel]) {
      DEBUG ((EFI_D_WARN,
        "DWMMC: speed grade %u rejected (sts[16]=%u)\n",
        Sel, SwSt[16] & 0xF));
      if (Sel > 1) {
        Sel--;
      }
    }
  }

  //
  // Set clock and CLKSEL, apply IOS.
  //
  TargetClk = SwitchClk[Sel];
  mHost.CardClock = TargetClk;

  if (Sel == 3) {
    ClkselVal = mHost.Sdr104Clksel;
  } else if (Sel == 2) {
    ClkselVal = mHost.Sdr50Clksel;
  } else {
    ClkselVal = mHost.SdrClksel;
  }

  mHost.MaxClock = TargetClk;
  DwMmcSetIos (TargetClk, ClkselVal);
  MicroSecondDelay (5000);

  //
  // Tuning for SDR50 / SDR104.
  // DwMmcSetIos() picks up mHost.IsTuned automatically.
  //
  if (Sel >= 2) {
    Status = DwMmcExecuteTuning (CMD19_SEND_TUNING);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN,
        "DWMMC: tuning failed, speed may be unreliable\n"));
      mHost.IsTuned = FALSE;
    }
    //
    // Re-apply IOS so CLKSEL reflects tuned phase.
    //
    DwMmcSetIos (TargetClk, ClkselVal);
  }

  DEBUG ((EFI_D_WARN, "DWMMC: SD speed: %s %d-bit %d MHz\n",
    (Sel >= 3) ? "SDR104" :
    (Sel == 2) ? "SDR50"  :
    (Sel == 1) ? "HS"     : "DS",
    mHost.BusWidth, mHost.CardClock / 1000000));
  return EFI_SUCCESS;
}

/**
  Full SD card bring-up: init -> identify -> decode info -> adjust speed.

  @retval EFI_SUCCESS   Card fully initialised and ready for I/O.
**/
STATIC
EFI_STATUS
DwMmcInitAndIdentify (
  VOID
  )
{
  EFI_STATUS  Status;

  Status = DwMmcSetInitState ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = DwMmcSdInitCard ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_WARN, "DWMMC: SD init failed: %r\n", Status));
    return Status;
  }

  Status = DwMmcIdentifyCard ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  DwMmcDecodeSdInfo ();
  DwMmcSdAdjustSpeed ();

  //
  // Set block length to 512 bytes.
  //
  {
    UINT32  Resp[4];
    Status = DwMmcSendCommand (CMD16_SET_BLOCKLEN, 512,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (EFI_ERROR (Status) || (Resp[0] & R1_BLOCK_LEN_ERROR)) {
      DEBUG ((EFI_D_ERROR, "DWMMC: CMD16 failed\n"));
      return EFI_DEVICE_ERROR;
    }
  }

  mHost.CardPresent = TRUE;
  mHost.InitDone    = TRUE;

  DEBUG ((EFI_D_WARN, "DWMMC: Init complete -- SD, %u blocks, %d-bit, %d MHz\n",
    mHost.NumBlocks, mHost.BusWidth, mHost.CardClock / 1000000));
  return EFI_SUCCESS;
}

/**
  Erase a range of blocks on the SD card.

  Uses CMD32/CMD33 for SD erase.  Waits up to 240 seconds for completion.

  @param[in] Lba          Start LBA.
  @param[in] BlockCount   Number of blocks to erase.
  @param[in] SecurePurge  If TRUE, use secure erase (if supported).

  @retval EFI_SUCCESS        Erase completed.
  @retval EFI_DEVICE_ERROR   Card not in TRAN state.
  @retval EFI_TIMEOUT        Erase timed out.
**/
STATIC
EFI_STATUS
DwMmcErase (
  EFI_LBA  Lba,
  UINTN    BlockCount,
  BOOLEAN  SecurePurge
  )
{
  EFI_STATUS  Status;
  UINT32      Arg;
  UINT32      CardState;
  UINT32      EraseTimeout;
  UINT32      Start;
  UINT32      End;

  if (BlockCount == 0) {
    return EFI_SUCCESS;
  }

  //
  // Card must be in TRAN state.
  //
  Status = DwMmcGetCardStatus (1000, &CardState);
  if (EFI_ERROR (Status) || CardState != 4) {
    DEBUG ((EFI_D_ERROR,
      "DWMMC: Card not in TRAN state for erase (state=%d)\n", CardState));
    return EFI_DEVICE_ERROR;
  }

  //
  // For SD, use CMD32/CMD33 (byte address for SDSC, sector for SDHC).
  //
  Start = (UINT32)Lba;
  End   = (UINT32)(Lba + BlockCount - 1);
  if (!mHost.IsSdhc) {
    Start *= MMC_MAX_BLOCK_LEN;
    End   *= MMC_MAX_BLOCK_LEN;
  }

  //
  // Set erase start address.
  //
  Arg = Start;
  Status = DwMmcSendCommand (CMD32_ERASE_WR_BLK_START, Arg,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD32 (erase start) failed: %r\n", Status));
    return Status;
  }

  //
  // Set erase end address.
  //
  Arg = End;
  Status = DwMmcSendCommand (CMD33_ERASE_WR_BLK_END, Arg,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD33 (erase end) failed: %r\n", Status));
    return Status;
  }

  //
  // Issue CMD38 (ERASE).
  //
  Arg = SecurePurge ? MMC_ERASE_SECURE : MMC_ERASE_NORMAL;
  Status = DwMmcSendCommand (CMD38_ERASE, Arg,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE | MMC_RSP_BUSY, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD38 erase failed: %r\n", Status));
    return Status;
  }

  //
  // Wait for erase completion (can take several seconds).
  //
  for (EraseTimeout = 0;
       EraseTimeout < MMC_ERASE_TIMEOUT_SEC * 100;
       EraseTimeout++) {
    MicroSecondDelay (10000);
    Status = DwMmcGetCardStatus (0, &CardState);
    if (!EFI_ERROR (Status) && (CardState == 4)) {
      break;
    }
  }

  if (EraseTimeout >= MMC_ERASE_TIMEOUT_SEC * 100) {
    DEBUG ((EFI_D_WARN,
      "DWMMC: Erase timeout after %d seconds\n", MMC_ERASE_TIMEOUT_SEC));
    return EFI_TIMEOUT;
  }

  DEBUG ((EFI_D_INFO, "DWMMC: %a %lu blocks from LBA %lu\n",
    SecurePurge ? "Secure purged" : "Erased", BlockCount, Lba));
  return EFI_SUCCESS;
}

typedef struct {
  EFI_BLOCK_IO_PROTOCOL      BlockIo;
  EFI_ERASE_BLOCK_PROTOCOL   EraseBlock;
  UINTN                      Signature;
  EFI_BLOCK_IO_MEDIA         Media;
} DW_BLKDEV;

#define DW_SIG  SIGNATURE_32 ('D','W','S','D')
STATIC DW_BLKDEV  *mDev;

EFI_GUID  gExynosSdCardGuid = EXYNOS_SD_CARD_GUID;

/**
  Build an EFI device path for the SD controller.

  @param[in] Slot   Slot identifier (channel number).

  @return Pointer to device path, or NULL on allocation failure.
**/
STATIC
EXYNOS_SD_DEVICE_PATH *
DwMmcBuildDevicePath (
  UINT8  Slot
  )
{
  EXYNOS_SD_DEVICE_PATH  *Dp;

  Dp = AllocateZeroPool (sizeof (EXYNOS_SD_DEVICE_PATH));
  if (!Dp) {
    return NULL;
  }

  Dp->VendorDp.Header.Type    = HARDWARE_DEVICE_PATH;
  Dp->VendorDp.Header.SubType = HW_VENDOR_DP;
  SetDevicePathNodeLength (&Dp->VendorDp.Header,
    sizeof (VENDOR_DEVICE_PATH) + sizeof (UINT8));
  CopyMem (&Dp->VendorDp.Guid, &gExynosSdCardGuid, sizeof (EFI_GUID));
  Dp->Slot = Slot;

  SetDevicePathEndNode (&Dp->End);
  return Dp;
}

//
// Block I/O Protocol functions
//

/**
  Reset the block device.

  @param[in] This                  Block I/O protocol instance.
  @param[in] ExtendedVerification  Unused.

  @retval EFI_SUCCESS always.
**/
STATIC
EFI_STATUS
EFIAPI
BlkReset (
  EFI_BLOCK_IO_PROTOCOL  *This,
  BOOLEAN                 ExtendedVerification
  )
{
  return EFI_SUCCESS;
}

/**
  Read blocks from the SD card.

  @param[in]  This        Block I/O protocol instance.
  @param[in]  MediaId     Media ID (must match device's MediaId).
  @param[in]  Lba         Starting logical block address.
  @param[in]  BufferSize  Size of Buffer in bytes.
  @param[out] Buffer      Destination buffer.

  @retval EFI_SUCCESS            Read completed.
  @retval EFI_MEDIA_CHANGED      MediaId mismatch.
**/
STATIC
EFI_STATUS
EFIAPI
BlkRead (
  IN  EFI_BLOCK_IO_PROTOCOL  *This,
  IN  UINT32                  MediaId,
  IN  EFI_LBA                 Lba,
  IN  UINTN                   BufferSize,
  OUT VOID                   *Buffer
  )
{
  DW_BLKDEV  *Dev = (DW_BLKDEV *)This;
  UINT32      N;
  UINT32      Arg;
  EFI_STATUS  Status;

  if (MediaId != Dev->Media.MediaId) {
    return EFI_MEDIA_CHANGED;
  }
  if (!BufferSize || !Buffer) {
    return EFI_SUCCESS;
  }

  N   = (UINT32)(BufferSize / MMC_MAX_BLOCK_LEN);
  Arg = mHost.IsSdhc ? (UINT32)Lba : (UINT32)(Lba * MMC_MAX_BLOCK_LEN);

  if (N == 1) {
    Status = DwMmcSendCommandData (CMD17_READ_SINGLE, Arg,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, FALSE, MMC_MAX_BLOCK_LEN, 1, Buffer);
  } else {
    //
    // Use CMD23 (SET_BLOCK_COUNT) before CMD18 for SD cards that support it.
    //
    DwMmcSendCommand (CMD23_SET_BLKCNT, N,
      MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, NULL);
    Status = DwMmcSendCommandData (CMD18_READ_MULTI, Arg,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, FALSE, MMC_MAX_BLOCK_LEN, N, Buffer);
  }

  return Status;
}

/**
  Write blocks to the SD card.

  @param[in] This        Block I/O protocol instance.
  @param[in] MediaId     Media ID (must match device's MediaId).
  @param[in] Lba         Starting logical block address.
  @param[in] BufferSize  Size of Buffer in bytes.
  @param[in] Buffer      Source buffer.

  @retval EFI_SUCCESS            Write completed.
  @retval EFI_MEDIA_CHANGED      MediaId mismatch.
  @retval EFI_DEVICE_ERROR       Write rejected by card (R1_WP_VIOLATION, etc.).
**/
STATIC
EFI_STATUS
EFIAPI
BlkWrite (
  IN EFI_BLOCK_IO_PROTOCOL  *This,
  IN UINT32                  MediaId,
  IN EFI_LBA                 Lba,
  IN UINTN                   BufferSize,
  IN VOID                   *Buffer
  )
{
  DW_BLKDEV  *Dev = (DW_BLKDEV *)This;
  UINT32      N;
  UINT32      Arg;
  EFI_STATUS  Status;

  if (MediaId != Dev->Media.MediaId) {
    return EFI_MEDIA_CHANGED;
  }
  if (!BufferSize || !Buffer) {
    return EFI_SUCCESS;
  }

  if (RD (DWMCI_WRTPRT) & WRTPRT_WRITE_PROTECT) {
    DEBUG ((EFI_D_ERROR, "DWMMC: Card is write-protected\n"));
    return EFI_WRITE_PROTECTED;
  }

  N   = (UINT32)(BufferSize / MMC_MAX_BLOCK_LEN);
  Arg = mHost.IsSdhc ? (UINT32)Lba : (UINT32)(Lba * MMC_MAX_BLOCK_LEN);

  if (N == 1) {
    Status = DwMmcSendCommandData (CMD24_WRITE_SINGLE, Arg,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, TRUE, MMC_MAX_BLOCK_LEN, 1, Buffer);
  } else {
    //
    // Use CMD23 (SET_BLOCK_COUNT) before CMD25.
    //
    DwMmcSendCommand (CMD23_SET_BLKCNT, N,
      MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, NULL);
    Status = DwMmcSendCommandData (CMD25_WRITE_MULTI, Arg,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, TRUE, MMC_MAX_BLOCK_LEN, N, Buffer);
  }

  return Status;
}

/**
  Flush cached writes to the card.

  @param[in] This   Block I/O protocol instance.

  @retval EFI_SUCCESS always.
**/
STATIC
EFI_STATUS
EFIAPI
BlkFlush (
  IN EFI_BLOCK_IO_PROTOCOL  *This
  )
{
  return EFI_SUCCESS;
}

//
// Erase Block Protocol
//

/**
  Erase blocks callback (synchronous implementation).

  @param[in] This      Block I/O protocol instance.
  @param[in] MediaId   Media ID.
  @param[in] LBA       Starting LBA.
  @param[in] Token     Async token (Event is signalled on completion).
  @param[in] Size      Number of bytes to erase.

  @retval EFI_SUCCESS   Erase queued / completed.
**/
STATIC
EFI_STATUS
EFIAPI
EraseBlocks (
  IN EFI_BLOCK_IO_PROTOCOL     *This,
  IN UINT32                     MediaId,
  IN EFI_LBA                    LBA,
  IN OUT EFI_ERASE_BLOCK_TOKEN  *Token,
  IN UINTN                      Size
  )
{
  DW_BLKDEV  *Dev = (DW_BLKDEV *)This;
  EFI_STATUS  Status;
  UINTN       BlockCount;

  if (MediaId != Dev->Media.MediaId) {
    return EFI_MEDIA_CHANGED;
  }
  if (Size == 0) {
    return EFI_SUCCESS;
  }
  if (Size % Dev->Media.BlockSize != 0) {
    return EFI_INVALID_PARAMETER;
  }

  BlockCount = Size / Dev->Media.BlockSize;

  Status = DwMmcErase (LBA, BlockCount, FALSE);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_WARN,
      "DWMMC: EraseBlocks failed (%r), trying secure purge\n", Status));
    Status = DwMmcErase (LBA, BlockCount, TRUE);
  }

  //
  // Signal completion if async token was provided.
  //
  if (Token != NULL) {
    Token->TransactionStatus = Status;
    if (Token->Event != NULL) {
      gBS->SignalEvent (Token->Event);
    }
  }

  return Status;
}

/**
  Entry point for the SD/MMC Block I/O DXE driver.

  Initialises the SD platform hardware, detects and initialises the SD card,
  and installs the EFI_BLOCK_IO_PROTOCOL and EFI_ERASE_BLOCK_PROTOCOL.

  @param[in] ImageHandle   Firmware-allocated handle for the driver image.
  @param[in] SystemTable   EFI System Table.

  @retval EFI_SUCCESS           Driver initialised and protocols installed.
  @retval EFI_NOT_FOUND         No SD card detected.
  @retval EFI_DEVICE_ERROR      Hardware initialisation failed.
  @retval EFI_OUT_OF_RESOURCES  Memory allocation failed.
**/
EFI_STATUS
EFIAPI
InitDwMmcDriver (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS          Status;
  EFI_HANDLE          Handle = NULL;
  MMC_PLATFORM_HOST   Plat;

  //
  // Initialise the SD card platform (PMIC, GPIO, CMU).
  //
  Status = MmcPlatformInit (MMC_CHANNEL_SD, &Plat);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: SD platform init failed: %r\n", Status));
    return Status;
  }

  mChannel = MMC_CHANNEL_SD;
  ZeroMem (&mHost, sizeof (mHost));
  mHost.IoBase       = Plat.IoBase;
  mHost.SdrClksel    = Plat.SdrClksel;
  mHost.DdrClksel    = Plat.DdrClksel;
  mHost.Sdr50Clksel  = Plat.Sdr50Clksel;
  mHost.Sdr104Clksel = Plat.Sdr104Clksel;
  mHost.BusHz        = Plat.BusHz;
  mHost.PhaseDivide  = Plat.PhaseDivide;
  mHost.Ciudiv       = Plat.Ciudiv;
  mHost.FifoDepth    = Plat.FifoDepth;
  mHost.Secure       = Plat.Secure;
  mHost.MpsSecurity  = Plat.MpsSecurity;

  DEBUG ((EFI_D_WARN, "DWMMC: SD channel @ 0x%lx Bus=%d MHz\n",
    mHost.IoBase, Plat.BusHz / 1000000));

  Status = DwMmcInitAndIdentify ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: SD init failed: %r\n", Status));
    return Status;
  }

  //
  // Allocate and populate the Block I/O device.
  //
  mDev = AllocateZeroPool (sizeof (DW_BLKDEV));
  if (!mDev) {
    return EFI_OUT_OF_RESOURCES;
  }

  mDev->Signature             = DW_SIG;
  mDev->Media.MediaId         = 1;
  mDev->Media.RemovableMedia  = TRUE;
  mDev->Media.MediaPresent    = mHost.CardPresent;
  mDev->Media.BlockSize       = mHost.BlockSize;
  mDev->Media.LastBlock       = mHost.NumBlocks > 0 ?
                                  (mHost.NumBlocks - 1) : 0;
  mDev->BlockIo.Revision      = EFI_BLOCK_IO_PROTOCOL_REVISION2;
  mDev->BlockIo.Media         = &mDev->Media;
  mDev->BlockIo.Reset         = BlkReset;
  mDev->BlockIo.ReadBlocks    = BlkRead;
  mDev->BlockIo.WriteBlocks   = BlkWrite;
  mDev->BlockIo.FlushBlocks   = BlkFlush;

  mDev->EraseBlock.Revision               = EFI_ERASE_BLOCK_PROTOCOL_REVISION;
  mDev->EraseBlock.EraseLengthGranularity = 1;
  mDev->EraseBlock.EraseBlocks            = EraseBlocks;

  //
  // Build device path and install protocols.
  //
  {
    EXYNOS_SD_DEVICE_PATH  *Dp = DwMmcBuildDevicePath ((UINT8)mChannel);
    if (!Dp) {
      FreePool (mDev);
      return EFI_OUT_OF_RESOURCES;
    }

    Status = gBS->InstallMultipleProtocolInterfaces (
                    &Handle,
                    &gEfiBlockIoProtocolGuid,  &mDev->BlockIo,
                    &gEfiEraseBlockProtocolGuid, &mDev->EraseBlock,
                    &gEfiDevicePathProtocolGuid, Dp,
                    NULL);
    if (EFI_ERROR (Status)) {
      FreePool (mDev);
      FreePool (Dp);
      return Status;
    }
  }

  DEBUG ((EFI_D_INFO,
    "DWMMC: SD card BlockIO installed (%u blocks, %lld MB)\n",
    mHost.NumBlocks, mHost.Capacity / (1024 * 1024)));

  return EFI_SUCCESS;
}
