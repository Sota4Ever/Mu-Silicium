#ifndef __SD_MMC_DXE_INTERNAL_H__
#define __SD_MMC_DXE_INTERNAL_H__

#include <Uefi.h>
#include <Protocol/BlockIo.h>
#include <Library/DwMmcRegs.h>

//
// IDMAC descriptor table — up to 256 entries, each 64 bytes = 16 KB.
// Must be 64-byte aligned for cache maintenance.
//
STATIC DWMMC_IDMAC_DESC  mDmaDesc[256] __attribute__((aligned(64)));

//
// Tuning data buffer (64 bytes for SD CMD19 4-bit tuning block).
//
STATIC UINT8  mTuningBuf[TUNING_BLOCK_SIZE] __attribute__((aligned(64)));

//
// Current DMA transfer state.
//
STATIC UINT32   mDmaBlockSize;
STATIC UINT32   mDmaBlockCnt;
STATIC BOOLEAN  mDmaIsRead;
STATIC VOID    *mDmaBuffer;

//
// Erase Block Protocol callback type.
//
typedef struct {
  EFI_EVENT   Event;
  EFI_STATUS  TransactionStatus;
} EFI_ERASE_BLOCK_TOKEN;

typedef
EFI_STATUS
(EFIAPI *EFI_BLOCK_ERASE)(
  IN EFI_BLOCK_IO_PROTOCOL     *This,
  IN UINT32                     MediaId,
  IN EFI_LBA                    LBA,
  IN OUT EFI_ERASE_BLOCK_TOKEN  *Token,
  IN UINTN                      Size
  );

typedef struct {
  UINT64              Revision;
  UINT32              EraseLengthGranularity;
  EFI_BLOCK_ERASE     EraseBlocks;
} EFI_ERASE_BLOCK_PROTOCOL;

#define EFI_ERASE_BLOCK_PROTOCOL_REVISION  ((2 << 16) | (60))

STATIC EFI_STATUS
DwMmcSendCommand (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp
  );

STATIC EFI_STATUS
DwMmcSendCommandData (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp,
  BOOLEAN  IsWrite,
  UINT32   BlockSize,
  UINT32   BlockCnt,
  VOID    *Buf
  );

STATIC EFI_STATUS
DwMmcChangeClock (
  UINT32  TargetClk
  );

STATIC EFI_STATUS
DwMmcControlClken (
  UINT32  Val
  );

STATIC EFI_STATUS
DwMmcGetCardStatus (
  UINT32   TimeoutMs,
  UINT32  *OutState
  );

STATIC EFI_STATUS
DwMmcSetIos (
  UINT32  Clock,
  UINT32  ClkselVal
  );

STATIC EFI_STATUS
DwMmcSetIosSimple (
  UINT32  Clock
  );

STATIC EFI_STATUS
DwMmcExecuteTuning (
  UINT32  CmdIdx
  );

STATIC EFI_STATUS
DwMmcErase (
  EFI_LBA  Lba,
  UINTN    BlockCount,
  BOOLEAN  SecurePurge
  );

STATIC
EFI_STATUS
EFIAPI
EraseBlocks (
  IN EFI_BLOCK_IO_PROTOCOL     *This,
  IN UINT32                     MediaId,
  IN EFI_LBA                    LBA,
  IN OUT EFI_ERASE_BLOCK_TOKEN  *Token,
  IN UINTN                      Size
  );

STATIC
VOID
DwMmcChangeClksel (
  UINT32  PassIndex
  );

#endif // __SD_MMC_DXE_INTERNAL_H__
