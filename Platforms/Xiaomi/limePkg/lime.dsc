## @file
#
#  Copyright (c) 2011-2015, ARM Limited. All rights reserved.
#  Copyright (c) 2014, Linaro Limited. All rights reserved.
#  Copyright (c) 2015 - 2016, Intel Corporation. All rights reserved.
#  Copyright (c) 2018 - 2019, Bingxing Wang. All rights reserved.
#  Copyright (c) 2022, Xilin Wu. All rights reserved.
#
#  SPDX-License-Identifier: BSD-2-Clause-Patent
#
##

################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  PLATFORM_NAME                  = lime
  PLATFORM_GUID                  = 7004a90e-23e3-409f-887e-26cfff6e4ae5
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010005
  OUTPUT_DIRECTORY               = Build/limePkg
  SUPPORTED_ARCHITECTURES        = AARCH64
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = limePkg/lime.fdf
  USE_DISPLAYDXE                 = 0
  AB_SLOT_SUPPORT                = 0
  USE_UART                       = 0

[LibraryClasses.common]
  PlatformMemoryMapLib|limePkg/Library/PlatformMemoryMapLib/PlatformMemoryMapLib.inf
  PlatformPeiLib|limePkg/Library/PlatformPei/PlatformPeiLib.inf

[PcdsFixedAtBuild.common]
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x40000000         # Starting address
!if $(RAM_SIZE) == 6
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x180000000        # 6GB Size
!else
!if $(RAM_SIZE) == 4
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x100000000        # 4GB Size
!else
!error "Invaild Mem Size! Use 6 or 4."
!endif
!endif

  gArmTokenSpaceGuid.PcdCpuVectorBaseAddress|0x5FF8C000

  gEmbeddedTokenSpaceGuid.PcdPrePiStackBase|0x5FF90000
  gEmbeddedTokenSpaceGuid.PcdPrePiStackSize|0x00040000      # 256K stack

  # Simple FrameBuffer
  gQcomTokenSpaceGuid.PcdMipiFrameBufferWidth|1080
  gQcomTokenSpaceGuid.PcdMipiFrameBufferHeight|2340
  gQcomTokenSpaceGuid.PcdMipiFrameBufferPixelBpp|32

  # UART
  gQcomTokenSpaceGuid.PcdDebugUartPortBase|0x4a90000

  # Device Info
  gQcomTokenSpaceGuid.PcdSmbiosSystemVendor|"Xiaomi"
  gQcomTokenSpaceGuid.PcdSmbiosSystemModel|"Redmi 9T"
  gQcomTokenSpaceGuid.PcdSmbiosSystemRetailModel|"lime"
  gQcomTokenSpaceGuid.PcdSmbiosSystemRetailSku|"Redmi_9T_lime"
  gQcomTokenSpaceGuid.PcdSmbiosBoardModel|"Redmi 9T"

[PcdsDynamicDefault.common]
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoHorizontalResolution|1080
  gEfiMdeModulePkgTokenSpaceGuid.PcdVideoVerticalResolution|2340
  gEfiMdeModulePkgTokenSpaceGuid.PcdSetupVideoHorizontalResolution|1080
  gEfiMdeModulePkgTokenSpaceGuid.PcdSetupVideoVerticalResolution|2340

!include SM6115Pkg/SM6115.dsc.inc
