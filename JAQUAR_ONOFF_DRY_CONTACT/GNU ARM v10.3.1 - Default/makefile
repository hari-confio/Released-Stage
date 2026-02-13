################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_4.1.1/util/third_party/freertos/kernel/portable/SiliconLabs/subdir.mk
-include gecko_sdk_4.1.1/util/third_party/freertos/kernel/portable/MemMang/subdir.mk
-include gecko_sdk_4.1.1/util/third_party/freertos/kernel/portable/GCC/ARM_CM4F/subdir.mk
-include gecko_sdk_4.1.1/util/third_party/freertos/kernel/subdir.mk
-include gecko_sdk_4.1.1/util/third_party/freertos/cmsis/Source/subdir.mk
-include gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/subdir.mk
-include gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/subdir.mk
-include gecko_sdk_4.1.1/util/third_party/crypto/mbedtls/library/subdir.mk
-include gecko_sdk_4.1.1/util/silicon_labs/silabs_core/memory_manager/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/SwitchOnOff/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/config/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/ZWavePlusInfo/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Version/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Supervision/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/PowerLevel/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/MultiChan/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/ManufacturerSpecific/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Indicator/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/FirmwareUpdate/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/DeviceResetLocally/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Configuration/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Common/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/BinarySwitch/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Basic/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Association/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/_commonIF/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/TrueStatusEngine/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/PowerManagement/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/EventHandling/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/Actuator/subdir.mk
-include gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/subdir.mk
-include gecko_sdk_4.1.1/platform/service/system/src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/power_manager/src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/mpu/src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/iostream/src/subdir.mk
-include gecko_sdk_4.1.1/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.1.1/platform/radio/rail_lib/plugin/pa-conversions/subdir.mk
-include gecko_sdk_4.1.1/platform/emlib/src/subdir.mk
-include gecko_sdk_4.1.1/platform/emdrv/nvm3/src/subdir.mk
-include gecko_sdk_4.1.1/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_4.1.1/platform/driver/debug/src/subdir.mk
-include gecko_sdk_4.1.1/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.1.1/platform/common/src/subdir.mk
-include gecko_sdk_4.1.1/platform/bootloader/app_properties/subdir.mk
-include gecko_sdk_4.1.1/platform/bootloader/api/subdir.mk
-include gecko_sdk_4.1.1/platform/Device/SiliconLabs/ZGM13/Source/subdir.mk
-include gecko_sdk_4.1.1/platform/CMSIS/RTOS2/Source/subdir.mk
-include gecko_sdk_4.1.1/hardware/driver/configuration_over_swo/src/subdir.mk
-include gecko_sdk_4.1.1/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := JAQUAR_ONOFF_DRY_CONTACT
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: JAQUAR_ONOFF_DRY_CONTACT.axf

# Tool invocations
JAQUAR_ONOFF_DRY_CONTACT.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\DELL\SimplicityStudio\v5_workspace\JAQUAR_ONOFF_DRY_CONTACT\autogen\linkerfile.ld" -Wl,--start-group -lgcc -lc -Wl,--end-group -Xlinker --gc-sections -Xlinker -Map="JAQUAR_ONOFF_DRY_CONTACT.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o JAQUAR_ONOFF_DRY_CONTACT.axf -Wl,--start-group "./8PB_SwitchOnOff.o" "./CC_Configuration_Parameters.o" "./app.o" "./board_config.o" "./main.o" "./wall_controller.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_iostream_handles.o" "./autogen/sl_iostream_init_usart_instances.o" "./gecko_sdk_4.1.1/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.1.1/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.1.1/hardware/driver/configuration_over_swo/src/sl_cos.o" "./gecko_sdk_4.1.1/platform/CMSIS/RTOS2/Source/os_systick.o" "./gecko_sdk_4.1.1/platform/Device/SiliconLabs/ZGM13/Source/startup_zgm13.o" "./gecko_sdk_4.1.1/platform/Device/SiliconLabs/ZGM13/Source/system_zgm13.o" "./gecko_sdk_4.1.1/platform/bootloader/api/btl_interface.o" "./gecko_sdk_4.1.1/platform/bootloader/api/btl_interface_storage.o" "./gecko_sdk_4.1.1/platform/bootloader/app_properties/app_properties.o" "./gecko_sdk_4.1.1/platform/common/src/sl_assert.o" "./gecko_sdk_4.1.1/platform/common/src/sl_slist.o" "./gecko_sdk_4.1.1/platform/common/src/sli_cmsis_os2_ext_task_register.o" "./gecko_sdk_4.1.1/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.1.1/platform/common/toolchain/src/sl_storage.o" "./gecko_sdk_4.1.1/platform/driver/debug/src/sl_debug_swo.o" "./gecko_sdk_4.1.1/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_4.1.1/platform/emdrv/nvm3/src/nvm3_default_common_linker.o" "./gecko_sdk_4.1.1/platform/emdrv/nvm3/src/nvm3_hal_flash.o" "./gecko_sdk_4.1.1/platform/emdrv/nvm3/src/nvm3_lock.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_acmp.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_adc.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_core.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_cryotimer.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_crypto.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_csen.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_dbg.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_emu.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_gpcrc.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_i2c.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_idac.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_ldma.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_lesense.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_letimer.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_leuart.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_msc.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_opamp.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_pcnt.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_prs.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_rmu.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_rtcc.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_system.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_timer.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_usart.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_vdac.o" "./gecko_sdk_4.1.1/platform/emlib/src/em_wdog.o" "./gecko_sdk_4.1.1/platform/radio/rail_lib/plugin/pa-conversions/pa_conversions_efr32.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_dcdc_s1.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_emu_s1.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_hfxo_s1.o" "./gecko_sdk_4.1.1/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.1.1/platform/service/iostream/src/sl_iostream.o" "./gecko_sdk_4.1.1/platform/service/iostream/src/sl_iostream_uart.o" "./gecko_sdk_4.1.1/platform/service/iostream/src/sl_iostream_usart.o" "./gecko_sdk_4.1.1/platform/service/mpu/src/sl_mpu.o" "./gecko_sdk_4.1.1/platform/service/power_manager/src/sl_power_manager.o" "./gecko_sdk_4.1.1/platform/service/power_manager/src/sl_power_manager_debug.o" "./gecko_sdk_4.1.1/platform/service/power_manager/src/sl_power_manager_hal_s0_s1.o" "./gecko_sdk_4.1.1/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_4.1.1/platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" "./gecko_sdk_4.1.1/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_4.1.1/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.1.1/platform/service/system/src/sl_system_kernel.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/Actuator/ZAF_Actuator.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/AppTimer.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_CC_Invoker.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_CmdPublisher.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_command_class_utils.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_network_learn.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_network_management.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_nvm_app.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_transport.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZAF_tx_mutex.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZW_TransportEndpoint.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZW_TransportMulticast.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/ZW_TransportSecProtocol.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/EventHandling/zaf_event_helper.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/EventHandling/zaf_job_helper.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/PowerManagement/ZAF_PM_Wrapper.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/TrueStatusEngine/ZAF_TSE.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/TrueStatusEngine/ZAF_TSE_weak.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/_commonIF/ZAF_Common_helper.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/ApplicationUtilities/_commonIF/ZAF_Common_interface.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Association/CC_Association.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Association/CC_AssociationGroupInfo.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Association/CC_MultiChanAssociation.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Association/agi.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Association/association_plus.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Basic/CC_Basic.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/BinarySwitch/CC_BinarySwitch.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Common/CC_Common.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Configuration/CC_Configuration.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/DeviceResetLocally/CC_DeviceResetLocally.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/DeviceResetLocally/CC_DeviceResetLocally_weak.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/FirmwareUpdate/CC_FirmwareUpdate.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/FirmwareUpdate/ota_util.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Indicator/CC_Indicator.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Indicator/CC_Indicator_weak.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/ManufacturerSpecific/CC_ManufacturerSpecific.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/ManufacturerSpecific/CC_ManufacturerSpecific_config.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/MultiChan/CC_MultiChan.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/MultiChan/multichannel.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/PowerLevel/CC_PowerLevel.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Supervision/CC_Supervision.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/Version/CC_Version.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/ZWavePlusInfo/CC_ZWavePlusInfo.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/CommandClasses/ZWavePlusInfo/CC_ZWavePlusInfo_config.o" "./gecko_sdk_4.1.1/protocol/z-wave/ZAF/config/zaf_config_api.o" "./gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/SwitchOnOff/SwitchOnOff_hw.o" "./gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/ADC.o" "./gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/board.o" "./gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/board_BRD420x.o" "./gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/board_indicator.o" "./gecko_sdk_4.1.1/protocol/z-wave/platform/SiliconLabs/AppsHw/src/common/board_init.o" "./gecko_sdk_4.1.1/util/silicon_labs/silabs_core/memory_manager/sl_malloc.o" "./gecko_sdk_4.1.1/util/third_party/crypto/mbedtls/library/constant_time.o" "./gecko_sdk_4.1.1/util/third_party/crypto/mbedtls/library/platform.o" "./gecko_sdk_4.1.1/util/third_party/crypto/mbedtls/library/platform_util.o" "./gecko_sdk_4.1.1/util/third_party/crypto/mbedtls/library/psa_crypto_client.o" "./gecko_sdk_4.1.1/util/third_party/crypto/mbedtls/library/sha1.o" "./gecko_sdk_4.1.1/util/third_party/crypto/mbedtls/library/threading.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/mbedtls_sha.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/sl_mbedtls.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/crypto_management.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_aead.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_cipher.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_hash.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_mac.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_psa_driver_common.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_psa_driver_init.o" "./gecko_sdk_4.1.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_se_version_dependencies.o" "./gecko_sdk_4.1.1/util/third_party/freertos/cmsis/Source/cmsis_os2.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/croutine.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/event_groups.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/list.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/queue.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/stream_buffer.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/tasks.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/timers.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/portable/GCC/ARM_CM4F/port.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/portable/MemMang/heap_4.o" "./gecko_sdk_4.1.1/util/third_party/freertos/kernel/portable/SiliconLabs/tick_power_manager.o" "C:/Users/DELL/SimplicityStudio/SDKs/gecko_sdk_3//platform/emdrv/nvm3/lib/libnvm3_CM4_gcc.a" "C:/Users/DELL/SimplicityStudio/SDKs/gecko_sdk_3//platform/radio/rail_lib/autogen/librail_release/librail_config_zgm130s037hgn_gcc.a" "C:/Users/DELL/SimplicityStudio/SDKs/gecko_sdk_3//platform/radio/rail_lib/autogen/librail_release/librail_module_efr32xg13_gcc_release.a" "C:/Users/DELL/SimplicityStudio/SDKs/gecko_sdk_3//protocol/z-wave/ZWave/lib/libZWaveSlave_700s.a" "C:/Users/DELL/SimplicityStudio/SDKs/gecko_sdk_3//protocol/z-wave/platform/SiliconLabs/PAL/lib/libzpal_zgm130s037hgn.a" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building out file: JAQUAR_ONOFF_DRY_CONTACT.out'
	arm-none-eabi-objcopy "JAQUAR_ONOFF_DRY_CONTACT.axf" "JAQUAR_ONOFF_DRY_CONTACT.out"
	@echo ' '

	@echo 'Building hex file: JAQUAR_ONOFF_DRY_CONTACT.hex'
	arm-none-eabi-objcopy -O ihex "JAQUAR_ONOFF_DRY_CONTACT.axf" "JAQUAR_ONOFF_DRY_CONTACT.hex"
	@echo ' '

	@echo 'Building bin file: JAQUAR_ONOFF_DRY_CONTACT.bin'
	arm-none-eabi-objcopy -O binary "JAQUAR_ONOFF_DRY_CONTACT.axf" "JAQUAR_ONOFF_DRY_CONTACT.bin"
	@echo ' '

	@echo 'Building s37 file: JAQUAR_ONOFF_DRY_CONTACT.s37'
	arm-none-eabi-objcopy -O srec "JAQUAR_ONOFF_DRY_CONTACT.axf" "JAQUAR_ONOFF_DRY_CONTACT.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "JAQUAR_ONOFF_DRY_CONTACT.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) JAQUAR_ONOFF_DRY_CONTACT.axf
	-@echo ' '

post-build:
	-"C:\SiliconLabs\SimplicityStudio\v5\developer\adapter_packs\commander\commander.exe" gbl create "JAQUAR_ONOFF_DRY_CONTACT.gbl" --app "JAQUAR_ONOFF_DRY_CONTACT.hex" --sign "C:/Users/DELL/SimplicityStudio/SDKs/gecko_sdk_3/protocol/z-wave/keys/Confio_sign.key" --encrypt  "C:/Users/DELL/SimplicityStudio/SDKs/gecko_sdk_3//protocol/z-wave/keys/Confio_encrypt.key"  --compress lz4
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
