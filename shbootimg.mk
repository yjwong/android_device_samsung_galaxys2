LOCAL_PATH := $(call my-dir)

INSTALLED_BOOTIMAGE_TARGET := $(PRODUCT_OUT)/boot.img
$(INSTALLED_BOOTIMAGE_TARGET): $(TARGET_PREBUILT_KERNEL) $(recovery_ramdisk) $(INSTALLED_RAMDISK_TARGET) $(PRODUCT_OUT)/utilities/flash_image $(PRODUCT_OUT)/utilities/busybox
	$(call pretty,"Boot image: $@")
	$(hide) ./device/samsung/galaxys2/mkshbootimg.py $@ $(TARGET_PREBUILT_KERNEL) $(INSTALLED_RAMDISK_TARGET) $(recovery_ramdisk)
	cp device/samsung/galaxys2/zImage $(PRODUCT_OUT)/boot.img
	cp device/samsung/galaxys2/zImage $(PRODUCT_OUT)/recovery.img

$(INSTALLED_RECOVERYIMAGE_TARGET): $(INSTALLED_BOOTIMAGE_TARGET)
	$(ACP) $(INSTALLED_BOOTIMAGE_TARGET) $@
