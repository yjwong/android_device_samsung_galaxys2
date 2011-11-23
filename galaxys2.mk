#
# Copyright (C) 2011 The Android Open-Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

$(call inherit-product, device/samsung/galaxys2/galaxys2_base.mk)

# Keylayout
PRODUCT_COPY_FILES += \
	device/samsung/galaxys2/keylayout/AVRCP.kl:/system/usr/keylayout/AVRCP.kl \
	device/samsung/galaxys2/keylayout/Broadcom_Bluetooth_HID.kl:/system/usr/keylayout/Broadcom_Bluetooth_HID.kl \
	device/samsung/galaxys2/keylayout/sec_jack.kl:/system/usr/keylayout/sec_jack.kl \
	device/samsung/galaxys2/keylayout/sec_key.kl:/system/usr/keylayout/sec_key.kl \
	device/samsung/galaxys2/keylayout/sec_touchkey.kl:/system/usr/keylayout/sec_touchkey.kl

# kernel modules for ramdisk
RAMDISK_MODULES := $(addprefix device/samsung/galaxys2/modules/,bthid.ko dhd.ko gspca_main.ko j4fs.ko \
	scsi_wait_scan.ko Si4709_driver.ko vibrator.ko)
PRODUCT_COPY_FILES += $(foreach module,\
	$(RAMDISK_MODULES),\
	$(module):root/lib/modules/$(notdir $(module)))

# other kernel modules not in ramdisk
PRODUCT_COPY_FILES += $(foreach module,\
	$(filter-out $(RAMDISK_MODULES),$(wildcard device/samsung/galaxys2/modules/*.ko)),\
	$(module):system/lib/modules/$(notdir $(module)))

# kernel modules for recovery ramdisk
PRODUCT_COPY_FILES += \
    device/samsung/galaxys2/modules/j4fs.ko:recovery/root/lib/modules/j4fs.ko

# the kernel itself
PRODUCT_COPY_FILES += \
    device/samsung/galaxys2/kernel:kernel

$(call inherit-product, frameworks/base/build/phone-hdpi-512-dalvik-heap.mk)
$(call inherit-product-if-exists, vendor/samsung/galaxys2/galaxys2-vendor.mk)
