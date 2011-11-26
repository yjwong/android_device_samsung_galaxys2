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
DEVICE_PACKAGE_OVERLAYS := device/samsung/galaxys2/overlay

# This device is hdpi.
PRODUCT_AAPT_CONFIG := normal hdpi
PRODUCT_AAPT_PREF_CONFIG := hdpi
PRODUCT_LOCALES += hdpi

# Init files
PRODUCT_COPY_FILES := \
	device/samsung/galaxys2/lpm.rc:root/lpm.rc \
	device/samsung/galaxys2/init.smdkv310.usb.rc:root/init.smdkv310.usb.rc \
	device/samsung/galaxys2/init.smdkc210.rc:root/init.smdkc210.rc \
	device/samsung/galaxys2/init.smdkv310.rc:root/init.smdkv310.rc \
	device/samsung/galaxys2/ueventd.smdkv310.rc:root/ueventd.smdkv310.rc

# APNs - REMOVE IF VENDOR CYANOGEN IS BACK
PRODUCT_COPY_FILES += \
	development/data/etc/apns-conf.xml:system/etc/apns-conf.xml \
	device/samsung/galaxys2/configs/spn-conf.xml:system/etc/spn-conf.xml

# Audio
# soundbooster.txt - needs to be at /data/soundbooster.txt
PRODUCT_COPY_FILES += \
	device/samsung/galaxys2/configs/soundbooster.txt:system/etc/audio/soundbooster.txt

# Touchscreen
PRODUCT_COPY_FILES += \
	device/samsung/galaxys2/configs/sec_ts_ics_bio.idc:system/usr/idc/sec_ts_ics_bio.idc

# Vold
PRODUCT_COPY_FILES += \
	device/samsung/galaxys2/configs/vold.fstab:system/etc/vold.fstab \

# Bluetooth configuration files
PRODUCT_COPY_FILES += \
	system/bluetooth/data/main.le.conf:system/etc/bluetooth/main.conf

# Wifi
PRODUCT_COPY_FILES += \
	device/samsung/galaxys2/configs/nvram_net.txt:system/etc/nvram_net.txt \
	device/samsung/galaxys2/configs/wpa_supplicant.conf:system/etc/wifi/wpa_supplicant.conf \
	device/samsung/galaxys2/configs/bcmdhd.cal:system/etc/wifi/bcmdhd.cal

PRODUCT_PROPERTY_OVERRIDES := \
	wifi.interface=eth0 \
	wifi.supplicant_scan_interval=15

$(call inherit-product-if-exists, hardware/broadcom/wlan/bcmdhd/firmware/bcm4330/device-bcm.mk)

# Gps
PRODUCT_COPY_FILES += \
	device/samsung/galaxys2/configs/gps.conf:system/etc/gps.conf \
	device/samsung/galaxys2/configs/sirfgps.conf:system/etc/sirfgps.conf

# Packages
PRODUCT_PACKAGES := \
    audio.primary.smdkv310 \
    audio_policy.smdkv310 \
    gps.smdkv310 \
    smdkv310_hdcp_keys \
    com.android.future.usb.accessory

# Charger
#PRODUCT_PACKAGES += \
#	charger \
#	charger_res_images

# Camera
PRODUCT_PACKAGES += \
	Camera

# Sensors
PRODUCT_PACKAGES += \
	lights.smdkv310 \
	sensors.smdkv310

# Ril
PRODUCT_PROPERTY_OVERRIDES += \
    ro.telephony.ril_class=samsung \
    ro.telephony.ril.v3=1 \
    ro.telephony.sends_barcount=1 \
    mobiledata.interfaces=pdp0,eth0,gprs,ppp0

# Filesystem management tools
PRODUCT_PACKAGES += \
	static_busybox \
	make_ext4fs \
	setup_fs

# Live Wallpapers
PRODUCT_PACKAGES += \
	Galaxy4 \
	HoloSpiralWallpaper \
	LiveWallpapers \
	LiveWallpapersPicker \
	MagicSmokeWallpapers \
	NoiseField \
	PhaseBeam \
	VisualizationWallpapers \
	librs_jni

# These are the hardware-specific features
PRODUCT_COPY_FILES += \
	frameworks/base/data/etc/handheld_core_hardware.xml:system/etc/permissions/handheld_core_hardware.xml \
	frameworks/base/data/etc/android.hardware.camera.flash-autofocus.xml:system/etc/permissions/android.hardware.camera.flash-autofocus.xml \
	frameworks/base/data/etc/android.hardware.camera.front.xml:system/etc/permissions/android.hardware.camera.front.xml \
	frameworks/base/data/etc/android.hardware.location.gps.xml:system/etc/permissions/android.hardware.location.gps.xml \
	frameworks/base/data/etc/android.hardware.wifi.xml:system/etc/permissions/android.hardware.wifi.xml \
	frameworks/base/data/etc/android.hardware.wifi.direct.xml:system/etc/permissions/android.hardware.wifi.direct.xml \
	frameworks/base/data/etc/android.hardware.sensor.proximity.xml:system/etc/permissions/android.hardware.sensor.proximity.xml \
	frameworks/base/data/etc/android.hardware.sensor.light.xml:system/etc/permissions/android.hardware.sensor.light.xml \
	frameworks/base/data/etc/android.hardware.sensor.gyroscope.xml:system/etc/permissions/android.hardware.sensor.gyroscope.xml \
	frameworks/base/data/etc/android.hardware.sensor.barometer.xml:system/etc/permissions/android.hardware.sensor.barometer.xml \
	frameworks/base/data/etc/android.hardware.touchscreen.multitouch.jazzhand.xml:system/etc/permissions/android.hardware.touchscreen.multitouch.jazzhand.xml \
	frameworks/base/data/etc/android.software.sip.voip.xml:system/etc/permissions/android.software.sip.voip.xml \
	frameworks/base/data/etc/android.hardware.usb.accessory.xml:system/etc/permissions/android.hardware.usb.accessory.xml \
	frameworks/base/data/etc/android.hardware.usb.host.xml:system/etc/permissions/android.hardware.usb.host.xml \
	packages/wallpapers/LivePicker/android.software.live_wallpaper.xml:system/etc/permissions/android.software.live_wallpaper.xml

PRODUCT_PROPERTY_OVERRIDES += \
	ro.opengles.version=131072 \
	hwui.render_dirty_regions=false \
	hwui.disable_vsync=true

PRODUCT_PROPERTY_OVERRIDES += \
	ro.sf.lcd_density=240

PRODUCT_TAGS += dalvik.gc.type-precise

# Set default USB interface
PRODUCT_DEFAULT_PROPERTY_OVERRIDES += \
	persist.sys.usb.config=mtp

$(call inherit-product, frameworks/base/build/phone-hdpi-512-dalvik-heap.mk)
