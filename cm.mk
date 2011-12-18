## Specify phone tech before including full_phone
$(call inherit-product, vendor/cm/config/gsm.mk)

# Release name
PRODUCT_RELEASE_NAME := SGS2

# Inherit some common CM stuff.
$(call inherit-product, vendor/cm/config/common_full_phone.mk)

# Inherit device configuration
$(call inherit-product, device/samsung/galaxys2/full_galaxys2.mk)

## Device identifier. This must come after all inclusions
PRODUCT_DEVICE := galaxys2
PRODUCT_NAME := cm_galaxys2
PRODUCT_BRAND := Samsung
PRODUCT_MODEL := Galaxy S II

#Set build fingerprint / ID / Prduct Name ect.
PRODUCT_BUILD_PROP_OVERRIDES += PRODUCT_NAME=GT-I9100 TARGET_DEVICE=GT-I9100 BUILD_ID=GRJ22 BUILD_DISPLAY_ID=IML74K BUILD_FINGERPRINT=samsung/GT-I9100/GT-I9100:2.3.4/GINGERBREAD/XXKG2:user/release-keys PRIVATE_BUILD_DESC="GT-I9100-user 2.3.4 GINGERBREAD XXKG2 release-keys"
