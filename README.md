Define your toolchain
====================================
export PATH=~/Your_Toolchain_PATH/
for example /alps/prebuilt/linux-x86/toolchain/arm-linux-androideabi-4.4.x/bin

Build uboot
====================================
1. cd bootable/bootloader/uboot/
2. TARGET_PRODUCT=huaqin77_cu_a510_ics2 make huaqin77_cu_a510_ics2_config
3. TARGET_PRODUCT=huaqin77_cu_a510_ics2 make huaqin77_cu_a510_ics2

Build kernel
====================================
1. cd kernel/
2. TARGET_PRODUCT=huaqin77_cu_a510_ics2 MTK_ROOT_CUSTOM=../mediatek/custom/ make
