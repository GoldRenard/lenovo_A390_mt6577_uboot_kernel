ifeq ($(CONFIG_THERMAL),y)
obj-$(CONFIG_THERMAL) += mtk_thermal_monitor.o
obj-$(CONFIG_THERMAL) += mtk_cooler_shutdown.o
obj-$(CONFIG_THERMAL) += mtk_cooler_backlight.o
#obj-$(CONFIG_THERMAL) += mtk_cooler_spkrvol.o
#obj-$(CONFIG_THERMAL) += mtk_cooler_dropcall.o
else
obj-y += mtk_thermal_monitor_dummy.o
endif