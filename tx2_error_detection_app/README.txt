修改启动项:
	px4:src/Firmware/ROMFS/px4fmu_common/init.d/rcS.最后一句前加入 tx2_error_detection.
	apm:mk/PX4/ROMFS/init.d/rcS (不是特别确定.)

这个脚本需要测试.
