#~ set(BOARD px4-stm32f4ve CACHE string "" FORCE)
#~ set(FW_NAME px4-stm32f4ve.elf CACHE string "" FORCE)
#~ set(FW_PROTOTYPE px4-stm32f4ve CACHE string "" FORCE)


px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common IO px4io-v2)
#~ set(config_bl_file ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/extras/px4fmuv3_bl.bin)
#~ px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

# user-configurable UART ports
#~ set(board_serial_ports
	#~ GPS1:/dev/ttyS3
	#~ TEL1:/dev/ttyS1
	#~ TEL2:/dev/ttyS2
	#~ TEL4:/dev/ttyS6)

set(config_module_list
	#
	# Board support modules
	#
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	
	drivers/px4flow
	drivers/px4fmu
	drivers/px4io
	modules/sensors
	#~ drivers/pwm_input
	#~ drivers/mkblctrl
	
	#~ drivers/barometer/bmp280
	#~ drivers/gps
	#~ drivers/imu/l3gd20
	drivers/imu/mpu9250
	drivers/magnetometer/hmc5883

	#
	# System commands
	#
	#~ systemcmds/bl_update
	#~ systemcmds/mixer
	#~ systemcmds/param
	#~ systemcmds/perf
	#~ systemcmds/pwm
	#~ systemcmds/reboot
	#~ systemcmds/top
	#~ systemcmds/config
	#~ systemcmds/nshterm
	#~ systemcmds/ver

	#
	# Demo apps
	#
	#examples/math_demo
	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
	#~ examples/px4_simple_app

	# Tutorial code from
	# https://px4.io/dev/debug_values
	#examples/px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
	#examples/fixedwing_control

	# Hardware test
	#examples/hwtest
)
