BSP_ROOT = BSP
APP_ROOT = APP
COMPONENTS_ROOT = Components

BSP_C_SOURCES = \
bsp_uart.c \
bsp_tasks.c  \
bsp_i2c.c \
bsp_motor.c \
bsp_vl53.c \
bsp_led.c

APP_C_SOURCES = \
app_entry.c \
Mapping/src/base.c \
Mapping/src/bfs.c \
Mapping/src/build_map.c \
Mapping/src/test.c

Components_C_SOURCES = \
algs/fifo.c \
vl53l0x/vl53l0x.c \
algs/pid.c \
openocd_rtos_helper/FreeRTOS-openocd.c

Components_C_INCLUDES = \
algs/Inc \
vl53l0x/Inc

include Components/mpu6050/component.mk

User_C_Sources = \
$(addprefix $(BSP_ROOT)/, $(BSP_C_SOURCES)) \
$(addprefix $(COMPONENTS_ROOT)/, $(Components_C_SOURCES)) \
$(addprefix $(APP_ROOT)/, $(APP_C_SOURCES))

USER_C_INCLUDES = Drivers/CMSIS/DSP/Include\
-IAPP/Inc \
-IAPP/Entries \
-IAPP/Mapping/include \
-IBSP/Inc \
$(addprefix -I$(COMPONENTS_ROOT)/, $(Components_C_INCLUDES))
