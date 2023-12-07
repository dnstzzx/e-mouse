BSP_ROOT = BSP
APP_ROOT = APP
COMPONENTS_ROOT = Components

BSP_C_SOURCES = \
bsp_uart.c \
bsp_tasks.c  \
bsp_i2c.c \
bsp_motor.c \
bsp_vl53.c \
bsp_led.c \
bsp_5883.c \
bsp_int.c

APP_C_SOURCES = \
app_entry.c \
Mapping/src/odometer.c  \
Mapping/src/sensor.c  \
Mapping/src/action.c  \
Mapping/src/ropen.c \
Mapping/src/map.c \
Mapping/src/build_map.c

Components_C_SOURCES = \
algs/fifo.c \
algs/filters.c \
algs/vector.c \
vl53l0x/simple/vl53l0x.c \
algs/pid.c \
openocd_rtos_helper/FreeRTOS-openocd.c \
hmc5883l/hmc5883l.c

Components_C_INCLUDES = \
algs/Inc \
vl53l0x/simple/Inc \
Mapping/src \
hmc5883l

include Components/mpu6050/component.mk
#include Components/vl53l0x/component.mk

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
