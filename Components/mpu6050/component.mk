COMPONENT_ROOT = mpu6050
Component_C_SOURCES = 
Component_C_INCLUDES =

include $(COMPONENTS_ROOT)/$(COMPONENT_ROOT)/basic/basic.mk

Components_C_SOURCES += $(addprefix $(COMPONENT_ROOT)/, $(Component_C_SOURCES))
USER_C_INCLUDES += $(addprefix -I$(COMPONENT_ROOT)/, $(Component_C_INCLUDES))