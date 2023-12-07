COMPONENT_ROOT = vl53l0x
Component_C_SOURCES = 
Component_C_INCLUDES =

#include $(COMPONENTS_ROOT)/$(COMPONENT_ROOT)/st/st.mk
include $(COMPONENTS_ROOT)/$(COMPONENT_ROOT)/simple/simple.mk

Components_C_SOURCES += $(addprefix $(COMPONENT_ROOT)/, $(Component_C_SOURCES))
Components_C_INCLUDES += $(addprefix $(COMPONENT_ROOT)/, $(Component_C_INCLUDES))