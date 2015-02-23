# name of your application
APPLICATION = sensor-node

# If no BOARD is found in the environment, use this default:
BOARD ?= samr21-xpro

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../RIOT

# Uncomment this to enable scheduler statistics for ps:
#CFLAGS += -DSCHEDSTATISTICS

# Uncomment this to enable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
export QUIET ?= 1

# Modules to include:
USEMODULE += defaulttransceiver
USEMODULE += srf02
USEMODULE += vtimer

export SENSOR_NODE_SRC_ID ?= 11
export SENSOR_NODE_PAN_ID ?= 12
export CENTRAL_NODE_ADDR ?= 50
export SENSOR_NODE_CHANNEL ?= 26

# Define default settings
export SRF02_I2C ?= I2C_0
export SRF02_SPEED ?= I2C_SPEED_NORMAL
export SRF02_MODE ?= SRF02_MODE_REAL_CM

include $(RIOTBASE)/Makefile.include

ifneq (,$(SENSOR_NODE_CHANNEL))
export CFLAGS += -DSENSOR_NODE_CHANNEL=$(SENSOR_NODE_CHANNEL)
endif
ifneq (,$(SENSOR_NODE_PAN_ID))
export CFLAGS += -DSENSOR_NODE_PAN_ID=$(SENSOR_NODE_PAN_ID)
endif
ifneq (,$(SENSOR_NODE_SRC_ID))
export CFLAGS += -DSENSOR_NODE_SRC_ID=$(SENSOR_NODE_SRC_ID)
endif
ifneq (,$(CENTRAL_NODE_ADDR))
export CFLAGS += -DCENTRAL_NODE_ADDR=$(CENTRAL_NODE_ADDR)
endif
ifneq (,$(SRF02_I2C))
export CFLAGS += -DSRF02_I2C=$(SRF02_I2C)
endif
ifneq (,$(SRF02_SPEED))
export CFLAGS += -DSRF02_SPEED=$(SRF02_SPEED)
endif
ifneq (,$(SRF02_MODE))
export CFLAGS += -DSRF02_MODE=$(SRF02_MODE)
endif
