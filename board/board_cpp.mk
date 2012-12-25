CPPFLAGS += \
	-D$(CHIP) 				\
	-D$(BOARD)				\
	-DMACH_TYPE=$(MACH_TYPE) 		\
	-DTOP_OF_MEMORY=$(TOP_OF_MEMORY)	\
	-D$(CRYSTAL)

ASFLAGS += \
	-D$(CHIP)				\
	-D$(BOARD)				\
	-DMACH_TYPE=$(MACH_TYPE) 		\
	-DTOP_OF_MEMORY=$(TOP_OF_MEMORY)	\
	-D$(CRYSTAL)

include	board/$(BOARD)/board.mk

ifeq ($(CONFIG_THUMB),y)
CPPFLAGS += -DCONFIG_THUMB -mthumb -mthumb-interwork
ASFLAGS += -DCONFIG_THUMB -mthumb-interwork
endif

ifeq ($(CONFIG_SCLK),y)
CPPFLAGS += -DCONFIG_SCLK
endif

# Crystal frequency

ifeq ($(CONFIG_CRYSTAL_12_000MHZ),y)
CPPFLAGS += -DCONFIG_CRYSTAL_12_000MHZ
endif

ifeq ($(CONFIG_CRYSTAL_16_000MHZ),y)
CPPFLAGS += -DCONFIG_CRYSTAL_16_000MHZ
endif

ifeq ($(CONFIG_CRYSTAL_16_36766MHZ),y)
CPPFLAGS += -DCONFIG_CRYSTAL_16_36766MHZ
endif

ifeq ($(CONFIG_CRYSTAL_18_432MHZ),y)
CPPFLAGS += -DCONFIG_CRYSTAL_18_432MHZ
endif

# CPU clock

ifeq ($(CONFIG_CPU_CLK_180MHZ),y)
CPPFLAGS += -DCONFIG_CPU_CLK_180MHZ
endif

ifeq ($(CONFIG_CPU_CLK_166MHZ),y)
CPPFLAGS += -DCONFIG_CPU_CLK_166MHZ
endif

ifeq ($(CONFIG_CPU_CLK_200MHZ),y)
CPPFLAGS += -DCONFIG_CPU_CLK_200MHZ
endif

ifeq ($(CONFIG_CPU_CLK_240MHZ),y)
CPPFLAGS += -DCONFIG_CPU_CLK_240MHZ
endif

ifeq ($(CONFIG_CPU_CLK_266MHZ),y)
CPPFLAGS += -DCONFIG_CPU_CLK_266MHZ
endif

ifeq ($(CONFIG_CPU_CLK_400MHZ),y)
CPPFLAGS += -DCONFIG_CPU_CLK_400MHZ
endif

ifeq ($(CONFIG_CPU_CLK_533MHZ),y)
CPPFLAGS += -DCONFIG_CPU_CLK_533MHZ
endif

# Bus speed

ifeq ($(CONFIG_BUS_SPEED_83MHZ),y)
CPPFLAGS += -DCONFIG_BUS_SPEED_83MHZ
endif

ifeq ($(CONFIG_BUS_SPEED_90MHZ),y)
CPPFLAGS += -DCONFIG_BUS_SPEED_90MHZ
endif

ifeq ($(CONFIG_BUS_SPEED_100MHZ),y)
CPPFLAGS += -DCONFIG_BUS_SPEED_100MHZ
endif

ifeq ($(CONFIG_BUS_SPEED_133MHZ),y)
CPPFLAGS += -DCONFIG_BUS_SPEED_133MHZ
endif

# other

ifeq ($(CONFIG_HAS_PIO3),y)
CPPFLAGS += -DCONFIG_HAS_PIO3
endif

ifeq ($(CPU_HAS_PMECC),y)
CPPFLAGS += -DCPU_HAS_PMECC
endif

ifeq ($(CONFIG_LOAD_ONE_WIRE), y)
CPPFLAGS += -DCONFIG_LOAD_ONE_WIRE
endif

ifeq ($(CONFIG_MMC_SUPPORT), y)
CPPFLAGS += -DCONFIG_MMC_SUPPORT
endif
