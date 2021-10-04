UK_ROOT ?= $(PWD)/../../unikraft
UK_LIBS ?= $(PWD)/../../libs
PLATS   ?= $(PWD)/../../plat-zynqmp

all:
	@$(MAKE) -C $(UK_ROOT) A=$(PWD) P=$(PLATS)

$(MAKECMDGOALS):
	@$(MAKE) -C $(UK_ROOT) A=$(PWD) P=$(PLATS) $(MAKECMDGOALS)
