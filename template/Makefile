
ifeq ($(PARAM_FILE), )
	PARAM_FILE:=../../../Makefile.param
	include $(PARAM_FILE)
endif
DEV:=  coc_drv
DRV_SRC := tp2801_nvp6021.c  gpio_i2c.c
$(DEV)-objs = $(DRV_SRC:%.c=%.o)

obj-m := $(DEV).o

default:
	@make -C $(LINUX_ROOT) M=$(PWD) modules
	cp *.ko   /home/testuser1/nfs/

clean: 
	@make -C $(LINUX_ROOT) M=$(PWD) clean 
