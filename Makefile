#这种写法可以用全局变量
obj-m+= st7789_lcd.o
KERNELDIR:=/root/linux-3.4.y


PWD:=$(shell pwd)

default:
	make -C $(KERNELDIR) M=$(PWD) modules

clean:
	rm -rf *.o *.order .*.cmd *.ko *.mod.c *.symvers

