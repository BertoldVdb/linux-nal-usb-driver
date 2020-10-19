obj-m += nal.o
KDIR=/lib/modules/$(shell uname -r)


all:
	make -C $(KDIR)/build M=$(PWD) modules

clean:
	make -C $(KDIR)/build M=$(PWD) clean

install:
	mkdir -p $(KDIR)/extra/
	cp nal.ko $(KDIR)/extra/
	depmod -a
