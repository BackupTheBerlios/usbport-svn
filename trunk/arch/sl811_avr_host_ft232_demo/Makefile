TARGET:=main

USBSTACKINCLUDE:=../../

USBMON:=1
DEBUG:=0
LIBMODE:=1



GCC:=avr-gcc -mmcu=atmega32 -Os -I$(USBSTACKINCLUDE) -DF_CPU=16000000 -Wall -DUSBMON=$(USBMON) -I./ -DLIBMODE=$(LIBMODE)
FILES:= main.o sl811hs-io.o uart.o core.o mon.o sl811hs-hcd.o wait.o usb.o list.o ft232.o

main.elf: $(FILES)
	$(GCC) $(FILES) -o main.elf
	avr-objcopy main.elf -O ihex main.hex

main.o: main.c
	$(GCC) -c main.c 

wait.o: wait.c wait.h
	$(GCC) -c wait.c 

uart.o: uart.c uart.h
	$(GCC) -c uart.c 

sl811hs-io.o: sl811hs-io.c sl811hs-io.h
	$(GCC) -c sl811hs-io.c

list.o: $(USBSTACKINCLUDE)lib/list.c $(USBSTACKINCLUDE)lib/list.h
	$(GCC) -c $(USBSTACKINCLUDE)lib/list.c

core.o: $(USBSTACKINCLUDE)core/core.c $(USBSTACKINCLUDE)core/core.h
	$(GCC) -c $(USBSTACKINCLUDE)core/core.c

usb.o: $(USBSTACKINCLUDE)core/usb.c $(USBSTACKINCLUDE)core/usb.h
	$(GCC) -c $(USBSTACKINCLUDE)core/usb.c

mon.o: $(USBSTACKINCLUDE)drivers/mon/mon.c $(USBSTACKINCLUDE)drivers/mon/mon.h
	$(GCC) -c $(USBSTACKINCLUDE)drivers/mon/mon.c

#hub.o: $(USBSTACKINCLUDE)drivers/class/hub.c 
#	$(GCC) -c $(USBSTACKINCLUDE)drivers/class/hub.c

#storage.o: $(USBSTACKINCLUDE)drivers/class/storage.c 
#	$(GCC) -c $(USBSTACKINCLUDE)drivers/class/storage.c

ft232.o: $(USBSTACKINCLUDE)uclibusb/ft232.c $(USBSTACKINCLUDE)uclibusb/ft232.h 
	$(GCC) -c $(USBSTACKINCLUDE)uclibusb/ft232.c

sl811hs-hcd.o: $(USBSTACKINCLUDE)host/sl811hs-hcd.c $(USBSTACKINCLUDE)host/host.h
	$(GCC) -c $(USBSTACKINCLUDE)host/sl811hs-hcd.c

download:
	avrdude -p m32 -c avrispv2 -P usb -U flash:w:main.hex 

reset:
	avrdude -p m32 -c avrispv2 -P usb

clean:
	rm -f *.o *.lst *.elf *.hex *.bin
