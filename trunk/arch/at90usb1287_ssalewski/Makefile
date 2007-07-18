TARGET:=main

USBSTACKINCLUDE:=../../

USBMON:=1
DEBUG:=1
LIBMODE:=1


GCC:=avr-gcc -mmcu=at90usb1287 -Os -I$(USBSTACKINCLUDE) -DF_CPU=16000000 -Wall -DUSBMON=$(USBMON) -I./ -DLIBMODE=$(LIBMODE) -DDEBUG=$(DEBUG)
FILES:= main.o core.o at90usb1287-hcd.o wait.o usb.o list.o uart.o mon.o
#FILES:= main.o at90usb1287-io.o uart.o core.o mon.o sl811hs-hcd.o wait.o usb.o list.o hub.o storage.o ft232.o

main.elf: $(FILES)
	$(GCC) $(FILES) -o main.elf
	avr-objcopy main.elf -O ihex main.hex

main.o: main.c
	$(GCC) -c main.c 

wait.o: wait.c wait.h
	$(GCC) -c wait.c 

uart.o: uart.c uart.h
	$(GCC) -c uart.c 

list.o: $(USBSTACKINCLUDE)lib/list.c $(USBSTACKINCLUDE)lib/list.h
	$(GCC) -c $(USBSTACKINCLUDE)lib/list.c

core.o: $(USBSTACKINCLUDE)core/core.c $(USBSTACKINCLUDE)core/core.h
	$(GCC) -c $(USBSTACKINCLUDE)core/core.c

usb.o: $(USBSTACKINCLUDE)core/usb.c $(USBSTACKINCLUDE)core/usb.h
	$(GCC) -c $(USBSTACKINCLUDE)core/usb.c

mon.o: $(USBSTACKINCLUDE)drivers/mon/mon.c $(USBSTACKINCLUDE)drivers/mon/mon.h
	$(GCC) -c $(USBSTACKINCLUDE)drivers/mon/mon.c

hub.o: $(USBSTACKINCLUDE)drivers/class/hub.c 
	$(GCC) -c $(USBSTACKINCLUDE)drivers/class/hub.c

storage.o: $(USBSTACKINCLUDE)drivers/class/storage.c 
	$(GCC) -c $(USBSTACKINCLUDE)drivers/class/storage.c

ft232.o: $(USBSTACKINCLUDE)uclibusb/ft232.c $(USBSTACKINCLUDE)uclibusb/ft232.h 
	$(GCC) -c $(USBSTACKINCLUDE)uclibusb/ft232.c

at90usb1287-hcd.o: $(USBSTACKINCLUDE)host/at90usb1287-hcd.c $(USBSTACKINCLUDE)host/host.h
	$(GCC) -c $(USBSTACKINCLUDE)host/at90usb1287-hcd.c

download:
	avrdude -p usb1287 -c avrispv2 -P usb -U flash:w:main.hex 

reset:
	avrdude -p usb1287 -c avrispv2 -P usb

clean:
	rm -f *.o *.lst *.elf *.hex *.bin
