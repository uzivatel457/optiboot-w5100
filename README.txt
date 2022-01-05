See http://sowerbutts.com/optiboot-w5100/

Please note that this has only been tested on the ATmega328P-based Sparkfun Pro
Ethernet.

There are two versions of the bootloader. The 2K bootloader is faster but
requires twice the flash memory (about 40% of which is unused). The 1K
bootloader is a little slower than RS232 at 115200bps. Both have identical
functionality. I would recommend the 1K bootloader, since it gets the job done
and uses less memory, but the 2K bootloader is there as an option if you're an
impatient person.

Some timings using a 14,930 byte program and avrdude 5.10

                 Write         Read          Total
1K ethernet      2.37s         1.83s         4.20s
2K ethernet      1.19s         0.40s         1.59s
serial           2.17s         1.59s         3.76s

To program your device with this bootloader, you will need an ISP programmer
such as the STK200 or something else that speaks SPI to the AVR chip. 

The following command will set the correct fuse flags for a 1KB bootloader and
upload the program:

avrdude -p m328p -c stk200 -P /dev/parport0 -U lfuse:w:0xff:m -U hfuse:w:0xd4:m -U efuse:w:0x05:m -U flash:w:./optiboot_atmega328.hex

The following will flash the 2K bootloader (note high fuse byte is different):

avrdude -p m328p -c stk200 -P /dev/parport0 -U lfuse:w:0xff:m -U hfuse:w:0xd2:m -U efuse:w:0x05:m -U flash:w:./optiboot_atmega328_fat.hex

On a normal boot the bootloader will flash the LED twice and then listen on the
serial port for around 500ms.

The layer 2 and 3 configuration for the device is contained in EEPROM, see the
source code or the web page for details.

To trigger the ethernet bootloader you can;

1. Reset the device and then send the letter 'N' to the serial port during this
500ms window. The device will respond with "OK" and then restart in ethernet
mode. The included script "test-netboot" does exactly this. This is intended
primarily for testing since if you can access the serial port you can
presumably just upload your program using the serial port!

2. Have your application write 0x55 to offset 0x22 in the EEPROM, and then
restart the device. This is the mode intended for production use. The following
code illustrates how to do this.

    #include <avr/eeprom.h>
    #include <avr/wdt.h>

    eeprom_write_byte((uint8_t*)0x22, 0x55); // set bootloader flag
    wdt_enable(WDTO_15MS);                   // 15ms watchdog delay
    while(1);                                // loop forever until watchdog resets us

To upload a program to the device over ethernet, simply connect to TCP port
61440 and use the STK500 protocol exactly as if you were using a serial port.
avrdude supports this mode of operation (at least, it does on Linux):

avrdude -p atmega328p -c arduino -P net:192.168.100.233:61440 -U firmware.hex

Happy hacking!

Will Sowerbutts
will@sowerbutts.com
