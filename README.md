# optiboot-w5100

AVR Bootloader with Wiznet W5100 ethernet support
Introduction

I've extended the Optiboot bootloader to add support for interfacing to the bootloader over TCP/IP as well as through the serial port.

I have a Sparkfun Ethernet Pro board, which is basically an Arduino Pro board combined with an Arduino Ethernet shield on a single board. It's based on an Atmel ATmega328P microcontroller and a Wiznet W5100 chip. It's a great little board, and it's a lot of fun to program a system which is both large enough to do real work and yet small enough that one can reasonably expect to learn how every piece of hardware works.

I have received reports that the Arduino Uno with the original Ethernet Shield is indeed compatible with this software. I have also used this bootstrap on an Arduino Ethernet board successfully.

I've never used the Arduino software suite. I use the excellent avr-libc together with avr-binutils and avr-gcc. It seems that the Arduino project uses these same tools so perhaps this will be useful to Arduino users too.

Sparkfun Pro Ethernet board

ATmega microcontrollers are typically programmed through SPI, but to make development more convenient the top part of flash memory (up to 4KB on this model) can be programmed with a special "bootloader" program that runs on reset before the main application and can reprogram the rest of the flash memory when required. On Arduino boards this typically contains bootloader that supports loading the program through the serial port, using the "STK500" protocol. The RESET line of the microcontroller is tied to the DTR pin on the serial port, so it's very convenient for the developer's workstation to reset the microcontroller into the bootloader, and then send it a new program.

This is great when you have access to the serial port on the board, but if you've deployed a bunch of these and want to update the firmware on them all it could be something of a pain to go and physically touch each board. I have exactly this situation with a project I am working on for a client, and my solution to this is to extend the bootloader with support for loading the program over TCP/IP.

For (hopefully obvious!) security reasons I have not made it possible to reset the AVR into ethernet bootloader mode without the co-operation of the running application. The application is responsible for authenticating any such request before executing it.

The Wiznet W5100 is a remarkable little chip. It combines a 10/100Mbit full duplex ethernet interface with a full hardwired TCP/IP stack and 8KB of internal data buffers all accessed over the 4-wire SPI protocol. The device is limited to four concurrent TCP or UDP conversations, but this is more than enough for most embedded applications. The device does not support IPv6, only IPv4. Because so much of the network stack is implemented on the chip, the interface to the microcontroller is sufficiently simple that it can be implemented in very little code.
Operation

My bootloader is based on Optiboot and in most circumstances it behaves exactly as Optiboot does. There are three main changes;

    The bootloader is 1024 bytes rather than 512 bytes. The additional flash memory is required for the extra code to support the W5100. There is also a 2048 byte version which is faster but has otherwise identical functionality.
    A new STK500 command (operation code "N") causes the bootloader to respond with a two-byte acknowledgement ("OK") and then restart itself in ethernet mode. This is useful primarily for testing the bootloader when you have access to the serial port.
    The EEPROM contains all the configuration required for ethernet support. The MAC address, IP address, subnet mask and gateway are stored in EEPROM. In addition, one byte is a flag used to indicate that when it is next run the bootloader should listen for a programmer connecting over TCP/IP rather than on the serial port. 

You can upload programs through the serial port exactly as you normally would, because without a special flag in EEPROM the serial port is used in preference to the ethernet port. This means that you can develop your application exactly as you normally would.

To trigger the ethernet mode of the bootloader, you have two options:

    Reset the device and then send the letter 'N' to the serial port during this 500ms window. The device will respond with "OK" and then restart in ethernet mode. This is intended primarily for testing since if you can access the serial port you can presumably just upload your program using the serial port!
    Have your application write 0x55 to offset 0x22 in the EEPROM, and then restart the device. This is the mode intended for production use. 

Once in ethernet mode, you can use avrdude to program an updated application over TCP/IP. The device will listen for a connection for about ten minutes. If no connection is received it will reboot the device and run the application normally. The device listens on TCP port 61440, so you want to do something like this (replace "192.168.100.233" with the IP of your device):

avrdude -p atmega328p -c arduino -P net:192.168.100.233:61440 -U firmware.hex

After a connection is opened, the device will reboot itself if no traffic is received for around 8 seconds.

When entering the bootloader in serial mode, the LED connected to pin B5 (labelled "L" on the Sparkfun board) flashes twice. When entering the bootloader in ethernet mode, the LED flashes five times. The LED flashes slightly faster than it does in the normal Optiboot bootloader; just think of the milliseconds you'll save.
Application code

To support remote updating, you need to modify your application so it can accept a command over the network and boot itself into the bootloader in ethernet mode. I assume that your application already implements some sort of network protocol, so you just need to add a new command and ensure that only privileged operators can execute it.

When the bootloader starts up, it checks a flag byte at address 0x22 in EEPROM. If this byte contains value 0x55, it rewrites the EEPROM byte to 0xFF (so the bootloader will run in the normal serial mode again next time). It then initialises the ethernet interface using the configuration in EEPROM and waits for an incoming TCP/IP connection. It will wait for about ten minutes before giving up and rebooting.

The application code to achieve this is very simple:

#include <avr/eeprom.h>
#include <avr/wdt.h>

void reboot_to_ethernet_bootloader(void)
{
    eeprom_write_byte((uint8_t*)0x22, 0x55); // set bootloader flag
    wdt_enable(WDTO_15MS);                   // 15ms watchdog delay
    while(1);                                // loop forever until watchdog resets us
}

EEPROM layout

The configuration is stored in EEPROM between addresses 0x10 and 0x22. The ordering of fields in the EEPROM may seem a bit random but this was selected to match the ordering of registers in the W5100 and thus reduce the bootloader size by allowing the full configuration to be loaded by a single loop.

ADDRESS  0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
0x0000  -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
0x0010  GG GG GG GG SS SS SS SS MM MM MM MM MM MM II II
0x0020  II II BB -- -- -- -- -- -- -- -- -- -- -- -- --

    -- indicates an unused byte
    BB: address 0x22, 1 byte: Bootloader flag (0x55: ethernet mode, anything else: serial mode)
    MM: address 0x18, 6 bytes: MAC address of the device
    II: address 0x1E, 4 bytes: IPv4 address of the device
    SS: address 0x14, 4 bytes: Subnet mask of the device
    GG: address 0x10, 4 bytes: Gateway address 

Note that, like optiboot, this bootloader does not support programming the EEPROM. The simple solution to this problem is to either use an ISP programmer, or write a small program that programs the EEPROM for you, upload that and run it. Here's one I made earlier; it's not pretty, but it works.
Faster programming option

There are two versions of my bootloader. The first fits into 1KB flash and is about 20% faster than using RS232 at 115,200bps to reprogram the device. The second fits into 2KB flash and is about twice as fast as the 1KB version.

Both versions are built from a single source code file and they have identical functionality. The only difference is the amount of flash memory that they use and the speed of programming. The reason for the difference is that the 1K bootloader always reads or writes one byte at a time from the W5100, while the 2K bootloader reads or writes in larger blocks when it can. The additional code required to do block reads and writes pushes us over the 1KB limit.

My intention is primarily to produce a bootloader that I can use to upgrade devices once they have been deployed in the field, which should be a very rare operation, and so I recommend the 1KB bootloader since it gets the job done at a reasonable speed and leaves a little more flash memory free for my program. The 2KB version is there as an option if you're a complete speed freak.

I suspect with some clever optimising it would be possible to fit the block read/write code into the 1KB bootloader. Patches welcome!

Some example timings using a 14,930 byte program and avrdude 5.10:

	Write	Read	Total
1KB Ethernet	1.78s	1.24s	3.02s
2KB Ethernet	1.15s	0.28s	1.43s
Serial 115200bps	2.17s	1.59s	3.76s
Download

    Source code (last updated 2011-09-22; includes compiled hex files) 

I release my changes to Optiboot under the GPL 2 license.

To rebuild the bootloader, use the build script included with the source. This will build both the 1KB and 2KB ("fat") bootloaders.

To program the bootloader you need an ISP programmer (ie one that talks to the device over SPI; you cannot reprogram the bootloader using a bootloader). To program the flash and set the fuses correctly for a 1KB bootloader on an ATmega328P with my STK200 programmer I use:

avrdude -p m328p -c stk200 -P /dev/parport0 \
        -U lfuse:w:0xff:m -U hfuse:w:0xd4:m -U efuse:w:0x05:m \
        -U flash:w:./optiboot_atmega328.hex

The command to flash the 2KB bootloader is in the README file inside the source.

Note that on the Sparkfun board the W5100 always drives the SPI port which may interfere with programming the ATmega; you need to pull the SEN pin low on the W5100 to make it to tri-state. On the Sparkfun board there is a solder jumper on the rear of the device (labelled "PROG") which does exactly this. You need to close this jumper, flash the bootloader, then open the jumper again. I found that in practice my 5V programmer was strong enough to override the signals from the W5100 without closing the jumper ... but it's probably not optimal for the health of the hardware.
Caveats

If you've not flashed a bootloader before, backup your existing bootloader (flash and fuse settings) first. You can use avrdude to read out the contents of the flash and fuse memories first, and program them back later if it all goes wrong (to program them back, change the ":r:" for ":w:" in each command):

avrdude -p m328p -c stk200 -P /dev/parport0 -U flash:r:flashrom.hex:i
avrdude -p m328p -c stk200 -P /dev/parport0 -U eeprom:r:eeprom.hex:i
avrdude -p m328p -c stk200 -P /dev/parport0 -U lfuse:r:lfuse.hex:i
avrdude -p m328p -c stk200 -P /dev/parport0 -U efuse:r:efuse.hex:i
avrdude -p m328p -c stk200 -P /dev/parport0 -U hfuse:r:hfuse.hex:i

When in bootloader mode, the device will accept the first incoming connection it gets on port 61440. If this is from someone other than you, you lose. Don't use this on public networks.

If you're going to reprogram a remote device, be sure that the code actually works before you program it. Because the bootloader only enters ethernet mode when the flag in EEPROM is configured appropriately, and it turns off the flag immediately, and the application is the only thing that can write to that EEPROM, you need a working application to accept the command to enter remote programming mode. If you flash a broken application, you'll need to go and reprogram the device over serial or using the ISP header.

If you break your hardware, you get to keep both bits.

This software comes with NO WARRANTY, explicit or implied.
Further extensions

The code fits neatly into 1KB; there's 34 bytes free for 17 additional instructions. There is ample space (over 800 bytes) free inside the 2KB bootloader.

If you want to free up a little space in the 1KB bootloader, my suggestion is to get rid of the code for flashing the LED and configuring pin B5 as an output—this will save you about 36 bytes. Then I would get rid of the new 'N' bootloader command, which is really only useful for testing. After that you're on your own to search for further optimisations.

One improvement I have considered is having the device open a connection to an address and port configured in EEPROM rather than listening for an incoming connection. This largely resolves the present security risk whereby there is a race to connect to the TCP port on the device. Unfortunately avrdude doesn't support listening for an incoming connection, as far as I can see. If you fancy patching avrdude then I suspect there would be just about enough space to jam the additional code into the 1KB version. There is certainly space in the 2KB version.
Feedback

Get in touch and let me know if it works for you!

Happy hacking!
Acknowledgements

Ethernet Pro board picture shamelessly borrowed from sparkfun.com. This is the first Sparkfun product I've bought, and I suspect it will not be the last.

Thanks to Peter Knight and others for their work on Optiboot.

Thanks to Karl Lunt for his W5100 page which provides a good introduction to programming the chip. 
