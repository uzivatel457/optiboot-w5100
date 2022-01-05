/* 2011-09-18: Extension to support W5100 by William R Sowerbutts <will@sowerbutts.com> */
/* See http://sowerbutts.com/optiboot-w5100/                                            */
/* Based on Optiboot -- http://code.google.com/p/optiboot/                              */

/**********************************************************/
/* Optiboot bootloader for Arduino                        */
/*                                                        */
/* Heavily optimised bootloader that is faster and        */
/* smaller than the Arduino standard bootloader           */
/*                                                        */
/* Enhancements:                                          */
/*   Fits in 512 bytes, saving 1.5K of code space         */
/*   Background page erasing speeds up programming        */
/*   Higher baud rate speeds up programming               */
/*   Written almost entirely in C                         */
/*   Customisable timeout with accurate timeconstant      */
/*                                                        */
/* What you lose:                                         */
/*   Implements a skeleton STK500 protocol which is       */
/*     missing several features including EEPROM          */
/*     programming and non-page-aligned writes            */
/*   High baud rate breaks compatibility with standard    */
/*     Arduino flash settings                             */
/*                                                        */
/* Currently supports:                                    */
/*   ATmega168 based devices (Diecimila etc)              */
/*   ATmega328P based devices (Duemilanove etc)           */
/*                                                        */
/* Does not support:                                      */
/*   ATmega1280 based devices (eg. Mega)                  */
/*                                                        */
/* Assumptions:                                           */
/*   The code makes several assumptions that reduce the   */
/*   code size. They are all true after a hardware reset, */
/*   but may not be true if the bootloader is called by   */
/*   other means or on other hardware.                    */
/*     No interrupts can occur                            */
/*     UART and Timer 1 are set to their reset state      */
/*     SP points to RAMEND                                */
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/


#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <util/delay.h>


/*
 * EEPROM layout:
 * ADDRESS _0 _1 _2 _3 _4 _5 _6 _7 _8 _9 _A _B _C _D _E _F
 * 0x0000  -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  / -- = unused (0xFF)
 * 0x0010  GG GG GG GG SS SS SS SS MM MM MM MM MM MM II II  / MM = MAC address; QQ = bootloader flag
 * 0x0020  II II QQ -- -- -- -- -- -- -- -- -- -- -- -- --  / II = IPv4 address; SS = subnet mask; GG = gateway
 *
 * OFFSET LENGTH DESCRIPTION
 *  0x10  4      IPv4 gateway address
 *  0x14  4      IPv4 subnet mask
 *  0x18  6      Ethernet MAC address
 *  0x1E  4      IPv4 address
 *  0x22  1      Bootloader flag
 *
 *  Example:
 *
 *  ADDRESS _0 _1 _2 _3 _4 _5 _6 _7 _8 _9 _A _B _C _D _E _F
 *  0x0010  c0 a8 64 01 ff ff ff 00 00 16 36 de 58 f6 c0 a8 
 *  0x0020  64 e9 55 
 *
 *  This configures IP address 192.168.100.233, subnet mask 255.255.255.0,
 *  gateway 192.168.100.1, and MAC address 00:16:36:de:58:f6. Note that the
 *  bootloader flag is set in this example.
 *
 */

#define EEPROM_GATEWAY_OFFSET                 0x10
#define EEPROM_SNMASK_OFFSET                  0x14
#define EEPROM_MAC_OFFSET                     0x18
#define EEPROM_IP_ADDR_OFFSET                 0x1E
#define EEPROM_BOOTLOADER_FLAG_OFFSET         0x22
#define EEPROM_BOOTLOADER_MAGIC_VALUE         0x55
#define EEPROM_BOOTLOADER_BORING_VALUE        0xFF

// W5100 registers
#define  W5100_MR               0x0000      /* Mode Register */
#define  W5100_GAR              0x0001      /* Gateway Address: 0x0001 to 0x0004 */
#define  W5100_SUBR             0x0005      /* Subnet mask Address: 0x0005 to 0x0008 */
#define  W5100_SHAR             0x0009      /* Source Hardware Address (MAC): 0x0009 to 0x000E */
#define  W5100_SIPR             0x000F      /* Source IP Address: 0x000F to 0x0012 */
#define  W5100_IMR              0x0016      /* Interrupt Mask Register */
#define  W5100_SKT_REG_BASE     0x0400      /* start of socket registers */
#define  W5100_SKT_OFFSET       0x0100      /* offset to each socket regester set */
#define  W5100_SKT_BASE(n)      (W5100_SKT_REG_BASE+(n*W5100_SKT_OFFSET))

// socket register offsets
#define  W5100_MR_OFFSET        0x0000      /* socket Mode Register offset */
#define  W5100_CR_OFFSET        0x0001      /* socket Command Register offset */
#define  W5100_IR_OFFSET        0x0002      /* socket Interrupt Register offset */
#define  W5100_SR_OFFSET        0x0003      /* socket Status Register offset */
#define  W5100_PORT_OFFSET      0x0004      /* socket Port Register offset (2 bytes) */
#define  W5100_DHAR_OFFSET      0x0006      /* socket Destination Hardware Address Register (MAC, 6 bytes) */
#define  W5100_DIPR_OFFSET      0x000C      /* socket Destination IP Address Register (IP, 4 bytes) */
#define  W5100_DPORT_OFFSET     0x0010      /* socket Destination Port Register (2 bytes) */
#define  W5100_MSS_OFFSET       0x0012      /* socket Maximum Segment Size (2 bytes) */
#define  W5100_PROTO_OFFSET     0x0014      /* socket IP Protocol Register */
#define  W5100_TOS_OFFSET       0x0015      /* socket Type Of Service Register */
#define  W5100_TTL_OFFSET       0x0016      /* socket Time To Live Register */
#define  W5100_TX_FSR_OFFSET    0x0020      /* socket Transmit Free Size Register (2 bytes) */
#define  W5100_TX_RR_OFFSET     0x0022      /* socket Transmit Read Pointer Register (2 bytes) */
#define  W5100_TX_WR_OFFSET     0x0024      /* socket Transmit Write Pointer Register (2 bytes) */
#define  W5100_RX_RSR_OFFSET    0x0026      /* socket Receive Received Size Register (2 bytes) */
#define  W5100_RX_RD_OFFSET     0x0028      /* socket Receive Read Pointer Register (2 bytes) */

// Device Mode Register
#define  W5100_MR_SOFTRST       (1<<7)      /* soft-reset */
#define  W5100_MR_PINGBLK       (1<<4)      /* block responses to ping request */
#define  W5100_MR_PPPOE         (1<<3)      /* enable PPPoE */
#define  W5100_MR_AUTOINC       (1<<1)      /* address autoincrement (indirect interface ONLY!) */
#define  W5100_MR_INDINT        (1<<0)      /* use indirect interface (parallel interface ONLY!) */

// Socket mode register
#define  W5100_SKT_MR_CLOSE     0x00        /* Unused socket */
#define  W5100_SKT_MR_TCP       0x01        /* TCP */
#define  W5100_SKT_MR_UDP       0x02        /* UDP */
#define  W5100_SKT_MR_IPRAW     0x03        /* IP LAYER RAW SOCK */
#define  W5100_SKT_MR_MACRAW    0x04        /* MAC LAYER RAW SOCK */
#define  W5100_SKT_MR_PPPOE     0x05        /* PPPoE */
#define  W5100_SKT_MR_ND        0x20        /* No Delayed Ack(TCP) flag */
#define  W5100_SKT_MR_MULTI     0x80        /* support multicasting */

// Socket command register
#define  W5100_SKT_CR_OPEN      0x01        /* open the socket */
#define  W5100_SKT_CR_LISTEN    0x02        /* wait for TCP connection (server mode) */
#define  W5100_SKT_CR_CONNECT   0x04        /* listen for TCP connection (client mode) */
#define  W5100_SKT_CR_DISCON    0x08        /* close TCP connection */
#define  W5100_SKT_CR_CLOSE     0x10        /* mark socket as closed (does not close TCP connection) */
#define  W5100_SKT_CR_SEND      0x20        /* transmit data in TX buffer */
#define  W5100_SKT_CR_SEND_MAC  0x21        /* SEND, but uses destination MAC address (UDP only) */
#define  W5100_SKT_CR_SEND_KEEP 0x22        /* SEND, but sends 1-byte packet for keep-alive (TCP only) */
#define  W5100_SKT_CR_RECV      0x40        /* receive data into RX buffer */

// Socket status register
#define  W5100_SKT_SR_CLOSED      0x00      /* closed */
#define  W5100_SKT_SR_INIT        0x13      /* init state */
#define  W5100_SKT_SR_LISTEN      0x14      /* listen state */
#define  W5100_SKT_SR_SYNSENT     0x15      /* connection state */
#define  W5100_SKT_SR_SYNRECV     0x16      /* connection state */
#define  W5100_SKT_SR_ESTABLISHED 0x17      /* success to connect */
#define  W5100_SKT_SR_FIN_WAIT    0x18      /* closing state */
#define  W5100_SKT_SR_CLOSING     0x1A      /* closing state */
#define  W5100_SKT_SR_TIME_WAIT   0x1B      /* closing state */
#define  W5100_SKT_SR_CLOSE_WAIT  0x1C      /* closing state */
#define  W5100_SKT_SR_LAST_ACK    0x1D      /* closing state */
#define  W5100_SKT_SR_UDP         0x22      /* UDP socket */
#define  W5100_SKT_SR_IPRAW       0x32      /* IP raw mode socket */
#define  W5100_SKT_SR_MACRAW      0x42      /* MAC raw mode socket */
#define  W5100_SKT_SR_PPPOE       0x5F      /* PPPOE socket */

// TX and RX buffers
#define  W5100_TXBUFADDR        0x4000      /* W5100 Send Buffer Base Address */
#define  W5100_RXBUFADDR        0x6000      /* W5100 Read Buffer Base Address */
#define  W5100_BUFSIZE          0x0800      /* W5100 buffers are sized 2K */
#define  W5100_TX_BUF_MASK      0x07FF      /* Tx 2K Buffer Mask */
#define  W5100_RX_BUF_MASK      0x07FF      /* Rx 2K Buffer Mask */

#define W5100_WRITE_OPCODE          0xF0
#define W5100_READ_OPCODE           0x0F

//#define LED_DATA_FLASH

#ifndef LED_START_FLASHES
#define LED_START_FLASHES 0
#endif

/* Build-time variables */
/* BAUD_RATE       Programming baud rate */
/* LED_NO_FLASHES  Number of LED flashes on boot */
/* FLASH_TIME_MS   Duration of each LED flash */
/* BOOT_TIMEOUT_MS Serial port wait time before exiting bootloader */

/* set the UART baud rate */
#ifndef BAUD_RATE
#define BAUD_RATE   19200
#endif

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
/* Onboard LED is connected to pin PB5 in Arduino NG, Diecimila, and Duemilanove */ 
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PINB
#define LED         PINB5

/* Ports for soft UART */
#ifdef SOFT_UART
#define UART_PORT   PORTD
#define UART_PIN    PIND
#define UART_DDR    DDRD
#define UART_TX_BIT 1
#define UART_RX_BIT 0
#endif
#endif

#if defined(__AVR_ATtiny84__)
/* Onboard LED is connected to pin PB5 in Arduino NG, Diecimila, and Duemilanove */ 
#define LED_DDR     DDRA
#define LED_PORT    PORTA
#define LED_PIN     PINA
#define LED         PINA4

/* Ports for soft UART - left port only for now*/
#ifdef SOFT_UART
#define UART_PORT   PORTA
#define UART_PIN    PINA
#define UART_DDR    DDRA
#define UART_TX_BIT 2
#define UART_RX_BIT 3
#endif
#endif

/* STK500 constants list, from AVRDUDE */
#define STK_OK              0x10
#define STK_FAILED          0x11  // Not used
#define STK_UNKNOWN         0x12  // Not used
#define STK_NODEVICE        0x13  // Not used
#define STK_INSYNC          0x14  // ' '
#define STK_NOSYNC          0x15  // Not used
#define ADC_CHANNEL_ERROR   0x16  // Not used
#define ADC_MEASURE_OK      0x17  // Not used
#define PWM_CHANNEL_ERROR   0x18  // Not used
#define PWM_ADJUST_OK       0x19  // Not used
#define CRC_EOP             0x20  // 'SPACE'
#define STK_GET_SYNC        0x30  // '0'
#define STK_GET_SIGN_ON     0x31  // '1'
#define STK_SET_PARAMETER   0x40  // '@'
#define STK_GET_PARAMETER   0x41  // 'A'
#define STK_SET_DEVICE      0x42  // 'B'
#define STK_SET_DEVICE_EXT  0x45  // 'E'
#define STK_ENTER_PROGMODE  0x50  // 'P'
#define STK_LEAVE_PROGMODE  0x51  // 'Q'
#define STK_CHIP_ERASE      0x52  // 'R'
#define STK_CHECK_AUTOINC   0x53  // 'S'
#define STK_LOAD_ADDRESS    0x55  // 'U'
#define STK_UNIVERSAL       0x56  // 'V'
#define STK_PROG_FLASH      0x60  // '`'
#define STK_PROG_DATA       0x61  // 'a'
#define STK_PROG_FUSE       0x62  // 'b'
#define STK_PROG_LOCK       0x63  // 'c'
#define STK_PROG_PAGE       0x64  // 'd'
#define STK_PROG_FUSE_EXT   0x65  // 'e'
#define STK_READ_FLASH      0x70  // 'p'
#define STK_READ_DATA       0x71  // 'q'
#define STK_READ_FUSE       0x72  // 'r'
#define STK_READ_LOCK       0x73  // 's'
#define STK_READ_PAGE       0x74  // 't'
#define STK_READ_SIGN       0x75  // 'u'
#define STK_READ_OSCCAL     0x76  // 'v'
#define STK_READ_FUSE_EXT   0x77  // 'w'
#define STK_READ_OSCCAL_EXT 0x78  // 'x'

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))

/* Function Prototypes */
/* The main function is in init9, which removes the interrupt vector table */
/* we don't need. It is also 'naked', which means the compiler does not    */
/* generate any entry or exit code itself. */
int main(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
void putch(char);
void ethernet_init(void);
uint8_t getch(void);
void getNch(uint8_t);
void verifySpace();
static inline void flash_led(uint8_t);
// uint8_t getLen();
void watchdogReset();
void watchdogConfig(uint8_t x);
#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart() __attribute__ ((naked));

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(0x100))
#ifdef VIRTUAL_BOOT_PARTITION
#define rstVect (*(uint16_t*)(0x204))
#define wdtVect (*(uint16_t*)(0x206))
#endif
#define ethernet_mode  (*(uint8_t*)(0x208)) // mode flag. non-zero for ethernet mode, zero for serial mode.
#ifdef FAST_BUT_FAT
#define discard ((uint8_t*)(0x210)) // junk gets written here
void recvblock(uint8_t *ptr, uint16_t length);
void sendblock(uint8_t *ptr, uint16_t length);
#endif
/* main program starts here */
int main(void) {
    // After the zero init loop, this is the first code to run.
    //
    // This code makes the following assumptions:
    //  No interrupts will execute
    //  SP points to RAMEND
    //  r1 contains zero
    //
    // If not, uncomment the following instructions:
    // cli();
    // SP=RAMEND;  // This is done by hardware reset
    asm volatile ("clr __zero_reg__");

    uint8_t ch;
    uint16_t address;
    uint8_t  length;

#ifndef SOFT_UART
    UCSR0A = _BV(U2X0); //Double speed mode USART0
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
    UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
    UBRR0L = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#endif

    // Set LED pin as output
    LED_DDR |= _BV(LED);

    if(eeprom_read_byte((uint8_t*)EEPROM_BOOTLOADER_FLAG_OFFSET) == EEPROM_BOOTLOADER_MAGIC_VALUE){
        // ethernet bootloader requested
        eeprom_write_byte((uint8_t*)EEPROM_BOOTLOADER_FLAG_OFFSET, EEPROM_BOOTLOADER_BORING_VALUE); // unset flag so we boot normally next time around
go_go_gadget_network:
        MCUSR = 0;
        watchdogConfig(WATCHDOG_8S);
        flash_led(10);
        ethernet_init();
        ethernet_mode = 1;
    }else{
        // Adaboot no-wait mod
        ch = MCUSR;
        MCUSR = 0;
        if (!(ch & _BV(EXTRF))) appStart();

        watchdogConfig(WATCHDOG_500MS);
        flash_led(4);

        // Set up watchdog to trigger after 500ms
        ethernet_mode = 0;
    }

#ifdef SOFT_UART
    /* Set TX pin as output */
    UART_DDR |= _BV(UART_TX_BIT);
#endif

    address = 0;

    /* Forever loop */
    for (;;) {
        /* get character from UART */
        ch = getch();
        if(ch == STK_GET_PARAMETER){
            // GET PARAMETER returns a generic 0x03 reply - enough to keep Avrdude happy
            getNch(1);
            putch(0x03);
        }else if(ch == STK_SET_DEVICE){
            // SET DEVICE is ignored
            getNch(20);
        }else if(ch == STK_SET_DEVICE_EXT){
            // SET DEVICE EXT is ignored
            getNch(5);
        }else if(ch == STK_LOAD_ADDRESS){
            // LOAD ADDRESS
            address = getch();
            address = (address & 0xff) | (getch() << 8);
            address += address; // Convert from word address to byte address
            verifySpace();
        }else if(ch == STK_UNIVERSAL){
            // UNIVERSAL command is ignored
            getNch(4);
            putch(0x00);
        }else if(ch == STK_PROG_PAGE){
            uint8_t *bufPtr;
            uint16_t addrPtr;
            /* Write memory, length is big endian and is in bytes  */
            // PROGRAM PAGE - we support flash programming only, not EEPROM

            // getLen();
            getch(); 
            length = getch();
            getch();

            // Immediately start page erase - this will 4.5ms
            boot_page_erase((uint16_t)(void*)address);

            // While that is going on, read in page contents
#ifdef FAST_BUT_FAT
            recvblock(buff, length);
#else
            bufPtr = buff;
            do *bufPtr++ = getch();
            while (--length);
#endif

            // Read command terminator, start reply
            verifySpace();

            // If only a partial page is to be programmed, the erase might not be complete.
            // So check that here
            boot_spm_busy_wait();

#ifdef VIRTUAL_BOOT_PARTITION
            if ((uint16_t)(void*)address == 0) {
                // This is the reset vector page. We need to live-patch the code so the
                // bootloader runs.
                //
                // Move RESET vector to WDT vector
                uint16_t vect = buff[0] | (buff[1]<<8);
                rstVect = vect;
                wdtVect = buff[10] | (buff[11]<<8);
                vect -= 4; // Instruction is a relative jump (rjmp), so recalculate.
                buff[10] = vect & 0xff;
                buff[11] = vect >> 8;

                // Add jump to bootloader at RESET vector
                buff[0] = 0x7f;
                buff[1] = 0xce; // rjmp 0x1d00 instruction
            }
#endif

            // Copy buffer into programming buffer
            bufPtr = buff;
            addrPtr = (uint16_t)(void*)address;
            ch = SPM_PAGESIZE / 2;
            do {
                uint16_t a;
                a = *bufPtr++;
                a |= (*bufPtr++) << 8;
                boot_page_fill((uint16_t)(void*)addrPtr,a);
                addrPtr += 2;
            } while (--ch);

            // Write from programming buffer
            boot_page_write((uint16_t)(void*)address);
            boot_spm_busy_wait();

#if defined(RWWSRE)
            // Reenable read access to flash
            boot_rww_enable();
#endif
        }else if(ch == STK_READ_PAGE){
            /* Read memory block mode, length is big endian.  */
            // READ PAGE - we only read flash

            // getLen();
            getch();
            length = getch();
            getch();

            verifySpace();
#ifdef VIRTUAL_BOOT_PARTITION
            do {
                // Undo vector patch in bottom page so verify passes
                if (address == 0)       ch=rstVect & 0xff;
                else if (address == 1)  ch=rstVect >> 8;
                else if (address == 10)  ch=wdtVect & 0xff;
                else if (address == 11) ch=wdtVect >> 8;
                else ch = pgm_read_byte_near(address);
                address++;
                putch(ch);
            } while (--length);
#else
#ifdef FAST_BUT_FAT
            uint8_t *ptr = buff;
            uint16_t l = length;
            do{
                *(ptr++) = pgm_read_byte_near(address++);
            }while(--length);
            sendblock(buff, l);
#else
            do putch(pgm_read_byte_near(address++));
            while (--length);
#endif
#endif
        }else if(ch == STK_READ_SIGN){
            /* Get device signature bytes  */
            // READ SIGN - return what Avrdude wants to hear
            verifySpace();
            putch(SIGNATURE_0);
            putch(SIGNATURE_1);
            putch(SIGNATURE_2);
        }else if(ch == 'Q'){
            // Adaboot no-wait mod
            watchdogConfig(WATCHDOG_16MS);
            verifySpace();
        }else if(ch == 'N'){
            // acknowledge request
            putch('O');
            putch('K');
            // WRS extension: jump back and start again in ethernet bootloader mode.
            // I would like to defend myself by saying this is the first time I've used 
            // goto in about 15 years. I feel dirty.
            goto go_go_gadget_network; 
        }else{
            // This covers the response to commands like STK_ENTER_PROGMODE
            verifySpace();
        }
        putch(STK_OK);
    }
}

uint8_t W51_xfer(uint16_t addr, uint8_t data, uint8_t opcode)
{
    PORTB &= ~(_BV(PB2));                       // Make SPI SS low
    SPDR = opcode;
    while (!(SPSR & _BV(SPIF))); // wait for SPI
    SPDR = addr >> 8;
    while (!(SPSR & _BV(SPIF))); // wait for SPI
    SPDR = addr & 0xff;
    while (!(SPSR & _BV(SPIF))); // wait for SPI
    SPDR = data;
    while (!(SPSR & _BV(SPIF))); // wait for SPI
    PORTB |= (_BV(PB2));                        // Make SPI SS high
    return SPDR;
}

void W51_write(uint16_t addr, uint8_t data)
{
    W51_xfer(addr, data, W5100_WRITE_OPCODE);
}

uint8_t W51_read(uint16_t addr)
{
    return W51_xfer(addr, 0, W5100_READ_OPCODE);
}

void W51_write16(uint16_t addr, uint16_t data)
{
    W51_write(addr, data >> 8);     // write MSB
    W51_write(addr+1, data & 0xFF); // write LSB
}

uint16_t W51_read16(uint16_t addr)
{
    uint16_t val;

    val = W51_read(addr) << 8;  // read MSB (must be read first)
    val |= W51_read(addr+1);    // read LSB

    return val;
}

void W51_execute(uint16_t cmd)
{
    W51_write(W5100_SKT_BASE(0) + W5100_CR_OFFSET, cmd);
    while(W51_read(W5100_SKT_BASE(0) + W5100_CR_OFFSET));
}

void ethernet_init(void)
{
    // prepare SPI
    PORTB |= _BV(PORTB2);                         // make sure SS is high
    DDRB = _BV(PORTB2)|_BV(PORTB3)|_BV(PORTB5);   // set MOSI, SCK and SS as output, others as input
    SPCR = _BV(SPE)|_BV(MSTR);                    // enable SPI, master mode 0
    // omitting this saves 6 bytes, and it's not like we're a high performance system...
    SPSR |= _BV(SPI2X);                           // enable SPI double speed clock

    // wait for ethernet chip to initialise
    _delay_ms(10); // datasheet says max 10ms but I have seen figures up to 300ms elsewhere.

    W51_write(W5100_MR, W5100_MR_SOFTRST);      // force the w5100 to soft-reset
    _delay_ms(10);                               // wait for chip to reset

    // use 2K buffers for RX/TX for each socket.
    // W51_write(W5100_RMSR, 0x55);  -- pointless; W5100_MR_SOFTRST does this
    // W51_write(W5100_TMSR, 0x55);  -- pointless; W5100_MR_SOFTRST does this
    
#if 0
    // this way costs a lot of instructions
    uint8_t i;
    for(i=0; i<4; i++)
        W51_write(W5100_GAR + i, eeprom_read_byte((uint8_t*)EEPROM_GATEWAY_OFFSET + i));
    
    // load subnet
    for(i=0; i<4; i++)
        W51_write(W5100_SUBR + i, eeprom_read_byte((uint8_t*)EEPROM_SNMASK_OFFSET + i));
    
    // load MAC
    for(i=0; i<6; i++)
        W51_write(W5100_SHAR + i, eeprom_read_byte((uint8_t*)EEPROM_MAC_OFFSET + i));
    
    // load IP
    for(i=0; i<4; i++)
        W51_write(W5100_SIPR + i, eeprom_read_byte((uint8_t*)EEPROM_IP_ADDR_OFFSET + i));
#else
    // I laid out the config in EEPROM to match the registers in the W5100.
    // doing it this way saves a bunch of instructions!
    uint8_t r, w;

    r = EEPROM_GATEWAY_OFFSET;
    w = W5100_GAR;
    do{
        W51_write(w, eeprom_read_byte((uint8_t*)((uint16_t)r)));
        w++;
        r++;
    }while(r < EEPROM_BOOTLOADER_FLAG_OFFSET);
#endif

    // put socket 0 into listen mode
    W51_write(W5100_SKT_BASE(0) + W5100_MR_OFFSET, W5100_SKT_MR_TCP);
    W51_write(W5100_SKT_BASE(0) + W5100_PORT_OFFSET, 0xf0); // port 0xf000 = 61440
    W51_execute(W5100_SKT_CR_OPEN);
    // ... assume success
    W51_execute(W5100_SKT_CR_LISTEN);
    // ... assume success

    // now wait for an incoming connection -- may take a while; watchdog will reset us if
    // no connection arrives.
    uint16_t a = 0xffff;
    while(a){
        watchdogReset();
        if(W51_read(W5100_SKT_BASE(0) + W5100_SR_OFFSET) == W5100_SKT_SR_ESTABLISHED)
            break;
        _delay_ms(9); // 0xffff * 9 == 589 sec
        a--;
    }
}

#ifdef FAST_BUT_FAT
void recvblock(uint8_t *ptr, uint16_t length)
{
    uint16_t buffer_ptr, i;

    watchdogReset();
    if(ethernet_mode){
        // wait for space/data in buffer
        while(W51_read16(W5100_SKT_BASE(0) + W5100_RX_RSR_OFFSET) < length);

        // get pointer into buffer
        buffer_ptr = W51_read16(W5100_SKT_BASE(0) + W5100_RX_RD_OFFSET);

        // transfer data from buffer
        for(i=0; i<length; i++){
            *(ptr++) = W51_read(W5100_RXBUFADDR + ((buffer_ptr++) & W5100_RX_BUF_MASK));
        }

        // write updated buffer pointer
        W51_write16(W5100_SKT_BASE(0) + W5100_RX_RD_OFFSET, buffer_ptr);

        // execute receive command
        W51_execute(W5100_SKT_CR_RECV);
        watchdogReset();
    }else{
        while(length--){ // uses less code than for(i=0; i<length; i++)
            while(!(UCSR0A & _BV(RXC0)));
            *(ptr++) = UDR0;
            watchdogReset();
        }
    }
}

void sendblock(uint8_t *ptr, uint16_t length)
{
    uint16_t buffer_ptr, i;

    watchdogReset();
    if(ethernet_mode){
        // wait for space/data in buffer
        while(W51_read16(W5100_SKT_BASE(0) + W5100_TX_FSR_OFFSET) < length);

        // get pointer into buffer
        buffer_ptr = W51_read16(W5100_SKT_BASE(0) + W5100_TX_WR_OFFSET);

        // transfer data to buffer
        for(i=0; i<length; i++){
            W51_write(W5100_TXBUFADDR + ((buffer_ptr++) & W5100_TX_BUF_MASK), *(ptr++));
        }

        // write updated buffer pointer
        W51_write16(W5100_SKT_BASE(0) + W5100_TX_WR_OFFSET, buffer_ptr);

        // execute transmit command
        W51_execute(W5100_SKT_CR_SEND);
        watchdogReset();
    }else{
        while(length--){ // uses less code than for(i=0; i<length; i++)
            while(!(UCSR0A & _BV(UDRE0)));
            UDR0 = *(ptr++);
            watchdogReset();
        }
    }
}
#endif

void putch(char ch) {
#ifndef SOFT_UART
    if(ethernet_mode){
#ifdef FAST_BUT_FAT
        sendblock((uint8_t*)&ch, 1);
#else
        // wait for space in buffer
        while(W51_read16(W5100_SKT_BASE(0) + W5100_TX_FSR_OFFSET) == 0);
        uint16_t tx_ptr;
        tx_ptr = W51_read16(W5100_SKT_BASE(0) + W5100_TX_WR_OFFSET);
        W51_write(W5100_TXBUFADDR + (tx_ptr & W5100_TX_BUF_MASK), ch);
        tx_ptr++;
        W51_write16(W5100_SKT_BASE(0) + W5100_TX_WR_OFFSET, tx_ptr);
        W51_execute(W5100_SKT_CR_SEND);
#endif
    }else{
        while (!(UCSR0A & _BV(UDRE0)));
        UDR0 = ch;
    }
#else
  __asm__ __volatile__ (
    "   com %[ch]\n" // ones complement, carry set
    "   sec\n"
    "1: brcc 2f\n"
    "   cbi %[uartPort],%[uartBit]\n"
    "   rjmp 3f\n"
    "2: sbi %[uartPort],%[uartBit]\n"
    "   nop\n"
    "3: rcall uartDelay\n"
    "   rcall uartDelay\n"
    "   lsr %[ch]\n"
    "   dec %[bitcnt]\n"
    "   brne 1b\n"
    :
    :
      [bitcnt] "d" (10),
      [ch] "r" (ch),
      [uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
      [uartBit] "I" (UART_TX_BIT)
    :
      "r25"
  );
#endif
}

uint8_t getch(void) {
  uint8_t ch;

  watchdogReset();

#ifdef SOFT_UART
  __asm__ __volatile__ (
    "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
    "   rjmp  1b\n"
    "   rcall uartDelay\n"          // Get to middle of start bit
    "2: rcall uartDelay\n"              // Wait 1 bit period
    "   rcall uartDelay\n"              // Wait 1 bit period
    "   clc\n"
    "   sbic  %[uartPin],%[uartBit]\n"
    "   sec\n"                          
    "   dec   %[bitCnt]\n"
    "   breq  3f\n"
    "   ror   %[ch]\n"
    "   rjmp  2b\n"
    "3:\n"
    :
      [ch] "=r" (ch)
    :
      [bitCnt] "d" (9),
      [uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
      [uartBit] "I" (UART_RX_BIT)
    :
      "r25"
);
  return ch;
#else
  if(ethernet_mode){
#ifdef FAST_BUT_FAT
      recvblock(&ch, 1);
      return ch;
#else
      while(W51_read16(W5100_SKT_BASE(0) + W5100_RX_RSR_OFFSET) == 0);
      // load single byte from ethernet device
      uint16_t rx_ptr;
      rx_ptr = W51_read16(W5100_SKT_BASE(0) + W5100_RX_RD_OFFSET);
      ch = W51_read(W5100_RXBUFADDR + (rx_ptr & W5100_RX_BUF_MASK));
      W51_write16(W5100_SKT_BASE(0) + W5100_RX_RD_OFFSET, rx_ptr+1);
      W51_execute(W5100_SKT_CR_RECV);
      return ch;
#endif
  }else{
      while(!(UCSR0A & _BV(RXC0)));
      return UDR0;
  }
#endif
}

#ifdef SOFT_UART
//#define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
  __asm__ __volatile__ (
    "ldi r25,%[count]\n"
    "1:dec r25\n"
    "brne 1b\n"
    "ret\n"
    ::[count] "M" (UART_B_VALUE)
  );
}
#endif

void getNch(uint8_t count) {
#ifdef FAST_BUT_FAT
    recvblock(discard, count);
#else
    do getch(); while (--count);
#endif
    verifySpace();
}

void verifySpace() {
    if (getch() != CRC_EOP) appStart();
    putch(STK_INSYNC);
}

void flash_led(uint8_t count) {
    do {
        _delay_ms(40);
        LED_PIN |= _BV(LED);
    } while (--count);
    watchdogReset();
}

// uint8_t getLen() {
//   getch();
//   length = getch();
//   return getch();
// }

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

void watchdogConfig(uint8_t x) {
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = x;
}

void appStart() {
  watchdogConfig(WATCHDOG_OFF);
  __asm__ __volatile__ (
#ifdef VIRTUAL_BOOT_PARTITION
    // Jump to WDT vector
    "ldi r30,5\n"
    "clr r31\n"
#else
    // Jump to RST vector
    "clr r30\n"
    "clr r31\n"
#endif
    "ijmp\n"
  );
}
