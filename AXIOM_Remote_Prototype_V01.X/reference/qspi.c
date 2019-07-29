
/*	PIC32MZ Demo Code
**
**	Copyright (C) 2019 Herbert Poetzl
**
**	This program is free software; you can redistribute it and/or modify
**    	it under the terms of the GNU General Public License 2 as published 
**	by the Free Software Foundation.
**
**
**	D2  GND  D1  #CS  GND		#WP GND  SO  #CS  GND
**	---------------------		---------------------
**      D0  GND  SCK  D3  GND		SI  GND  SCK #HLD GND
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

	// DEVCFG0
#pragma config BOOTISA = MIPS32
#pragma config ICESEL = ICS_PGx1
#pragma config FECCCON = OFF_UNLOCKED
#pragma config FSLEEP = 0

	// DEVCFG1
#pragma	config FDMTEN = OFF
#pragma	config FWDTEN = OFF
#pragma config POSCMOD = OFF
#pragma config OSCIOFNC = ON
#pragma config FSOSCEN = OFF
#pragma config FNOSC = SPLL
#pragma config FCKSM = CSECMD

	// DEVCFG2
#pragma config FPLLICLK = PLL_FRC
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLRNG = RANGE_5_10_MHZ
#pragma config FPLLMULT = MUL_100
#pragma config FPLLODIV = DIV_4
#pragma config UPLLEN = OFF
#pragma config UPLLFSEL = FREQ_24MHZ

	// DEVCFG3
#pragma config USERID = 0xC0DE
#pragma config FMIIEN = OFF
#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF
#pragma config FUSBIDIO = OFF

	// DEVCP0
#pragma config CP = OFF



#define SQI_D0		PORTGbits.RG13
#define SQI_D0_O	LATGbits.LATG13
#define SQI_D0_T	TRISGbits.TRISG13

#define SQI_D1		PORTGbits.RG12
#define SQI_D1_O	LATGbits.LATG12
#define SQI_D1_T	TRISGbits.TRISG12

#define SQI_D2		PORTGbits.RG14
#define SQI_D2_O	LATGbits.LATG14
#define SQI_D2_T	TRISGbits.TRISG14

#define SQI_D3		PORTAbits.RA7
#define SQI_D3_O	LATAbits.LATA7
#define SQI_D3_T	TRISAbits.TRISA7

#define SQI_CLK_O	LATAbits.LATA6
#define SQI_CLK_T	TRISAbits.TRISA6

#define SQI_CS0_O	LATDbits.LATD4
#define SQI_CS0_T	TRISDbits.TRISD4



static inline
void	unlock(void)
{
	SYSKEY = 0xAA996655;
	SYSKEY = 0x556699AA;
}

static inline
void	lock(void)
{
	SYSKEY = 0x33333333;
}

static inline
void	irq_disable(void)
{
	asm volatile("di");
	asm volatile("ehb");
}

static inline
void	irq_enable(void)
{
	asm volatile("ei");
}


void	init_pbus(void)
{
	unlock();
	PB2DIVbits.PBDIV = 0b000001;	// divide by 2
	PB2DIVbits.ON = 1;

	PB7DIVbits.PBDIV = 0b000000;	// divide by 1
	PB7DIVbits.ON = 1;

	REFO2CONbits.RODIV = 0;		// divide by ?
	REFO2CONbits.ROSEL = 0b0011;	// FRC
	REFO2CONbits.ON = 1;		
	lock();
}



void	init_uart2(void)
{
	irq_disable();

	U2MODEbits.ON = 0;

	TRISEbits.TRISE8 = 0;		// U2TX out
	TRISEbits.TRISE9 = 1;		// U2RX in
	ANSELEbits.ANSE8 = 0;		// digital
	ANSELEbits.ANSE9 = 0;		// digital

	CFGCONbits.IOLOCK = 0;
	RPE8Rbits.RPE8R = 0b0010;	// U2TX
	U2RXRbits.U2RXR = 0b1101;	// RPE9
	CFGCONbits.IOLOCK = 1;

	IPC36bits.U2TXIP = 0b000;	//! Interrupt priority of 7
	IPC36bits.U2TXIS = 0b00;	//! Interrupt sub-priority of 0
	IPC36bits.U2RXIP = 0b111;	//! Interrupt priority of 7
	IPC36bits.U2RXIS = 0b00;	//! Interrupt sub-priority of 0

	IEC4SET = _IEC4_U2RXIE_MASK;	//! Rx INT Enable
	IFS4bits.U2TXIF = 0;		//! Clear Tx flag
	IFS4bits.U2RXIF = 0;		//! Clear Rx flag

	U2BRG = 24;			// 1MBaud @ 50MHz
	U2STA = 0;
	
	U2MODEbits.BRGH = 1;
	U2MODEbits.PDSEL = 0b00;
	U2MODEbits.STSEL = 0;
	U2MODEbits.UEN = 0b00;
	U2MODEbits.ON = 1;
	U2STASET = 0x9400;		// Enable Transmit and Receive

	irq_enable();
}


static inline
void	uart2_ch(char ch)
{
	while (U2STAbits.UTXBF);
	U2TXREG = ch;
}

static inline
void	uart2_hex(uint8_t hex)
{
	hex &= 0xF;
	if (hex > 9)
	    uart2_ch(hex + 'A' - 10);
	else
	    uart2_ch(hex + '0');
}

static inline
void	uart2_byte(uint8_t val)
{
	uart2_hex(val >> 4);
	uart2_hex(val);
}

static inline
void	uart2_word(uint16_t val)
{
	uart2_byte(val >> 8);
	uart2_byte(val);
}

static inline
void	uart2_long(uint32_t val)
{
	uart2_word(val >> 16);
	uart2_word(val);
}

static inline
void	uart2_bin(uint32_t val, unsigned cnt)
{
	val <<= (32 - cnt);

	while (cnt--) {
	    uart2_ch((val & (1 << 31)) ? '1' : '0');
	    val <<= 1;
	}
}

static inline
void	uart2_str0(const char *str)
{
	while (*str)
	    uart2_ch(*str++);
}


static inline
void	delay(unsigned cnt)
{
	unsigned i;
	while (cnt--)
	    for (i=0; i<200; i++);
}



static 
uint32_t spi_trans(uint32_t val, unsigned cnt)
{
	val <<= (32 - cnt);

	while (cnt > 0) {
	    SQI_CLK_O = 0;
	    SQI_D0_O = (val & (1 << 31)) ? 1: 0;
	    delay(10);

	    SQI_CLK_O = 1;
	    val <<= 1;
	    val |= (SQI_D1) ? 1 : 0;
	    delay(10);
	
	    cnt--;
	}

	return val;	
}


static inline
uint32_t sqi_intstat(void)
{
	uint32_t stat = SQI1INTSTAT;

	uart2_ch('{');
	uart2_bin(stat, 11);
	uart2_ch('}');

	return stat;
}	

static inline
uint32_t sqi_stat1(void)
{
	uint32_t stat = SQI1STAT1;

	uart2_ch('<');
	uart2_byte(stat >> 16);
	uart2_ch(',');
	uart2_byte(stat);
	uart2_ch('>');

	return stat;
}	

static inline
uint32_t sqi_stat2(void)
{
	uint32_t stat = SQI1STAT2;

	uart2_ch('[');
	uart2_bin(stat, 8);
	uart2_ch(']');

	return stat;
}	

static inline
uint8_t sqi_rxcnt(unsigned min)
{
	uint8_t rxcnt = 0;
 
	while (1) { 
	    rxcnt = sqi_stat1() & 0xFF;
	    if (rxcnt >= min)
		break;
	    delay(5);
	}

	return rxcnt;
}



void __attribute__((vector(_UART2_RX_VECTOR), interrupt(IPL7SRS), nomips16)) uart2_isr(void)
{
	while (U2STAbits.URXDA) {	// process buffer 
	    char ch = U2RXREG;

	    uart2_ch(ch);	// echo back
	}
       
	IFS4CLR = _IFS4_U2RXIF_MASK;	// clear UART2 Rx IRQ
}


void __attribute__((vector(_SQI1_VECTOR), interrupt(IPL5AUTO), nomips16)) sqi_isr(void)
{
	uart2_ch('@');	
	IFS5CLR = _IFS5_SQI1IF_MASK;	// clear SQI Interrupt
}


int	main(void)
{
	init_pbus();
	init_uart2();

	delay(1000);
	uart2_str0("\f\n\rAXIOM Remote SQI v0.1\n\r");

	SQI1CFG = 0;
	// SQI1CFGbits.SQIEN = 0;
	// SQI1CFGbits.CSEN = 0b00;

	SQI_CS0_O = 1;

	SQI_D2_O = 1;	// WP = high
	SQI_D3_O = 1;	// #HOLD = high

	//

	SQI_D0_T = 0;	// DO
	SQI_D1_T = 1;	// DI

	SQI_D2_T = 0;	// WP
	SQI_D3_T = 0;	// #HOLD

	SQI_CLK_T = 0;
	SQI_CS0_T = 0;


	uart2_str0("\n\rSQI bitbang\n\r");
	delay(1000);


	uint32_t val;


	/* read MF/DEV id */

	SQI_CS0_O = 0;	// #CS low
	
	val = spi_trans(0x90000000, 32);
	uart2_long(val);
	uart2_ch(';');

	val = spi_trans(0x0000, 16);
	uart2_word(val);
	uart2_str0("\n\r");

	SQI_CS0_O = 1;	// #CS high


	/* read jedc id */

	SQI_CS0_O = 0;	// #CS low
	
	val = spi_trans(0x9F000000, 32);
	uart2_long(val);
	uart2_str0("\n\r");

	SQI_CS0_O = 1;	// #CS high



	delay(1000);
	uart2_str0("\n\rSQI controller\n\r");
	delay(1000);


	SQI1CFGbits.SQIEN = 1;		// enable SQI
	SQI1CFGbits.RESET = 1;		// reset SQI

	SQI1CLKCONbits.CLKDIV = 16;	// divide by 64
	SQI1CLKCONbits.EN = 1;		// enable clock

	while (!SQI1CLKCONbits.STABLE)
	    uart2_ch('.');
	uart2_str0(" clock stable.\n\r");

	SQI1CFGbits.CSEN = 0b01;	// CS0 is used
	SQI1CFGbits.DATAEN = 0b00;	// ID0 only
	SQI1CFGbits.HOLD = 0;		// disable HOLD
	SQI1CFGbits.WP = 1;		// enable WP
	SQI1CFGbits.RXLATCH = 0;	// no RX on TX
	SQI1CFGbits.LSBF = 0;		// MSB first
	SQI1CFGbits.CPOL = 1;		// Mode 3
	SQI1CFGbits.CPHA = 1;		// Mode 3
	SQI1CFGbits.MODE = 0b001;	// CPU PIO Mode

	SQI1INTSEN = 0;			// disable IRQs
	SQI1INTEN = ~0;			// enable flags


	SQI1THR = 4;			// Threshold
	SQI1INTTHRbits.TXINTTHR = 31;	// Transmit IRQ
	SQI1INTTHRbits.RXINTTHR = 31;	// Receive IRQ



	uart2_str0("\n\rTX-1, RX-3 ...\n\r");

	SQI1CMDTHRbits.TXCMDTHR = 4;		// Transmit
	SQI1CON = (0 << 22) | (1 << 16) | 1;	// Transmit 1 Byte

	*(uint8_t *)&SQI1TXBUF = 0x9F;		// JTAG ID	

	SQI1CMDTHRbits.RXCMDTHR = 3;		// Receive
	SQI1CON = (1 << 22) | (2 << 16) | 3;	// Receive 3 Byte

	for (int i=0; i<3; i++) {
	    sqi_rxcnt(1);

	    val = *(uint8_t *)&SQI1RXBUF;
	    uart2_str0(" = ");
	    uart2_byte(val);
	    uart2_ch(' ');
	}

	uart2_str0("\n\r");
	delay(1000);



	uart2_str0("\n\rTX-1 + RX-4 ...\n\r");

	SQI1CFGbits.RXLATCH = 1;		// RX on TX
	SQI1CMDTHRbits.TXCMDTHR = 4;		// Transmit
	SQI1CMDTHRbits.RXCMDTHR = 4;		// Receive
	SQI1CON = (0 << 22) | (1 << 16) | 1;	// Transmit 1 Byte
	SQI1CON = (1 << 22) | (2 << 16) | 3;	// Receive 3 Byte

	*(uint8_t *)&SQI1TXBUF = 0x9F;		// JTAG ID	

	sqi_rxcnt(4);
	sqi_intstat();
	val = *(uint32_t *)&SQI1RXBUF;
	uart2_str0(" => ");
	uart2_long(val);

	uart2_str0("\n\r");
	delay(1000);
	


	uart2_str0("\n\rEnable Quad ...\n\r");

	SQI1CFGbits.RXLATCH = 0;		// no RX on TX
	SQI1CMDTHRbits.TXCMDTHR = 1;		// Transmit
	SQI1CON = (1 << 22) | (0 << 18) | (1 << 16) | 1;
	*(uint8_t *)&SQI1TXBUF = 0x50;		// Enable Vol

	SQI1CFGbits.RXLATCH = 0;		// no RX on TX
	SQI1CMDTHRbits.TXCMDTHR = 2;		// Transmit
	SQI1CON = (1 << 22) | (0 << 18) | (1 << 16) | 2;
	*(uint16_t *)&SQI1TXBUF = 0x0231;	// SR2 = QE

	SQI1CMDTHRbits.TXCMDTHR = 1;		// Transmit
	SQI1CMDTHRbits.RXCMDTHR = 1;		// Receive
	SQI1CON = (0 << 22) | (0 << 18) | (1 << 16) | 1;
	SQI1CON = (1 << 22) | (0 << 18) | (2 << 16) | 1;
	*(uint8_t *)&SQI1TXBUF = 0x35;		// Read SR2
	
	sqi_rxcnt(1);
	val = *(uint8_t *)&SQI1RXBUF;
	uart2_str0(" => ");
	uart2_byte(val);

	uart2_str0("\n\r");
	delay(1000);


	uart2_str0("\n\rMF/Dev ID (Single)...\n\r");

	SQI1CFGbits.DATAEN = 0b00;		// ID0 
	SQI1CFGbits.RXLATCH = 0;		// no RX on TX
	SQI1CMDTHRbits.TXCMDTHR = 4;		// Transmit
	SQI1CMDTHRbits.RXCMDTHR = 2;		// Receive
	SQI1CON = (0 << 22) | (0 << 18) | (1 << 16) | 4;
	SQI1CON = (1 << 22) | (0 << 18) | (2 << 16) | 2;

	*(uint32_t *)&SQI1TXBUF = 0x00000090;	// Dev ID	

	sqi_rxcnt(2);
	sqi_intstat();
	val = *(uint16_t *)&SQI1RXBUF;
	uart2_str0(" => ");
	uart2_word(val);

	uart2_str0("\n\r");
	delay(1000);



	uart2_str0("\n\rMF/Dev ID (Dual)...\n\r");

	SQI1CFGbits.DATAEN = 0b01;		// ID0-ID1
	SQI1CFGbits.RXLATCH = 0;		// no RX on TX
	SQI1CMDTHRbits.TXCMDTHR = 4;		// Transmit
	SQI1CMDTHRbits.RXCMDTHR = 2;		// Receive
	SQI1CON = (0 << 22) | (0 << 18) | (1 << 16) | 4;
	SQI1CON = (1 << 22) | (1 << 18) | (2 << 16) | 2;

	*(uint32_t *)&SQI1TXBUF = 0x00000092;	// Dev ID Dual	

	sqi_rxcnt(2);
	sqi_intstat();
	val = *(uint16_t *)&SQI1RXBUF;
	uart2_str0(" => ");
	uart2_word(val);

	uart2_str0("\n\r");
	delay(1000);



	uart2_str0("\n\rMF/Dev ID (Quad)...\n\r");

	SQI1CFGbits.DATAEN = 0b10;		// ID0-ID1
	SQI1CFGbits.RXLATCH = 0;		// no RX on TX
	SQI1CMDTHRbits.TXCMDTHR = 4;		// Transmit
	SQI1CMDTHRbits.RXCMDTHR = 2;		// Receive
	SQI1CON = (0 << 22) | (0 << 18) | (1 << 16) | 4;
	SQI1CON = (1 << 22) | (2 << 18) | (2 << 16) | 2;

	*(uint32_t *)&SQI1TXBUF = 0x00000094;	// Dev ID Quad	

	sqi_rxcnt(2);
	sqi_intstat();
	val = *(uint16_t *)&SQI1RXBUF;
	uart2_str0(" => ");
	uart2_word(val);

	uart2_str0("\n\r");
	delay(1000);
	
	
	delay(1000);

	return 0;
}
