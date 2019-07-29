
/*	PIC32MZ Serial Programming and Debug Interface
**
**	Copyright (C) 2019 Herbert Poetzl
**
**	This program is free software; you can redistribute it and/or modify
**	it under the terms of the GNU General Public License 2 as published 
**	by the Free Software Foundation.
**
*/

/*	Clock Setup
**	
**	24MHz External Oscillator
**	PLL in = (24MHz/2) = 12MHz	[8-16MHz range]
**	PLL freq = 12MHz*32 = 384MHz	[350MHz - 700MHz]
**	PLL out = 384MHz/2 = 192MHz
*/

#define SYS_FREQ (192000000L)
#define GetSystemClock() (192000000ul)  // PIC32MZ System Clock in Hertz.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

	// DEVCFG0
#pragma config BOOTISA = MIPS32
#pragma config ICESEL = ICS_PGx1
#pragma config FECCCON = OFF_UNLOCKED
#pragma config EJTAGBEN = NORMAL
#pragma config DBGPER = PG_ALL
#pragma config FSLEEP = 0

	// DEVCFG1
#pragma	config FDMTEN = OFF
#pragma	config FWDTEN = OFF
#pragma config POSCMOD = EC
#pragma config OSCIOFNC = OFF
#pragma config FSOSCEN = OFF
#pragma config FNOSC = SPLL
#pragma config FCKSM = CSECMD

	// DEVCFG2
#pragma config FPLLICLK = PLL_POSC
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLRNG = RANGE_8_16_MHZ
#pragma config FPLLMULT = MUL_32
#pragma config FPLLODIV = DIV_2
#pragma config UPLLEN = OFF
#pragma config UPLLFSEL = FREQ_24MHZ

	// DEVCFG3
#pragma config USERID = 0xC0DE
#pragma config FMIIEN = OFF
#pragma config PGL1WAY = OFF
#pragma config PMDL1WAY = OFF
#pragma config IOL1WAY = OFF
#pragma config FUSBIDIO = ON

	// DEVCP0
#pragma config CP = OFF


static uint32_t hexval = 0;
static uint32_t hexdat = 0;

static uint32_t seq[256];

static uint8_t seq_idx = 0;
static uint8_t seq_cnt = 0;


#define	OSC_EN_O	LATCbits.LATC15
#define	OSC_EN_T	TRISCbits.TRISC15

/*
**	KMW_MCLR	C13
**	KMW_PDAT	RF2	U6TX,SDO4,SDO6	U4RX,U6RX,SDI6
**	KMW_PCLK	RF8	U4TX,U6TX,SDO6	U2RX
**
**	SDA4		RG7	SDO1-SDO5,U1TX,U5TX
**	SCL4		RG8	SDO1-3,SDO5,U3TX
*/

#define ICSP_W_MCLR_O	LATCbits.LATC13
#define ICSP_W_MCLR_T	TRISCbits.TRISC13

#define	ICSP_W_PCLK_O	LATFbits.LATF8
#define	ICSP_W_PCLK_T	TRISFbits.TRISF8
#define ICSP_W_PCLK_U	CNPUFbits.CNPUF8

#define	ICSP_W_PDAT_O	LATFbits.LATF2
#define ICSP_W_PDAT_I	PORTFbits.RF2
#define ICSP_W_PDAT_T	TRISFbits.TRISF2
#define ICSP_W_PDAT_U	CNPUFbits.CNPUF2



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
void	io_unlock(void)
{
	CFGCONbits.IOLOCK = 0;
}

static inline
void	io_lock(void)
{
	CFGCONbits.IOLOCK = 1;
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
	PB2DIVbits.PBDIV = 0b000000;	// divide by 1
	PB2DIVbits.ON = 1;

	PB7DIVbits.PBDIV = 0b000000;	// divide by 1
	PB7DIVbits.ON = 1;
	lock();
}

void	init_mvec(void)
{
	INTCONbits.MVEC = 1;		// Multi Vector Interrupts
	PRISSbits.SS0 = 0;		// Normal Register Set
	PRISSbits.PRI1SS = 1;		// Assign Shadow Register Set
	PRISSbits.PRI2SS = 2;		// Assign Shadow Register Set
	PRISSbits.PRI3SS = 3;		// Assign Shadow Register Set
	PRISSbits.PRI4SS = 4;		// Assign Shadow Register Set
	PRISSbits.PRI5SS = 5;		// Assign Shadow Register Set
	PRISSbits.PRI6SS = 6;		// Assign Shadow Register Set
	PRISSbits.PRI7SS = 7;		// Assign Shadow Register Set
}

void	init_kmw(void)
{
	ICSP_W_MCLR_T = 0;		// MCLR out
	ICSP_W_PCLK_T = 0;		// PCLK out
	ICSP_W_PDAT_T = 0;		// PDAT out
}


void	init_i2c3(void)
{
	I2C3ADD = 0xFF;
	I2C3MSK = 0xFF;

	I2C3BRG = 128;			// 100 kHz
}


void	init_uart2(void)
{
	U2MODE = 0;

	LATEbits.LATE8 = 1;		// U2TX high
	TRISEbits.TRISE8 = 0;		// U2TX out
	TRISEbits.TRISE9 = 1;		// U2RX in
	ANSELEbits.ANSE8 = 0;		// digital
	ANSELEbits.ANSE9 = 0;		// digital

	io_unlock();
	RPE8Rbits.RPE8R = 0b0010;	// U2TX
	U2RXRbits.U2RXR = 0b1101;	// RPE9
	io_lock();

	IPC36bits.U2TXIP = 7;		// Interrupt priority of 7
	IPC36bits.U2TXIS = 0;		// Interrupt sub-priority of 0
	IPC36bits.U2RXIP = 7;		// Interrupt priority of 7
	IPC36bits.U2RXIS = 0;		// Interrupt sub-priority of 0

	IEC4SET = _IEC4_U2RXIE_MASK;	// Rx INT Enable
	IFS4bits.U2TXIF = 0;		// Clear Tx flag
	IFS4bits.U2RXIF = 0;		// Clear Rx flag

	U2BRG = 47;			// 1MBaud @ 192MHz
	// U2BRG = 23;			// 2MBaud @ 192MHz
	// U2BRG = 15;			// 3MBaud @ 192MHz
	// U2BRG = 11;			// 4MBaud @ 192MHz
	// U2BRG = 7;			// 6MBaud @ 192MHz
	// U2BRG = 5;			// 8MBaud @ 192MHz
	// U2BRG = 3;			// 12MBaud @ 192MHz
	U2STA = 0;

	U2MODEbits.BRGH = 1;
	U2MODEbits.PDSEL = 0b00;
	U2MODEbits.STSEL = 0;
	U2MODEbits.UEN = 0b00;
	U2MODEbits.ON = 1;
	U2STASET = 0x9400;		// Enable Transmit and Receive
}

void	init_uart3(void)
{
	U3MODE = 0;

	LATGbits.LATG8 = 1;		// SCL4 high
	TRISGbits.TRISG8 = 0;		// SCL4 out
	ANSELGbits.ANSG8 = 0;		// digital

	io_unlock();
	RPG8Rbits.RPG8R = 0b0001;	// U3TX
	io_lock();

	U3BRG = 47;			// 1MBaud @ 192MHz
	// U3BRG = 23;			// 2MBaud @ 192MHz
	// U3BRG = 15;			// 3MBaud @ 192MHz
	// U3BRG = 11;			// 4MBaud @ 192MHz
	// U3BRG = 7;			// 6MBaud @ 192MHz
	// U3BRG = 5;			// 8MBaud @ 192MHz
	// U3BRG = 3;			// 12MBaud @ 192MHz
	U3STA = 0;

	U3MODEbits.BRGH = 1;
	U3MODEbits.PDSEL = 0b00;
	U3MODEbits.STSEL = 0;
	U3MODEbits.UEN = 0b00;
	U3MODEbits.ON = 1;
	U3STASET = 0x8400;		// Enable Transmit
}

void	init_uart6(void)
{
	U6MODE = 0;

	io_unlock();
	U6RXRbits.U6RXR = 0b1011;	// RPF2
	io_lock();

	IPC47bits.U6RXIP = 3;		// Interrupt priority of 3
	IPC47bits.U6RXIS = 0;		// Interrupt sub-priority of 0

	IEC5SET = _IEC5_U6RXIE_MASK;	// Rx INT Enable
	IFS5bits.U6RXIF = 0;		// Clear Rx flag

	U6BRG = 47;			// 1MBaud @ 192MHz
	// U6BRG = 23;			// 2MBaud @ 192MHz
	// U6BRG = 15;			// 3MBaud @ 192MHz
	// U6BRG = 11;			// 4MBaud @ 192MHz
	// U6BRG = 7;			// 6MBaud @ 192MHz
	// U6BRG = 5;			// 8MBaud @ 192MHz
	// U6BRG = 3;			// 12MBaud @ 192MHz
	U6STA = 0;

	U6MODEbits.BRGH = 1;
	U6MODEbits.PDSEL = 0b00;
	U6MODEbits.STSEL = 0;
	U6MODEbits.UEN = 0b00;
	U6MODEbits.ON = 1;		// enable UART6
	U6STASET = 0x1000;		// Enable Receive

}

void	init_spi5(void)
{
	SPI5CON = 0;

	LATGbits.LATG7 = 1;		// SDA4 high
	TRISGbits.TRISG7 = 0;		// SDA4 out

	SPI5CONbits.MSSEN = 0;
	SPI5CONbits.MSTEN = 1;
	SPI5CONbits.DISSDI = 1;
	SPI5CONbits.ENHBUF = 1;
	SPI5CONbits.MODE32 = 1;

	SPI5CONbits.STXISEL = 0b10;	// half empty

	IPC44bits.SPI5TXIP = 5;		// Interrupt priority of 5
	IPC44bits.SPI5TXIS = 0;		// Interrupt sub-priority of 0

	IEC5SET = _IEC5_SPI5TXIE_MASK;	// Tx INT Enable
	IFS5CLR = _IFS5_SPI5TXIF_MASK;	// clear SPI5 Tx IRQ

	io_unlock();
	RPG7Rbits.RPG7R = 0b1001;	// SDO5
	io_lock();

	SPI5BRG = 95;			// 1Mbit / 1us
	// SPI5BRG = 47;		// 2Mbit / 500ns
	// SPI5BRG = 23;		// 4Mbit / 250ns
	// SPI5BRG = 11;		// 8Mbit / 125ns
	// SPI5BRG = 5;			// 16Mbit / 62.5ns
	// SPI5BRG = 2;			// 32Mbit / 31.25ns

	SPI5CON2bits.AUDEN = 0;
	SPI5CONbits.ON = 1;
}

void	init_spi6(void)
{
	SPI6CON = 0;
	SPI6CONbits.MSSEN = 0;
	SPI6CONbits.MSTEN = 1;
	SPI6CONbits.DISSDI = 1;
	SPI6CONbits.ENHBUF = 1;
	SPI6CONbits.MODE32 = 1;

	SPI6CONbits.STXISEL = 0b10;	// half empty

	io_unlock();
	RPF8Rbits.RPF8R = 0b1010;	// SDO6
	io_lock();

	SPI6BRG = 95;			// 1Mbit / 1us
	// SPI6BRG = 47;		// 2Mbit / 500ns
	// SPI6BRG = 23;		// 4Mbit / 250ns
	// SPI6BRG = 11;		// 8Mbit / 125ns
	// SPI6BRG = 5;			// 16Mbit / 62.5ns
	// SPI6BRG = 2;			// 32Mbit / 31.25ns

	SPI6CON2bits.AUDEN = 0;
	SPI6CONbits.ON = 1;
}




void	conf_i2c(void)
{
	// U6MODEbits.ON = 0;		// disable UART6
	U6STACLR = 0x1000;		// disable UART6 RX
	SPI6CONbits.ON = 0;		// disable SPI6

	ICSP_W_PCLK_U = 1;
	ICSP_W_PDAT_U = 1;

	ICSP_W_PDAT_O = 0;		// clear SDA
	ICSP_W_PDAT_T = 0;		// SDA to out
	ICSP_W_PCLK_O = 1;		// set SCL
	ICSP_W_PCLK_T = 1;		// SCL to in

	I2C3CONbits.ON = 1;		// enable I2C3
}

void	conf_icsp(void)
{
	// U6MODEbits.ON = 0;		// disable UART6
	U6STACLR = 0x1000;		// disable UART6 RX
	SPI6CONbits.ON = 0;		// disable SPI6
	I2C3CONbits.ON = 0;		// disable I2C3

	ICSP_W_PCLK_U = 0;
	ICSP_W_PDAT_U = 0;

	ICSP_W_PDAT_O = 0;		// clear PDAT
	ICSP_W_PDAT_T = 0;		// PDAT to out
	ICSP_W_PCLK_O = 0;		// clear PCLK
	ICSP_W_PCLK_T = 0;		// PCLK to out
}

void	conf_ktest(void)
{
	I2C3CONbits.ON = 0;		// disable I2C3

	ICSP_W_PCLK_U = 0;
	ICSP_W_PDAT_U = 0;

	ICSP_W_PDAT_O = 1;		// set RX
	ICSP_W_PDAT_T = 1;		// RX to in
	ICSP_W_PCLK_O = 0;		// clr SDO
	ICSP_W_PCLK_T = 0;		// SDO to out

	// U6MODEbits.ON = 1;		// enable UART6
	U6STASET = 0x1000;		// enable UART6 RX
	SPI6CONbits.ON = 1;		// enable SPI6
}

static inline
void	set_mclr(char val)
{
	if (val)
	    ICSP_W_MCLR_O = 1;
	else
	    ICSP_W_MCLR_O = 0;
}



static inline
uint8_t	i2c3_start(void)
{
	I2C3CONbits.SEN = 1;		// Send Start
	while (I2C3CONbits.SEN);
	return I2C3STAT & 0xFF;
}

static inline
uint8_t	i2c3_restart(void)
{
	I2C3CONbits.RSEN = 1;		// Send Restart
	while (I2C3CONbits.RSEN);
	return I2C3STAT & 0xFF;
}


static inline
uint8_t	i2c3_stop(void)
{
	if ((I2C3CON & 0x1F) == 0)
	    I2C3CONbits.PEN = 1;	// Send Stop
	return I2C3CON & 0x1F;
}

static inline
void	i2c_bb_delay(unsigned cnt)
{
	unsigned i;
	while (cnt--)
	    for (i=0; i<200; i++);
}

static inline
void	i2c3_bb_stop(void)
{
	i2c_bb_delay(1);
	I2C3CONbits.ON = 0;		// disable I2C
	i2c_bb_delay(1);
	ICSP_W_PDAT_T = 1;		// SDA to input
	i2c_bb_delay(2);
	I2C3CONbits.ON = 1;		// enable I2C
	ICSP_W_PDAT_O = 0;		// clear SDA
	ICSP_W_PDAT_T = 0;		// SDA to out
	ICSP_W_PCLK_O = 1;		// set SCL
	ICSP_W_PCLK_T = 1;		// SCL to in
}


static inline
bool	i2c3_write(uint8_t byte)
{
	I2C3TRN = byte;
	while (I2C3STATbits.TRSTAT);
	return I2C3STATbits.ACKSTAT;
}

static inline
uint8_t	i2c3_read(bool ackdt)
{
	while (I2C3STATbits.RBF)
	    (void)I2C3RCV;
	if (I2C3CON & 0x1F)
	    return 0xFF;

	I2C3CONbits.RCEN = 1;
	while (!I2C3STATbits.RBF);

	I2C3CONbits.ACKDT = ackdt;
	I2C3CONbits.ACKEN = 1;
	while (I2C3CONbits.ACKEN);

	return I2C3RCV;
}



static inline
void	icsp_w_mclr(unsigned val)
{
	ICSP_W_MCLR_O = (val & 1) ? 1 : 0;
	ICSP_W_PCLK_O = (val & 2) ? 1 : 0;
	ICSP_W_PDAT_O = (val & 4) ? 1 : 0;
}

static inline
void	icsp_w_out(uint32_t val, unsigned len)
{
	while (len) {
	    bool bit = val & 1;

	    ICSP_W_PCLK_O = 1;
	    ICSP_W_PDAT_O = bit;
	    val >>= 1;
	    ICSP_W_PCLK_O = 0;
	    len--;
	}
}

static inline
uint32_t icsp_w_in(unsigned len)
{
	uint32_t val = 0;
	unsigned shift = 32 - len;

	ICSP_W_PDAT_T = 1;
	while (len) {
	    ICSP_W_PCLK_O = 1;
	    val >>= 1;
	    len--;

	    bool bit = ICSP_W_PDAT_I;

	    ICSP_W_PCLK_O = 0;
	    val |= bit ? (1<<31) : 0;
	}
	ICSP_W_PDAT_T = 0;
	return val >> shift;
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
void	uart2_str0(char *str)
{
	while (*str)
	    uart2_ch(*str++);
}


static inline
void	uart3_ch(char ch)
{
	while (U3STAbits.UTXBF);
	U3TXREG = ch;
}

static inline
void	spi5_out(uint32_t val)
{
	while (SPI5STATbits.SPITBF);
	SPI5BUF = val;
}

static inline
void	spi6_out(uint32_t val)
{
	while (SPI6STATbits.SPITBF);
	SPI6BUF = val;
}



void __attribute__((vector(_UART2_RX_VECTOR), interrupt(IPL7SRS))) 
uart2_isr(void)
{
	while (U2STAbits.URXDA) {	// process buffer 
	    char ch = U2RXREG;

	    switch (ch) {
		case '0' ... '9':
		    hexval <<= 4;
		    hexval |= ch - '0';
		    uart2_ch(ch);	// echo back
		    break;
		case 'A' ... 'F':
		    hexval <<= 4;
		    hexval |= ch - 'A' + 10;
		    uart2_ch(ch);	// echo back
		    break;
		case 'a' ... 'f':
		    hexval <<= 4;
		    hexval |= ch - 'a' + 10;
		    uart2_ch(ch);	// echo back
		    break;

		case '#':		// copy val
		    hexdat = hexval;
		    uart2_ch(ch);	// echo back
		    hexval = 0;
		    break;


		case '=':		// set mclr/pclk/pdat
		    icsp_w_mclr(hexval);
		    uart2_ch(ch);	// echo back
		    hexval = 0;
		    break;

		case '>':		// out icsp seq
		    icsp_w_out(hexdat, hexval & 0x3F);
		    uart2_ch(ch);	// echo back
		    hexval = 0;
		    break;

		case '<':		// out icsp seq
		    hexdat = icsp_w_in(hexval & 0x1F);
		    uart2_ch(ch);	// echo back
		    if (hexval > 16)
			uart2_long(hexdat);
		    else if (hexval > 8)
			uart2_word(hexdat);
		    else 
			uart2_byte(hexdat);
		    hexval = 0;
		    break;

		case '.':		// ignore
		    break;


		case '!':		// switch to ICSP
		    conf_icsp();
		    uart2_ch(ch);	// echo back
		    break;

		case '%':		// switch to I2C
		    conf_i2c();
		    uart2_ch(ch);	// echo back
		    break;

		case '?':		// switch to KTest
		    conf_ktest();
		    uart2_ch(ch);	// echo back
		    break;


		case '&':		// set seq index
		    seq_idx = hexval;
		    uart2_ch(ch);	// echo back
		    hexval = 0;
		    break;

		case '|':		// set seq data
		    seq[seq_idx++] = hexval;
		    uart2_ch(ch);	// echo back
		    hexval = 0;
		    break;

		case '*':		// activate sequence
		    seq_cnt = hexval;
		    IEC5bits.SPI5TXIE = 1;
		    uart2_ch(ch);	// echo back
		    hexval = 0;
		    break;

		case 'M':		// MCLR
		    set_mclr(hexval);
		    uart2_ch(ch);	// echo back
		    hexval = 0;
		    break;


		case 'S':		// I2C Start
		    hexdat = i2c3_start();
		    uart2_ch(ch);	// echo back
		    uart2_byte(hexdat);
		    break;

		case 's':		// I2C Retart
		    hexdat = i2c3_restart();
		    uart2_ch(ch);	// echo back
		    uart2_byte(hexdat);
		    break;

		case 'P':		// I2C Stop
		    i2c3_bb_stop();
		    uart2_ch(ch);	// echo back
		    break;

		case 'W':		// I2C Write
		    hexdat = i2c3_write(hexval);
		    uart2_ch(ch);	// echo back
		    uart2_hex(hexdat);
		    hexval = 0;
		    break;

		case 'R':		// I2C Read
		    hexdat = i2c3_read(hexval);
		    uart2_ch(ch);	// echo back
		    uart2_byte(hexdat);
		    hexval = 0;
		    break;

		default:
		    uart2_ch(ch);	// echo back
	    }
	}

	IFS4CLR = _IFS4_U2RXIF_MASK;	// clear UART2 Rx IRQ
}


void __attribute__((vector(_UART6_RX_VECTOR), interrupt(IPL3SRS)))
uart6_isr(void)
{
	while (U6STAbits.URXDA) {	// process buffer
	    char ch = U6RXREG;
	    uart2_ch(ch);		// echo back
	}
	IFS5CLR = _IFS5_U6RXIF_MASK;	// clear UART6 Rx IRQ
}



void __attribute__((vector(_SPI5_TX_VECTOR), interrupt(IPL5SRS)))
spi5_isr(void)
{
	if (seq_cnt > 0) {
	    uint32_t val = seq[seq_idx++];
	    spi5_out(val);
	    spi6_out(val);
	    seq_cnt--;
	    uart3_ch('^');
	} else {
	    IEC5bits.SPI5TXIE = 0;	// Tx INT Disable
	}
	IFS5bits.SPI5TXIF = 0;		// clear SPI5 Tx IRQ
}


static inline
void	spi_seq(uint8_t idx, uint8_t cnt)
{
	seq_idx = idx;
	seq_cnt = cnt;
	IEC5bits.SPI5TXIE = 1;		// Tx INT Enable
}


static inline
void	delay16(uint16_t cnt)
{
	while (cnt--)
	    asm volatile("nop");
}

static inline
void	delay32(uint32_t cnt)
{
	while (cnt--)
	    asm volatile("nop");
}

int	main(void)
{
	OSC_EN_O = 1;
	OSC_EN_T = 0;

	irq_disable();
	init_kmw();
	init_pbus();
	init_mvec();
	init_uart2();
	init_uart3();
	init_uart6();
	init_spi5();
	init_spi6();

	conf_icsp();

	while (OSCCONbits.SLOCK == 0);
	irq_enable();

	for (int i=0; i<256; i++)
	    seq[i] = 0xAA000000 | i;

	uart2_str0("\f\n\rAXIOM\n\r");

	uart3_ch('<');
	spi_seq(0,5);
	uart3_ch('>');

	uart2_ch('*');

	while (1)
	    asm volatile("wait");
}
