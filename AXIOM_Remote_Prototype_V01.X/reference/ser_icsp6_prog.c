
/*  Copyright (C) 2018 Herbert Poetzl
**
**      This program is free software: you can redistribute it and/or
**      modify it under the terms of the GNU General Public License
**      as published by the Free Software Foundation, either version
**      2 of the License, or (at your option) any later version.
*/

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include <fcntl.h>
#include <stropts.h>
#include <linux/termios.h>

#define BAUD 1000000

#define KEY_MCHP 0x4D434850

#define min(a,b)			\
	({ __typeof__ (a) _a = (a);	\
	   __typeof__ (b) _b = (b);	\
	   _a <= _b ? _a : _b; })



#define	MAX_DATA	64
#define	BUF_SIZE	(MAX_DATA * 32)

static char	gw_buf[BUF_SIZE];
static char	gr_buf[BUF_SIZE];


// #define	DEBUG

#ifdef	DEBUG
#define	dump_buf(b,l,p)	_dump_buf(b, l, p)
#else
static	volatile unsigned _cnt;
#define	dump_buf(b,l,p) while (0) { _cnt = l; }
#endif

void	_dump_buf(const char *buf, unsigned len, const char *prefix)
{
	fprintf(stderr, "%s »%*.*s«\n", prefix, len, len, buf);
}


int	readn(int fd, char *buf, unsigned len)
{
	while (len>0) {
	    int cnt = read(fd, buf, len);
		
	    if (cnt >= 0) {
		len -= cnt;
		buf += cnt;
	    } else
		return -1;
	}
	return 0;
}


int	icsp_buf_flush(int fd,
	char *wbuf, unsigned wlen,
	char *rbuf, unsigned rlen)
{
	int res;

	write(fd, wbuf, wlen);
	dump_buf(wbuf, wlen, "W:");
	res = readn(fd, rbuf, rlen);
	dump_buf(rbuf, rlen, "R:");
	return res;	
}


void	icsp_sel(int fd, bool east)
{
        char buf[3];

        snprintf(buf, 3, "%u!", east ? 1 : 0);
	icsp_buf_flush(fd, buf, 2, buf, 2);
}

void	icsp_mclr(int fd, bool high)
{
	char buf[3];
	unsigned bits = high ? 1 : 0;

	snprintf(buf, 3, "%u=", bits);
	icsp_buf_flush(fd, buf, 2, buf, 2);
}



int	icsp_buf_write(char *buf, uint32_t data, unsigned len)
{
	if (len > 16) {
	    snprintf(buf, 13, "%08X#%02X>", data, len);
	    return 12;
	} else if (len > 8) {
	    snprintf(buf, 9, "%04X#%02X>", data & 0xFFFF, len);
	    return 8;
	} else {
	    snprintf(buf, 6, "%02X#%X>", data & 0xFF, len);
	    return 5;
	}
}	

void	icsp_write(int fd, uint32_t data, unsigned len)
{
	char buf[14];
	int dlen;

	dlen = icsp_buf_write(buf, data, len);
	icsp_buf_flush(fd, buf, dlen, buf, dlen);
}

int	icsp_buf_read_size(unsigned len)
{
	if (len > 16)
	    return 8;
	else if (len > 8)
	    return 4;
	else 
	    return 2;
}

int	icsp_buf_read(char *buf, unsigned len)
{
	const char *str = "........";
	int rlen = icsp_buf_read_size(len);

	if (len > 8) {
	    snprintf(buf, 4 + rlen, "%02X<%s", len, str + (8 - rlen));
	    return 3 + rlen;
	} else {
	    snprintf(buf, 3 + rlen, "%X<%s", len, str + (8 - rlen));
	    return 2 + rlen;
	}
}



uint32_t icsp_read(int fd, unsigned len)
{
	uint32_t data = 0;
	char buf[14];
	int wlen, rlen;


	wlen = icsp_buf_read(buf, len);
	rlen = wlen;
	
	icsp_buf_flush(fd, buf, wlen, buf, rlen);
	buf[rlen] = '\0';

	sscanf(buf + wlen, "%X", &data);
	return data;	
}



void	icsp_enter_lvp(int fd)
{
	icsp_write(fd, KEY_MCHP, 33);
}

void	icsp_load_conf(int fd, uint32_t data)
{
	icsp_write(fd, 0x00, 6);		// LOAD_CONF
	icsp_write(fd, (data & 0x3FFF) << 1, 16);
}

int	icsp_buf_load_data(char *buf, uint32_t data)
{
	int len;

	len = icsp_buf_write(buf, 0x02, 6);	// LOAD_DATA
	buf += len;
	len += icsp_buf_write(buf, (data & 0x3FFF) << 1, 16);
	return len;
}

void	icsp_load_data(int fd, uint32_t data)
{
	icsp_write(fd, 0x02, 6);		// LOAD_DATA
	icsp_write(fd, (data & 0x3FFF) << 1, 16);
}

int	icsp_buf_read_data(char *buf)
{
	int len;

	len = icsp_buf_write(buf, 0x04, 6);	// READ_DATA
	buf += len;
	len += icsp_buf_read(buf, 16);
	return len;
}


void	icsp_read_data(int fd, uint32_t *data)
{
	icsp_write(fd, 0x04, 6);		// READ_DATA
	*data = icsp_read(fd, 16) >> 2;
}

int	icsp_buf_inc_addr(char *buf)
{
	return icsp_buf_write(buf, 0x06, 6);	// INC_ADDR
}

void	icsp_inc_addr(int fd)
{
	icsp_write(fd, 0x06, 6);		// INC_ADDR
}

void	icsp_reset_addr(int fd)
{
	icsp_write(fd, 0x16, 6);		// RST_ADDR
}

void	icsp_inc_loop(int fd, unsigned count)
{
	while (count--)
	    icsp_inc_addr(fd);
}


void	icsp_read_nvm(int fd, uint32_t *data, unsigned len)
{
	int dlen, rlen, wlen = 0, cnt = len;
	char *rbuf, *wbuf = gw_buf;

	while (cnt--) {
	    dlen = icsp_buf_read_data(wbuf);
	    wbuf += dlen;
	    wlen += dlen;
	    dlen = icsp_buf_inc_addr(wbuf);
	    wbuf += dlen;
	    wlen += dlen;
	}	

	rlen = wlen;
	icsp_buf_flush(fd, gw_buf, wlen, gr_buf, rlen);

	rbuf = gr_buf;
	while (len--) {
	    uint32_t word;

	    while (*rbuf++ != '<');
	    sscanf(rbuf, "%4X", &word);
	    *data++ = word >> 2;
	}
}

void	icsp_load_nvm(int fd, uint32_t *data, unsigned len)
{
	char *wbuf = gw_buf;
	int dlen, rlen, wlen = 0;

	while (len--) {
	    dlen = icsp_buf_load_data(wbuf, *data++);
	    wbuf += dlen;
	    wlen += dlen;
	    if (len) {
		dlen = icsp_buf_inc_addr(wbuf);
		wbuf += dlen;
		wlen += dlen;
	    }
	}
	rlen = wlen;
	icsp_buf_flush(fd, gw_buf, wlen, gr_buf, rlen);
}

void	icsp_program(int fd)
{
	icsp_write(fd, 0x08, 6);	// PROG_INT
	usleep(2800);			// TPINT = 2.8ms
}

void	icsp_row_erase(int fd)
{
	icsp_write(fd, 0x11, 6);	// ROW_ERASE
	usleep(2800);			// TERAR = 2.8ms
}

void	icsp_bulk_erase(int fd)
{
	icsp_write(fd, 0x09, 6);	// BULK_ERASE
	usleep(8400);			// TERAB = 8.4ms
}

void	__dump_data(uint32_t *data, unsigned len, const char *nfo, const char *fmt)
{
	printf("%s", nfo);
	while (len--)
	    printf(fmt, *data++, len ? ", " : "\n");
}

#define dump_data(d, l, n) __dump_data(d, l, n, "0x%08X%s")
#define dump_data_fmt(d, l, n, f) __dump_data(d, l, n, f "%s")



int	parse_ihex(const char *line, uint8_t *bytes, uint32_t *addr, uint8_t *type)
{
	int sum, num = 0;
	int len, word, byte, cksum;

	if (line[0] != ':')
	    return -1;
	if (strlen(line) < 11)
	    return -2;
	
	line++;
	if (!sscanf(line, "%02x", &len))
	    return -3;

	line += 2;
	if (strlen(line) < (8 + 2*len))
	    return -2;
	if (!sscanf(line, "%04x", &word))
	    return -4;

	*addr = (*addr & ~0xFFFF) | (word & 0xFFFF);

	line += 4;
	if (!sscanf(line, "%02x", &byte))
	    return -5;

	*type = byte;
	sum = (len & 0xFF) + (byte & 0xFF) +
	    ((word >> 8) & 0xFF) + (word & 0xFF);

	while (num != len) {
	    line += 2;
	    if (!sscanf(line, "%02x", &byte))
		return -5;

	    sum += byte & 0xFF;
	    bytes[num++] = byte;

	    if (num == 256)
		return -6;
	}

	line += 2;
	if (!sscanf(line, "%02x", &cksum))
	    return -7;
	
	if (((sum & 0xFF) + (cksum & 0xFF)) & 0xFF)
	    return -8;

	switch(*type) {
	    case 0:
		break;

	    case 4:
		if (len != 2)
		    return -9;
		
		word = (bytes[0] << 8) | bytes[1];
		*addr = (*addr & 0xFFFF) | (word << 16); 
		break;

	    default:
		return -10;
	} 
	
	return num;
}



int	main(int argc, char *argv[])
{
	char *dev = "/dev/ttyUSB0";
	unsigned long baud = BAUD;
	struct termios2 tio;
	int fd, res, ret = 0;

	if (argc > 1) {
	    dev = argv[1];
	    if (argc > 3)
		baud = strtol(argv[3], NULL, 0);
	}

	fd = open(dev, O_RDWR);
	if (fd == -1) {
	    fprintf(stderr,
		"error opening >%s<.\n%s\n",
		dev, strerror(errno));
	    exit(1);
	}

	res = ioctl(fd, TCGETS2, &tio);
	if (res == -1) {
	    fprintf(stderr,
		"ioctl(TCGETS2): %s\n", strerror(errno));
	    exit(2);
	}

	tio.c_cflag &= ~PARENB;
	tio.c_cflag &= ~CSTOPB;
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag &= ~CBAUD;;
	tio.c_cflag |= CS8;
	tio.c_cflag |= CLOCAL;
	tio.c_cflag |= BOTHER;

	tio.c_iflag = ICANON;
	tio.c_oflag &= ~OPOST;

	tio.c_ispeed = baud;
	tio.c_ospeed = baud;

	tio.c_cc[VTIME] = 10;
	tio.c_cc[VMIN] = 0;

	res = ioctl(fd, TCSETS2, &tio);
	if (res == -1) {
	    fprintf(stderr,
		"ioctl(TCSETS2): %s\n", strerror(errno));
	    exit(4);
	}

	res = ioctl(fd, TCFLSH, TCIOFLUSH);
	if (res == -1) {
	    fprintf(stderr,
		"ioctl(TCFLSH): %s\n", strerror(errno));
	    exit(4);
	}

	uint32_t data[32] = {0};

	// icsp_sel(fd, false);	// West
	icsp_sel(fd, true);	// East

	icsp_mclr(fd, false);

	icsp_enter_lvp(fd);
	icsp_load_conf(fd, 0);
	icsp_inc_loop(fd, 5);
	icsp_read_nvm(fd, data, 2);

	dump_data_fmt(data, 2, "cid: ", "0x%04X");

	if ((data[0] == 0x3FFFFF) ||
	    (data[0] == 0x0) ||
	    (data[1] == 0x3FFFFF) ||
	    (data[1] == 0x0)) {
	    printf("ERROR: no pic device found\n");
	    ret = 6;
	    goto err_mclr;
	}

	unsigned row_size = 0;
	unsigned num_rows = 0;

	if ((data[0] == 0x2000) && (data[1] == 0x305B)) {
	    printf("PIC16F1718 detected\n");
	    row_size = 32;
	    num_rows = 512;
	} else {
	    printf("Unknown PIC detected\n");
	    goto err_mclr;
	}

	unsigned total = row_size * num_rows * 2;
	uint8_t *pgm = malloc(total);

	for (int i=0; i<total; i++)
	    pgm[i] = 0xFF;

	unsigned cfgsz = 32;
	uint8_t *cfg = malloc(cfgsz);

	for (int i=0; i<cfgsz; i++)
	    cfg[i] = 0xFF;


	if (argc > 2) {
	    FILE *fp = fopen(argv[2], "r");

	    if (fp == NULL) {
		printf("ERROR: fopen >%s<\n", argv[2]);
		ret = 7;
		goto err_mclr;
	    }

	    char line[256];
	    uint8_t bytes[256];
	    uint8_t type;
	    uint32_t addr = 0;

	    while (!feof(fp) && !ferror(fp)) {
		fgets(line, sizeof(line), fp);
		unsigned len = strlen(line);

		if (line[len-1] == '\n')
		    line[--len] = 0;
		if (line[len-1] == '\r')
		    line[--len] = 0;

		int num = parse_ihex(line, bytes, &addr, &type);
		
		if ((num > 0) && (type == 0)) {
		    // printf("%08X: %3d ", addr, num);
		    for (int i=0; i<num; i++) {
			unsigned idx = addr + i;

			if (idx < total)
			    pgm[idx] = bytes[i];
			if (idx > 0x10000) {
			    idx -= 0x10000;
			    if (idx < cfgsz)
				cfg[idx] = bytes[i];

			}
		    }
		}
	    }
	}

	// icsp_load_addr(fd, 0x80FE);	// Only User Flash
	// icsp_load_addr(fd, 0x0000);
	icsp_bulk_erase(fd);

	icsp_reset_addr(fd);

	unsigned inc_loop = 0;

	for (int row=0; row<num_rows; row++) {
	    unsigned addr = row * row_size;
	    uint8_t *ptr = pgm + addr * 2;
	    uint16_t mask = 0xFFFF;
	
	    for (int i = 0; i<row_size; i++) {
		unsigned idx = i * 2;
		uint16_t word = (ptr[idx+1] << 8) | ptr[idx];

		mask &= word;
		data[i] = word;
	    }

	    if (mask == 0xFFFF) {
		inc_loop += 32;
		continue;
	    }
	
	    printf("%03X: 0x%04X\n", row, mask);
	    // dump_data(data, row_size, "pgm: ");
	    icsp_inc_loop(fd, inc_loop);
	    icsp_load_nvm(fd, data, row_size);
	    icsp_program(fd);
	    inc_loop = 1;
	}

	icsp_load_conf(fd, 0);

	for (int idx=0; idx<cfgsz; idx+=2) {
	    unsigned addr = 0x8000 + (idx >> 1);
	    unsigned word = (cfg[idx+1] << 8) | cfg[idx];
	
	    if (word == 0xFFFF) {
		icsp_inc_addr(fd);
		continue;
	    }

	    printf("%03X: 0x%04X\n", addr, word);
	    // icsp_load_addr(fd, addr);
	    icsp_load_data(fd, word);
	    icsp_program(fd);
	    icsp_inc_addr(fd);
	}

	icsp_reset_addr(fd);
	icsp_read_nvm(fd, data, 32);
	dump_data_fmt(data, 32, "pgm: ", "%04X");

	icsp_load_conf(fd, 0);
	icsp_read_nvm(fd, data, 32);
	dump_data_fmt(data, 32, "cfg: ", "%04X");

err_mclr:
	icsp_mclr(fd, true);

	close(fd);

	exit(ret);
}


