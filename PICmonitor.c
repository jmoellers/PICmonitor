# include	<xc.h>
# include	<ctype.h>

# pragma config FWDTEN = OFF    // 0    FWDTEN - Watchdog Timer is disabled
# pragma config FNOSC = FRCPLL	// 001  FRCPLL - Fast RC Oscillator with Postscaler and PLL module
# pragma config JTAGEN = OFF	// 0 = JTAG port is disabled

void my_puts(char *s);
void my_puthex16(unsigned int u16);
void my_putc(char c);
char *skip_white_space(char *s);

void (*getErrLoc(void))(void);  // Get Address Error Loc
// void __attribute__((__interrupt__)) _OscillatorFail(void);
// void __attribute__((__interrupt__)) _AddressError(void);
// void __attribute__((__interrupt__)) _StackError(void);
// void __attribute__((__interrupt__)) _MathError(void);

void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void)
{
    INTCON1bits.OSCFAIL = 0;        //Clear the trap flag
    my_putc('O');
    while (1);
}

void __attribute__((interrupt, no_auto_psv)) _AddressError(void)
{
    void *errLoc;

    errLoc=getErrLoc();
    INTCON1bits.ADDRERR = 0;        //Clear the trap flag
    my_putc('A');
    while (1);
}

void __attribute__((interrupt, no_auto_psv)) _StackError(void)
{
    INTCON1bits.STKERR = 0;         //Clear the trap flag
    my_putc('S');
    while (1);
}

void __attribute__((interrupt, no_auto_psv)) _MathError(void)
{
    INTCON1bits.MATHERR = 0;        //Clear the trap flag
    my_putc('M');
    while (1);
}

/*********\
 * UART1 *
\*********/

/* Default value for U1BRG. Set to 0 to enable autobaud */
# define	DEFAULT_U1BRG	0

char my_getc(void);

UART_init(unsigned int ubrg, unsigned char brgh)
{
    /*
     * Unlock IOPORTS
     */
    __builtin_write_OSCCONL(OSCCON & ~0x40);

    /*
     * assign U1TX to RP2 Output Pin
     * (AN4/C1IN-/SDA2/RP2/CN6/RB2)
     */
    RPOR1bits.RP2R = 3;

    /*
     * CONFIG_RB3_AS_INPUT
     */
    // _TRISB3 = 1;

    /*
     * Assign U1RX To Pin RP3
     * (AN5/C1IN+/SCL2/RP3/CN7/RB3)
     */
    RPINR18bits.U1RXR = 3;

    /*
     * Re-lock IOPORTS
     */
    __builtin_write_OSCCONL(OSCCON | 0x40);

    /*
     * Clear all U1MODE bits
     * This will result in 8n1
     */
    U1MODE = 0;

    /*
     * Enable UART:
     * - UxTX and UxRX pins are enabled and used;
     *   UxCTS and UxRTS/BCLKx pins are controlled by PORT latches
     * -  UARTx is enabled; all UARTx pins are controlled by UARTx as defined by UEN<1:0>
     * - Transmit is enabled, UxTX pin is controlled by UARTx
     */
    U1MODEbits.UEN = 0;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;

    /*
     * Set baud rate
     */
    U1MODEbits.BRGH = brgh;
    if (ubrg != 0) {
	/* Fixed baud rate */
	U1BRG = ubrg;
    } else {
        volatile char dummy;

	do {
	    /* Autobaud */
	    U1MODEbits.ABAUD = 1;
	    while (U1MODEbits.ABAUD)
		;
	    /* Get sync character */
	    dummy = U1RXREG;
	    dummy = my_getc();
	} while (dummy != 'U');
    }


    /*
     * Flush receive queue
     */
    while (U1STAbits.URXDA)
    {
        volatile char dummy;
        dummy = U1RXREG;
    }

    return;
}

void
my_putc(char c)
{
    if (c == '\n')
        my_putc('\r');
    while (U1STAbits.UTXBF) ;
    U1TXREG = c;
}

void
my_puts(char *s)
{
    while (*s)
        my_putc(*s++);
}

static const char hex[16] = "0123456789ABCDEF";

void
my_puthex16(unsigned int u16)
{
    my_putc(hex[(u16 >> 12) & 0x0f]);
    my_putc(hex[(u16 >>  8) & 0x0f]);
    my_putc(hex[(u16 >>  4) & 0x0f]);
    my_putc(hex[ u16        & 0x0f]);
}

void
my_puthex8(unsigned char u8)
{
    my_putc(hex[(u8 >>  4) & 0x0f]);
    my_putc(hex[ u8        & 0x0f]);
}

char
my_getc(void)
{
    char c;

    while (!U1STAbits.URXDA) ;
    c = U1RXREG;
    if (c == '\r')
        c = '\n';

    my_putc(c);	// echo

    return c;
}

int
my_gets(char *buf, int bufsiz)
{
    char c;
    int nread;

    bufsiz--;	// reserve space for NUL
    nread = 0;
    while ((c = my_getc()) != '\n')	// my_getc maps \r to \n
    {
        if (c == '\b' && nread > 0)
	{
	    my_puts(" \b");
	    nread--;
	}
	else if (nread++ < bufsiz)
	    *buf++ = c;
    }

    *buf = '\0';

    return nread;
}

/********\
 * I2C1 *
\********/

# define	I2C_ACK	0	// ACKSTAT = 0 ->  ACK was detected last
# define	I2C_NACK 1	// ACKSTAT = 1 -> NACK was detected last

static inline void
i2c_init(unsigned int brg)
{
    I2C1BRG = brg;
    // I2C1BRG = 0x9d;	// standard speed (100kHz) @ 32/16 MHz
    I2C1CONbits.I2CEN = 1;	// Enable I2C1

# if A3_SILICON /* { */
    TRISBbits.TRISB9 = 0;	// TDO/SDA1/RP9/CN21/PMD3/RB9
    LATBbits.LATB9 = 0;
    LATBbits.LATB9 = 1;
    TRISBbits.TRISB9 = 1;
# endif /* } */

    return;
}

static inline void
I2C1start()
{
    I2C1CONbits.SEN = 1;	// start
    while (I2C1CONbits.SEN);	// wait until start finished
}

static inline void
I2C1restart()
{
    I2C1CONbits.RSEN = 1;	// restart
    while (I2C1CONbits.RSEN);	// wait until restart finished
}

static inline void
I2C1stop()
{
    I2C1CONbits.PEN = 1;	// stop
    while (I2C1CONbits.PEN);	// wait unti stop finished
}

static inline int
i2c_send(unsigned char byte)
{
    I2C1TRN = byte;	// send byte;
    while (I2C1STATbits.TRSTAT) ;	// wait for transmition to finish

    return I2C1STATbits.ACKSTAT == I2C_ACK;
}

static inline unsigned char
i2c_recv(unsigned int ack)
{
    unsigned char byte;

    while (I2C1CON & 0x1f);	// wait for idle
    I2C1CONbits.RCEN = 1;	// bit 3: Receive Enable
    while (!I2C1STATbits.RBF);	// bit 1:wait for receive buffer full
    byte = I2C1RCV;		// get byte
    while (I2C1CON & 0x1f);	// again, wait for idle
    I2C1CONbits.ACKDT = ack;
    I2C1CONbits.ACKEN = 1;	// bit 4: send ACK/NACK
    while (I2C1CONbits.ACKEN);	// bit 4: wait for completion

    return byte;
}

static unsigned int
i2c_sendrecv(unsigned int nsend, unsigned char *send_buf,
	unsigned int nrecv, unsigned char *recv_buf)
{
    unsigned int i;

    I2C1start();	// establish the start condition

    if (nsend > 1)	// more than just the address?
    {
	// Send slave address
	if (!i2c_send(send_buf[0] & ~0x01))	// clear R/W bit -> WRite
	{
	    // address byte failure
	    nsend = 0;	// no more data to send
	    nrecv = 0;	// no data to receive
	}

	// Start at 1 as we have already sent the slave address
	for (i = 1; i < nsend; i++)
	{
	    if (!i2c_send(send_buf[i]))
		break;
	}
    }

    if (nrecv != 0)
    {
	/*
	 * If we sent more than just the address,
	 * then we need a restart
	 */
	if (nsend > 1)
	    I2C1restart();

	/* (Re-)send address */
	if (!i2c_send(send_buf[0] | 0x01))	// set R/W bit -> ReaD
	    nrecv = 0;

	for (i = 0; i < nrecv; i++)
	{
	    /*
	     * Receive next byte
	     * If more bytes are expected, send ACK
	     * If it is the last byte we expect, send NACK
	     */
	    recv_buf[i] = i2c_recv((i < (nrecv-1)) ? I2C_ACK : I2C_NACK);
	}
    }

    return nrecv;
}

void
i2c(char *args)
{
    unsigned char buf[32];
    unsigned int i, nsend, nrecv;
    static unsigned char initialized = 0;

    nsend = 0;
    for (args = skip_white_space(args); isxdigit(*args); args = skip_white_space(args))
    {
	unsigned int value;

	for (value = 0; *args != '\0' && isxdigit(*args); args++)
	    if (*args >= '0' && *args <= '9')
	        value = (value * 16) | (*args - '0');
	    else if (*args >= 'a' && *args <= 'f')
	        value = (value * 16) | (*args - 'a' + 10);
	    else if (*args >= 'A' && *args <= 'F')
	        value = (value * 16) | (*args - 'A' + 10);
	if (nsend < sizeof(buf))
	    buf[nsend++] = value;
    }

    if (nsend < 1)
    {
        my_puts("? Address\n");
	return;
    }

    if (*args == '-')
    {
	args = skip_white_space(&args[1]); // skip '-' and following white space

	for (nrecv = 0; *args != '\0' && isxdigit(*args); args++)
	    if (*args >= '0' && *args <= '9')
	        nrecv = (nrecv * 16) | (*args - '0');
	    else if (*args >= 'a' && *args <= 'f')
	        nrecv = (nrecv * 16) | (*args - 'a' + 10);
	    else if (*args >= 'A' && *args <= 'F')
	        nrecv = (nrecv * 16) | (*args - 'A' + 10);
    }
    else
        nrecv = 0;

    if (nrecv > sizeof(buf))
    {
        my_puts("? recv size\n");
	return;
    }

    if (!initialized)
    {
	i2c_init(0x25);		// full speed (400kHz) @ 32/16 MHz

	initialized = 1;
    }

    // We re-use buf to hold the received data
    nrecv = i2c_sendrecv(nsend, buf, nrecv, buf);

    if (nrecv)
    {
	for (i = 0; i < nrecv; i++)
	{
	    my_putc(' ');
	    my_puthex8(buf[i]);
	}
	my_putc('\n');
    }
}

/**********\
 * TIMER1 *
\**********/
static volatile unsigned int t1cnt;

void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void)
{
    if (t1cnt)
    {
        t1cnt--;

	my_putc('T');
    }
    else
        IEC0bits.T1IE = 0;
}

void
timer1(char *args)
{
    t1cnt = 0;

    args = skip_white_space(args);

    for (t1cnt = 0; *args != '\0' && isxdigit(*args); args++)
	if (*args >= '0' && *args <= '9')
	    t1cnt = (t1cnt * 16) | (*args - '0');
	else if (*args >= 'a' && *args <= 'f')
	    t1cnt = (t1cnt * 16) | (*args - 'a' + 10);
	else if (*args >= 'A' && *args <= 'F')
	    t1cnt = (t1cnt * 16) | (*args - 'A' + 10);

    IEC0bits.T1IE = 1;
}

/******************************************************************************/
char *
skip_white_space(char *s)
{
    /* skip white space */
    while ((*s != '\0') && isspace(*s))
	s++;

    return s;
}

void
setup(void)
{
    // OSCCONbits.SOSCEN = 0;	// Disables Secondary Oscillator

    UART_init(DEFAULT_U1BRG, 1);

    my_puts("PIC24FJ64GA002\n");
    my_puts("RCON="); my_puthex16(RCON); my_putc('\n');
    RCON = 0;
}

static inline int
strcasecmp(const char *s1, const char *s2)
{
    int result;
    if (s1 == s2)
        return 0;
    while ((result = tolower(*s1) - tolower(*s2++)) == 0)
        if (*s1++ == '\0')
	    break;

    return result;
}

struct {
    char *name;
    unsigned short addr;
} reg2addr[] = {
    /* INTERRUPT CONTROLLER REGISTER MAP */
    { "INTCON1", 0x0080 },
    { "INTCON2", 0x0082 },
    { "IFS0", 0x0084 },
    { "IEC0", 0x0094 },
    { "IPC0", 0x00a4 },

    /* TIMER REGISTER MAP */
    { "TMR1", 0x0100 },
    { "PR1", 0x0102 },
    { "T1CON", 0x0104 },
    { "TMR2", 0x0106 },
    { "TMR3HLD", 0x0108 },
    { "TMR3", 0x010A },
    { "PR2", 0x010C },
    { "PR3", 0x010E },
    { "T2CON", 0x0110 },
    { "T3CON", 0x0112 },
    { "TMR4", 0x0114 },
    { "TMR5HLD", 0x0116 },
    { "TMR5", 0x0118 },
    { "PR4", 0x011A },
    { "PR5", 0x011C },
    { "T4CON", 0x011E },
    { "CT5CON", 0x0120 },

    /* I2C(TM) REGISTER MAP */
    { "I2C1RCV", 0x0200 },
    { "I2C1TRN", 0x0202 },
    { "I2C1BRG", 0x0204 },
    { "I2C1CON", 0x0206 },
    { "I2C1STAT", 0x0208 },
    { "I2C1ADD", 0x020A },
    { "I2C1MSK", 0x020C },
    { "I2C2RCV", 0x0210 },
    { "I2C2TRN", 0x0212 },
    { "I2C2BRG", 0x0214 },
    { "I2C2CON", 0x0216 },
    { "I2C2STAT", 0x0218 },
    { "I2C2ADD", 0x021A },
    { "I2C2MSK", 0x021C },
 
    /* UART REGISTER MAP */
    { "U1MODE", 0x0220 },
    { "U1STA", 0x0222 },
    { "U1TXREG", 0x0224 },
    { "U1RXREG", 0x0226 },
    { "U1BRG", 0x0228 },
    { "U2MODE", 0x0230 },
    { "U2STA", 0x0232 },
    { "U2TXREG", 0x0234 },
    { "U2RXREG", 0x0236 },
    { "U2BRG", 0x0238 },

    /* PORTA REGISTER MAP */
    { "TRISA", 0x02C0 },
    { "PORTA", 0x02C2 },
    { "LATA", 0x02C4 },
    { "ODCA", 0x02C6 },
    /* PORTB REGISTER MAP */
    { "TRISB", 0x02C8 },
    { "PORTB", 0x02CA },
    { "LATB", 0x02CC },
    { "ODCB", 0x02CE },
    /* PORTC REGISTER MAP */
    { "TRISC", 0x02D0 },
    { "PORTC", 0x02D2 },
    { "LATC", 0x02D4 },
    { "ODCC", 0x02D6 },

    /* PAD CONFIGURATION REGISTER MAP */
    { "PADCFG1", 0x02FC },

    /* CLOCK CONTROL REGISTER MAP */
    { "RCON", 0x740 },
    { "OSCCON", 0x0742 },
    { "CLKDIV", 0x0744 },
    { "OSCTUN", 0x0746 },
};

struct {
    char *name;
    void (*func)(char *);
} functab[] = {
    { "i", i2c },
    { "timer1", timer1 },
};

/*
 * i <n>+ : <m>
 * <n> bytes to send, 1st byte is address
 * <m> number of receive bytes
 */
void
loop(void)
{
    char line[32];

    my_putc('>');

    if (my_gets(line, sizeof(line)))
    {
        char *bp = line;
	char *s, sav;
	int i;
	char op;
	unsigned int value;
	int invert;
	unsigned int addr;

	bp = skip_white_space(bp);

	if (*bp == '\0')
	    return;

	if (*bp == '*')	// *<addr>
	{
	    bp++;	// skip '*'

	    if (!isxdigit(*bp))
	    {
	        my_puts("? addr\n");
		return;
	    }

	    for (addr = 0; *bp != '\0' && isxdigit(*bp); bp++)
		if (*bp >= '0' && *bp <= '9')
		    addr = (addr * 16) | (*bp - '0');
		else if (*bp >= 'a' && *bp <= 'f')
		    addr = (addr * 16) | (*bp - 'a' + 10);
		else if (*bp >= 'A' && *bp <= 'F')
		    addr = (addr * 16) | (*bp - 'A' + 10);
	}
	else
	{
	    for (s = bp+1; *s != '\0' && isalnum(*s); s++)
		;

	    sav = *s;
	    *s = '\0';
	    for (i = 0; i < (sizeof(functab) / sizeof(functab[0])); i++)
		if (strcasecmp(bp, functab[i].name) == 0)
		    break;

	    if (i < (sizeof(functab) / sizeof(functab[0])))
	    {
		*s = sav;
		functab[i].func(s);
		return;
	    }

	    for (i = 0; i < (sizeof(reg2addr) / sizeof(reg2addr[0])); i++)
		if (strcasecmp(bp, reg2addr[i].name) == 0)
		    break;

	    if (i >= (sizeof(reg2addr) / sizeof(reg2addr[0])))
	    {
		my_puts("? name\n");
		return;
	    }

	    *s = sav;
	    bp = s;

	    addr = reg2addr[i].addr;
	}

	bp = skip_white_space(bp);

	if (*bp == '\0')
	{
	    my_puthex16(*(unsigned int *) addr);
	    my_putc('\n');

	    return;
	}

	if (*bp == '=')
	{
	    op = '=';
	    bp++;
	}
	else if (*bp == '|' && bp[1] == '=')
	{
	    op = '|';
	    bp += 2;
	}
	else if (*bp == '&' && bp[1] == '=')
	{
	    op = '&';
	    bp += 2;
	}
	else
	{
	    my_puts("? op\n");
	    return;
	}

	bp = skip_white_space(bp);

	if (invert = (*bp == '~'))
	    bp++;

	/* No blanks between ~ and number */
	if (*bp == '\0' || !isxdigit(*bp))
	{
	    my_puts("? value\n");
	    return;
	}

	for (value = 0; *bp != '\0' && isxdigit(*bp); bp++)
	    if (*bp >= '0' && *bp <= '9')
	        value = (value * 16) | (*bp - '0');
	    else if (*bp >= 'a' && *bp <= 'f')
	        value = (value * 16) | (*bp - 'a' + 10);
	    else if (*bp >= 'A' && *bp <= 'F')
	        value = (value * 16) | (*bp - 'A' + 10);

	if (invert)
	    value = ~value;

	switch (op)
	{
	case '=': 
	    *(unsigned int *) addr = value;
	    break;
	case '|': 
	    *(unsigned int *) addr |= value;
	    break;
	case '&': 
	    *(unsigned int *) addr &= value;
	    break;
	}

	my_puthex16(*(unsigned int *) addr);
	my_putc('\n');
    }
}

void
main(void)
{
    /*
     * Wait for PLL to synchronize
     */
    while (!OSCCONbits.LOCK) ;
    /*
     * Set FRC Postscaler to 1:1
     */
    CLKDIVbits.RCDIV=0;

    /*
     * Pin for corresponding analog channel is configured in Digital mode;
     * I/O port read is enabled
     */
    AD1PCFG = 0xffff;

    setup();

    while (1)
        loop()
	;
}
