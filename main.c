/*
 * MyBootloader.c
 *
 * Created: 1/4/2020 11:59:39 PM
 * Author : user
 */ 
#define F_CPU				(16000000UL)

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <stdbool.h>
#include <util/delay.h>

#define BOOTLOADER_ADDRESS	(0x7000)
#define MAIN_FW_ADDRESS		(0)

#define SUPPORT_FW_UPDATE	(0)

#define BAUD_RATE_115200	(115200L)
#define BAUD_RATE_57600		(57600L)
#define BAUD_RATE_38400		(38400L)

#define FLASH_PAGE_SIZE_WORDS	(64)
#define FLASH_PAGE_SIZE_BYTES	(FLASH_PAGE_SIZE_WORDS * sizeof(uint16_t))

#define GCODE_MAX_SIZE_BYTES	(2048)
#define GCODE_MAX_SIZE_PAGES	(GCODE_MAX_SIZE_BYTES / SPM_PAGESIZE)

#define GCODE_BASE_ADDR			(BOOTLOADER_ADDRESS - GCODE_MAX_SIZE_BYTES)

#define BIT(x)				(1 << x)
#define BIT_GET(data, bt)	(data & BIT(bt) ? 1 : 0)
#define BIT_SET(data, bt)	(data |= BIT(bt))
#define BIT_CLR(data, bt)	(data &= ~BIT(bt))
#define BIT_INV(data, bt)	(data ^= BIT(bt))

#define PIN_VALUE(port, pin) (BIT_GET(port, pin))

#define UP			(1)
#define DOWN		(0)

#define PGM_PORT	(PINC)
#define PGM_PIN		(PORTC0)

#define LED_PORT	(PORTB)
#define LED_PIN		(PORTB5)

#define SET_PIN(port, pin, val) (val ? BIT_SET(port, pin) : BIT_CLR(port, pin))
#define GET_PIN(port, pin) (BIT_GET(port, pin))
#define TOGGLE_PIN(port, pin) (BIT_INV(port, pin))

#define LED_ON		(BIT_SET(LED_PORT, LED_PIN))
#define LED_OFF		(BIT_CLR(LED_PORT, LED_PIN))
#define LED_TOGGLE	(BIT_INV(LED_PORT, LED_PIN))

#define BUTTON		(BIT_GET(PGM_PORT, PGM_PIN))

// declare jump to main FW
typedef void (*p_void_func)(void);
p_void_func reset = (p_void_func) MAIN_FW_ADDRESS;

uint8_t uart_buffer[SPM_PAGESIZE];
bool wdt_isr = false;

void wdt_init(uint8_t timeout, bool isr_en, bool rst_en)
{
	uint8_t wdt_settings =  (isr_en << WDIE) | (rst_en << WDE) | ((timeout > WDTO_2S) << WDP3) | (timeout & 0x07);
	wdt_reset();
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = wdt_settings;
}

void isr_init(void)
{
	// Move interrupts to Boot Flash section
	MCUCR = _BV(IVCE);
	MCUCR = _BV(IVSEL);
}

void clock_init(void)
{
	// no external clock divider
	CLKPR = BIT(CLKPCE);
	CLKPR = 0;
}

void gpio_init(void)
{
	// reset/abort button, input, pull-up enabled
	PORTC = BIT(PORTC0);
	BIT_CLR(DDRC, PORTC0);
	
	// led pin PB5, output, off by default
	DDRB = BIT(PORTB5);
	PORTB = BIT(PORTB5);
}

void uart_init(uint32_t baud_rate)
{
	uint16_t ubrr0_value;
	
	// Set baud rate
	if (baud_rate < BAUD_RATE_57600)
	{
		ubrr0_value = ((F_CPU / (8L * baud_rate)) - 1)/2 ;
		UCSR0A &= ~(1 << U2X0); // baud doubler off
	}
	else
	{
		ubrr0_value = ((F_CPU / (4L * baud_rate)) - 1)/2;
		UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200		
	}

	UBRR0H = ubrr0_value >> 8;
	UBRR0L = ubrr0_value;

	// defaults to 8-bit, no parity, 1 stop bit
	UCSR0C = (3 << UCSZ00);  // Asynchronous mode 8-bit data and 1-stop bit
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable Receiver and Transmitter
}

void uart_putch(uint8_t data)
{
	// Wait for empty transmit buffer
	while (!(UCSR0A & BIT(UDRE0)));
	
	// Put data into buffer, sends the data
	UDR0 = data;
}

uint8_t uart_getch(void)
{
	// Wait for data to be received
	while (!(UCSR0A & BIT(RXC0)));
	
	// Get and return received data from buffer
	return UDR0;
}

bool uart_scan(uint8_t * data, bool timeout)
{
	bool status = false;
	if (timeout)
	{
		cli();
		wdt_reset();
		wdt_isr = false;
		sei();
	}
	
	while (!timeout || !wdt_isr)
	{
		LED_TOGGLE;
		if ((UCSR0A & BIT(RXC0)))
		{
			*data = UDR0;
			status = true;
			break;
		}
	}
	
	return status;
}


void uart_print(const char * str)
{
	while (*str)
	{
 		if (*str == '\n')
 		{
 			uart_putch('\r');
 		}
 		
		uart_putch(*str);
		str++;
	}
}

void fill_page_buffer(uint16_t address, uint8_t * buffer)
{
	uint16_t data;
	
	for (uint8_t offset = 0; offset < SPM_PAGESIZE; offset += sizeof(uint16_t))
	{
 		data = *buffer++;
 		data |= (*buffer++) << 8;
 		boot_page_fill(address + offset, data);
	}
}

void program_page(uint16_t address, uint8_t *buffer)
{
	uint8_t sreg = SREG;
	cli();

	eeprom_busy_wait ();
	boot_page_erase (address);
	boot_spm_busy_wait ();      // Wait until the memory is erased.

	fill_page_buffer(address, buffer);
	boot_page_write (address);     // Store buffer in flash page.
	boot_spm_busy_wait();       // Wait until the memory is written.
	boot_rww_enable ();

	// Re-enable interrupts (if they were ever enabled).
	SREG = sreg;
}

void update_gcode(void)
{
	uint16_t address = GCODE_BASE_ADDR;
	uint8_t bytes_received = 0;
	bool last_page = false;
	
	uart_print("Update G-code:\n");
	
	for (uint8_t gcode_pages = 0; gcode_pages < GCODE_MAX_SIZE_PAGES; gcode_pages++)
	{
		if (last_page)
		{
			boot_page_erase_safe(address);
		}
		else
		{
			// get page from UART
			for (bytes_received = 0; bytes_received < SPM_PAGESIZE; bytes_received++)
			{
				last_page = !uart_scan(&uart_buffer[bytes_received], (bytes_received + gcode_pages > 0));
				if (last_page)
				{
					break;
				}
				uart_putch(uart_buffer[bytes_received]);
			}
			
			// fill the page till the end to erase previous g-code file
			while (bytes_received < SPM_PAGESIZE)
			{
				uart_buffer[bytes_received++] = 0xFF;
			}
			
			program_page(address, uart_buffer);
			bytes_received = 0;
		}

		address += SPM_PAGESIZE;			
	}
	
	// skip bytes
	bytes_received = 0;
	while (uart_scan(&uart_buffer[0], true))
	{
		bytes_received++;
	}

	if (bytes_received)
	{
		uart_print("\nWARNING: File exceeds the maximum g-code size 2048 bytes, truncated.\n");
	}	
	uart_print("\nDone\n");

	
	boot_rww_enable();
}

void update_fw(void)
{
	uart_print("Update FW:\n");
}

void restart(void)
{
	wdt_disable();
	reset();
}

ISR(WDT_vect)
{
	wdt_reset();
	WDTCSR |= (1 << WDIE);
	wdt_isr = true;
	LED_TOGGLE;
}

int main(void)
{
	wdt_init(WDTO_1S, true, false);
	isr_init();
	gpio_init();
	sei();
	
	if (BUTTON == DOWN)
	{
		uart_init(BAUD_RATE_115200);
		wdt_init(WDTO_250MS, true, false);
		
		while (1)
		{
			//uart_print("\033[H");
			uart_print("G - update g-code\n");
#if (SUPPORT_FW_UPDATE == 1)
			uart_print("F - update firmware\n");
#endif
			uart_print("R - reset\n");
			
			uint8_t input = uart_getch();
			
			switch (input)
			{
			case 'G':
			case 'g':
				update_gcode();
				break;
#if (SUPPORT_FW_UPDATE == 1)				
			case 'F':
			case 'f':
				update_fw();
				break;
#endif				
			case 'R':
			case 'r':
				restart();
				break;
			default:
				break;
			}		
		}
	}

	reset();	
}

