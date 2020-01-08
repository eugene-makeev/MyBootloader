/*
 * MyBootloader.c
 *
 * Created: 1/4/2020 11:59:39 PM
 * Author : user
 */ 

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <stdbool.h>

#define MAIN_FW_ADDRESS		(0)
#define F_CPU				(16000000L)

#define SUPPORT_FW_UPDATE	(0)

#define BAUD_RATE_115200	(115200L)
#define BAUD_RATE_57600		(57600L)

#define FLASH_PAGE_SIZE_WORDS	(64)
#define FLASH_PAGE_SIZE_BYTES	(FLASH_PAGE_SIZE_WORDS * sizeof(uint16_t))

#define GCODE_BASE_ADDR			(0x3E00) // TBD
#define GCODE_MAX_SIZE_BYTES	(2048)
#define GCODE_MAX_SIZE_PAGES	(GCODE_MAX_SIZE_BYTES / FLASH_PAGE_SIZE_BYTES)


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

uint8_t uart_buffer[FLASH_PAGE_SIZE_BYTES];


void gpio_init(void)
{
	// reset/abort button, input, pullup enabled
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
		UCSR0A &= ~BIT(U2X0); // baud doubler off  - Only needed on Uno XXX
	}
	else
	{
		ubrr0_value = ((F_CPU / (4L * baud_rate)) - 1)/2;
		UCSR0A |= BIT(U2X0);  // baud doubler on for high baud rates, i.e. 115200		
	}

	UBRR0H = ubrr0_value >> 8;
	UBRR0L = ubrr0_value;

	// defaults to 8-bit, no parity, 1 stop bit
	UCSR0C = BIT(USBS0) | BIT(UCSZ00);  // Asynchronous mode 8-bit data and 1-stop bit
	UCSR0B = BIT(RXEN0) | BIT(TXEN0);   // Enable Receiver and Transmitter
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

void uart_print(const char * str)
{
	while (*str)
	{
		if (*str == '\n')
		{
			uart_putch('\r');
		}
		
		uart_putch(*str++);
	}
}

void led_toggle(void)
{
	LED_TOGGLE;
}

void fill_page_buffer(uint16_t address, uint8_t * buffer)
{
	uint8_t page_word = 0;
	uint16_t data;
	
	while (page_word < FLASH_PAGE_SIZE_WORDS)
	{
		data = *buffer++;
		data |= (*buffer++) << 8;
		boot_page_fill(address, data);
		address += sizeof(uint16_t);
	}
}

uint8_t get_page_data(uint8_t * buffer)
{
	uint8_t page_bytes = 0;
	
	while (page_bytes < FLASH_PAGE_SIZE_BYTES)
	{
		*buffer++ = uart_getch();
		uart_putch('*');
		page_bytes++;
	}
	
	return page_bytes;
}

void update_gcode(void)
{
	uint8_t gcode_pages = 0;
	bool last_page = false;
	uint16_t address = GCODE_BASE_ADDR;
	
	uart_print("Update G-code");
	
	do
	{
		if (last_page)
		{
			//boot_page_erase_safe(address);
		}
		else
		{
			uint8_t bytes_received = get_page_data(uart_buffer);
		
			// fill the page till the end to erase previous gcode file
			if (bytes_received < FLASH_PAGE_SIZE_BYTES)
			{
				last_page = true;
				while (bytes_received < FLASH_PAGE_SIZE_BYTES)
				{
					uart_buffer[bytes_received] = 0xFF;
					bytes_received++;				
				}
			}
		
			fill_page_buffer(address, uart_buffer);
			//boot_page_erase(address);
			//boot_page_write_safe(address);			
		}

		address += FLASH_PAGE_SIZE_WORDS;			
		gcode_pages++;
	} while ((gcode_pages < GCODE_MAX_SIZE_PAGES));
}

void update_fw(void)
{
	uart_print("Update FW");
}

void restart(void)
{
	wdt_disable();
	reset();
}

ISR(WDT_vect)
{
	LED_TOGGLE;
	wdt_reset();
}

int main(void)
{
	gpio_init();
	
	if (BUTTON == DOWN)
	{
		LED_ON;
		
		uart_init(BAUD_RATE_115200);
		wdt_enable(WDTO_1S);
		
		while (1)
		{
			uart_print("G - ???????? g-???? ?????????\n");
#if (SUPPORT_FW_UPDATE == 1)
			uart_print("F - ?????????? ????????\n");
#endif
			uart_print("R - ?????\n");
			
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

