#include <iostm8s103f3.h>
#include "binary_clock.h"
#include "delay.h"
#include "scheduler.h"
#include "i2c_bb.h"
#include "i2c_ds3231_bb.h"
#include "uart.h"
#include "eep.h"

extern uint32_t t2_millis;      // Updated in TMR2 interrupt

char     rs232_inbuf[UART_BUFLEN]; // buffer for RS232 commands
uint8_t  rs232_ptr     = 0;        // index in RS232 buffer
char     bin_clk_ver[] = "Binary Clock v0.1\n";

uint8_t led_r[NR_LEDS]; // Array with 8-bit red colour for all WS2812
uint8_t led_g[NR_LEDS]; // Array with 8-bit green colour for all WS2812
uint8_t led_b[NR_LEDS]; // Array with 8-bit blue colour for all WS2812
uint8_t seconds = 0;
uint8_t minutes = 0;
uint8_t hours   = 0;
uint8_t enable_test_pattern = 0; // 1 = enable WS2812 test-pattern
uint8_t watchdog_test       = 0; // 1 = watchdog test modus
uint8_t led_intensity;           // Intensity of WS2812 LEDs

/*-----------------------------------------------------------------------------
  Purpose  : This is the interrupt routine for the Timer 2 Overflow handler.
             It runs at 1 kHz and drives the scheduler and the multiplexer.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
    scheduler_isr();  // Run scheduler interrupt function
    t2_millis++;      // update milliseconds timer
    if (!(t2_millis % 250)) set_colon_leds(0x08);
    TIM2_SR1_UIF = 0; // Reset the interrupt otherwise it will fire again straight away.
} // TIM2_UPD_OVF_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises the system clock to run at 16 MHz.
             It uses the internal HSI oscillator.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void initialise_system_clock(void)
{
    CLK_ICKR       = 0;           //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;           //  Enable the HSI.
    while (CLK_ICKR_HSIRDY == 0); //  Wait for the HSI to be ready for use.
    CLK_CKDIVR     = 0;           //  Ensure the clocks are running at full speed.
 
    // The datasheet lists that the max. ADC clock is equal to 6 MHz (4 MHz when on 3.3V).
    // Because fMASTER is now at 16 MHz, we need to set the ADC-prescaler to 4.
    ADC_CR1_SPSEL  = 0x02;        //  Set prescaler to 4, fADC = 4 MHz
    CLK_SWIMCCR    = 0;           //  Set SWIM to run at clock / 2.
    CLK_SWR        = 0xe1;        //  Use HSI as the clock source.
    CLK_SWCR       = 0;           //  Reset the clock switch control register.
    CLK_SWCR_SWEN  = 1;           //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);  //  Pause while the clock switch is busy.
} // initialise_system_clock()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 2 to generate a 1 kHz interrupt.
             16 MHz / (  16 *  1000) = 1000 Hz (1000 = 0x03E8)
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer2(void)
{
    TIM2_PSCR    = 0x04;  //  Prescaler = 16
    TIM2_ARRH    = 0x03;  //  High byte of 1000
    TIM2_ARRL    = 0xE8;  //  Low  byte of 1000
    TIM2_IER_UIE = 1;     //  Enable the update interrupts
    TIM2_CR1_CEN = 1;     //  Finally enable the timer
} // setup_timer2()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises all the GPIO pins of the STM8 uC.
             See binary_clock.h for a detailed description of all pin-functions.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_output_ports(void)
{
  PB_ODR     |=  (I2C_SCL | I2C_SDA);   // Must be set here, or I2C will not work
  PB_DDR     |=  (I2C_SCL | I2C_SDA);   // Set as outputs
  PB_CR2     &= ~(I2C_SCL | I2C_SDA);   // O: Set speed to 2 MHz, I: disable IRQ
  
  PC_DDR     |= DI_3V3 | LED3 | LED4;   // Set as output
  PC_CR1     |= DI_3V3 | LED3 | LED4;   // Set to Push-Pull
  PC_ODR     &= ~(DI_3V3 | LED3| LED4); // Turn off outputs
  
  PD_DDR     |= TX | LED1 | LED2;       // Set as output
  PD_CR1     |= TX | LED1 | LED2;       // Set to Push-Pull
  PD_ODR     |= TX;                     // Set TX high
  PD_ODR     &= ~(LED1 | LED2);         // LEDs off
  PD_DDR     &= ~RX;                    // Set UART1-RX as input
  PD_CR1     &= ~RX;                    // Set to floating
} // setup_output_ports()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends one byte to the WS2812B LED-string.
  Variables: bt: the byte to send
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812b_send_byte(uint8_t bt)
{
    uint8_t i,x = 0x80; // Start with MSB first
    
    for (i = 0; i < 8; i++)
    {
        if (bt & x)
        {    // Send a 1   
             ws2812b_send_1;
        } // if
        else 
        {   // Send a 0
            ws2812b_send_0;
        } // else
        x >>= 1; // Next bit
    } // for i
} // ws2812b_send_byte()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends a test pattern to all WS2812B LEDs. It is 
             called by pattern_task() every 100 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void test_pattern(void)
{
    static uint8_t cntr_b = 0, tmr_b = 0;
    uint8_t i;

    if (++tmr_b >= 20)
    {   // change colour every 2 seconds
        tmr_b = 0;
        switch (cntr_b)
        {
            case 0: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = 0x10;
                    led_g[i] = led_r[i] = 0x00;
                } // for
                cntr_b = 1; // next colour
                break;
            case 1: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = led_r[i] = 0x00;
                    led_g[i] = 0x10;
                } // for
                cntr_b = 2;
                break;
            case 2: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = led_g[i] = 0x00;
                    led_r[i] = 0x10;
                } // for
                cntr_b = 0;
                break;
        } // switch
    } // if
} // test_pattern()

/*-----------------------------------------------------------------------------
  Purpose: This functions sets or reset the colon leds of the binary clock.
           There are 4 leds and they are coded as:
                 LED1     LED3
                LED2     LED4
  Variables: 
       leds: bit 0: LED1 ; bit 1: LED2 ; bit 2: LED3 ; bit 3: LED4
  Returns  : -
  ---------------------------------------------------------------------------*/
void set_colon_leds(uint8_t leds)
{
	if (leds & 0x01) PD_ODR |=  LED1;
	else             PD_ODR &= ~LED1;
	if (leds & 0x02) PD_ODR |=  LED2;
	else             PD_ODR &= ~LED2;
	if (leds & 0x04) PC_ODR |=  LED3;
	else             PC_ODR &= ~LED3;
	if (leds & 0x08) PC_ODR |=  LED4;
	else             PC_ODR &= ~LED4;
} // set_colon_leds()

uint8_t get_colon_leds(void)
{
    uint8_t retv = 0;
    
    if (PD_IDR & LED1) retv |= 0x01;
    if (PD_IDR & LED2) retv |= 0x02;
    if (PC_IDR & LED3) retv |= 0x04;
    if (PC_IDR & LED4) retv |= 0x08;
    return retv;
} // get_colon_leds()

/*------------------------------------------------------------------------
  Purpose  : Encode a byte into 2 BCD numbers.
  Variables: x: the byte to encode
  Returns  : the two encoded BCD numbers
  ------------------------------------------------------------------------*/
uint8_t encode_to_bcd(uint8_t x)
{
	uint8_t temp;
	uint8_t retv = 0;
		
	temp   = x / 10;
	retv  |= (temp & 0x0F);
	retv <<= 4; // SHL 4
	temp   = x - temp * 10;
	retv  |= (temp & 0x0F);
	return retv;
} // encode_to_bcd()

/*-----------------------------------------------------------------------------
  Purpose  : This routine creates a pattern for the LEDs and stores it in
             the arrays led_r, led_g and led_b
             It uses the global variables seconds, minutes and hours and is
             called every 100 msec. by the scheduler.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void pattern_task(void)
{
    uint8_t x,i;
    //static uint8_t colon_tmr = 0;
    
    if (enable_test_pattern)
    {   // WS2812 test-pattern
	test_pattern(); 
    } // if
    else
    {
    	x = encode_to_bcd(seconds);
    	for (i = 0; i <= 3; i++)
    	{   // 4 bits seconds LSB: LED 16-19
        	if (x & (1<<i)) led_b[i+16] = led_intensity;
        	else            led_b[i+16] = 0x00;
    	} // for i
    	for (i = 4; i <= 6; i++)
    	{   // 3 bits seconds MSB: LED 13-15
        	if (x & (1<<i)) led_b[i+9] = led_intensity;
        	else            led_b[i+9] = 0x00;
    	} // for i
    	x = encode_to_bcd(minutes);
    	for (i = 0; i <= 3; i++)
    	{   // 4 bits minutes LSB: LED 09-12
    	    if (x & (1<<i)) led_g[i+9] = led_intensity;
    	    else            led_g[i+9] = 0x00;
    	} // for i
    	for (i = 4; i <= 6; i++)
    	{   // 3 bits minutes MSB: LED 06-08
    	    if (x & (1<<i)) led_g[i+2] = led_intensity;
    	    else            led_g[i+2] = 0x00;
    	} // for i
    	x = encode_to_bcd(hours);
    	for (i = 0; i <= 3; i++)
    	{   // 4 bits hours LSB: LED 02-05
    	    if (x & (1<<i)) led_r[i+2] = led_intensity;
    	    else            led_r[i+2] = 0x00;
    	} // for i
    	for (i = 4; i <= 5; i++)
    	{   // 4 bits hours MSB: LED 00-01
    	    if (x & (1<<i)) led_r[i-4] = led_intensity;
    	    else            led_r[i-4] = 0x00;
    	} // for i
//        if (++colon_tmr == 5)
//        {
//            colon_tmr = 0;
//            if (get_colon_leds() == 0x0C)
//                 set_colon_leds(0x03);
//            else set_colon_leds(0x0C);
//        }
    } // else
    set_colon_leds(0x04);
} // pattern_task()    
        
/*-----------------------------------------------------------------------------
  Purpose  : This routine sends the RGB-bytes for every LED to the WS2812B
             LED string. It is called every 100 msec. by the scheduler.
  Variables: 
      led_g: the (global) green byte array
      led_r: the (global) red byte array
      led_b: the (global) blue byte array
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812_task(void)
{
    uint8_t i;

    __disable_interrupt();   // disable IRQ for time-sensitive LED-timing
    for (i = 0; i < NR_LEDS; i++)
    {
        ws2812b_send_byte(led_g[i]); // Send one byte of Green
        ws2812b_send_byte(led_r[i]); // Send one byte of Red
        ws2812b_send_byte(led_b[i]); // Send one byte of Blue
    } // for i
    __enable_interrupt(); // enable IRQ again
    if (!watchdog_test)   // only refresh when watchdog_test == 0 (X0 command)
        IWDG_KR = IWDG_KR_KEY_REFRESH;   // Refresh watchdog (reset after 500 msec.)
    set_colon_leds(0x01);
} // ws2812_task()

/*-----------------------------------------------------------------------------
  Purpose  : This routine reads the date and time info from the DS3231 RTC and
             stores this info into the global variables seconds, minutes and
             hours.
  Variables: 
    seconds: global variable [0..59]
    minutes: global variable [0..59]
      hours: global variable [0..23]
  Returns  : -
  ---------------------------------------------------------------------------*/
void clock_task(void)
{
    Time p;

    ds3231_gettime(&p);
    __disable_interrupt();
    seconds = p.sec;
    minutes = p.min;
    hours   = p.hour;
    __enable_interrupt();
    set_colon_leds(0x02);
} // clock_task()

/*-----------------------------------------------------------------------------
  Purpose  : This routine prints the day-of-week to the uart
  Variables: 
        dow: [0..7], 1=Monday, 7 = Sunday
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_dow(uint8_t dow)
{
    char day[8][4] = {"???","Mon","Tue","Wed","Thu","Fri","Sat","Sun"};

    uart_printf(day[dow]);
} // print_dow()

/*-----------------------------------------------------------------------------
  Purpose  : This routine reads the time and date from the DS3231 RTC and 
             prints this info to the uart.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_date_and_time(void)
{
    char s2[40]; // Used for printing to UART
    Time p;
    
    ds3231_gettime(&p);
    uart_printf("DS3231: ");
    print_dow(p.dow);
    sprintf(s2," %d-%d-%d, %d:%d.%d\n",p.date,p.mon,p.year,p.hour,p.min,p.sec);
    uart_printf(s2);
} // print_date_and_time()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  Returns  : -
  ---------------------------------------------------------------------------*/
void execute_single_command(char *s)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   char     s2[40]; // Used for printing to RS232 port
   char     *s1;
   uint8_t  d,m;
   uint16_t i,y;
   int16_t  temp;
   
   switch (s[0])
   {
	case 'd': // Set Date, 1 = Get Date
		 switch (num)
		 {
                    case 0: // Set Date
			    s1 = strtok(&s[3],":-");
                            d  = atoi(s1);
                            s1 = strtok(NULL ,":-");
                            m  = atoi(s1);
                            s1 = strtok(NULL ,":-");
                            y  = atoi(s1);
                            uart_printf("Date: ");
                            print_dow(ds3231_calc_dow(d,m,y));
                            sprintf(s2," %d-%d-%d\n",d,m,y);
                            uart_printf(s2);
                            ds3231_setdate(d,m,y); // write to DS3231 IC
                            break;
                    case 1: // Set Time
                            s1      = strtok(&s[3],":-.");
                            hours   = atoi(s1);
                            s1      = strtok(NULL ,":-.");
                            minutes = atoi(s1);
                            s1      = strtok(NULL ,":-.");
                            seconds = atoi(s1);
                            sprintf(s2,"Time: %d:%d:%d\n",hours,minutes,seconds);
                            uart_printf(s2);
                            ds3231_settime(hours,minutes,seconds); // write to DS3231 IC
                            break;
                    case 2: // Get Date & Time
                            print_date_and_time(); 
                            break;
                    case 3: // Get Temperature
                            temp = ds3231_gettemp();
                            sprintf(s2,"DS3231: %d.",temp>>2);
                            uart_printf(s2);
                            switch (temp & 0x03)
                            {
				case 0: uart_printf("00 C\n"); break;
				case 1: uart_printf("25 C\n"); break;
				case 2: uart_printf("50 C\n"); break;
				case 3: uart_printf("75 C\n"); break;
                            } // switch
                            break;
                   default: break;
                 } // switch
                 break;
  
	case 'i': // Set intensity of WS2812 LEDs between 1..255
		 if (num > 0)
                 {
                     led_intensity = num;
                     eeprom_write_config(EEP_LED_INTENSITY,led_intensity);
                 } // if
		 break;

	case 'l': // Switch colon leds
		 set_colon_leds(num);
		 break;

	case 's': // System commands
		 switch (num)
		 {
                    case 0: // revision
                            uart_printf(bin_clk_ver);
                            break;
                    case 1: // List all tasks
                            list_all_tasks(); 
                            break;
                    case 2: // I2C-scan
			    uart_printf("I2C-scan: ");
			    for (i = 0x02; i < 0xff; i+=2)
			    {
				if (i2c_start_bb(i) == I2C_ACK)
				{
					sprintf(s2,"0x%x, ",i);
		  		    	uart_printf(s2);
				} // if
				i2c_stop_bb();
			    } // for
			    uart_putc('\n');
                            break;
                   default: break;
                 } // switch
		 break;
                                        
	case 'w': // WS2812 test-pattern command
		 enable_test_pattern = num; // 1 = enable test-pattern
                 if (!num)
                 {  // clear all leds when finished with test-pattern
                    for (i = 0; i < NR_LEDS; i++)
                    {
                        led_g[i] = led_r[i] = led_b[i] = 0x00;
                    } // for i
                 } // if
		 break;

        default: break;
   } // switch
} // execute_single_command()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the USB port
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void rs232_command_handler(void)
{
  char    ch;
  static uint8_t cmd_rcvd = 0;
  
  if (!cmd_rcvd && uart_kbhit())
  { // A new character has been received
    ch = tolower(uart_getc()); // get character as lowercase
    switch (ch)
	{
            case '\n': break;
            case '\r': cmd_rcvd  = 1;
		       rs232_inbuf[rs232_ptr] = '\0';
		       rs232_ptr = 0;
                       uart_putc('\n');
                       break;
            default  : if (rs232_ptr < UART_BUFLEN)
                       {   
                           rs232_inbuf[rs232_ptr++] = ch;
                           uart_putc(ch);
                       }
                       else rs232_ptr = 0;
                       break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
	  cmd_rcvd = 0;
	  execute_single_command(rs232_inbuf);
  } // if
} // rs232_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : This functions initializes the independent watchdog (IWDG) and 
             sets the watchdog timeout to the maximum of T = 512 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void init_watchdog(void)
{
	IWDG_KR  = IWDG_KR_KEY_ENABLE;  // start the IWDG
	IWDG_KR  = IWDG_KR_KEY_ACCESS;  // enable access to IWDG_PR and IWDG_RLR registers
	IWDG_PR  = 0x05;                // prescaler divider 128
	IWDG_RLR = 0xFF;	        // Reload register to maximum
	IWDG_KR  = IWDG_KR_KEY_REFRESH; // reset the IWDG
} // init_watchdog()

/*-----------------------------------------------------------------------------
  Purpose  : This is the main entry-point for the program
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
int main(void)
{
    uint8_t i2c_err;
	
    __disable_interrupt();
    initialise_system_clock(); // Set system-clock to 16 MHz
    setup_output_ports();      // Init. needed output-ports for LED and keys
    setup_timer2();            // Set Timer 2 to 1 kHz
    i2c_err = i2c_reset_bus(); // Init. I2C-peripheral
    uart_init();               // Init. UART-peripheral
    uart_printf(bin_clk_ver);  // Print welcome message
    uart_printf("i2c_reset_bus:");
    uart_putc(0x30+i2c_err);
    uart_putc('\n');
    led_intensity = (uint8_t)eeprom_read_config(EEP_LED_INTENSITY);
    if (!led_intensity)
    {   // First time power-up: eeprom value is 0x00
        led_intensity = 0x10;
        eeprom_write_config(EEP_LED_INTENSITY,led_intensity);
    } // if
    
    // Initialise all tasks for the scheduler
    scheduler_init();                          // clear task_list struct
    add_task(pattern_task, "PTRN"  , 0,  100); // every 100 msec.
    add_task(ws2812_task , "WS2812",50,  100); // every 100 msec.
    add_task(clock_task  , "CLK"   ,80, 1000); // every second
    init_watchdog();                           // init. the IWDG watchdog
    __enable_interrupt();

    while (1)
    {   // background-processes
        dispatch_tasks();        // Run task-scheduler()
        rs232_command_handler(); // run command handler continuously
    } // while
} // main()
