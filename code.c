#include <avr32/io.h>
#include <asf.h>
/*SD card added tuesday morning*/
#include <stdint.h>
#include <stdbool.h>
#include "conf_sd_mmc_spi.h"
#include <string.h>
#include <file.h>



// Dummy char table
const char dummy_data[] =
#include "dummy.h"
;



#define PBA_HZ                32000000

//! \brief Number of bytes in the receive buffer when operating in slave mode
#define BUFFERSIZE            64

#define SD_MMC_CARD_DETECT_PIN      AVR32_PIN_PA00
#define SD_MMC_WRITE_PROTECT_PIN    AVR32_PIN_PA18
#define SD_MMC_SPI                  (&AVR32_SPI0)
#define SD_MMC_SPI_NPCS             1
#define SD_MMC_SPI_SCK_PIN          AVR32_SPI0_SCK_0_0_PIN
#define SD_MMC_SPI_SCK_FUNCTION     AVR32_SPI0_SCK_0_0_FUNCTION
#define SD_MMC_SPI_MISO_PIN         AVR32_SPI0_MISO_0_0_PIN
#define SD_MMC_SPI_MISO_FUNCTION    AVR32_SPI0_MISO_0_0_FUNCTION
#define SD_MMC_SPI_MOSI_PIN         AVR32_SPI0_MOSI_0_0_PIN
#define SD_MMC_SPI_MOSI_FUNCTION    AVR32_SPI0_MOSI_0_0_FUNCTION
#define SD_MMC_SPI_NPCS_PIN         AVR32_SPI0_NPCS_1_0_PIN
#define SD_MMC_SPI_NPCS_FUNCTION    AVR32_SPI0_NPCS_1_0_FUNCTION

#define AVR32_PDCA_CHANNEL_USED_RX AVR32_PDCA_PID_SPI0_RX
#define AVR32_PDCA_CHANNEL_USED_TX AVR32_PDCA_PID_SPI0_TX
#define AVR32_PDCA_CHANNEL_SPI_RX 0 // In the example we will use the pdca channel 0.
#define AVR32_PDCA_CHANNEL_SPI_TX 1 // In the example we will use the pdca channel 1.

volatile avr32_pdca_channel_t* pdca_channelrx ;
volatile avr32_pdca_channel_t* pdca_channeltx ;

// Used to indicate the end of PDCA transfer
volatile bool end_of_transfer;

// Local RAM buffer for the example to store data received from the SD/MMC card
volatile char ram_buffer[1000];
uint8_t sd_buffer[30];

static char par_str1[MAX_FILE_PATH_LENGTH];
static char par_str2[MAX_FILE_PATH_LENGTH];

// Flag to update the timer value
volatile static bool update_timer = true;
// Variable to contain the time ticks occurred
volatile static uint32_t tc_tick = 0;



//xbee
#  define EXAMPLE_USART0                 (&AVR32_USART0) 
#  define EXAMPLE_USART0_RX_PIN          AVR32_USART0_RXD_0_1_PIN 
#  define EXAMPLE_USART0_RX_FUNCTION     AVR32_USART0_RXD_0_1_FUNCTION 
#  define EXAMPLE_USART0_TX_PIN          AVR32_USART0_TXD_0_1_PIN 
#  define EXAMPLE_USART0_TX_FUNCTION     AVR32_USART0_TXD_0_1_FUNCTION 
#  define EXAMPLE_USART0_CLOCK_MASK      AVR32_USART0_CLK_PBA 
#  define EXAMPLE_PDCA_CLOCK_HSB        AVR32_PDCA_CLK_HSB 
#  define EXAMPLE_PDCA_CLOCK_PB         AVR32_PDCA_CLK_PBA 
#  define EXAMPLE_USART0_IRQ             AVR32_USART0_IRQ

//gps
#  define EXAMPLE_USART1                 (&AVR32_USART1) 
#  define EXAMPLE_USART1_RX_PIN          AVR32_USART1_RXD_0_2_PIN 
#  define EXAMPLE_USART1_RX_FUNCTION     AVR32_USART1_RXD_0_2_FUNCTION 
#  define EXAMPLE_USART1_TX_PIN          AVR32_USART1_TXD_0_2_PIN 
#  define EXAMPLE_USART1_TX_FUNCTION     AVR32_USART1_TXD_0_2_FUNCTION 
#  define EXAMPLE_USART1_CLOCK_MASK      AVR32_USART1_CLK_PBA 
#  define EXAMPLE_PDCA_CLOCK_HSB		 AVR32_PDCA_CLK_HSB 
#  define EXAMPLE_PDCA_CLOCK_PB          AVR32_PDCA_CLK_PBA 

//px50-pq2-pwm3
#define EXAMPLE_TC1						(&AVR32_TC1)			
#define EXAMPLE_TC1_CHANNEL2			2					
#define EXAMPLE_TC1_CHANNEL2_ID			2
#define EXAMPLE_TC1_CHANNEL2_PIN        AVR32_TC1_B2_0_PIN    
#define EXAMPLE_TC1_CHANNEL2_FUNCTION   AVR32_TC1_B2_0_FUNCTION
//px15-pl3-pwm2
#define EXAMPLE_TC0						(&AVR32_TC0)
#define EXAMPLE_TC0_CHANNEL0			0					
#define EXAMPLE_TC0_CHANNEL0_ID1		0
#define EXAMPLE_TC0_CHANNEL0_PIN_B      AVR32_TC0_B0_0_1_PIN
#define EXAMPLE_TC0_CHANNEL0_FUNCTION_B AVR32_TC0_B0_0_1_FUNCTION
//67-px16-pl2  -pwm0
#define EXAMPLE_TC0_CHANNEL1			1					
#define EXAMPLE_TC0_CHANNEL1_ID1		1
#define EXAMPLE_TC0_CHANNEL1_PIN_A      AVR32_TC0_A1_0_1_PIN
#define EXAMPLE_TC0_CHANNEL1_FUNCTION_A AVR32_TC0_A1_0_1_FUNCTION
//68-px17-pl1  -pwm4
#define EXAMPLE_TC0_CHANNEL1_PIN_B      AVR32_TC0_B1_0_1_PIN
#define EXAMPLE_TC0_CHANNEL1_FUNCTION_B AVR32_TC0_B1_0_1_FUNCTION
//70-px19-pk7     -pwm1
#define EXAMPLE_TC0_CHANNEL2			2					
#define EXAMPLE_TC0_CHANNEL2_ID1		2
#define EXAMPLE_TC0_CHANNEL2_PIN_B      AVR32_TC0_B2_0_PIN
#define EXAMPLE_TC0_CHANNEL2_FUNCTION_B AVR32_TC0_B2_0_FUNCTION



int i=0;
uint8_t read_data[2];
int8_t read_data_s[2];
uint8_t read_data2[2];
volatile avr32_tc_t *tc0 = EXAMPLE_TC0;//--global declaration for isr
volatile avr32_tc_t *tc1 = EXAMPLE_TC1;
//pressure and temp sensor
extern int usart_getchar_gps(volatile avr32_usart_t *usart);

#define TWIM0               (&AVR32_TWIM0)  //! TWIM Module Used

#define TWIM               (&AVR32_TWIM1)  //! TWIM Module Used


#define WRITE_ADDRESS		  0x77//from 0xEE		//TARGET_ADDRESS     0x05            //! Target's TWI address
#define READ_ADDRESS		  0x77//0xEF		//TARGET_ADDRESS     0x05            //! Target's TWI address
#define TARGET_ADDR_LGT_UP    1               //! Internal Address length for UP
#define VIRTUALMEM_ADDR_UP    0xF4        //! Internal Address
#define TARGET_ADDR_LGT_UT    1               //! Internal Address length for UT
#define VIRTUALMEM_ADDR_UT    0xF4        //! Internal Address

#define TWIM_MASTER_SPEED  400000//150000           //! Speed of TWI



#include <twim.h>


void Serial_PC();
void Serial_init();
void Serial_init2();
void click();
void heart_initialize();
void heartBeat();
void i2c(void);
void i2c2(void);
void twim_init(void);
void i2c3(void);
void twim_init2(void);
void twim_init3(void);
static void usart_int_handler(void);

uint8_t xbeechar[60];
uint8_t startbit=0,endbit=1;
int counter = 0;
int errfound = 0;
int buf1 = 5125, buf2 = 5125, buf3 = 5125,buf4 = 5125, buf5 = 5125;
int ibuf = 0;

//sd card added tuesday morning
void wait()
{
  volatile int i;
  for(i = 0 ; i < 5000; i++);
}


/* interrupt handler to notify if the Data reception from flash is
 * over, in this case lunch the Memory(ram_buffer) to USART transfer and
 * disable interrupt*/
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined (__ICCAVR32__)
__interrupt
#endif
static void pdca_int_handler(void)
{
  // Disable all interrupts.
  Disable_global_interrupt();

  // Disable interrupt channel.
  pdca_disable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_RX);

  sd_mmc_spi_read_close_PDCA();//unselects the SD/MMC memory.
  wait();
  // Disable unnecessary channel
  pdca_disable(AVR32_PDCA_CHANNEL_SPI_TX);
  pdca_disable(AVR32_PDCA_CHANNEL_SPI_RX);

  // Enable all interrupts.
  Enable_global_interrupt();

  end_of_transfer = true;
}


/*! \brief Initializes SD/MMC resources: GPIO, SPI and SD/MMC.
 */
static void sd_mmc_resources_init(void)
{
  // GPIO pins used for SD/MMC interface
  static const gpio_map_t SD_MMC_SPI_GPIO_MAP =
  {
    {SD_MMC_SPI_SCK_PIN,  SD_MMC_SPI_SCK_FUNCTION },  // SPI Clock.
    { SD_MMC_SPI_MISO_PIN, SD_MMC_SPI_MISO_FUNCTION},  // MISO.
    {SD_MMC_SPI_MOSI_PIN, SD_MMC_SPI_MOSI_FUNCTION},  // MOSI.
    {SD_MMC_SPI_NPCS_PIN, SD_MMC_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
  };

  // SPI options.
  spi_options_t spiOptions =
  {
    .reg          = SD_MMC_SPI_NPCS,
    .baudrate     = SD_MMC_SPI_MASTER_SPEED,  // Defined in conf_sd_mmc_spi.h.
    .bits         = SD_MMC_SPI_BITS,          // Defined in conf_sd_mmc_spi.h.
    .spck_delay   = 0,
    .trans_delay  = 0,
    .stay_act     = 1,
    .spi_mode     = 0,
    .modfdis      = 1
  };

  // Assign I/Os to SPI.
  gpio_enable_module(SD_MMC_SPI_GPIO_MAP,
                     sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));

  // Initialize as master.
  spi_initMaster(SD_MMC_SPI, &spiOptions);
  


  // Set SPI selection mode: variable_ps, pcs_decode, delay.
  spi_selectionMode(SD_MMC_SPI, 0, 0, 0);
  spi_setupChipReg(SD_MMC_SPI,&spiOptions,32000000);
  // Enable SPI module.
  spi_enable(SD_MMC_SPI);

  // Initialize SD/MMC driver with SPI clock (PBA).
  sd_mmc_spi_init(spiOptions, PBA_HZ);

}


/*! \brief Initialize PDCA (Peripheral DMA Controller A) resources for the SPI transfer and start a dummy transfer
 */
void local_pdca_init(void)
{
  // this PDCA channel is used for data reception from the SPI
  pdca_channel_options_t pdca_options_SPI_RX ={ // pdca channel options

    .addr = ram_buffer,
    // memory address. We take here the address of the string dummy_data. This string is located in the file dummy.h

    .size = 512,                              // transfer counter: here the size of the string
    .r_addr = NULL,                           // next memory address after 1st transfer complete
    .r_size = 0,                              // next transfer counter not used here
    .pid = AVR32_PDCA_CHANNEL_USED_RX,        // select peripheral ID - data are on reception from SPI1 RX line
    .transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer: 8,16,32 bits
  };

  // this channel is used to activate the clock of the SPI by sending a dummy variables
  pdca_channel_options_t pdca_options_SPI_TX ={ // pdca channel options

    .addr = (void *)&dummy_data,              // memory address.
                                              // We take here the address of the string dummy_data.
                                              // This string is located in the file dummy.h
    .size = 512,                              // transfer counter: here the size of the string
    .r_addr = NULL,                           // next memory address after 1st transfer complete
    .r_size = 0,                              // next transfer counter not used here
    .pid = AVR32_PDCA_CHANNEL_USED_TX,        // select peripheral ID - data are on reception from SPI1 RX line
    .transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer: 8,16,32 bits
  };

  // Init PDCA transmission channel
  pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_TX, &pdca_options_SPI_TX);

  // Init PDCA Reception channel
  pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_RX, &pdca_options_SPI_RX);

  //! \brief Enable pdca transfer interrupt when completed
  INTC_register_interrupt(&pdca_int_handler, AVR32_PDCA_IRQ_0, AVR32_INTC_INT1);  // pdca_channel_spi1_RX = 0

}

//end sd card stuff














int main (void)
{   //volatile avr32_tc_t *tc0 = EXAMPLE_TC0;--global declaration for isr
	//volatile avr32_tc_t *tc1 = EXAMPLE_TC1;--global declaration for isr		
	uint8_t c='u';
	pcl_switch_to_osc(PCL_OSC0, 32000000, OSC_STARTUP_4096);	
	sysclk_set_prescalers(0, 0, 0);
	board_init();
	heart_initialize();// Initialize the heart, requires GPIO and RTC libraries
	/*volatile avr32_tc_t *tc = EXAMPLE_TC;
	volatile avr32_tc_t *tc1 = EXAMPLE_TC;	*/
	Serial_init();// Initialize Serial Communications, requires Delay and USART libraries
	Serial_init2();
	//delay_init(32000000);
    //usart_write_line(EXAMPLE_USART1,"\r\n in main");
	int isd, jsd,disp1,disp2;	
	gpio_clr_gpio_pin(AVR32_PIN_PX37);
	gpio_clr_gpio_pin(AVR32_PIN_PA09);
	gpio_set_gpio_pin(AVR32_PIN_PX59);
	
usart_write_line(EXAMPLE_USART0, "reset");
	
	bool check1;
	//TC1-B2
tc_waveform_opt_t waveform_opt0 =
  {
    .channel  = EXAMPLE_TC1_CHANNEL2,        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_CLEAR,           // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_SET,           // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_NOOP,         // RC compare effect on TIOA: toggle.
    .acpa     = TC_EVT_EFFECT_NOOP,         // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      // Waveform selection: Up mode without automatic trigger on RC compare.
    .enetrg   = false,                        // External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_XC0_OUTPUT,  // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               // External event edge selection.
    .cpcdis   = false,                        // Counter disable when RC compare.
    .cpcstop  = false,                        // Counter clock stopped with RC compare.
    .burst    = TC_BURST_NOT_GATED,           // Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC5			  //Internal source clock 5, connected to fPBA / 8.
  };
  gpio_enable_module_pin(EXAMPLE_TC1_CHANNEL2_PIN,  EXAMPLE_TC1_CHANNEL2_FUNCTION);
  
  //TC0-B0
  tc_waveform_opt_t waveform_opt1 =
  {
    .channel  = EXAMPLE_TC0_CHANNEL0,        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_CLEAR,           // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_SET,           // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_NOOP,         // RC compare effect on TIOA: toggle.
    .acpa     = TC_EVT_EFFECT_NOOP,         // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      // Waveform selection: Up mode without automatic trigger on RC compare.
    .enetrg   = false,                        // External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_XC0_OUTPUT,  // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               // External event edge selection.
    .cpcdis   = false,                        // Counter disable when RC compare.
    .cpcstop  = false,                        // Counter clock stopped with RC compare.
    .burst    = TC_BURST_NOT_GATED,           // Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC5			  //Internal source clock 5, connected to fPBA / 8.
  };
  gpio_enable_module_pin(EXAMPLE_TC0_CHANNEL0_PIN_B,  EXAMPLE_TC0_CHANNEL0_FUNCTION_B);
  //TC0-A1,TC0-B1  
    tc_waveform_opt_t waveform_opt23 =
  {
    .channel  = EXAMPLE_TC0_CHANNEL1,        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_CLEAR,           // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_SET,           // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOA.
    .acpc     =  TC_EVT_EFFECT_CLEAR,         // RC compare effect on TIOA: toggle.
    .acpa     =  TC_EVT_EFFECT_SET,         // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      // Waveform selection: Up mode without automatic trigger on RC compare.
    .enetrg   = false,                        // External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_XC0_OUTPUT,  // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               // External event edge selection.
    .cpcdis   = false,                        // Counter disable when RC compare.
    .cpcstop  = false,                        // Counter clock stopped with RC compare.
    .burst    = TC_BURST_NOT_GATED,           // Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC5			  //Internal source clock 5, connected to fPBA / 8.
  };
  gpio_enable_module_pin(EXAMPLE_TC0_CHANNEL1_PIN_A,  EXAMPLE_TC0_CHANNEL1_FUNCTION_A);
  gpio_enable_module_pin(EXAMPLE_TC0_CHANNEL1_PIN_B,  EXAMPLE_TC0_CHANNEL1_FUNCTION_B);
  
  //TC0-B2
    tc_waveform_opt_t waveform_opt4 =
  {
    .channel  = EXAMPLE_TC0_CHANNEL2,        // Channel selection.

    .bswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOB.
    .beevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOB.
    .bcpc     = TC_EVT_EFFECT_CLEAR,           // RC compare effect on TIOB.
    .bcpb     = TC_EVT_EFFECT_SET,           // RB compare effect on TIOB.

    .aswtrg   = TC_EVT_EFFECT_NOOP,           // Software trigger effect on TIOA.
    .aeevt    = TC_EVT_EFFECT_NOOP,           // External event effect on TIOA.
    .acpc     = TC_EVT_EFFECT_NOOP,         // RC compare effect on TIOA: toggle.
    .acpa     = TC_EVT_EFFECT_NOOP,         // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

    .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,      // Waveform selection: Up mode without automatic trigger on RC compare.
    .enetrg   = false,                        // External event trigger enable.
    .eevt     = TC_EXT_EVENT_SEL_XC0_OUTPUT,  // External event selection.
    .eevtedg  = TC_SEL_NO_EDGE,               // External event edge selection.
    .cpcdis   = false,                        // Counter disable when RC compare.
    .cpcstop  = false,                        // Counter clock stopped with RC compare.
    .burst    = TC_BURST_NOT_GATED,           // Burst signal selection.
    .clki     = TC_CLOCK_RISING_EDGE,         // Clock inversion.
    .tcclks   = TC_CLOCK_SOURCE_TC5			  //Internal source clock 5, connected to fPBA / 8.
  };
  gpio_enable_module_pin(EXAMPLE_TC0_CHANNEL2_PIN_B,  EXAMPLE_TC0_CHANNEL2_FUNCTION_B);
  
  tc_init_waveform(tc1, &waveform_opt0);  // Initialize the timer/counter waveform.
  tc_init_waveform(tc0, &waveform_opt1);
  tc_init_waveform(tc0, &waveform_opt23);
  tc_init_waveform(tc0, &waveform_opt4);
	
  tc_write_rb(tc1, EXAMPLE_TC1_CHANNEL2, 5125);     // Set RA value.
  tc_write_rc(tc1, EXAMPLE_TC1_CHANNEL2, 5500);     // Set RC value.
   
  tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL0, 5125);    // Set RA value.
  tc_write_rc(tc0, EXAMPLE_TC0_CHANNEL0, 5500);     // Set RC value.
  
  tc_write_ra(tc0, EXAMPLE_TC0_CHANNEL1, 5125);     // Set RA value.
  tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL1, 5125);     // Set RA value.
  tc_write_rc(tc0, EXAMPLE_TC0_CHANNEL1, 5500);     // Set RC value.
  
  tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL2, 5125);     // Set RA value.
  tc_write_rc(tc0, EXAMPLE_TC0_CHANNEL2, 5500);     // Set RC value.

  // Start the timer/counter.
  tc_start(tc1, EXAMPLE_TC1_CHANNEL2);
  tc_start(tc0, EXAMPLE_TC0_CHANNEL0);
  tc_start(tc0, EXAMPLE_TC0_CHANNEL1);
  tc_start(tc0, EXAMPLE_TC0_CHANNEL2);
  
  
  
  //interrupt stuff-added sunday morning
  
  	Disable_global_interrupt();
	//    usart_write_line(EXAMPLE_USART1,"\r\nInit SD/MMC Driver");
//  usart_write_line(EXAMPLE_USART1,"\r\nInsert SD/MMC...");

	// Initialize interrupt vectors.
	INTC_init_interrupts();

	/*
	 * Register the USART interrupt handler to the interrupt controller.
	 * usart_int_handler is the interrupt handler to register.
	 * EXAMPLE_USART_IRQ is the IRQ of the interrupt handler to register.
	 * AVR32_INTC_INT0 is the interrupt priority level to assign to the
	 * group of this IRQ.
	 */
	INTC_register_interrupt(&usart_int_handler, EXAMPLE_USART0_IRQ,
		AVR32_INTC_INT0);

	// Enable USART Rx interrupt.
	EXAMPLE_USART0->ier = AVR32_USART_IER_RXRDY_MASK;

  // Initialize SD/MMC driver resources: GPIO, SPI and SD/MMC.
  sd_mmc_resources_init();
     gpio_clr_gpio_pin(AVR32_PIN_PB03);
 // usart_write_line(EXAMPLE_USART1,"test here");
  // Wait for a card to be inserted
  while (!sd_mmc_spi_mem_check());
 // usart_write_line(EXAMPLE_USART1,"\r\nCard detected!");

  // Read Card capacity
  sd_mmc_spi_get_capacity();
  //usart_write_line(EXAMPLE_USART1,"Capacity = ");
  capacity=(capacity>>20)+48;
//  usart_putchar(EXAMPLE_USART1,capacity); //>> 20);//changed from ulong
 // usart_write_line(EXAMPLE_USART1," MBytes");


	// Enable all interrupts.
	Enable_global_interrupt();
	
  // Initialize PDCA controller before starting a transfer
  local_pdca_init();
	
	
	 // Read the first sectors number 1, 2, 3 of the card
  for(jsd = 1; jsd <= 3; jsd++)
  {
    // Configure the PDCA channel: the adddress of memory ram_buffer to receive the data at sector address j
    pdca_load_channel( AVR32_PDCA_CHANNEL_SPI_RX,
                     &ram_buffer,
                     512);

    pdca_load_channel( AVR32_PDCA_CHANNEL_SPI_TX,
                     (void *)&dummy_data,
                     512); //send dummy to activate the clock

    end_of_transfer = false;
    // open sector number j
    if(sd_mmc_spi_read_open_PDCA (jsd))
    {
    //  usart_write_line(EXAMPLE_USART1,"\r\nFirst 512 Bytes of Transfer number ");
	  disp1=jsd+48;
    //  usart_putchar(EXAMPLE_USART1,disp1);
   //   usart_write_line(EXAMPLE_USART1," :\r\n");

      spi_write(SD_MMC_SPI,0xFF); // Write a first dummy data to synchronise transfer
      pdca_enable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_RX);
      pdca_channelrx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_RX); // get the correct PDCA channel pointer
      pdca_channeltx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_TX); // get the correct PDCA channel pointer
      pdca_channelrx->cr = AVR32_PDCA_TEN_MASK; // Enable RX PDCA transfer first
      pdca_channeltx->cr = AVR32_PDCA_TEN_MASK; // and TX PDCA transfer

      while(!end_of_transfer);

      // Display the first 2O bytes of the ram_buffer content
      for( isd = 0; isd < 20; isd++)
      {
	   disp2=((U8)(*(ram_buffer + isd)))+48;
       //usart_putchar( EXAMPLE_USART1,disp2);
      }
    }
    else
    {
   //   usart_write_line(EXAMPLE_USART1,"\r\n! Unable to open memory \r\n");
    }
  }
 // usart_write_line(EXAMPLE_USART1,"\r\nEnd of the example.\r\n");
  
    isd=nav_drive_nb();
	isd=isd+48;
	//usart_putchar(EXAMPLE_USART1,isd);
	 nav_reset();
          // Select the desired drive.
        check1= nav_drive_set(0);
		 if(check1==true)
		 {
		// 	usart_write_line(EXAMPLE_USART1,"set drive\r\n");
		 }
		 else
		 {		 //	usart_write_line(EXAMPLE_USART1,"no set drive\r\n");
		 }
		 
		
			check1=fat_mount();
		 if(check1==true)
		 {
		// 	usart_write_line(EXAMPLE_USART1,"mounted\r\n");
		 }
		 else
		 {	disp2=fs_g_status+48;	 	
		//	usart_putchar(EXAMPLE_USART1,disp2);
		 }		 	
  
  if(!nav_setcwd("./test_dir/",false,true))
{disp2=fs_g_status+48;	 	
	//		usart_putchar(EXAMPLE_USART1,disp2);
//	usart_write_line(EXAMPLE_USART1,"Error in creating directory\r\n");
}
else{//usart_write_line(EXAMPLE_USART1,"Directory open\r\n");
//	usart_write_line(EXAMPLE_USART1,"Creating file\r\n");
	if(nav_file_create("a5.txt")){
	//	usart_write_line(EXAMPLE_USART1,"file created");
	}else{//usart_write_line(EXAMPLE_USART1,"Error in file creating\r\n");
	}
	}	
 
  
  
  
  
  int logval;
	int checkval;
	while(true)//The main while loop, the state0 of the state machine.
	{   usart_write_line(EXAMPLE_USART0, "in main");
		if(errfound == 1)
		{
		//usart_write_line(EXAMPLE_USART1,"In the if in main");
		errfound = 0;
		}

		//usart_write_line(EXAMPLE_USART0,"\r\nnew stream");
	/*	do 
		{		c=usart_getchar_gps(EXAMPLE_USART1);
				} while (c!='$');
		do 
		{
		usart_putchar(EXAMPLE_USART0,c);
		c=usart_getchar_gps(EXAMPLE_USART1);} while (c!='\n');
		*/	
// old while ends
//new begins
/*
		c=usart_getchar(EXAMPLE_USART0);
		usart_putchar(EXAMPLE_USART0,c);
		if(startbit==1&&endbit==0)
		{   //usart_putchar(EXAMPLE_USART0,c);
			xbeechar[counter] = c;
			counter++;
		}
		if(c=='x')
		{
			startbit = 1;
			endbit = 0;
		}
		else if(c=='y')
		{
			counter = 0;
			startbit =0;
			endbit = 1;
		//PWM3			
		tc_write_rb(tc1, EXAMPLE_TC1_CHANNEL2,((xbeechar[0]-48)*1000+(xbeechar[1]-48)*100+(xbeechar[2]-48)*10+(xbeechar[3]-48)));     // Set RA value.
		//PWM2
		tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL0,((xbeechar[4]-48)*1000+(xbeechar[5]-48)*100+(xbeechar[6]-48)*10+(xbeechar[7]-48)));     // Set RA value.
		//PWM0
		tc_write_ra(tc0, EXAMPLE_TC0_CHANNEL1,((xbeechar[8]-48)*1000+(xbeechar[9]-48)*100+(xbeechar[10]-48)*10+(xbeechar[11]-48)));     // Set RA value.
        //PWM4
		tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL1,((xbeechar[12]-48)*1000+(xbeechar[13]-48)*100+(xbeechar[14]-48)*10+(xbeechar[15]-48)));     // Set RA value.
		//PWM1
		tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL2,((xbeechar[16]-48)*1000+(xbeechar[17]-48)*100+(xbeechar[18]-48)*10+(xbeechar[19]-48)));     // Set RA value.
		}*/
		//click();
		//Serial_PC();
		
		//interrupt stuff-added sunday morning
				/*
		 * If there is a chance that any PB write operations are
		 * incomplete, the CPU should perform a read operation from any
		 * register on the PB bus before executing the sleep
		 * instruction.
		 */
		AVR32_INTC.ipr[0];  // Dummy read

		// Go to FROZEN sleep mode.
	//	SLEEP(AVR32_PM_SMODE_FROZEN);
		/*
		 * When the device wakes up due to an interrupt, once the
		 * interrupt has been serviced, go back into FROZEN sleep mode.
		 */
		//end interrupt stuff
		//i2c2();
		checkval = tc_read_ra(tc0,EXAMPLE_TC0_CHANNEL1);
		if(checkval < 5025 || checkval >5225)
		{
			tc_write_ra(tc0,EXAMPLE_TC0_CHANNEL1,buf1);
		}
		else
		{
			buf1 = checkval;	
		}	
		checkval = tc_read_rb(tc0,EXAMPLE_TC0_CHANNEL1);
		if(checkval < 5025 || checkval >5225)
		{
			tc_write_rb(tc0,EXAMPLE_TC0_CHANNEL1,buf2);
		}
		else
		{
			buf2 = checkval;	
		}	
		checkval = tc_read_rb(tc0,EXAMPLE_TC0_CHANNEL0);
		if(checkval < 5025 || checkval >5225)
		{
			tc_write_rb(tc0,EXAMPLE_TC0_CHANNEL0,buf3);
		}
		else
		{
			buf3 = checkval;	
		}	
		checkval = tc_read_rb(tc0,EXAMPLE_TC0_CHANNEL2);
		if(checkval < 5025 || checkval >5225)
		{
			tc_write_rb(tc0,EXAMPLE_TC0_CHANNEL2,buf4);
		}
		else
		{
			buf4 = checkval;	
		}	
		checkval = tc_read_rb(tc1,EXAMPLE_TC1_CHANNEL2);
		if(checkval < 5025 || checkval >5225)
		{
			tc_write_rb(tc1,EXAMPLE_TC1_CHANNEL2,buf5);
		}
		else
		{
			buf5 = checkval;	
		}	
		usart_write_line(EXAMPLE_USART0,"just b4 i2c\r\n");
		i2c();
		delay_ms(100);
		i2c2();
		delay_ms(100);
		usart_write_line(EXAMPLE_USART0,"new stream\r\n");
		i2c3();
		delay_ms(100);
		
		if(file_open(FOPEN_MODE_APPEND)){
		//	usart_write_line(EXAMPLE_USART1,"File opened\r\n"); 
			//file_flush();
			for(logval=0;logval<19;logval++)
			{
			file_hex(sd_buffer[logval]);
			file_putc('\r');
			file_putc('\n');	
			}
			//file_set_eof();
			file_close();
		}			
			
	//		usart_write_line(EXAMPLE_USART1,"End of writing\r\n");
		
		
		if(i!=ibuf && i == 1)	
		{usart_write_line(EXAMPLE_USART0, "\r\nsssssss");}
		//i=1;}
		else if(i!= ibuf && i==0)
		{usart_write_line(EXAMPLE_USART0, "\r\nxxxxxxx");}
		ibuf = i;
		//i=0;}	
		
		
	}
 }		
		
void click()
	{   //delay_init(32000000);
		//turn on pulse
		gpio_set_gpio_pin(AVR32_PIN_PX24);
		delay_ms(500);
		gpio_clr_gpio_pin(AVR32_PIN_PX24);
		
		//wait
		delay_ms(3500);
		
		//turn on shutter
		gpio_set_gpio_pin(AVR32_PIN_PX52);
		delay_ms(500);
		gpio_clr_gpio_pin(AVR32_PIN_PX52);
		
		
		delay_ms(1000);
		gpio_set_gpio_pin(AVR32_PIN_PX24);
		delay_ms(500);
		gpio_clr_gpio_pin(AVR32_PIN_PX24);/*
		delay_ms(3000);
		
		//turn off pulse
		gpio_set_gpio_pin(AVR32_PIN_PX49);
		delay_ms(200);
		gpio_clr_gpio_pin(AVR32_PIN_PX49);*/
	}

/* Initialization of the "heart"
 * Depends on the RTC module
 * If enabled correctly, pin PA0 should turn on
 * Otherwise pin PA7 turns on
 */
void heart_initialize()
{
	int initialized_correctly;
	initialized_correctly = 0;
	//tc_init_capture(AVR32_TC0,&AVR32_TC)
	initialized_correctly = rtc_init(AVR32_RTC_ADDRESS,RTC_OSC_RC,RTC_PSEL_RC_1_76HZ);
	rtc_enable(&AVR32_RTC);
	if(initialized_correctly==1)
	{
		//gpio_clr_gpio_pin(AVR32_PIN_PB00);
	}
	else
	{	
		//gpio_clr_gpio_pin(AVR32_PIN_PB07);
	}
}
/* Code for one heartbeat
 * Toggles pin PA1 depending on RTC value
 * If connected to LED1, will cause it to blink
 * Remember to put this inside the outermost while loop
 */
void heartBeat()
{	
	//rtc_set_value(&AVR32_RTC,(unsigned long)2);//rtc_init(AVR32_RTC_ADDRESS,RTC_OSC_32KHZ,RTC_PSEL_32KHZ_1HZ);
	if(rtc_get_value(&AVR32_RTC)%2==1) 
	{
		gpio_clr_gpio_pin(AVR32_PIN_PB01);
	}		
	else
	{
		gpio_set_gpio_pin(AVR32_PIN_PB01);
	}		
}

/* Code for UART connection via RS 232
 * Remember to connect STK600.PORTA.PA3 to STK600.RS232 SPARE.RXD
 * and STK600.PORTA.PA4 to STK600.RS232 SPARE.TXD
 * Also include these libraries : DELAY, USART
 */

void Serial_init()
{
	
	delay_init(32000000);
	
	static const gpio_map_t USART_GPIO_MAP = { 
      { 
         EXAMPLE_USART0_RX_PIN, EXAMPLE_USART0_RX_FUNCTION 
      }, 
      { 
         EXAMPLE_USART0_TX_PIN, EXAMPLE_USART0_TX_FUNCTION 
      } 
   }; 

	static const usart_options_t usart_opts = { 
      .baudrate = 9600, 
      .charlength = 8, 
      .paritytype = USART_NO_PARITY, 
      .stopbits = USART_1_STOPBIT, 
      .channelmode = USART_NORMAL_CHMODE 
   }; 
   
   gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0])); 
   usart_init_rs232(EXAMPLE_USART0, &usart_opts, 32000000);
   
}

/* Code for UART connection via RS 232
 * Remember to connect STK600.PN2 to STK600.RS232 SPARE.RXD
 * and STK600.PN1 to STK600.RS232 SPARE.TXD
 * Also include these libraries : DELAY, USART
 */
void Serial_init2()
{
	delay_init(32000000);
	
	static const gpio_map_t USART_GPIO_MAP = { 
      { 
         EXAMPLE_USART1_RX_PIN, EXAMPLE_USART1_RX_FUNCTION 
      }, 
      { 
         EXAMPLE_USART1_TX_PIN, EXAMPLE_USART1_TX_FUNCTION 
      } 
   }; 

	static const usart_options_t usart_opts = { 
      .baudrate =4800, //changed from 38400
      .charlength = 8, 
      .paritytype = USART_NO_PARITY, 
      .stopbits = USART_1_STOPBIT, 
      .channelmode = USART_NORMAL_CHMODE 
   }; 
   
   gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0])); 
   usart_init_rs232(EXAMPLE_USART1, &usart_opts, 32000000);
}


void Serial_PC()
{
	// Use the following line to print a whole line to UART/Teraterm
//	usart_write_line(EXAMPLE_USART1,"\r\n hello world");

	// Use the following format to wait for input from user via keyboard
//	while (usart_get_echo_line(EXAMPLE_USART1) != USART_SUCCESS);
	
	// Use the following line to print a single character to UART/Teraterm
	//usart_putchar(EXAMPLE_USART, 'p'); 

	
    delay_ms(500); 
	
}
const 	uint8_t write_data13[]={0xEE,0xF4,0x34};
const   uint8_t write_data4[]={0xEE,0xF6};
const 	uint8_t write_data2[]={0x41};
const 	uint8_t write_data[]={0xEE,0xF4,0x2e};
const   uint8_t write_data14[]={0xEE,0xf6};	
const   uint8_t write_data_ac1[]={0xEE,0xbe};		
	
//this is for bmp085
//twim0 or TWIM0
void i2c (void)
{   
	int i;
	twi_package_t packet, packet_received;

	for (i = 0; i < 2; i++) {
		read_data[i]=0xEF;
		}			
		
	status_code_t status;
	//usart_write_line(EXAMPLE_USART0,"\r\n In i2c");
	/*irq_initialize_vectors();
	cpu_irq_enable();*/
	
	// Initialize the TWIM Module
	twim_init ();//------------------Intitialization important
	// TWI chip address to communicate with	
	packet.chip = WRITE_ADDRESS;//TARGET_ADDRESS;				//0x77
	// TWI address/commands to issue to the other chip (node)
	packet.addr = VIRTUALMEM_ADDR_UT;//VIRTUALMEM_ADDR;			//0xf4
	// Length of the TWI data address segment (1-3 bytes)
	packet.addr_length = TARGET_ADDR_LGT_UT;//TARGET_ADDR_LGT	//1
	// Where to find the data to be written
	packet.buffer = (void *) write_data;						//0xee 0xf4, 0x34
	// How many bytes do we want to write
	packet.length = 3;
	// Write data to TARGET
	status = twi_master_write (TWIM0, &packet);
	// Check status of transfer
	
	//status=twim_write(TWIM, write_data, 0x77, 3, 0);
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART0,"\r\n WRITE TEST1:\tPASS\r\n");
	} else {
	//usart_write_line(EXAMPLE_USART0,"\r\n WRITE TEST1:\tFAIL\r\n");
	}
	

	delay_ms(30);
	
	packet.buffer = (void *) write_data4;			//0xee, 0xf6
	// How many bytes do we want to write
	packet.length = 2;
	// Write data to TARGET
	status = twi_master_write (TWIM0, &packet);
	//status = twim_write (TWIM, write_data4, 0x77, 2, 0);
	// Check status of transfer
	
	if (status == STATUS_OK) {
	// display test result to user
	//usart_write_line(EXAMPLE_USART1,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
	// display test result to user
	//usart_write_line(EXAMPLE_USART1,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	
	// Read data from TARGET
	status = twim_read(TWIM0,read_data,2,0x77,0);
	// Check Status
	
	if (status == STATUS_OK) {
	// Display test result to user
	//usart_write_line(EXAMPLE_USART0,"\r\n up bmp \tPASS\r\n");
	/*	for (i = 0; i < 2; i++) {         
		usart_putchar(EXAMPLE_USART0,read_data[i]);
		//usart_write_line(EXAMPLE_USART0,"\r\n");
	*/
		usart_write_line(EXAMPLE_USART0,"\r\nUT1");
	//	usart_write_line(EXAMPLE_USART1,"\r\nUT1");
		//for (i = 0; i < 2; i++) {         
		//usart_putchar(EXAMPLE_USART0,read_data[i]);
	print_hex(EXAMPLE_USART0,read_data[0]);
	usart_write_line(EXAMPLE_USART0,"\r\nUT2");
	
	print_hex(EXAMPLE_USART0,read_data[1]);
//		print_hex(EXAMPLE_USART1,read_data[0]);
//	usart_write_line(EXAMPLE_USART1,"\r\nUT2");
	//print_hex(EXAMPLE_USART1,read_data[1]);
			sd_buffer[0]=read_data[0];
		sd_buffer[1]=read_data[1];		
	//usart_write_line(EXAMPLE_USART0,"\r\n");	
	} else {
	// Display test result to user
	//usart_write_line(EXAMPLE_USART1,"\r\n READ TEST:\tFAIL\r\n");
	}	
	
	
	
	//******************
		packet.buffer = (void *) write_data13;						//0xee 0xf4, 0x2e
	// How many bytes do we want to write
	packet.length = 3;
	// Write data to TARGET
	status = twi_master_write (TWIM0, &packet);
	// Check status of transfer
	
	//status=twim_write(TWIM, write_data, 0x77, 3, 0);
	if (status == STATUS_OK) {
//	usart_write_line(EXAMPLE_USART1,"\r\n WRITE UT:\tPASS\r\n");
	} else {
	//usart_write_line(EXAMPLE_USART1,"\r\n WRITE UT:\tFAIL\r\n");
	}
	

	delay_ms(100);
	
	packet.buffer = (void *) write_data14;			//0xee, 0xf6
	// How many bytes do we want to write
	packet.length = 2;
	// Write data to TARGET
	status = twi_master_write (TWIM0, &packet);
	//status = twim_write (TWIM, write_data4, 0x77, 2, 0);
	// Check status of transfer
	
	if (status == STATUS_OK) {
	// display test result to user
	//usart_write_line(EXAMPLE_USART1,"\r\n WRITE UT:\tPASS\r\n");
	} else {
	// display test result to user
	//usart_write_line(EXAMPLE_USART1,"\r\n WRITE UT:\tFAIL\r\n");
	}
	
	// Read data from TARGET
	status = twim_read(TWIM0,read_data,22,0x77,0);
	// Check Status
	
	if (status == STATUS_OK) {
	// Display test result to user
	usart_write_line(EXAMPLE_USART0,"\r\nUP1");
	//usart_write_line(EXAMPLE_USART1,"\r\nUP1");
		//for (i = 0; i < 2; i++) {         
		//usart_putchar(EXAMPLE_USART0,read_data[i]);
	print_hex(EXAMPLE_USART0,read_data[0]);
	//print_hex(EXAMPLE_USART1,read_data[0]);
	usart_write_line(EXAMPLE_USART0,"\r\nUP2");
	print_hex(EXAMPLE_USART0,read_data[1]);
	//usart_write_line(EXAMPLE_USART1,"\r\nUP2");
	//print_hex(EXAMPLE_USART1,read_data[1]);
	sd_buffer[2]=read_data[0];
	sd_buffer[3]=read_data[1];
	//	usart_write_line(EXAMPLE_USART0,"\r\n");
	//	}	
	//usart_write_line(EXAMPLE_USART0,"\r\n");	
	} else {
	// Display test result to user
	//usart_write_line(EXAMPLE_USART1,"\r\n READ TEST:\tFAIL\r\n");
	}	
	
	//*************************
	/*delay_ms(30);
	//get ac1
	packet.buffer = (void *) write_data_ac1;			//0xee, 0xf6
	// How many bytes do we want to write
	packet.length = 2;
	// Write data to TARGET
	status = twi_master_write (TWIM0, &packet);
	//status = twim_write (TWIM, write_data4, 0x77, 2, 0);
	// Check status of transfer
	
	if (status == STATUS_OK) {
	// display test result to user
	//usart_write_line(EXAMPLE_USART0,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
	// display test result to user
	usart_write_line(EXAMPLE_USART0,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	
	// Read data from TARGET
	status = twim_read(TWIM0,read_data,2,0x77,0);
	// Check Status
	
	if (status == STATUS_OK) {
	// Display test result to user
	usart_write_line(EXAMPLE_USART0,"\r\n UT bmp \tPASS\r\n");
		for (i = 0; i < 2; i++) {         
		//usart_putchar(EXAMPLE_USART0,read_data[i]);
		print_hex(EXAMPLE_USART0,read_data[i]);
		usart_write_line(EXAMPLE_USART0,"\r\n");
		}	
	//usart_write_line(EXAMPLE_USART0,"\r\n");	
	} else {
	// Display test result to user
	usart_write_line(EXAMPLE_USART0,"\r\n READ TEST:\tFAIL\r\n");
	}	
	*/
	
	
return ;
}

//twim0 or TWIM0
//this is for hmc6352
void i2c2 (void)
{   
	int i;
	twi_package_t packet, packet_received;

	for (i = 0; i < 2; i++) {
		read_data2[i]=0xEF;
		}			
		
	status_code_t status;
	//usart_write_line(EXAMPLE_USART0,"\r\n In i2c");
	
	/*irq_initialize_vectors();
	cpu_irq_enable();*/
	
	// Initialize the TWIM Module
	twim_init2 ();//------------------Intitialization important
	// TWI chip address to communicate with
	packet.chip = 0x21;//TARGET_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet.addr = 0x21;//VIRTUALMEM_ADDR;
	// Length of the TWI data address segment (1-3 bytes)
	packet.addr_length = TARGET_ADDR_LGT_UT;//TARGET_ADDR_LGT
	// Where to find the data to be written
	packet.buffer = (void *) write_data2;
	// How many bytes do we want to write
	packet.length = 2;
	// Write data to TARGET
	//status = twi_master_write (TWIM, &packet);
	status = twim_write (TWIM0, write_data2,1,0x21,0);
	// Check status of transfer
	if (status == STATUS_OK) {
	//	usart_write_line(EXAMPLE_USART1,"\r\n WRITE TEST1:\tPASS\r\n");
	} else {
//		usart_write_line(EXAMPLE_USART1,"\r\n WRITE TEST1:\tFAIL\r\n");
	}

	delay_ms(7);

	// Read data from TARGET
	status = twim_read(TWIM0,read_data,2,0x21,0);

	// Check Status
	if (status == STATUS_OK) {/*
	usart_write_line(EXAMPLE_USART0,"\r\n READ TEST hmc:\tPASS\r\n");
	print_hex(EXAMPLE_USART0,read_data[0]);
	//print_hex(EXAMPLE_USART1,read_data[0]);
	usart_write_line(EXAMPLE_USART0,"\r\n");		
	print_hex(EXAMPLE_USART0,read_data[1]);
	//print_hex(EXAMPLE_USART1,read_data[1]);
	usart_write_line(EXAMPLE_USART0,"\r\n");	
	} else {
	usart_write_line(EXAMPLE_USART0,"\r\n READ TEST:\tFAIL\r\n");*/
	usart_write_line(EXAMPLE_USART0,"\r\nHMC1");
	//usart_write_line(EXAMPLE_USART1,"\r\nHMC1");
		//for (i = 0; i < 2; i++) {         
		//usart_putchar(EXAMPLE_USART0,read_data[i]);
	
	print_hex(EXAMPLE_USART0,read_data[0]);
	//print_hex(EXAMPLE_USART1,read_data[0]);
	usart_write_line(EXAMPLE_USART0,"\r\nHMC2");
	//usart_write_line(EXAMPLE_USART1,"\r\nHMC2");
			
	print_hex(EXAMPLE_USART0,read_data[1]);
	//print_hex(EXAMPLE_USART1,read_data[1]);
			sd_buffer[4]=read_data[0];
	sd_buffer[5]=read_data[1];
	usart_write_line(EXAMPLE_USART0,"\r\n");
	//usart_write_line(EXAMPLE_USART1,"\r\n");
	}
return ;
}

//this is for mpu6050

const	uint8_t write_data5[]={0x6b, 0x00};
const   uint8_t write_data6[]={0x6b};
const   uint8_t write_data7[]={0x43};	
const   uint8_t write_data8[]={0x45};	
const   uint8_t write_data9[]={0x47};	
const   uint8_t write_data10[]={0x3B};	
const   uint8_t write_data11[]={0x3D};	
const   uint8_t write_data12[]={0x3F};		
	
	
////this is for mpu6050
//twim1 or TWIM
void i2c3 (void)
{   uint8_t c='\n';
	int i;
	twi_package_t packet, packet_received;

	for (i = 0; i < 2; i++) {
		read_data_s[i]=0xEF;
		}			
		
	status_code_t status;
	//usart_write_line(EXAMPLE_USART,"\r\n In i2c3");
	/*irq_initialize_vectors();
	cpu_irq_enable();*/
	
	// Initialize the TWIM Module
	twim_init3 ();//------------------Intitialization important

	// TWI chip address to communicate with
	packet.chip = 0x68;//TARGET_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet.addr = 0x75;//VIRTUALMEM_ADDR;
	// Length of the TWI data address segment (1-3 bytes)
	packet.addr_length = 1;//TARGET_ADDR_LGT_UT;//TARGET_ADDR_LGT
	// Where to find the data to be written
	/*
	packet.buffer = (void *) write_data5;
	// How many bytes do we want to write
	packet.length = 1;
	// Write data to TARGET
	status = twi_master_write (TWIM, &packet);	*/
	status = twim_write (TWIM, write_data5,2,0x68,0);
	
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART1,"\r\n WRITE TEST2:\tPASS\r\n");
	} else {
	//usart_write_line(EXAMPLE_USART1,"\r\n WRITE TEST2:\tFAIL\r\n");
	}
	

	delay_ms(10);
	status = twim_write (TWIM, write_data6,1,0x68,0);
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST3:\tPASS\r\n");
	} else {
	//usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST3:\tFAIL\r\n");
	}
	delay_ms(10);
	// Read data from TARGET
	status = twim_read(TWIM,read_data_s,1,0x68,0);

	// Check Status
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART,"\r\n READ TEST:\tPASS\r\n");
	//print_hex(EXAMPLE_USART,read_data[0]);
	//usart_write_line(EXAMPLE_USART,"\r\n");		
	//print_hex(EXAMPLE_USART,read_data[1]);
	//usart_write_line(EXAMPLE_USART,"\r\n");	
	} else {
	//usart_write_line(EXAMPLE_USART,"\r\n READ TEST:\tFAIL\r\n");
	}
	

//get x gyro

	delay_ms(10);
	status = twim_write (TWIM, write_data7,1,0x68,0);
	if (status == STATUS_OK) {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	delay_ms(10);
	// Read data from TARGET
	status = twim_read(TWIM,read_data_s,2,0x68,0);

	// Check Status
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ x gyro:\tPASS\r\n");
//	print_hex(EXAMPLE_USART1,read_data_s[0]);
//	usart_write_line(EXAMPLE_USART1,"\r\nxgyro1");
	print_hex(EXAMPLE_USART0,read_data_s[0]);
	sd_buffer[6]=read_data[0];
	sd_buffer[7]=read_data[1];		
	//usart_putchar(EXAMPLE_USART1,c);
	//usart_write_line(EXAMPLE_USART1,"\r\n");
	//read_data[1]=read_data[1]<<2|read_data[0];
//	print_hex(EXAMPLE_USART1,read_data_s[1]);
	usart_write_line(EXAMPLE_USART0,"\n");
	//usart_write_line(EXAMPLE_USART0,"\r\nxgyro2");		
	print_hex(EXAMPLE_USART0,read_data_s[1]);
	//usart_putchar(EXAMPLE_USART1,c);
	usart_write_line(EXAMPLE_USART0,"\n");	
	} else {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ TEST2:\tFAIL\r\n");
	}
	
	
	//get y gyro
	
	
	
	delay_ms(10);
	status = twim_write (TWIM, write_data8,1,0x68,0);
	if (status == STATUS_OK) {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
	//usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	delay_ms(10);
	// Read data from TARGET
	status = twim_read(TWIM,read_data_s,2,0x68,0);

	// Check Status
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ y gyro:\tPASS\r\n");
//	print_hex(EXAMPLE_USART1,read_data_s[0]);
//	usart_write_line(EXAMPLE_USART1,"\r\nygyro1");
	print_hex(EXAMPLE_USART0,read_data_s[0]);
			sd_buffer[8]=read_data_s[0];
			sd_buffer[9]=read_data_s[1];
	//usart_putchar(EXAMPLE_USART1,c);
	//read_data[1]=(read_data[1]<<2)|read_data[0];
	//usart_write_line(EXAMPLE_USART1,"\r\n");		
//	print_hex(EXAMPLE_USART1,read_data_s[1]);
	usart_write_line(EXAMPLE_USART0,"\n");
	//usart_write_line(EXAMPLE_USART0,"\r\nygyro2");
	print_hex(EXAMPLE_USART0,read_data_s[1]);
	//usart_putchar(EXAMPLE_USART1,c);
	usart_write_line(EXAMPLE_USART0,"\n");	
	} else {
	//usart_write_line(EXAMPLE_USART0,"\n READ TEST2:\tFAIL\r\n");
	}
	
//get z gyro	

	delay_ms(10);
	status = twim_write (TWIM, write_data9,1,0x68,0);
	if (status == STATUS_OK) {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	delay_ms(10);
	// Read data from TARGET
	status = twim_read(TWIM,read_data_s,2,0x68,0);

	// Check Status
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ z gyro:\tPASS\r\n");
	//print_hex(EXAMPLE_USART1,read_data_s[0]);
//	usart_write_line(EXAMPLE_USART1,"\r\nzgyro1");
	print_hex(EXAMPLE_USART0,read_data_s[0]);
	sd_buffer[10]=read_data_s[0];
	sd_buffer[11]=read_data_s[1];
	//usart_putchar(EXAMPLE_USART1,c);
	//usart_write_line(EXAMPLE_USART1,"\r\n");		
	//print_hex(EXAMPLE_USART1,read_data_s[1]);
	usart_write_line(EXAMPLE_USART0,"\n");
	//usart_write_line(EXAMPLE_USART0,"\r\nzgyro2");
	print_hex(EXAMPLE_USART0,read_data[1]);
	//usart_putchar(EXAMPLE_USART1,c);0
	usart_write_line(EXAMPLE_USART0,"\n");	
	} else {
	usart_write_line(EXAMPLE_USART0,"\r\n READ TEST2:\tFAIL\r\n");
	}
	
//get x acceleration	
	delay_ms(50);
	status = twim_write (TWIM, write_data10,1,0x68,0);
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	delay_ms(10);
	// Read data from TARGET
	status = twim_read(TWIM,read_data_s,2,0x68,0);

	// Check Status
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ x accel:\tPASS\r\n");
//	print_hex(EXAMPLE_USART1,read_data_s[0]);
//	usart_write_line(EXAMPLE_USART1,"\r\nxacc1");
	print_hex(EXAMPLE_USART0,read_data_s[0]);
	//usart_putchar(EXAMPLE_USART1,c);
	//usart_write_line(EXAMPLE_USART1,"\r\n");		
	//print_hex(EXAMPLE_USART1,read_data_s[1]);
		sd_buffer[12]=read_data_s[0];
	sd_buffer[13]=read_data_s[1];
	usart_write_line(EXAMPLE_USART0,"\n");
	//usart_write_line(EXAMPLE_USART0,"\r\nxacc2");
	print_hex(EXAMPLE_USART0,read_data_s[1]);
	//usart_putchar(EXAMPLE_USART1,c);
	usart_write_line(EXAMPLE_USART0,"\n");	
	} else {
	usart_write_line(EXAMPLE_USART0,"\r\n READ TEST2:\tFAIL\r\n");
	}
	
	
	
	//get y acceleration	
	delay_ms(10);
	status = twim_write (TWIM, write_data11,1,0x68,0);
	if (status == STATUS_OK) {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	delay_ms(10);
	// Read data from TARGET
	status = twim_read(TWIM,read_data_s,2,0x68,0);

	// Check Status
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ y accel:\tPASS\r\n");
//	print_hex(EXAMPLE_USART1,read_data_s[0]);
	//usart_write_line(EXAMPLE_USART1,"\r\nyacc1");
	print_hex(EXAMPLE_USART0,read_data_s[0]);
	//usart_putchar(EXAMPLE_USART1,c);
	//usart_write_line(EXAMPLE_USART1,"\r\n");		
//	print_hex(EXAMPLE_USART1,read_data_s[1]);
	usart_write_line(EXAMPLE_USART0,"\n");
		sd_buffer[14]=read_data_s[0];
	sd_buffer[15]=read_data_s[1];
	//usart_write_line(EXAMPLE_USART0,"\r\nyacc2");
	print_hex(EXAMPLE_USART0,read_data_s[1]);
	//usart_putchar(EXAMPLE_USART1,c);
	usart_write_line(EXAMPLE_USART0,"\n");	
	} else {
	usart_write_line(EXAMPLE_USART0,"\r\n READ TEST2:\tFAIL\r\n");
	}
	
	
	//get z acceleration	
	delay_ms(10);
	status = twim_write (TWIM, write_data11,1,0x68,0);
	if (status == STATUS_OK) {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tPASS\r\n");
	} else {
//	usart_write_line(EXAMPLE_USART,"\r\n WRITE TEST4:\tFAIL\r\n");
	}
	delay_ms(10);
	// Read data from TARGET
	status = twim_read(TWIM,read_data,2,0x68,0);

	// Check Status
	if (status == STATUS_OK) {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ z accel:\tPASS\r\n");
//	print_hex(EXAMPLE_USART1,read_data_s[0]);
	//usart_write_line(EXAMPLE_USART1,"\r\nzacc1");
	print_hex(EXAMPLE_USART0,read_data_s[0]);
	//usart_putchar(EXAMPLE_USART1,c);
	//usart_write_line(EXAMPLE_USART1,"\r\n");	
	usart_write_line(EXAMPLE_USART0,"\n");
	//usart_write_line(EXAMPLE_USART0,"\r\nzacc2");	
	print_hex(EXAMPLE_USART0,read_data_s[1]);
	//usart_putchar(EXAMPLE_USART1,c);
//	print_hex(EXAMPLE_USART1,read_data_s[1]);
		sd_buffer[16]=read_data_s[0];
	sd_buffer[17]=read_data_s[1];	
	usart_write_line(EXAMPLE_USART0,"\n");	
	} else {
	//usart_write_line(EXAMPLE_USART1,"\r\n READ TEST2:\tFAIL\r\n");
	}
	
return ;
}




void twim_init (void)
{   //usart_write_line(EXAMPLE_USART,"\r\n In i2c init");
	gpio_clr_gpio_pin(AVR32_PIN_PB04);
	int8_t status;

	const gpio_map_t TWIM_GPIO_MAP = {
	{AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION},
	{AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION}
	};

	// Set TWIM options
	const twi_options_t TWIM_OPTIONS = {
		.pba_hz = 32000000,
		.speed = 115200,
		.chip = READ_ADDRESS,
		.smbus = false,
	};
	// TWIM gpio pins configuration
	gpio_enable_module (TWIM_GPIO_MAP,
			sizeof (TWIM_GPIO_MAP) / sizeof (TWIM_GPIO_MAP[0]));
	// Initialize as master.
	status = twim_master_init (TWIM0, &TWIM_OPTIONS);
	// Check whether TARGET device is connected
	if (status == STATUS_OK) {
		
	//	usart_write_line(EXAMPLE_USART1,"\r\n TARGET SLAVE FIND:\tPASS\r\n");
	} else if (status==-8) {
//		usart_write_line(EXAMPLE_USART1,"\r\n TARGET SLAVE FIND:\tERR_INVALID_ARG\r\n");
	} else if (status==-1) 
	{//usart_write_line(EXAMPLE_USART1,"\r\n TARGET SLAVE FIND:\tERR_INVALID_IO\r\n");
	}
}

void twim_init2 (void)
{   //usart_write_line(EXAMPLE_USART,"\r\n In i2c init");
	gpio_clr_gpio_pin(AVR32_PIN_PB04);
	int8_t status;

const gpio_map_t TWIM_GPIO_MAP = {
	{AVR32_TWIMS0_TWCK_0_0_PIN, AVR32_TWIMS0_TWCK_0_0_FUNCTION},
	{AVR32_TWIMS0_TWD_0_0_PIN, AVR32_TWIMS0_TWD_0_0_FUNCTION}
	};

	// Set TWIM options
	const twi_options_t TWIM_OPTIONS = {
		.pba_hz = 32000000,
		.speed = 115200,
		.chip = 0x21,
		.smbus = false,
	};
	// TWIM gpio pins configuration
	gpio_enable_module (TWIM_GPIO_MAP,
			sizeof (TWIM_GPIO_MAP) / sizeof (TWIM_GPIO_MAP[0]));
	// Initialize as master.
	status = twim_master_init (TWIM0, &TWIM_OPTIONS);
	// Check whether TARGET device is connected
	if (status == STATUS_OK) {
		
	//	usart_write_line(EXAMPLE_USART0,"\r\n TARGET SLAVE FIND:\tPASS\r\n");
	} else if (status==-8) {
	//	usart_write_line(EXAMPLE_USART0,"\r\n TARGET SLAVE FIND:\tERR_INVALID_ARG\r\n");
	} else if (status==-1) 
	{//usart_write_line(EXAMPLE_USART0,"\r\n TARGET SLAVE FIND:\tERR_INVALID_IO\r\n");
	}
}


void twim_init3 (void)
{   
	gpio_clr_gpio_pin(AVR32_PIN_PB04);
	int8_t status;

	const gpio_map_t TWIM_GPIO_MAP = {
	{AVR32_TWIMS1_TWCK_0_PIN, AVR32_TWIMS1_TWD_0_FUNCTION},
	{AVR32_TWIMS1_TWD_0_PIN, AVR32_TWIMS1_TWD_0_FUNCTION}
	};

	// Set TWIM options
	const twi_options_t TWIM_OPTIONS = {
		.pba_hz = 32000000,
		.speed = 115200,
		.chip = 0x68,
		.smbus = false,
	};
	// TWIM gpio pins configuration
	gpio_enable_module (TWIM_GPIO_MAP,
			sizeof (TWIM_GPIO_MAP) / sizeof (TWIM_GPIO_MAP[0]));
//usart_write_line(EXAMPLE_USART1,"\r\n In i2c init");
	// Initialize as master.
	status = twim_master_init (TWIM, &TWIM_OPTIONS);

	// Check whether TARGET device is connected
	if (status == STATUS_OK) {
		
	//	usart_write_line(EXAMPLE_USART1,"\r\n TARGET SLAVE FIND:\tPASS\r\n");
	} else if (status==-8) {
	//	usart_write_line(EXAMPLE_USART1,"\r\n TARGET SLAVE FIND:\tERR_INVALID_ARG\r\n");
	} else if (status==-1) 
	{//
	//usart_write_line(EXAMPLE_USART0,"\r\n TARGET SLAVE FIND:\tERR_INVALID_IO\r\n");
	}
	// usart_write_line(EXAMPLE_USART1,"\r\n leave i2c init");
}


//interrupt stuff-added sunday morning
/**
 * \brief The USART interrupt handler.
 *
 * \note The `__attribute__((__interrupt__))' (under GNU GCC for AVR32) and
 *       `__interrupt' (under IAR Embedded Workbench for Atmel AVR32) C function
 *       attributes are used to manage the `rete' instruction.
 */
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void usart_int_handler(void)
{ 	int16_t xbeec='u';
	int checkval;
	int checknum;
	xbeec=usart_getchar(EXAMPLE_USART0);
	
	if ((xbeec)==USART_FAILURE)
	{
//	usart_write_line(EXAMPLE_USART1,"\r\n in usart error");	
	errfound = 1;
	
	Disable_global_interrupt();
	
		Serial_init();	
			INTC_init_interrupts();

	/*
	 * Register the USART interrupt handler to the interrupt controller.
	 * usart_int_handler is the interrupt handler to register.
	 * EXAMPLE_USART_IRQ is the IRQ of the interrupt handler to register.
	 * AVR32_INTC_INT0 is the interrupt priority level to assign to the
	 * group of this IRQ.
	 */
	INTC_register_interrupt(&usart_int_handler, EXAMPLE_USART0_IRQ,
		AVR32_INTC_INT0);

	// Enable USART Rx interrupt.
	EXAMPLE_USART0->ier = AVR32_USART_IER_RXRDY_MASK;


	// Enable all interrupts.
	Enable_global_interrupt();	
	xbeechar[counter] = 'a';
	counter++;
	xbeechar[counter] = 'a';
	counter++;
	//return;
	}
	//	usart_putchar(EXAMPLE_USART1,(xbeec));
		if(startbit==1&&endbit==0)
		{   //usart_putchar(EXAMPLE_USART0,xbeec);
			xbeechar[counter] = xbeec;
			counter++;
		}
		if(xbeec=='x')
		{//usart_putchar(EXAMPLE_USART0,xbeec);
			startbit = 1;
			endbit = 0;
		}
		else if(xbeec=='y')
		{//usart_putchar(EXAMPLE_USART0,xbeec);
			counter = 0;
			startbit =0;
			endbit = 1;
		//PWM3			
		if(xbeechar[0]!='a' && xbeechar[1]!='a' && xbeechar[2]!='a' && xbeechar[3]!='a')
		{
			checknum = (xbeechar[0]-48)*1000+(xbeechar[1]-48)*100+(xbeechar[2]-48)*10+(xbeechar[3]-48);
			if(checknum >= 5025 && checknum <=5225)
			{
				tc_write_rb(tc1, EXAMPLE_TC1_CHANNEL2,((xbeechar[0]-48)*1000+(xbeechar[1]-48)*100+(xbeechar[2]-48)*10+(xbeechar[3]-48)));     // Set RA value.
			}

			
		}
		
		//PWM2
		if(xbeechar[4]!='a' && xbeechar[5]!='a' && xbeechar[6]!='a' && xbeechar[7]!='a')
		{
			checknum = (xbeechar[4]-48)*1000+(xbeechar[5]-48)*100+(xbeechar[6]-48)*10+(xbeechar[7]-48);
			if(checknum >= 5025 && checknum <=5225)
			{
			tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL0,((xbeechar[4]-48)*1000+(xbeechar[5]-48)*100+(xbeechar[6]-48)*10+(xbeechar[7]-48)));     // Set RA value.
			}		
		}
		
		//PWM0
		if(xbeechar[8]!='a' && xbeechar[9]!='a' && xbeechar[10]!='a' && xbeechar[11]!='a')
		{
			checknum = (xbeechar[8]-48)*1000+(xbeechar[9]-48)*100+(xbeechar[10]-48)*10+(xbeechar[11]-48);
			if(checknum >= 5025 && checknum <=5225)
			{
			tc_write_ra(tc0, EXAMPLE_TC0_CHANNEL1,((xbeechar[8]-48)*1000+(xbeechar[9]-48)*100+(xbeechar[10]-48)*10+(xbeechar[11]-48)));
			}
			
		}     // Set RA value.
        //PWM4
		if(xbeechar[12]!='a' && xbeechar[13]!='a' && xbeechar[14]!='a' && xbeechar[15]!='a')
		{
			checknum = (xbeechar[12]-48)*1000+(xbeechar[13]-48)*100+(xbeechar[14]-48)*10+(xbeechar[15]-48);
			if(checknum >= 5025 && checknum <=5225)
			{
				tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL1,((xbeechar[12]-48)*1000+(xbeechar[13]-48)*100+(xbeechar[14]-48)*10+(xbeechar[15]-48)));
			}
		
		}     // Set RA value.
		//PWM1
		if(xbeechar[16]!='a' && xbeechar[17]!='a' && xbeechar[18]!='a' && xbeechar[19]!='a')
		{
			checknum = (xbeechar[16]-48)*1000+(xbeechar[17]-48)*100+(xbeechar[18]-48)*10+(xbeechar[19]-48);
			if(checknum >= 5025 && checknum <=5225)
			{
			tc_write_rb(tc0, EXAMPLE_TC0_CHANNEL2,((xbeechar[16]-48)*1000+(xbeechar[17]-48)*100+(xbeechar[18]-48)*10+(xbeechar[19]-48)));  
			}
 
		   // Set RA value.
		}
		if(xbeechar[20]=='1' || xbeechar[21]=='1' || xbeechar[22]=='1' || xbeechar[23]=='1')
		{
		click();  
		//usart_write_line(EXAMPLE_USART1, "\r\ncleeeck");
		   // Set RA value.
		}
		
		if(xbeechar[24]=='1' || xbeechar[25]=='1' || xbeechar[26]=='1' || xbeechar[27]=='1')
		{
	//	usart_write_line(EXAMPLE_USART1, "\r\nmux");
		
		if(i==0)	
		{gpio_set_gpio_pin(AVR32_PIN_PX37);
			//usart_write_line(EXAMPLE_USART0, "\r\nsssssss");
		i=1;}
		else if(i==1)
		{gpio_clr_gpio_pin(AVR32_PIN_PX37);
			//usart_write_line(EXAMPLE_USART0, "\r\nxxxxxxx");
		i=0;}	
		//click();  

		   // Set RA value.
		}
	/*	checkval = tc_read_ra(tc0,EXAMPLE_TC0_CHANNEL1);
		if(checkval < 5025 || checkval >5225)
		{   print_char(EXAMPLE_USART1, checkval);
			tc_write_ra(tc0,EXAMPLE_TC0_CHANNEL1,buf1);
		}
		else
		{
			buf1 = checkval;	
		}	
		checkval = tc_read_rb(tc0,EXAMPLE_TC0_CHANNEL1);
		if(checkval < 5025 || checkval >5225)
		{print_char(EXAMPLE_USART1, checkval);
			tc_write_rb(tc0,EXAMPLE_TC0_CHANNEL1,buf2);
		}
		else
		{
			buf2 = checkval;	
		}	
		checkval = tc_read_rb(tc0,EXAMPLE_TC0_CHANNEL0);
		if(checkval < 5025 || checkval >5225)
		{print_char(EXAMPLE_USART1, checkval);
			tc_write_rb(tc0,EXAMPLE_TC0_CHANNEL0,buf3);
		}
		else
		{
			buf3 = checkval;	
		}	
		checkval = tc_read_rb(tc0,EXAMPLE_TC0_CHANNEL2);
		if(checkval < 5025 || checkval >5225)
		{print_char(EXAMPLE_USART1, checkval);
			tc_write_rb(tc0,EXAMPLE_TC0_CHANNEL2,buf4);
		}
		else
		{
			buf4 = checkval;	
		}	
		checkval = tc_read_rb(tc1,EXAMPLE_TC1_CHANNEL2);
		if(checkval < 5025 || checkval >5225)
		{print_char(EXAMPLE_USART1, checkval);
			tc_write_rb(tc1,EXAMPLE_TC1_CHANNEL2,buf5);
		}
		else
		{
			buf5 = checkval;	
		}*/
		}
}




//start timer interrupt stuff
/**
 * \brief TC interrupt.
 *
 * The ISR handles RC compare interrupt and sets the update_timer flag to
 * update the timer value.
 */
__attribute__((__interrupt__))
static void tc_irq(void)
{
	// Increment the ms seconds counter
	tc_tick++;
	/*
	 * TODO: Place a breakpoint here and watch the update of tc_tick variable
	 * in the Watch Window.
	 */

	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(EXAMPLE_TC, EXAMPLE_TC_CHANNEL);

	// specify that an interrupt has been raised
	update_timer = true;
	// Toggle the GPIO line
	gpio_tgl_gpio_pin(EXAMPLE_TOGGLE_PIN);
}
//end interrupt stuff

