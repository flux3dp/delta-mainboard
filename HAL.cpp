// **************************************************************************
//
// Description:          *** HAL for Arduino Due ***
//
// **************************************************************************

#include "HAL.h"
#include "Configuration.h"

#include <Wire.h>

uint8_t MCUSR;
uint8_t HAL_step_timer_RC;

// disable interrupts
void cli(void) {
  noInterrupts();
}

// enable interrupts
void sei(void) {
  interrupts();
}

void _delay_ms (int delay_ms) {
  //todo: port for Due?
  delay (delay_ms);
}

void _delay_us (int delay_us) {
  delayMicroseconds(delay_us);
}

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

#ifndef DUE_SOFTWARE_SPI
  int spiDueDividors[] = {10,21,42,84,168,255,255};
#endif

// return free memory between end of heap (or end bss) and whatever is current
int freeMemory() {
  int free_memory;
  int heap_end = (int)_sbrk(0);

  if(heap_end == 0)
    free_memory = ((int)&free_memory) - ((int)&_ebss);
  else
    free_memory = ((int)&free_memory) - heap_end;

  return free_memory;
}


// --------------------------------------------------------------------------
// hardware SPI
// --------------------------------------------------------------------------
bool spiInitMaded = false;
void spiBegin() {
  if(spiInitMaded == false)
  {
    // Configre SPI pins
    PIO_Configure(
       g_APinDescription[SCK_PIN].pPort,
       g_APinDescription[SCK_PIN].ulPinType,
       g_APinDescription[SCK_PIN].ulPin,
       g_APinDescription[SCK_PIN].ulPinConfiguration);
    PIO_Configure(
       g_APinDescription[MOSI_PIN].pPort,
       g_APinDescription[MOSI_PIN].ulPinType,
       g_APinDescription[MOSI_PIN].ulPin,
       g_APinDescription[MOSI_PIN].ulPinConfiguration);
    PIO_Configure(
       g_APinDescription[MISO_PIN].pPort,
       g_APinDescription[MISO_PIN].ulPinType,
       g_APinDescription[MISO_PIN].ulPin,
       g_APinDescription[MISO_PIN].ulPinConfiguration);

    // set master mode, peripheral select, fault detection
    SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PS);
    SPI_Enable(SPI0);
   
    #if MB(ALLIGATOR)
      // Init Motor Fault
      pinMode( MOTOR_FAULT_PIN, INPUT);
      pinMode( DAC_SYNC, OUTPUT);
      pinMode( SPI_EEPROM1_CS, OUTPUT);
      pinMode( SPI_EEPROM2_CS, OUTPUT);
      pinMode( SPI_FLASH_CS, OUTPUT);
      //pinMode( SPI_PIN, OUTPUT);
      digitalWrite( DAC_SYNC , HIGH );
      digitalWrite( SPI_EEPROM1_CS , HIGH );
      digitalWrite( SPI_EEPROM2_CS , HIGH );
      digitalWrite( SPI_FLASH_CS , HIGH );
      digitalWrite( SDSS , HIGH );
      //pinMode( SPI_PIN, OUTPUT);
    #endif//MB(ALLIGATOR)
/*
    PIO_Configure(
       g_APinDescription[SPI_PIN].pPort,
       g_APinDescription[SPI_PIN].ulPinType,
       g_APinDescription[SPI_PIN].ulPin,
       g_APinDescription[SPI_PIN].ulPinConfiguration);
 */
    spiInit(1);
    spiInitMaded = true;
  }
}

void spiInit(uint8_t spiClock) {
  if(spiInitMaded == false)
  {
    if(spiClock>4) spiClock = 1;
    #if MB(ALLIGATOR)
      // Set SPI mode 1, clock, select not active after transfer, with delay between transfers  
      SPI_ConfigureNPCS(SPI0, SPI_CHAN_DAC, SPI_CSR_CSAAT |
        SPI_CSR_SCBR(spiDueDividors[spiClock]) | SPI_CSR_DLYBCT(1));
      // Set SPI mode 0, clock, select not active after transfer, with delay between transfers 
      SPI_ConfigureNPCS(SPI0, SPI_CHAN_EEPROM1,SPI_CSR_NCPHA | SPI_CSR_CSAAT |
        SPI_CSR_SCBR(spiDueDividors[spiClock]) | SPI_CSR_DLYBCT(1));
    #endif//MB(ALLIGATOR)

    // Set SPI mode 0, clock, select not active after transfer, with delay between transfers
    //SPI_ConfigureNPCS(SPI0, SPI_CHAN, SPI_CSR_NCPHA | SPI_CSR_CSAAT |
      //SPI_CSR_SCBR(spiDueDividors[spiClock]) | SPI_CSR_DLYBCT(1));
    //SPI_Enable(SPI0);
    spiInitMaded = true;
  }
}

void spiSendByte(uint32_t chan, byte b) {
  uint8_t dummy_read = 0;
  // wait for transmit register empty
  while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
  // write byte with address and end transmission flag
  SPI0->SPI_TDR = (uint32_t)b | SPI_PCS(chan) | SPI_TDR_LASTXFER;
  // wait for receive register 
  while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
  // clear status
  while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
    dummy_read = SPI0->SPI_RDR;
}
  
void spiSend(uint32_t chan ,const uint8_t* buf , size_t n) {
  uint8_t dummy_read = 0;
  if (n == 0) return;
  for (int i=0; i<n-1; i++)
  {
    while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
    SPI0->SPI_TDR = (uint32_t)buf[i] | SPI_PCS(chan);
    while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
     while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
      dummy_read = SPI0->SPI_RDR;
  }
  spiSendByte(chan, buf[n-1]);
}

uint8_t spiReceive(uint32_t chan) {
  uint8_t spirec_tmp;
  // wait for transmit register empty
  while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
  while ((SPI0->SPI_SR & SPI_SR_RDRF) == 1)
    spirec_tmp =  SPI0->SPI_RDR;

  // write dummy byte with address and end transmission flag
  SPI0->SPI_TDR = 0x000000FF | SPI_PCS(chan) | SPI_TDR_LASTXFER;

  // wait for receive register 
  while ((SPI0->SPI_SR & SPI_SR_RDRF) == 0);
  // get byte from receive register
  return SPI0->SPI_RDR;
}

// --------------------------------------------------------------------------
// eeprom
// --------------------------------------------------------------------------

static bool eeprom_initialised = false;
static uint8_t eeprom_device_address = 0x50;

static void eeprom_init(void) {
  #if MB(ALLIGATOR)
  #else
    if (!eeprom_initialised) {
      Wire.begin();
      eeprom_initialised = true;
    }
  #endif// MB(ALLIGATOR)
}

#if MB(ALLIGATOR)
  static void eprBurnValue(unsigned int pos, int size, unsigned char * newvalue) {
    uint8_t eeprom_temp[3];

    /*write enable*/
    eeprom_temp[0] = 6;//WREN
    digitalWrite( SPI_EEPROM1_CS , LOW );
    spiSend(SPI_CHAN_EEPROM1 ,eeprom_temp , 1);
    digitalWrite( SPI_EEPROM1_CS , HIGH );
    _delay_ms(1);

    /*write addr*/
    eeprom_temp[0] = 2;//WRITE
    eeprom_temp[1] = ((pos>>8) & 0xFF);//addrH
    eeprom_temp[2] = (pos& 0xFF);//addrL
    digitalWrite( SPI_EEPROM1_CS , LOW );
    spiSend(SPI_CHAN_EEPROM1 ,eeprom_temp , 3);        

    spiSend(SPI_CHAN_EEPROM1 ,newvalue , 1);
    digitalWrite( SPI_EEPROM1_CS , HIGH );
    _delay_ms(7);   // wait for page write to complete
  }

  // Read any data type from EEPROM that was previously written by eprBurnValue
  static uint8_t eprGetValue(unsigned int pos) {
    int i = 0;
    uint8_t v;
    uint8_t eeprom_temp[3];
    // set read location
    // begin transmission from device

    eeprom_temp[0] = 3;//READ
    eeprom_temp[1] = ((pos>>8) & 0xFF);//addrH
    eeprom_temp[2] = (pos& 0xFF);//addrL
    digitalWrite( SPI_EEPROM1_CS , HIGH );
    digitalWrite( SPI_EEPROM1_CS , LOW );
    spiSend(SPI_CHAN_EEPROM1 ,eeprom_temp , 3);

    v = spiReceive(SPI_CHAN_EEPROM1); 
    digitalWrite( SPI_EEPROM1_CS , HIGH );
    return v;
  }
#endif

void eeprom_write_byte(unsigned char *pos, unsigned char value) {
  #if MB(ALLIGATOR)
    eprBurnValue((unsigned) pos, 1, &value);
  #else

    unsigned eeprom_address = (unsigned) pos;

    eeprom_init();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)(eeprom_address > 8));   // MSB
    Wire.write((int)(eeprom_address & 0xFF)); // LSB
    Wire.write(value);
    Wire.endTransmission();

    // wait for write cycle to complete
    // this could be done more efficiently with "acknowledge polling"
    delay(5);
  #endif// MB(ALLIGATOR)
}

unsigned char eeprom_read_byte(unsigned char *pos) {
  #if MB(ALLIGATOR)
      return eprGetValue((unsigned) pos);
  #else
    byte data = 0xFF;
    unsigned eeprom_address = (unsigned) pos;

    eeprom_init ();

    Wire.beginTransmission(eeprom_device_address);
    Wire.write((int)(eeprom_address > 8));   // MSB
    Wire.write((int)(eeprom_address & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(eeprom_device_address, (byte)1);
    if (Wire.available()) data = Wire.read();
    return data;
  #endif// MB(ALLIGATOR)
}

// --------------------------------------------------------------------------
// Timers
// --------------------------------------------------------------------------

typedef struct {
    Tc          *pTimerRegs;
    uint16_t    channel;
    IRQn_Type   IRQ_Id;
  } tTimerConfig ;

#define  NUM_HARDWARE_TIMERS 9

static const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] =
  {
    { TC0, 0, TC0_IRQn},
    { TC0, 1, TC1_IRQn},
    { TC0, 2, TC2_IRQn},
    { TC1, 0, TC3_IRQn},
    { TC1, 1, TC4_IRQn},
    { TC1, 2, TC5_IRQn},
    { TC2, 0, TC6_IRQn},
    { TC2, 1, TC7_IRQn},
    { TC2, 2, TC8_IRQn},
  };


/*
	Timer_clock1: Prescaler 2 -> 42MHz
	Timer_clock2: Prescaler 8 -> 10.5MHz
	Timer_clock3: Prescaler 32 -> 2.625MHz
	Timer_clock4: Prescaler 128 -> 656.25kHz
*/
	
// new timer by Ps991
// thanks for that work
// http://forum.arduino.cc/index.php?topic=297397.0

void HAL_step_timer_start() {
  uint32_t tc_count, tc_clock;
  uint8_t timer_num;
  
  pmc_set_writeprotect(false); //remove write protection on registers
  NVIC_SetPriorityGrouping(4);
  
  // Timer for stepper
  // Timer 3 HAL.h STEP_TIMER_NUM
  timer_num = STEP_TIMER_NUM;
  
  // Get the ISR from table
  Tc *tc = TimerConfig [timer_num].pTimerRegs;
  IRQn_Type irq = TimerConfig [timer_num].IRQ_Id;
  uint32_t channel = TimerConfig [timer_num].channel;
  
  pmc_enable_periph_clk((uint32_t)irq); //we need a clock?
  NVIC_SetPriority(irq, NVIC_EncodePriority(4, 4, 0));
  
  TC_Configure(tc, channel, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_CPCTRG); //set clock rate (CLOCK1 is MCK/2) and reset counter register C on match
  tc->TC_CHANNEL[channel].TC_IER |= TC_IER_CPCS; //enable interrupt on timer match with register C

  tc->TC_CHANNEL[channel].TC_RC   = (VARIANT_MCK >> 1) / 1000; // start with 1kHz as frequency; //interrupt occurs every x interations of the timer counter
  TC_Start(tc, channel); //start timer counter
    
  //tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  //tc->TC_CHANNEL[channel].TC_IDR =~ TC_IER_CPCS;
  
  NVIC_EnableIRQ(irq); //enable Nested Vector Interrupt Controller
}

void HAL_temp_timer_start (uint8_t timer_num) {
		
	Tc *tc = TimerConfig [timer_num].pTimerRegs;
	IRQn_Type irq = TimerConfig [timer_num].IRQ_Id;
	uint32_t channel = TimerConfig [timer_num].channel;

	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);
	
	NVIC_SetPriorityGrouping(4);
	
	NVIC_SetPriority(irq, NVIC_EncodePriority(4, 6, 0));
	
	TC_Configure (tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);

	uint32_t rc = VARIANT_MCK / 128 / (512*12);
	TC_SetRC(tc, channel, rc);
	TC_Start(tc, channel);

	//enable interrupt on RC compare
	tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
	tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;

	NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt (uint8_t timer_num) {
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IER = TC_IER_CPCS;
}

void HAL_timer_disable_interrupt (uint8_t timer_num) {
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IDR = ~TC_IER_CPCS;
}

void HAL_timer_set_count (uint8_t timer_num, uint32_t count) {
	if (count < 210) count = 210;
	const tTimerConfig *pConfig = &TimerConfig [timer_num];
	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_RC = count;

	//TC_SetRC (pConfig->pTimerRegs, pConfig->channel, count);

	if(TC_ReadCV(pConfig->pTimerRegs, pConfig->channel)>count)
		TC_Start(pConfig->pTimerRegs, pConfig->channel);
}

void HAL_timer_isr_prologue (uint8_t timer_num) {
	const tTimerConfig *pConfig = &TimerConfig [timer_num];
  uint32_t dummy;
  dummy = pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_SR;
}

int HAL_timer_get_count (uint8_t timer_num) {
	Tc *tc = TimerConfig [timer_num].pTimerRegs;
	uint32_t channel = TimerConfig [timer_num].channel;
	return tc->TC_CHANNEL[channel].TC_RC;
}
