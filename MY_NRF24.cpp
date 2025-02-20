#include "MY_NRF24.h"

//*** Variables declaration ***//
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

//**** Functions prototypes ****//
//Microsecond delay function
void NRF24L01::NRF24_DelayMicroSeconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
	while(uSecVar--);
}

NRF24L01::NRF24L01(GPIO_TypeDef *nrf24PORTCSN, uint16_t nrfCSN_Pin,GPIO_TypeDef *nrf24PORTCE, uint16_t nrfCE_Pin, SPI_HandleTypeDef *nrfSPI)
{
	//Copy SPI handle variable
	nrf24_hspi =  nrfSPI;
	//Copy Pins and Port variables
	nrf24_PORTCSN = nrf24PORTCSN;
	nrf24_CSN_PIN = nrfCSN_Pin;
	nrf24_PORTCE = nrf24PORTCE;
	nrf24_CE_PIN = nrfCE_Pin;
}

//Chip Select function
void NRF24L01::NRF24_csn(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORTCSN, nrf24_CSN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORTCSN, nrf24_CSN_PIN, GPIO_PIN_RESET);
}
//Chip Enable
void NRF24L01::NRF24_ce(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORTCE, nrf24_CE_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORTCE, nrf24_CE_PIN, GPIO_PIN_RESET);
}
//Read single byte from a register
uint8_t NRF24L01::NRF24_read_register(uint8_t reg)
{
	uint8_t spiBuf[3];
	uint8_t retData;
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(nrf24_hspi, spiBuf, 1, 100);
	//Receive data
	HAL_SPI_Receive(nrf24_hspi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];
	//Bring CSN high
	NRF24_csn(1);
	return retData;
}
//Read multiple bytes register
void NRF24L01::NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	//spiStatus = NRF24_SPI_Write(spiBuf, 1);
	HAL_SPI_Transmit(nrf24_hspi, spiBuf, 1, 100);
	//Receive data
	HAL_SPI_Receive(nrf24_hspi, buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//Write single byte register
void NRF24L01::NRF24_write_register(uint8_t reg, uint8_t value)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	spiBuf[1] = value;
	HAL_SPI_Transmit(nrf24_hspi, spiBuf, 2, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//Write multipl bytes register
void NRF24L01::NRF24_write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	HAL_SPI_Transmit(nrf24_hspi, spiBuf, 1, 100);
	HAL_SPI_Transmit(nrf24_hspi, (uint8_t*)buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//Write transmit payload
void NRF24L01::NRF24_write_payload(uint8_t* buf, uint8_t len)
{
	uint8_t wrPayloadCmd;
	//Bring CSN low
	NRF24_csn(0);
	//Send Write Tx payload command followed by pbuf data
	wrPayloadCmd = CMD_W_TX_PAYLOAD;
	HAL_SPI_Transmit(nrf24_hspi, &wrPayloadCmd, 1, 100);
	HAL_SPI_Transmit(nrf24_hspi, (uint8_t *)buf, len, 100);
	//Bring CSN high
	NRF24_csn(1);
}
//Read receive payload
void NRF24L01::NRF24_read_payload(uint8_t* buf, uint8_t len)
{
	uint8_t cmdRxBuf;
	//Get data length using payload size
	uint8_t data_len = MIN(len, NRF24_getPayloadSize());
	//Read data from Rx payload buffer
	NRF24_csn(0);
	cmdRxBuf = CMD_R_RX_PAYLOAD;
	HAL_SPI_Transmit(nrf24_hspi, &cmdRxBuf, 1, 100);
	HAL_SPI_Receive(nrf24_hspi, buf, data_len, 100);
	NRF24_csn(1);
}

//Flush Tx buffer
void NRF24L01::NRF24_flush_tx(void)
{
	NRF24_write_register(CMD_FLUSH_TX, 0xFF);
}
//Flush Rx buffer
void NRF24L01::NRF24_flush_rx(void)
{
	NRF24_write_register(CMD_FLUSH_RX, 0xFF);
}
//Get status register value
uint8_t NRF24L01::NRF24_get_status(void)
{
	uint8_t statReg;
	statReg = NRF24_read_register(REG_STATUS);
	return statReg;
}

//Begin function
void NRF24L01::NRF24_begin(void)
{	
	//Put pins to idle state
	NRF24_csn(1);
	NRF24_ce(0);
	
	//**** Soft Reset Registers default values ****//
	NRF24_write_register(0x00, 0x08);
	NRF24_write_register(0x01, 0x3f);
	NRF24_write_register(0x02, 0x03);
	NRF24_write_register(0x03, 0x03);
	NRF24_write_register(0x04, 0x03);
	NRF24_write_register(0x05, 0x02);
	NRF24_write_register(0x06, 0x0f);
	NRF24_write_register(0x07, 0x0e);
	NRF24_write_register(0x08, 0x00);
	NRF24_write_register(0x09, 0x00);
	uint8_t pipeAddrVar[6];
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	NRF24_write_registerN(0x0A, pipeAddrVar, 5);
	pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; 
	NRF24_write_registerN(0x0B, pipeAddrVar, 5);
	NRF24_write_register(0x0C, 0xC3);
	NRF24_write_register(0x0D, 0xC4);
	NRF24_write_register(0x0E, 0xC5);
	NRF24_write_register(0x0F, 0xC6);
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	NRF24_write_registerN(0x10, pipeAddrVar, 5);
	NRF24_write_register(0x11, 0);
	NRF24_write_register(0x12, 0);
	NRF24_write_register(0x13, 0);
	NRF24_write_register(0x14, 0);
	NRF24_write_register(0x15, 0);
	NRF24_write_register(0x16, 0);
	
	NRF24_ACTIVATE_cmd();
	NRF24_write_register(0x1c, 0);
	NRF24_write_register(0x1d, 0);
	//Initialise retries 15 and delay 1250 usec
	NRF24_setRetries(15, 15);
	//Initialise PA level to max (0dB)
	NRF24_setPALevel(RF24_PA_0dB);
	//Initialise data rate to 1Mbps
	NRF24_setDataRate(RF24_2MBPS);
	//Initalise CRC length to 16-bit (2 bytes)
	NRF24_setCRCLength(RF24_CRC_16);
	//Disable dynamic payload
	NRF24_disableDynamicPayloads();
	//Set payload size
	NRF24_setPayloadSize(32);
	
	//Reset status register
	NRF24_resetStatus();
	//Initialise channel to 76
	NRF24_setChannel(76);
	//Flush buffers
	NRF24_flush_tx();
	NRF24_flush_rx();
	
	NRF24_powerDown();
	
}
//Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
void NRF24L01::NRF24_startListening(void)
{
	//Power up and set to RX mode
	NRF24_write_register(REG_CONFIG, NRF24_read_register(REG_CONFIG) | (1UL<<1) |(1UL <<0));
	//Restore pipe 0 address if exists
	if(pipe0_reading_address)
		NRF24_write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);
	
	//Flush buffers
	NRF24_flush_tx();
	NRF24_flush_rx();
	//Set CE HIGH to start listenning
	NRF24_ce(1);
	//Wait for 130 uSec for the radio to come on
	NRF24_DelayMicroSeconds(150);
}
//14. Stop listening (essential before any write operation)
void NRF24L01::NRF24_stopListening(void)
{
	NRF24_ce(0);
	NRF24_flush_tx();
	NRF24_flush_rx();
}
//Write(Transmit data), returns true if successfully sent
bool NRF24L01::NRF24_write( uint8_t* buf, uint8_t len )
{
	bool retStatus;
	//Start writing
	NRF24_resetStatus();
	NRF24_startWrite(buf,len);
	//Data monitor
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 10; //ms to wait for timeout
	do
  {
    NRF24_read_registerN(REG_OBSERVE_TX,&observe_tx,1);
		//Get status register
		status = NRF24_get_status();
  }
  while( ! ( status & ( _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );
	
//	printConfigReg();
//	printStatusReg();
	
	bool tx_ok, tx_fail;
  NRF24_whatHappened(&tx_ok,&tx_fail, &ack_payload_available);
	retStatus = tx_ok;
	if ( ack_payload_available )
  {
    ack_payload_length = NRF24_getDynamicPayloadSize();
	}
	
	//Power down
	NRF24_available();
	NRF24_flush_tx();
	return retStatus;
}
//Check for available data to read
bool NRF24L01::NRF24_available(void)
{
	return NRF24_availablePipe(NULL);
}
//Read received data
bool NRF24L01::NRF24_read( uint8_t* buf, uint8_t len )
{
	NRF24_read_payload( buf, len );
	uint8_t rxStatus = NRF24_read_register(REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY);
	NRF24_flush_rx();
	NRF24_getDynamicPayloadSize();
	return rxStatus;
}
//Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
void NRF24L01::NRF24_openWritingPipe(uint64_t address)
{
	NRF24_write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
  NRF24_write_registerN(REG_TX_ADDR, (uint8_t *)(&address), 5);
	
	const uint8_t max_payload_size = 32;
  NRF24_write_register(REG_RX_PW_P0,MIN(payload_size,max_payload_size));
}
//Open reading pipe
void NRF24L01::NRF24_openReadingPipe(uint8_t number, uint64_t address)
{
	if (number == 0)
    pipe0_reading_address = address;
	
	if(number <= 6)
	{
		if(number < 2)
		{
			//Address width is 5 bytes
			NRF24_write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 5);
		}
		else
		{
			NRF24_write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 1);
		}
		//Write payload size
		NRF24_write_register(RF24_RX_PW_PIPE[number],payload_size);
		//Enable pipe
		NRF24_write_register(REG_EN_RXADDR, NRF24_read_register(REG_EN_RXADDR) | _BV(number));
	}
	
}
//set transmit retries (rf24_Retries_e) and delay
void NRF24L01::NRF24_setRetries(uint8_t delay, uint8_t count)
{
	NRF24_write_register(REG_SETUP_RETR,(delay&0xf)<<BIT_ARD | (count&0xf)<<BIT_ARC);
}

//Set RF channel frequency
void NRF24L01::NRF24_setChannel(uint8_t channel)
{
	const uint8_t max_channel = 127;
  NRF24_write_register(REG_RF_CH,MIN(channel,max_channel));
}
//Set payload size
void NRF24L01::NRF24_setPayloadSize(uint8_t size)
{
	const uint8_t max_payload_size = 32;
  payload_size = MIN(size,max_payload_size);
}
//Get payload size
uint8_t NRF24L01::NRF24_getPayloadSize(void)
{
	return payload_size;
}
//Get dynamic payload size, of latest packet received
uint8_t NRF24L01::NRF24_getDynamicPayloadSize(void)
{
	return NRF24_read_register(CMD_R_RX_PL_WID);
}
//Enable payload on Ackknowledge packet
void NRF24L01::NRF24_enableAckPayload(void)
{
	//Need to enable dynamic payload and Ack payload together
	 NRF24_write_register(REG_FEATURE,NRF24_read_register(REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL) );
	if(!NRF24_read_register(REG_FEATURE))
	{
		NRF24_ACTIVATE_cmd();
		NRF24_write_register(REG_FEATURE,NRF24_read_register(REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL) );
	}
	// Enable dynamic payload on pipes 0 & 1
	NRF24_write_register(REG_DYNPD,NRF24_read_register(REG_DYNPD) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
}
//Enable dynamic payloads
void NRF24L01::NRF24_enableDynamicPayloads(void)
{
	//Enable dynamic payload through FEATURE register
	NRF24_write_register(REG_FEATURE,NRF24_read_register(REG_FEATURE) |  _BV(BIT_EN_DPL) );
	if(!NRF24_read_register(REG_FEATURE))
	{
		NRF24_ACTIVATE_cmd();
		NRF24_write_register(REG_FEATURE,NRF24_read_register(REG_FEATURE) |  _BV(BIT_EN_DPL) );
	}
	//Enable Dynamic payload on all pipes
	NRF24_write_register(REG_DYNPD,NRF24_read_register(REG_DYNPD) | _BV(BIT_DPL_P5) | _BV(BIT_DPL_P4) | _BV(BIT_DPL_P3) | _BV(BIT_DPL_P2) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
  dynamic_payloads_enabled = true;
	
}

void NRF24L01::NRF24_disableDynamicPayloads(void)
{
	NRF24_write_register(REG_FEATURE,NRF24_read_register(REG_FEATURE) &  ~(_BV(BIT_EN_DPL)) );
	//Disable for all pipes 
	NRF24_write_register(REG_DYNPD,0);
	dynamic_payloads_enabled = false;
}
//Check if module is NRF24L01+ or normal module
bool NRF24L01::NRF24_isNRF_Plus(void)
{
	return p_variant;
}
//Set Auto Ack for all
void NRF24L01::NRF24_setAutoAck(bool enable)
{
	if ( enable )
    NRF24_write_register(REG_EN_AA, 0x3F);
  else
    NRF24_write_register(REG_EN_AA, 0x00);
}
//Set Auto Ack for certain pipe
void NRF24L01::NRF24_setAutoAckPipe( uint8_t pipe, bool enable )
{
	if ( pipe <= 6 )
  {
    uint8_t en_aa = NRF24_read_register( REG_EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    NRF24_write_register( REG_EN_AA, en_aa ) ;
  }
}
//Set transmit power level
void NRF24L01::NRF24_setPALevel( rf24_pa_dbm_e level )
{
	uint8_t setup = NRF24_read_register(REG_RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_0dB)
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_m6dB )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_m12dB )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_m18dB )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  NRF24_write_register( REG_RF_SETUP, setup ) ;
}
//Get transmit power level
rf24_pa_dbm_e NRF24L01::NRF24_getPALevel( void )
{
	rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = NRF24_read_register(REG_RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_0dB ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_m6dB ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_m12dB ;
  }
  else
  {
    result = RF24_PA_m18dB ;
  }

  return result ;
}
//Set data rate (250 Kbps, 1Mbps, 2Mbps)
bool NRF24L01::NRF24_setDataRate(rf24_datarate_e speed)
{
	bool result = false;
  uint8_t setup = NRF24_read_register(REG_RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  NRF24_write_register(REG_RF_SETUP,setup);

  // Verify our result
  if ( NRF24_read_register(REG_RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}
//Get data rate
rf24_datarate_e NRF24L01::NRF24_getDataRate( void )
{
	rf24_datarate_e result ;
  uint8_t dr = NRF24_read_register(REG_RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
  
  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}
//Set crc length (disable, 8-bits or 16-bits)
void NRF24L01::NRF24_setCRCLength(rf24_crclength_e length)
{
	uint8_t config = NRF24_read_register(REG_CONFIG) & ~( _BV(BIT_CRCO) | _BV(BIT_EN_CRC)) ;
  
  // switch uses RAM
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(BIT_EN_CRC);
  }
  else
  {
    config |= _BV(BIT_EN_CRC);
    config |= _BV( BIT_CRCO );
  }
  NRF24_write_register( REG_CONFIG, config );
}
//Get CRC length
rf24_crclength_e NRF24L01::NRF24_getCRCLength(void)
{
	rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = NRF24_read_register(REG_CONFIG) & ( _BV(BIT_CRCO) | _BV(BIT_EN_CRC)) ;

  if ( config & _BV(BIT_EN_CRC ) )
  {
    if ( config & _BV(BIT_CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}
//Disable CRC
void NRF24L01::NRF24_disableCRC( void )
{
	uint8_t disable = NRF24_read_register(REG_CONFIG) & ~_BV(BIT_EN_CRC) ;
  NRF24_write_register( REG_CONFIG, disable ) ;
}
//power up
void NRF24L01::NRF24_powerUp(void)
{
	NRF24_write_register(REG_CONFIG,NRF24_read_register(REG_CONFIG) | _BV(BIT_PWR_UP));
}
//power down
void NRF24L01::NRF24_powerDown(void)
{
	NRF24_write_register(REG_CONFIG,NRF24_read_register(REG_CONFIG) & ~_BV(BIT_PWR_UP));
}
//Check if data are available and on which pipe (Use this for multiple rx pipes)
bool NRF24L01::NRF24_availablePipe(uint8_t* pipe_num)
{
	uint8_t status = NRF24_get_status();

  bool result = ( status & _BV(BIT_RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> BIT_RX_P_NO ) & 0x7;

    // Clear the status bit
    NRF24_write_register(REG_STATUS,_BV(BIT_RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(BIT_TX_DS) )
    {
      NRF24_write_register(REG_STATUS,_BV(BIT_TX_DS));
    }
  }
  return result;
}
//Start write (for IRQ mode)
void NRF24L01::NRF24_startWrite( uint8_t* buf, uint8_t len )
{
	// Transmitter power-up
  NRF24_write_register(REG_CONFIG, ( NRF24_read_register(REG_CONFIG) | _BV(BIT_PWR_UP) ) & ~_BV(BIT_PRIM_RX) );
  NRF24_DelayMicroSeconds(150);

  // Send the payload
  NRF24_write_payload( buf, len );

  // Enable Tx for 15usec
  NRF24_ce(1);
  NRF24_DelayMicroSeconds(15);
  NRF24_ce(0);
}
//Write acknowledge payload
void NRF24L01::NRF24_writeAckPayload(uint8_t pipe, uint8_t* buf, uint8_t len)
{
	const uint8_t* current = (uint8_t *)buf;
	const uint8_t max_payload_size = 32;
  uint8_t data_len = MIN(len,max_payload_size);
	
  NRF24_csn(0);
	NRF24_write_registerN(CMD_W_ACK_PAYLOAD | ( pipe & 0x7 ) , current, data_len);
  NRF24_csn(1);
}
//Check if an Ack payload is available
bool NRF24L01::NRF24_isAckPayloadAvailable(void)
{
	bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}
//Check interrupt flags
void NRF24L01::NRF24_whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready)
{
	uint8_t status = NRF24_get_status();
	*tx_ok = 0;
	NRF24_write_register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
  // Report to the user what happened
  *tx_ok = status & _BV(BIT_TX_DS);
  *tx_fail = status & _BV(BIT_MAX_RT);
  *rx_ready = status & _BV(BIT_RX_DR);
}
//Test if there is a carrier on the previous listenning period (useful to check for intereference)
bool NRF24L01::NRF24_testCarrier(void)
{
	return NRF24_read_register(REG_CD) & 1;
}
//Test if a signal carrier exists (=> -64dB), only for NRF24L01+
bool NRF24L01::NRF24_testRPD(void)
{
	return NRF24_read_register(REG_RPD) & 1;
}

//Reset Status
void NRF24L01::NRF24_resetStatus(void)
{
	NRF24_write_register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
}

//ACTIVATE cmd
void NRF24L01::NRF24_ACTIVATE_cmd(void)
{
	uint8_t cmdRxBuf[2];
	//Read data from Rx payload buffer
	NRF24_csn(0);
	cmdRxBuf[0] = CMD_ACTIVATE;
	cmdRxBuf[1] = 0x73;
	HAL_SPI_Transmit(nrf24_hspi, cmdRxBuf, 2, 100);
	NRF24_csn(1);
}
//Get AckPayload Size
uint8_t NRF24L01::NRF24_GetAckPayloadSize(void)
{
	return ack_payload_length;
}
