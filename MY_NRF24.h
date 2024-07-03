//List of header files  
#include "stm32f1xx_hal.h"   //** Change this according to your STM32 series **//
#include "nRF24L01.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

//**** TypeDefs ****//
//1. Power Amplifier function, NRF24_setPALevel() 
typedef enum { 
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR 
}rf24_pa_dbm_e ;

//2. NRF24_setDataRate() input
typedef enum { 
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
}rf24_datarate_e;

//3. NRF24_setCRCLength() input
typedef enum { 
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;

//4. Pipe address registers
static const uint8_t NRF24_ADDR_REGS[7] = {
	REG_RX_ADDR_P0,
	REG_RX_ADDR_P1,
	REG_RX_ADDR_P2,
	REG_RX_ADDR_P3,
	REG_RX_ADDR_P4,
	REG_RX_ADDR_P5,
	REG_TX_ADDR
};

//5. RX_PW_Px registers addresses
static const uint8_t RF24_RX_PW_PIPE[6] = {
	REG_RX_PW_P0, 
	REG_RX_PW_P1,
	REG_RX_PW_P2,
	REG_RX_PW_P3,
	REG_RX_PW_P4,
	REG_RX_PW_P5
};

class NRF24L01
{
	public:
		NRF24L01(GPIO_TypeDef *nrf24PORTCSN, uint16_t nrfCSN_Pin,GPIO_TypeDef *nrf24PORTCE, uint16_t nrfCE_Pin, SPI_HandleTypeDef *nrfSPI);

		//Begin function
		void NRF24_begin(void);
		//Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
		void NRF24_startListening(void);
		//Stop listening (essential before any write operation)
		void NRF24_stopListening(void);
	
		//Write(Transmit data), returns true if successfully sent
		bool NRF24_write( uint8_t* buf, uint8_t len );
		//Check for available data to read
		bool NRF24_available(void);
		//Read received data
		bool NRF24_read( uint8_t* buf, uint8_t len );
		//Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
		void NRF24_openWritingPipe(uint64_t address);
		//Open reading pipe
		void NRF24_openReadingPipe(uint8_t number, uint64_t address);
		//set transmit retries (rf24_Retries_e) and delay
		void NRF24_setRetries(uint8_t delay, uint8_t count);
		//Set RF channel frequency
		void NRF24_setChannel(uint8_t channel);
		//Set payload size
		void NRF24_setPayloadSize(uint8_t size);
		//Get payload size
		uint8_t NRF24_getPayloadSize(void);
		//Get dynamic payload size, of latest packet received
		uint8_t NRF24_getDynamicPayloadSize(void);
		//Enable payload on Ackknowledge packet
		void NRF24_enableAckPayload(void);
		//Enable dynamic payloads
		void NRF24_enableDynamicPayloads(void);
		//Disable dynamic payloads
		void NRF24_disableDynamicPayloads(void);
		//Check if module is NRF24L01+ or normal module
		bool NRF24_isNRF_Plus(void) ;
		//Set Auto Ack for all
		void NRF24_setAutoAck(bool enable);
		//Set Auto Ack for certain pipe
		void NRF24_setAutoAckPipe( uint8_t pipe, bool enable ) ;
		//Set transmit power level
		void NRF24_setPALevel( rf24_pa_dbm_e level ) ;
		//Get transmit power level
		rf24_pa_dbm_e NRF24_getPALevel( void ) ;
		//Set data rate (250 Kbps, 1Mbps, 2Mbps)
		bool NRF24_setDataRate(rf24_datarate_e speed);
		//Get data rate
		rf24_datarate_e NRF24_getDataRate( void );
		//Set crc length (disable, 8-bits or 16-bits)
		void NRF24_setCRCLength(rf24_crclength_e length);
		//Get CRC length
		rf24_crclength_e NRF24_getCRCLength(void);
		//Disable CRC
		void NRF24_disableCRC( void ) ;
		//power up
		void NRF24_powerUp(void) ;
		//power down
		void NRF24_powerDown(void);
		//Check if data are available and on which pipe (Use this for multiple rx pipes)
		bool NRF24_availablePipe(uint8_t* pipe_num);
		//Start write (for IRQ mode)
		void NRF24_startWrite( uint8_t* buf, uint8_t len );
		//Write acknowledge payload
		void NRF24_writeAckPayload(uint8_t pipe, uint8_t* buf, uint8_t len);
		//Check if an Ack payload is available
		bool NRF24_isAckPayloadAvailable(void);
		//Check interrupt flags
		void NRF24_whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready);
		//Test if there is a carrier on the previous listenning period (useful to check for intereference)
		bool NRF24_testCarrier(void);
		//Test if a signal carrier exists (=> -64dB), only for NRF24L01+
		bool NRF24_testRPD(void) ;
		//Reset Status
		void NRF24_resetStatus(void);
		//ACTIVATE cmd
		void NRF24_ACTIVATE_cmd(void);
		//Get AckPayload Size
		uint8_t NRF24_GetAckPayloadSize(void);
	
	private:
		//*** Library variables ***//
		uint64_t pipe0_reading_address;
		bool ack_payload_available; /**< Whether there is an ack payload waiting */
		uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
		uint8_t payload_size; /**< Fixed size of payloads */
		bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
		bool p_variant; /* False for RF24L01 and true for RF24L01P */
		bool wide_band; /* 2Mbs data rate in use? */

		//*** NRF24L01 pins and handles ***//
		//CE and CSN pins
		GPIO_TypeDef		*nrf24_PORTCSN;
		GPIO_TypeDef		*nrf24_PORTCE;
		uint16_t				nrf24_CSN_PIN;
		uint16_t				nrf24_CE_PIN;
		
		//SPI handle
		SPI_HandleTypeDef *nrf24_hspi;
	
		//**** Functions prototypes ****//
		//Microsecond delay function
		void NRF24_DelayMicroSeconds(uint32_t uSec);

		//Chip Select function
		void NRF24_csn(int mode);
		//Chip Enable
		void NRF24_ce(int level);
		//Read single byte from a register
		uint8_t NRF24_read_register(uint8_t reg);
		//Read multiple bytes register
		void NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len);
		//Write single byte register
		void NRF24_write_register(uint8_t reg, uint8_t value);
		//Write multipl bytes register
		void NRF24_write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len);	
		//Write transmit payload
		void NRF24_write_payload(uint8_t* buf, uint8_t len);
		//Read receive payload
		void NRF24_read_payload(uint8_t* buf, uint8_t len);
		//Flush Tx buffer
		void NRF24_flush_tx(void);
		//Flush Rx buffer
		void NRF24_flush_rx(void);
		//Get status register value
		uint8_t NRF24_get_status(void);
		
};
