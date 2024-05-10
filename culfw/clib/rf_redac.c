/*
 * Copyright by D.Tostmann
 * Inspired by code from TI.com AN
 * License: GPL v2
 */

#include <avr/io.h>                     // for bit_is_set, _BV
#include <stdint.h>                     // for uint8_t

#include "board.h"                      // for MBUS_NO_TX, CC1100_CS_DDR, etc
#include "led.h"                        // for CLEAR_BIT, SET_BIT
#ifdef HAS_REDAC
#include <avr/pgmspace.h>               // for PSTR
#include <string.h>                     // for NULL, memset

#include "cc1100.h"                     // for ccStrobe, cc1100_sendbyte, etc
#include "delay.h"                      // for my_delay_us, my_delay_ms
#include "display.h"                    // for DS_P, DH2, DNL, DC
#include "redac/3outof6.h"               // for DECODING_3OUTOF6_OK
#include "redac/redac_defs.h"             // for uint8, FALSE, TRUE
#include "redac/rf_settings.h"            // for rCFG
#include "redac/redac_packet.h"           // for REDAC_MODE, etc
#include "rf_redac.h"
#include "rf_receive.h"                 // for REP_RSSI
#include "stringfunc.h"                 // for fromhex
#include "rf_mode.h"
#include "multi_CC.h"

#ifdef USE_HAL
#include "hal.h"
#endif

// Buffers
uint8 RedacPacket[60];
uint8 RedacBytes[100];

// Radio Mode
#define RADIO_MODE_NONE  0
#define RADIO_MODE_TX    1
#define RADIO_MODE_RX    2

uint8   radio_mode = RADIO_MODE_NONE;
uint8_t  redac_mode = REDAC_NONE;
RXinfoDescr RXinfo;

static void halRfReadFifo(uint8* data, uint8 length, uint8 *rssi, uint8 *lqi) {
  CC1100_ASSERT;

  cc1100_sendbyte( CC1100_RXFIFO|CC1100_READ_BURST );
  for (uint8_t i = 0; i < length; i++)
    data[i] = cc1100_sendbyte( 0 );
	
  if (rssi) {
    *rssi = cc1100_sendbyte( 0 );
    if (lqi) {
      *lqi =  cc1100_sendbyte( 0 );
    }
  }
  CC1100_DEASSERT;
}

uint8_t halRfWriteFifo(const uint8* data, uint8 length) {
    
    CC1100_ASSERT;

    cc1100_sendbyte( CC1100_TXFIFO|CC1100_WRITE_BURST );
    for (uint8_t i = 0; i < length; i++)
      cc1100_sendbyte( data[i] );
    
    CC1100_DEASSERT;
    
    return 1;
}


static void halRfWriteReg( uint8_t reg, uint8_t value ) {
  cc1100_writeReg( reg, value );
}

uint8_t halRfGetTxStatus(void) {
  return(ccStrobe(CC1100_SNOP));
}

static uint8_t rf_redac_on(uint8_t force) {

  // already in RX?
  if (!force && (cc1100_readReg( CC1100_MARCSTATE ) == MARCSTATE_RX))
    return 0;

  // init RX here, each time we're idle
  RXinfo.state = 0;

  ccStrobe( CC1100_SIDLE );
  while((cc1100_readReg( CC1100_MARCSTATE ) != MARCSTATE_IDLE));
  ccStrobe( CC1100_SFTX  );
  ccStrobe( CC1100_SFRX  );

  // Initialize RX info variable
  RXinfo.lengthField = 0;           // Length Field in the wireless MBUS packet
  RXinfo.length      = 0;           // Total length of bytes to receive packet
  RXinfo.bytesLeft   = 0;           // Bytes left to to be read from the RX FIFO
  RXinfo.pByteIndex  = RedacBytes;     // Pointer to current position in the byte array
  RXinfo.format      = INFINITE;    // Infinite or fixed packet mode
  RXinfo.start       = TRUE;        // Sync or End of Packet
  RXinfo.complete    = FALSE;       // Packet Received
  RXinfo.mode        = redac_mode;   // Wireless MBUS mode

  // Set RX FIFO threshold to 4 bytes
  halRfWriteReg(CC1100_FIFOTHR, RX_FIFO_START_THRESHOLD);
  // Set infinite length 
  halRfWriteReg(CC1100_PKTCTRL0, INFINITE_PACKET_LENGTH);

  ccStrobe( CC1100_SRX   );
  while((cc1100_readReg( CC1100_MARCSTATE ) != MARCSTATE_RX));

  RXinfo.state = 1;

  return 1; // this will indicate we just have re-started RX
}

void rf_redac_init(uint8_t mmode, uint8_t rmode) {

  redac_mode  = REDAC_NONE;
  radio_mode = RADIO_MODE_NONE;

#ifdef USE_HAL
  hal_CC_GDO_init(CC_INSTANCE,INIT_MODE_IN_CS_IN);
  hal_enable_CC_GDOin_int(CC_INSTANCE,FALSE); // disable INT - we'll poll...

#else
  CLEAR_BIT( GDO0_DDR, GDO0_BIT );
  CLEAR_BIT( GDO2_DDR, GDO2_BIT );

  EIMSK &= ~_BV(CC1100_INT);                 // disable INT - we'll poll...
  SET_BIT( CC1100_CS_DDR, CC1100_CS_PIN );   // CS as output
#endif


  CC1100_DEASSERT;                           // Toggle chip select signal
  my_delay_us(30);
  CC1100_ASSERT;
  my_delay_us(30);
  CC1100_DEASSERT;
  my_delay_us(45);

  ccStrobe( CC1100_SRES );                   // Send SRES command
  my_delay_us(100);

  // load configuration
  switch (mmode) {
    case REDAC_MODE:
      for (uint8_t i = 0; i<200; i += 2) {
        if (rCFG(i)>0x40)
          break;
        cc1100_writeReg( rCFG(i), rCFG(i+1) );
      }
      break;
    default:
      return;
  }
  
  redac_mode  = mmode;
  radio_mode = rmode;

  ccStrobe( CC1100_SCAL );

  memset( &RXinfo, 0, sizeof( RXinfo ));

  my_delay_ms(4);
}

void rf_redac_task(void) {
  uint8 bytesDecoded[2];
  uint8 fixedLength;

  if (radio_mode != RADIO_MODE_RX)
    return;
  
  if (redac_mode == REDAC_NONE)
    return;

  switch (RXinfo.state) {
    case 0:
      rf_redac_on( TRUE );
      return;

     // RX active, awaiting SYNC
    case 1:
#ifdef USE_HAL
      if (hal_CC_Pin_Get(CC_INSTANCE,CC_Pin_In)) {
#else
      if (bit_is_set(GDO2_PIN,GDO2_BIT)) {
#endif
        RXinfo.state = 2;
      }
      break;

    // awaiting pkt len to read
    case 2:
#ifdef USE_HAL
      if (hal_CC_Pin_Get(CC_INSTANCE,CC_Pin_Out)) {
#else
      if (bit_is_set(GDO0_PIN,GDO0_BIT)) {
#endif
        // Read the 3 first bytes
        halRfReadFifo(RXinfo.pByteIndex, 3, NULL, NULL);

        // - Calculate the total number of bytes to receive -
        if (RXinfo.mode == REDAC_MODE) {
          // T-Mode
          // Possible improvment: Check the return value from the deocding function,
          // and abort RX if coding error. 
          if (decode3outof6(RXinfo.pByteIndex, bytesDecoded, 0) != DECODING_3OUTOF6_OK) {
            RXinfo.state = 0;
            return;
	  }		
          RXinfo.lengthField = bytesDecoded[0];
          RXinfo.length = byteSize(0, 0, (packetSize(RXinfo.lengthField)));
        }

	// check if incoming data will fit into buffer
	if (RXinfo.length>sizeof(RedacBytes)) {
          RXinfo.state = 0;
          return;
 	}

        // we got the length: now start setup chip to receive this much data
        // - Length mode -
        // Set fixed packet length mode is less than 256 bytes
        if (RXinfo.length < (MAX_FIXED_LENGTH)) {
          halRfWriteReg(CC1100_PKTLEN, (uint8)(RXinfo.length));
          halRfWriteReg(CC1100_PKTCTRL0, FIXED_PACKET_LENGTH);
          RXinfo.format = FIXED;
        }
      
        // Infinite packet length mode is more than 255 bytes
        // Calculate the PKTLEN value
        else {
          fixedLength = RXinfo.length  % (MAX_FIXED_LENGTH);
          halRfWriteReg(CC1100_PKTLEN, (uint8)(fixedLength)); 
        }
      
        RXinfo.pByteIndex += 3;
        RXinfo.bytesLeft   = RXinfo.length - 3;
      
        // Set RX FIFO threshold to 32 bytes
        RXinfo.start = FALSE;
        RXinfo.state = 3;
        halRfWriteReg(CC1100_FIFOTHR, RX_FIFO_THRESHOLD);
      }
      break;

    // awaiting more data to be read
    case 3:
#ifdef USE_HAL
      if (hal_CC_Pin_Get(CC_INSTANCE,CC_Pin_Out)) {
#else
      if (bit_is_set(GDO0_PIN,GDO0_BIT)) {
#endif
        // - Length mode -
        // Set fixed packet length mode is less than MAX_FIXED_LENGTH bytes
        if (((RXinfo.bytesLeft) < (MAX_FIXED_LENGTH )) && (RXinfo.format == INFINITE)) {
          halRfWriteReg(CC1100_PKTCTRL0, FIXED_PACKET_LENGTH);
          RXinfo.format = FIXED;
        }
  
        // Read out the RX FIFO
        // Do not empty the FIFO (See the CC110x or 2500 Errata Note)
        halRfReadFifo(RXinfo.pByteIndex, RX_AVAILABLE_FIFO - 1, NULL, NULL);

        RXinfo.bytesLeft  -= (RX_AVAILABLE_FIFO - 1);
        RXinfo.pByteIndex += (RX_AVAILABLE_FIFO - 1);

      }
      break;
  }

  // END OF PAKET
#ifdef USE_HAL
  if (!hal_CC_Pin_Get(CC_INSTANCE,CC_Pin_In) && RXinfo.state>1) {
#else
  if (!bit_is_set(GDO2_PIN,GDO2_BIT) && RXinfo.state>1) {
#endif
    uint8_t rssi = 0;
    uint8_t lqi = 0;
    halRfReadFifo(RXinfo.pByteIndex, (uint8)RXinfo.bytesLeft, &rssi, &lqi);
    RXinfo.complete = TRUE;

    // decode!
    uint16_t rxStatus = PACKET_CODING_ERROR;

    if (RXinfo.mode == REDAC_MODE)
      rxStatus = decodeRXBytesRedac(RedacBytes, RedacPacket, packetSize(RXinfo.lengthField));

    if (rxStatus == PACKET_OK) {

      MULTICC_PREFIX();
      DC( 'b' );

      for (uint8_t i=0; i < packetSize(RedacPacket[0]); i++) {
        DH2( RedacPacket[i] );
//	DC( ' ' );
      }

      if (TX_REPORT & REP_RSSI) {
        DH2(lqi);	
        DH2(rssi);
      }
      DNL();
    }
    RXinfo.state = 0;
    return;
  }

  rf_redac_on( FALSE );
}

static void redac_status(void) {
  if (radio_mode == RADIO_MODE_RX ) {
    switch (redac_mode) {
    case REDAC_MODE:
      MULTICC_PREFIX();
      DS_P(PSTR("REDAC"));
      break;
    default:
      MULTICC_PREFIX();
      DS_P(PSTR("OFF"));
    }
  }
  else {
    MULTICC_PREFIX();
    DS_P(PSTR("OFF"));
  }
  DNL();
}

void rf_redac_func(char *in) {
  if((in[1] == 'r') && in[2]) {     // Reception on
#ifdef USE_RF_MODE
    if(in[2] == 'r') {
      set_RF_mode(RF_mode_REDAC_S);
    } else {                        // Off
      set_RF_mode(RF_mode_off);
    }
#else
    if(in[2] == 'r') {
      rf_redac_init(REDAC_MODE,RADIO_MODE_RX);
    } else {                        // Off
      rf_redac_init(REDAC_NONE,RADIO_MODE_NONE);
    }	
#endif
  }

  redac_status();
}

#endif
