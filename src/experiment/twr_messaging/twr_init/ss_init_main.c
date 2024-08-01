/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention: Modified for RFClock experiments
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "nrf_drv_gpiote.h"

#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 100

/* Frames used in the ranging process. See NOTE 1,2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 15
#define RESP_MSG_TS_LEN 5
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint64 *ts);
static TaskHandle_t  task_handle; 


/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter
 
 typedef unsigned long long uint64;
 //static uint64 tx_ts;
 static uint64 get_tx_timestamp_u64(void);

static uint32 INTERRUPTsystem_time_counter;  
static int timeout=0; 

void pin_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){

//  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
 // nrf_delay_us(100);
  //uint8 txfrs= dwt_read8bitoffsetreg(SYS_STATUS_ID, 0);
  INTERRUPTsystem_time_counter = dwt_readsystimestamphi32();
  //printf("INTERRUPTsystem_time_counter: %u\r\n", INTERRUPTsystem_time_counter);

 // clear ext sync bit in DW
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ESYNCR);
  nrf_delay_us(50);
  timeout=1;

        //nrf_drv_gpiote_in_event_enable(30, false);
        //TaskHandle_t task_handle;
        //printf('task handle: %u', task_handle);
        //vTaskNotifyGiveFromISR(task_handle, pdTRUE);
        //vTaskDelay(pdMS_TO_TICKS(100));
}
/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/
int ss_init_run(void)
{
  uint32_t ec_ctrl_val = 0;

  ec_ctrl_val = EC_CTRL_OSTRM  | EC_CTRL_WAIT_MASK | EC_CTRL_PLLLCK; //| (1U << 3);
  //ec_ctrl_val= EC_CTRL_OSTSM | EC_CTRL_PLLLCK | EC_CTRL_WAIT_MASK;
  //ec_ctrl_val = EC_CTRL_OSTRM | EC_CTRL_OSTSM | EC_CTRL_WAIT_MASK | EC_CTRL_PLLLCK;
  dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);

  uint32 TASKsystem_time_counter;
  TASKsystem_time_counter = dwt_readsystimestamphi32();
  printf("TASKsystem_time_counter: %u\r\n", TASKsystem_time_counter);


  /* Loop forever initiating ranging exchanges. */


  /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  uint8 txfrs= dwt_read8bitoffsetreg(SYS_STATUS_ID, 0);
  printf("txfrs after writing: %x\r\n", txfrs);

  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */

 // dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  //task_handle= xTaskGetCurrentTaskHandle();
 //printf('task handle: %u', task_handle);
  //ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000));
  //nrf_delay_ms(10);

/*while(1){
uint8 a= dwt_read8bitoffsetreg(SYS_STATUS_ID, 0) & 0xf2;
  if (((dwt_read8bitoffsetreg(SYS_STATUS_ID, 0)& 0xf0) & 0xf0)==0xf0){
    break;
  }
  }*/

  while(1){
    if(timeout==1){
      timeout=0;
      break;
      }
  }
  

  //txfrs= dwt_read8bitoffsetreg(SYS_STATUS_ID, 0);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  tx_count++;
  printf(" Transmission # : %d\r\n", tx_count);

  //uint64 tx_ts=  get_tx_timestamp_u64();
  uint32 tX_timestamp_L= dwt_readtxtimestamplo32();
  uint32 tX_timestamp_H = dwt_readtxtimestamphi32();
  uint32 raw_TX_timestamp= dwt_read32bitoffsetreg(TX_TIME_ID, 6);

  printf("ttimestamp_LO32: %u\r\n", tX_timestamp_L); 
  //printf("ttimestamp_HI32: %x\r\n", tX_timestamp_H); 
  printf("raw_timestamp: %u\r\n\n", raw_TX_timestamp);
  //printf("TS_40bit: %x\r\n\n", tx_ts);




  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 4 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {};

    #if 0  // include if required to help debug timeouts.
    int temp = 0;		
    if(status_reg & SYS_STATUS_RXFCG )
    temp =1;
    else if(status_reg & SYS_STATUS_ALL_RX_TO )
    temp =2;
    if(status_reg & SYS_STATUS_ALL_RX_ERR )
    temp =3;
    #endif

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG)
  {		
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the companion "SS TWR responder" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
    {	
      rx_count++;
      printf("Reception # : %d\r\n",rx_count);
      uint32 resp_rx_ts_L, resp_rx_ts_H, poll_tx_ts_L, poll_tx_ts_H;
      uint64 resp_rx_ts, poll_rx_ts, resp_tx_ts, poll_tx_ts, poll_tx_ts_64;
      uint32 resp_rx_ts_raw;
      int64 rtd_init, rtd_resp;
      float phase_offset;
      double clockOffsetRatio ;

      /* Retrieve poll transmission and response reception timestamps. See NOTE 5 below. */
      poll_tx_ts_L = dwt_readtxtimestamplo32();
      poll_tx_ts_H = dwt_readtxtimestamphi32();
      poll_tx_ts =  ((poll_tx_ts_L) | (((uint64)poll_tx_ts_H) << 8));

      poll_tx_ts_64 = ((uint64)poll_tx_ts);
      //resp_rx_ts = dwt_readrxtimestamplo32();
      resp_rx_ts_L = dwt_readrxtimestamplo32();
      resp_rx_ts_H = dwt_readrxtimestamphi32();
      uint64 a= ((uint64)resp_rx_ts_H) << 8;
      resp_rx_ts =  ((resp_rx_ts_L) | (((uint64)resp_rx_ts_H) << 8));
      uint32 raw_RX_timestamp= dwt_read32bitoffsetreg(RX_TIME_ID, 10);


      //resp_rx_ts_raw= dwt_read32bitoffsetreg(RX_TIME_ID, RX_TIME_FP_RAWST_OFFSET+1);
      

      /* Read carrier integrator value and calculate clock offset ratio. See NOTE 7 below. */
      clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6) ;

      /* Get timestamps embedded in response message. */
      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX+1], &poll_rx_ts);
      poll_rx_ts=   (poll_rx_ts << 8) | rx_buffer[10];
      resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX+1], &resp_tx_ts);
      resp_tx_ts=   (resp_tx_ts << 8) | rx_buffer[15];

      /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
      //rtd_init = resp_rx_ts_raw - raw_TX_timestamp;
      rtd_init = resp_rx_ts - poll_tx_ts;
      rtd_resp = resp_tx_ts - poll_rx_ts;

      printf(" anch_poll_txed_ts  %u\r\n",poll_tx_ts_64);
      printf(" tag_poll_rxed_ts  %u\r\n",poll_rx_ts);
      printf(" tag_resp_txed_ts  %u\r\n",resp_tx_ts);
      printf(" anch_resp_rxed_ts  %u\r\n",resp_rx_ts);

      printf("rtd_init : %u\r\n", rtd_init);
      printf("rtd_resp : %u\r\n", rtd_resp);

      tof = ((rtd_init - rtd_resp) / 2.0f) * 15.625e-12; // Specifying 1.0f and 2.0f are floats to clear warning 
      distance = tof * SPEED_OF_LIGHT;

     // float a= ((resp_rx_ts- resp_tx_ts)- (poll_rx_ts-poll_tx_ts))*15.625e-12;
      phase_offset= (((resp_rx_ts- resp_tx_ts)+ (poll_tx_ts- poll_rx_ts))*15.625e-12)/2.0f;
      
      printf("Distance : %lf\r\n",distance);
      printf("TOF : %e\r\n",tof);
      //printf("a : %f\r\n", a);
      printf("Phase_offset : %e\r\n",phase_offset);
      printf("clockOffsetRatio : %.12lf\r\n\n",clockOffsetRatio);
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }

  /* Execute a delay between ranging exchanges. */
  //     deca_sleep(RNG_DELAY_MS);

  //	return(1);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_get_ts(uint8 *ts_field, uint64 *ts)
{
  int i;
  *ts = 0x0000000000000000;
  for (i = 0; i < 4; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}

static uint64 get_tx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}


/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dwt_setleds(DWT_LEDS_ENABLE);

  while (true)
  {
    ss_init_run();
    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}
/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 5. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 6. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 7. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
