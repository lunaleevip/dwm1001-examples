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
* @attention
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
#include "ss_tag_main.h"

#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Frames used in the ranging process. See NOTE 1,2 below. */
//41 88 37 CA DE FF FF 35 09 91 2F F4 C3
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x35, 0x09, 0x91, 0, 0};
//41 88 09 CA DE 35 09 2D C1 93 2F 95 3E 92 04 B4 C4 80 C8
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x35, 0x09, 0x2D, 0xC1, 0x93, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
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
static uint32 g_poll_tx_ts = 0;
final_msg_xexun_t g_final_msg = {0};

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

/*Interrupt flag*/
static volatile int tx_int_flag = 0 ; // Transmit success interrupt flag
static volatile int rx_int_flag = 0 ; // Receive success interrupt flag
static volatile int to_int_flag = 0 ; // Timeout interrupt flag
static volatile int er_int_flag = 0 ; // Error interrupt flag 

/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter 

uint16_t g_pan_id = 0xDECA;    //ÀàËÆÐ¡×éID
uint8_t  g_rNum = 0;
//uint8_t  g_role = TAG_MODE;
uint8_t  g_resp_count = 0;

static uwb_pckt_t tx_msg = {0};
static uwb_pckt_t rx_msg = {0};

static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

// convert microseconds to device time
uint64 convertmicrosectodevicetimeu (double microsecu)
{
    uint64 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64) (dtime) ;

    return dt;
}

double convertdevicetimetosec(int32 dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

char *Bytetohex(char *in_buf, int16_t len)
{
    static char *buf = NULL;
    char tmp[4];

    if(buf)
    {
        free(buf);
        buf = NULL;
    }

    if(len < 0)
    {
        return NULL;
    }

    buf = calloc((len*3+1), 1);
    if(!buf)
        return NULL;
    
    //strcpy(buf, "");
    if(len)
    {
        for(int i = 0; i < len; i++)
        {
        	if (i==len-1)
        	{
        		sprintf(tmp, "%02X", in_buf[i]);
        	}
        	else
        	{
            	sprintf(tmp, "%02X ", in_buf[i]);
            }
            strcat(buf, tmp);
        }
    }
    return buf;
}


static error_e
tx_start(uwb_pckt_t * pTxPckt)
{
    error_e ret = _NO_ERR;
    uint8_t  txFlag = 0;

    //printf("%s|%d\n", __FUNCTION__, __LINE__);    
    dwt_forcetrxoff();    //Stop the Receiver and Write Control and Data

#ifdef SAFE_TXDATA
    dwt_writetxdata(pTxPckt->psduLen, (uint8_t *) &pTxPckt->msg.stdMsg, 0);
#endif

    dwt_writetxfctrl(pTxPckt->psduLen, 0, 1);

    //Setup for delayed Transmit
    if(pTxPckt->delayedTxTimeH_sy != 0UL)
    {
        dwt_setdelayedtrxtime(pTxPckt->delayedTxTimeH_sy) ;
    }

    if(pTxPckt->Flag & DWT_RESPONSE_EXPECTED)
    {
        dwt_setrxaftertxdelay(pTxPckt->delayedRxTime_sy);
        dwt_setrxtimeout(pTxPckt->delayedRxTimeout_sy);
    }

    // Begin delayed TX of frame
    txFlag = (pTxPckt->delayedTxTimeH_sy != 0UL) | (pTxPckt->Flag);

    if(dwt_starttx(txFlag) != DWT_SUCCESS)
    {
        ret = _Err_DelayedTX_Late;

    }
    else
    {
        //m_dw1000_tx_sign_light = 5;
    }

#ifndef SAFE_TXDATA
    /* while transmitting of the preamble we can fill the data path
     * to save time : this is not "safe" approach and
     * additional check to be done to use this feature.
     * */
    if (ret == _NO_ERR)
    {
        dwt_writetxdata(pTxPckt->psduLen, (uint8_t *)  &pTxPckt->msg.stdMsg, 0);
    }
#endif

    if(1)
    {
        printf("Send: Len: %d, RAW: %s, ret %d\n", pTxPckt->psduLen, Bytetohex((char *) &pTxPckt->msg,pTxPckt->psduLen), ret);
    }
    
    return (ret);
}

void Tag_poll(void)
{
    poll_xexun_msg_t *poll_msg = &tx_msg.msg.pollXexunMsg;
    final_msg_xexun_t * final_msg = &g_final_msg;
    g_resp_count = 0;

    //printf("%s|%d\n", __FUNCTION__, __LINE__);    
    poll_msg->mac.frameCtrl[0] = Head_Msg_STD;
    poll_msg->mac.frameCtrl[1] = Frame_Ctrl_SS;
    poll_msg->mac.seqNum = (++frame_seq_nb);
    
    poll_msg->mac.panID[0] = g_pan_id & 0xff;
    poll_msg->mac.panID[1] = g_pan_id >> 8 & 0xff;
    poll_msg->mac.sourceAddr[0] = dwt_getpartid() & 0xff;
    poll_msg->mac.sourceAddr[1] = dwt_getpartid() >> 8 & 0xff;
    
    poll_msg->mac.destAddr[0] = 0xff;
    poll_msg->mac.destAddr[1] = 0xff;

    poll_msg->poll.fCode = Twr_Fcode_Tag_Poll_Xexun;
    //poll_msg->poll.id_ext[0] = dwt_getpartid() >> 16 & 0xff;
    //poll_msg->poll.id_ext[1] = dwt_getpartid() >> 24 & 0xff;
    poll_msg->poll.rNum = (++g_rNum);
    tx_msg.psduLen = sizeof(poll_xexun_msg_t);
    tx_msg.Flag = DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED;
    tx_msg.delayedTxTimeH_sy = 0;
    tx_msg.delayedRxTime_sy = 100;
    tx_msg.delayedRxTimeout_sy = 2000*(4-g_resp_count);
    
    memset(final_msg, 0, sizeof(final_msg_xexun_t));

    final_msg->mac.frameCtrl[0] = Head_Msg_STD;
    final_msg->mac.frameCtrl[1] = Frame_Ctrl_SS;

    final_msg->mac.panID[0] = g_pan_id & 0xff;
    final_msg->mac.panID[1] = g_pan_id >> 8 & 0xff;
    final_msg->mac.sourceAddr[0] = dwt_getpartid() & 0xff;
    final_msg->mac.sourceAddr[1] = dwt_getpartid() >> 8 & 0xff;
    
    final_msg->mac.destAddr[0] = 0xff;
    final_msg->mac.destAddr[1] = 0xff;

    final_msg->final.rNum = poll_msg->poll.rNum;
    final_msg->final.fCode = Twr_Fcode_Tag_Final_Xexun;
    final_msg->final.id_ext[0] = dwt_getpartid() >> 16 & 0xff;
    final_msg->final.id_ext[1] = dwt_getpartid() >> 24 & 0xff;

    printf("%s|%d, rNum: %3d\n", __FUNCTION__, __LINE__, poll_msg->poll.rNum);    
    tx_start(&tx_msg);
}

void Final_send(void)
{
    final_msg_xexun_t * final_msg = &g_final_msg;
    uint8_t resp_count = g_resp_count;
    
    final_msg->mac.seqNum = (++frame_seq_nb);

    memcpy(&tx_msg.msg.finalXexunMsg, final_msg, sizeof(final_msg_xexun_t));
    tx_msg.psduLen = sizeof(final_msg_xexun_t);
    tx_msg.Flag = DWT_START_TX_IMMEDIATE;
    tx_msg.delayedTxTimeH_sy = 0;//resp_tx_time;
    g_resp_count = 0;
    
    tx_start(&tx_msg);
    printf("%s|%d, rNum: %3d, nResp: %d\n", __FUNCTION__, __LINE__, final_msg->final.rNum, resp_count);    
}

#pragma GCC optimize ("O3")
void Resp_Recved(resp_xexun_msg_t *resp_msg)
{
    uint32 resp_rx_ts, poll_rx_ts, resp_tx_ts, resp_tx_time = 0;
    int32 rtd_init, rtd_resp;
    float clockOffsetRatio;
    uint32_t anc_id = 0;
    int ret = 0;
    int32 carrier;
    final_msg_xexun_t * final_msg = &g_final_msg;

    //printf("%s|%d, poll_rx_delay %d\n", __FUNCTION__, __LINE__, poll_rx_delay);    
    anc_id = resp_msg->mac.sourceAddr[0] | resp_msg->mac.sourceAddr[1] << 8 | resp_msg->resp.id_ext[0] << 16 | resp_msg->resp.id_ext[1] << 24;
    /* Retrieve poll transmission and response reception timestamps. See NOTE 4 below. */
    //poll_tx_ts = dwt_readtxtimestamplo32();

    resp_rx_ts = dwt_readrxtimestamplo32();

    carrier = dwt_readcarrierintegrator();

    if(carrier == 0)
        carrier = 4000;

    /* Read carrier integrator value and calculate clock offset ratio. See NOTE 6 below. */
    clockOffsetRatio = carrier * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

    /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
    rtd_init = resp_rx_ts - g_poll_tx_ts;

    rtd_resp = resp_msg->resp.slotCorr_us;

    tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
    distance = tof * SPEED_OF_LIGHT;
    //memset(&rx_msg, 0, sizeof(rx_msg));

    switch(g_resp_count)
    {
        case 1:
            final_msg->final.anchor_id0 = anc_id;
            final_msg->final.distance0 = (float)distance;
            break;
        case 2:
            final_msg->final.anchor_id1 = anc_id;
            final_msg->final.distance1 = (float)distance;
            break;
        case 3:
            final_msg->final.anchor_id2 = anc_id;
            final_msg->final.distance2 = (float)distance;
            break;
        case 4:
            final_msg->final.anchor_id3 = anc_id;
            final_msg->final.distance3 = (float)distance;
            Final_send();
            break;
    }

    printf("Anc_ID: %08X, Distance : %f, carrier %d, offset %e, RAW %s\r\n",anc_id, distance,carrier, clockOffsetRatio, Bytetohex((char *) resp_msg, sizeof(resp_xexun_msg_t)));
    //printf("%s|%d, Twr_Fcode_Tag_Final_Xexun\n", __FUNCTION__, __LINE__);    

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

  /* Loop forever initiating ranging exchanges. */


  /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /*Waiting for transmission success flag*/
  while (!(tx_int_flag))
  {};

  if (tx_int_flag)
  {
    tx_count++;
    printf("Transmission # : %d\r\n",tx_count);

    /*Reseting tx interrupt flag*/
    tx_int_flag = 0 ;
  }

  /* Wait for reception, timeout or error interrupt flag*/
  while (!(rx_int_flag || to_int_flag|| er_int_flag))
  {};

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (rx_int_flag)
  {		
    uint32 frame_len;

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the companion "SS TWR responder" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer+3, rx_resp_msg+3, ALL_MSG_COMMON_LEN-6) == 0)
    {	
      rx_count++;
      printf("Reception # : %d\r\n",rx_count);
      float reception_rate = (float) rx_count / (float) tx_count * 100;
      printf("Reception rate # : %f\r\n",reception_rate);
      uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
      int32 rtd_init, rtd_resp;
      float clockOffsetRatio ;

      /* Retrieve poll transmission and response reception timestamps. See NOTE 4 below. */
      poll_tx_ts = dwt_readtxtimestamplo32();
      resp_rx_ts = dwt_readrxtimestamplo32();

      /* Read carrier integrator value and calculate clock offset ratio. See NOTE 6 below. */
      clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

      /* Get timestamps embedded in response message. */
      //resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
      //resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

      /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
      rtd_init = resp_rx_ts - poll_tx_ts;
      //rtd_resp = resp_tx_ts - poll_rx_ts;
      memcpy(&rtd_resp, &rx_buffer[11], 4);

      tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
      distance = tof * SPEED_OF_LIGHT;
      printf("Distance : %f, clockOffsetRatio %e\r\n",distance, clockOffsetRatio);

      /*Reseting receive interrupt flag*/
      rx_int_flag = 0; 
    }
   }

  if (to_int_flag || er_int_flag)
  {
    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();

    /*Reseting interrupt flag*/
    to_int_flag = 0 ;
    er_int_flag = 0 ;
  }

    /* Execute a delay between ranging exchanges. */
    //     deca_sleep(RNG_DELAY_MS);
    //	return(1);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_ok_cb()
*
* @brief Callback to process RX good frame events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  rx_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #1 */
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_to_cb()
*
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  to_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #2 */
  printf("TimeOut\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_err_cb()
*
* @brief Callback to process RX error events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  er_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #3 */
  printf("Transmission Error : may receive package from different UWB device\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn tx_conf_cb()
*
* @brief Callback to process TX confirmation events
*
* @param  cb_data  callback data
*
* @return  none
*/
void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
  * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
  * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
  * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
  * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

  tx_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #4 */
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
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
  *ts += ts_field[i] << (i * 8);
  }
}


/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

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
* 4. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 6. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
