/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <string.h>
#include <stm32f4xx.h>
#include <semihosting.h>
#include <libsbp/sbp.h>
#include <libsbp/navigation.h>
#include <tutorial_implementation.h>

/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t sbp_state;

/* SBP structs that messages from Piksi will feed. */
msg_pos_llh_t      pos_llh;
msg_baseline_ned_t baseline_ned;
msg_vel_ned_t      vel_ned;
msg_dops_t         dops;
msg_gps_time_t     gps_time;

/*
 * SBP callback nodes must be statically allocated. Each message ID / callback
 * pair must have a unique sbp_msg_callbacks_node_t associated with it.
 */
sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t baseline_ned_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;

/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */
void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}
void sbp_baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  baseline_ned = *(msg_baseline_ned_t *)msg;
}
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
}
void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  dops = *(msg_dops_t *)msg;
}
void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  gps_time = *(msg_gps_time_t *)msg;
}

/*
 * Set up SwiftNav Binary Protocol (SBP) nodes; the sbp_process function will
 * search through these to find the callback for a particular message ID.
 *
 * Example: sbp_pos_llh_callback is registered with sbp_state, and is associated
 * with both a unique sbp_msg_callbacks_node_t and the message ID SBP_POS_LLH.
 * When a valid SBP message with the ID SBP_POS_LLH comes through the UART, written
 * to the FIFO, and then parsed by sbp_process, sbp_pos_llh_callback is called
 * with the data carried by that message.
 */
void sbp_setup(void)
{
  /* SBP parser state must be initialized before sbp_process is called. */
  sbp_state_init(&sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback,
                        NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback,
                        NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_MSG_BASELINE_NED, &sbp_baseline_ned_callback,
                        NULL, &baseline_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback,
                        NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_DOPS, &sbp_dops_callback,
                        NULL, &dops_node);
}

int main(void){

  /* Set unbuffered mode for stdout (newlib) */
  setvbuf(stdout, 0, _IONBF, 0);

  leds_setup();
  usarts_setup();
  sbp_setup();

  /* Use sprintf to right justify floating point prints. */
  char rj[30];
  /* Only want 1 call to SH_SendString as semihosting is quite slow.
   * sprintf everything to this array and then print using array. */
  char str[1000];
  int str_i;

  while(1){

    /*
     * sbp_process must be called periodically in your
     * main program loop to consume the received bytes
     * from Piksi and parse the SBP messages from them.
     *
     * In this tutorial we use a FIFO structure to hold the data
     * before it is consumed by sbp_process; this helps ensure that no
     * data is lost or overwritten between calls to sbp_process. See
     * tutorial_implementation.c for the interaction between the USART
     * and the FIFO.
     *
     * sbp_process must be passed a function that conforms to the definition
     *     u32 get_bytes(u8 *buff, u32 n, void *context);
     * that provides access to the bytes received from Piksi. See fifo_read and
     * related code in tutorial_implementation.c for a reference.
     */
    s8 ret = sbp_process(&sbp_state, &fifo_read);
    /* Semihosting is slow - each loop the FIFO fills up and packets get
     * dropped, so we don't check the return value from sbp_process. It's a good
     * idea to incorporate this check into your host's code, though. */
    //if (ret < 0)
    //  printf("sbp_process error: %d\n", (int)ret);

    /* Print data from messages received from Piksi. */
    DO_EVERY(10000,

      str_i = 0;
      memset(str, 0, sizeof(str));

      str_i += sprintf(str + str_i, "\n\n\n\n");

      /* Print GPS time. */
      str_i += sprintf(str + str_i, "GPS Time:\n");
      str_i += sprintf(str + str_i, "\tWeek\t\t: %6d\n", (int)gps_time.wn);
      sprintf(rj, "%6.2f", ((float)gps_time.tow)/1e3);
      str_i += sprintf(str + str_i, "\tSeconds\t: %9s\n", rj);
      str_i += sprintf(str + str_i, "\n");

      /* Print absolute position. */
      str_i += sprintf(str + str_i, "Absolute Position:\n");
      sprintf(rj, "%4.10lf", pos_llh.lat);
      str_i += sprintf(str + str_i, "\tLatitude\t: %17s\n", rj);
      sprintf(rj, "%4.10lf", pos_llh.lon);
      str_i += sprintf(str + str_i, "\tLongitude\t: %17s\n", rj);
      sprintf(rj, "%4.10lf", pos_llh.height);
      str_i += sprintf(str + str_i, "\tHeight\t: %17s\n", rj);
      str_i += sprintf(str + str_i, "\tSatellites\t:     %02d\n", pos_llh.n_sats);
      str_i += sprintf(str + str_i, "\n");

      /* Print NED (North/East/Down) baseline (position vector from base to rover). */
      str_i += sprintf(str + str_i, "Baseline (mm):\n");
      str_i += sprintf(str + str_i, "\tNorth\t\t: %6d\n", (int)baseline_ned.n);
      str_i += sprintf(str + str_i, "\tEast\t\t: %6d\n", (int)baseline_ned.e);
      str_i += sprintf(str + str_i, "\tDown\t\t: %6d\n", (int)baseline_ned.d);
      str_i += sprintf(str + str_i, "\n");

      /* Print NED velocity. */
      str_i += sprintf(str + str_i, "Velocity (mm/s):\n");
      str_i += sprintf(str + str_i, "\tNorth\t\t: %6d\n", (int)vel_ned.n);
      str_i += sprintf(str + str_i, "\tEast\t\t: %6d\n", (int)vel_ned.e);
      str_i += sprintf(str + str_i, "\tDown\t\t: %6d\n", (int)vel_ned.d);
      str_i += sprintf(str + str_i, "\n");

      /* Print Dilution of Precision metrics. */
      str_i += sprintf(str + str_i, "Dilution of Precision:\n");
      sprintf(rj, "%4.2f", ((float)dops.gdop/100));
      str_i += sprintf(str + str_i, "\tGDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.hdop/100));
      str_i += sprintf(str + str_i, "\tHDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.pdop/100));
      str_i += sprintf(str + str_i, "\tPDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.tdop/100));
      str_i += sprintf(str + str_i, "\tTDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.vdop/100));
      str_i += sprintf(str + str_i, "\tVDOP\t\t: %7s\n", rj);
      str_i += sprintf(str + str_i, "\n");

      SH_SendString(str);
    );
  }
}
