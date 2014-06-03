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

#include <stm32f4xx.h>

#include <sbp.h>
#include <sbp_messages.h>

#include <tutorial_implementation.h>

/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t sbp_state;

/* Structs that messages from Piksi will feed. */
sbp_pos_llh_t      pos_llh;
sbp_baseline_ned_t baseline_ned;
sbp_vel_ned_t      vel_ned;
sbp_dops_t         dops;
sbp_gps_time_t     gps_time;

/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */
void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(sbp_pos_llh_t *)msg;
}
void sbp_baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  baseline_ned = *(sbp_baseline_ned_t *)msg;
}
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(sbp_vel_ned_t *)msg;
}
void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  dops = *(sbp_dops_t *)msg;
}
void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  gps_time = *(sbp_gps_time_t *)msg;
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

  /*
   * SBP callback nodes must be statically allocated. Each message ID / callback
   * pair must have a unique sbp_msg_callbacks_node_t associated with it.
   */
  static sbp_msg_callbacks_node_t gps_time_node;
  static sbp_msg_callbacks_node_t pos_llh_node;
  static sbp_msg_callbacks_node_t baseline_ned_node;
  static sbp_msg_callbacks_node_t vel_ned_node;
  static sbp_msg_callbacks_node_t dops_node;

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_GPS_TIME, &sbp_gps_time_callback,
                        NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_POS_LLH, &sbp_pos_llh_callback,
                        NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_BASELINE_NED, &sbp_baseline_ned_callback,
                        NULL, &baseline_ned_node);
  sbp_register_callback(&sbp_state, SBP_VEL_NED, &sbp_vel_ned_callback,
                        NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_DOPS, &sbp_dops_callback,
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

  while(1){

    /*
     * sbp_process must be called periodically in your
     * main program loop to consume the received bytes
     * from Piksi and parse the SBP messages.
     *
     * In this tutorial we use a FIFO to hold the data
     * before it is consumed by sbp_process; this helps ensure that no
     * data is lost or overwritten between calls to sbp_process.
     *
     * sbp_process must be passed a function that conforms to the definition
     *     u32 get_bytes(u8 *buff, u32 n, void *context);
     * that provides access to the bytes received from Piksi. See fifo_read and
     * related code in tutorial_implementation.c for a reference.
     */
    while (!fifo_empty()) {
      s8 ret = sbp_process(&sbp_state, &fifo_read);
      if (ret < 0)
        printf("sbp_process error.\n");
    }

    /* Print data from messages received from Piksi. */
    DO_EVERY(500000,

      printf("\n\n\n\n");

      /* Print GPS time. */
      printf("GPS Time:\n");
      printf("\tWeek\t\t: %6d\n", (int)gps_time.wn);
      sprintf(rj, "%6.2f", ((float)gps_time.tow)/1e3);
      printf("\tSeconds\t: %9s\n", rj);
      printf("\n");

      /* Print absolute position. */
      printf("Absolute Position:\n");
      sprintf(rj, "%4.10lf", pos_llh.lat);
      printf("\tLatitude\t: %17s\n", rj);
      sprintf(rj, "%4.10lf", pos_llh.lon);
      printf("\tLongitude\t: %17s\n", rj);
      sprintf(rj, "%4.10lf", pos_llh.height);
      printf("\tHeight\t: %17s\n", rj);
      printf("\tSatellites\t:     %02d\n", pos_llh.n_sats);
      printf("\n");

      /* Print NED (North/East/Down) baseline (position vector from base to rover). */
      printf("Baseline (mm):\n");
      printf("\tNorth\t\t: %6d\n", (int)baseline_ned.n);
      printf("\tEast\t\t: %6d\n", (int)baseline_ned.e);
      printf("\tDown\t\t: %6d\n", (int)baseline_ned.d);
      printf("\n");

      /* Print NED velocity. */
      printf("Velocity (mm/s):\n");
      printf("\tNorth\t\t: %6d\n", (int)vel_ned.n);
      printf("\tEast\t\t: %6d\n", (int)vel_ned.e);
      printf("\tDown\t\t: %6d\n", (int)vel_ned.d);
      printf("\n");

      /* Print Dilution of Precision metrics. */
      printf("Dilution of Precision:\n");
      sprintf(rj, "%4.2f", ((float)dops.gdop/100));
      printf("\tGDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.hdop/100));
      printf("\tHDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.pdop/100));
      printf("\tPDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.tdop/100));
      printf("\tTDOP\t\t: %7s\n", rj);
      sprintf(rj, "%4.2f", ((float)dops.vdop/100));
      printf("\tVDOP\t\t: %7s\n", rj);
      printf("\n");

      leds_toggle();
    );
  }

}
