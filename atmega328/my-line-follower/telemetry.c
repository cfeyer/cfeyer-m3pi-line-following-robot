#include "telemetry.h"

#include <pololu/3pi.h>
#include <stdio.h>
#include <string.h>

static char tx_str_buffer[100];

static int min( int x, int y )
{
   if( x < y )
   {
      return x;
   }
   else
   {
      return y;
   }
}

void telemetry_init()
{
   set_digital_input(IO_D0, PULL_UP_ENABLED);
   serial_set_baud_rate(115200);
   serial_send_blocking( "Hello\n", sizeof("Hello\n") );
}


void telemetry_send_frame_report( int frame_number,
                                  int line_detected,
                                  float line_position,
                                  float steering_command )
{
   int line_position_int = (int)(1000.0f * line_position);
   int steering_command_int = (int)(1000.0 * steering_command);

   snprintf( tx_str_buffer,
             100,
             "f, %d, d, %d, p, %d, s, %d\n",
             frame_number,
             line_detected,
             line_position_int,
             steering_command_int );
   serial_send_blocking( tx_str_buffer, min(100, strlen(tx_str_buffer)) );
}
