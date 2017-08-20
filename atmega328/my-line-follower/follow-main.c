#include "follow-main.h"

#include <pololu/3pi.h>
#include <avr/pgmspace.h>

#include "follow-controller-constants.h"

static const char back_line2[] PROGMEM = "\6B";

static void auto_calibrate() // Borrow from Pololu's serial-slave.c
{
        time_reset();
        set_motors(60, -60);
        while(get_ms() < 250)
                calibrate_line_sensors(IR_EMITTERS_ON);
        set_motors(-60, 60);
        while(get_ms() < 750)
                calibrate_line_sensors(IR_EMITTERS_ON);
        set_motors(60, -60);
        while(get_ms() < 1000)
                calibrate_line_sensors(IR_EMITTERS_ON);
        set_motors(0, 0);
}

float sense_line_position_left()
{
   unsigned int sensor_readings[5] = {0};
   int line_position = read_line( sensor_readings, IR_EMITTERS_ON );
   return ((float)line_position - 2048.0f)/(2048.0f);
}


float bound( float x, float x_min, float x_max )
{
   float y = 0.0;

   if( x >= x_min )
   {   
      if( x <= x_max )
      {   
         y = x;
      }   
      else
      {   
         y = x_max;
      }   
   }   
   else
   {   
      y = x_min;
   }   

   return y;
}


void actuate_steer_command_left( float steer_command_left )
{
    const float straight_speed = 0.1f;

    float left_speed = bound( ((1.0f - steer_command_left) * straight_speed), 0.0f, 1.0f );
    float right_speed = bound( ((1.0f + steer_command_left) * straight_speed), 0.0f, 1.0f );

    set_motors( (int)(left_speed * 255.0f),
                (int)(right_speed * 255.0f) );
}

void follow_main()
{
   clear();

   lcd_goto_xy(0,1);
   print_from_program_space(back_line2);

   pololu_3pi_init( 1000 );

   time_reset();
   while(get_ms() < 1000);
   auto_calibrate();

   const float steer_setpoint = 0.0f;
   float line_position_left = 0.0f;
   float steer_error = 0.0f;
   float steer_error_cumulative = 0.0f;
   float steer_error_previous = 0.0f;
   float steer_error_delta = 0.0f;
   float steer_command_left = 0.0f;

   while( !button_is_pressed(BUTTON_B) )
   {
      line_position_left = sense_line_position_left();

      steer_error = steer_setpoint - line_position_left;
      steer_error_cumulative += steer_error;
      steer_error_delta = steer_error - steer_error_previous;

      steer_command_left = feedback_sign *
                                   ( kp * steer_error
                                   + ki * steer_error_cumulative
                                   + kd * steer_error_delta );

      actuate_steer_command_left( steer_command_left );

      steer_error_previous = steer_error;
   } 
}

