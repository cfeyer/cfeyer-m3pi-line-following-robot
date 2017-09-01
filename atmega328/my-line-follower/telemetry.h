#ifndef TELEMETRY_H
#define TELEMETRY_H

void telemetry_init();
void telemetry_send_frame_report( int frame_number,
                                  int line_detected,
                                  float line_position_left,
                                  float steering_command );

#endif /*TELEMETRY_H*/
