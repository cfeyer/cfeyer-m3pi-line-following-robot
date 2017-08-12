// MIT License
// 
// Copyright (c) 2017 Chris Feyerchak
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "mbed.h"
#include "m3pi.h"

DigitalOut led1(LED1);

m3pi m3pi1;

// u == 0  ==>  steer straight
// u > 0   ==>  steer right
// u < 0   ==>  steer left
void steer( float u );

// main() runs in its own thread in the OS
int main() {
    m3pi1.cls();
    m3pi1.leds(1);
    wait(0.5);
    m3pi1.leds(3);
    m3pi1.sensor_auto_calibrate();
    m3pi1.leds(5);

     // setpoint; 0.0 = line centered
    const float r = 0.0f;

    // "leftness" of steering / "rightness" of line
    // control output aka plant output
    // y < 0   ==>  steered right of line
    // y > 0   ==>  steered left of line
    // y == 0  ==>  steered on line
    float y = 0.0f;

    // error = setpoint - plant output
    float e = 0.0f;

    // control input aka steering command
    float u = 0.0f;

    while (true) {
        led1 = !led1;
        y = m3pi1.line_position();
        e = r - y;
        u = e;
        steer( u );
    }
}

void steer( float u )
{
    const float straight_speed = 0.1f;

    // "Bang-bang" / "on-off" control logic

    // Steer left
    if( u < 0.0f )
    {
       m3pi1.left_motor( 0.0f );
       m3pi1.right_motor( straight_speed );
    }
    // Steer right
    else if( u > 0.0f )
    {
       m3pi1.left_motor( straight_speed );
       m3pi1.right_motor( 0.0f );
    }
    // Steer straight
    else
    {
       m3pi1.forward( straight_speed );
    }
}
