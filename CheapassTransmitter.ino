/*  
A basic 4 channel transmitter using the nRF24L01 module.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeOut = 0xE8E8F0F0E1LL;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte dial;
  byte switches;
};

MyData data;

void resetData() 
{
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.dial = 0;
  data.switches = 0;
 
}

void setup()
{
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);

  radio.openWritingPipe(pipeOut);

  resetData();
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void loop()
{
  // The calibration numbers used here should be measured 
  // for your joysticks using the TestJoysticks sketch.
  data.throttle = mapJoystickValues( analogRead(0), 206, 535, 869, false );
  data.yaw      = mapJoystickValues( analogRead(1),  220, 552, 853, false );
  data.pitch    = mapJoystickValues( analogRead(2), 221, 586, 893, true );
  data.roll     = mapJoystickValues( analogRead(3), 181, 462, 763, true );
  
  data.dial     = constrain( map( analogRead(4), 900, 27, 0, 255 ), 0, 255);
  data.switches = 0;
  if ( ! digitalRead(2) ) data.switches |= 0x1;
  if ( ! digitalRead(4) ) data.switches |= 0x2;

  radio.write(&data, sizeof(MyData));
}



