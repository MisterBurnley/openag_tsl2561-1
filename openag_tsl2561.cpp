/*
 * Digital_Light_Sensor.ino
 * A library for TSL2561
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : zhangkun
 * Create Time:
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "openag_tsl2561.h"

Tsl2561::Tsl2561(int i2c_address) {
  _i2c_address = i2c_address = TSL2561_Address;
  status_level = OK;
  status_msg = "";
}

void Tsl2561::begin(){
  // from original code
  Serial3.begin(9600);
  Serial3.println('Hi');
  Wire.begin();
  writeRegister(_i2c_address,TSL2561_Control,0x03);  // POWER UP
  writeRegister(_i2c_address,TSL2561_Timing,0x00);  //No High Gain (1x), integration time of 13ms
  writeRegister(_i2c_address,TSL2561_Interrupt,0x00);
  writeRegister(_i2c_address,TSL2561_Control,0x00);  // POWER Down
  // from dht22 example & sensor_tsl2561
  calibrtion_to_vernier_lux_ = 0.78;
  calibration_to_vernier_par_ = 0.02;
  measuring_indoor_par_correction_ = 0.86; //reduction by 14%
  read_register_timeout_ = 5; // milliseconds
  _time_of_last_query = 0;
}

void Tsl2561::update() {
  if (millis() - _time_of_last_query > _min_update_interval) {
    readSensorData();
   _time_of_last_query = millis();
  }
}


bool Tsl2561::get_light_illuminance(std_msgs::Float32 &msg) {
  msg.data = _light_illuminance;
  bool res = _light_illuminance;
  _send_light_illuminance = false;
  return res;
}

//.............................................. Private ..........................................//
void Tsl2561::readSensorData()
{
  writeRegister(_i2c_address,TSL2561_Control,0x03);  // POWER UP
  delay(14);
  float lux_average = 0;
  float samples = 40;
  int i;
  for (i=0; i<samples; i++){
    getLux();
    //if(ch1 == 0)
    //{ 
     //return;
    //}
    if(ch0/ch1 < 2 && ch0 > 4900)
    {
      lux_ = -1;
      return;  //ch0 out of range, but ch1 not. the lux is not valid in this situation.
    }
    lux_average += (float) calculateLux(0, 0, 0);
  }
  lux_average /= samples;
  lux_ = lux_average*calibrtion_to_vernier_lux_;
  par_ = lux_average*calibration_to_vernier_par_*measuring_indoor_par_correction_;
  writeRegister(_i2c_address,TSL2561_Control,0x00);  // POWER Down
}

uint8_t Tsl2561::readRegister(int deviceAddress, int address)
{
  uint8_t value;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address);                // register to read
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1); // read a byte
  uint32_t start_time = millis();
  while(!Wire.available()){
    if (millis() - start_time > read_register_timeout_) {
      read_register_error_ = 1;
      return 0;
    }
  }
  value = Wire.read();
  return value;
}

void Tsl2561::writeRegister(int deviceAddress, int address, uint8_t val)
{
  Wire.beginTransmission(deviceAddress);  // start transmission to device
  Wire.write(address);                    // send register address
  Wire.write(val);                        // send value to write
  Wire.endTransmission();                 // end transmission
  //delay(100);
}

void Tsl2561::getLux(void)
{
  CH0_LOW=readRegister(_i2c_address,TSL2561_Channal0L);
  CH0_HIGH=readRegister(_i2c_address,TSL2561_Channal0H);
  
  //read two bytes from registers 0x0E and 0x0F
  CH1_LOW=readRegister(_i2c_address,TSL2561_Channal1L);
  CH1_HIGH=readRegister(_i2c_address,TSL2561_Channal1H);
  
  ch0 = (CH0_HIGH<<8) | CH0_LOW;
  ch1 = (CH1_HIGH<<8) | CH1_LOW;
}

unsigned long Tsl2561::calculateLux(unsigned int iGain, unsigned int tInt,int iType)
{
 switch (tInt)
 {
  case 0:  // 13.7 msec
  chScale = CHSCALE_TINT0;
  break;
  case 1: // 101 msec
  chScale = CHSCALE_TINT1;
  break;
  default: // assume no scaling
  chScale = (1 << CH_SCALE);
  break;
}
if (!iGain)  chScale = chScale << 4; // scale 1X to 16X
// scale the channel values
channel0 = (ch0 * chScale) >> CH_SCALE;
channel1 = (ch1 * chScale) >> CH_SCALE;

  ratio1 = 0;
 if (channel0!= 0) ratio1 = (channel1 << (RATIO_SCALE+1))/channel0;
// round the ratio value
 unsigned long ratio = (ratio1 + 1) >> 1;

 switch (iType)
 {
 case 0: // T package
   if ((ratio >= 0) && (ratio <= K1T))
    {b=B1T; m=M1T;}
   else if (ratio <= K2T)
    {b=B2T; m=M2T;}
   else if (ratio <= K3T)
    {b=B3T; m=M3T;}
   else if (ratio <= K4T)
    {b=B4T; m=M4T;}
   else if (ratio <= K5T)
    {b=B5T; m=M5T;}
   else if (ratio <= K6T)
    {b=B6T; m=M6T;}
   else if (ratio <= K7T)
    {b=B7T; m=M7T;}
   else if (ratio > K8T)
    {b=B8T; m=M8T;}
 break;
  case 1:// CS package
   if ((ratio >= 0) && (ratio <= K1C))
    {b=B1C; m=M1C;}
   else if (ratio <= K2C)
    {b=B2C; m=M2C;}
  else if (ratio <= K3C)
   {b=B3C; m=M3C;}
  else if (ratio <= K4C)
   {b=B4C; m=M4C;}
  else if (ratio <= K5C)
   {b=B5C; m=M5C;}
  else if (ratio <= K6C)
   {b=B6C; m=M6C;}
  else if (ratio <= K7C)
    {b=B7C; m=M7C;}
 }
  temp=((channel0*b)-(channel1*m));
  if(temp<0) temp=0;
  temp+=(1<<(LUX_SCALE-1));
  // strip off fractional portion
  _send_light_illuminance = true;
  _light_illuminance = temp>>LUX_SCALE;
  return (_light_illuminance);
  
  Serial3.println(_light_illuminance);
 }
