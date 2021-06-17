#include <Wire.h>   
#include <EEPROM.h> 

//PID gain and limit settings
float pidPGainRoll = 1.3, pidIGainRoll = 0.04, pidDGainRoll = 18.0;
int pidMaxRoll = 400;
float pidPGainPitch = pidPGainRoll, pidIGainPitch = pidIGainRoll, pidDGainPitchhh = pidDGainRoll; 
int pidMaxPitch = pidMaxRoll;
float pidPGainYaw = 4.0, pidIGainYaw = 0.02, pidDGainYaw = 0.0;
int pidMaxYaw = 400;
boolean autoLevel = true, gyroAnglesSet;
byte lastCH1, lastCH2, lastCH3, lastCH4, eeprom_data[36], highByte, lowByte;
volatile int receiverINCH1, receiverINCH2, receiverINCH3, receiverINCH4;
int esc1, esc2, esc3, esc4, throttle, batteryVolt, calInt, start, gyroAd;
int receiverIn[5], accAxis[4], gyroAxis[4], temperature;
float adjustRoll, adjustPitch;
long accX, accY, accZ, accTotalVec;
unsigned long timerCH1, timerCH2, timerCH3, timerCH4, esc_timer, escLoopTimer;
unsigned long timer1, timer2, timer3, timer4, currentTime, loopTimer;
double gyroPitch, gyroRoll, gyroYaw, gyroAxisCal[4];
float pidMemRoll_I, pidRollSet, gyroRollIn, pidRollOut, pidRollError;
float pidMemPitch_I, pidPitchSet, gyroPitchIn, pidPitchOut, pidPitchError;
float pidMemYaw_I, pidYawSet, gyroYawIn, pidYawOut, pidYawError;
float accAngleRoll, accAnglePitch, anglePitch, angleRoll, pidTempError;

void setup(){
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;
  gyroAd = eeprom_data[32];
  
  Wire.begin();

  TWBR = 12;
  DDRD |= B11110000;
  DDRB |= B00110000;

  digitalWrite(12,HIGH);
  
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);
  
  set_gyro_registers();
  
  for (calInt = 0; calInt < 1250 ; calInt ++){
    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delayMicroseconds(3000);
  }
  for (calInt = 0; calInt < 2000 ; calInt ++){
    if(calInt % 15 == 0)digitalWrite(12, !digitalRead(12));
    gyro_signalen();
    gyroAxisCal[1] += gyroAxis[1];
    gyroAxisCal[2] += gyroAxis[2];
    gyroAxisCal[3] += gyroAxis[3];
    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delay(3);
  }
  
  gyroAxisCal[1] /= 2000;
  gyroAxisCal[2] /= 2000;
  gyroAxisCal[3] /= 2000;
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  while(receiverINCH3 < 990 || receiverINCH3 > 1020 || receiverINCH4 < 1400){
    receiverINCH3 = convert_receiver_channel(3);
    receiverINCH4 = convert_receiver_channel(4);
    start ++;
    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delay(3);
    if(start == 125){
      digitalWrite(12, !digitalRead(12));
      start = 0;
    }
  }
  start = 0;
  batteryVolt = (analogRead(0) + 65) * 1.2317;
  loopTimer = micros();
  digitalWrite(12,LOW);
}
//Main program loop
void loop(){
  gyroRollIn = (gyroRollIn * 0.7) + ((gyroRoll / 65.5) * 0.3);
  gyroPitchIn = (gyroPitchIn * 0.7) + ((gyroPitch / 65.5) * 0.3);
  gyroYawIn = (gyroYawIn * 0.7) + ((gyroYaw / 65.5) * 0.3);
  anglePitch += gyroPitch * 0.0000611;
  angleRoll += gyroRoll * 0.0000611;
  anglePitch -= angleRoll * sin(gyroYaw * 0.000001066);
  angleRoll += anglePitch * sin(gyroYaw * 0.000001066);
  accTotalVec = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));
  if(abs(accY) < accTotalVec){
    accAnglePitch = asin((float)accY/accTotalVec)* 57.296;
  }
  if(abs(accX) < accTotalVec){
    accAngleRoll = asin((float)accX/accTotalVec)* -57.296;
  }
  accAnglePitch -= 0.0;
  accAngleRoll -= 0.0;
  anglePitch = anglePitch * 0.9996 + accAnglePitch * 0.0004;
  angleRoll = angleRoll * 0.9996 + accAngleRoll * 0.0004;
  adjustPitch = anglePitch * 15;
  adjustRoll = angleRoll * 15;

  if(!autoLevel){
    adjustPitch = 0;
    adjustRoll = 0;
  }
  if(receiverINCH3 < 1050 && receiverINCH4 < 1050)start = 1;
  if(start == 1 && receiverINCH3 < 1050 && receiverINCH4 > 1450){
    start = 2;
    anglePitch = accAnglePitch;
    angleRoll = accAngleRoll;
    gyroAnglesSet = true;
    pidMemRoll_I = 0;
    pidRollError = 0;
    pidMemPitch_I = 0;
    pidPitchError = 0;
    pidMemYaw_I = 0;
    pidYawError = 0;
  }
  if(start == 2 && receiverINCH3 < 1050 && receiverINCH4 > 1950)start = 0;
  pidRollSet = 0;
  if(receiverINCH1 > 1508)pidRollSet = receiverINCH1 - 1508;
  else if(receiverINCH1 < 1492)pidRollSet = receiverINCH1 - 1492;

  pidRollSet -= adjustRoll;
  pidRollSet /= 3.0;
  pidPitchSet = 0;
  
  if(receiverINCH2 > 1508)pidPitchSet = receiverINCH2 - 1508;
  else if(receiverINCH2 < 1492)pidPitchSet = receiverINCH2 - 1492;

  pidPitchSet -= adjustPitch;
  pidPitchSet /= 3.0;
  pidYawSet = 0;
  
  if(receiverINCH3 > 1050){
    if(receiverINCH4 > 1508)pidYawSet = (receiverINCH4 - 1508)/3.0;
    else if(receiverINCH4 < 1492)pidYawSet = (receiverINCH4 - 1492)/3.0;
  }
  
  calculate_pid();
  batteryVolt = batteryVolt * 0.92 + (analogRead(0) + 65) * 0.09853;
  if(batteryVolt < 1000 && batteryVolt > 600)digitalWrite(12, HIGH);

  throttle = receiverINCH3;

  if (start == 2){
    if (throttle > 1800) throttle = 1800;
    esc1 = throttle - pidPitchOut + pidRollOut - pidYawOut;
    esc2 = throttle + pidPitchOut + pidRollOut + pidYawOut;
    esc3 = throttle + pidPitchOut - pidRollOut - pidYawOut;
    esc4 = throttle - pidPitchOut - pidRollOut + pidYawOut;

    if (batteryVolt < 1240 && batteryVolt > 800){
      esc1 += esc1 * ((1240 - batteryVolt)/(float)3500);
      esc2 += esc2 * ((1240 - batteryVolt)/(float)3500);
      esc3 += esc3 * ((1240 - batteryVolt)/(float)3500);
      esc4 += esc4 * ((1240 - batteryVolt)/(float)3500);
    } 

    if (esc1 < 1100) esc1 = 1100;
    if (esc2 < 1100) esc2 = 1100;
    if (esc3 < 1100) esc3 = 1100;
    if (esc4 < 1100) esc4 = 1100;
    if(esc1 > 2000)esc1 = 2000;
    if(esc2 > 2000)esc2 = 2000;
    if(esc3 > 2000)esc3 = 2000;
    if(esc4 > 2000)esc4 = 2000;
  }

  else{
    esc1 = 1000;
    esc2 = 1000;
    esc3 = 1000;
    esc4 = 1000;
  }
  
 if(micros() - loopTimer > 4050)digitalWrite(12, HIGH);
 
  while(micros() - loopTimer < 4000);
  loopTimer = micros();
  PORTD |= B11110000;
  timerCH1 = esc1 + loopTimer;
  timerCH2 = esc2 + loopTimer;
  timerCH3 = esc3 + loopTimer;
  timerCH4 = esc4 + loopTimer;
  gyro_signalen();

  while(PORTD >= 16){
    escLoopTimer = micros();
    if(timerCH1 <= escLoopTimer)PORTD &= B11101111;
    if(timerCH2 <= escLoopTimer)PORTD &= B11011111;
    if(timerCH3 <= escLoopTimer)PORTD &= B10111111;
    if(timerCH4 <= escLoopTimer)PORTD &= B01111111;
  }
}

ISR(PCINT0_vect){
  currentTime = micros();
  if(PINB & B00000001){
    if(lastCH1 == 0){
      lastCH1 = 1;
      timer1 = currentTime;
    }
  }
  else if(lastCH1 == 1){
    lastCH1 = 0;
    receiverIn[1] = currentTime - timer1;
  }
  if(PINB & B00000010 ){
    if(lastCH2 == 0){
      lastCH2 = 1;
      timer2 = currentTime;
    }
  }
  else if(lastCH2 == 1){
    lastCH2 = 0;
    receiverIn[2] = currentTime - timer2;
  }
  if(PINB & B00000100 ){
    if(lastCH3 == 0){
      lastCH3 = 1;
      timer3 = currentTime;
    }
  }
  else if(lastCH3 == 1){
    lastCH3 = 0;
    receiverIn[3] = currentTime - timer3;

  }
  if(PINB & B00001000 ){
    if(lastCH4 == 0){
      lastCH4 = 1;
      timer4 = currentTime;
    }
  }
  else if(lastCH4 == 1){
    lastCH4 = 0;
    receiverIn[4] = currentTime - timer4;
  }
}

void gyro_signalen(){
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyroAd);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyroAd,14);
    
    receiverINCH1 = convert_receiver_channel(1);
    receiverINCH2 = convert_receiver_channel(2);
    receiverINCH3 = convert_receiver_channel(3);
    receiverINCH4 = convert_receiver_channel(4);
    
    while(Wire.available() < 14);
    accAxis[1] = Wire.read()<<8|Wire.read();
    accAxis[2] = Wire.read()<<8|Wire.read();
    accAxis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyroAxis[1] = Wire.read()<<8|Wire.read();
    gyroAxis[2] = Wire.read()<<8|Wire.read();
    gyroAxis[3] = Wire.read()<<8|Wire.read();
  }

  if(calInt == 2000){
    gyroAxis[1] -= gyroAxisCal[1];
    gyroAxis[2] -= gyroAxisCal[2];
    gyroAxis[3] -= gyroAxisCal[3];
  }
  gyroRoll = gyroAxis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000)gyroRoll *= -1;
  gyroPitch = gyroAxis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000)gyroPitch *= -1;
  gyroYaw = gyroAxis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000)gyroYaw *= -1;
  accX = accAxis[eeprom_data[29] & 0b00000011];
  if(eeprom_data[29] & 0b10000000)accX *= -1;
  accY = accAxis[eeprom_data[28] & 0b00000011];
  if(eeprom_data[28] & 0b10000000)accY *= -1;
  accZ = accAxis[eeprom_data[30] & 0b00000011];
  if(eeprom_data[30] & 0b10000000)accZ *= -1;
}

void calculate_pid(){
  pidTempError = gyroRollIn - pidRollSet;
  pidMemRoll_I += pidIGainRoll * pidTempError;
  if(pidMemRoll_I > pidMaxRoll)pidMemRoll_I = pidMaxRoll;
  else if(pidMemRoll_I < pidMaxRoll * -1)pidMemRoll_I = pidMaxRoll * -1;

  pidRollOut = pidPGainRoll * pidTempError + pidMemRoll_I + pidDGainRoll * (pidTempError - pidRollError);
  if(pidRollOut > pidMaxRoll)pidRollOut = pidMaxRoll;
  else if(pidRollOut < pidMaxRoll * -1)pidRollOut = pidMaxRoll * -1;

  pidRollError = pidTempError;

  pidTempError = gyroPitchIn - pidPitchSet;
  pidMemPitch_I += pidIGainPitch * pidTempError;
  if(pidMemPitch_I > pidMaxPitch)pidMemPitch_I = pidMaxPitch;
  else if(pidMemPitch_I < pidMaxPitch * -1)pidMemPitch_I = pidMaxPitch * -1;

  pidPitchOut = pidPGainPitch * pidTempError + pidMemPitch_I + pidDGainPitchhh * (pidTempError - pidPitchError);
  if(pidPitchOut > pidMaxPitch)pidPitchOut = pidMaxPitch;
  else if(pidPitchOut < pidMaxPitch * -1)pidPitchOut = pidMaxPitch * -1;

  pidPitchError = pidTempError;

  pidTempError = gyroYawIn - pidYawSet;
  pidMemYaw_I += pidIGainYaw * pidTempError;
  if(pidMemYaw_I > pidMaxYaw)pidMemYaw_I = pidMaxYaw;
  else if(pidMemYaw_I < pidMaxYaw * -1)pidMemYaw_I = pidMaxYaw * -1;

  pidYawOut = pidPGainYaw * pidTempError + pidMemYaw_I + pidDGainYaw * (pidTempError - pidYawError);
  if(pidYawOut > pidMaxYaw)pidYawOut = pidMaxYaw;
  else if(pidYawOut < pidMaxYaw * -1)pidYawOut = pidMaxYaw * -1;

  pidYawError = pidTempError;
}

int convert_receiver_channel(byte function){
  byte channel, reverse;
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;
  else reverse = 0;

  actual = receiverIn[channel];
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

  if(actual < center){
    if(actual < low)actual = low;
    difference = ((long)(center - actual) * (long)500) / (center - low);
    if(reverse == 1)return 1500 + difference;
    else return 1500 - difference;
  }
  else if(actual > center){
    if(actual > high)actual = high;
    difference = ((long)(actual - center) * (long)500) / (high - center);
    if(reverse == 1)return 1500 - difference;
    else return 1500 + difference;
  }
  else return 1500;
}

void set_gyro_registers(){
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyroAd);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(gyroAd);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
    Wire.beginTransmission(gyroAd);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(gyroAd);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(gyroAd, 1);
    while(Wire.available() < 1);
    if(Wire.read() != 0x08){
      digitalWrite(12,HIGH);
      while(1)delay(10);
    }

    Wire.beginTransmission(gyroAd);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

  }  
}

