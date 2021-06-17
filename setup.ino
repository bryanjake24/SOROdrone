#include <Wire.h>  
#include <EEPROM.h> 

byte lastCH1, lastCH2, lastCH3, lastCH4, lowByte, highByte, type, gyroAd, error, clockSpd;
byte assignCH1, assignCH2, assignCH3, assignCH4, rollAxis, pitchAxis, yawAxis, receiverCheckByte, gyroCheckByte;
volatile int receiverIN1, receiverIN2, receiverIN3, receiverIN4;
int centerCH1, centerCH2, centerCH3, centerCH4, address, cal_int;
int highCH1, highCH2, highCH3, highCH4, lowCH1, lowCH2, lowCH3, lowCH4;
unsigned long timer0, timer1, timer2, timer3, timer4, currentTime;
float gyroPitch, gyroRoll, gyroYaw, calibrateRoll, calibratePitch, calibrateYaw;

void setup(){
  pinMode(12, OUTPUT);
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  Wire.begin();
  Serial.begin(57600);
  delay(250);
}
void loop(){
  intro();
  delay(1000);
  Serial.println(F("Checking I2C clock speed."));
  delay(1000);
  
  TWBR = 12;
  
  #if F_CPU == 16000000L
    clockSpd = 1;
  #endif 
  
  if(TWBR == 12 && clockSpd){
    Serial.println(F("I2C is set to 400kHz."));
  }
  else{
    Serial.println(F("I2C is not set to 400kHz.mERROR)"));
    error = 1;
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("........................................"));
    Serial.println(F("Transmitter setup"));
    Serial.println(F("........................................"));
    delay(1000);
    Serial.print(F("Checking for receiver signals."));
    waitReceiver();
    Serial.println(F(""));
  }
  if(error == 0){
    delay(2000);
    Serial.println(F("Place all sticks and subtrims in the center position within 10 seconds."));
    for(int i = 9;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    Serial.println(" ");
    centerCH1 = receiverIN1;
    centerCH2 = receiverIN2;
    centerCH3 = receiverIN3;
    centerCH4 = receiverIN4;
    Serial.println(F(""));
    Serial.println(F("Center positions stored."));
    Serial.print(F("Digital input 08 = "));
    Serial.println(receiverIN1);
    Serial.print(F("Digital input 09 = "));
    Serial.println(receiverIN2);
    Serial.print(F("Digital input 10 = "));
    Serial.println(receiverIN3);
    Serial.print(F("Digital input 11 = "));
    Serial.println(receiverIN4);
    Serial.println(F(""));
    Serial.println(F(""));
  }
  if(error == 0){  
    Serial.println(F("Move the throttle stick to full throttle and back to center"));
    receiverCheck(1);
    Serial.print(F("Throttle is connected to digital input "));
    Serial.println((assignCH3 & 0b00000111) + 7);
    if(assignCH3 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    centerPos();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the roll stick to simulate left wing up and back to center"));
    receiverCheck(2);
    Serial.print(F("Roll is connected to digital input "));
    Serial.println((assignCH1 & 0b00000111) + 7);
    if(assignCH1 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    centerPos();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the pitch stick to simulate nose up and back to center"));
    receiverCheck(3);
    Serial.print(F("Pitch is connected to digital input "));
    Serial.println((assignCH2 & 0b00000111) + 7);
    if(assignCH2 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    centerPos();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the yaw stick to simulate nose right and back to center"));
    receiverCheck(4);
    Serial.print(F("Yaw is connected to digital input "));
    Serial.println((assignCH4 & 0b00000111) + 7);
    if(assignCH4 & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    centerPos();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Gently move all the sticks simultaneously to their extends"));
    Serial.println(F("When ready put the sticks back in their center positions"));
    regminmax();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("High, low and center values found during setup"));
    Serial.print(F("Digital input 08 values:"));
    Serial.print(lowCH1);
    Serial.print(F(" - "));
    Serial.print(centerCH1);
    Serial.print(F(" - "));
    Serial.println(highCH1);
    Serial.print(F("Digital input 09 values:"));
    Serial.print(lowCH2);
    Serial.print(F(" - "));
    Serial.print(centerCH2);
    Serial.print(F(" - "));
    Serial.println(highCH2);
    Serial.print(F("Digital input 10 values:"));
    Serial.print(lowCH3);
    Serial.print(F(" - "));
    Serial.print(centerCH3);
    Serial.print(F(" - "));
    Serial.println(highCH3);
    Serial.print(F("Digital input 11 values:"));
    Serial.print(lowCH4);
    Serial.print(F(" - "));
    Serial.print(centerCH4);
    Serial.print(F(" - "));
    Serial.println(highCH4);
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    checkContinue();
  }
    
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("..................................................."));
    Serial.println(F("Gyro search"));
    Serial.println(F("..................................................."));
    delay(2000);
    Serial.println(F("Searching for MPU-6050"));
    delay(1000);
    if(gyroSearch(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68"));
      type = 1;
      gyroAd = 0x68;
    }
    if(type == 0){
      Serial.println(F("Searching for MPU-6050"));
      delay(1000);
      if(gyroSearch(0x69, 0x75) == 0x68){
        Serial.println(F("MPU-6050 found"));
        type = 1;
        gyroAd = 0x69;
      }
    }
    if(type == 0){
      Serial.println(F("No gyro device found! ERROR"));
      error = 1;
    }
    else{
      delay(3000);
      Serial.println(F(""));
      Serial.println(F("..................................................."));
      Serial.println(F("register Gyro settings"));
      Serial.println(F("..................................................."));
      gyroStart();
    }
  }

  if(error == 0){
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("..................................................."));
    Serial.println(F("Gyro calibration"));
    Serial.println(F("..................................................."));
    Serial.println(F("Don't move the quadcopter. Calibration starts in 3..."));
    delay(1000);
    Serial.println(F("2..."));
    delay(1000);
    Serial.println(F("1..."));
    delay(1000);
    Serial.println(F("Calibrating the gyro....."));
    Serial.print(F("Please wait"));
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){
      if(cal_int % 100 == 0)Serial.print(F("."));
      gyroSignal();
      calibrateRoll += gyroRoll;
      calibratePitch += gyroPitch;
      calibrateYaw += gyroYaw;
      delay(4);
    }
    calibrateRoll /= 2000;
    calibratePitch /= 2000;
    calibrateYaw /= 2000;

    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(calibrateRoll);
    Serial.print(F("Axis 2 offset="));
    Serial.println(calibratePitch);
    Serial.print(F("Axis 3 offset="));
    Serial.println(calibrateYaw);
    Serial.println(F(""));
    Serial.println(F("..................................................."));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("..................................................."));
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    gyroAxesCheck(1);
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(rollAxis & 0b00000011);
      if(rollAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      checkContinue();

      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      gyroAxesCheck(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitchAxis & 0b00000011);
      if(pitchAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      checkContinue();

      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      gyroAxesCheck(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yawAxis & 0b00000011);
      if(yawAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      checkContinue();
    }
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("..................................................."));
    Serial.println(F("LED test"));
    Serial.println(F("..................................................."));
    digitalWrite(12, HIGH);
    Serial.println(F("The LED should now be lit"));
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    checkContinue();
    digitalWrite(12, LOW);
  }
  
  Serial.println(F(""));
  
  if(error == 0){
    Serial.println(F("..................................................."));
    Serial.println(F("Final setup check"));
    Serial.println(F("..................................................."));
    delay(1000);
    if(receiverCheckByte == 0b00001111){
      Serial.println(F("Receiver channels ok"));
    }
    else{
      Serial.println(F("Receiver channel verification failed! ERROR"));
      error = 1;
    }
    delay(1000);
    if(gyroCheckByte == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed! ERROR"));
      error = 1;
    }
  }     
  
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("..................................................."));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("..................................................."));
    Serial.println(F("Writing EEPROM"));
    delay(1000);
    Serial.println(F("Done!"));
    EEPROM.write(0, centerCH1 & 0b11111111);
    EEPROM.write(1, centerCH1 >> 8);
    EEPROM.write(2, centerCH2 & 0b11111111);
    EEPROM.write(3, centerCH2 >> 8);
    EEPROM.write(4, centerCH3 & 0b11111111);
    EEPROM.write(5, centerCH3 >> 8);
    EEPROM.write(6, centerCH4 & 0b11111111);
    EEPROM.write(7, centerCH4 >> 8);
    EEPROM.write(8, highCH1 & 0b11111111);
    EEPROM.write(9, highCH1 >> 8);
    EEPROM.write(10, highCH2 & 0b11111111);
    EEPROM.write(11, highCH2 >> 8);
    EEPROM.write(12, highCH3 & 0b11111111);
    EEPROM.write(13, highCH3 >> 8);
    EEPROM.write(14, highCH4 & 0b11111111);
    EEPROM.write(15, highCH4 >> 8);
    EEPROM.write(16, lowCH1 & 0b11111111);
    EEPROM.write(17, lowCH1 >> 8);
    EEPROM.write(18, lowCH2 & 0b11111111);
    EEPROM.write(19, lowCH2 >> 8);
    EEPROM.write(20, lowCH3 & 0b11111111);
    EEPROM.write(21, lowCH3 >> 8);
    EEPROM.write(22, lowCH4 & 0b11111111);
    EEPROM.write(23, lowCH4 >> 8);
    EEPROM.write(24, assignCH1);
    EEPROM.write(25, assignCH2);
    EEPROM.write(26, assignCH3);
    EEPROM.write(27, assignCH4);
    EEPROM.write(28, rollAxis);
    EEPROM.write(29, pitchAxis);
    EEPROM.write(30, yawAxis);
    EEPROM.write(31, type);
    EEPROM.write(32, gyroAd);
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
        
    Serial.println(F("Verify EEPROM data"));
    delay(1000);
    if(centerCH1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(centerCH2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(centerCH3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(centerCH4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(highCH1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(highCH2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(highCH3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(highCH4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(lowCH1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(lowCH2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(lowCH3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(lowCH4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(assignCH1 != EEPROM.read(24))error = 1;
    if(assignCH2 != EEPROM.read(25))error = 1;
    if(assignCH3 != EEPROM.read(26))error = 1;
    if(assignCH4 != EEPROM.read(27))error = 1;
    
    if(rollAxis != EEPROM.read(28))error = 1;
    if(pitchAxis != EEPROM.read(29))error = 1;
    if(yawAxis != EEPROM.read(30))error = 1;
    if(type != EEPROM.read(31))error = 1;
    if(gyroAd != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed. ERROR"));
    else Serial.println(F("Verification done"));
  }
  
  
  if(error == 0){
    Serial.println(F("Setup is finished."));
  }
  else{
   Serial.println(F("The setup is aborted. ERROR."));
  }
  while(1);
}

byte gyroSearch(int gyroAd, int who_am_i){
  Wire.beginTransmission(gyroAd);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyroAd, 1);
  timer0 = millis() + 100;
  while(Wire.available() < 1 && timer0 > millis());
  lowByte = Wire.read();
  address = gyroAd;
  return lowByte;
}

void gyroStart(){
  if(type == 1){
    
    Wire.beginTransmission(address);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    
    Wire.beginTransmission(address);
    Wire.write(0x6B);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    while(Wire.available() < 1);
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
    
    Wire.beginTransmission(address);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    while(Wire.available() < 1);
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void gyroSignal(){
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);
    Wire.write(168);
    Wire.endTransmission();
    Wire.requestFrom(address, 6);
    while(Wire.available() < 6);
    lowByte = Wire.read();
    highByte = Wire.read();
    gyroRoll = ((highByte<<8)|lowByte);
    if(cal_int == 2000)gyroRoll -= calibrateRoll;
    lowByte = Wire.read();
    highByte = Wire.read();
    gyroPitch = ((highByte<<8)|lowByte);
    if(cal_int == 2000)gyroPitch -= calibratePitch;
    lowByte = Wire.read();
    highByte = Wire.read();
    gyroYaw = ((highByte<<8)|lowByte);
    if(cal_int == 2000)gyroYaw -= calibrateYaw;
  }
  if(type == 1){
    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(address,6);
    while(Wire.available() < 6);
    gyroRoll=Wire.read()<<8|Wire.read();
    if(cal_int == 2000)gyroRoll -= calibrateRoll;
    gyroPitch=Wire.read()<<8|Wire.read();
    if(cal_int == 2000)gyroPitch -= calibratePitch;
    gyroYaw=Wire.read()<<8|Wire.read();
    if(cal_int == 2000)gyroYaw -= calibrateYaw;
  }
}

void receiverCheck(byte movement){
  byte trigger = 0;
  int pulseLength;
  timer0 = millis() + 30000;
  while(timer0 > millis() && trigger == 0){
    delay(250);
    if(receiverIN1 > 1750 || receiverIN1 < 1250){
      trigger = 1;
      receiverCheckByte |= 0b00000001;
      pulseLength = receiverIN1;
    }
    if(receiverIN2 > 1750 || receiverIN2 < 1250){
      trigger = 2;
      receiverCheckByte |= 0b00000010;
      pulseLength = receiverIN2;
    }
    if(receiverIN3 > 1750 || receiverIN3 < 1250){
      trigger = 3;
      receiverCheckByte |= 0b00000100;
      pulseLength = receiverIN3;
    }
    if(receiverIN4 > 1750 || receiverIN4 < 1250){
      trigger = 4;
      receiverCheckByte |= 0b00001000;
      pulseLength = receiverIN4;
    } 
  }
  if(trigger == 0){
    error = 1;
    Serial.println(F("No stick movement detected in the last 30 seconds! ERROR"));
  }
  else{
    if(movement == 1){
      assignCH3 = trigger;
      if(pulseLength < 1250)assignCH3 += 0b10000000;
    }
    if(movement == 2){
      assignCH1 = trigger;
      if(pulseLength < 1250)assignCH1 += 0b10000000;
    }
    if(movement == 3){
      assignCH2 = trigger;
      if(pulseLength < 1250)assignCH2 += 0b10000000;
    }
    if(movement == 4){
      assignCH4 = trigger;
      if(pulseLength < 1250)assignCH4 += 0b10000000;
    }
  }
}

void checkContinue(){
  byte continue_byte = 0;
  while(continue_byte == 0){
    if(assignCH2 == 0b00000001 && receiverIN1 > centerCH1 + 150)continue_byte = 1;
    if(assignCH2 == 0b10000001 && receiverIN1 < centerCH1 - 150)continue_byte = 1;
    if(assignCH2 == 0b00000010 && receiverIN2 > centerCH2 + 150)continue_byte = 1;
    if(assignCH2 == 0b10000010 && receiverIN2 < centerCH2 - 150)continue_byte = 1;
    if(assignCH2 == 0b00000011 && receiverIN3 > centerCH3 + 150)continue_byte = 1;
    if(assignCH2 == 0b10000011 && receiverIN3 < centerCH3 - 150)continue_byte = 1;
    if(assignCH2 == 0b00000100 && receiverIN4 > centerCH4 + 150)continue_byte = 1;
    if(assignCH2 == 0b10000100 && receiverIN4 < centerCH4 - 150)continue_byte = 1;
    delay(100);
  }
  centerPos();
}

void centerPos(){
  byte zero = 0;
  while(zero < 15){
    if(receiverIN1 < centerCH1 + 20 && receiverIN1 > centerCH1 - 20)zero |= 0b00000001;
    if(receiverIN2 < centerCH2 + 20 && receiverIN2 > centerCH2 - 20)zero |= 0b00000010;
    if(receiverIN3 < centerCH3 + 20 && receiverIN3 > centerCH3 - 20)zero |= 0b00000100;
    if(receiverIN4 < centerCH4 + 20 && receiverIN4 > centerCH4 - 20)zero |= 0b00001000;
    delay(100);
  }
}

void waitReceiver(){
  byte zero = 0;
  timer0 = millis() + 10000;
  while(timer0 > millis() && zero < 15){
    if(receiverIN1 < 2100 && receiverIN1 > 900)zero |= 0b00000001;
    if(receiverIN2 < 2100 && receiverIN2 > 900)zero |= 0b00000010;
    if(receiverIN3 < 2100 && receiverIN3 > 900)zero |= 0b00000100;
    if(receiverIN4 < 2100 && receiverIN4 > 900)zero |= 0b00001000;
    delay(500);
    Serial.print(F("."));
  }
  if(zero == 0){
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found! ERROR"));
  }
  else Serial.println(F(" OK"));
}

void regminmax(){
  byte zero = 0;
  lowCH1 = receiverIN1;
  lowCH2 = receiverIN2;
  lowCH3 = receiverIN3;
  lowCH4 = receiverIN4;
  while(receiverIN1 < centerCH1 + 20 && receiverIN1 > centerCH1 - 20)delay(250);
  Serial.println(F("Measuring endpoints...."));
  while(zero < 15){
    if(receiverIN1 < centerCH1 + 20 && receiverIN1 > centerCH1 - 20)zero |= 0b00000001;
    if(receiverIN2 < centerCH2 + 20 && receiverIN2 > centerCH2 - 20)zero |= 0b00000010;
    if(receiverIN3 < centerCH3 + 20 && receiverIN3 > centerCH3 - 20)zero |= 0b00000100;
    if(receiverIN4 < centerCH4 + 20 && receiverIN4 > centerCH4 - 20)zero |= 0b00001000;
    if(receiverIN1 < lowCH1)lowCH1 = receiverIN1;
    if(receiverIN2 < lowCH2)lowCH2 = receiverIN2;
    if(receiverIN3 < lowCH3)lowCH3 = receiverIN3;
    if(receiverIN4 < lowCH4)lowCH4 = receiverIN4;
    if(receiverIN1 > highCH1)highCH1 = receiverIN1;
    if(receiverIN2 > highCH2)highCH2 = receiverIN2;
    if(receiverIN3 > highCH3)highCH3 = receiverIN3;
    if(receiverIN4 > highCH4)highCH4 = receiverIN4;
    delay(100);
  }
}

void gyroAxesCheck(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  gyroSignal();
  timer0 = millis() + 10000;    
  while(timer0 > millis() && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyroSignal();
    if(type == 2 || type == 3){
      gyro_angle_roll += gyroRoll * 0.00007;
      gyro_angle_pitch += gyroPitch * 0.00007;
      gyro_angle_yaw += gyroYaw * 0.00007;
    }
    if(type == 1){
      gyro_angle_roll += gyroRoll * 0.0000611;
      gyro_angle_pitch += gyroPitch * 0.0000611;
      gyro_angle_yaw += gyroYaw * 0.0000611;
    }
    
    delayMicroseconds(3700);
  }
  if((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyroCheckByte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyroCheckByte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyroCheckByte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds! ERROR"));
  }
  else
  if(movement == 1)rollAxis = trigger_axis;
  if(movement == 2)pitchAxis = trigger_axis;
  if(movement == 3)yawAxis = trigger_axis;
  
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
    receiverIN1 = currentTime - timer1;
  }
  if(PINB & B00000010 ){
    if(lastCH2 == 0){
      lastCH2 = 1;
      timer2 = currentTime;
    }
  }
  else if(lastCH2 == 1){
    lastCH2 = 0;
    receiverIN2 = currentTime - timer2;
  }
  if(PINB & B00000100 ){
    if(lastCH3 == 0){
      lastCH3 = 1;
      timer3 = currentTime;
    }
  }
  else if(lastCH3 == 1){
    lastCH3 = 0;
    receiverIN3 = currentTime - timer3;

  }
  if(PINB & B00001000 ){
    if(lastCH4 == 0){
      lastCH4 = 1;
      timer4 = currentTime;
    }
  }
  else if(lastCH4 == 1){
    lastCH4 = 0;
    receiverIN4 = currentTime - timer4;
  }
}

void intro(){
  Serial.println(F("..................................................."));
  Serial.println(F("Setup Program"));
  Serial.println(F(""));
  Serial.println(F("..................................................."));
}
