//r = receiver signals
//a = quadcopter angles
//1 = check motor 1 rotation
//2 = check motor 2 rotation
//3 = check motor 3 rotation
//4 = check motor 4 rotation
//5 = check all motors
#include <Wire.h>
#include <EEPROM.h>

byte lastCH1, lastCH2, lastCH3, lastCH4, eeprom_data[36], start, data;
boolean reqNewFunc,firstAngle;
volatile int counterCH4CH1, counterCH4CH2, counterCH4CH3, counterCH4CH4;
int esc1, esc2, esc3, esc4, receiverIn[5], loopCtr, gyroAd, vibrationCtr, temperature;
long accX, accY, accZ, accTotalVec[20], accAvVec, vibrationTot;
unsigned long timerCH1, timerCH2, timerCH3, timerCH4, escTimer, escLoopTimer;
unsigned long zeroTimer, timer1, timer2, timer3, timer4, currentTime;
int accAxis[4], gyroAxis[4];
double gyroPitch, gyroRoll, gyroYaw;
float accAngleRoll, accAnglePitch, anglePitch, angleRoll;
int calInt;
double gyroAxisCal[4];

void setup(){
  Serial.begin(57600);
  Wire.begin();
  
  TWBR = 12;
  DDRD |= B11110000;
  DDRB |= B00010000;
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  for(data = 0; data <= 35; data++)eeprom_data[data] = EEPROM.read(data);

  gyroAd = eeprom_data[32];
  setGyroReg();

  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B'){
    delay(500);
    digitalWrite(12, !digitalRead(12));
  }
  waitReceiver();
  zeroTimer = micros();

  while(Serial.available())data = Serial.read();
  data = 0;
}

void loop(){
  while(zeroTimer + 4000 > micros());
  zeroTimer = micros();
  
  if(Serial.available() > 0){
    data = Serial.read();
    delay(100);
    while(Serial.available() > 0)loopCtr = Serial.read();
    reqNewFunc = true;
    loopCtr = 0;
    calInt = 0;
    start = 0;
    firstAngle = false;
    if(data == 'r')Serial.println("Reading receiver signals.");
    if(data == 'a')Serial.println("Print the quadcopter angles.");
    if(data == 'a')Serial.println("Gyro calibration starts in 2 seconds.");
    if(data == '1')Serial.println("Test motor 1.");
    if(data == '2')Serial.println("Test motor 2.");
    if(data == '3')Serial.println("Test motor 3.");
    if(data == '4')Serial.println("Test motor 4.)");
    if(data == '5')Serial.println("Test all motors together");

    for(vibrationCtr = 0; vibrationCtr < 625; vibrationCtr++){
      delay(3);
      esc1 = 1000;
      esc2 = 1000;
      esc3 = 1000;
      esc4 = 1000;
      escPulseout();
    }
    vibrationCtr = 0;
  }

  counterCH4CH3 = receiverChConvert(3);
  if(counterCH4CH3 < 1025)reqNewFunc = false;

  if(data == 0 && reqNewFunc == false){
    counterCH4CH3 = receiverChConvert(3);
    esc1 = counterCH4CH3;
    esc2 = counterCH4CH3;
    esc3 = counterCH4CH3;
    esc4 = counterCH4CH3;
    escPulseout();
  }

  if(data == 'r'){
    loopCtr ++;
    counterCH4CH1 = receiverChConvert(1);
    counterCH4CH2 = receiverChConvert(2);
    counterCH4CH3 = receiverChConvert(3);
    counterCH4CH4 = receiverChConvert(4);
    if(loopCtr == 125){
      signalPrint();
      loopCtr = 0;
    }

    if(counterCH4CH3 < 1050 && counterCH4CH4 < 1050)start = 1;
    if(start == 1 && counterCH4CH3 < 1050 && counterCH4CH4 > 1450)start = 2;
    if(start == 2 && counterCH4CH3 < 1050 && counterCH4CH4 > 1950)start = 0;

    esc1 = 1000;
    esc2 = 1000;
    esc3 = 1000;
    esc4 = 1000;
    escPulseout();
  }

  if(data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){
    loopCtr ++;
    if(reqNewFunc == true && loopCtr == 250){
      Serial.print("Set throttle to 1000 (low). It's now set to: ");
      Serial.println(counterCH4CH3);
      loopCtr = 0;
    }
    if(reqNewFunc == false){
      counterCH4CH3 = receiverChConvert(3);
      if(data == '1' || data == '5')esc1 = counterCH4CH3;
      else esc1 = 1000;
      if(data == '2' || data == '5')esc2 = counterCH4CH3;
      else esc2 = 1000;
      if(data == '3' || data == '5')esc3 = counterCH4CH3;
      else esc3 = 1000;
      if(data == '4' || data == '5')esc4 = counterCH4CH3;
      else esc4 = 1000;
      escPulseout();

      if(eeprom_data[31] == 1){
        Wire.beginTransmission(gyroAd);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(gyroAd,6);
        while(Wire.available() < 6);
        accX = Wire.read()<<8|Wire.read();
        accY = Wire.read()<<8|Wire.read();
        accZ = Wire.read()<<8|Wire.read();

        accTotalVec[0] = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));

       accAvVec = accTotalVec[0];

        for(start = 16; start > 0; start--){
          accTotalVec[start] = accTotalVec[start - 1];
          accAvVec += accTotalVec[start];
        }

        accAvVec /= 17;

        if(vibrationCtr < 20){
          vibrationCtr ++;
          vibrationTot += abs(accTotalVec[0] - accAvVec);
        }
        else{
          vibrationCtr = 0;
          Serial.println(vibrationTot/50);
          vibrationTot = 0;
        }
      }
    }
  }
  if(data == 'a'){

    if(calInt != 2000){
      Serial.print("Calibrating the gyro");
      for (calInt = 0; calInt < 2000 ; calInt ++){
        if(calInt % 125 == 0){
          digitalWrite(12, !digitalRead(12));
          Serial.print(".");
        }
        gyroSignal();
        gyroAxisCal[1] += gyroAxis[1];
        gyroAxisCal[2] += gyroAxis[2];
        gyroAxisCal[3] += gyroAxis[3];
        PORTD |= B11110000;
        delayMicroseconds(1000);
        PORTD &= B00001111;
        delay(3);
      }
      Serial.println(".");
      gyroAxisCal[1] /= 2000;
      gyroAxisCal[2] /= 2000;
      gyroAxisCal[3] /= 2000;
    }
    else{
      PORTD |= B11110000;
      delayMicroseconds(1000);
      PORTD &= B00001111;
      gyroSignal();
      anglePitch += gyroPitch * 0.0000611;
      angleRoll += gyroRoll * 0.0000611;
      anglePitch -= angleRoll * sin(gyroYaw * 0.000001066);
      angleRoll += anglePitch * sin(gyroYaw * 0.000001066);
      accTotalVec[0] = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));
      accAnglePitch = asin((float)accY/accTotalVec[0])* 57.296;
      accAngleRoll = asin((float)accX/accTotalVec[0])* -57.296;
      
      if(!firstAngle){
        anglePitch = accAnglePitch;
        angleRoll = accAngleRoll;
        firstAngle = true;
      }
      else{
        anglePitch = anglePitch * 0.9996 + accAnglePitch * 0.0004;
        angleRoll = angleRoll * 0.9996 + accAngleRoll * 0.0004;
      }
      if(loopCtr == 0)Serial.print("Pitch: ");
      if(loopCtr == 1)Serial.print(anglePitch ,0);
      if(loopCtr == 2)Serial.print(" Roll: ");
      if(loopCtr == 3)Serial.print(angleRoll ,0);
      if(loopCtr == 4)Serial.print(" Yaw: ");
      if(loopCtr == 5)Serial.println(gyroYaw / 65.5 ,0);

      loopCtr ++;
      if(loopCtr == 60)loopCtr = 0;      
    }
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

void waitReceiver(){
  byte zero = 0;
  while(zero < 15){
    if(receiverIn[1] < 2100 && receiverIn[1] > 900)zero |= 0b00000001;
    if(receiverIn[2] < 2100 && receiverIn[2] > 900)zero |= 0b00000010;
    if(receiverIn[3] < 2100 && receiverIn[3] > 900)zero |= 0b00000100;
    if(receiverIn[4] < 2100 && receiverIn[4] > 900)zero |= 0b00001000;
    delay(500);
  }
}

int receiverChConvert(byte function){
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

void signalPrint(){
  Serial.print("Start:");
  Serial.print(start);
  Serial.print("  Roll:");
  if(counterCH4CH1 - 1480 < 0)Serial.print("<<<");
  else if(counterCH4CH1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(counterCH4CH1);
  Serial.print("  Pitch:");
  if(counterCH4CH2 - 1480 < 0)Serial.print("^^^");
  else if(counterCH4CH2 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(counterCH4CH2);
  Serial.print("  Throttle:");
  if(counterCH4CH3 - 1480 < 0)Serial.print("vvv");
  else if(counterCH4CH3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(counterCH4CH3);
  Serial.print("  Yaw:");
  if(counterCH4CH4 - 1480 < 0)Serial.print("<<<");
  else if(counterCH4CH4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(counterCH4CH4);
}

void escPulseout(){
  zeroTimer = micros();
  PORTD |= B11110000;
  timerCH1 = esc1 + zeroTimer;
  timerCH2 = esc2 + zeroTimer;
  timerCH3 = esc3 + zeroTimer;
  timerCH4 = esc4 + zeroTimer;

  while(PORTD >= 16){
    escLoopTimer = micros();
    if(timerCH1 <= escLoopTimer)PORTD &= B11101111;
    if(timerCH2 <= escLoopTimer)PORTD &= B11011111;
    if(timerCH3 <= escLoopTimer)PORTD &= B10111111;
    if(timerCH4 <= escLoopTimer)PORTD &= B01111111;
  }
}

void setGyroReg(){
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

void gyroSignal(){
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyroAd);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyroAd,14);
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







