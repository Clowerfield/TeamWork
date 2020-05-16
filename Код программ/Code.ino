œ–Œ√–¿Ã¿  ≈–”¬¿ÕÕﬂ œ–»…ŒÃŒÃ ƒ¿Õ»’ « œ–»—“–Œﬁ
#include <Wire.h>
#include "Kalman.h"
Kalman kalmanX;
Kalman kalmanY;
uint8_t IMUAddress = 0x68;
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
double accXangle; // Angle calculate using the accelerometer
double accYangle;
double temp;
double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180;
double compAngleX = 180; // Calculate the angle using a Kalman filter
double compAngleY = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;
uint32_t timer;
void setup() {
	Serial1.begin(57600);
Serial.begin(57600);
  Wire.begin();
  Serial.begin(9600);
  i2cWrite(0x6B,0x00); // Disable sleep mode      
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  timer = micros();
}
void loop() {
  /* Update all the values */
 while (Serial.available())
 Serial1.write(Serial.read());
 while (Serial1.available())
 Serial.write(Serial1.read());

  uint8_t* data = i2cRead(0x3B,14);
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);
  tempRaw = ((data[6] << 8) | data[7]);
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);
  /* Calculate the angls based on the different sensors and algorithm */
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
  timer = micros();
Serial.println();
    Serial.print("X:");
    Serial.print(kalAngleX,0);
    Serial.print(" ");
    Serial.print("Y:");
    Serial.print(kalAngleY,0);
    Serial.println(" ");
  // The accelerometer's maximum samples rate is 1kHz
}
void i2cWrite(uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data [i]= Wire.read();
  return data;
}




œ–Œ√–¿Ã¿ ƒÀﬂ œ≤ƒ—»—“≈Ã»  ≈–”¬¿ÕÕﬂ  ƒ¬»√”Õ¿Ã»
#define PIN_OUT A1
int raw = 0;
float temp = 0;
int enA = 10;
int in1 = 9;
int in2 = 8;
int enB = 5;
int in3 = 7;
int in4 = 6;
#include <TroykaCurrent.h>
void setup() {
  pinMode( A0, INPUT );
  Serial.begin(9600);
pinMode(enA, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);
}

void loop() {

  raw = analogRead(A0);
  temp = ( raw/1023.0 )*5.0*1000/10;
  if (temp < 80)
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);
   Serial.print("i = ");
   Serial.print(dataI.readCurrentDC());
   Serial.println(" A");
    delay(1000); 
    demoOne();
   delay(1000);
   demoTwo();
   delay(1000);
}
void demoOne()
{

digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);

analogWrite(enA, 200);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);

analogWrite(enB, 200);
delay(2000);
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
delay(2000);
// ‚˚ÍÎ˛˜‡ÂÏ ‰‚Ë„‡ÚÂÎË
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
void demoTwo()
{

digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
for (int i = 0; i < 256; i++)
{
analogWrite(enA, i);
analogWrite(enB, i);
delay(20);
}
for (int i = 255; i >= 0; --i)
{
analogWrite(enA, i);
analogWrite(enB, i);
delay(20);
}
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}



œ–Œ√–¿Ã¿ ƒÀﬂ œ≤ƒ—»—“≈Ã» œ–»…ŒÃ” ƒ¿Õ»’ œ–»—“–Œ™Ã

#include <SerialFlow.h>
SerialFlow rd(9,10);
const unsigned long data_to = 100;
unsigned long tm, data_next;
void setup(void){
    rd.setPacketFormat(2, 1);
    rd.begin(0xF0F0F0F0E1LL,0xF0F0F0F0D2LL);
 Serial.begin(57600);
    rd.setPacketFormat(2, 1);
    rd.begin(0xF0F0F0F0D2LL,0xF0F0F0F0E1LL);

}
void loop(void){
    tm = millis();
    if( tm > data_next ){
        data_next = tm + data_to;
        rd.setPacketValue( tm );
        rd.sendPacket();
 unsigned int v;
    if( rd.receivePacket() ){
        v = rd.getPacketValue(0);
        Serial.println(v);

    }
}


