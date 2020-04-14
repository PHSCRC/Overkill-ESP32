#include <CAN.h>

#include <stdint.h>
/*
 * an arduino sketch to interface with a ps/2 mouse.
 * Also uses serial protocol to talk back to the host
 * and report what it finds.
 */

/*
 * Pin 5 is the mouse data pin, pin 6 is the clock pin
 * Feel free to use whatever pins are convenient.
 */
#define MCLK0 13
#define MDATA0 12
#define MCLK1 14
#define MDATA1 27

#define M_PI 3.14159265f

#define MYID 100
#define SYNCHRONISEDID 0

inline float sqrtSquaredSum(float a, float b){
  return sqrt(a*a+b*b);
}

float toStdPos(float angle){
  while(angle<0){angle+=2*M_PI;}
  while(angle>=2*M_PI){angle-=2*M_PI;}
  return angle;
}

uint16_t getFixedPoint(float a, uint16_t max_, bool sign){
  if(sign) {
    int16_t representation=a/max_*32767;
    return representation;
  } else {
    uint16_t representation=a/max_*65535;
    return representation;
  }
}
inline uint8_t extractByte(uint16_t inByte){
  return (uint8_t)(inByte>>1);
}

inline int16_t extractDataX(uint16_t infoRaw, uint16_t dataRaw){
  int16_t out;

  uint8_t infoByte=extractByte(infoRaw);
  uint8_t mask=1;
  mask<<=1;
  if(mask & infoByte){
    out=255;
  } else {
    out=extractByte(dataRaw);
  }
  mask<<=2;
  if(mask & infoByte){
    out=-out;
  }
  return out;
}

inline int16_t extractDataY(uint16_t infoRaw, uint16_t dataRaw){
  int16_t out;

  uint8_t infoByte=extractByte(infoRaw);
  uint8_t mask=1;
  mask<<=0;
  if(mask & infoByte){
    out=255;
  } else {
    out=extractByte(dataRaw);
  }
  mask<<=2;
  if(mask & infoByte){
    out=-out;
  }
  return out;
}

/*
 * according to some code I saw, these functions will
 * correctly set the mouse clock and data pins for
 * various conditions.
 */
inline void gohi(uint8_t pin)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

inline void golo(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void mouse_write(uint8_t data, uint8_t clkPin, uint8_t dataPin)
{
  uint8_t i;
  uint8_t parity = 1;

  //  Serial.print("Sending ");
  //  Serial.print(data, HEX);
  //  Serial.print(" to mouse\n");
  //  Serial.print("RTS");
  /* put pins in output mode */
  gohi(dataPin);
  gohi(clkPin);
  delayMicroseconds(300);
  golo(clkPin);
  delayMicroseconds(300);
  golo(dataPin);
  delayMicroseconds(10);
  /* start bit */
  gohi(clkPin);
  /* wait for mouse to take control of clock); */
  while (digitalRead(clkPin) == HIGH)
    ;
  /* clock is low, and we are clear to send data */
  for (i=0; i < 8; i++) {
    if (data & 0x01) {
      gohi(dataPin);
    }
    else {
      golo(dataPin);
    }
    /* wait for clock cycle */
    while (digitalRead(clkPin) == LOW)
      ;
    while (digitalRead(clkPin) == HIGH)
      ;
    parity = parity ^ (data & 0x01);
    data = data >> 1;
  }
  /* parity */
  if (parity) {
    gohi(dataPin);
  }
  else {
    golo(dataPin);
  }
  while (digitalRead(clkPin) == LOW)
    ;
  while (digitalRead(clkPin) == HIGH)
    ;
  /* stop bit */
  gohi(dataPin);
  delayMicroseconds(50);
  while (digitalRead(clkPin) == HIGH)
    ;
  /* wait for mouse to switch modes */
  while ((digitalRead(clkPin) == LOW) || (digitalRead(dataPin) == LOW))
    ;
  /* put a hold on the incoming data. */
  //golo(MCLK);
  //  Serial.print("done.\n");
}

/*
 * Get a byte of data from the mouse
 */
char mouse_read(uint8_t clkPin, uint8_t dataPin)
{
  char data = 0x00;
  int i;
  char bit = 0x01;

  //  Serial.print("reading byte from mouse\n");
  /* start the clock */
  gohi(clkPin);
  gohi(dataPin);
  delayMicroseconds(50);
  while (digitalRead(clkPin) == HIGH)
    ;
  delayMicroseconds(5);  /* not sure why */
  while (digitalRead(clkPin) == LOW) /* eat start bit */
    ;
  for (i=0; i < 8; i++) {
    while (digitalRead(clkPin) == HIGH)
      ;
    if (digitalRead(dataPin) == HIGH) {
      data = data | bit;
    }
    while (digitalRead(clkPin) == LOW)
      ;
    bit = bit << 1;
  }
  /* eat parity bit, which we ignore */
  while (digitalRead(clkPin) == HIGH)
    ;
  while (digitalRead(clkPin) == LOW)
    ;
  /* eat stop bit */
  while (digitalRead(clkPin) == HIGH)
    ;
  while (digitalRead(clkPin) == LOW)
    ;

  /* put a hold on the incoming data. */
  golo(clkPin);
  //  Serial.print("Recvd data ");
  //  Serial.print(data, HEX);
  //  Serial.print(" from mouse\n");
  return data;
}

bool activateInterupts[2];

void mouse_init(uint8_t clkPin, uint8_t dataPin, uint8_t i)
{
  gohi(clkPin);
  gohi(dataPin);
  //  Serial.print("Sending reset to mouse\n");
  mouse_write(0xff, clkPin, dataPin);
  mouse_read(clkPin, dataPin);  // ack byte
  mouse_read(clkPin, dataPin);  //self test successful
  mouse_read(clkPin, dataPin);  //mouse id
  mouse_write(0xf3, clkPin, dataPin); //set sample rate
  mouse_read(clkPin, dataPin);
  mouse_write(0xc8, clkPin, dataPin); //set the sample rate as high as the ps/2 protocol can go
  mouse_read(clkPin, dataPin);
  mouse_write(0xe8, clkPin, dataPin); //set counts/mm
  mouse_read(clkPin, dataPin);
  mouse_write(0x03, clkPin, dataPin); //set it at 8/mm (prob not followed) but still tells it go as high as it can go.
  mouse_read(clkPin, dataPin);
  gohi(clkPin);
  activateInterupts[i]=true;
  mouse_write(0xf4, clkPin, dataPin); //start streaming.
  mouse_read(clkPin, dataPin);

  //  Serial.print("Sending remote mode code\n");
  //should now be in stream mode
  /*
  mouse_write(0xf0); //  remote mode
  mouse_read();  // ack
  //  Serial.print("Read ack byte2\n");
  */
}
volatile uint16_t fullTransmitionBits[2][256];
volatile uint64_t transmitionTimes[2][256];
volatile uint8_t transNum[2];
volatile uint8_t bitNumInTransmition[2];

inline void generalISR(uint8_t whichOne, uint8_t dataPin){
   if(activateInterupts[whichOne]){
    if(bitNumInTransmition==0){
      transmitionTimes[whichOne][transNum[whichOne]]=micros();
    }
    fullTransmitionBits[whichOne][transNum[whichOne]]|=digitalRead(dataPin)<<bitNumInTransmition[whichOne];
    if(bitNumInTransmition[whichOne]==10){
      bitNumInTransmition[whichOne]=0;
      ++transNum[whichOne];
    } else {
      ++bitNumInTransmition[whichOne];
    }
  }
}

void IRAM_ATTR CLKFALLING0() {
 generalISR(0, MDATA0);
}
void IRAM_ATTR CLKFALLING1() {
 generalISR(1, MDATA1);
}

#define leftAxisZero true

#define countsTomm 0.125f
#define wheelDist 20.0f

//meas_period*pll_kp must be <1
#define pll_kp_ 25  //Odrive is at 1/8000, and they set it to 1000, we are at 1/200, so we set it to 25 (proportions)
#define pll_ki_ (0.25f * (pll_kp_ * pll_kp_)) // Critically damped

uint8_t lastAccumalatedTrans[2];
uint32_t nextTargetMs;

float distEstimate[2];
float distRaw[2];
float velEstimate[2];
bool updatedSinceTotalAccum[2];

float dTheta;
float dX;
float dY;

uint64_t latestNMicros;
uint8_t latestN;

void broadcastOdomOverCAN(){
  CAN.beginPacket(MYID);
  //these are both actuallly ints underneath
  uint16_t dXFixed=getFixedPoint(dX, 1, true);
  uint16_t dYFixed=getFixedPoint(dY, 1, true);

  uint16_t dThetaFixed=getFixedPoint(dTheta, 7, false);

  CAN.write((uint8_t)(dXFixed<<8));
  CAN.write((uint8_t)dXFixed);
  CAN.write((uint8_t)(dYFixed<<8));
  CAN.write((uint8_t)dYFixed);
  CAN.write((uint8_t)(dThetaFixed<<8));
  CAN.write((uint8_t)dThetaFixed);

  CAN.write(latestN);
  CAN.write((uint8_t)(min(min(transmitionTimes[0][lastAccumalatedTrans[0]-3], transmitionTimes[1][lastAccumalatedTrans[1]-3])-latestNMicros)/1000, 255));

  CAN.endPacket();
}

void canCallback(int packetLength){
  //we should be filtering for only synchronised time, so no need to check ids or anything (in theory)
  latestNMicros=micros();
  latestN=CAN.read();
}

void resetRobotAccum(){
  for(uint8_t i=0; i<2; ++i){
     distEstimate[i]=0;
     distRaw[i]=0;
    //not velEstimate cause that needs to be continuous
    updatedSinceTotalAccum[i]=false;
  }
}

void robotAccum(){
  uint8_t rightAxis=!leftAxisZero ? 0 : 1;
  uint8_t leftAxis=leftAxisZero ? 0 : 1;
  float old_theta=dTheta;
  dTheta=(distEstimate[rightAxis]-distEstimate[leftAxis])/wheelDist;
  float d_theta=dTheta-old_theta;
  float d_f=((velEstimate[rightAxis]+velEstimate[leftAxis])/2.0f) //forward velocity
  /
  ((velEstimate[rightAxis]-velEstimate[leftAxis])/wheelDist) //angular velocity
  *
  2.0f*fabsf(sin(d_theta*0.5f)); //sqrt(sin^2(x)+(1-cos(x))^2)=2*abs(sin(x/2))
  dX+=d_f*cos(dTheta);
  dY+=d_f*sin(dTheta);
}

void setup()
{
  CAN.begin(250E3);
  CAN.filter(SYNCHRONISEDID);
  CAN.onReceive(canCallback);
   //Serial.begin(9600);
  attachInterrupt(MCLK0, CLKFALLING0, FALLING);
  mouse_init(MCLK0, MDATA0, 0);
  attachInterrupt(MCLK1, CLKFALLING1, FALLING);
  mouse_init(MCLK1, MDATA1, 1);
}

void loop()
{

  /*
  // get a reading from the mouse
  mouse_write(0xeb);  // give me data!
  mouse_read();      // ignore ack
  mstat = mouse_read();
  mx = mouse_read();
  my = mouse_read();

  // send the data back up
  Serial.print(mstat, BIN);
  Serial.print("\tX=");
  Serial.print(mx, DEC);
  Serial.print("\tY=");
  Serial.print(my, DEC);
  Serial.println();
  delay(20);  // twiddle
  */

  while(millis()<nextTargetMs){
    for(uint8_t i=0; i<2; ++i){
      if(transNum[i]-lastAccumalatedTrans[i]>=3){
        updatedSinceTotalAccum[i]=true;
        distRaw[i]+=sqrtSquaredSum(countsTomm*extractDataX(fullTransmitionBits[i][lastAccumalatedTrans[i]], fullTransmitionBits[i][lastAccumalatedTrans[i]+1]),
                                   countsTomm*extractDataY(fullTransmitionBits[i][lastAccumalatedTrans[i]], fullTransmitionBits[i][lastAccumalatedTrans[i]+2]));
        float dT=(transmitionTimes[i][lastAccumalatedTrans[i]]-transmitionTimes[i][lastAccumalatedTrans[i]-3])/1000000.0f;
        //basically copied from odrive
        distEstimate[i]+=velEstimate[i]*dT;
        float delta_pos = distRaw[i] - distEstimate[i];
        distEstimate[i] += dT * pll_kp_ * delta_pos;
        velEstimate[i] += dT * pll_ki_ * delta_pos;
        lastAccumalatedTrans[i]+=3;
      }
    }
    if(updatedSinceTotalAccum[0] && updatedSinceTotalAccum[1]){
      robotAccum();
    }
    delay(2);
  }
  broadcastOdomOverCAN();
  resetRobotAccum();
  nextTargetMs+=50; //20hz
}
