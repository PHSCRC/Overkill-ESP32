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
#define MDATA 5
#define MCLK 6

bool activateInterupts=false;

inline uint8_t extractByte(uint16_t inByte){
  return (uint8_t)(inByte>>1);
}

int16_t extractDataX(uint16_t infoRaw, uint16_t dataRaw){
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

int16_t extractDataY(uint16_t infoRaw, uint16_t dataRaw){
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
void gohi(int pin)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void golo(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void mouse_write(char data)
{
  char i;
  char parity = 1;

  //  Serial.print("Sending ");
  //  Serial.print(data, HEX);
  //  Serial.print(" to mouse\n");
  //  Serial.print("RTS");
  /* put pins in output mode */
  gohi(MDATA);
  gohi(MCLK);
  delayMicroseconds(300);
  golo(MCLK);
  delayMicroseconds(300);
  golo(MDATA);
  delayMicroseconds(10);
  /* start bit */
  gohi(MCLK);
  /* wait for mouse to take control of clock); */
  while (digitalRead(MCLK) == HIGH)
    ;
  /* clock is low, and we are clear to send data */
  for (i=0; i < 8; i++) {
    if (data & 0x01) {
      gohi(MDATA);
    }
    else {
      golo(MDATA);
    }
    /* wait for clock cycle */
    while (digitalRead(MCLK) == LOW)
      ;
    while (digitalRead(MCLK) == HIGH)
      ;
    parity = parity ^ (data & 0x01);
    data = data >> 1;
  }
  /* parity */
  if (parity) {
    gohi(MDATA);
  }
  else {
    golo(MDATA);
  }
  while (digitalRead(MCLK) == LOW)
    ;
  while (digitalRead(MCLK) == HIGH)
    ;
  /* stop bit */
  gohi(MDATA);
  delayMicroseconds(50);
  while (digitalRead(MCLK) == HIGH)
    ;
  /* wait for mouse to switch modes */
  while ((digitalRead(MCLK) == LOW) || (digitalRead(MDATA) == LOW))
    ;
  /* put a hold on the incoming data. */
  golo(MCLK);
  //  Serial.print("done.\n");
}

/*
 * Get a byte of data from the mouse
 */
char mouse_read(void)
{
  char data = 0x00;
  int i;
  char bit = 0x01;

  //  Serial.print("reading byte from mouse\n");
  /* start the clock */
  gohi(MCLK);
  gohi(MDATA);
  delayMicroseconds(50);
  while (digitalRead(MCLK) == HIGH)
    ;
  delayMicroseconds(5);  /* not sure why */
  while (digitalRead(MCLK) == LOW) /* eat start bit */
    ;
  for (i=0; i < 8; i++) {
    while (digitalRead(MCLK) == HIGH)
      ;
    if (digitalRead(MDATA) == HIGH) {
      data = data | bit;
    }
    while (digitalRead(MCLK) == LOW)
      ;
    bit = bit << 1;
  }
  /* eat parity bit, which we ignore */
  while (digitalRead(MCLK) == HIGH)
    ;
  while (digitalRead(MCLK) == LOW)
    ;
  /* eat stop bit */
  while (digitalRead(MCLK) == HIGH)
    ;
  while (digitalRead(MCLK) == LOW)
    ;

  /* put a hold on the incoming data. */
  golo(MCLK);
  //  Serial.print("Recvd data ");
  //  Serial.print(data, HEX);
  //  Serial.print(" from mouse\n");
  return data;
}

void mouse_init()
{
  gohi(MCLK);
  gohi(MDATA);
  //  Serial.print("Sending reset to mouse\n");
  mouse_write(0xff);
  mouse_read();  // ack byte
  mouse_read();  //self test successful
  mouse_read();  //mouse id
  mouse_write(0xf3); //set sample rate
  mouse_read();
  mouse_write(0xc8); //set the sample rate as high as the ps/2 protocol can go
  mouse_read();
  mouse_write(0xe8); //set counts/mm
  mouse_read();
  mouse_write(0x03); //set it at 8/mm (prob not followed) but still tells it go as high as it can go.
  mouse_read();
  gohi(MCLK);
  activateInterupts=true;
  mouse_write(0xf4); //start streaming.
  mouse_read();

  //  Serial.print("Sending remote mode code\n");
  //should now be in stream mode
  /*
  mouse_write(0xf0); //  remote mode
  mouse_read();  // ack
  //  Serial.print("Read ack byte2\n");
  */
}
volatile uint16_t fullTransmitionBits[256];
volatile uint64_t transmitionTimes[256];
volatile uint8_t transNum=0;
volatile uint8_t bitNumInTransmition=0;

inline void generalISR(uint8_t dataPin){
   if(activateInterupts){
    if(bitNumInTransmition==0){
      transmitionTimes[transNum]=micros();
    }
    fullTransmitionBits[transNum]|=digitalRead(dataPin)<<bitNumInTransmition;
    if(bitNumInTransmition==10){
      bitNumInTransmition=0;
      ++transNum;
    } else {
      ++bitNumInTransmition;
    }
  }
}

void IRAM_ATTR CLKFALLING() {
 generalISR(MDATA);
}


const float countsTomm=0.125f;



void setup()
{
  Serial.begin(9600);
  attachInterrupt(MCLK, CLKFALLING, FALLING);
  mouse_init();
}

/*
 * get a reading from the mouse and report it back to the
 * host via the serial line.
 */
uint8_t lastAccumalatedTrans=0;
uint32_t nextTargetMs;
uint32_t accumalatedCountX;
uint32_t accumalatedCountY;
void broadcastOdomOverCAN(){
  Serial.println(accumalatedCountX, accumalatedCountY);
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
    if(transNum-lastAccumalatedTrans>=3){
      accumalatedCountX+=extractDataX(fullTransmitionBits[lastAccumalatedTrans], fullTransmitionBits[lastAccumalatedTrans+1]);
      accumalatedCountY+=extractDataY(fullTransmitionBits[lastAccumalatedTrans], fullTransmitionBits[lastAccumalatedTrans+2]);
      lastAccumalatedTrans+=3;
    }
    delay(2);
  }
  broadcastOdomOverCAN();
  accumalatedCountX=0;
  accumalatedCountY=0;
  //20 hz
  nextTargetMs+=50;
}
