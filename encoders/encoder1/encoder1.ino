#include <Arduino.h>
#include <Servo.h>
#include <digitalWriteFast.h>
// product website:https://www.sparkfun.com/rotary-encoder-1024-p-r-quadrature.html#content-documentation
// One encoder
#define EncoderPinA 18   // black wire
#define EncoderPinB 17   // white wire

volatile bool EncoderBSet;
volatile long EncoderTicks = 0;

Servo RightServo;
Servo LeftServo;

int potpin = A0;
int val;

void setup()
{
  Serial.begin(115200);

  RightServo.attach(2);
  LeftServo.attach(3);

  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(EncoderPinA),
    HandleEncoderInterruptA,
    RISING
  );
}

void loop()
{
  val = analogRead(potpin);
  val = map(val, 0, 1023, 0, 179);

  RightServo.write(val);
  LeftServo.write(val);

  noInterrupts();
  long ticksCopy = EncoderTicks;
  interrupts();

  Serial.println(ticksCopy);

  delay(20);
}

void HandleEncoderInterruptA()
{
  EncoderBSet = digitalReadFast(EncoderPinB);

  EncoderTicks += EncoderBSet ? -1 : +1;
}