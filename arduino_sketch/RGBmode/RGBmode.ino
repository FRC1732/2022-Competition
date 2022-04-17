#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN1 11 // On Trinket or Gemma, suggest changing this to 1
#define PIN2 10 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 10
#define DELAY_TIME 200
#define INTENSITY 150

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2(NUMPIXELS, PIN2, NEO_GRB + NEO_KHZ800);

#define DIGITAL_ZERO 2
#define DIGITAL_ONE 3
#define DIGITAL_TWO 4

int mode = 0;
int timer = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  pinMode(DIGITAL_ZERO, INPUT);
  pinMode(DIGITAL_ONE, INPUT);
  pinMode(DIGITAL_TWO, INPUT);

  pixels.begin();
  pixels2.begin();
}

void loop() {
  mode = (int)(digitalRead(DIGITAL_ZERO)) + (int)(digitalRead(DIGITAL_ONE) << 1) + (int)(digitalRead(DIGITAL_TWO) << 2);
  Serial.print("Mode: ");
  Serial.print(mode);
  Serial.print(", DIO 0: ");
  Serial.print(digitalRead(DIGITAL_ZERO));
  Serial.print(", DIO 1: ");
  Serial.print(digitalRead(DIGITAL_ONE));
  Serial.print(", DIO 2: ");
  Serial.println(digitalRead(DIGITAL_TWO));
  

  if (mode == 0) // nothing
  {
    setColor(false, false, false);
  }

  if (mode == 1) // auto/ 1 ball
  {
    blinkColor(false, false, true);
  }

  if (mode == 2) // auto/ 2 ball
  {
    blinkColor(true, true, false);
  }

  if (mode == 3) // tele / 1 ball
  {
    setColor(false, false, true);
  }

  if (mode == 4) // tele / 2 ball
  {
    setColor(true, true, false);
  }

  if (mode == 5) // tele / aligning
  {
    setColor(true, false, false);
  }

  if (mode == 6) // tele / shooting
  {
    flashFast(false, true, false);
  }

  if (mode == 7) // party mode
  {
    setColor(true, true, true);
  }
  timer++;
  delay(1);
  //  setColor(false, false, false);
}

void setColor(bool red, bool green, bool blue)
{
  pixels.clear();
  pixels2.clear();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
    pixels2.setPixelColor(i, pixels2.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
  }
  pixels.show();
  pixels2.show();
}


void blinkColor(bool red, bool green, bool blue)
{
  if (timer < 150)
  {
    setColor(red, green, blue);
  }
  if (timer < 300 && timer > 150)
  {
    setColor(false, false, false);
  }
  if (timer > 301)
  {
    timer = 0;
  }
}

void flashFast(bool red, bool green, bool blue)
{
  if (timer < 60)
  {
    setColor(red, green, blue);
  }
  if (timer < 120 && timer > 60)
  {
    setColor(false, false, false);
  }
  if (timer > 121)
  {
    timer = 0;
  }
}
