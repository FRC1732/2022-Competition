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

#define DIGITAL_ONE 3
#define DIGITAL_TWO 4
#define DIGITAL_THREE 5

int mode = 0;
int timer = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  pinMode(DIGITAL_ONE, INPUT);
  pinMode(DIGITAL_TWO, INPUT);
  pinMode(DIGITAL_THREE, INPUT);

  pixels.begin();
  pixels2.begin();
}

void loop() {
  mode = (int)(digitalRead(DIGITAL_ONE)) | (int)(digitalRead(DIGITAL_TWO) << 1) | (int)(digitalRead(DIGITAL_THREE) << 2);
  Serial.println(mode);

  if (mode == 0)
  {
    setColor(false, false, false);
  }

  if (mode == 1)
  {
    blinkColor(false, false, true);
  }

  if (mode == 2)
  {
    blinkColor(true, true, false);
  }

  if (mode == 3)
  {
    setColor(false, false, true);
  }

  if (mode == 4)
  {
    setColor(true, true, false);
  }

  if (mode == 5)
  {
    setColor(true, false, false);
  }

  if (mode == 6)
  {
    flashFast(false, true, false);
  }

  if (mode == 7)
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
