#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include "MAX30105.h"
#include "heartRate.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// long prevDisplay = 0;
int circularBuffer[SCREEN_WIDTH - 16];
int curWriteIndex = 0;
int xPos = 0;
int yPos = 0;
int bpm = 0;
int lineHeight = 0;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                    // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED

  u8g2.begin();

  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_helvB14_te);
    u8g2.drawStr(26, 14 + 12, "Medical");
    u8g2.drawStr(48, 36 + 12, "Pro");
    delay(64);

  } while (u8g2.nextPage());
  delay(2000);
}

void loop(void)
{
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true)
  {
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
      rateSpot %= RATE_SIZE;                    // Wrap variable

      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  if (irValue < 50000)
    Serial.print(" No finger?");
  Serial.println();

  circularBuffer[curWriteIndex++] = (int)beatsPerMinute;
  // Set the circular buffer index back to zero when it reaches the right of the screen
  if (curWriteIndex >= (SCREEN_WIDTH - 16))
  {
    curWriteIndex = 0;
  }

  bpm = circularBuffer[curWriteIndex];
  lineHeight = map(bpm, 0, 255, 0, 40);
  // Serial.printf("BPM= %d, LH: %d\n", bpm, lineHeight);
  // Serial.printf("xPos= %d, yPos: %d\n", curWriteIndex, lineHeight);

  // for (int i = 0; i < _curWriteIndex; i++)
  // {
  //   int beatsPerMinute = _circularBuffer[i];
  //   int lineHeight = map(beatsPerMinute, 20, 255, 0, SCREEN_HEIGHT);
  //   int yPos = SCREEN_HEIGHT - lineHeight;
  //   u8g2.drawVLine(xPos, yPos, lineHeight);
  //   xPos++;
  // }

  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_helvB08_te);
    u8g2.drawStr(8, 16, ("BPM " + String(bpm) + "    AVG " + String(beatAvg)).c_str());

    xPos = 0;
    for (int i = curWriteIndex; i < (SCREEN_WIDTH - 16); i++)
    {
      u8g2.drawPixel((xPos + 8), (SCREEN_HEIGHT - 8) - lineHeight);
      xPos++;
    }
    for (int i = 0; i < curWriteIndex; i++)
    {
      u8g2.drawPixel((xPos + 8), (SCREEN_HEIGHT - 8) - lineHeight);
      xPos++;
    }

  } while (u8g2.nextPage());
}