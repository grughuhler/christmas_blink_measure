/* This program detects the frequency of light blinking using
 *  a phototransistor and op amp as a comparator.  It runs on
 *  an esp32.
 *  
 *  It's very basic.  The comparator drives an edge-triggered
 *  interrupt.  The time between edges gives the period of the
 *  blinking.
 *  
 *  It also captures and displays samples of the input that
 *  feeds the comparator.
 *  
 *  Hardware: ESP32, visible light phototransistor, rail-to-rail
 *  op amp that works at 3.3 V (such as MCP6002), and a 128x32
 *  SSD1306 oled display.
 *
 * License: BSD 2-Clause
 */

// Time to toggle display bewteen wave and frequency
#define UPDATE_RATE 1700

// Screen size
#define SCREEN_X 128
#define SCREEN_Y 32

// ESP32 pins used
// Pin 23 is a SPI pin.  This is OK but one might use a different pin (like 32) to free SPI for
// connection to a digital potentiometer rather than the manual trim pot used here.
#define IRQ_PIN 23
#define BUTTON_PIN 0

// ADC
#define LIGHT_PIN 36
#define THRESHOLD_PIN 39


// Two periods of a 60Hz signal, SCREEN_X total samples,
// so who screen shows 33.2 ms of time.
#define DEF_SAMPLE_PERIOD (2*1000000/60/SCREEN_X)
// Don't try to see two cycles of anything less than 2 Hz.
#define MIN_SAMPLE_PERIOD (2*1000000/2/SCREEN_X)

#include "SSD1306Wire.h"
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);

float period = 0.0;
volatile unsigned int irq_count = 0;
unsigned int sample_index = 0;
unsigned int samples[SCREEN_X];

#define TIMEBASE_120 0
#define TIMEBASE_2PERIODS 1
unsigned int timebase_mode = TIMEBASE_120;

// Button on module switches bewteen timebase values. Switch does not bounce too much,
// but debounce would be a good idea.

void IRAM_ATTR button(void)
{
   timebase_mode = 1 - timebase_mode; 
}

void IRAM_ATTR edge_detect(void)
{
  unsigned long now;
  static unsigned long last;
  static unsigned int loc_irq_count = 0;
  
  now = micros();
 
  period = (now - last)/1000000.0;
  last = now;

  loc_irq_count += 1;
  irq_count = loc_irq_count;
}

// Collect samples to display
float collect_samples(unsigned int sample_period)
{
  unsigned int i, val;
  unsigned long next_sample_time, now;
  float fval, vmin = 16.0, vmax = -16.0;

  // Don't sample too slowly
  if (sample_period > MIN_SAMPLE_PERIOD) sample_period = MIN_SAMPLE_PERIOD;

  now = micros();
  for (i = 0; i < SCREEN_X; i++) {
    next_sample_time = now + sample_period;
    val = analogRead(LIGHT_PIN);
    samples[i] = val/128;
    fval = val*3.3/4096.0;
    if (fval < vmin) vmin = fval;
    if (fval > vmax) vmax = fval;
    while ((now = micros()) < next_sample_time) {}
  }

  return (vmax + vmin)/2.0;
}

void display_samples()
{
  unsigned int i;
  
  display.clear();
  for (i = 0; i < SCREEN_X; i++) {
     display.setPixel(127 - i ,31 - samples[i]);
  }
  display.display(); 
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("Welcome to freq");

  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  attachInterrupt(IRQ_PIN, edge_detect, RISING);
  attachInterrupt(BUTTON_PIN, button, FALLING);
}

boolean await_irq(unsigned int timeout_ms)
{
  unsigned int start = millis(), last_irq_count = irq_count, now;

  while (irq_count == last_irq_count) {
    if ((now = millis()) > start + timeout_ms) {
      /* no interrupt so no edge detected */
      return false;
    }
  }

  if (period > 0.0) 
      return true;
  else
      return false;
}

void loop()
{
   unsigned int sample_period;
   float mid;

   display.setFont(ArialMT_Plain_24);
   
   if (await_irq(500)) {
      // Display a period
      if (timebase_mode == TIMEBASE_2PERIODS)
         sample_period = 2*1000000*period/SCREEN_X;
      else
         sample_period = DEF_SAMPLE_PERIOD;
      mid = collect_samples(sample_period);
      Serial.print(1.0/period); Serial.print(" ");
      display.clear();
      display.drawString(0, 0, String((int)(1.0/period+0.5)) + " Hz");
      display.setFont(ArialMT_Plain_10);
      if (timebase_mode == TIMEBASE_2PERIODS)
         display.drawString(0, 22, "TB: auto");
      else
         display.drawString(0, 22, "TB: 33ms");
   } else {
      mid = collect_samples(DEF_SAMPLE_PERIOD);
      Serial.print("No edge ");
      display.clear();
      display.drawString(0, 0, "NO EDGE");
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 22, "TB: 33ms");
   }
   display.drawString(50, 22, "M: " + String(mid) + ", " + "T: " + String(analogRead(THRESHOLD_PIN)*3.3/4096.0));
   display.display();

   Serial.println(irq_count);
   delay(UPDATE_RATE);
   display_samples();
   delay(UPDATE_RATE);
}
