#include "softi2c.h"
#include "SSD1306.h"

void setup() {
  // put your setup code here, to run once:
  // I2C_Init();
  I2C_Init();
  ssd1306_begin();

  clearDisplay();
  drawPixel(1, 1, SSD1306_WHITE);
  drawPixel(10, 10, SSD1306_WHITE);
  drawPixel(11, 11, SSD1306_BLACK);
  display();
}

void loop() {
  // put your main code here, to run repeatedly:

}
