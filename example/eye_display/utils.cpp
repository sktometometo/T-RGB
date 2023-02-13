#include "Wire.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "XL9535_driver.h"
#include "T-RGB/sample_img.h"
#include "T-RGB/pin_config.h"
#include "utils.h"
#include <Arduino.h>

extern XL9535 xl;

void scan_iic(void)
{
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found");
  }
}

void print_chip_info(void)
{
  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());
  Serial.print("ChipRevision: ");
  Serial.println(ESP.getChipRevision());
  Serial.print("Psram size: ");
  Serial.print(ESP.getPsramSize() / 1024);
  Serial.println("KB");
  Serial.print("Flash size: ");
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println("KB");
  Serial.print("CPU frequency: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println("MHz");
}

void lcd_cmd(const uint8_t cmd)
{
  xl.digitalWrite(LCD_CS_PIN, 0);
  xl.digitalWrite(LCD_SDA_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 1);
  lcd_send_data(cmd);
  xl.digitalWrite(LCD_CS_PIN, 1);
}

void lcd_data(const uint8_t* data, int len)
{
  uint32_t i = 0;
  if (len == 0)
    return;  // no need to send anything
  do
  {
    xl.digitalWrite(LCD_CS_PIN, 0);
    xl.digitalWrite(LCD_SDA_PIN, 1);
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(*(data + i));
    xl.digitalWrite(LCD_CS_PIN, 1);
    i++;
  } while (len--);
}

void lcd_send_data(uint8_t data)
{
  uint8_t n;
  for (n = 0; n < 8; n++)
  {
    if (data & 0x80)
      xl.digitalWrite(LCD_SDA_PIN, 1);
    else
      xl.digitalWrite(LCD_SDA_PIN, 0);

    data <<= 1;
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
  }
}

void tft_init(void)
{
  xl.digitalWrite(LCD_CS_PIN, 1);
  xl.digitalWrite(LCD_SDA_PIN, 1);
  xl.digitalWrite(LCD_CLK_PIN, 1);

  // Reset the display
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  int cmd = 0;
  while (st_init_cmds[cmd].databytes != 0xff)
  {
    lcd_cmd(st_init_cmds[cmd].cmd);
    lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
    if (st_init_cmds[cmd].databytes & 0x80)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    cmd++;
  }
  Serial.println("Register setup complete");
}
