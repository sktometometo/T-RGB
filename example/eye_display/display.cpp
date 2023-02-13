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

XL9535 xl;

void setup()
{
  // put your setup code here, to run once:
  pinMode(BAT_VOLT_PIN, ANALOG);

  Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t)400000);
  Serial.begin(115200);
  xl.begin();
  uint8_t pin = (1 << PWR_EN_PIN) | (1 << LCD_CS_PIN) | (1 << TP_RES_PIN) | (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) |
                (1 << LCD_RST_PIN) | (1 << SD_CS_PIN);

  xl.pinMode8(0, pin, OUTPUT);
  xl.digitalWrite(PWR_EN_PIN, 1);
  print_chip_info();
  pinMode(EXAMPLE_PIN_NUM_BK_LIGHT, OUTPUT);
  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

  tft_init();
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_rgb_panel_config_t panel_config = {
      .clk_src = LCD_CLK_SRC_PLL160M,
      .timings =
          {
              .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
              .h_res = EXAMPLE_LCD_H_RES,
              .v_res = EXAMPLE_LCD_V_RES,
              // The following parameters should refer to LCD spec
              .hsync_pulse_width = 1,
              .hsync_back_porch = 30,
              .hsync_front_porch = 50,
              .vsync_pulse_width = 1,
              .vsync_back_porch = 30,
              .vsync_front_porch = 20,
              .flags =
                  {
                      .pclk_active_neg = 1,
                  },
          },
      .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
      .psram_trans_align = 64,
      .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
      .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
      .de_gpio_num = EXAMPLE_PIN_NUM_DE,
      .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
      .data_gpio_nums =
          {
              // EXAMPLE_PIN_NUM_DATA0,
              EXAMPLE_PIN_NUM_DATA13,
              EXAMPLE_PIN_NUM_DATA14,
              EXAMPLE_PIN_NUM_DATA15,
              EXAMPLE_PIN_NUM_DATA16,
              EXAMPLE_PIN_NUM_DATA17,

              EXAMPLE_PIN_NUM_DATA6,
              EXAMPLE_PIN_NUM_DATA7,
              EXAMPLE_PIN_NUM_DATA8,
              EXAMPLE_PIN_NUM_DATA9,
              EXAMPLE_PIN_NUM_DATA10,
              EXAMPLE_PIN_NUM_DATA11,
              // EXAMPLE_PIN_NUM_DATA12,

              EXAMPLE_PIN_NUM_DATA1,
              EXAMPLE_PIN_NUM_DATA2,
              EXAMPLE_PIN_NUM_DATA3,
              EXAMPLE_PIN_NUM_DATA4,
              EXAMPLE_PIN_NUM_DATA5,
          },
      .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
      .on_frame_trans_done = NULL,
      .user_ctx = NULL,
      .flags =
          {
              .fb_in_psram = 1, // allocate frame buffer in PSRAM
          },
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 480, logo_img);
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(2);
}
