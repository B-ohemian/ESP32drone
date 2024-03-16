#include "ledc_motor.h"

void ledc_motor_init(void)
{
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = LEDC_DUTY_RES,//更改分辨率
      .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
      .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL,
      .timer_sel = LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = LEDC_OUTPUT_IO,
      .duty = 0, // Set duty to 0%
      .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel2 = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL2,
      .timer_sel = LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = LEDC_OUTPUT_IO2,
      .duty = 0, // Set duty to 0%
      .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel3 = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL3,
      .timer_sel = LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = LEDC_OUTPUT_IO3,
      .duty = 0, // Set duty to 0%
      .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel3));
  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel4 = {
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL4,
      .timer_sel = LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = LEDC_OUTPUT_IO4,
      .duty = 0, // Set duty to 0%
      .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel4));
}

void ledc_motor_run(uint16_t motor1_duty, uint16_t motor2_duty,uint16_t motor3_duty,uint16_t motor4_duty)
{
  // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, motor1_duty));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, motor2_duty));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL3, motor3_duty));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL4, motor4_duty));

  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL3));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL4));
}

