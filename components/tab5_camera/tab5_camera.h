#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
// Forward declaration
struct esp_cam_sensor_device_t;

extern "C" {
  #include "esp_cam_ctlr.h"
  #include "esp_cam_ctlr_csi.h"
  #include "driver/isp.h"
  #include "esp_ldo_regulator.h"
}
#endif

namespace esphome {
namespace tab5_camera {

enum CameraResolution {
    RESOLUTION_SVGA = 0,  // 800x640 uniquement
};

enum PixelFormat {
  PIXEL_FORMAT_RGB565 = 0,
  PIXEL_FORMAT_YUV422 = 1,
  PIXEL_FORMAT_RAW8 = 2,
  PIXEL_FORMAT_JPEG = 3,
};

struct CameraResolutionInfo {
  uint16_t width;
  uint16_t height;
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_flip_mirror(bool enable) { this->flip_mirror_ = enable; }

  // Configuration
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_pwdn_pin(GPIOPin *pin) { this->pwdn_pin_ = pin; }
  void set_sensor_address(uint8_t address) { this->sensor_address_ = address; }
  void set_resolution(CameraResolution resolution) { this->resolution_ = resolution; }
  void set_pixel_format(PixelFormat format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }
  void set_i2c_pins(uint8_t scl, uint8_t sda) { this->i2c_scl_pin_ = scl; this->i2c_sda_pin_ = sda; }



  // Opérations
  bool capture_frame();
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return this->streaming_; }
  
  // Accès données
  uint8_t* get_image_data() { return this->current_frame_buffer_; }
  size_t get_image_size() const { return this->frame_buffer_size_; }
  uint16_t get_image_width() const;
  uint16_t get_image_height() const;

 protected:
  uint8_t external_clock_pin_{36};  // Numéro de GPIO pour CAM_MCLK
  uint32_t external_clock_frequency_{24000000};
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *pwdn_pin_{nullptr};
  uint8_t sensor_address_{0x36};
  uint8_t i2c_scl_pin_{8};  // CAM_SCL
  uint8_t i2c_sda_pin_{7};  // CAM_SDA
  std::string name_{"Tab5 Camera"};
  
  CameraResolution resolution_{RESOLUTION_SVGA};
  PixelFormat pixel_format_{PIXEL_FORMAT_RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{30};

  bool flip_mirror_{false};

  void apply_manual_white_balance_();

  
  

  
  bool initialized_{false};
  bool streaming_{false};
  bool frame_ready_{false};
  
  uint8_t *frame_buffers_[2]{nullptr, nullptr};
  uint8_t *current_frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};
  uint8_t buffer_index_{0};
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  esp_cam_sensor_device_t *sensor_device_{nullptr};
  esp_cam_ctlr_handle_t csi_handle_{nullptr};
  isp_proc_handle_t isp_handle_{nullptr};
  esp_ldo_channel_handle_t ldo_handle_{nullptr};
  
  bool init_sensor_();
  bool init_ldo_();
  bool init_csi_();
  bool init_isp_();
  bool allocate_buffer_();
  void configure_isp_color_correction_();  // Nouvelle fonction
  CameraResolutionInfo get_resolution_info_() const;
  
  static bool IRAM_ATTR on_csi_new_frame_(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data
  );
  
  static bool IRAM_ATTR on_csi_frame_done_(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data
  );
#endif
};

}  // namespace tab5_camera
}  // namespace esphome
