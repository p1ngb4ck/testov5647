#pragma once

#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_esphome.h"
#include "../tab5_camera/tab5_camera.h"

namespace esphome {
namespace lvgl_camera_display {

class LVGLCameraDisplay : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  void set_camera(tab5_camera::Tab5Camera *camera) { this->camera_ = camera; }
  void set_update_interval(uint32_t interval_ms) { this->update_interval_ = interval_ms; }
  
  // Configuration du canvas - à appeler depuis YAML on_boot
  void configure_canvas(lv_obj_t *canvas);
  
  // Vérifier si le canvas est configuré
  bool is_canvas_configured() const { return this->canvas_obj_ != nullptr; }
  
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  tab5_camera::Tab5Camera *camera_{nullptr};
  lv_obj_t *canvas_obj_{nullptr};
  
  uint32_t update_interval_{20};  // 50 FPS par défaut
  uint32_t last_update_{0};
  
  // Statistiques
  uint32_t frame_count_{0};
  uint32_t dropped_frames_{0};
  bool first_update_{true};
  bool canvas_warning_shown_{false};
  
  void update_canvas_();
};

}  // namespace lvgl_camera_display
}  // namespace esphome
