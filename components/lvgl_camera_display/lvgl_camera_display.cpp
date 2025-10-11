#include "lvgl_camera_display.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace lvgl_camera_display {

static const char *const TAG = "lvgl_camera_display";

void LVGLCameraDisplay::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Configuration LVGL Camera Display...");
  
  if (this->camera_ == nullptr) {
    ESP_LOGE(TAG, "❌ Camera non configurée");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "✅ LVGL Camera Display initialisé");
  ESP_LOGI(TAG, "   Update interval: %u ms (~%d FPS)", 
           this->update_interval_, 1000 / this->update_interval_);
  ESP_LOGI(TAG, "   ⚠️  Canvas sera configuré via on_boot lambda");
}

void LVGLCameraDisplay::loop() {
  uint32_t now = millis();
  
  // Vérifier si c'est le moment de mettre à jour
  if (now - this->last_update_ < this->update_interval_) {
    return;
  }
  
  this->last_update_ = now;
  
  // Vérifier que le canvas est configuré
  if (this->canvas_obj_ == nullptr) {
    if (!this->canvas_warning_shown_) {
      ESP_LOGW(TAG, "❌ Canvas non configuré - utilisez on_boot pour appeler configure_canvas()");
      this->canvas_warning_shown_ = true;
    }
    return;
  }
  
  // Si la caméra est en streaming, capturer ET mettre à jour le canvas
  if (this->camera_->is_streaming()) {
    bool frame_captured = this->camera_->capture_frame();
    
    if (frame_captured) {
      this->update_canvas_();
      this->frame_count_++;
      
      // Logger FPS réel toutes les 100 frames
      if (this->frame_count_ % 100 == 0) {
        static uint32_t last_time = 0;
        uint32_t now_time = millis();
        if (last_time > 0) {
          float elapsed = (now_time - last_time) / 1000.0f;
          float fps = 100.0f / elapsed;
          ESP_LOGI(TAG, "🎞️  Frames: %u - FPS moyen: %.2f - Dropped: %u", 
                   this->frame_count_, fps, this->dropped_frames_);
        }
        last_time = now_time;
      }
    } else {
      this->dropped_frames_++;
    }
  }
}

void LVGLCameraDisplay::dump_config() {
  ESP_LOGCONFIG(TAG, "LVGL Camera Display:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  FPS cible: ~%d", 1000 / this->update_interval_);
  ESP_LOGCONFIG(TAG, "  Canvas configuré: %s", this->canvas_obj_ ? "OUI" : "NON");
  ESP_LOGCONFIG(TAG, "  Caméra liée: %s", this->camera_ ? "OUI" : "NON");
  
  if (this->camera_) {
    ESP_LOGCONFIG(TAG, "  Résolution caméra: %ux%u", 
                  this->camera_->get_image_width(), 
                  this->camera_->get_image_height());
  }
}

void LVGLCameraDisplay::update_canvas_() {
  if (this->camera_ == nullptr || this->canvas_obj_ == nullptr) {
    return;
  }
  
  uint8_t* img_data = this->camera_->get_image_data();
  uint16_t width = this->camera_->get_image_width();
  uint16_t height = this->camera_->get_image_height();
  
  if (img_data == nullptr) {
    ESP_LOGW(TAG, "⚠️  Données image nulles");
    return;
  }
  
  if (this->first_update_) {
    ESP_LOGI(TAG, "🖼️  Premier update canvas:");
    ESP_LOGI(TAG, "   Dimensions: %ux%u", width, height);
    ESP_LOGI(TAG, "   Buffer: %p", img_data);
    ESP_LOGI(TAG, "   Premiers pixels (RGB565): %02X%02X %02X%02X %02X%02X", 
             img_data[0], img_data[1], img_data[2], img_data[3], img_data[4], img_data[5]);
    
    // Vérifier que les dimensions du canvas correspondent
    lv_coord_t canvas_w = lv_obj_get_width(this->canvas_obj_);
    lv_coord_t canvas_h = lv_obj_get_height(this->canvas_obj_);
    
    if (canvas_w != width || canvas_h != height) {
      ESP_LOGW(TAG, "⚠️  Dimensions canvas (%dx%d) != caméra (%dx%d)", 
               canvas_w, canvas_h, width, height);
    }
    
    this->first_update_ = false;
  }
  
  // Mettre à jour le buffer du canvas
  lv_canvas_set_buffer(this->canvas_obj_, img_data, width, height, LV_IMG_CF_TRUE_COLOR);
  lv_obj_invalidate(this->canvas_obj_);
}

void LVGLCameraDisplay::configure_canvas(lv_obj_t *canvas) { 
  if (canvas == nullptr) {
    ESP_LOGE(TAG, "❌ Canvas fourni est NULL");
    return;
  }
  
  this->canvas_obj_ = canvas;
  ESP_LOGI(TAG, "🎨 Canvas configuré: %p", canvas);
  
  lv_coord_t w = lv_obj_get_width(canvas);
  lv_coord_t h = lv_obj_get_height(canvas);
  ESP_LOGI(TAG, "   Taille canvas: %dx%d", w, h);
  
  if (this->camera_) {
    uint16_t cam_w = this->camera_->get_image_width();
    uint16_t cam_h = this->camera_->get_image_height();
    
    if (w != cam_w || h != cam_h) {
      ESP_LOGW(TAG, "⚠️  ATTENTION: Dimensions incompatibles!");
      ESP_LOGW(TAG, "   Canvas: %dx%d, Caméra: %dx%d", w, h, cam_w, cam_h);
    }
  }
  
  // Reset du flag d'avertissement
  this->canvas_warning_shown_ = false;
  
  ESP_LOGI(TAG, "✅ Canvas prêt pour l'affichage");
}

}  // namespace lvgl_camera_display
}  // namespace esphome
