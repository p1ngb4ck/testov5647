#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

// ============================================================================
// HEADERS ESP-IDF NÉCESSAIRES
// ============================================================================

extern "C" {
#include "driver/i2c_master.h"
}

// ============================================================================
// DÉFINITIONS COMPLÈTES DES STRUCTURES CAMERA SENSOR
// ============================================================================

extern "C" {

// Type pour SCCB handle
typedef struct esp_sccb_io_t* esp_sccb_io_handle_t;

// Structure SCCB IO config
typedef struct {
    i2c_addr_bit_len_t dev_addr_length;
    uint16_t device_address;
    uint32_t scl_speed_hz;
    uint32_t addr_bits_width;
    uint32_t val_bits_width;
} sccb_i2c_config_t;

// Structure pour l'ID du capteur
typedef struct {
    uint8_t midh;
    uint8_t midl;
    uint16_t pid;
    uint8_t ver;
} esp_cam_sensor_id_t;

// Type de port du capteur
typedef enum {
    ESP_CAM_SENSOR_DVP,
    ESP_CAM_SENSOR_MIPI_CSI,
} esp_cam_sensor_port_t;

// Forward declaration pour les opérations
struct esp_cam_sensor_device_t;
typedef struct esp_cam_sensor_ops_t {
    int (*set_format)(struct esp_cam_sensor_device_t *dev, const void *format);
    int (*get_format)(struct esp_cam_sensor_device_t *dev, void *format);
    int (*priv_ioctl)(struct esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg);
    int (*del)(struct esp_cam_sensor_device_t *dev);
} esp_cam_sensor_ops_t;

// Structure principale du device
typedef struct esp_cam_sensor_device_t {
    char *name;
    esp_sccb_io_handle_t sccb_handle;
    int8_t xclk_pin;
    int8_t reset_pin;
    int8_t pwdn_pin;
    esp_cam_sensor_port_t sensor_port;
    const void *cur_format;
    esp_cam_sensor_id_t id;
    uint8_t stream_status;
    const esp_cam_sensor_ops_t *ops;
    void *priv;
} esp_cam_sensor_device_t;

// Configuration du capteur
typedef struct {
    esp_sccb_io_handle_t sccb_handle;
    int8_t reset_pin;
    int8_t pwdn_pin;
    int8_t xclk_pin;
    int32_t xclk_freq_hz;
    esp_cam_sensor_port_t sensor_port;
} esp_cam_sensor_config_t;

// Type pour detect function
typedef struct {
    union {
        esp_cam_sensor_device_t *(*detect)(void *);
        esp_cam_sensor_device_t *(*fn)(void *);
    };
    esp_cam_sensor_port_t port;
    uint16_t sccb_addr;
} esp_cam_sensor_detect_fn_t;

// Structure pour le format du capteur
typedef struct {
    const char *name;
    uint32_t format;
    esp_cam_sensor_port_t port;
    int xclk;
    uint16_t width;
    uint16_t height;
    const void *regs;
    int regs_size;
    uint8_t fps;
    const void *isp_info;
    void *reserved;
} esp_cam_sensor_format_t;

// ============================================================================
// CODE COMPLETE DU DRIVER OV54647 INTEGRATED
// ============================================================================

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

// Types OV5647
typedef struct {
    uint16_t reg;
    uint8_t val;
} ov5647_reginfo_t;

// Registres ov5647
#define OV5647_REG_DELAY            0xeeee
#define OV5647_REG_END              0xffff
#define OV5647_REG_SENSOR_ID_H      0x300a
#define OV5647_REG_SENSOR_ID_L      0x300b
#define OV5647_REG_SLEEP_MODE       0x0100
#define OV5647_REG_MIPI_CTRL00      0x4800
#define OV5647_REG_FRAME_OFF_NUMBER 0x4202
#define OV5640_REG_PAD_OUT          0x300d

#define BIT(nr)                                   (1UL << (nr))
#define OV5647_IDI_CLOCK_RATE_800x800_50FPS       (100000000ULL)
#define OV5647_MIPI_CSI_LINE_RATE_800x800_50FPS   (OV5647_IDI_CLOCK_RATE_800x800_50FPS * 4)
#define OV5647_IDI_CLOCK_RATE_800x640_50FPS       (100000000ULL)
#define OV5647_MIPI_CSI_LINE_RATE_800x640_50FPS   (OV5647_IDI_CLOCK_RATE_800x640_50FPS * 4)
#define OV5647_IDI_CLOCK_RATE_800x1280_50FPS      (100000000ULL)
#define OV5647_MIPI_CSI_LINE_RATE_800x1280_50FPS  (OV5647_IDI_CLOCK_RATE_800x1280_50FPS * 4)
#define OV5647_IDI_CLOCK_RATE_1920x1080_30FPS     (81666700ULL)
#define OV5647_MIPI_CSI_LINE_RATE_1920x1080_30FPS (OV5647_IDI_CLOCK_RATE_1920x1080_30FPS * 5)
#define OV5647_IDI_CLOCK_RATE_1280x960_45FPS      (88333333ULL)
#define OV5647_MIPI_CSI_LINE_RATE_1280x960_45FPS  (OV5647_IDI_CLOCK_RATE_1280x960_45FPS * 5)
#define OV5647_8BIT_MODE                          (0x18)
#define OV5647_10BIT_MODE                         (0x1A)
#define OV5647_MIPI_CTRL00_CLOCK_LANE_GATE        BIT(5)
#define OV5647_MIPI_CTRL00_LINE_SYNC_ENABLE       BIT(4)
#define OV5647_MIPI_CTRL00_BUS_IDLE               BIT(2)
#define OV5647_MIPI_CTRL00_CLOCK_LANE_DISABLE     BIT(0)

#define OV5647_PID               0x5647
#define OV5647_SENSOR_NAME       "OV5647"
#define OV5647_AE_TARGET_DEFAULT (0x36)

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms) vTaskDelay((ms > portTICK_PERIOD_MS ? ms / portTICK_PERIOD_MS : 1))

// Configuration 800x640 RAW8 OPTIMISÉE pour 50fps
static const ov5647_reginfo_t ov5647_input_24M_MIPI_2lane_raw8_800x640_50fps[] = {
    {0x3034, OV5647_8BIT_MODE},
    {0x3035, 0x41},
    {0x3036, ((OV5647_IDI_CLOCK_RATE_800x640_50FPS * 8 * 4) / 25000000)},
    {0x303c, 0x11},
    {0x3106, 0xf5},
    {0x3821, 0x03},
    {0x3820, 0x41},
    {0x3827, 0xec},
    {0x370c, 0x0f},
    {0x3612, 0x59},
    {0x3618, 0x00},
    {0x5000, 0xff},
    {0x583e, 0xf0},
    {0x583f, 0x20},
    {0x5002, 0x41},
    {0x5003, 0x08},
    {0x5a00, 0x08},
    {0x3000, 0x00},
    {0x3001, 0x00},
    {0x3002, 0x00},
    {0x3016, 0x08},
    {0x3017, 0xe0},
    {0x3018, 0x44},
    {0x301c, 0xf8},
    {0x301d, 0xf0},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3c01, 0x80},
    {0x3c00, 0x40},
    {0x3b07, 0x0c},
    {0x380c, (1896 >> 8) & 0x1F},
    {0x380d, 1896 & 0xFF},
    {0x380e, (984 >> 8) & 0xFF},
    {0x380f, 984 & 0xFF},
    {0x3814, 0x31},
    {0x3815, 0x31},
    {0x3708, 0x64},
    {0x3709, 0x52},
    {0x3800, (500 >> 8) & 0x0F},
    {0x3801, 500 & 0xFF},
    {0x3802, (0 >> 8) & 0x07},
    {0x3803, 0 & 0xFF},
    {0x3804, ((2624 - 1) >> 8) & 0x0F},
    {0x3805, (2624 - 1) & 0xFF},
    {0x3806, ((1954 - 1) >> 8) & 0x07},
    {0x3807, (1954 - 1) & 0xFF},
    {0x3808, (800 >> 8) & 0x0F},
    {0x3809, 800 & 0xFF},
    {0x380a, (640 >> 8) & 0x7F},
    {0x380b, 640 & 0xFF},
    {0x3810, (8 >> 8) & 0x0F},
    {0x3811, 8 & 0xFF},
    {0x3812, (0 >> 8) & 0x07},
    {0x3813, 0 & 0xFF},
    {0x3630, 0x2e},
    {0x3632, 0xe2},
    {0x3633, 0x23},
    {0x3634, 0x44},
    {0x3636, 0x06},
    {0x3620, 0x64},
    {0x3621, 0xe0},
    {0x3600, 0x37},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x3731, 0x02},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3f05, 0x02},
    {0x3f06, 0x10},
    {0x3f01, 0x0a},
    {0x3a08, 0x01},
    {0x3a09, 0x27},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0d, 0x04},
    {0x3a0e, 0x03},
    {0x3a0f, 0x58},
    {0x3a10, 0x50},
    {0x3a1b, 0x58},
    {0x3a1e, 0x50},
    {0x3a11, 0x60},
    {0x3a1f, 0x28},
    {0x4001, 0x02},
    {0x4004, 0x02},
    {0x4000, 0x09},
    {0x4837, (1000000000 / (OV5647_IDI_CLOCK_RATE_800x640_50FPS / 4))},
    {0x4050, 0x6e},
    {0x4051, 0x8f},
    {OV5647_REG_END, 0x00},
};

} // extern "C"

// ============================================================================
// IMPLÉMENTATIONS SCCB ET SENSOR
// ============================================================================

struct esp_sccb_io_t {
    esphome::i2c::I2CDevice *i2c_device;
    uint32_t addr_bits_width;
    uint32_t val_bits_width;
};

// Forward declarations
esp_err_t sccb_new_i2c_io_esphome(esphome::i2c::I2CDevice *i2c_device,
                                   const sccb_i2c_config_t *config,
                                   esp_sccb_io_handle_t *io_handle);

esp_err_t esp_sccb_transmit_reg_a16v8(esp_sccb_io_handle_t handle, 
                                       uint16_t reg_addr, 
                                       uint8_t reg_val);

esp_err_t esp_sccb_transmit_receive_reg_a16v8(esp_sccb_io_handle_t handle, 
                                              uint16_t reg_addr, 
                                              uint8_t *reg_val);

esp_err_t sccb_new_i2c_io_esphome(esphome::i2c::I2CDevice *i2c_device,
                                   const sccb_i2c_config_t *config,
                                   esp_sccb_io_handle_t *io_handle) {
    if (!i2c_device || !config || !io_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_sccb_io_t *sccb = (esp_sccb_io_t*)malloc(sizeof(esp_sccb_io_t));
    if (!sccb) {
        return ESP_ERR_NO_MEM;
    }
    
    sccb->i2c_device = i2c_device;
    sccb->addr_bits_width = config->addr_bits_width ? config->addr_bits_width : 8;
    sccb->val_bits_width = config->val_bits_width ? config->val_bits_width : 8;
    
    *io_handle = sccb;
    return ESP_OK;
}

esp_err_t esp_cam_sensor_get_format(esp_cam_sensor_device_t *dev, 
                                    esp_cam_sensor_format_t *format) {
    if (!dev || !format || !dev->ops) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->ops->get_format) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_format(dev, format);
}

esp_err_t esp_cam_sensor_ioctl(esp_cam_sensor_device_t *dev, 
                               uint32_t cmd, 
                               void *arg) {
    if (!dev || !dev->ops) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->ops->priv_ioctl) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return dev->ops->priv_ioctl(dev, cmd, arg);
}

esp_err_t esp_sccb_transmit_reg_a16v8(esp_sccb_io_handle_t handle, 
                                       uint16_t reg_addr, 
                                       uint8_t reg_val) {
    if (!handle || !handle->i2c_device) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[3] = {
        (uint8_t)((reg_addr >> 8) & 0xFF),
        (uint8_t)(reg_addr & 0xFF),
        reg_val
    };
    
    esphome::i2c::ErrorCode err = handle->i2c_device->write(data, 3);
    return (err == esphome::i2c::ERROR_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_sccb_transmit_receive_reg_a16v8(esp_sccb_io_handle_t handle, 
                                              uint16_t reg_addr, 
                                              uint8_t *reg_val) {
    if (!handle || !handle->i2c_device || !reg_val) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t addr_buf[2] = {
        (uint8_t)((reg_addr >> 8) & 0xFF),
        (uint8_t)(reg_addr & 0xFF)
    };
    
    esphome::i2c::ErrorCode err = handle->i2c_device->write(addr_buf, 2);
    if (err != esphome::i2c::ERROR_OK) {
        return ESP_FAIL;
    }
    
    err = handle->i2c_device->read(reg_val, 1);
    return (err == esphome::i2c::ERROR_OK) ? ESP_OK : ESP_FAIL;
}

__attribute__((weak)) esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_start = {};
__attribute__((weak)) esp_cam_sensor_detect_fn_t __esp_cam_sensor_detect_fn_array_end = {};

extern "C" {

static const char *OV5647_TAG = "ov5647";

static esp_err_t ov5647_write(esp_sccb_io_handle_t handle, uint16_t reg, uint8_t data) {
    return esp_sccb_transmit_reg_a16v8(handle, reg, data);
}

static esp_err_t ov5647_read(esp_sccb_io_handle_t handle, uint16_t reg, uint8_t *data) {
    return esp_sccb_transmit_receive_reg_a16v8(handle, reg, data);
}

static esp_err_t ov5647_write_array(esp_sccb_io_handle_t handle, ov5647_reginfo_t *regarray) {
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && regarray[i].reg != OV5647_REG_END) {
        if (regarray[i].reg != OV5647_REG_DELAY) {
            ret = ov5647_write(handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    return ret;
}

static esp_err_t ov5647_set_reg_bits(esp_sccb_io_handle_t handle, 
                                       uint16_t reg, 
                                       uint8_t offset, 
                                       uint8_t length,
                                       uint8_t value) {
    esp_err_t ret = ESP_OK;
    uint8_t reg_data = 0;

    ret = ov5647_read(handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (reg_data & ~mask) | ((value << offset) & mask);
    ret = ov5647_write(handle, reg, value);
    return ret;
}

// CORRIGÉ : Registres OV5647 corrects pour flip/mirror
static esp_err_t ov5647_set_mirror(esp_cam_sensor_device_t *dev, int enable) {
    // OV5647 : 0x3821 bit 1 pour miroir horizontal
    return ov5647_set_reg_bits(dev->sccb_handle, 0x3821, 1, 1, enable ? 0x01 : 0x00);
}

static esp_err_t ov5647_set_vflip(esp_cam_sensor_device_t *dev, int enable) {
    // OV5647 : 0x3820 bit 1 pour flip vertical
    return ov5647_set_reg_bits(dev->sccb_handle, 0x3820, 1, 1, enable ? 0x01 : 0x00);
}

static esp_err_t ov5647_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id) {
    uint8_t pid_h, pid_l;
    esp_err_t ret = ov5647_read(dev->sccb_handle, OV5647_REG_SENSOR_ID_H, &pid_h);
    if (ret != ESP_OK) return ret;
    
    ret = ov5647_read(dev->sccb_handle, OV5647_REG_SENSOR_ID_L, &pid_l);
    if (ret != ESP_OK) return ret;
    
    id->pid = (pid_h << 8) | pid_l;
    return ESP_OK;
}

// AJOUT : Séquence d'initialisation de base
static esp_err_t ov5647_init_sensor(esp_cam_sensor_device_t *dev) {
    esp_err_t ret;
    
    // Software reset
    ret = ov5647_write(dev->sccb_handle, 0x0103, 0x01);
    if (ret != ESP_OK) return ret;
    delay_ms(10);
    
    // Sortir du mode sleep
    ret = ov5647_write(dev->sccb_handle, 0x0100, 0x00);
    if (ret != ESP_OK) return ret;
    
    // Configuration de base
    ret = ov5647_write(dev->sccb_handle, 0x3035, 0x41);
    if (ret != ESP_OK) return ret;
    
    ret = ov5647_write(dev->sccb_handle, 0x303c, 0x11);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(OV5647_TAG, "✓ Init de base OK");
    return ESP_OK;
}

static esp_err_t ov5647_set_stream(esp_cam_sensor_device_t *dev, int enable) {
    esp_err_t ret = ov5647_write(dev->sccb_handle, OV5647_REG_SLEEP_MODE, enable ? 0x01 : 0x00);
    dev->stream_status = enable;
    ESP_LOGI(OV5647_TAG, "Stream=%d", enable);
    return ret;
}

static esp_err_t ov5647_set_format(esp_cam_sensor_device_t *dev, const void *format) {
    const ov5647_reginfo_t *reg_list = ov5647_input_24M_MIPI_2lane_raw8_800x640_50fps;

    ESP_LOGI(OV5647_TAG, "Configuration : SVGA 800x640@50fps");

    if (reg_list == NULL) {
        ESP_LOGE(OV5647_TAG, "Liste de registres invalide");
        return ESP_FAIL;
    }

    esp_err_t ret = ov5647_write_array(dev->sccb_handle, (ov5647_reginfo_t*)reg_list);
    if (ret != ESP_OK) {
        ESP_LOGE(OV5647_TAG, "Échec config format : %d", ret);
        return ret;
    }

    ESP_LOGI(OV5647_TAG, "✓ Format SVGA configuré");
    return ESP_OK;
}

static esp_err_t ov5647_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg) {
    esp_err_t ret = ESP_OK;
    
    switch (cmd) {
        case 0x04000004: // ESP_CAM_SENSOR_IOC_S_STREAM
            ret = ov5647_set_stream(dev, *(int*)arg);
            break;
            
        case 0x04000010: // ESP_CAM_SENSOR_IOC_S_VFLIP
            ret = ov5647_set_vflip(dev, *(int*)arg);
            ESP_LOGI(OV5647_TAG, "VFlip: %d", *(int*)arg);
            break;
            
        case 0x04000011: // ESP_CAM_SENSOR_IOC_S_HMIRROR
            ret = ov5647_set_mirror(dev, *(int*)arg);
            ESP_LOGI(OV5647_TAG, "HMirror: %d", *(int*)arg);
            break;
            
        default:
            ESP_LOGW(OV5647_TAG, "IOCTL non supporté: 0x%08X", cmd);
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }
    
    return ret;
}

static esp_err_t ov5647_delete(esp_cam_sensor_device_t *dev) {
    if (dev) {
        if (dev->priv) free(dev->priv);
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t ov5647_ops = {
    .set_format = (int (*)(esp_cam_sensor_device_t*, const void*))ov5647_set_format,
    .priv_ioctl = (int (*)(esp_cam_sensor_device_t*, uint32_t, void*))ov5647_priv_ioctl,
    .del = (int (*)(esp_cam_sensor_device_t*))ov5647_delete,
};

esp_cam_sensor_device_t *ov5647_detect(esp_cam_sensor_config_t *config) {
    if (!config) return NULL;
    
    esp_cam_sensor_device_t *dev = (esp_cam_sensor_device_t*)calloc(1, sizeof(esp_cam_sensor_device_t));
    if (!dev) {
        ESP_LOGE(OV5647_TAG, "No memory");
        return NULL;
    }
    
    dev->name = (char*)OV5647_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &ov5647_ops;
    
    if (ov5647_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(OV5647_TAG, "Get sensor ID failed");
        free(dev);
        return NULL;
    }
    
    if (dev->id.pid != OV5647_PID) {
        ESP_LOGE(OV5647_TAG, "Wrong PID: 0x%x", dev->id.pid);
        free(dev);
        return NULL;
    }
    
    ESP_LOGI(OV5647_TAG, "ov5647 detected, PID=0x%x", dev->id.pid);
    
    return dev;
}

} // extern "C"

#endif  // USE_ESP32_VARIANT_ESP32P4

// ============================================================================
// CODE TAB5 CAMERA
// ============================================================================

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGI(TAG, "🎥 Initialisation Tab5 Camera");
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(20);
  }
  
  if (this->pwdn_pin_ != nullptr) {
    this->pwdn_pin_->setup();
    this->pwdn_pin_->digital_write(false);
  }
  
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "❌ Échec init capteur");
    this->mark_failed();
    return;
  }
  
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "❌ Échec init LDO");
    this->mark_failed();
    return;
  }
  
  if (!this->init_csi_()) {
    ESP_LOGE(TAG, "❌ Échec init CSI");
    this->mark_failed();
    return;
  }
  
  if (!this->init_isp_()) {
    ESP_LOGE(TAG, "❌ Échec init ISP");
    this->mark_failed();
    return;
  }
  
  if (!this->allocate_buffer_()) {
    ESP_LOGE(TAG, "❌ Échec allocation buffer");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Caméra prête (50fps optimisé)");
  
#else
  ESP_LOGE(TAG, "❌ ESP32-P4 requis");
  this->mark_failed();
#endif
}

#ifdef USE_ESP32_VARIANT_ESP32P4

bool Tab5Camera::init_sensor_() {
  ESP_LOGI(TAG, "Init capteur ov5647");
  
  sccb_i2c_config_t sccb_config = {};
  sccb_config.device_address = this->sensor_address_;
  sccb_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  sccb_config.scl_speed_hz = 100000;
  sccb_config.addr_bits_width = 16;
  sccb_config.val_bits_width = 8;
  
  esp_sccb_io_handle_t sccb_handle;
  esp_err_t ret = sccb_new_i2c_io_esphome(this, &sccb_config, &sccb_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SCCB init failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ SCCB initialisé");
  
  uint32_t *resolution_ptr = (uint32_t*)malloc(sizeof(uint32_t));
  if (!resolution_ptr) {
    ESP_LOGE(TAG, "Erreur allocation mémoire");
    return false;
  }
  *resolution_ptr = (uint32_t)this->resolution_;
  
  esp_cam_sensor_config_t sensor_config = {};
  sensor_config.sccb_handle = sccb_handle;
  sensor_config.reset_pin = -1;
  sensor_config.pwdn_pin = -1;
  sensor_config.xclk_pin = -1;
  sensor_config.xclk_freq_hz = this->external_clock_frequency_;
  sensor_config.sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
  
  this->sensor_device_ = ov5647_detect(&sensor_config);
  
  if (this->sensor_device_ == nullptr) {
    free(resolution_ptr);
    ESP_LOGE(TAG, "ov5647 detection failed");
    return false;
  }
  
  // AJOUT : Init de base du capteur
  if (ov5647_init_sensor(this->sensor_device_) != ESP_OK) {
    ESP_LOGE(TAG, "Init sensor failed");
    free(resolution_ptr);
    free(this->sensor_device_);
    this->sensor_device_ = nullptr;
    return false;
  }
  
  this->sensor_device_->priv = resolution_ptr;
  
  if (ov5647_set_format(this->sensor_device_, NULL) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set format");
    free(resolution_ptr);
    free(this->sensor_device_);
    this->sensor_device_ = nullptr;
    return false;
  }
  
  // CORRIGÉ : enable = 1 au lieu de 0
  if (this->flip_mirror_) {
    int enable = 1;
    esp_cam_sensor_ioctl(this->sensor_device_, 0x04000010, &enable);
    esp_cam_sensor_ioctl(this->sensor_device_, 0x04000011, &enable);
    ESP_LOGI(TAG, "✓ Flip/Mirror activé");
  }
  
  ESP_LOGI(TAG, "✓ ov5647 détecté (PID: 0x%04X)", this->sensor_device_->id.pid);
  return true;
}

bool Tab5Camera::init_ldo_() {
  ESP_LOGI(TAG, "Init LDO MIPI");
  
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &this->ldo_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LDO failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ LDO OK (2.5V)");
  return true;
}

bool Tab5Camera::init_csi_() {
  ESP_LOGI(TAG, "Init MIPI-CSI (optimisé 50fps)");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_config.h_res = res.width;
  csi_config.v_res = res.height;
  
  // OPTIMISÉ : Lane bit rate augmenté pour 50fps
  csi_config.lane_bit_rate_mbps = 400;  // Au lieu de 250
  
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 2;
  csi_config.byte_swap_en = false;
  
  // OPTIMISÉ : Plus de buffers pour éviter les drops
  csi_config.queue_items = 5;  // Au lieu de 3
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI failed: %d", ret);
    return false;
  }
  
  esp_cam_ctlr_evt_cbs_t callbacks = {
    .on_get_new_trans = Tab5Camera::on_csi_new_frame_,
    .on_trans_finished = Tab5Camera::on_csi_frame_done_,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->csi_handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Callbacks failed: %d", ret);
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Enable CSI failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ CSI OK (%ux%u @ 400Mbps/lane)", res.width, res.height);
  return true;
}

bool Tab5Camera::init_isp_() {
  ESP_LOGI(TAG, "Init ISP (optimisé 50fps)");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  
  // OPTIMISÉ : Clock ISP augmenté pour 50fps
  uint32_t isp_clock_hz = 160000000;  // 160 MHz au lieu de 120 MHz
  
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_src = ISP_CLK_SRC_DEFAULT;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.h_res = res.width;
  isp_config.v_res = res.height;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.clk_hz = isp_clock_hz;
  
  // Essayez 0, puis 1, 2, 3 si couleurs incorrectes
  int bayer_pattern = 1;
  isp_config.bayer_order = (color_raw_element_order_t)bayer_pattern;
  
  const char* bayer_names[] = {"RGGB", "GRBG", "GBRG", "BGGR"};
  ESP_LOGI(TAG, "Pattern Bayer: %s (%d)", bayer_names[bayer_pattern], bayer_pattern);
  
  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP creation failed: 0x%x", ret);
    return false;
  }
  
  ret = esp_isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP enable failed: 0x%x", ret);
    esp_isp_del_processor(this->isp_handle_);
    this->isp_handle_ = nullptr;
    return false;
  }
  
  ESP_LOGI(TAG, "✓ ISP OK (160 MHz, bayer=%s)", bayer_names[bayer_pattern]);
  
  this->configure_isp_color_correction_();
  
  return true;
}

void Tab5Camera::configure_isp_color_correction_() {
  ESP_LOGI(TAG, "Configuration corrections couleur");
  
#ifdef CONFIG_ISP_COLOR_ENABLED
  esp_isp_color_config_t color_config = {};
  color_config.color_contrast = {145, 145, 145};
  color_config.color_saturation = {135, 135, 135};
  color_config.color_hue = 0;
  color_config.color_brightness = 60;
  
  esp_err_t ret = esp_isp_color_configure(this->isp_handle_, &color_config);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✓ Corrections couleur configurées");
  }
#endif

  if (this->sensor_device_) {
    int awb_value = 1;
    esp_err_t ret = esp_cam_sensor_ioctl(this->sensor_device_, 0x03010001, &awb_value);
    
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "✓ AWB activé");
    } else {
      ESP_LOGW(TAG, "AWB non supporté, balance manuelle");
      this->apply_manual_white_balance_();
    }
  }
}

void Tab5Camera::apply_manual_white_balance_() {
  ESP_LOGI(TAG, "Application balance manuelle");
  
#ifdef CONFIG_ISP_COLOR_ENABLED
  esp_isp_color_config_t color_config = {};
  color_config.color_contrast = {145, 145, 145};
  color_config.color_saturation = {135, 135, 135};
  color_config.color_hue = 0;
  color_config.color_brightness = 40;
  
  esp_err_t ret = esp_isp_color_configure(this->isp_handle_, &color_config);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✓ Balance manuelle appliquée");
  }
#endif
}

bool Tab5Camera::allocate_buffer_() {
  CameraResolutionInfo res = this->get_resolution_info_();
  this->frame_buffer_size_ = res.width * res.height * 2;
  
  // Allocation optimisée avec alignment
  this->frame_buffers_[0] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  this->frame_buffers_[1] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, 
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );
  
  if (!this->frame_buffers_[0] || !this->frame_buffers_[1]) {
    ESP_LOGE(TAG, "Buffer alloc failed");
    return false;
  }
  
  this->current_frame_buffer_ = this->frame_buffers_[0];
  
  ESP_LOGI(TAG, "✓ Buffers: 2x%u bytes (aligned 64)", this->frame_buffer_size_);
  return true;
}

bool IRAM_ATTR Tab5Camera::on_csi_new_frame_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  
  trans->buffer = cam->frame_buffers_[cam->buffer_index_];
  trans->buflen = cam->frame_buffer_size_;
  
  return false;
}

bool IRAM_ATTR Tab5Camera::on_csi_frame_done_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  
  if (trans->received_size > 0) {
    cam->frame_ready_ = true;
    cam->buffer_index_ = (cam->buffer_index_ + 1) % 2;
  }
  
  return false;
}

CameraResolutionInfo Tab5Camera::get_resolution_info_() const {
    return {800, 640};
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }
  
  ESP_LOGI(TAG, "Démarrage streaming");
  
  CameraResolutionInfo res = this->get_resolution_info_();
  ESP_LOGI(TAG, "Résolution: %ux%u @ 50fps", res.width, res.height);
  
  // Diagnostic : vérifier état du capteur
  if (this->sensor_device_) {
    uint8_t sleep_mode = 0;
    ov5647_read(this->sensor_device_->sccb_handle, 0x0100, &sleep_mode);
    ESP_LOGI(TAG, "Mode sleep avant: 0x%02X", sleep_mode);
  }
  
  if (this->sensor_device_) {
    int enable = 1;
    esp_err_t ret = esp_cam_sensor_ioctl(
      this->sensor_device_, 
      0x04000004,
      &enable
    );
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start sensor: %d", ret);
      return false;
    }
    
    delay(100);
    
    // Vérifier démarrage
    uint8_t sleep_mode_after = 0;
    ov5647_read(this->sensor_device_->sccb_handle, 0x0100, &sleep_mode_after);
    ESP_LOGI(TAG, "Mode sleep après: 0x%02X", sleep_mode_after);
  }
  
  esp_err_t ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Start CSI failed: %d", ret);
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming actif (50fps optimisé)");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }
  
  esp_cam_ctlr_stop(this->csi_handle_);
  
  if (this->sensor_device_) {
    int enable = 0;
    esp_cam_sensor_ioctl(this->sensor_device_, 0x04000004, &enable);
  }
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "⏹ Streaming arrêté");
  return true;
}

bool Tab5Camera::capture_frame() {
  if (!this->streaming_) {
    ESP_LOGW(TAG, "Pas de streaming actif");
    return false;
  }
  
  bool was_ready = this->frame_ready_;
  
  if (was_ready) {
    this->frame_ready_ = false;
    
    uint8_t last_complete_buffer = (this->buffer_index_ + 1) % 2;
    this->current_frame_buffer_ = this->frame_buffers_[last_complete_buffer];
    
    // Diagnostic : vérifier contenu
    uint32_t sum = 0;
    for(int i = 0; i < 100; i++) {
      sum += this->current_frame_buffer_[i];
    }
    ESP_LOGD(TAG, "✓ Frame capturée, checksum: %u", sum);
  }
  
  return was_ready;
}

uint16_t Tab5Camera::get_image_width() const {
  return this->get_resolution_info_().width;
}

uint16_t Tab5Camera::get_image_height() const {
  return this->get_resolution_info_().height;
}

#endif  // USE_ESP32_VARIANT_ESP32P4

void Tab5Camera::loop() {
  // Géré par callbacks ISR
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Capteur: ov5647 @ 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  Résolution: %ux%u @ 50fps", 
                this->get_image_width(), this->get_image_height());
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  CSI: 400 Mbps/lane");
  ESP_LOGCONFIG(TAG, "  ISP: 160 MHz");
  ESP_LOGCONFIG(TAG, "  Flip/Mirror: %s", this->flip_mirror_ ? "OUI" : "NON");
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "OUI" : "NON");
}

}  // namespace tab5_camera
}  // namespace esphome
