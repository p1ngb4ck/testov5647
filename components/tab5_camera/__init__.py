import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_FREQUENCY,
)
from esphome import pins

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c", "esp32"]

MULTI_CONF = True

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Configuration pour OV5647
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_FRAMERATE = "framerate"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"
CONF_ADDRESS_SENSOR_OV5647 = "address_sensor_ov5647"
CONF_FLIP_MIRROR = "flip_mirror"

# Déclarer les enums C++
CameraResolution = tab5_camera_ns.enum("CameraResolution")
RESOLUTION_SVGA = CameraResolution.RESOLUTION_SVGA
#RESOLUTION_720P = CameraResolution.RESOLUTION_720P
#RESOLUTION_1080P = CameraResolution.RESOLUTION_1080P

PixelFormat = tab5_camera_ns.enum("PixelFormat")
PIXEL_FORMAT_RGB565 = PixelFormat.PIXEL_FORMAT_RGB565
PIXEL_FORMAT_YUV422 = PixelFormat.PIXEL_FORMAT_YUV422
PIXEL_FORMAT_RAW8 = PixelFormat.PIXEL_FORMAT_RAW8
PIXEL_FORMAT_JPEG = PixelFormat.PIXEL_FORMAT_JPEG

# Mapping pour validation YAML
CAMERA_RESOLUTIONS = {
    "SVGA": RESOLUTION_SVGA,
    #"720P": RESOLUTION_720P,
    #"1080P": RESOLUTION_1080P,
}

PIXEL_FORMATS = {
    "RGB565": PIXEL_FORMAT_RGB565,
    "YUV422": PIXEL_FORMAT_YUV422,
    "RAW8": PIXEL_FORMAT_RAW8,
    "JPEG": PIXEL_FORMAT_JPEG,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Camera),
            cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.Any(
                cv.int_range(min=0, max=50),
                pins.internal_gpio_output_pin_schema
            ),
            cv.Optional(CONF_FREQUENCY, default=24000000): cv.int_range(min=6000000, max=40000000),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS_SENSOR_OV5647, default=0x36): cv.i2c_address,
            cv.Optional(CONF_RESOLUTION, default="SVGA"): cv.enum(CAMERA_RESOLUTIONS, upper=True),
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.enum(PIXEL_FORMATS, upper=True),
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=30): cv.int_range(min=1, max=60),
            cv.Optional(CONF_FLIP_MIRROR, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    cg.add(var.set_name(config[CONF_NAME]))
    
    # Gérer le pin externe de l'horloge
    ext_clock_pin_config = config[CONF_EXTERNAL_CLOCK_PIN]
    if isinstance(ext_clock_pin_config, int):
        cg.add(var.set_external_clock_pin(ext_clock_pin_config))
    else:
        pin_num = ext_clock_pin_config[pins.CONF_NUMBER]
        cg.add(var.set_external_clock_pin(pin_num))
    
    cg.add(var.set_external_clock_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_sensor_address(config[CONF_ADDRESS_SENSOR_OV5647]))
    
    # Configuration résolution et format
    cg.add(var.set_resolution(config[CONF_RESOLUTION]))
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Flip/Mirror
    cg.add(var.set_flip_mirror(config[CONF_FLIP_MIRROR]))
    
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    
    # Configuration pour ESP32-P4 avec ESP-IDF 5.x
    cg.add_build_flag("-DBOARD_HAS_PSRAM")
    cg.add_build_flag("-DCONFIG_CAMERA_CORE0=1")
    cg.add_build_flag("-DCONFIG_CAMERA_OV5647=1")
    cg.add_build_flag("-DCONFIG_CAMERA_OV5647_ANA_GAIN_PRIORITY=1")
    cg.add_build_flag("-DCONFIG_CAMERA_OV5647_ABSOLUTE_GAIN_LIMIT=63008")
    cg.add_build_flag("-DCONFIG_CAMERA_OV5647_MAX_SUPPORT=1")
    cg.add_build_flag("-DCONFIG_CAMERA_OV5647_MIPI_IF_FORMAT_INDEX_DAFAULT=0")
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")



