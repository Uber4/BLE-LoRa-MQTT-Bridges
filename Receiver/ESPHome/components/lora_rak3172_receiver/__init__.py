import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, text_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
# Auto-load dependencies for entity publishing.
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]

CONF_FREQUENCY = "frequency"
CONF_SF = "sf"
CONF_BW = "bw"
CONF_CR = "cr"
CONF_PREAMBLE = "preamble"
CONF_TX_POWER = "tx_power"

CONF_OUT_TEMP = "out_temp"
CONF_OUT_HUM = "out_hum"
CONF_OUT_BATT = "out_batt"
CONF_INT_TEMP = "int_temp"
CONF_INT_HUM = "int_hum"
CONF_INT_BATT = "int_batt"
CONF_FAN_STATE = "fan_state"
CONF_RSSI = "rssi"
CONF_SNR = "snr"
CONF_LAST_RAW_PAYLOAD = "last_raw_payload"
CONF_LAST_CRC_STATUS = "last_crc_status"

lora_ns = cg.esphome_ns.namespace("lora_rak3172_receiver")
LoRaRAK3172Receiver = lora_ns.class_(
    "LoRaRAK3172Receiver", cg.Component, uart.UARTDevice
)

# LoRa P2P parameter schema (validated by ESPHome).
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LoRaRAK3172Receiver),
        cv.Optional(CONF_FREQUENCY, default=868000000): cv.int_,
        cv.Optional(CONF_SF, default=10): cv.int_,
        cv.Optional(CONF_BW, default=1): cv.int_,
        cv.Optional(CONF_CR, default=1): cv.int_,
        cv.Optional(CONF_PREAMBLE, default=8): cv.int_,
        cv.Optional(CONF_TX_POWER, default=22): cv.int_,
        cv.Optional(CONF_OUT_TEMP): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUT_HUM): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_OUT_BATT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_INT_TEMP): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_INT_HUM): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_INT_BATT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_FAN_STATE): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_RSSI): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_SNR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_LAST_RAW_PAYLOAD): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_LAST_CRC_STATUS): cv.use_id(text_sensor.TextSensor),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Register component and UART transport.
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Apply radio parameters.
    cg.add(var.set_lora_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_lora_sf(config[CONF_SF]))
    cg.add(var.set_lora_bw(config[CONF_BW]))
    cg.add(var.set_lora_cr(config[CONF_CR]))
    cg.add(var.set_lora_preamble(config[CONF_PREAMBLE]))
    cg.add(var.set_lora_tx_power(config[CONF_TX_POWER]))

    if CONF_OUT_TEMP in config:
        sens = await cg.get_variable(config[CONF_OUT_TEMP])
        cg.add(var.set_out_temp(sens))
    if CONF_OUT_HUM in config:
        sens = await cg.get_variable(config[CONF_OUT_HUM])
        cg.add(var.set_out_hum(sens))
    if CONF_OUT_BATT in config:
        sens = await cg.get_variable(config[CONF_OUT_BATT])
        cg.add(var.set_out_batt(sens))

    if CONF_INT_TEMP in config:
        sens = await cg.get_variable(config[CONF_INT_TEMP])
        cg.add(var.set_int_temp(sens))
    if CONF_INT_HUM in config:
        sens = await cg.get_variable(config[CONF_INT_HUM])
        cg.add(var.set_int_hum(sens))
    if CONF_INT_BATT in config:
        sens = await cg.get_variable(config[CONF_INT_BATT])
        cg.add(var.set_int_batt(sens))

    if CONF_FAN_STATE in config:
        sens = await cg.get_variable(config[CONF_FAN_STATE])
        cg.add(var.set_fan_state(sens))

    if CONF_RSSI in config:
        sens = await cg.get_variable(config[CONF_RSSI])
        cg.add(var.set_rssi(sens))
    if CONF_SNR in config:
        sens = await cg.get_variable(config[CONF_SNR])
        cg.add(var.set_snr(sens))

    if CONF_LAST_RAW_PAYLOAD in config:
        sens = await cg.get_variable(config[CONF_LAST_RAW_PAYLOAD])
        cg.add(var.set_last_raw_payload(sens))
    if CONF_LAST_CRC_STATUS in config:
        sens = await cg.get_variable(config[CONF_LAST_CRC_STATUS])
        cg.add(var.set_last_crc_status(sens))
