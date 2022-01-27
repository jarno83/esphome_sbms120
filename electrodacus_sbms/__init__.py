import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["jarno83"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

CONF_SBMS_ELECTRODACUS_ID = "sbms_electrodacus_id"

electrodacus_sbms = cg.esphome_ns.namespace("electrodacus_sbms")
ElectrodacusSbmsComponent = electrodacus_sbms.class_(
    "ElectrodacusSbmsComponent", cg.PollingComponent, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(ElectrodacusSbmsComponent)})
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("30s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
