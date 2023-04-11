/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz-aux-ctrl-firmware/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <pico/stdlib.h>
#include <hardware/watchdog.h>

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-Cyphal-Support.h>

#include <107-Arduino-MCP2515.h>
#include <107-Arduino-littlefs.h>
#include <107-Arduino-24LCxx.hpp>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
#define DBG_ENABLE_VERBOSE
#include <107-Arduino-Debug.hpp>

#undef max
#undef min
#include <algorithm>

#include <Adafruit_NeoPixel.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::_register;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

static int const NEOPIXEL_PIN    = 13;
static int const MCP2515_CS_PIN  = 17;
static int const MCP2515_INT_PIN = 20;

static int const NEOPIXEL_NUM_PIXELS = 8; /* Popular NeoPixel ring size */

static CanardNodeID const DEFAULT_AUX_CONTROLLER_NODE_ID = 20;

static SPISettings  const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};

static CanardPortID const ID_LIGHT_MODE = 2010U;

static int8_t const LIGHT_MODE_RED   = 1;
static int8_t const LIGHT_MODE_GREEN = 2;
static int8_t const LIGHT_MODE_BLUE  = 3;
static int8_t const LIGHT_MODE_WHITE = 4;
static int8_t const LIGHT_MODE_AMBER = 5;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame);
void onLightMode_Received(Integer8_1_0 const & msg);
ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

DEBUG_INSTANCE(80, Serial);

ArduinoMCP2515 mcp2515([]()
                       {
                         digitalWrite(MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MCP2515_CS_PIN, HIGH);
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); }, DEFAULT_AUX_CONTROLLER_NODE_ID);

Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);

//static Adafruit_NeoPixel neo_pixel_ctrl(NEOPIXEL_NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB);

Subscription light_mode_subscription =
  node_hdl.create_subscription<Integer8_1_0>(
    ID_LIGHT_MODE,
    [/*&neo_pixel_ctrl*/](Integer8_1_0 const & /*msg*/)
    {
      /*
      if (msg.value == LIGHT_MODE_RED)
        neo_pixel_ctrl.fill(neo_pixel_ctrl.Color(55, 0, 0));
      else if (msg.value == LIGHT_MODE_GREEN)
        neo_pixel_ctrl.fill(neo_pixel_ctrl.Color(0, 55, 0));
      else if (msg.value == LIGHT_MODE_BLUE)
        neo_pixel_ctrl.fill(neo_pixel_ctrl.Color(0, 0, 55));
      else if (msg.value == LIGHT_MODE_WHITE)
        neo_pixel_ctrl.fill(neo_pixel_ctrl.Color(55, 55, 55));
      else if (msg.value == LIGHT_MODE_AMBER)
        neo_pixel_ctrl.fill(neo_pixel_ctrl.Color(55, 40, 0));
      else
        neo_pixel_ctrl.clear();

      neo_pixel_ctrl.show();
       */
    });

ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(
  ExecuteCommand::Request_1_1::_traits_::FixedPortId,
  2*1000*1000UL,
  onExecuteCommand_1_1_Request_Received);

/* LITTLEFS/EEPROM ********************************************************************/

EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                     EEPROM_I2C_DEV_ADDR,
                     [](size_t const dev_addr) { Wire.beginTransmission(dev_addr); },
                     [](uint8_t const data) { Wire.write(data); },
                     []() { return Wire.endTransmission(); },
                     [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                     []() { return Wire.available(); },
                     []() { return Wire.read(); });

lfs_config cfg;
littlefs::Filesystem filesystem(cfg);

#if __GNUC__ >= 11
cyphal::support::platform::storage::KeyValueStorage_littlefs kv_storage(filesystem);
#endif /* __GNUC__ >= 11 */

/* REGISTER ***************************************************************************/

static CanardNodeID node_id = DEFAULT_AUX_CONTROLLER_NODE_ID;

#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_cyphal_node_id          = node_registry->expose("cyphal.node.id", {true}, node_id);
const auto reg_ro_cyphal_node_description = node_registry->route ("cyphal.node.description", {true}, []() { return "L3X-Z AUX_CONTROLLER"; });

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }
  delay(1000);

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();

  if (!eeprom.isConnected()) {
    DBG_ERROR("Connecting to EEPROM failed.");
    return;
  }
  Serial.println(eeprom);

  // block device operations
  cfg.read  = +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) -> int
  {
    eeprom.read_page((block * c->block_size) + off, (uint8_t *)buffer, size);
    return LFS_ERR_OK;
  };
  cfg.prog  = +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) -> int
  {
    eeprom.write_page((block * c->block_size) + off, (uint8_t const *)buffer, size);
    return LFS_ERR_OK;
  };
  cfg.erase = +[](const struct lfs_config *c, lfs_block_t block) -> int
  {
    for(size_t off = 0; off < c->block_size; off += eeprom.page_size())
      eeprom.fill_page((block * c->block_size) + off, 0xFF);
    return LFS_ERR_OK;
  };
  cfg.sync  = +[](const struct lfs_config *c) -> int
  {
    return LFS_ERR_OK;
  };

  // block device configuration
  cfg.read_size      = eeprom.page_size();
  cfg.prog_size      = eeprom.page_size();
  cfg.block_size     = (eeprom.page_size() * 4); /* littlefs demands (erase) block size to exceed read/prog size. */
  cfg.block_count    = eeprom.device_size() / cfg.block_size;
  cfg.block_cycles   = 500;
  cfg.cache_size     = eeprom.page_size();
  cfg.lookahead_size = eeprom.page_size();

  // mount the filesystem
  auto err_mount = filesystem.mount();

  // reformat if we can't mount the filesystem
  // this should only happen on the first boot
  if (err_mount != littlefs::Error::OK) {
    DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount));
    (void)filesystem.format();
  }

  err_mount = filesystem.mount();
  if (err_mount != littlefs::Error::OK) {
    DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(err_mount));
    return;
  }

#if __GNUC__ >= 11
  DBG_INFO("cyphal::support::load ... ");
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
  node_hdl.setNodeId(node_id); /* Update node if a different value has been loaded from the permanent storage. */
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

  /* NODE INFO **************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* cyphal.node.Version.1.0 protocol_version */
    1, 0,
    /* cyphal.node.Version.1.0 hardware_version */
    1, 0,
    /* cyphal.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "107-systems.l3xz-aux-ctrl"
  );

  /* Setup LED pins and initialize */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Initialize NeoPixel control. */
  /*
  neo_pixel_ctrl.begin();
  neo_pixel_ctrl.fill(neo_pixel_ctrl.Color(55, 55, 55));
  neo_pixel_ctrl.show();
   */
}

void loop()
{
  while(digitalRead(MCP2515_INT_PIN) == LOW)
    mcp2515.onExternalEventHandler();

  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_heartbeat = 0;

  unsigned long const now = millis();

  /* Publish the heartbeat once/second */
  if(now - prev_heartbeat > 1000)
  {
    prev_heartbeat = now;

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    DBG_INFO("%d", msg.uptime);
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req)
{
  ExecuteCommand::Response_1_1 rsp;

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    auto const rc_mount = filesystem.mount();
    if (rc_mount != littlefs::Error::OK) {
      DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(rc_mount));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry);
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
     rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else {
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}
