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

#include <I2C_eeprom.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-UniqueId.h>
#include <107-Arduino-CriticalSection.h>

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

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]()
                       {
                         noInterrupts();
                         SPI.beginTransaction(MCP2515x_SPI_SETTING);
                         digitalWrite(MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MCP2515_CS_PIN, HIGH);
                         SPI.endTransaction();
                         interrupts();
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); }, DEFAULT_AUX_CONTROLLER_NODE_ID);

Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
  (Heartbeat_1_0::_traits_::FixedPortId, 1*1000*1000UL /* = 1 sec in usecs. */);

static Adafruit_NeoPixel neo_pixel_ctrl(NEOPIXEL_NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB);

Subscription light_mode_subscription =
  node_hdl.create_subscription<Integer8_1_0>(
    ID_LIGHT_MODE,
    CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
    [&neo_pixel_ctrl](Integer8_1_0 const & msg)
    {
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
    });

/* REGISTER ***************************************************************************/

static CanardNodeID node_id = DEFAULT_AUX_CONTROLLER_NODE_ID;

#if __GNUC__ >= 11

Registry reg(node_hdl, micros);

const auto reg_rw_cyphal_node_id = reg.expose("cyphal.node.id", node_id);
const auto reg_ro_cyphal_node_description = reg.route("cyphal.node.description", {true}, []() { return "L3X-Z AUX_CONTROLLER" });

#endif /* __GNUC__ >= 11 */

/* NODE INFO **************************************************************************/

static NodeInfo node_info
(
  node_hdl,
  /* cyphal.node.Version.1.0 protocol_version */
  1, 0,
  /* cyphal.node.Version.1.0 hardware_version */
  1, 0,
  /* cyphal.node.Version.1.0 software_version */
  0, 1,
  /* saturated uint64 software_vcs_revision_id */
  0,
  /* saturated uint8[16] unique_id */
  OpenCyphalUniqueId(),
  /* saturated uint8[<=50] name */
  "107-systems.l3xz-aux-ctrl"
);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  //while (!Serial) { }

  /* Setup LED pins and initialize */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /* Setup SPI access */
  SPI.begin();
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Initialize NeoPixel control. */
  neo_pixel_ctrl.begin();
  neo_pixel_ctrl.fill(neo_pixel_ctrl.Color(55, 55, 55));
  neo_pixel_ctrl.show();
}

void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

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
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}
