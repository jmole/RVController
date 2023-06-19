
#include <mcp_can.h>
#include <SPI.h>
#include <string>
#include <vector>
#include <functional>
#include "lighting.h"
#include "utils.h"


static const uint8_t SPI_CS_PIN = 9;
static const uint8_t MCP_INTERRUPT_PIN = 11;
static const uint8_t CAN_LED_MSG_PREFIX = 0x20;
static const uint8_t CAN_STATUS_MSG_PREFIX = 0x10;


LEDController controllers[5] = { LEDController(0), LEDController(1), LEDController(2), LEDController(3), LEDController(4) };
int16_t controller_leds[5][8];
CircularIterator<LEDController, 5> c_iter(controllers);
byte send_buffer[8];


MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin

void setup() {
  delay (100);
  Serial.begin(115200);
  delay(100);
  Serial.println("Hello!");

  // set SPI pins
  SPI.setRX(4);
  SPI.setTX(3);
  SPI.setSCK(2);
  Serial.println("SPI pins set.");

  // set interrupt pin
  pinMode(MCP_INTERRUPT_PIN, INPUT);

  while (CAN.begin(CAN_250KBPS) != 0)  // init can bus : baudrate = 500k
  {
    Serial.println("Error starting MCP2515");
    delay(100);
  }
  Serial.println("Started MCP2515");
}


const int lpf_shift = 5;
void loop() {
  {
    // go to next controller
    LEDController& ctrl = c_iter.next();
    int c = ctrl.address;
    // load led data
    for (int i = 0; i < ctrl.num_leds; ++i)
      send_buffer[i] = controller_leds[c][i] >> lpf_shift;
    uint8_t message_id = ctrl.address | CAN_LED_MSG_PREFIX;
    CAN.sendMsgBuf(message_id, 0, ctrl.num_leds, send_buffer);
    uint8_t int_flag_register = CAN.mcp2515_readRegister(MCP_CANINTF);
    while ((int_flag_register & (MCP_TX0IF | MCP_TX1IF | MCP_TX2IF)) == 0) {
      int_flag_register = CAN.mcp2515_readRegister(MCP_CANINTF);
    }
    CAN.mcp2515_setRegister(MCP_CANINTF, (int_flag_register & ~(MCP_TX0IF | MCP_TX1IF | MCP_TX2IF)));
  }
  if (CAN.checkReceive()) {
    long unsigned int rx_message_id;
    uint8_t rx_message_length;
    uint8_t rx_message_buffer[8];
    CAN.readMsgBufID(&rx_message_id, &rx_message_length, rx_message_buffer);
    if ((CAN_STATUS_MSG_PREFIX & rx_message_id) == CAN_STATUS_MSG_PREFIX) {
      uint8_t rx_address = rx_message_id & 0x0F;
      if (rx_address < 5 && rx_message_length == 3) {
        LEDController& lc = controllers[rx_address];
        lc.cs.buttons = rx_message_buffer[0];
        lc.cs.adc_values[0] = rx_message_buffer[1];
        lc.cs.adc_values[1] = rx_message_buffer[2];
        rp2040.fifo.push_nb(rx_address);
      }
    }
  }
}

int k = 0;
int lpf_cabin_lights = 0;

vector<Light*> _lc(int c, int i) {
  return { &(controllers[c].lights[i]) };
}

// turn a Nx2 vector or controller,channel into a vector of pointers to lights
vector<Light*> _l(vector<vector<int>> index) {
  vector<Light*> lights;
  lights.reserve(index.size());
  for (auto l : index) {
    lights.push_back(&(controllers[l[0]].lights[l[1]]));
  }
  return lights;
}

// get Button by index: controller,channel
Button& _b(int c, int i) {
  return controllers[c].buttons[i];
}

vector<vector<int>> cabin_lights = { { 0, 4 }, { 1, 5 }, { 2, 3 } };
vector<vector<int>> overhead_lights = { { 0, 5 }, { 1, 6 }, { 2, 4 } };
auto cabin = _l(cabin_lights);
auto overhead = _l(overhead_lights);

void setup1() {
  for (int c = 0; c < 3; c++) {
    auto& leds = controllers[c].led_pwm;
    for (int i = 0; i < 8; ++i) {
      leds[i] = 0;
    }
  }
  light_button_mapping();
}

void light_button_mapping() {
  auto all = _l({ { 0, 0 }, { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 0 }, { 1, 1 }, { 1, 2 }, { 2, 0 }, { 2, 1 }, { 2, 2 } });
  _b(0, 0).setSelf(_lc(0, 0));
  _b(0, 1).setSelf(_lc(0, 1));
  _b(0, 2).setSelf(_lc(0, 2));
  _b(0, 3).setSelf(_lc(0, 3));
  auto g0 = _l({ { 0, 0 }, { 0, 1 }, { 0, 2 }, { 0, 3 } });
  _b(0, 0).setGroup(g0);
  _b(0, 1).setGroup(g0);
  _b(0, 2).setGroup(g0);
  _b(0, 3).setGroup(g0);
  _b(0, 0).setAll(all);
  _b(0, 1).setAll(all);
  _b(0, 2).setAll(all);
  _b(0, 3).setAll(all);
  // Controller 1
  _b(1, 0).setSelf(_lc(1, 0));
  _b(1, 1).setSelf(_lc(1, 1));
  _b(1, 2).setSelf(_lc(1, 2));
  auto g1 = _l({ { 1, 0 }, { 1, 1 }, { 1, 2 } });
  _b(1, 0).setGroup(g1);
  _b(1, 1).setGroup(g1);
  _b(1, 2).setGroup(g1);
  _b(1, 0).setAll(all);
  _b(1, 1).setAll(all);
  _b(1, 2).setAll(all);
  // Controller 1 bathroom
  _b(1, 3).setSelf(_l({ { 1, 3 }, { 1, 4 } }));
  _b(1, 3).setGroup(_l({ { 1, 3 }, { 1, 4 } }));
  _b(1, 3).setAll(_l({ { 1, 3 }, { 1, 4 } }));
  // Controller 2
  _b(2, 0).setSelf(_lc(2, 0));
  _b(2, 1).setSelf(_lc(2, 1));
  _b(2, 2).setSelf(_lc(2, 2));
  auto g2 = _l({ { 2, 0 }, { 2, 1 }, { 2, 2 } });
  _b(2, 0).setGroup(g2);
  _b(2, 1).setGroup(g2);
  _b(2, 2).setGroup(g2);
  _b(2, 0).setAll(all);
  _b(2, 1).setAll(all);
  _b(2, 2).setAll(all);
  // buttons near touchscreen

  _b(4, 0).setSelf(all);
  _b(4, 0).setGroup(all);
  _b(4, 0).setAll(all);
  _b(4, 0).disable_groups();
  _b(4, 1).setSelf(cabin);
  _b(4, 1).setGroup(cabin);
  _b(4, 1).setAll(cabin);
  _b(4, 1).disable_groups();
  _b(4, 2).setSelf(overhead);
  _b(4, 2).setGroup(overhead);
  _b(4, 2).setAll(overhead);
  _b(4, 2).disable_groups();
}


void loop1() {


  int outputVal = map(controllers[0].cs.adc_values[0], 70, 180, 0, 150);
  //int outputVal = map(controllers[0].cs.adc_values[0], 30, 255, 0, 255);
  // constrain output value to 0-255 range
  outputVal = constrain(outputVal, 0, 255);
  for (auto lp : cabin) {
    lp->set_pwm2(outputVal);
  }
  outputVal = map(outputVal, 0, 255, 0, 120);
  for (auto lp : overhead) {
    lp->set_pwm2(outputVal);
  }

  while (rp2040.fifo.available()) {
    uint32_t addr = rp2040.fifo.pop();
    controllers[addr].update_buttons();
  }

  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 8; ++j) {
      controller_leds[i][j] += controllers[i].led_pwm[j];
      controller_leds[i][j] -= controller_leds[i][j] >> lpf_shift;
    }



  delay(1);
}


// END FILE
