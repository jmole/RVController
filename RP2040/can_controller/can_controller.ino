
#include <mcp_can.h>
#include <SPI.h>
#include <string>
#include <vector>

enum class ButtonState {
  Idle,
  Down,
  Dimming,
  DownUp,
  Switching,
  DownUpDown,
  GroupDimming,
  DownUpDownUp,
  GroupSwitching,
  DownUpDownUpDown,
  AllDimming,
  DownUpDownUpDownUp,
  AllSwitching
};

enum class DimmingDirection {
  Up,
  Down
};

class BSMachine {
private:
    const int downtime = 500;
    const int uptime = 175;
    ButtonState bs;
    bool previously_pressed;
    unsigned long last_time;
public:
    BSMachine() : bs(ButtonState::Idle) {}
    ButtonState state() {
      return bs;
    }
    bool update(bool pressed) {
      ButtonState old_state = bs;
      int now = millis();
      int timer = now-last_time;
      switch(bs) {

        case ButtonState::Idle:
          if (pressed) {
            bs = ButtonState::Down;
          }
        break;

        case ButtonState::Down:
          if (!pressed) {
            if (timer < downtime)
              bs = ButtonState::DownUp;
          } else {  // if still pressed
            if (timer > downtime) {
              bs = ButtonState::Dimming;
            }
          }
        break;

        case ButtonState::Dimming: 
          if (!pressed) {
            bs = ButtonState::Idle;
          }
        break;

        case ButtonState::DownUp:
          if (!pressed) {
            if (timer > uptime) {
              bs = ButtonState::Switching;
            }
          } else { //if pressed 
            if (timer < uptime) {
              bs = ButtonState::DownUpDown;
            }
          }
        break;

        case ButtonState::Switching:
            bs = ButtonState::Idle;
        break;

        case ButtonState::DownUpDown:
          if (!pressed) {
            if (timer < downtime) {
              bs = ButtonState::DownUpDownUp;
            }
          } else {  // if still pressed
            if (timer > downtime) {
              bs = ButtonState::GroupDimming;
            }
          }
        break;

        case ButtonState::GroupDimming:
          if (!pressed) {
            bs = ButtonState::Idle;
          }
        break;

        case ButtonState::DownUpDownUp:
          if (!pressed) {
            if (timer > uptime) {
              bs = ButtonState::GroupSwitching;
            }
          } else { //if pressed
            if (timer < uptime) {
              bs = ButtonState::DownUpDownUpDown;
            }
          }
        break;

        case ButtonState::GroupSwitching:
            bs = ButtonState::Idle;
        break;

        case ButtonState::DownUpDownUpDown:
          if (!pressed) {
            if (timer < downtime) {
              bs = ButtonState::DownUpDownUpDownUp;
            }
          } else {  // if still pressed
            if (timer > downtime) {
              bs = ButtonState::AllDimming;
            }
          }
        break;
        
        case ButtonState::AllDimming:
          if (!pressed) {
            bs = ButtonState::Idle;
          }
        break;

        case ButtonState::DownUpDownUpDownUp:
          bs = ButtonState::AllSwitching;
        break;

        case ButtonState::AllSwitching:
          bs = ButtonState::Idle;
        break;

        default:
        bs = ButtonState::Idle;
        break;
      }
      if (previously_pressed != pressed) {
        previously_pressed = pressed;
        last_time = now;
      }
      return old_state != bs;
    }

    std::string stateString() {
      switch(bs) {
        case ButtonState::Idle:
          return "Idle";
        case ButtonState::Down:
          return "Down";
        case ButtonState::Dimming:
          return "Dimming";
        case ButtonState::DownUp:
          return "DownUp";
        case ButtonState::Switching:
          return "Switching";
        case ButtonState::DownUpDown:
          return "DownUpDown";
        case ButtonState::GroupDimming:
          return "GroupDimming";
        case ButtonState::DownUpDownUp:
          return "DownUpDownUp";
        case ButtonState::GroupSwitching:
          return "GroupSwitching";
        case ButtonState::DownUpDownUpDown:
          return "DownUpDownUpDown";
        case ButtonState::AllDimming:
          return "AllDimming";
        case ButtonState::DownUpDownUpDownUp:
          return "DownUpDownUpDownUp";
        case ButtonState::AllSwitching:
          return "AllSwitching";
        default:
          return "Unknown";
      }
    }
};

class Light { 
  
}


static const uint8_t SPI_CS_PIN = 9;
static const uint8_t MCP_INTERRUPT_PIN = 11;
static const uint8_t CAN_LED_MSG_PREFIX = 0x20;
static const uint8_t CAN_STATUS_MSG_PREFIX = 0x10;

struct ControllerStatus {
  ControllerStatus() : buttons(0) , adc_values{0,0} {}
  uint8_t buttons;
  uint8_t adc_values[2];
};

struct LEDController {
  LEDController(uint8_t address) : address(address), led_pwm{0,0,0,0,0,0,0,0} {}
  uint8_t address; 
  std::vector<uint8_t> led_pwm;
  ControllerStatus cs;
};


std::vector<LEDController> controllers {
  LEDController(0),
  LEDController(1),
  LEDController(2),
  LEDController(3)
};

template <typename T>
class CircularIterator {
public:
    CircularIterator(const std::vector<T>& vec) : vec_(vec), index_(0) {}
    T Next() {
        T result = vec_[index_];
        index_ = (index_ + 1) % vec_.size();
        return result;
    }
private:
    const std::vector<T>& vec_;
    size_t index_;
};




MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin
                                  
void setup()
{
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Hello!");
    
    // set SPI pins
    SPI.setRX(4);
    SPI.setTX(3);
    SPI.setSCK(2);
    Serial.println("SPI pins set.");

    // set interrupt pin
    pinMode(MCP_INTERRUPT_PIN, INPUT);
      
    while (CAN.begin(CAN_250KBPS) != 0)    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }
    Serial.println("CAN BUS OK!");
}

CircularIterator c_iter(controllers);
byte send_buffer[8];
void loop()
{
  {
    const LEDController& ctrl = c_iter.Next();
    auto & leds = ctrl.led_pwm;
    for (int i=0; i<leds.size(); ++i)
      send_buffer[i] = leds[i]; 
    uint8_t message_id = ctrl.address | CAN_LED_MSG_PREFIX;
    CAN.sendMsgBuf(message_id, 0, leds.size(), send_buffer);
    uint8_t int_flag_register = CAN.mcp2515_readRegister(MCP_CANINTF);
    while ((int_flag_register & (MCP_TX0IF|MCP_TX1IF|MCP_TX2IF)) == 0) {
      int_flag_register = CAN.mcp2515_readRegister(MCP_CANINTF); 
    }
    CAN.mcp2515_setRegister(MCP_CANINTF, (int_flag_register & ~(MCP_TX0IF|MCP_TX1IF|MCP_TX2IF)));
  }
  if (CAN.checkReceive()) {
    long unsigned int rx_message_id;
    uint8_t rx_message_length;
    uint8_t rx_message_buffer[8];
    CAN.readMsgBufID(&rx_message_id, &rx_message_length, rx_message_buffer);
    if ((CAN_STATUS_MSG_PREFIX & rx_message_id) == CAN_STATUS_MSG_PREFIX) {
      uint8_t rx_address = rx_message_id & 0x0F;
      if (rx_address < controllers.size() && rx_message_length == 3) {
        controllers[rx_address].cs.buttons = rx_message_buffer[0];
        controllers[rx_address].cs.adc_values[0] = rx_message_buffer[1];
        controllers[rx_address].cs.adc_values[1] = rx_message_buffer[2];
        rp2040.fifo.push_nb(rx_address);
        
      }
    }
  }
}

int k=0;
int lpf_cabin_lights = 0;

std::vector<std::vector<uint8_t>> cabin_lights = {{0,4}, {1,5}, {2,3}};

void setup1() {
    for (int c = 0; c<3; c++) {
      auto & leds = controllers[c].led_pwm;
      for (int i=0; i<8; ++i) {
        leds[i] = 0;
      }
    }
}
int v = 0;

BSMachine bsm;

void loop1()
{
  int outputVal = map(controllers[0].cs.adc_values[0], 70, 195, 0, 255);
  // constrain output value to 0-255 range
  outputVal = constrain(outputVal, 0, 255);
  lpf_cabin_lights += outputVal;
  lpf_cabin_lights -= lpf_cabin_lights >> 4;
  outputVal = lpf_cabin_lights >> 4;
  constrain(outputVal, 0, 255);
  for (auto i : cabin_lights) {
    controllers[i[0]].led_pwm[i[1]] = outputVal;
  }
  
  delay(1);                       // send data per 100ms
  while (rp2040.fifo.available()) {
    uint32_t addr = rp2040.fifo.pop();
    bool button_status = (controllers[addr].cs.buttons & 1) == 1;
    bool state_changed = bsm.update(button_status);
    if (state_changed)
      Serial.println(bsm.stateString().c_str());
    switch (bsm.state()) {
      case ButtonState::Switching:
        if (controllers[addr].led_pwm[0] == 0)
          controllers[addr].led_pwm[0] = 255;
        else controllers[addr].led_pwm[0] = 0;
        break;
      case ButtonState::Dimming:
        controllers[addr].led_pwm[0] = (controllers[addr].led_pwm[0]+1)%255;
        break;
      case ButtonState::GroupDimming:
        if (controllers[addr].led_pwm[0] != 0)
          controllers[addr].led_pwm[0]--;
        break;
      
    }
  /*
    Serial.println("\ndata received");
    Serial.print(addr);
    Serial.print(": b(");
    Serial.print(controllers[addr].cs.buttons);
    Serial.print(") adc1(");
    Serial.print(controllers[addr].cs.adc_values[0]);
    Serial.print(") adc2(");
    Serial.print(controllers[addr].cs.adc_values[1]);
    Serial.println(")");
    */
  }
}

void not_called_1()
{
  v = (v+1)%510;
  int k = abs(255-v);
  k = abs(255-k);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<8; ++j) {
      controllers[i].led_pwm[j] = k;
    }
    rp2040.fifo.push_nb(i);
  }
  wait_and_check_status(3);
}

void wait_and_check_status(int delaytime) {
    unsigned long earlier = millis(); // Get the current time

  // Check if the interval has passed
  while (millis() - earlier <= delaytime) {
    if (rp2040.fifo.available()) {
      uint32_t addr = rp2040.fifo.pop();
    /*  Serial.println("\ndata received");
      Serial.print(addr);
      Serial.print(": b(");
      Serial.print(controllers[addr].cs.buttons);
      Serial.print(") adc1(");
      Serial.print(controllers[addr].cs.adc_values[0]);
      Serial.print(") adc2(");
      Serial.print(controllers[addr].cs.adc_values[1]);
      Serial.println(")");
      */
    }
  }
}

void not_called() {
  int outputVal = map(controllers[0].cs.adc_values[0], 70, 195, 0, 255);
  // constrain output value to 0-255 range
  outputVal = constrain(outputVal, 0, 255);
  lpf_cabin_lights += outputVal;
  lpf_cabin_lights -= lpf_cabin_lights >> 4;
  outputVal = lpf_cabin_lights >> 4;
  constrain(outputVal, 0, 255);
  for (auto i : cabin_lights) {
    controllers[i[0]].led_pwm[i[1]] = outputVal;
  }
  delay(1);                       // send data per 100ms
  while (rp2040.fifo.available()) {
    uint32_t addr = rp2040.fifo.pop();
    Serial.println("\ndata received");
    Serial.print(addr);
    Serial.print(": b(");
    Serial.print(controllers[addr].cs.buttons);
    Serial.print(") adc1(");
    Serial.print(controllers[addr].cs.adc_values[0]);
    Serial.print(") adc2(");
    Serial.print(controllers[addr].cs.adc_values[1]);
    Serial.println(")");
  }

}


enum states {
  start,
  down1,
  up1,
  down2,
  up2,
  down3,
  up3,
  

};

void state_machine() {

}


// END FILE
