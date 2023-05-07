#pragma once

// classes for lighting control

using std::vector;
using std::reference_wrapper;

// forward declarations
class LEDController;
class Light;
class Button;
class Dimmer;
class BSMachine;


// States for Button state machine
//
// single press = switch
// single press-and-hold = dim
//
// double press = group switch
// double press-and-hold = group dim
//
// triple press = all switch
// triple press-and-hold = all dim

enum class ButtonState {
  Idle,
  Down,
  DimmingStart,
  Dimming,
  DimmingEnd,
  DownUp,
  Switching,
  DownUpDown,
  GroupDimmingStart,
  GroupDimming,
  GroupDimmingEnd,
  DownUpDownUp,
  GroupSwitching,
  DownUpDownUpDown,
  AllDimmingStart,
  AllDimming,
  AllDimmingEnd,
  DownUpDownUpDownUp,
  AllSwitching
};

enum class DimmingDirection {
  Up,
  Down
};

// The Button State Machine
// takes a boolean input for button state update(bool) and determines the next state
class BSMachine {
private:
  // how long to hold for dimming
  static const int downtime = 500;
  // how long to release before we call it a press 
  // (button down, button up, wait for uptime milliseconds)
  static const int uptime = 250;
  ButtonState bs = ButtonState::Idle;
  bool previously_pressed;
  unsigned long last_time;
public:
  BSMachine()
    : bs(ButtonState::Idle) {}
  BSMachine(const BSMachine& bsm)
    : bs(bsm.bs) {}
  BSMachine& operator=(const BSMachine& bsm) {
    bs = bsm.bs;
    return *this;
  }
  ButtonState state() {
    return bs;
  }

  // returns true if state changes
  bool update(bool pressed) {
    ButtonState old_state = bs;
    int now = millis();
    int timer = now - last_time;
    switch (bs) {

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
            bs = ButtonState::DimmingStart;
          }
        }
        break;
      
      case ButtonState::DimmingStart:
        bs = ButtonState::Dimming;
        break;

      case ButtonState::Dimming:
        if (!pressed) {
          bs = ButtonState::DimmingEnd;
        }
        break;
      
      case ButtonState::DimmingEnd:
        bs = ButtonState::Idle;
        break;

      case ButtonState::DownUp:
        if (!pressed) {
          if (timer > uptime) {
            bs = ButtonState::Switching;
          }
        } else {  //if pressed
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
            bs = ButtonState::GroupDimmingStart;
          }
        }
        break;

      case ButtonState::GroupDimmingStart:
        bs = ButtonState::GroupDimming;
        break;

      case ButtonState::GroupDimming:
        if (!pressed) {
          bs = ButtonState::GroupDimmingEnd;
        }
        break;
      
      case ButtonState::GroupDimmingEnd:
        bs = ButtonState::Idle;
        break;

      case ButtonState::DownUpDownUp:
        if (!pressed) {
          if (timer > uptime) {
            bs = ButtonState::GroupSwitching;
          }
        } else {  //if pressed
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
            bs = ButtonState::AllDimmingStart;
          }
        }
        break;

      case ButtonState::AllDimmingStart:
        bs = ButtonState::AllDimming;
        break;

      case ButtonState::AllDimming:
        if (!pressed) {
          bs = ButtonState::AllDimmingEnd;
        }
        break;
      
      case ButtonState::AllDimmingEnd:
        bs = ButtonState::Idle;
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
    switch (bs) {
      case ButtonState::Idle:
        return "Idle";
      case ButtonState::Down:
        return "Down";
      case ButtonState::DimmingStart:
        return "DimmingStart";
      case ButtonState::Dimming:
        return "Dimming";
      case ButtonState::DimmingEnd:
        return "DimmingEnd";
      case ButtonState::DownUp:
        return "DownUp";
      case ButtonState::Switching:
        return "Switching";
      case ButtonState::DownUpDown:
        return "DownUpDown";
      case ButtonState::GroupDimmingStart:
        return "GroupDimmingStart";
      case ButtonState::GroupDimming:
        return "GroupDimming";
      case ButtonState::GroupDimmingEnd:
        return "GroupDimmingEnd";
      case ButtonState::DownUpDownUp:
        return "DownUpDownUp";
      case ButtonState::GroupSwitching:
        return "GroupSwitching";
      case ButtonState::DownUpDownUpDown:
        return "DownUpDownUpDown";
      case ButtonState::AllDimmingStart:
        return "AllDimmingStart";
      case ButtonState::AllDimming:
        return "AllDimming";
      case ButtonState::AllDimmingEnd:
        return "AllDimmingEnd";
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
  uint8_t channel;
  LEDController& controller;
public:
  Light(LEDController& controller, uint8_t channel);
  virtual void set_pwm(uint8_t value);
  virtual uint8_t get_pwm();
};


// object that takes button state and does something to a light (switch, dim, etc)
// can also control groups of lights
class Dimmer {
  vector<Light*> lights;
  DimmingDirection direction;
  int pwm_value;
  Button* current_dimmer = nullptr;
public:
  Dimmer(vector<Light*> lights) : 
  lights(lights),
  direction(DimmingDirection::Down),
  pwm_value(0),
  current_dimmer(nullptr)
   {}

  virtual bool switch_pressed() {
    int lights_on = 0; 
    for (auto light : lights) {
      if(light->get_pwm() > 0)
        lights_on++;
    }
    int new_light_value = 255;
    direction = DimmingDirection::Down;
    if (lights_on > lights.size()>>1) {
      new_light_value = 0;
      direction = DimmingDirection::Up;
    }
    for (auto light : lights) {
      light->set_pwm(new_light_value);
    }
    return new_light_value != 0;
  }

  virtual bool dim_start(Button* button) {
    if (current_dimmer)
      return false;
    pwm_value = 0;
    for (auto light : lights) {
      pwm_value += light->get_pwm();
    }
    pwm_value /= lights.size();
    if (pwm_value < 32)
      direction = DimmingDirection::Up;
    else if (pwm_value > 220)
      direction = DimmingDirection::Down;
    current_dimmer = button;
    return true;
  }

  virtual bool dim(Button* button) {
    if (current_dimmer != button)
      return false;
    if (direction == DimmingDirection::Up) {
      pwm_value++;
      if (pwm_value > 255) {
        pwm_value = 255;
      }
    } else {
      pwm_value--;
      if (pwm_value < 0) {
        pwm_value = 0;
      }
    }
    for (auto light : lights) {
      light->set_pwm(pwm_value);
    }
    return true;
  }

  virtual bool dim_end(Button* button) {
    if (current_dimmer != button)
      return false;
    if (direction == DimmingDirection::Down)
      direction = DimmingDirection::Up;
    else 
      direction = DimmingDirection::Down;
    current_dimmer = nullptr;
    return true;
  }
};

class NoDimmer : public Dimmer {
public:
  NoDimmer() : Dimmer({}) {}
  bool switch_pressed() override {
    return false;
  }
  bool dim_start(Button* button) override  {
    return false;
  }
  bool dim(Button* button) override  {
    return false;
  }
  bool dim_end(Button* button) override  {
    return false;
  }
};

class Button {
  NoDimmer no_dimmer;
  uint8_t channel;
  LEDController* controller;
  BSMachine bsm;
  Dimmer* self;
  Dimmer* group;
  Dimmer* all; 
public:
  Button(LEDController *controller, uint8_t channel, Dimmer* self = nullptr, Dimmer* group = nullptr, Dimmer* all = nullptr) : 
      controller(controller), 
      channel(channel),
      self(self? self : &no_dimmer),
      group(group? group :&no_dimmer),
      all(all? all : &no_dimmer) {}
  
  void setController(LEDController *controller) {
    this->controller = controller;
  }

  void setSelf(vector<Light*> lights) {
    Serial.println("setting self");
    self = new Dimmer(lights);
  }

  void setGroup(vector<Light*> lights) {
    group = new Dimmer(lights);
  }

  void setAll(vector<Light*> lights) {
    all = new Dimmer(lights);
  }

  std::string stateString() {
    return bsm.stateString();
  }

  bool update(bool pressed) {
    bool state_changed = bsm.update(pressed);
    if (state_changed) {
      Serial.print("Button ");
      Serial.print(channel);
      Serial.print(" state changed to ");
      Serial.println(bsm.stateString().c_str());
    }
    switch (bsm.state()) {
      case ButtonState::Switching:
        self->switch_pressed();
        break;
      case ButtonState::DimmingStart:
        self->dim_start(this);
        break;
      case ButtonState::Dimming:
        self->dim(this);
        break;
      case ButtonState::DimmingEnd:
        self->dim_end(this);
        break;
      case ButtonState::GroupSwitching:
        group->switch_pressed();
        break;  
      case ButtonState::GroupDimmingStart:
        group->dim_start(this);
        break;
      case ButtonState::GroupDimming:
        group->dim(this);
        break;
      case ButtonState::GroupDimmingEnd:
        group->dim_end(this);
        break;
      case ButtonState::AllSwitching:
        all->switch_pressed();
        break;  
      case ButtonState::AllDimmingStart:
        all->dim_start(this);
        break;
      case ButtonState::AllDimming:
        all->dim(this);
        break;
      case ButtonState::AllDimmingEnd:
        all->dim_end(this);
        break;
      default:
        break;
    }
    return state_changed;
  }
};

struct ControllerStatus {
  ControllerStatus()
    : buttons(0), adc_values{ 0, 0 } {}
  uint8_t buttons;
  uint8_t adc_values[2];
};

struct LEDController {
  static const uint8_t num_leds = 8;
  static const uint8_t num_buttons = 6;
  
  uint8_t address;
  uint8_t led_pwm[num_leds] = {0,0,0,0,0,0,0,0};
  ControllerStatus cs;
  vector<Light> lights;
  vector<Button> buttons;
  LEDController(uint8_t address)
    : address(address) {
      init_lights_buttons();
    }
  
  // delete copy constructors, we want these to be uncopyable
  LEDController(const LEDController& other) {
    this->address = other.address;
    this->cs = other.cs;
    std::copy(std::begin(other.led_pwm), std::end(other.led_pwm), std::begin(led_pwm));
    for (int i=0; i<num_leds; ++i)
      lights.push_back(Light(*this, i));
    std::copy(std::begin(other.buttons), std::end(other.buttons), std::begin(buttons));
    for (int i=0; i<num_buttons; ++i)
      buttons[i].setController(this);
  }
  LEDController& operator=(const LEDController& other) {
    this->address = other.address;
    this->cs = other.cs;
    std::copy(std::begin(other.led_pwm), std::end(other.led_pwm), std::begin(led_pwm));
    for (int i=0; i<num_leds; ++i)
      lights.push_back(Light(*this, i));
    std::copy(std::begin(other.buttons), std::end(other.buttons), std::begin(buttons));
    for (int i=0; i<num_buttons; ++i)
      buttons[i].setController(this);
    return *this;
  };

  // add move constructor
  LEDController& operator=(LEDController&& other) noexcept {
    this->address = other.address;
    this->cs = other.cs;
    std::copy(std::begin(other.led_pwm), std::end(other.led_pwm), std::begin(led_pwm));
    for (int i=0; i<num_leds; ++i)
      lights.push_back(Light(*this, i));
    std::copy(std::begin(other.buttons), std::end(other.buttons), std::begin(buttons));
    for (int i=0; i<num_buttons; ++i)
      buttons[i].setController(this);
    return *this;
  };

  LEDController(LEDController&& other) noexcept {
    this->address = other.address;
    this->cs = other.cs;
    std::copy(std::begin(other.led_pwm), std::end(other.led_pwm), std::begin(led_pwm));
    for (int i=0; i<num_leds; ++i)
      lights.push_back(Light(*this, i));
    std::copy(std::begin(other.buttons), std::end(other.buttons), std::begin(buttons));
    for (int i=0; i<num_buttons; ++i)
      buttons[i].setController(this);
  }
  
  void init_lights_buttons() {
    lights.clear();
    buttons.clear();
    for (int i=0; i<num_leds; ++i)
      lights.push_back(Light(*this, i));
    for (int i=0; i<num_buttons; ++i)
      buttons.push_back(Button(this, i));
  }

  // updates all button states for this controller.
  // only call when new data is available
  void update_buttons() {
    for (int i=0; i<6; ++i) {
      bool button_status = (cs.buttons & (1<<i)) == (1<<i);
      bool state_changed = buttons[i].update(button_status);
      if (state_changed){
        //Serial.println(buttons[i].stateString().c_str());
        //printStatus();
      }
    }
  }

  Button& getButton(uint8_t channel) {
    return buttons[channel];
  }

  void printStatus() {
    Serial.print("buttons: ");
    for (int i=0;i<6;++i){
      Serial.print((cs.buttons & (1<<i))== (1<<i));
      Serial.print(" ");
    }
    Serial.println("");
    Serial.print("adc1: ");
    Serial.print(cs.adc_values[0]);
    Serial.print(" adc2: ");
    Serial.println(cs.adc_values[1]);
    Serial.println("LEDs:");
    for (int i=0; i<num_leds; ++i) {
      Serial.print(" led_pwm[x]: ");
      Serial.print(led_pwm[i]);
      Serial.print(" light: ");
      Serial.println(lights[i].get_pwm());
    }
  }
};

Light::Light(LEDController& controller, uint8_t channel) : 
      controller(controller), 
      channel(channel) {}
void Light::set_pwm(uint8_t value) {
    controller.led_pwm[channel] = value;
  }
uint8_t Light::get_pwm() {
    return controller.led_pwm[channel];
  }