#include <LiquidCrystal_I2C.h> 
#include <Servo.h> 
#include <Wire.h> 
#include <PCF8574.h> 
#include <AccelStepper.h> 
 
#define LCDADDR 0x27 
#define PCFADDR 0x20 
 
#define TCSout A3 
#define S0 10 
#define S1 11 
#define S2 12 
#define S3 13 
 
#define LEFT    0 
#define OK      1 
#define RIGHT   2 
#define PCF_GREEN_PIN   3 
#define PCF_YELLOW_PIN  4 
#define PCF_RED_PIN     5 
#define PCF_BUZZER_PIN  6 
 
#define place_arm_1 2 
#define place_arm_2 3 
#define arm_1 4 
#define arm_2 5 
 
#define IN_1 6 
#define IN_2 7 
#define IN_3 8 
#define IN_4 9 
 
#define Trig A1 
#define Echo A2 
#define PHOTO_RESISTOR A0 
 
PCF8574 PCF(PCFADDR); 
LiquidCrystal_I2C LCD(LCDADDR, 16, 2); 
 
Servo ARM_1; 
Servo ARM_2; 
Servo SERVOPLACE_1; 
Servo SERVOPLACE_2; 
 
bool setting = false; 
bool set_colour = false; 
 
const float STEP = 4096.0; 
float duration = 0; 
float distance = 0; 
int targetpos = 0; 
float target = 0; 
float momentpos = 0; 
float armpos = 30; 
float arm_aim = 0; 
bool stepdone = false; 
bool placedone = false; 
bool armdone = false; 
float arm_do = 30; 
constexpr float pi = 3.14159265359f; 
 
int val = 0; 
int colourpos1 = 1; 
int colourpos2 = 1; 
int colourpos3 = 1; 
int aim = 0; 
int RorS = 0; 
 
AccelStepper chain_step(AccelStepper::HALF4WIRE, IN_1, IN_3, IN_2, IN_4); 
 
enum class System_State { 
  SET_UP, 
  WAIT_PRODUCT, 
  PROCESS, 
  PLACE, 
  ARM_RUNNING, 
  HOMING, 
  STOPPED, 
}; 
 
int colour_choice = 1; 
const char* colour[11] = { 
  " ", "RED", "BLUE", "GREEN", "BLACK", "WHITE", 
  "YELLOW", "PINK", "BROWN", "ORANGE", "PURPLE" 
}; 
  
const char* systemMenu[4] = { " ", "START", "STOP", "RESTART" }; 
int sys_choice = 1; 
 
unsigned long timescaling = 0; 
 
enum class HMI_State { 
  SET_COLOUR_POS1, 
  SET_COLOUR_POS2, 
  SET_COLOUR_POS3, 
  SET_SYS 
}; 
 
System_State systemstate = System_State::SET_UP; 
HMI_State hmistate = HMI_State::SET_COLOUR_POS1; 
 
int pos1[2][3]; 
int pos2[2][3]; 
int pos3[2][3]; 
 
struct colourpick { 
  int RED[2][3] = { 
    {50, 200, 175}, 
    {100, 250, 225} 
  }; 
  int BLUE[2][3] = { 
    {250, 125, 50}, 
    {300, 175, 100} 
  }; 
  int GREEN[2][3] = { 
    {200, 140, 150}, 
    {250, 200, 100} 
  }; 
  int BLACK[2][3] = { 
    {300, 300, 225}, 
    {350, 350, 275} 
  }; 
  int WHITE[2][3] = { 
    {0, 0, 0}, 
    {50, 50, 50} 
  }; 
  int YELLOW[2][3] = { 
    {50, 50, 75}, 
    {100, 100, 125} 
  }; 
  int PINK[2][3] = { 
    {75, 225, 225}, 
    {125, 275, 275} 
  }; 
  int BROWN[2][3] = { 
    {150, 225, 200}, 
    {225, 275, 250} 
  }; 
  int ORANGE[2][3] = { 
    {50, 150, 125}, 
    {100, 200, 175} 
  }; 
  int PURPLE[2][3] = { 
    {200, 250, 150}, 
    {250, 300, 200} 
  }; 
}; 
 
colourpick colour_pick; 
 
void setline(uint8_t row); 
void product_detection(); 
void colour_detection(); 
void chain(int position); 
void chainpos1(); 
void chainpos2(); 
void chainpos3(); 
void chain_unknown(); 
void place(int position); 
void placepos1(); 
void placepos2(); 
void placepos3(); 
void place_unknown(); 
void arm(); 
void homing(); 
void stopped(); 
void feedback(int position); 
void feedback_pos1(); 
void feedback_pos2(); 
void feedback_pos3(); 
void dimension(); 
bool check_pos(); 
void set_productpos(); 
void green_on(); 
void green_off(); 
void yellow_on(); 
void yellow_off(); 
void red_on(); 
void red_off(); 
void buzzer(); 
void left(); 
void right(); 
void ok(); 
void button(); 
 
void copy_colour(int dst[2][3], int colour_id) { 
  int (*src)[3] = nullptr; 
 
  switch (colour_id) { 
    case 1:  src = colour_pick.RED; break; 
    case 2:  src = colour_pick.BLUE; break; 
    case 3:  src = colour_pick.GREEN; break; 
    case 4:  src = colour_pick.BLACK; break; 
    case 5:  src = colour_pick.WHITE; break; 
    case 6:  src = colour_pick.YELLOW; break; 
    case 7:  src = colour_pick.PINK; break; 
    case 8:  src = colour_pick.BROWN; break; 
    case 9:  src = colour_pick.ORANGE; break; 
    case 10: src = colour_pick.PURPLE; break; 
    default: return; 
  } 
 
  for (int i = 0; i < 2; i++) { 
    for (int j = 0; j < 3; j++) { 
      dst[i][j] = src[i][j]; 
    } 
  } 
} 
 
void setline(uint8_t row) { 
  LCD.setCursor(0, row); 
  LCD.print("                "); 
  LCD.setCursor(0, row); 
} 
 
void product_detection() { 
  val = analogRead(PHOTO_RESISTOR); 
 
  if (val >= 200) { 
    setline(0); 
    LCD.print("PRODUCT DETECTED"); 
    delay(500); 
  } 
} 
 
void colour_detection() { 
  int R = 0; 
  int G = 0; 
  int B = 0; 
 
  digitalWrite(S2, LOW); 
  digitalWrite(S3, LOW); 
  R = pulseIn(TCSout, LOW, 2000); 
 
  digitalWrite(S2, HIGH); 
  digitalWrite(S3, HIGH); 
  G = pulseIn(TCSout, LOW, 2000); 
 
  digitalWrite(S2, LOW); 
  digitalWrite(S3, HIGH); 
  B = pulseIn(TCSout, LOW, 2000); 
 
  if (R >= pos1[0][0] && R <= pos1[1][0] && 
      G >= pos1[0][1] && G <= pos1[1][1] && 
      B >= pos1[0][2] && B <= pos1[1][2]) { 
    aim = 1; 
    setline(0); 
    LCD.print("FOUND "); 
    setline(1); 
    LCD.print(colour[colourpos1]); 
    systemstate = System_State::PLACE; 
  } 
  else if (R >= pos2[0][0] && R <= pos2[1][0] && 
           G >= pos2[0][1] && G <= pos2[1][1] && 
           B >= pos2[0][2] && B <= pos2[1][2]) { 
    aim = 2; 
    setline(0); 
    LCD.print("FOUND "); 
    setline(1); 
    LCD.print(colour[colourpos2]); 
    systemstate = System_State::PLACE; 
  } 
  else if (R >= pos3[0][0] && R <= pos3[1][0] && 
           G >= pos3[0][1] && G <= pos3[1][1] && 
           B >= pos3[0][2] && B <= pos3[1][2]) { 
    aim = 3; 
    setline(0); 
    LCD.print("FOUND "); 
    setline(1); 
    LCD.print(colour[colourpos3]); 
    systemstate = System_State::PLACE; 
  } 
  else { 
    setline(0); 
    LCD.print("UNKNOWN"); 
    aim = 4; 
    systemstate = System_State::PLACE; 
  } 
} 
 
void chain(int position) { 
  setline(0); 
  LCD.print("RUNNING..."); 
 
  if (position == 1) chainpos1(); 
  if (position == 2) chainpos2(); 
  if (position == 3) chainpos3(); 
  if (position == 4) chain_unknown(); 
} 
 
bool stepping = false;
void go_to(int targetpos){
  chain_step.moveTo(targetpos);
  stepping = true;
}
void check_step(){
  if(stepping == true && chain_step.distanceToGo() ==0 ){
    stepping = false;
    stepdone = true;
  }
}
void chainpos1() { 
  if (!stepdone) { 
    targetpos = (4.5 / (8 * pi)) * STEP; 
    chain_step.run();
    go_to(targetpos); 
    check_step(); 
  } 
 
  feedback(1); 
 
  if (stepdone && placedone) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false; 
    placedone = false; 
  } 
} 
 
void chainpos2() { 
  if (!stepdone) { 
    targetpos = (10.5 / (8 * pi)) * STEP; 
    chain_step.run();
    go_to(targetpos); 
    check_step(); 
  }  
 
  feedback(2); 
 
  if (stepdone && placedone) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false; 
    placedone = false; 
  } 
} 
 
void chainpos3() { 
  if (!stepdone) { 
    targetpos = (16.5 / (8 * pi)) * STEP; 
    chain_step.run();
    go_to(targetpos); 
    check_step(); 
  }  
 
  feedback(3); 
 
  if (stepdone && placedone) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false; 
    placedone = false; 
  } 
} 
 
void chain_unknown() { 
  if (!stepdone) { 
    chain_step.move(-90); 
    chain_step.runToPosition(); 
    stepdone = true; 
  } 
 
  setline(0); 
  LCD.print("REMOVED UNKNOWN"); 
  systemstate = System_State::HOMING; 
  stepdone = false; 
} 
 
void place(int position) { 
  if (position == 1) placepos1(); 
  if (position == 2) placepos2(); 
  if (position == 3) placepos3(); 
  if (position == 4) place_unknown(); 
} 
 
void placepos1() { 
  placedone = true; 
 
  if (stepdone && placedone) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false; 
    placedone = false; 
  } 
} 
 
void placepos2() { 
  arm_aim = 90; 
 
  if (armpos < arm_aim) { 
    armpos++; 
    SERVOPLACE_1.write(armpos); 
    SERVOPLACE_2.write(180 - armpos); 
    delay(8); 
  } 
 
  if (armpos >= arm_aim) { 
    placedone = true; 
  } 
 
  if (stepdone && placedone) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false; 
    placedone = false; 
  } 
} 
 
void placepos3() { 
  arm_aim = 150; 
 
  if (armpos < arm_aim) { 
    armpos++; 
    SERVOPLACE_1.write(armpos); 
    SERVOPLACE_2.write(150 - armpos); 
    delay(8); 
  } 
 
  if (armpos >= arm_aim) { 
    placedone = true; 
  } 
 
  if (stepdone && placedone) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false; 
    placedone = false; 
  } 
} 
 
void place_unknown() { 
  placedone = true; 
 
  if (stepdone && placedone) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false; 
    placedone = false; 
  } 
} 
 
void arm() { 
  if (arm_do < 90) { 
    arm_do++; 
    ARM_1.write(arm_do); 
    ARM_2.write(180 - arm_do); 
    delay(8); 
  } 
  else { 
    LCD.print("COMPLETE"); 
    systemstate = System_State::HOMING; 
  } 
} 
 
void homing(int aim) { 
  if (!stepdone) {  
    if(aim == 1){
      targetpos = (5.5 / (8 * pi)) * STEP;
      chain_step.run();
      go_to(-targetpos);
      check_step();
    }
    else if(aim == 2){
      targetpos = (11.5 / (8 * pi)) * STEP;
      chain_step.run();
      go_to(-targetpos);
      check_step();
    }
    else if(aim == 3){
      targetpos = (17.5 / (8 * pi)) * STEP;
      chain_step.run();
      go_to(-targetpos);
      check_step();
    }
  } 
 
  if (armpos > 0) { 
    armpos--; 
    SERVOPLACE_1.write((int)armpos); 
    SERVOPLACE_2.write((int)armpos); 
    delay(8); 
  } 
  else { 
    placedone = true; 
  } 
 
  if (arm_do > 0) { 
    arm_do--; 
    ARM_1.write(arm_do); 
    ARM_2.write(180 - arm_do); 
    delay(16); 
  } 
  else { 
    armdone = true; 
  } 
 
  if (stepdone && placedone && armdone) { 
    stepdone = false; 
    placedone = false; 
    armdone = false; 
    aim = 1;
 
    if (RorS == 0) { 
      systemstate = System_State::WAIT_PRODUCT; 
    } 
    else { 
      systemstate = System_State::SET_UP; 
      hmistate = HMI_State::SET_COLOUR_POS1; 
      RorS = 0; 
      setting = false; 
    } 
  } 
} 
 
void stopped() { 
  set_colour = false; 
  RorS = 1; 
 
  if (!setting) { 
    setline(0); 
    LCD.print("STOPPED"); 
    setline(1); 
    LCD.print("PRESS OK TO BACK"); 
    setting = true; 
  } 
 
  int noti = PCF.read(OK); 
  if (noti == LOW) { 
    systemstate = System_State::HOMING; 
    setting = false; 
  } 
} 
 
void feedback(int position) { 
  if (position == 1) feedback_pos1(); 
  else if (position == 2) feedback_pos2(); 
  else if (position == 3) feedback_pos3(); 
} 
 
void dimension() { 
  digitalWrite(Trig, LOW); 
  delayMicroseconds(2); 
  digitalWrite(Trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(Trig, LOW); 
 
  duration = pulseIn(Echo, HIGH, 3000); 
  distance = duration * 0.034f / 2.0f; 
} 
 
void feedback_pos1() { 
  target = 17.5; 
  dimension(); 
  if (distance < target) { 
    systemstate = System_State::ARM_RUNNING; 
    stepdone = false;
    placedone = false;
  } 
} 
 
void feedback_pos2() { 
  target = 11.5; 
  dimension(); 
  if (distance < target) { 
    systemstate = System_State::ARM_RUNNING; 
  } 
} 
 
void feedback_pos3() { 
  target = 5.3; 
  dimension(); 
  if (distance < target) { 
    systemstate = System_State::ARM_RUNNING; 
  } 
} 
 
bool check_pos() { 
  const float correctpos = 25.45; 
  dimension(); 
  float wrongpos = correctpos - distance; 
 
  if (wrongpos < -1 || wrongpos > 1) { 
    return false; 
  } 
  return true; 
} 
 
void set_productpos() { 
  const float correctpos = 25.45; 
  dimension(); 
  float wrongpos = correctpos - distance; 
  int go_to = (wrongpos / (8 * pi)) * STEP; 
 
  chain_step.move(go_to); 
  chain_step.runToPosition(); 
} 
 
void green_on() { 
  PCF.write(PCF_GREEN_PIN, HIGH); 
} 
 
void green_off() { 
  PCF.write(PCF_GREEN_PIN, LOW); 
} 
 
void yellow_on() { 
  PCF.write(PCF_YELLOW_PIN, HIGH); 
} 
 
void yellow_off() { 
  PCF.write(PCF_YELLOW_PIN, LOW); 
} 
 
void red_on() { 
  PCF.write(PCF_RED_PIN, HIGH); 
} 
 
void red_off() { 
  PCF.write(PCF_RED_PIN, LOW); 
} 
 
void buzzer() { 
  PCF.write(PCF_BUZZER_PIN, HIGH); 
  delay(200); 
  PCF.write(PCF_BUZZER_PIN, LOW); 
} 
 
void left() { 
  switch (hmistate) { 
    case HMI_State::SET_COLOUR_POS1: 
    case HMI_State::SET_COLOUR_POS2: 
    case HMI_State::SET_COLOUR_POS3: 
      if (colour_choice == 1) colour_choice = 10; 
      else colour_choice--;
      setline(1); 
      LCD.print(colour[colour_choice]); 
      break; 
 
    case HMI_State::SET_SYS: 
      if (sys_choice == 1) sys_choice = 3; 
      else sys_choice--; 
      setline(0); 
      LCD.print(systemMenu[sys_choice]); 
      break; 
  } 
} 
 
void right() { 
  switch (hmistate) { 
    case HMI_State::SET_COLOUR_POS1: 
    case HMI_State::SET_COLOUR_POS2: 
    case HMI_State::SET_COLOUR_POS3: 
      if (colour_choice == 10) colour_choice = 1; 
      else colour_choice++; 
      setline(1); 
      LCD.print(colour[colour_choice]); 
      break; 
 
    case HMI_State::SET_SYS: 
      if (sys_choice == 3) sys_choice = 1; 
      else sys_choice++; 
      setline(0); 
      LCD.print(systemMenu[sys_choice]); 
      break; 
  } 
} 
 
void ok() { 
  switch (hmistate) { 
    case HMI_State::SET_COLOUR_POS1: 
      copy_colour(pos1, colour_choice); 
      colourpos1 = colour_choice; 
      setline(1); 
      LCD.print("OK"); 
      delay(500); 
      colour_choice = 1; 
      setline(0); 
      LCD.print("POS 2 CHOICE"); 
      setline(1); 
      LCD.print(colour[colour_choice]); 
      hmistate = HMI_State::SET_COLOUR_POS2; 
      break; 
 
    case HMI_State::SET_COLOUR_POS2: 
      copy_colour(pos2, colour_choice); 
      colourpos2 = colour_choice; 
      setline(1); 
      LCD.print("OK"); 
      delay(500); 
      colour_choice = 1; 
      setline(0); 
      LCD.print("POS 3 CHOICE"); 
      setline(1); 
      LCD.print(colour[colour_choice]); 
      hmistate = HMI_State::SET_COLOUR_POS3; 
      break; 
 
    case HMI_State::SET_COLOUR_POS3: 
      copy_colour(pos3, colour_choice); 
      colourpos3 = colour_choice; 
      setline(0); 
      LCD.print("SYSTEM CHOICE"); 
      setline(1); 
      LCD.print(systemMenu[sys_choice]); 
      hmistate = HMI_State::SET_SYS; 
      set_colour = true; 
      break; 
 
    case HMI_State::SET_SYS: 
      setline(1); 
      LCD.print("OK"); 
      delay(500); 
 
      if (sys_choice == 1) { 
        systemstate = System_State::WAIT_PRODUCT; 
        setting = false; 
      } 
      else if (sys_choice == 2) { 
        systemstate = System_State::STOPPED; 
        setting = false; 
      } 
      else if (sys_choice == 3) { 
        systemstate = System_State::HOMING; 
        setting = false; 
      } 
      break; 
  } 
} 
 
void button() { 
  bool leftpressed  = (PCF.read(LEFT)  == HIGH); 
  bool okpressed    = (PCF.read(OK)    == HIGH); 
  bool rightpressed = (PCF.read(RIGHT) == HIGH); 
 
  if (leftpressed) { 
    left(); 
    delay(300);
  } 
  else if (rightpressed) { 
    right(); 
    delay(300);
  } 
  else if (okpressed) { 
    ok(); 
    delay(300);
  } 
} 
 
void setup() { 
  Serial.begin(9600); 
 
  Wire.begin(); 
  PCF.begin(); 
 
  PCF.write(LEFT, HIGH); 
  PCF.write(OK, HIGH); 
  PCF.write(RIGHT, HIGH); 
 
  PCF.write(PCF_GREEN_PIN, LOW); 
  PCF.write(PCF_YELLOW_PIN, LOW); 
  PCF.write(PCF_RED_PIN, LOW); 
  PCF.write(PCF_BUZZER_PIN, LOW); 
 
  LCD.init(); 
  LCD.backlight(); 
 
  chain_step.setMaxSpeed(1000.0); 
  chain_step.setAcceleration(500.0); 
 
  pinMode(Trig, OUTPUT); 
  pinMode(Echo, INPUT); 
 
  pinMode(S0, OUTPUT); 
  pinMode(S1, OUTPUT); 
  pinMode(S2, OUTPUT); 
  pinMode(S3, OUTPUT); 
  pinMode(TCSout, INPUT); 
 
  digitalWrite(S0, HIGH); 
  digitalWrite(S1, LOW); 
 
  SERVOPLACE_1.attach(place_arm_1); 
  SERVOPLACE_2.attach(place_arm_2); 
  ARM_1.attach(arm_1); 
  ARM_2.attach(arm_2); 
 
  SERVOPLACE_1.write(30); 
  SERVOPLACE_2.write(30); 
  ARM_1.write(30); 
  ARM_2.write(150); 
 
  momentpos = 0; 
  armpos = 30; 
  arm_do = 30; 
 
  setline(0); 
  LCD.print("PICK COLOUR POS1"); 
  setline(1); 
  LCD.print(colour[colour_choice]); 
} 

void loop() { 
  switch (systemstate) { 
    case System_State::SET_UP: 
      if (!setting) { 
        setline(0); 
        LCD.print("POS1 CALIBRATION"); 
        setline(1); 
        LCD.print(colour[colour_choice]); 
        setting = true; 
      } 
 
      switch (hmistate) { 
        case HMI_State::SET_COLOUR_POS1: 
        case HMI_State::SET_COLOUR_POS2: 
        case HMI_State::SET_COLOUR_POS3: 
          button(); 
          break; 
 
        case HMI_State::SET_SYS: 
          if (!setting) { 
            setline(0); 
            LCD.print("SETTING"); 
            setline(1); 
            LCD.print(systemMenu[sys_choice]); 
            setting = true; 
          } 
          button(); 
          break; 
      } 
      break; 
 
    case System_State::WAIT_PRODUCT: 
      red_off(); 
      green_on(); 
      product_detection(); 
 
      if (!check_pos()) { 
        set_productpos(); 
      } 
      else { 
        systemstate = System_State::PROCESS; 
      } 
      break; 
 
    case System_State::PROCESS: 
      green_off(); 
      yellow_on(); 
      colour_detection(); 
      break; 
 
    case System_State::PLACE: 
      yellow_off(); 
      chain(aim); 
      place(aim); 
      break; 
 
    case System_State::ARM_RUNNING: 
      arm(); 
      break; 
 
    case System_State::HOMING: 
      homing(aim); 
      break; 
 
    case System_State::STOPPED: 
      stopped(); 
      break; 
  } 
} 