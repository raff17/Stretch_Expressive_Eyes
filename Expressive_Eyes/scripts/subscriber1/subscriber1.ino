
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

#define PIN 4              // Arduino pin 6 to DIN of 8x32 matrix.
#define LED_COUNT 256      // 8x32 = 256 NeoPixel leds
#define BRIGHTNESS 8       // to reduce current for 256 NeoPixels

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);
bool Manip = false;
char incomingChar;
int i = 0;
int j = 0;
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const long interval = 7500;
const long interval2 = 9500;
int currentExpression;
bool emote;
String emotion = "base";

float MoFlL = 0.336; //3
float MoLB = 0.026; //5 
float MoBR = -0.252; //
float MoRFr = -0.563;
float MoFr = -0.908;

float vFdD = -0.337;
float vDB = 0.001;
float vBU = 0.139;
float vUFu = 0.277;
float upper_limit = 0.415;
float lower_limit = -1.917;


float MaFlL = -1.225; //3

float MaLB = -1.542; //5 
float MaBR = -1.859; //

float MaRFr = -2.196;

float MaFr = -4.045;
float horiz;
float vert;

float transition_counter = -.130;

//////////


void camOCb(const std_msgs::Float64MultiArray & state_msg){
  horiz = state_msg.data[0];
  vert = state_msg.data[1];
}

void cam1Cb(const std_msgs::String & state_msg){
  incomingChar = state_msg.data[0];
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("/head_camera_jointstate", camOCb);

ros::Subscriber<std_msgs::String> sub_1("/keyboard_input", cam1Cb);

void setup() 
{
  strip.begin();
  strip.show();            // Initialize all pixels to 'off'
  strip.setBrightness(BRIGHTNESS);   // overall brightness
  Serial.begin(57600);
  happyEyesMo();
  delay(5000);
  baseEyesMo();
  currentExpression = 1;
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_1);
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval){
    // save last time homie blinked
    previousMillis = currentMillis;
    if(Manip == false){
      blinkEyesMo(currentExpression);      
    }
    else{
      blinkEyesMa(currentExpression);
    }
  }
  if(emotion != "base"){
    if(currentMillis - previousMillis2 >= interval2){
      previousMillis2 = currentMillis; 
      emotion = "base";     
      
    }
  }

  
//  while (Serial.available()){
//    incomingChar = Serial.read();


    switch(incomingChar) {     
      case 'h':
        emotion = "happy";
        break;
      case 'b':
        emotion = "sad";
        break;
      case 'n':
        emotion = "angry";
        break;
      case 'c':
        emotion = "confused";
        break;
      case 'o':
        emotion = "base";
        break;      
    }


// replace to original code since that has connection to the join states
// this part is what makes the eyes transition from half of the LED to the other half
    if(horiz < -0.908 && transition_counter >= -0.908){
      Manip = true;
      i += 1;
      transition_counter = horiz;
    }
    else if(horiz < -0.908 && transition_counter < -0.908){
      Manip = true;
    }
    else if(horiz >= -0.908 && transition_counter < -0.908){
      Manip = false;
      i += 1;
      transition_counter = horiz;
    }
    else if(horiz >= -0.908 && transition_counter >= -0.908){
      Manip = false;
    }

    
    if(vert >= vDB && vert < vBU && horiz <= MaLB && horiz > MaBR && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      baseEyesMa();
      currentExpression = 1;
     
    }  
    else if(vert >= vBU && vert < vUFu && horiz <= MaLB && horiz > MaBR && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      slightUpMa();
      currentExpression = 2;
    }
    else if(vert >= vUFu && vert <= upper_limit && horiz <= MaFlL && horiz > MaRFr && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesUpMa();
      currentExpression = 3;
    } 
    else if(vert >= vFdD && vert < vDB && horiz <= MaLB && horiz > MaBR && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      slightDownMa();
      currentExpression = 4;
    }
    else if(vert >= lower_limit && vert < vFdD && horiz <= MaFlL && horiz > MaRFr && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesDownMa();
      currentExpression = 5;
    }
    else if(((vert >= vFdD && vert < vDB && horiz <= MaBR && horiz > MaRFr)||(vert >= lower_limit && vert < vFdD && horiz <= MaRFr && horiz > MaFr)) && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesDownLeftMa();
      currentExpression = 14;
    }
    else if(((vert >= vFdD && vert < vDB && horiz <= MaFlL && horiz > MaLB)||(vert >= lower_limit && vert < vFdD && horiz <= MoFr && horiz > MaFlL )) && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesDownRightMa();
      currentExpression = 15;
    } 
    else if(((vert >= vBU && vert < vUFu && horiz <= MaBR && horiz > MaRFr)||(vert >= vUFu && vert < upper_limit && horiz <= MaRFr && horiz > MaFr )) && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesUpLeftMa();
      currentExpression = 16;
    }
    else if(((vert >= vBU && vert < vUFu && horiz <= MaFlL && horiz > MaLB)||(vert >= vUFu && vert < upper_limit && horiz <= MoFr && horiz > MaFlL )) && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesUpRightMa();
      currentExpression = 17;
    }    
/////////// Manipulator side ///////////////////////////
    else if(vert >= vDB && vert < vBU && horiz <= MaFlL && horiz > MaLB && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      slightRightMa();
      currentExpression = 6;
    }    
    else if(vert >= vFdD && vert <= vUFu && horiz >= MaFlL && horiz < MoFr && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesRightMa();
      currentExpression = 7;
    } 
    else if(vert >= vDB && vert < vBU && horiz <= MaBR && horiz > MaRFr && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      slightLeftMa();
      currentExpression = 8; 
    }
    else if(vert >= vFdD && vert <= vUFu && horiz <= MaRFr && horiz > MaFr && Manip == true && emotion == "base"){
      if(i != j){
        mobToman();
        j = i;
      }
      eyesLeftMa();
      currentExpression = 9;    
    }

    if(vert >= vDB && vert < vBU && horiz <= MoLB && horiz > MoBR && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      baseEyesMo();
      currentExpression = 1;
     
    }  
    else if(vert >= vBU && vert < vUFu && horiz <= MoLB && horiz > MoBR && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      slightUpMo();
      currentExpression = 2;
    }
    else if(vert >= vUFu && vert <= upper_limit && horiz <= MoFlL && horiz > MoRFr && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesUpMo();
      currentExpression = 3;
    } 
    else if(vert >= vFdD && vert < vDB && horiz <= MoLB && horiz > MoBR && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      slightDownMo();
      currentExpression = 4;
    }
    else if(vert >= lower_limit && vert < vFdD && horiz <= MoFlL && horiz > MoRFr && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesDownMo();
      currentExpression = 5;
    } 
    else if(((vert >= vFdD && vert < vDB && horiz <= MoBR && horiz > MoRFr)||(vert >= lower_limit && vert < vFdD && horiz <= MoRFr && horiz > MoFr)) && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesUpLeftMo();
      currentExpression = 14;
    }
    else if(((vert >= vFdD && vert < vDB && horiz <= MoFlL && horiz > MoLB)||(vert >= lower_limit && vert < vFdD && horiz >= MoFlL && horiz <= 1.67)) && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesUpRightMo();
      currentExpression = 15;
    } 
    else if(((vert >= vBU && vert < vUFu && horiz <= MoBR && horiz > MoRFr)||(vert >= vUFu && vert < upper_limit && horiz <= MoRFr && horiz > MoFr )) && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesDownLeftMo();
      currentExpression = 16;
    }
    else if(((vert >= vBU && vert < vUFu && horiz <= MoFlL && horiz > MoLB)||(vert >= vUFu && vert < upper_limit && horiz >= MoFlL && horiz <= 1.67 )) && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesDownRightMo();
      currentExpression = 17;
    } 
/////////////// Front facing-eyes ///////////////////
    else if(vert >= vDB && vert < vBU && horiz <= MoFlL && horiz > MoLB && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      slightRightMo();
      currentExpression = 6;
    }    
    else if(vert >= vFdD && vert <= vUFu && horiz >= MoFlL && horiz <= 1.67 && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesRightMo();
      currentExpression = 7;
    } 
    else if(vert >= vDB && vert < vBU && horiz <= MoBR && horiz > MoRFr && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      slightLeftMo();
      currentExpression = 8; 
    }
    else if(vert >= vFdD && vert <= vUFu && horiz <= MoRFr && horiz > MoFr && Manip == false && emotion == "base"){
      if(i != j){
        manTomob();
        j = i;
      }
      eyesLeftMo();
      currentExpression = 9;    
    }

    else if(emotion == "happy"){
      currentExpression = 10;
      if(Manip == false){
        happyEyesMo();
      }
      else if (Manip == true){
        happyEyesMa();
      }
      
    }
    else if(emotion == "sad"){
      currentExpression = 11;
      if(Manip == false){
        sadEyesMo();
      }
      else if (Manip == true){
        sadEyesMa();
      }
    }
    else if(emotion == "angry"){
      currentExpression = 12;
      if(Manip == false){
        angryEyesMo();
      }
      else if (Manip == true){
        angryEyesMa();
      }
    }
    else if(emotion == "confused"){
      currentExpression = 13;
      if(Manip == false){
        confusedEyesMo();
      }
      else if (Manip == true){
        confusedEyesMa();
      } 
    } 
//      eyesLeftMo();
//      currentExpression = 9;    
    
    nh.spinOnce();
    delay(100);
}

 




// --------------------------------------------- Manipulation Eye States ------------------------------------------
void baseEyesMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(27, strip.Color(0, 250, 150));
  strip.setPixelColor(28, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(35, strip.Color(0, 250, 150));
  strip.setPixelColor(36, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 250, 150));
  strip.setPixelColor(92, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(99, strip.Color(0, 250, 150));
  strip.setPixelColor(100, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255));
  strip.show(); 
}
void confusedEyesMa(){
  strip.clear();
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(10, strip.Color(0, 0, 255));
  strip.setPixelColor(12, strip.Color(0, 0, 255));
  strip.setPixelColor(18, strip.Color(0, 0, 255));
  strip.setPixelColor(22, strip.Color(0, 0, 255));
  strip.setPixelColor(25, strip.Color(0, 0, 255));
  strip.setPixelColor(29, strip.Color(0, 0, 255));
  strip.setPixelColor(34, strip.Color(0, 0, 255));
  strip.setPixelColor(38, strip.Color(0, 0, 255));
  strip.setPixelColor(36, strip.Color(0, 250, 150));
  //strip.setPixelColor(35, strip.Color(0, 250, 150));
  strip.setPixelColor(41, strip.Color(0, 0, 255));
  strip.setPixelColor(45, strip.Color(0, 0, 255));
  //strip.setPixelColor(44, strip.Color(0, 250, 150));
  strip.setPixelColor(43, strip.Color(0, 250, 150));
  strip.setPixelColor(50, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(57, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(75, strip.Color(0, 0, 255));
  strip.setPixelColor(90, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 0, 255));
  strip.setPixelColor(92, strip.Color(0, 0, 255));
  strip.setPixelColor(93, strip.Color(0, 0, 255));
  strip.setPixelColor(97, strip.Color(0, 0, 255));
  strip.setPixelColor(102, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  //strip.setPixelColor(99, strip.Color(0, 250, 150));
  //strip.setPixelColor(100, strip.Color(0, 250, 150));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255));
  strip.setPixelColor(107, strip.Color(0, 250, 150));
  strip.setPixelColor(108, strip.Color(0, 250, 150));
//  strip.setPixelColor(111, strip.Color(0, 0, 255));
//  strip.setPixelColor(113, strip.Color(0, 0, 255));
//  strip.setPixelColor(118, strip.Color(0, 0, 255));
//  strip.setPixelColor(117, strip.Color(0, 0, 255));
//  strip.setPixelColor(116, strip.Color(0, 0, 255));
//  strip.setPixelColor(115, strip.Color(0, 0, 255));
//  strip.setPixelColor(114, strip.Color(0, 0, 255));
  //strip.setPixelColor(124, strip.Color(0, 0, 255));
  //strip.setPixelColor(125, strip.Color(0, 0, 255));
  strip.show(); 
}
void angryEyesMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(17, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(26, strip.Color(0, 250, 150));
  strip.setPixelColor(27, strip.Color(0, 250, 150));
  strip.setPixelColor(30, strip.Color(0, 0, 255));
  strip.setPixelColor(34, strip.Color(0, 0, 255));
  strip.setPixelColor(37, strip.Color(0, 250, 150));
  strip.setPixelColor(36, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(44, strip.Color(0, 0, 255));
  strip.setPixelColor(52, strip.Color(0, 0, 255));
  strip.setPixelColor(55, strip.Color(0, 0, 255));
  strip.setPixelColor(56, strip.Color(0, 0, 255));
  strip.setPixelColor(57, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  //strip.setPixelColor(60, strip.Color(0, 0, 255));
  //strip.setPixelColor(61, strip.Color(0, 0, 255));
  //strip.setPixelColor(66, strip.Color(0, 0, 255));
  //strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(70, strip.Color(0, 0, 255));
  strip.setPixelColor(71, strip.Color(0, 0, 255));
  strip.setPixelColor(72, strip.Color(0, 0, 255));
  strip.setPixelColor(75, strip.Color(0, 0, 255));  
  strip.setPixelColor(83, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 250, 150));
  strip.setPixelColor(90, strip.Color(0, 250, 150));
  strip.setPixelColor(93, strip.Color(0, 0, 255));
  strip.setPixelColor(97, strip.Color(0, 0, 255));
  strip.setPixelColor(100, strip.Color(0, 250, 150));
  strip.setPixelColor(101, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(110, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255));
//  strip.setPixelColor(67, strip.Color(50, 0, 155));
//  strip.setPixelColor(31, strip.Color(50, 0, 155));
//  strip.setPixelColor(32, strip.Color(50, 0, 155));
//  strip.setPixelColor(46, strip.Color(50, 0, 155));
//  strip.setPixelColor(50, strip.Color(50, 0, 155));
//  strip.setPixelColor(60, strip.Color(50, 0, 155));
//  strip.setPixelColor(96, strip.Color(50, 0, 155));
//  strip.setPixelColor(95, strip.Color(50, 0, 155));
//  strip.setPixelColor(81, strip.Color(50, 0, 155));
//  strip.setPixelColor(77, strip.Color(50, 0, 155));
  strip.show(); 
}
void sadEyesMa(){
  strip.clear();
  strip.setPixelColor(57, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(6, strip.Color(0, 0, 255));
  strip.setPixelColor(8, strip.Color(0, 0, 255));
  strip.setPixelColor(11, strip.Color(0, 0, 255));
  strip.setPixelColor(29, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255)); 
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(42, strip.Color(0, 250, 150));
  strip.setPixelColor(41, strip.Color(0, 250, 150));
  strip.setPixelColor(19, strip.Color(0, 0, 255));
  strip.setPixelColor(33, strip.Color(0, 0, 255));
  strip.setPixelColor(37, strip.Color(0, 250, 150));
  strip.setPixelColor(38, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(46, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(55, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));  
  strip.setPixelColor(70, strip.Color(0, 0, 255));
  strip.setPixelColor(72, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(81, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));  
  strip.setPixelColor(89, strip.Color(0, 250, 150));
  strip.setPixelColor(90, strip.Color(0, 250, 150));
  strip.setPixelColor(94, strip.Color(0, 0, 255));
  strip.setPixelColor(108, strip.Color(0, 0, 255));
  strip.setPixelColor(85, strip.Color(0, 250, 150));
  strip.setPixelColor(86, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(98, strip.Color(0, 0, 255));
  strip.setPixelColor(116, strip.Color(0, 0, 255));
  strip.setPixelColor(119, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(121, strip.Color(0, 0, 255));
  strip.show(); 
}

void closeEyesMa(){
  strip.clear();
  strip.setPixelColor(11, strip.Color(0, 0, 255));
  strip.setPixelColor(21, strip.Color(0, 0, 255));
  strip.setPixelColor(25, strip.Color(0, 0, 255));
  strip.setPixelColor(38, strip.Color(0, 0, 255));
  strip.setPixelColor(42, strip.Color(0, 0, 255));
  strip.setPixelColor(52, strip.Color(0, 0, 255));
  strip.setPixelColor(75, strip.Color(0, 0, 255));
  strip.setPixelColor(85, strip.Color(0, 0, 255));
  strip.setPixelColor(89, strip.Color(0, 0, 255));
  strip.setPixelColor(102, strip.Color(0, 0, 255));
  strip.setPixelColor(106, strip.Color(0, 0, 255));
  strip.setPixelColor(116, strip.Color(0, 0, 150));
  strip.show();
}
void happyEyesMa(){
  strip.clear();
  strip.setPixelColor(12, strip.Color(0, 0, 255));
  strip.setPixelColor(18, strip.Color(0, 0, 255));
  strip.setPixelColor(30, strip.Color(0, 0, 255));
  strip.setPixelColor(33, strip.Color(0, 0, 255));
  strip.setPixelColor(45, strip.Color(0, 0, 255));
  strip.setPixelColor(51, strip.Color(0, 0, 255));
  strip.setPixelColor(76, strip.Color(0, 0, 255));
  strip.setPixelColor(82, strip.Color(0, 0, 255));
  strip.setPixelColor(94, strip.Color(0, 0, 255));
  strip.setPixelColor(97, strip.Color(0, 0, 255));
  strip.setPixelColor(109, strip.Color(0, 0, 255));
  strip.setPixelColor(115, strip.Color(0, 0, 150));
  strip.show();
}
void sleepEyesMa(){
  strip.clear();
  strip.setPixelColor(11, strip.Color(0, 0, 255));
  strip.setPixelColor(20, strip.Color(0, 0, 255));
  strip.setPixelColor(27, strip.Color(0, 0, 255));
  strip.setPixelColor(36, strip.Color(0, 0, 255));
  strip.setPixelColor(43, strip.Color(0, 0, 255));
  strip.setPixelColor(52, strip.Color(0, 0, 255));
  strip.setPixelColor(75, strip.Color(0, 0, 255));
  strip.setPixelColor(84, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 0, 255));
  strip.setPixelColor(100, strip.Color(0, 0, 255));
  strip.setPixelColor(107, strip.Color(0, 0, 255));
  strip.setPixelColor(116, strip.Color(0, 0, 150));
  strip.show();
}
void slightRightMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(44, strip.Color(0, 250, 150));
  strip.setPixelColor(43, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(35, strip.Color(0, 250, 150));
  strip.setPixelColor(36, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(107, strip.Color(0, 250, 150));
  strip.setPixelColor(108, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(99, strip.Color(0, 250, 150));
  strip.setPixelColor(100, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void eyesDownRightMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(37, strip.Color(0, 250, 150));
  strip.setPixelColor(36, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(43, strip.Color(0, 250, 150));
  strip.setPixelColor(42, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(107, strip.Color(0, 250, 150));
  strip.setPixelColor(106, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(100, strip.Color(0, 250, 150));
  strip.setPixelColor(101, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void eyesUpRightMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(44, strip.Color(0, 250, 150));
  strip.setPixelColor(45, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(35, strip.Color(0, 250, 150));
  strip.setPixelColor(34, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(109, strip.Color(0, 250, 150));
  strip.setPixelColor(108, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(99, strip.Color(0, 250, 150));
  strip.setPixelColor(98, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void slightLeftMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(27, strip.Color(0, 250, 150));
  strip.setPixelColor(28, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(19, strip.Color(0, 250, 150));
  strip.setPixelColor(20, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 250, 150));
  strip.setPixelColor(92, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(83, strip.Color(0, 250, 150));
  strip.setPixelColor(84, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void eyesDownLeftMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(27, strip.Color(0, 250, 150));
  strip.setPixelColor(26, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(21, strip.Color(0, 250, 150));
  strip.setPixelColor(20, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 250, 150));
  strip.setPixelColor(90, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(85, strip.Color(0, 250, 150));
  strip.setPixelColor(84, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void eyesUpLeftMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(29, strip.Color(0, 250, 150));
  strip.setPixelColor(28, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(19, strip.Color(0, 250, 150));
  strip.setPixelColor(18, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(93, strip.Color(0, 250, 150));
  strip.setPixelColor(92, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(83, strip.Color(0, 250, 150));
  strip.setPixelColor(82, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void slightUpMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(29, strip.Color(0, 250, 150));
  strip.setPixelColor(28, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(35, strip.Color(0, 250, 150));
  strip.setPixelColor(34, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(93, strip.Color(0, 250, 150));
  strip.setPixelColor(92, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(99, strip.Color(0, 250, 150));
  strip.setPixelColor(98, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void slightDownMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(27, strip.Color(0, 250, 150));
  strip.setPixelColor(26, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(37, strip.Color(0, 250, 150));
  strip.setPixelColor(36, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 250, 150));
  strip.setPixelColor(90, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(101, strip.Color(0, 250, 150));
  strip.setPixelColor(100, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void eyesDownMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(25, strip.Color(0, 250, 150));
  strip.setPixelColor(26, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(37, strip.Color(0, 250, 150));
  strip.setPixelColor(38, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(89, strip.Color(0, 250, 150));
  strip.setPixelColor(90, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(101, strip.Color(0, 250, 150));
  strip.setPixelColor(102, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void eyesUpMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(29, strip.Color(0, 250, 150));
  strip.setPixelColor(30, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(33, strip.Color(0, 250, 150));
  strip.setPixelColor(34, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(93, strip.Color(0, 250, 150));
  strip.setPixelColor(94, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(97, strip.Color(0, 250, 150));
  strip.setPixelColor(98, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255));
  strip.show(); 
}
void eyesLeftMa(){
  strip.clear();
  strip.setPixelColor(2, strip.Color(0, 0, 255));
  strip.setPixelColor(3, strip.Color(0, 0, 255));
  strip.setPixelColor(4, strip.Color(0, 0, 255));
  strip.setPixelColor(5, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(11, strip.Color(0, 250, 150));
  strip.setPixelColor(12, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(19, strip.Color(0, 250, 150));
  strip.setPixelColor(20, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(53, strip.Color(0, 0, 255));
  strip.setPixelColor(52, strip.Color(0, 0, 255));
  strip.setPixelColor(51, strip.Color(0, 0, 255));
  strip.setPixelColor(50, strip.Color(0, 0, 255));
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(75, strip.Color(0, 250, 150));
  strip.setPixelColor(76, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(83, strip.Color(0, 250, 150));
  strip.setPixelColor(84, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(117, strip.Color(0, 0, 255));
  strip.setPixelColor(116, strip.Color(0, 0, 255));
  strip.setPixelColor(115, strip.Color(0, 0, 255));
  strip.setPixelColor(114, strip.Color(0, 0, 255)); 
  strip.show();
}
void eyesRightMa(){
  strip.clear();
  strip.setPixelColor(13, strip.Color(0, 0, 255));
  strip.setPixelColor(12, strip.Color(0, 0, 255));
  strip.setPixelColor(11, strip.Color(0, 0, 255));
  strip.setPixelColor(10, strip.Color(0, 0, 255));
  strip.setPixelColor(9, strip.Color(0, 0, 255));
  strip.setPixelColor(14, strip.Color(0, 0, 255));
  strip.setPixelColor(16, strip.Color(0, 0, 255));
  strip.setPixelColor(23, strip.Color(0, 0, 255));
  strip.setPixelColor(24, strip.Color(0, 0, 255));
  strip.setPixelColor(44, strip.Color(0, 250, 150));
  strip.setPixelColor(43, strip.Color(0, 250, 150));
  strip.setPixelColor(31, strip.Color(0, 0, 255));
  strip.setPixelColor(32, strip.Color(0, 0, 255));
  strip.setPixelColor(51, strip.Color(0, 250, 150));
  strip.setPixelColor(52, strip.Color(0, 250, 150));
  strip.setPixelColor(39, strip.Color(0, 0, 255));
  strip.setPixelColor(40, strip.Color(0, 0, 255));
  strip.setPixelColor(47, strip.Color(0, 0, 255));
  strip.setPixelColor(49, strip.Color(0, 0, 255));
  strip.setPixelColor(54, strip.Color(0, 0, 255));
  strip.setPixelColor(58, strip.Color(0, 0, 255));
  strip.setPixelColor(59, strip.Color(0, 0, 255));
  strip.setPixelColor(60, strip.Color(0, 0, 255));
  strip.setPixelColor(61, strip.Color(0, 0, 255));
  strip.setPixelColor(77, strip.Color(0, 0, 255));
  strip.setPixelColor(76, strip.Color(0, 0, 255));
  strip.setPixelColor(75, strip.Color(0, 0, 255));
  strip.setPixelColor(74, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(107, strip.Color(0, 250, 150));
  strip.setPixelColor(108, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(115, strip.Color(0, 250, 150));
  strip.setPixelColor(116, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255)); 
  strip.show();
}
void transitionEyes() {
  strip.clear();
  strip.setPixelColor(66, strip.Color(0, 0, 255));
  strip.setPixelColor(67, strip.Color(0, 0, 255));
  strip.setPixelColor(68, strip.Color(0, 0, 255));
  strip.setPixelColor(69, strip.Color(0, 0, 255));
  strip.setPixelColor(73, strip.Color(0, 0, 255));
  strip.setPixelColor(78, strip.Color(0, 0, 255));
  strip.setPixelColor(80, strip.Color(0, 0, 255));
  strip.setPixelColor(87, strip.Color(0, 0, 255));
  strip.setPixelColor(88, strip.Color(0, 0, 255));
  strip.setPixelColor(91, strip.Color(0, 250, 150));
  strip.setPixelColor(92, strip.Color(0, 250, 150));
  strip.setPixelColor(95, strip.Color(0, 0, 255));
  strip.setPixelColor(96, strip.Color(0, 0, 255));
  strip.setPixelColor(99, strip.Color(0, 250, 150));
  strip.setPixelColor(100, strip.Color(0, 250, 150));
  strip.setPixelColor(103, strip.Color(0, 0, 255));
  strip.setPixelColor(104, strip.Color(0, 0, 255));
  strip.setPixelColor(111, strip.Color(0, 0, 255));
  strip.setPixelColor(113, strip.Color(0, 0, 255));
  strip.setPixelColor(118, strip.Color(0, 0, 255));
  strip.setPixelColor(122, strip.Color(0, 0, 255));
  strip.setPixelColor(123, strip.Color(0, 0, 255));
  strip.setPixelColor(124, strip.Color(0, 0, 255));
  strip.setPixelColor(125, strip.Color(0, 0, 255));
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(155, strip.Color(0, 250, 150));
  strip.setPixelColor(156, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(163, strip.Color(0, 250, 150));
  strip.setPixelColor(164, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.show();  
}

// ---------------------------------------- Mobility Eye States ----------------------------------------------
void baseEyesMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(155, strip.Color(0, 250, 150));
  strip.setPixelColor(156, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(163, strip.Color(0, 250, 150));
  strip.setPixelColor(164, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(219, strip.Color(0, 250, 150));
  strip.setPixelColor(220, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(227, strip.Color(0, 250, 150));
  strip.setPixelColor(228, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void sadEyesMo(){
  strip.clear();
  strip.setPixelColor(185, strip.Color(0, 0, 255));
  strip.setPixelColor(198, strip.Color(0, 0, 255));
  strip.setPixelColor(134, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(136, strip.Color(0, 0, 255));
  strip.setPixelColor(139, strip.Color(0, 0, 255));
  strip.setPixelColor(147, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(169, strip.Color(0, 250, 150));
  strip.setPixelColor(170, strip.Color(0, 250, 150));
  strip.setPixelColor(157, strip.Color(0, 0, 255));
  strip.setPixelColor(161, strip.Color(0, 0, 255));
  strip.setPixelColor(165, strip.Color(0, 250, 150));
  strip.setPixelColor(166, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(174, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(183, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(200, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(209, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(217, strip.Color(0, 250, 150));
  strip.setPixelColor(218, strip.Color(0, 250, 150));
  strip.setPixelColor(222, strip.Color(0, 0, 255));
  strip.setPixelColor(226, strip.Color(0, 0, 255));
  strip.setPixelColor(214, strip.Color(0, 250, 150));
  strip.setPixelColor(213, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(236, strip.Color(0, 0, 255));
  strip.setPixelColor(244, strip.Color(0, 0, 255));
  strip.setPixelColor(247, strip.Color(0, 0, 255));
  strip.setPixelColor(249, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
//  strip.setPixelColor(252, strip.Color(0, 0, 255));
//  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void angryEyesMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(145, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(155, strip.Color(0, 250, 150));
  strip.setPixelColor(154, strip.Color(0, 250, 150));
  strip.setPixelColor(158, strip.Color(0, 0, 255));
  strip.setPixelColor(162, strip.Color(0, 0, 255));
  strip.setPixelColor(165, strip.Color(0, 250, 150));
  strip.setPixelColor(164, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(172, strip.Color(0, 0, 255));
  strip.setPixelColor(180, strip.Color(0, 0, 255));
  strip.setPixelColor(183, strip.Color(0, 0, 255));
  strip.setPixelColor(184, strip.Color(0, 0, 255));
  strip.setPixelColor(185, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(198, strip.Color(0, 0, 255));
  strip.setPixelColor(199, strip.Color(0, 0, 255));
  strip.setPixelColor(200, strip.Color(0, 0, 255));
  strip.setPixelColor(203, strip.Color(0, 0, 255));
  strip.setPixelColor(211, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(221, strip.Color(0, 0, 255));
  strip.setPixelColor(225, strip.Color(0, 0, 255));
  strip.setPixelColor(219, strip.Color(0, 250, 150));
  strip.setPixelColor(218, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(229, strip.Color(0, 250, 150));
  strip.setPixelColor(228, strip.Color(0, 250, 150));
  strip.setPixelColor(238, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
//  strip.setPixelColor(252, strip.Color(0, 0, 255));
//  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void confusedEyesMo(){
  strip.clear();
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(138, strip.Color(0, 0, 255));
  strip.setPixelColor(140, strip.Color(0, 0, 255));
  strip.setPixelColor(146, strip.Color(0, 0, 255));
  strip.setPixelColor(150, strip.Color(0, 0, 255));
  strip.setPixelColor(153, strip.Color(0, 0, 255));
  strip.setPixelColor(157, strip.Color(0, 0, 255));
  strip.setPixelColor(162, strip.Color(0, 0, 255));
  strip.setPixelColor(166, strip.Color(0, 0, 255));
  //strip.setPixelColor(155, strip.Color(0, 250, 150));
  //strip.setPixelColor(156, strip.Color(0, 250, 150));
  strip.setPixelColor(169, strip.Color(0, 0, 255));
  strip.setPixelColor(173, strip.Color(0, 0, 255));
  strip.setPixelColor(171, strip.Color(0, 250, 150));
  strip.setPixelColor(164, strip.Color(0, 250, 150));
  strip.setPixelColor(178, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(185, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(203, strip.Color(0, 0, 255));
  strip.setPixelColor(218, strip.Color(0, 0, 255));
  strip.setPixelColor(219, strip.Color(0, 0, 255));
  strip.setPixelColor(220, strip.Color(0, 0, 255));
  strip.setPixelColor(221, strip.Color(0, 0, 255));
  strip.setPixelColor(225, strip.Color(0, 0, 255));
  strip.setPixelColor(230, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  //strip.setPixelColor(219, strip.Color(0, 250, 150));
  //strip.setPixelColor(220, strip.Color(0, 250, 150));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(236, strip.Color(0, 250, 150));
  strip.setPixelColor(235, strip.Color(0, 250, 150));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
//  strip.setPixelColor(232, strip.Color(0, 0, 255));
//  strip.setPixelColor(239, strip.Color(0, 0, 255));
//  strip.setPixelColor(241, strip.Color(0, 0, 255));
//  strip.setPixelColor(246, strip.Color(0, 0, 255));
//  strip.setPixelColor(250, strip.Color(0, 0, 255));
//  strip.setPixelColor(251, strip.Color(0, 0, 255));
//  strip.setPixelColor(252, strip.Color(0, 0, 255));
//  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void happyEyesMo(){
  strip.clear();
  strip.setPixelColor(140, strip.Color(0, 0, 255));
  strip.setPixelColor(146, strip.Color(0, 0, 255));
  strip.setPixelColor(158, strip.Color(0, 0, 255));
  strip.setPixelColor(161, strip.Color(0, 0, 255));
  strip.setPixelColor(173, strip.Color(0, 0, 255));
  strip.setPixelColor(179, strip.Color(0, 0, 255));
  strip.setPixelColor(204, strip.Color(0, 0, 255));
  strip.setPixelColor(210, strip.Color(0, 0, 255));
  strip.setPixelColor(222, strip.Color(0, 0, 255));
  strip.setPixelColor(225, strip.Color(0, 0, 255));
  strip.setPixelColor(237, strip.Color(0, 0, 255));
  strip.setPixelColor(243, strip.Color(0, 0, 150));
  strip.show();
}
void closeEyesMo(){
  strip.clear();
  strip.setPixelColor(139, strip.Color(0, 0, 255));
  strip.setPixelColor(149, strip.Color(0, 0, 255));
  strip.setPixelColor(153, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(170, strip.Color(0, 0, 255));
  strip.setPixelColor(180, strip.Color(0, 0, 255));
  strip.setPixelColor(203, strip.Color(0, 0, 255));
  strip.setPixelColor(213, strip.Color(0, 0, 255));
  strip.setPixelColor(217, strip.Color(0, 0, 255));
  strip.setPixelColor(230, strip.Color(0, 0, 255));
  strip.setPixelColor(234, strip.Color(0, 0, 255));
  strip.setPixelColor(244, strip.Color(0, 0, 150));
  strip.show();
}
void slightRightMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(171, strip.Color(0, 250, 150));
  strip.setPixelColor(172, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(163, strip.Color(0, 250, 150));
  strip.setPixelColor(164, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(235, strip.Color(0, 250, 150));
  strip.setPixelColor(236, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(227, strip.Color(0, 250, 150));
  strip.setPixelColor(228, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesDownRightMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(173, strip.Color(0, 250, 150));
  strip.setPixelColor(172, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(163, strip.Color(0, 250, 150));
  strip.setPixelColor(162, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(237, strip.Color(0, 250, 150));
  strip.setPixelColor(236, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(227, strip.Color(0, 250, 150));
  strip.setPixelColor(226, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesUpRightMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(171, strip.Color(0, 250, 150));
  strip.setPixelColor(170, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(165, strip.Color(0, 250, 150));
  strip.setPixelColor(164, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(235, strip.Color(0, 250, 150));
  strip.setPixelColor(234, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(229, strip.Color(0, 250, 150));
  strip.setPixelColor(228, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesRightMo(){
  strip.clear();
  strip.setPixelColor(141, strip.Color(0, 0, 255));
  strip.setPixelColor(140, strip.Color(0, 0, 255));
  strip.setPixelColor(139, strip.Color(0, 0, 255));
  strip.setPixelColor(138, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(171, strip.Color(0, 250, 150));
  strip.setPixelColor(172, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(179, strip.Color(0, 250, 150));
  strip.setPixelColor(180, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(205, strip.Color(0, 0, 255));
  strip.setPixelColor(204, strip.Color(0, 0, 255));
  strip.setPixelColor(203, strip.Color(0, 0, 255));
  strip.setPixelColor(202, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(235, strip.Color(0, 250, 150));
  strip.setPixelColor(236, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(243, strip.Color(0, 250, 150));
  strip.setPixelColor(244, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void slightLeftMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(155, strip.Color(0, 250, 150));
  strip.setPixelColor(156, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(147, strip.Color(0, 250, 150));
  strip.setPixelColor(148, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(219, strip.Color(0, 250, 150));
  strip.setPixelColor(220, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(211, strip.Color(0, 250, 150));
  strip.setPixelColor(212, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesDownLeftMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(157, strip.Color(0, 250, 150));
  strip.setPixelColor(156, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(147, strip.Color(0, 250, 150));
  strip.setPixelColor(146, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(221, strip.Color(0, 250, 150));
  strip.setPixelColor(220, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(211, strip.Color(0, 250, 150));
  strip.setPixelColor(210, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesUpLeftMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(155, strip.Color(0, 250, 150));
  strip.setPixelColor(154, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(149, strip.Color(0, 250, 150));
  strip.setPixelColor(148, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(219, strip.Color(0, 250, 150));
  strip.setPixelColor(218, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(213, strip.Color(0, 250, 150));
  strip.setPixelColor(212, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesLeftMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(139, strip.Color(0, 250, 150));
  strip.setPixelColor(140, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(147, strip.Color(0, 250, 150));
  strip.setPixelColor(148, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(181, strip.Color(0, 0, 255));
  strip.setPixelColor(180, strip.Color(0, 0, 255));
  strip.setPixelColor(179, strip.Color(0, 0, 255));
  strip.setPixelColor(178, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(203, strip.Color(0, 250, 150));
  strip.setPixelColor(204, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(211, strip.Color(0, 250, 150));
  strip.setPixelColor(212, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(245, strip.Color(0, 0, 255));
  strip.setPixelColor(244, strip.Color(0, 0, 255));
  strip.setPixelColor(243, strip.Color(0, 0, 255));
  strip.setPixelColor(242, strip.Color(0, 0, 255));
  strip.show();
}
void slightUpMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(157, strip.Color(0, 250, 150));
  strip.setPixelColor(156, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(163, strip.Color(0, 250, 150));
  strip.setPixelColor(162, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(221, strip.Color(0, 250, 150));
  strip.setPixelColor(220, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(227, strip.Color(0, 250, 150));
  strip.setPixelColor(226, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesUpMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(157, strip.Color(0, 250, 150));
  strip.setPixelColor(158, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(161, strip.Color(0, 250, 150));
  strip.setPixelColor(162, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(221, strip.Color(0, 250, 150));
  strip.setPixelColor(222, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(225, strip.Color(0, 250, 150));
  strip.setPixelColor(226, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void slightDownMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(155, strip.Color(0, 250, 150));
  strip.setPixelColor(154, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(165, strip.Color(0, 250, 150));
  strip.setPixelColor(164, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(219, strip.Color(0, 250, 150));
  strip.setPixelColor(218, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(229, strip.Color(0, 250, 150));
  strip.setPixelColor(228, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}
void eyesDownMo(){
  strip.clear();
  strip.setPixelColor(130, strip.Color(0, 0, 255));
  strip.setPixelColor(131, strip.Color(0, 0, 255));
  strip.setPixelColor(132, strip.Color(0, 0, 255));
  strip.setPixelColor(133, strip.Color(0, 0, 255));
  strip.setPixelColor(137, strip.Color(0, 0, 255));
  strip.setPixelColor(142, strip.Color(0, 0, 255));
  strip.setPixelColor(144, strip.Color(0, 0, 255));
  strip.setPixelColor(151, strip.Color(0, 0, 255));
  strip.setPixelColor(152, strip.Color(0, 0, 255));
  strip.setPixelColor(153, strip.Color(0, 250, 150));
  strip.setPixelColor(154, strip.Color(0, 250, 150));
  strip.setPixelColor(159, strip.Color(0, 0, 255));
  strip.setPixelColor(160, strip.Color(0, 0, 255));
  strip.setPixelColor(165, strip.Color(0, 250, 150));
  strip.setPixelColor(166, strip.Color(0, 250, 150));
  strip.setPixelColor(167, strip.Color(0, 0, 255));
  strip.setPixelColor(168, strip.Color(0, 0, 255));
  strip.setPixelColor(175, strip.Color(0, 0, 255));
  strip.setPixelColor(177, strip.Color(0, 0, 255));
  strip.setPixelColor(182, strip.Color(0, 0, 255));
  strip.setPixelColor(186, strip.Color(0, 0, 255));
  strip.setPixelColor(187, strip.Color(0, 0, 255));
  strip.setPixelColor(188, strip.Color(0, 0, 255));
  strip.setPixelColor(189, strip.Color(0, 0, 255));
  strip.setPixelColor(194, strip.Color(0, 0, 255));
  strip.setPixelColor(195, strip.Color(0, 0, 255));
  strip.setPixelColor(196, strip.Color(0, 0, 255));
  strip.setPixelColor(197, strip.Color(0, 0, 255));
  strip.setPixelColor(201, strip.Color(0, 0, 255));
  strip.setPixelColor(206, strip.Color(0, 0, 255));
  strip.setPixelColor(208, strip.Color(0, 0, 255));
  strip.setPixelColor(215, strip.Color(0, 0, 255));
  strip.setPixelColor(216, strip.Color(0, 0, 255));
  strip.setPixelColor(217, strip.Color(0, 250, 150));
  strip.setPixelColor(218, strip.Color(0, 250, 150));
  strip.setPixelColor(223, strip.Color(0, 0, 255));
  strip.setPixelColor(224, strip.Color(0, 0, 255));
  strip.setPixelColor(229, strip.Color(0, 250, 150));
  strip.setPixelColor(230, strip.Color(0, 250, 150));
  strip.setPixelColor(231, strip.Color(0, 0, 255));
  strip.setPixelColor(232, strip.Color(0, 0, 255));
  strip.setPixelColor(239, strip.Color(0, 0, 255));
  strip.setPixelColor(241, strip.Color(0, 0, 255));
  strip.setPixelColor(246, strip.Color(0, 0, 255));
  strip.setPixelColor(250, strip.Color(0, 0, 255));
  strip.setPixelColor(251, strip.Color(0, 0, 255));
  strip.setPixelColor(252, strip.Color(0, 0, 255));
  strip.setPixelColor(253, strip.Color(0, 0, 255));
  strip.show();
}

// -------------------------------- Blink Functions -------------------------------------
void blinkEyesMo(int current){  // Want to use current eye state.
  closeEyesMo();
  delay(150);
  if(current == 1){
    baseEyesMo();
    Serial.println(current);
  }
  else if(current == 2){
    slightUpMo();
    Serial.println(current);
  }
  else if(current == 3){
    eyesUpMo();
    Serial.println(current);
  }
  else if(current == 4){
    slightDownMo();
    Serial.println(current);
  }
  else if(current == 5){
    eyesDownMo();
    Serial.println(current);
  }
  else if(current == 6){
    slightRightMo();
    Serial.println(current);
  }
  else if(current == 7){
    eyesRightMo();
    Serial.println(current);
  }
  else if(current == 8){
    Serial.println(current);
    slightLeftMo();
  }
  else if(current == 9){
    eyesLeftMo();
    Serial.println(current);
  }
  else if(current == 0){
    closeEyesMo();
    Serial.println(current);
  }
  else if(current == 10){
    happyEyesMo();
    Serial.println(current);
  }
  else if(current == 11){
    sadEyesMo();
    Serial.println(current);
  }
  else if(current == 12){
    angryEyesMo();
    Serial.println(current);
  }
  else if(current == 13){
    confusedEyesMo();
    Serial.println(current);
  }
  else if(current == 14){
    eyesDownLeftMo();
    Serial.println(current);
  }
  else if(current == 15){
    eyesDownRightMo();
    Serial.println(current);
  }
  else if(current == 16){
    eyesUpLeftMo();
    Serial.println(current);
  }
  else if(current == 17){
    eyesUpRightMo();
    Serial.println(current);
  }
}
void blinkEyesMa(int current){  // Want to use current eye state.
  closeEyesMa();
  delay(150);
  if(current == 1){
    baseEyesMa();
    Serial.println(current);
  }
  else if(current == 2){
    slightUpMa();
    Serial.println(current);
  }
  else if(current == 3){
    eyesUpMa();
    Serial.println(current);
  }
  else if(current == 4){
    slightDownMa();
    Serial.println(current);
  }
  else if(current == 5){
    eyesDownMa();
    Serial.println(current);
  }
  else if(current == 6){
    slightRightMa();
    Serial.println(current);
  }
  else if(current == 7){
    eyesRightMa();
    Serial.println(current);
  }
  else if(current == 8){
    slightLeftMa();
    Serial.println(current);
  }
  else if(current == 9){
    eyesLeftMa();
    Serial.println(current);
  }
  else if(current == 0){
    closeEyesMa();
    Serial.println(current);
  }
  else if(current == 10){
    happyEyesMa();
    Serial.println(current);
  }
  else if(current == 11){
    sadEyesMa();
    Serial.println(current);
  }
  else if(current == 12){
    angryEyesMa();
    Serial.println(current);
  }
  else if(current == 13){
    confusedEyesMa();
    Serial.println(current);
  }
  else if(current == 14){
    eyesDownLeftMa();
    Serial.println(current);
  }
  else if(current == 15){
    eyesDownRightMa();
    Serial.println(current);
  }
  else if(current == 16){
    eyesUpLeftMa();
    Serial.println(current);
  }
  else if(current == 17){
    eyesUpRightMa();
    Serial.println(current);
  }
}
// -------------------------------------- Transition Functions ----------------------------------------------
void mobToman(){
  Serial.println("mob to man");
  delay(500);
  transitionEyes();
  delay(250);
  baseEyesMa();
  
}
void manTomob(){
  Serial.println("man to mob");
  delay(500);
  transitionEyes();
  delay(250);
  baseEyesMo();
}
