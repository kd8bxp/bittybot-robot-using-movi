/* Copyright 2015 - LeRoy F. Miller, kd8bxp
 *  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses>

    Portitions of this code inspired by or based on
   https://arduino-info.wikispaces.com/DHT11-Humidity-TempSensor
   and
   http://henrysbench.capnfatz.com/henrys-bench/arduino-voltage-measurements/arduino-25v-voltage-sensor-module-user-manual/
   As well as the MOVI examples included in the MOVI library.
 */

#include "MOVIShield.h"
#include "BittyBot2.h"
#include <NewPing.h>
#include <TimedAction.h>
#include <dht.h>

#ifdef ARDUINO_ARCH_AVR 
#include <SoftwareSerial.h> // This is nice and flexible but only supported on AVR architecture, other boards need to use Serial1 
#include <avr/pgmspace.h>
#endif

MOVI recognizer(true);

int lowspeed = 75; //low speed set in PWM
int leftspeed; //low speed set
int rightspeed; //low speed set
int speedpercent = 100; //set speed 
int goRobotGo = 0; //flag to indicate if robot should be moving.
/*added for this sketch goRobotGo also tells type of movement
 *0 - STOP
 *1 - forward
 *2 - backward
 *3 - left
 *4 - right
  */
int objectFlag = 0; //if object is in path flag set to 1

BittyBot bot(44,46,36,38,40,42); //Left Enable, Right Enable, Pin1 for Left, Pin2 for Left, Pin 1 for Right, Pin 2 for Right

#define TRIGGER_PIN 22 //Arduino Pin Tied to Trigger Pin on the ultrasonic sensor
#define ECHO_PIN 24 //Arduino Pin Tied to echo pin on the ultrasonic sensor
#define MAX_DISTANCE 200 //Maximum distance we want to ping for (in centimeters).  Maximum sensor distance is rated at 400 - 500cm
unsigned int sI = 0;
unsigned int uS = 0;

//variables used for voltage sensor
int voltInput = A8;
float vout = 0.0;
float vin = 0.0;
float R1 = 30000.0; //  
float R2 = 7500.0; // 
int value = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void distance();
void voltage();
void readTemperature();
void adjustThreshold();
TimedAction pingAction = TimedAction(50, distance);
TimedAction voltAction = TimedAction(100, voltage);
TimedAction temperatureAction = TimedAction(150, readTemperature);
TimedAction thresholdAction = TimedAction(100, adjustThreshold);

//DHT11 variables and setup
dht DHT;
#define DHT11_PIN A9
#define fahrenheit 1 //set to zero for Celsius
int humidity;
int temperature;

//Movi Threshold variables
#define defaultThreshold 3 //73
#define switchS1 47 //minus 1
#define switchS2 45 //plus 1
#define switchS3 43 //minus 10
#define switchS4 41 //plus 10
int threshold = defaultThreshold;

void setup () {
	Serial.begin(9600);
 pinMode(switchS1, INPUT);
 pinMode(switchS2, INPUT);
 pinMode(switchS3, INPUT);
 pinMode(switchS4, INPUT);
	bot.begin();
  recognizer.init();
  recognizer.callSign("bittybot");
	recognizer.addSentence(F("stop"));
  recognizer.addSentence(F("forward"));
  recognizer.addSentence(F("go")); //may need to change sounds too much like SLOW
  recognizer.addSentence(F("back"));
  recognizer.addSentence(F("reverse"));
  recognizer.addSentence(F("left"));
  recognizer.addSentence(F("right"));
  recognizer.addSentence(F("status"));
  recognizer.addSentence(F("slow")); //may need to change sounds too much like GO
  recognizer.addSentence(F("medium"));
  recognizer.addSentence(F("fast"));
	recognizer.train();
 recognizer.setThreshold(threshold);
  setSpeed(speedpercent); //set speed 
  pinMode(voltInput, INPUT);
  recognizer.say("BittyBot Ready. ");
}

void loop() {

 thresholdAction.check();
 voltAction.check();
 pingAction.check(); 
 temperatureAction.check();
 movement();

//check for speech and speech logic
 int result=recognizer.poll();
  //if (result <= 0 || result >= 12) {recognizer.say("Command not recognized");}
  if (result == 1) {
      goRobotGo = 0;
      movement();
      recognizer.say("All Stop.");
      }
  if (result == 2 || result == 3) {
      bot.stop();
      goRobotGo = 1;
      movement();
      recognizer.say("Go Robot Go.");
      }
    
  if (result == 4 || result == 5) {
      bot.stop();
      goRobotGo = 2;
      movement();
      recognizer.say("One step forward, big step back.");
    }

  if (result == 6) {
      bot.stop();
      goRobotGo = 3;
      movement();
      recognizer.say("Only left handed people are in there right minds.");
    }

  if (result == 7) {
      bot.stop();
      goRobotGo = 4;
      movement();
      recognizer.say("Right!");
    }
  if (result == 8) {
    speakStatus();  
  }
  if (result == 9) {
    speedpercent = 0;
    setSpeed(speedpercent);    //set speed to lowest speed. slow
    recognizer.say("Speed set to slow.");  
  }
  if (result == 10) {
    speedpercent = 50;
    setSpeed(speedpercent); //set speed medium speed (50%)
    recognizer.say("Speed set to medium.");
  }
  if (result == 11) {
    speedpercent = 100;
    setSpeed(speedpercent); //set speed fastest speed (100%)
    recognizer.say("Speed set to fast.");
  }
  
	/*if (bot.IsRunning()) {

    thresholdAction.check();
		pingAction.check();	
		voltAction.check();
   temperatureAction.check();
		bot.update();
			}
	*/
  
}

int setSpeed(int speed) {
  leftspeed = map(speed, 0, 100, lowspeed, 255);
  rightspeed = map(speed, 0, 100, lowspeed, 255);
  bot.Speed(leftspeed,rightspeed);
}

void distance() {

  uS = sonar.ping();
  sI = (uS / US_ROUNDTRIP_IN);

  if (goRobotGo == 1 && bot.IsRunning()) {
  if (sI >= 30) {
    objectFlag = 0;
    setSpeed(speedpercent);
    bot.forward(500);
      }
  
  if (sI > 20 && sI <= 29) {
    objectFlag = 0;
    setSpeed(75); //75 percent slow down a little
    bot.forward(500);
      }
  
  if (sI > 5 && sI <= 19) {
    objectFlag = 0;
    setSpeed(50); //50 percent slow down a little more
    bot.forward(500);
      }
  
  if (sI <=5) {
    bot.stop();
    goRobotGo = 0;
    if (objectFlag == 0) {
    recognizer.say("Remove Object, Object detected in path");
    objectFlag = 1;
    }
    setSpeed(speedpercent); //reset Speed back to current percentage
    }
  }
}

void voltage() {
  value = analogRead(voltInput);
   vout = (value * 5.0) / 1024.0; // see text
   vin = vout / (R2/(R1+R2)); 
   Serial.print("Voltages");
   Serial.println(vin);
  //vin = 9; //remove this is for testing movi only
   if (vin < 7.0) {
     goRobotGo = 0;
     bot.stop();
     recognizer.say("Battery voltage critial low");
     recognizer.say("Replace battery and restart.");
     while(1) {}
   }
}

void readTemperature() {
  int chk = DHT.read11(DHT11_PIN);
  humidity = DHT.humidity;
  temperature=DHT.temperature; //Celius reading
  Serial.print(temperature);
  Serial.println("C");
  if (fahrenheit) {temperature = ((temperature * 9)/5)+32;
  Serial.print(temperature); Serial.println("F");}
}

void movement() {
  /*setSpeed(speedpercent);
  switch(goRobotGo) {
      case 0:
        bot.stop();
        break;
      case 1:
        bot.forward(100);
        break;
      case 2:
        bot.back(100);
        break;
      case 3:
        bot.left(100);
        break;
      case 4:
        bot.right(100);
        break;
      default:
        bot.stop(); //fail to stop
        break;       
  }
  */
}

void speakStatus() {
  //speak various sensor and status readings
  recognizer.say("Bittybot is currently ");
  if (goRobotGo > 0) { 
    recognizer.say("Moving ");
    if (goRobotGo == 1) {
      recognizer.say("forward.");
    }
    if (goRobotGo == 2) {
      recognizer.say("backward.");
    }
    if (goRobotGo == 3) {
      recognizer.say("turning left");
    }
    if (goRobotGo == 4) {
      recognizer.say("turning right");
    }
  } else {
    recognizer.say("Stopped.");
  }
  thresholdAction.check();
  pingAction.check();  
    voltAction.check();
    temperatureAction.check();
    bot.update();
  recognizer.say("My motors are set to ");
  if (speedpercent == 0) {recognizer.say("slow speed.");}
  if (speedpercent == 50) {recognizer.say("medium speed.");}
  if (speedpercent == 100) {recognizer.say("fast speed.");}
  thresholdAction.check();
  pingAction.check();  
    voltAction.check();
    temperatureAction.check();
    bot.update();
  recognizer.say("Current distance to object ");
  recognizer.say(String(sI));
  recognizer.say("inches.");
  pingAction.check();
  thresholdAction.check();  
    voltAction.check();
    temperatureAction.check();
    bot.update();
  recognizer.say("Current temperature is");
  recognizer.say(String(temperature));
    if (fahrenheit) {
  recognizer.say("fahrenheit."); } else {
    recognizer.say("celsius.");  }
  pingAction.check();
  thresholdAction.check();  
    voltAction.check();
    temperatureAction.check();
    bot.update();
    recognizer.say("Current humidity is ");
    recognizer.say(String(humidity));
    recognizer.say("percent.");
    thresholdAction.check();
    pingAction.check();  
    voltAction.check();
    temperatureAction.check();
    bot.update();
  recognizer.say("Battery voltage is");
  recognizer.say(String(vin));
  recognizer.say("volts.");
  thresholdAction.check();
  pingAction.check();  
    voltAction.check();
    temperatureAction.check();
    bot.update();
}

void adjustThreshold() {

if (digitalRead(switchS1) == LOW) {
  threshold = threshold - 1;
    if (threshold < defaultThreshold) {
      threshold = defaultThreshold;
      }
      recognizer.say("Threshold minus one, threshold set to");
      recognizer.say(String(threshold));
  }
if (digitalRead(switchS2) == LOW) {
  threshold = threshold + 1;
    if (threshold > 95) {
      threshold = 95;
    }
  recognizer.say("Threshold plus one, threshold set to");
      recognizer.say(String(threshold));
  }
if (digitalRead(switchS3) == LOW) {
  threshold = threshold - 10;
    if (threshold < defaultThreshold) {
      threshold = defaultThreshold;
    }
    recognizer.say("Threshold minus ten, threshold set to");
      recognizer.say(String(threshold));
  }
if (digitalRead(switchS4) == LOW) {
  threshold = threshold + 10;
    if (threshold > 95) {
      threshold = 95;
    }
    recognizer.say("Threshold plus ten, threshold set to");
      recognizer.say(String(threshold));
  }
  
}


