/*****************************************************************************/
//	Function:    Get the accelemeter of the x/y/z axis.
//  Hardware:    Grove - 3-Axis Digital Accelerometer(±1.5g)
//	Arduino IDE: Arduino-1.0
//	Author:	 Frankie.Chu
//	Date: 	 Jan 10,2013
//	Version: v0.9b
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include <Wire.h>
#include <CurieBLE.h>
#include "MMA7660.h"
#include "rgb_lcd.h"

MMA7660 accelemeter;
rgb_lcd lcd;
//LCD Initialization
const int colorR = 117;
const int colorG = 21;
const int colorB = 32;

const float t = .05;

float disp, vel = 0;
float dispX, dispY = 0;
int xMax, yMax, xMin, yMin = 0;

float axp, ayp, azp, dacc, pdisp = 0;
float dx, dy, dz, accp, ddisp = 0;

//Bluetooth Initialization
BLEService userService("AAAA");       // BLE User Data Service
BLECharacteristic envCharacteristic("AAAA", BLERead | BLENotify, 2);
BLEUnsignedCharCharacteristic lock("AAAB", BLERead | BLEWrite);
unsigned char envVars[] = {0, 0};

void setup()
{
  accelemeter.init();

  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, 255);

  // Print a message to the LCD.
  lcd.print("Ready to Pair");


  Serial.begin(9600);
  BLE.begin();

  // set advertised local name and service UUID:
  BLE.setLocalName("envMoni");
  BLE.setAdvertisedService(userService);
  // add service
  BLE.addService(userService);


  // add the characteristics to the service
  userService.addCharacteristic(envCharacteristic);
  // userService.addCharacteristic(lock);
  // start advertising
  BLE.advertise();

  // Wait
  delay(2000);
}
void loop()
{
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    lcd.clear();
    lcd.print("Connected: ");
    lcd.setCursor(0, 1);
    // print the central's MAC address:
    lcd.print(central.address());
    lcd.setRGB(255, colorG, colorB);
    delay(2000);
    lcd.clear();
    lcd.setRGB(255, 255, 255);

    while (central.connected()) {
      float aggvel = 0;
      pdisp = disp;
      for (int i = 0; i < (1 / t); i++) {
        int8_t x;
        int8_t y;
        int8_t z;
        float ax, ay, az;

        accelemeter.getXYZ(&x, &y, &z);
        if (x > xMax) {
          xMax = x;
        }
        else if (x < xMin) {
          xMin = x;
        }

        if (y > yMax) {
          yMax = y;
        }
        else if (y < yMin) {
          yMin = y;
        }


        accelemeter.getAcceleration(&ax, &ay, &az);
        float acc = absAcc(dx, dy, dz);
        dx = axp - ax;
        dy = ayp - ay;
        dz = azp - az;
        dacc = abs(accp) - abs(acc);

        vel += dacc * t;

        accp = acc;
        axp = ax;
        ayp = ay;
        azp = az;

        aggvel += vel;
        delay(t * 1000);
      }

      dispX = xMax - xMin;
      dispY = yMax - yMin;

      disp = disVector(dispX, dispY) * 1000 ;
      ddisp = disp - pdisp;
      Serial.println(ddisp);
      Serial.println(floor(abs(aggvel / (1 / t)) * 100));

      envVars[0] = ddisp;
      envVars[1] = floor(abs(aggvel / (1 / t)) * 100);
      
      envCharacteristic.setValue(envVars, 2);

      xMax, xMin, yMax, yMin = 0;
    }
    // when the central disconnects, print it out:
    lcd.setRGB(255, colorG, colorB);
    lcd.print(("Disconnected from central: "));
    lcd.setCursor(0,1);
    lcd.print(central.address());
    delay(1000);

    lcd.setRGB(colorR, colorG, 255);
    lcd.clear();
    lcd.print("Ready to Pair");
  }
}


float absAcc(float accX, float accY, float accZ) {
  return sqrt(pow(accX, 2) + pow(accY, 2));
}

float disVector(int x, int y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}
