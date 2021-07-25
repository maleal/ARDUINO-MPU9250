/* 
  Aero-Quad Carballo-Rinke - May 2019 -
  based on MPU9250 - Example Code
  by Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
  Modified by Brent Wilkins July 19, 2016

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.


  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND

  AHRS "attitude and heading reference system"
  MPU-9250: Has an accelerometer with a programmable full-scale range of �2g, �4g, �8g and �16g and integrated 16-bit ADCs
  and gyroscopes with a user-programmable full-scale range of �250, �500, �1,000 and �2,000�/sec and integrated 16-bit ADCs
*/

// Set to false for basic data read
#define     SAMPLES_TO_OPENGL
#include <EEPROM.h>
//#define     MONITOR_SETTING_UP_MPU9250
//#define     MONITOR_SERIE_DEBUG_ALL_SENSOR_DATA
//#define     MONITOR_SHOW_ATTITUDE
//#define     MONITOR_SELFTEST_GYRO_ACEL
//#define     MONITOR_MAGTER_AK8963_CALIBRATION


#include    "MPU9250.h"
#include    "quaternionFilters.h"
#include    "DataStorage.h"



int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

MPU9250 myIMU;

//To send Heading to my OpenGl app whit the following format
//  "043Stx|yaw= 80.56|pitch= 18.43|roll= 10.29|delta= 0.03|End"
union uTrama{
        char zLen[3];
        char zMsg[73];
};

struct zTemp {
    int iLen;
    char zYaw[10]; char zPitch[10]; char zRoll[10]; char zDelT[10];
    char zAll[70];
    //uTrama Msg;
};

zTemp zBuffer;
uTrama MsgToOpGL;

void setup()
{
      Wire.begin();
      // TWBR = 12;  // 400 kbit/sec I2C speed
      Serial.begin(115200); //28800 38400 57600 115200
    
      // Set up the interrupt pin, its set as active high, push-pull
      pinMode(intPin, INPUT);
      digitalWrite(intPin, LOW);
      pinMode(myLed, OUTPUT);
      digitalWrite(myLed, HIGH);
  
  
      // Read the WHO_AM_I register, this is a good test of communication
      byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

      if (c == 0x71) // WHO_AM_I should always be 0x71
      {
      
          myIMU.MPU9250SelfTest();

          // Calibrate gyro and accelerometers, load biases in bias registers
          myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    
    
          // Accelerometer, Gyroscope and Temperature sensors: Initialize devices for active mode read
          myIMU.initMPU9250();
    
          // Magnetometer: Read the WHO_AM_I register (communicatios test)
          byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
          if (d != 0x48)
          {
              Serial.println("MPU9250 initialized for active data mode....");
              Serial.println(F("Communication to magnetometer failed, abort!"));
              Serial.flush();
              abort();
          }

          // Magnetometer: first, We have to Get magnetometer calibration from AK8963 ROM (datos de fafrica)
          myIMU.initAK8963(myIMU.factoryMagCalibration);
          #ifdef MONITOR_SETTING_UP_MPU9250
          {
                  // Initialize device for active mode read of magnetometer
                  Serial.println("AK8963 Magnetometer was initialized for active data mode....");
                  Serial.println("Calibration values: ");
                  Serial.print("X-Axis factory sensitivity adjustment value ");
                  Serial.println(myIMU.factoryMagCalibration[0], 2);
                  Serial.print("Y-Axis factory sensitivity adjustment value ");
                  Serial.println(myIMU.factoryMagCalibration[1], 2);
                  Serial.print("Z-Axis factory sensitivity adjustment value ");
                  Serial.println(myIMU.factoryMagCalibration[2], 2);
          }
          #endif//MONITOR_SETTING_UP_MPU9250


          // Get sensor resolutions, only need to do this once
          myIMU.getAres();
          myIMU.getGres();
          myIMU.getMres();

          /*  Importante:
           *  Anotar los valores mostrados por los printf de mas abajo
           *  y transformarlos en constantes para solo tener que calibrar una vez
           *  Para continuar con la ejecucion del programa escribir ready
           *  en el Monitor Serie de Arduino
          */
          // The next call delays for 4 seconds, and then records about 15 seconds of
          // data to calculate bias and scale.
          //myIMU.magCalMPU9250(myIMU._magBias, myIMU.magScale);
		  myIMU.magCalMPU9250();
		  
		  #ifdef MONITOR_MAGTER_AK8963_CALIBRATION
          delay(2000); // Add delay to see results before serial spew of data
              Serial.println("AK8963 mag biases x y z(mG)");
              Serial.println(myIMU.magBias[0]);
              Serial.println(myIMU.magBias[1]);
              Serial.println(myIMU.magBias[2]);
          
              Serial.println("AK8963 mag scale (mG)");
              Serial.println(myIMU.magScale[0]);
              Serial.println(myIMU.magScale[1]);
              Serial.println(myIMU.magScale[2]); 
          while (!Serial.find("ready"));
          Serial.print("ready");
          #endif//MONITOR_MAGTER_AK8963_CALIBRATION

    
    
  } // if (c == 0x71)
  else
  {
          Serial.print("Could not connect to MPU9250: 0x");
          Serial.println(c, HEX);
      
          // Communication failed, stop here
          Serial.println(F("Communication failed, abort!"));
          Serial.flush();
          abort();
  }
}



void loop()
{
        // If intPin goes high, all data registers have new data
        // On interrupt, check if data ready interrupt
        if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
        {
              myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
          
              // Now we'll calculate the accleration value into actual g's
              // This depends on scale being set
              myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
              myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
              myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
          
              myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
          
              // Calculate the gyro value into actual degrees per second
              // This depends on scale being set
              myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
              myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
              myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
          
              myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
          
              // Calculate the magnetometer values in milliGauss
              // Include factory calibration per data sheet and user environmental
              // corrections
              // Get actual magnetometer value, this depends on scale being set
              myIMU.mx = ((float)myIMU.magCount[0] * myIMU.mRes
                         * myIMU.factoryMagCalibration[0]) - myIMU.GetMagBias()[0];
			        myIMU.mx *= myIMU.GetMagScales()[0];

              myIMU.my = ((float)myIMU.magCount[1] * myIMU.mRes
                         * myIMU.factoryMagCalibration[1]) - myIMU.GetMagBias()[1];
			        myIMU.my *= myIMU.GetMagScales()[1];

              myIMU.mz = ((float)myIMU.magCount[2] * myIMU.mRes
                         * myIMU.factoryMagCalibration[2]) - myIMU.GetMagBias()[2];
			        myIMU.mz *= myIMU.GetMagScales()[2];

              // Must be called before updating quaternions!
              myIMU.updateTime();
            
              // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
              // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
              // (+ up) of accelerometer and gyro! We have to make some allowance for this
              // orientationmismatch in feeding the output to the quaternion filter. For the
              // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
              // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
              // modified to allow any convenient orientation convention. This is ok by
              // aircraft orientation standards! Pass gyro rate as rad/s
              MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                                     myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                                     myIMU.mx, myIMU.mz, myIMU.deltat);
        
              // Define output variables from updated quaternion---these are Tait-Bryan
              // angles, commonly used in aircraft orientation. In this coordinate system,
              // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
              // x-axis and Earth magnetic North (or true North if corrected for local
              // declination, looking down on the sensor positive yaw is ..
              // Pitch is angle between sensor x-axis and Earth ground plane, toward the
              // Earth is positive, up toward the sky is negative. Roll is angle between
              // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
              // arise from the definition of the homogeneous rotation matrix constructed
              // from quaternions. Tait-Bryan angles as well as Euler angles are
              // non-commutative; that is, the get the correct orientation the rotations
              // must be applied in the correct order which for this configuration is yaw,
              // pitch, and then roll.
              // For more see
              // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
              // which has additional links.
              myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                                  * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                                  * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                                  * *(getQ() + 3));
              myIMU.pitch = asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                                          * *(getQ() + 2)));
              myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                                  * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                                  * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                                  * *(getQ() + 3));
              
			  myIMU.pitch *= RAD_TO_DEG;
              myIMU.yaw   *= RAD_TO_DEG;
        
              // Declination of Bs As 2017
              // - http://www.ngdc.noaa.gov/geomag-web/#declination
              myIMU.yaw  -= 8.36; //Buenos aires 2017
              myIMU.roll *= RAD_TO_DEG;
        }// if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  
        
        
        
        // Serial print and/or display at 0.5 s rate independent of data rates
        myIMU.dspDelt_t = millis() - myIMU.count;
  
        //
        if (myIMU.dspDelt_t > 500)
        {
  
            #ifdef MONITOR_SERIE_DEBUG_ALL_SENSOR_DATA
                      Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
                      Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
                      Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
                      Serial.println(" mG-force");
              
                      Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
                      Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
                      Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
                      Serial.println(" deg/s");
              
                      Serial.print("mx = ");  Serial.print((int)myIMU.mx);
                      Serial.print(" my = "); Serial.print((int)myIMU.my);
                      Serial.print(" mz = "); Serial.print((int)myIMU.mz);
                      Serial.println(" mGauss");
              
                      Serial.print("q0 = ");  Serial.print(*getQ());
                      Serial.print(" qx = "); Serial.print(*(getQ() + 1));
                      Serial.print(" qy = "); Serial.print(*(getQ() + 2));
                      Serial.print(" qz = "); Serial.println(*(getQ() + 3));
              
                      myIMU.tempCount = myIMU.readTempData();  // Read the adc values
                      
                      // Temperature in degrees Centigrade
                      myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
                      // Print temperature in degrees Centigrade
                      Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
                      Serial.println(" degrees C");
            #endif//MONITOR_SERIE_DEBUG_ALL_SENSOR_DATA
  
            // Define output variables from updated quaternion---these are Tait-Bryan
            // angles, commonly used in aircraft orientation. In this coordinate system,
            // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
            // x-axis and Earth magnetic North (or true North if corrected for local
            // declination, looking down on the sensor positive yaw is ..
            // Pitch is angle between sensor x-axis and Earth ground plane, toward the
            // Earth is positive, up toward the sky is negative. Roll is angle between
            // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
            // arise from the definition of the homogeneous rotation matrix constructed
            // from quaternions. Tait-Bryan angles as well as Euler angles are
            // non-commutative; that is, the get the correct orientation the rotations
            // must be applied in the correct order which for this configuration is yaw,
            // pitch, and then roll.
            // For more see
            // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            // which has additional links.
            /*
            myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                                        * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                                * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                                * *(getQ() + 3));
            myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                                        * *(getQ() + 2)));
            myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                                        * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                                * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                                * *(getQ() + 3));
            myIMU.pitch *= RAD_TO_DEG;
            myIMU.yaw   *= RAD_TO_DEG;
      
            // Declination of Bs As 2017
            // - http://www.ngdc.noaa.gov/geomag-web/#declination
            myIMU.yaw  -= 8.36; //Buenos aires 2017
            myIMU.roll *= RAD_TO_DEG;
            */
			#ifdef MONITOR_SHOW_ATTITUDE
					Serial.print("Yaw, Pitch, Roll: ");
					Serial.print(myIMU.yaw, 2);
					Serial.print(", ");
					Serial.print(myIMU.pitch, 2);
					Serial.print(", ");
					Serial.println(myIMU.roll, 2);
    
					Serial.print("rate = ");
					Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
					Serial.println(" Hz");
			#endif//MONITOR_SHOW_ATTITUDE
  
			myIMU.count = millis();
			myIMU.sumCount = 0;
			myIMU.sum = 0;
      
      }// if (myIMU.dspDelt_t > 500)
  
      
            
  #ifdef SAMPLES_TO_OPENGL  //Send Heading to OpenGl App !!!!!!!
                //043STx|yaw= 80.56|pitch= 18.43|roll= 10.29|End

        dtostrf(myIMU.yaw,    7, 2, zBuffer.zYaw);
        dtostrf(myIMU.pitch,  7, 2, zBuffer.zPitch);
        dtostrf(myIMU.roll,   7, 2, zBuffer.zRoll);
        dtostrf(myIMU.deltat, 7, 2, zBuffer.zDelT);
                
                
        //sprintf(zbuffer, "Stx|yaw=%s|pitch=%s|roll=%s|delta=%s|End", MsgToOpGL.zYaw, MsgToOpGL.zPitch, MsgToOpGL.zPitch, MsgToOpGL.zDelT);
        sprintf(zBuffer.zAll, "Stx|yaw=%s|pitch=%s|roll=%s|delta=%s|End", zBuffer.zYaw, zBuffer.zPitch, zBuffer.zRoll, zBuffer.zDelT);
                
        zBuffer.iLen = strlen(zBuffer.zAll);
        zBuffer.iLen += 3;
      
        //strcpy(uTrama.strLen, zbuffer);
                
        sprintf(MsgToOpGL.zLen, "%.03d", zBuffer.iLen);
        strcat(MsgToOpGL.zLen, zBuffer.zAll);
                
        //memcpy(MsgToOpGL.Msg.strLen, zbuffer, 3); 
                
        //
        Serial.print(MsgToOpGL.zMsg);
        //Serial.println(MsgToOpGL.zMsg);         
  #endif//SAMPLES_TO_OPENGL        

}
