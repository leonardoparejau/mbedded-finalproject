/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "TCS3472_I2C.h"
#include "MMA8451Q.h"
#include "MBed_Adafruit_GPS.h"
#include <math.h>

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
 
/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
 
/** Si7013 Read Temperature Command */
#define SI7013_READ_TEMP_POST   0xE0 /* Read previous T data from RH measurement
                                        command*/
#define SI7013_READ_TEMP        0xE3 /* Stand-alone read temperature command */

/** Si7013 Read RH Command */
#define SI7013_READ_RH          0xE5 /* Perform RH (and T) measurement. */
/** Si7013 Read ID */
#define SI7013_READ_ID1_1       0xFA
#define SI7013_READ_ID1_2       0x0F
#define SI7013_READ_ID2_1       0xFc
#define SI7013_READ_ID2_2       0xc9
/** Si7013 Read Firmware Revision */
#define SI7013_READ_FWREV_1     0x84
#define SI7013_READ_FWREV_2     0xB8
 
/** I2C device address for Si7013 */
#define SI7013_ADDR      0x82
/** I2C device address for Si7021 */
#define SI7021_ADDR      0x80
 
/** Device ID value for Si7013 */
#define SI7013_DEVICE_ID 0x0D
/** Device ID value for Si7020 */
#define SI7020_DEVICE_ID 0x14
/** Device ID value for Si7021 */
#define SI7021_DEVICE_ID 0x15
 
/** @endcond */

/** Soil Moisture and light sensor defines **/
#include "DigitalOut.h"
#include "InterruptIn.h"
#include "PinNames.h"
#include "mbed.h"
#include <cstdint>
#include <string>

/** RGB Defines **/
#define ENABLE           0x00
#define TCS34725_CDATAL (0x14) /**< Clear channel data low byte */
#define TCS34725_CDATAH (0x15) /**< Clear channel data high byte */
#define TCS34725_RDATAL (0x16) /**< Red channel data low byte */
#define TCS34725_RDATAH (0x17) /**< Red channel data high byte */
#define TCS34725_GDATAL (0x18) /**< Green channel data low byte */
#define TCS34725_GDATAH (0x19) /**< Green channel data high byte */
#define TCS34725_BDATAL (0x1A) /**< Blue channel data low byte */
#define TCS34725_BDATAH (0x1B) /**< Blue channel data high byte */

/******************** Variables I/O *****************************/

// Temperature_Humidity
I2C sensorI2C(PB_9,PB_8);

// Soil Moisture
AnalogIn soil_moisture_sensor(PA_0);

// Light Sensor
AnalogIn light_sensor(PA_4);

// RGB Sensor
TCS3472_I2C rgb_sensor(PB_9,PB_8);

// Accelerometer
MMA8451Q accel_sensor(PB_9,PB_8,0x1c<<1);

// GPS
using namespace std::chrono;
UnbufferedSerial * gps_Serial;

//USER_BUTTON
InterruptIn button(PB_2); 

//Mode Leds
DigitalOut led1(LED1); //Test Mode
DigitalOut led2(LED2); //Normal Mode
DigitalOut led3(LED3); //Advanced Mode

//RGB Led
DigitalOut redLed(PH_0);
DigitalOut greenLed(PH_1);
DigitalOut blueLed(PB_13);

/******************** Variables Globales *****************************/

// Temperature_Humidity
uint8_t  _address = 0;
uint8_t  _rx_buf[8] = {0};
uint8_t  _tx_buf[2] = {0};
 
uint32_t _rhData = 0;
float _rhDataf;
float retRH[3] = {0,1000000,0};
float _rhDatafMax = 50; 
float _rhDatafMin = 40; 

int32_t  _tData = 0;
float _tDataf;
float retTemp[3] = {0,1000000,0};
float _tDatafMax = 25; 
float _tDatafMin = 18; 

// Soil Moisture
float soilMoistureValue;
float retMoist[3] = {0,1000000,0};
float soilMoistureValueMax = 50;
float soilMoistureValueMin = 30;

//Light Sensor
float lightValue;
float retLight[3] = {0,1000000,0};
float lightValueMax = 40;  
float lightValueMin = 8;  

// RGB Sensor
int rgb_readings[4];
string dominant;
int red;
int green;
int blue;
int redCount = 0;
int greenCount = 0;
int blueCount = 0;
string dominantHour;
string dominantWanted = "Green";

// Accelerometer
float x,y,z;
float retX[3]={0,1000000,0};
float retY[3]={0,1000000,0};
float retZ[3]={0,1000000,0};
float accelMin = 0.75;
float accelMax = 1.25;
float accel;

//initial mode Test
int mode_counter = 0;

//mode Normal
int counter1hour = 0;

Timer alarm_Timer; 
int refresh_alarmTime=250; 
bool flag = false;

/******************** Methods ********************************************/

void button_isr(){
	if (mode_counter==2){
		mode_counter=0;
	}
	else {
		mode_counter ++;
	}
}

float *maxMinSum(float current, float ret[]) {
	float max=ret[0];
	float min=ret[1];
	float sum=ret[2];
	if (max<current){
		max = current;
	}
	if (min>current){
		min = current;
	}
	sum += current;	
	ret[0]=max;
	ret[1]=min;
	ret[2]=sum;
	return ret;
}

float *resetArr(float ret[]) {
	ret[0] =0;
	ret[1] =1000000;
	ret[2] =0;
	return ret;
}

void read_temp_hum(void) {
    int temp;
    unsigned int humidity;
   
    //send humidity command
    _tx_buf[0] = SI7013_READ_RH;
    sensorI2C.write(_address, (char*)_tx_buf, 1);
    sensorI2C.read(_address, (char*)_rx_buf, 2);
   
    /* Store raw RH info */
    humidity = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    /* Convert value to milli-percent */
    humidity = (((humidity) * 15625L) >> 13) - 6000;
   
    //send temperature command
    _tx_buf[0] = SI7013_READ_TEMP;
    sensorI2C.write(_address, (char*)_tx_buf, 1);
    sensorI2C.read(_address, (char*)_rx_buf, 2);
   
    /* Store raw temperature info */
    temp = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    /* Convert to milli-degC */
    temp = (((temp) * 21965L) >> 13) - 46850;
   
    _tData = temp;    
		_tDataf = (_tData+0.0)/1000; 
		_tDataf = round(_tDataf * 100) / 100;
		
		_rhData = humidity;
		_rhDataf = (_rhData+0.0)/1000; 
		_rhDataf = round(_rhDataf * 100) / 100;		
}

void read_soil_moisture(void) {
    soilMoistureValue = soil_moisture_sensor.read_u16();  
		soilMoistureValue = (soilMoistureValue*100)/60000;
}

void read_light(void) {
    lightValue = light_sensor.read_u16();
		lightValue = (lightValue*100)/60000;
}

void read_sensorRGB()
{
    rgb_sensor.getAllColors(rgb_readings);
		
		//Get dominant color and colour the RGB Led if test mode
		red = rgb_readings[1];
		green = rgb_readings[2];
		blue = rgb_readings[3];
		if (red>green and red>blue){
			dominant = "Red";
			redCount = redCount + 1;
			if (mode_counter == 0) {
				redLed = 0;
				greenLed = 1;
				blueLed = 1;
			}
		}
		else if (green>red and green>blue) {
			dominant = "Green";
			greenCount = greenCount + 1;
			if (mode_counter == 0) {
				redLed = 1;
				greenLed = 0;
				blueLed = 1;
			}
		}
		else if (blue>red and blue>green){
			dominant = "Blue";
			blueCount = blueCount + 1;
			if (mode_counter == 0) {
				redLed = 1;
				greenLed = 1;
				blueLed = 0;
			}
		}
}

void read_accel()
{
    x=accel_sensor.getAccX();
    y=accel_sensor.getAccY();
    z=accel_sensor.getAccZ();
}

void validateLimits()
{		
				if (_tDataf<_tDatafMin || _tDataf>_tDatafMax){	
						printf("Temperature value is outside the limits\n"); //led in yellow
						redLed = 0;
						greenLed = 0;
						blueLed = 1;
						alarm_Timer.reset();
						alarm_Timer.start();
						while (flag == false){
							if (duration_cast<milliseconds>(alarm_Timer.elapsed_time()).count() >= refresh_alarmTime) {
								flag = true;
								alarm_Timer.stop();
							}
					  }
						flag = false;
				}
				
				if (_rhDataf<_rhDatafMin || _rhDataf>_rhDatafMax){
						printf("Relative Humidity value is outside the limits\n"); //led in magenta
						redLed = 0;
						greenLed = 1;
						blueLed = 0;
						alarm_Timer.reset();
						alarm_Timer.start();
						while (flag == false){
							if (duration_cast<milliseconds>(alarm_Timer.elapsed_time()).count() >= refresh_alarmTime) {
								flag = true;
								alarm_Timer.stop();
							}
					  }
						flag = false;
				}
				if (soilMoistureValue<soilMoistureValueMin || soilMoistureValue>soilMoistureValueMax){	
						printf("Soil Moisture value is outside the limits\n"); //led in red
						redLed = 0;
						greenLed = 1;
						blueLed = 1;
						alarm_Timer.reset();
						alarm_Timer.start();
						while (flag == false){
							if (duration_cast<milliseconds>(alarm_Timer.elapsed_time()).count() >= refresh_alarmTime) {
								flag = true;
								alarm_Timer.stop();
							}
					  }
						flag = false;
				}
				if (lightValue<lightValueMin || lightValue>lightValueMax){	
						printf("Light value is outside the limits\n"); //led in cian
						redLed = 1;
						greenLed = 0;
						blueLed = 0;
						alarm_Timer.reset();
						alarm_Timer.start();
						while (flag == false){
							if (duration_cast<milliseconds>(alarm_Timer.elapsed_time()).count() >= refresh_alarmTime) {
								flag = true;
								alarm_Timer.stop();
							}
					  }
						flag = false;
				}
				if (dominant != dominantWanted){
						printf("Leave color is not green\n"); // led in green
						redLed = 1;
						greenLed = 0;
						blueLed = 1;
						alarm_Timer.reset();
						alarm_Timer.start();
						while (flag == false){
							if (duration_cast<milliseconds>(alarm_Timer.elapsed_time()).count() >= refresh_alarmTime) {
								flag = true;
								alarm_Timer.stop();
							}
					  }
						flag = false;
				}
				accel = sqrt((x*x) + (y*y) + (z*z));		
				if (accel<accelMin || accel>accelMax){	
						printf("Acceleration value is outside the limits\n"); //led in blue
						redLed = 1;
						greenLed = 1;
						blueLed = 0;
						alarm_Timer.reset();
						alarm_Timer.start();
						while (flag == false){
							if (duration_cast<milliseconds>(alarm_Timer.elapsed_time()).count() >= refresh_alarmTime) {
								flag = true;
								alarm_Timer.stop();
							}
					  }
						flag = false;
				}
				printf(" \n");
}

int main()
{
		button.rise(&button_isr);
	
		redLed=1;
		greenLed=1;
		blueLed=1;
	
    // Enable RGB Sensor
    rgb_sensor.enablePowerAndRGBC();
 
    //Check if the sensor is present
    _tx_buf[0] = SI7013_READ_ID2_1;
    _tx_buf[1] = SI7013_READ_ID2_2;
   
    _address = SI7021_ADDR; 
   
    sensorI2C.write(_address, (char*)_tx_buf, 2);
    sensorI2C.read(_address, (char*)_rx_buf, 8);
    //Check ID byte
    if(_rx_buf[0] != SI7021_DEVICE_ID) {
        printf("No sensor present!\r\n");
        while(1);
    }
   
    // GPS
    gps_Serial = new UnbufferedSerial(PA_9, PA_10,9600); //serial object for use w/ GPS
    Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
    char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
    Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
    int refresh_Time; //refresh time in ms
    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    printf("Connection established at 9600 baud...\r\n");
    ThisThread::sleep_for(1s);
    refresh_Timer.start();  //starts the clock on the timer   
    
		while(true) {				
				//check wich mode we are working
				if (mode_counter==0){					
					// Test Mode		
						led1=1;
						led2=0;
						led3=0;
						refresh_Time = 2000; //2s refresh time in ms
				}
				else if (mode_counter == 1){
					// Normal Mode	
						led1=0;
						led2=1;
						led3=0;
						refresh_Time = 30000; //30s refresh time in ms
				}
				else {
				  // Advanced Mode		
						led1=0;
						led2=0;
						led3=1;
				}
				
				c = myGPS.read();   //queries the GPS, needs to be sycncronicing all the time to work properly
						//check if we recieved a new message from GPS, if so, attempt to parse it,
						if ( myGPS.newNMEAreceived() ) {
							if ( !myGPS.parse(myGPS.lastNMEA()) ) {
									continue;
							}
						}
						
        if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {
						refresh_Timer.reset();
					
						read_soil_moisture();
						read_light();
						read_temp_hum();
						read_sensorRGB();
						read_accel();
						printf("Accelerometer: x = %f \t y = %f\t z = %f\n",x,y,z);
						printf("Temperature: %.1f%cC \n", _tDataf,248);
          	printf("Humidity: %.1f%%\n", _rhDataf);
            printf("Soil Moisture  = %.1f%%  \n", soilMoistureValue);  
            printf("Light  = %.1f%%  \n", lightValue);
            printf("RGB: Clear: %d, Red: %d, Green: %d, Blue: %d  -- Dominant color: %s\n",rgb_readings[0],rgb_readings[1],rgb_readings[2],rgb_readings[3],dominant.c_str());           
            printf("Time: %d:%d:%d.%u\r\n", myGPS.hour + 1, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
            printf("Date: %d/%d/20%d\r\n", myGPS.day, myGPS.month, myGPS.year);
            printf("Quality: %d\r\n", (int) myGPS.fixquality);
            if ((int)myGPS.fixquality > 0) {
                printf("Location: %5.2f %c, %5.2f %c\r\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
                printf("Speed: %5.2f knots\r\n", myGPS.speed);
                printf("Angle: %5.2f\r\n", myGPS.angle);
                printf("Altitude: %5.2f\r\n", myGPS.altitude);
                printf("Satellites: %d\r\n", myGPS.satellites);
            }
						printf(" \n");
												
						if (mode_counter == 1){		

							validateLimits();
														
							counter1hour ++;
							maxMinSum(_tDataf,retTemp); 
							maxMinSum(_rhDataf,retRH);
							maxMinSum(soilMoistureValue,retMoist);
							maxMinSum(lightValue,retLight);							
							if (redCount>greenCount and redCount>blueCount){
								dominantHour = "Red";
							}
							else if (greenCount>redCount and greenCount>blueCount) {
								dominantHour = "Green";
							}
							else if (blueCount>redCount and blueCount>greenCount){
								dominantHour = "Blue";
							}														
							maxMinSum(abs(x),retX);
							maxMinSum(abs(y),retY);
							maxMinSum(abs(z),retZ);
							
							if (counter1hour == 120){								
								printf("Hourly Report\n");								
								printf("Maximum Temperature: %.1f, Minimum Temperature: %.1f, Mean Temperature: %.1f\n", retTemp[0], retTemp[1], retTemp[2]/counter1hour);						
								printf("Maximum Humidity: %.1f, Minimum Humidity: %.1f, Mean Humidity: %.1f\n", retRH[0], retRH[1], retRH[2]/counter1hour);
								printf("Maximum Soil Moisture: %.1f, Minimum Soil Moisture: %.1f, Mean Soil Moisture: %.1f\n", retMoist[0], retMoist[1], retMoist[2]/counter1hour);  
								printf("Maximum Light: %.1f, Minimum Light %.1f, Mean Light: %.1f\n", retLight[0], retLight[1], retLight[2]/counter1hour);
								printf("Dominant colour of the leave: %s\n", dominantHour.c_str());
								printf("Accelerometer: Max x = %f Min x = %f,  Max y = %f Min y = %f,  Max z = %f Min z = %f\n",retX[0],retX[1],retY[0],retY[1],retZ[0],retZ[1]);								
								printf(" \n");								
								resetArr(retTemp);
								resetArr(retRH);
								resetArr(retMoist);
								resetArr(retLight);
								redCount=0; 
								greenCount=0; 
								blueCount =0;
								resetArr(retX);
								resetArr(retY);
								resetArr(retZ);								
								counter1hour = 0;								
							}							
						}						
        }  
    }
}

