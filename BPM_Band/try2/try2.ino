#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"


/*Number of samples of iir filter function*/
#define FILTER_ORDER 3
/*Minimum accepted heart rate of the function of iir filter*/
#define MIN_ACCEPTED_BPM    45
/*Miximum accepted heart rate of the function of iir filter*/
#define MAX_ACCEPTED_BPM    130

/*difference between reference &current bpm which represent awake state*/
#define DIFF_AWAKE      5
/*difference between reference &current bpm which represent fatigued state*/
#define DIFF_FATIGUED   8
/*difference between reference &current bpm which represent drowsy state*/
#define DIFF_DROWSY     13


/*Array which hold the 3 values of the bpm to filtered them*/
float x[FILTER_ORDER] = {0};
MAX30105 particleSensor;

/*Can be neglected*/
const byte RATE_SIZE = 1; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred


float beatsPerMinute;
int beatAvg;
float filteredValue;

/*reference value of the BPM while driving*/
int beat_ref;
/*Array which hold two inital values of BPM to compare them to get the reference value of the BPM while driving*/
int arr_of_initial[2];
int ref_flag=0;

String State;

/*number of readings to average*/
const int NUM_READINGS = 50; 
/*array to store the readings*/
int readings[NUM_READINGS]; 
/* index to keep track of the current reading in the array*/
int readingsIndex = 0; 
/* variable to hold the average BPM */
int averageBPM = 0;  
/*variable to keep track of the last time we averaged the readings*/
unsigned long lastAverageTime = 0; 
/*variable to hold the current time*/
unsigned long currentTime1 = 0; 

unsigned long previousTime=0;


/******************************************************************************************************************************************/

/*for the start of the trip
 get a reference value after determined time when the driver is relaxed to compare with the current BPM value to detect any change from
 his initial state
*/
void initial_ref(int input)
{
  static int i=0;
  if(input >= MIN_ACCEPTED_BPM && i <= 1)
  {
    arr_of_initial[i] = input;
     i++;
  
    if(abs(arr_of_initial[1] - arr_of_initial[0]) <=5)
    {
      beat_ref = (arr_of_initial[1] + arr_of_initial[0])/2;
      arr_of_initial[2]=0;
      i=0;
      ref_flag=1;
    }
    else if(i==2 && abs(arr_of_initial[1] - arr_of_initial[0]) > 5)
    {
       arr_of_initial[2]=0;
       i=0;
       ref_flag=0;
    }
  }
}

/**************************************************************************************************************/
/*filter function for beat per minutes to prevent values of bpm to go up or down inconsequentially*/
void iir_filter(float input, float *output) {

    static int i=0;
    float acc = 0;
    if(input > MIN_ACCEPTED_BPM && input < MAX_ACCEPTED_BPM)
    {
    x[i] = input;
    }
   if(i==2){
  acc = 0.2*x[2]+ 0.4*x[1] + 0.4*x[0] ;
    x[0] = x[1];
    x[1] = x[2];
    *output = acc;
   }
    else{
    i++;
    }
}

/**************************************************************************************************************************/
/*Function to compare between the beat reference & the current beat per minute to return the state of the driver*/
String Driver_state(int ref , int current_bpm)
{
  String My_State;
  /*this condition represent if the driver is awake*/
  if((current_bpm - ref)  < DIFF_AWAKE )
  {
    My_State = "Awake";
  }
  /*this condition if the driver is Fatigued*/
  else if((ref - current_bpm ) > DIFF_FATIGUED)
  {
    My_State = "Fatigued";
  }


   else if((ref - current_bpm ) > DIFF_DROWSY)
  {
    My_State = "Drowsy";
  }

  return My_State ;
  
}


/*****************************************************************************************************************************/
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}


/****************************************************************************************************************************************/


void loop() {

/*infrared value of the sensor*/
  long irValue = particleSensor.getIR();
/*function to check Existence of a beat */
  if (checkForBeat(irValue) == true)
  {
    Serial.println("We sensed a beat! ***********************************");

    /*calculate the time between beat and other  to get the bpm*/
    /*millis is a built in function get the runinng time in millisecond (act as timer) */
    long delta = millis() - lastBeat;
    lastBeat = millis();

    /*don't accept if the time between beat & other lower than 300 millisecond*/
    if(delta > 300)
    {
      /*you can calculate the bpm with
       60/total second between peak & peak ( R-R Interval)
      divided by 1000 to convert milliseconds into seconds
      */
    beatsPerMinute = 60 / (delta / 1000.0);

     /* filter the bpm values*/
    iir_filter(beatsPerMinute,&filteredValue);
      
        
      rates[rateSpot++] = (byte)filteredValue; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      
/* Next 4 lines can be neglected*/
      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
   
    }
  }

  // add the current beatAvg reading to the array
  readings[readingsIndex] = beatAvg;

  // increment the index and wrap around if necessary
  readingsIndex++;
  if (readingsIndex >= NUM_READINGS) {
    readingsIndex = 0;
  }

  // check if it's been one minute since the last time we averaged the readings
  currentTime1 = millis();
  if (currentTime1 - lastAverageTime >= 60000) {
    // calculate the average of the readings
    averageBPM = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
      averageBPM += readings[i];
    }
    averageBPM /= NUM_READINGS;

    // print the average
    Serial.print("Average BPM: ");
    Serial.println(averageBPM);

    // reset the last average time
    lastAverageTime = currentTime1;
  }


  unsigned long currentTime=millis();

      if(ref_flag==0 && currentTime - previousTime>=60000)
      {
          initial_ref(averageBPM);
         previousTime=currentTime;
      }

      Serial.print("ref: ");
    Serial.println(beat_ref);

    State = Driver_state(beat_ref,averageBPM);

    Serial.print("State: ");
    Serial.println(State);  

   
}
