
#include <I2C.h>
//#include <Wire.h>
#include <t5403.h>
#include <PID_v1.h>
#include <TimerOne.h>
#include <TimerThree.h>

#define CHANNEL_MODE 3
#define CHANNEL_THROTTLE 7
#define CHANNEL_OUTPUT 11


double mean2;
double Kp = 20, Ki = 2, Kd = 2, pressure_target, current_pressure, PID_output;


unsigned long ulToggleTimer, ulPrevToggleTimer, lastserialtime, calculatedtime, modetimer;
unsigned int prevpinvalue, modepinvalue, Current_mode, OutputValue;


T5403 barometer(MODE_I2C);
PID myPID(&current_pressure, &PID_output, &pressure_target, Kp, Ki, Kd, REVERSE);
void setup() {
  Serial.begin(9600);
  barometer.begin();

  pinMode(CHANNEL_MODE, INPUT);
  pinMode(CHANNEL_THROTTLE, INPUT);
  pinMode(CHANNEL_OUTPUT, OUTPUT);
  Current_mode = 0;
  prevpinvalue = LOW;
  lastserialtime = millis();
  modetimer = millis();
  attachInterrupt(0, mode_change_rising, RISING);
  attachInterrupt(4, throttle_rising, RISING);
  Timer1.initialize(5000);
  Timer1.stop();
  Timer1.restart();

  Timer3.initialize(5000);
  Timer3.stop();
}

void throttle_rising() {
  detachInterrupt(4);
  //Serial.println("Throttle rising edge");
  if (!Current_mode) {
    digitalWrite(CHANNEL_OUTPUT, HIGH);
    //Serial.println("CHANNEL_OUTPUT high");
    //digitalWrite(13, HIGH);
  }
  attachInterrupt(4, throttle_falling, FALLING);
}

void throttle_falling() {
  detachInterrupt(4);
  if (!Current_mode) {
    digitalWrite(CHANNEL_OUTPUT, LOW);
    //digitalWrite(13, LOW);
  }
  attachInterrupt(4, throttle_rising, RISING);
}

void mode_change_rising() {
  detachInterrupt(0);
  Timer1.restart();
  Timer1.start();
  attachInterrupt(0, mode_change_falling, FALLING);
}

void mode_change_falling() {
  detachInterrupt(0);
  calculatedtime = Timer1.read();
  Timer1.stop();
  prevpinvalue = LOW;
  if ((calculatedtime > 1000) && (calculatedtime < 2500)) {
    if (calculatedtime > 1500) {
      modepinvalue = HIGH;
    } else
    {
#if 0
      if (modepinvalue = HIGH) {
        Serial.println("Channel 5 went low");
        Serial.println(calculatedtime);
      }
#endif
      modepinvalue = LOW;

    }
    //Serial.println(calculatedtime);
    /*if ((calculatedtime < 1000) || (calculatedtime > 2500)) {

      Serial.println("ERROR");
      Serial.println(calculatedtime);
    }*/
  }
  attachInterrupt(0, mode_change_rising, RISING);
}

int verify_pressure(double x) {
  if ((x > 102000) || (x < 99000)) {
    return 1;
  }
  return 0;
}

void loop() {
  if (modepinvalue == LOW) { //manual throttle
    //Serial.println("Channel 5 low");
    if (Current_mode == 1)
    {
      Current_mode = 0;
      //switching from auto to manual mode. Do stuff here?
      myPID.SetMode(MANUAL);
    }
    //Code for throttle passthrough here. Not required. Handled in interrupts.
  } else if (modepinvalue == HIGH) { //Alt hold
    if (Current_mode == 0) {
      detachInterrupt(0);
      digitalWrite(13, HIGH);
      // Serial.println(Current_mode);
      //Setup hold ALT
      Serial.println("Entering AUTO mode");
      //if ((millis() - modetimer) < 1000 ) goto exitfirstauto; //switch debouncing for high speed loop later
      //  modetimer = millis();
      double samplepressure[5];
#if 1
sampling:
      //Need to add timeout here to sampling just incase it gets stuck here. Loop = no throttle
      barometer.begin();

      samplepressure[0] = barometer.getPressure(MODE_ULTRA);
      if (verify_pressure(samplepressure[0])) goto sampling;

      samplepressure[1] = barometer.getPressure(MODE_ULTRA);
      if (verify_pressure(samplepressure[1])) goto sampling;

      samplepressure[2] = barometer.getPressure(MODE_ULTRA);
      if (verify_pressure(samplepressure[2])) goto sampling;

      samplepressure[3] = barometer.getPressure(MODE_ULTRA);
      if (verify_pressure(samplepressure[3])) goto sampling;

      samplepressure[4] = barometer.getPressure(MODE_ULTRA);
      if (verify_pressure(samplepressure[4])) goto sampling;


      pressure_target = (samplepressure[0] + samplepressure[1] + samplepressure[2] + samplepressure[3] + samplepressure[4]) / 5;

#else
      pressure_target = barometer.getPressure(MODE_ULTRA);
#endif

      Serial.println(pressure_target);
      myPID.SetMode(AUTOMATIC); //turn on PID
      digitalWrite(13, LOW);
      //modepinvalue = LOW;
      Current_mode = 1;
exitfirstauto:


      attachInterrupt(0, mode_change_rising, RISING);


    } else
    {
      //Serial.println("Second AUTO");
      //      Serial.println(barometer.getPressure(MODE_ULTRA));
      //Do PID calcs
      current_pressure = barometer.getPressure(MODE_ULTRA);
      myPID.Compute();

      Serial.print(PID_output);
      Serial.print(" ");
      Serial.println(current_pressure);
      //PID myPID(&current_pressure, &PID_output, &pressure_target, Kp, Ki, Kd, REVERSE);
#if 0
      Serial.println("PID details");
      Serial.print(current_pressure);
      Serial.print(" ");
      Serial.print(PID_output);
      Serial.print(" ");
      Serial.println(pressure_target);
      Serial.print(" ");
      Serial.println(Kp);
      Serial.print(" ");
      Serial.println(Ki);
      Serial.print(" ");
      Serial.println(Kd);
#endif
      OutputValue = (PID_output * 3.92) + 1020;
      //Serial.println(temp);
      //Need to do PPM here on pin 11
      //analogWrite(11,OutputValue);
      digitalWrite(13, HIGH);
      digitalWrite(CHANNEL_OUTPUT, HIGH);
      Timer3.attachInterrupt(auto_output, OutputValue);
    }
  }
  //delay(100);
}//end main loop

void auto_output() {
  Timer3.detachInterrupt();
  digitalWrite(CHANNEL_OUTPUT, LOW);
  digitalWrite(13, LOW);
  Timer3.attachInterrupt(auto_output, (5130 - OutputValue));
}
