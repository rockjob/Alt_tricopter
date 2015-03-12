
#include <I2C.h>
//#include <Wire.h>
#include <t5403.h>
#include <PID_v1.h>
#include <TimerOne.h>

#define CHANNEL_MODE 3
#define CHANNEL_THROTTLE 7
#define CHANNEL_OUTPUT 8


double mean2;
double Kp = 1, Ki = 0.5, Kd = 0.25, pressure_target, current_pressure, PID_output;


unsigned long ulToggleTimer, ulPrevToggleTimer, lastserialtime, calculatedtime, modetimer;
unsigned int prevpinvalue, modepinvalue, Current_mode;


T5403 barometer(MODE_I2C);
PID myPID(&current_pressure, &PID_output, &pressure_target, Kp, Ki, Kd, 1);
void setup() {
  // put your setup code here, to run once:
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
  Timer1.initialize(5000);
  Timer1.stop();
  Timer1.restart();

}

void throttle_rising() {
  detachInterrupt(4);
  if (!Current_mode) digitalWrite(CHANNEL_OUTPUT, HIGH);
  attachInterrupt(4, throttle_falling, FALLING);
}

void throttle_falling() {
  detachInterrupt(4);
  if (!Current_mode) digitalWrite(CHANNEL_OUTPUT, LOW);
  attachInterrupt(4, throttle_rising, RISING);
}

void mode_change_rising() {



  detachInterrupt(0);
  Timer1.restart();
  Timer1.start();
  //prevpinvalue = HIGH;
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
    }
    //Code for throttle passthrough here
    //digitalWrite(CHANNEL_OUTPUT, digitalRead(CHANNEL_THROTTLE));

  } else if (modepinvalue == HIGH) { //Alt hold

    if (Current_mode == 0) {
      detachInterrupt(0);
      digitalWrite(13, HIGH);
      // Serial.println(Current_mode);
      //Setup hold ALT
      //Serial.println("Entering AUTO mode");
      //if ((millis() - modetimer) < 1000 ) goto exitfirstauto; //switch debouncing for high speed loop later
      //  modetimer = millis();

      double samplepressure[5];

#if 1
sampling:

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
      Current_mode = 1;
      Serial.println(pressure_target);

exitfirstauto:
      digitalWrite(13, LOW);
      modepinvalue = LOW;
      attachInterrupt(0, mode_change_rising, RISING);


    } else
    {
      //Serial.println("Second AUTO");
      Serial.println(barometer.getPressure(MODE_ULTRA));
      //Do PID calcs
    }
  }
  delay(50);
}//end main loop



