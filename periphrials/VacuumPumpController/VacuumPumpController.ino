#define USE_USBCON

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>


#define VAC_SENSOR_PIN A1
#define PUMP_PIN_1 5
#define PUMP_PIN_2 4
#define TOGGLE_PIN 7

ros::NodeHandle nh;
std_msgs::Float32 pressure_msg;
ros::Publisher pub_pressure("pressure", &pressure_msg);

float pressureReading;
unsigned long lastPressureTime;
bool pumpOn;

void vac_ctrlCallback(const std_msgs::Int8& vacuum_state){
  bool buttonSignal = digitalRead(TOGGLE_PIN);
  Serial.println(vacuum_state.data);
  if(vacuum_state.data == 1) {
    pumpOn = HIGH;
  } else {
    pumpOn = LOW;
  }
}

ros::Subscriber<std_msgs::Int8> sub_ctrl("vacuum_state", vac_ctrlCallback);

void setup() {
  Serial.begin(9600);
  Serial.println("Vaccum Sensor Started...");
  nh.initNode();
  nh.advertise(pub_pressure);

  nh.initNode();
  nh.subscribe(sub_ctrl);
  
  pinMode(PUMP_PIN_1,OUTPUT);
  pinMode(PUMP_PIN_2,OUTPUT);
  pinMode(TOGGLE_PIN,INPUT_PULLUP);
  pumpOn = 0;
}

void loop() {
  unsigned long timeSinceLastPressure = millis() - lastPressureTime;
  if (timeSinceLastPressure > 100) {
    pressureReading = getPressure();
    if(!digitalRead(TOGGLE_PIN)) {
      pumpOn = HIGH;
    }
    setMotorState(pumpOn);
    pressure_msg.data = pressureReading;
    pub_pressure.publish(&pressure_msg);
    nh.spinOnce();
    lastPressureTime = millis();
    Serial.println(pressureReading);
  }
}

double getPressure() {
  double Voltage = analogRead(VAC_SENSOR_PIN) / 1023.0 * 5.0;
  double Pressure = 1/0.018 * (Voltage/5.0 - 0.92);
  return Pressure;
}

void setMotorState(bool motorDirection) {
  // -1 is reverse
  // 0 is stopped
  // 1 is forward
  switch(motorDirection) {
    case LOW:
      digitalWrite(PUMP_PIN_1,LOW);
      digitalWrite(PUMP_PIN_2,LOW);
      break;
    case HIGH:
      digitalWrite(PUMP_PIN_1,LOW);
      digitalWrite(PUMP_PIN_2,HIGH);
      break;
  } 
}
