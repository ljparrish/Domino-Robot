#define VAC_SENSOR_PIN A1
#define PUMP_PIN_1 5
#define PUMP_PIN_2 4
#define TOGGLE_PIN 7

double pressureReading;
unsigned long lastPressureTime;
bool pumpOn;


void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Vaccum Sensor Started...");
  pinMode(PUMP_PIN_1,OUTPUT);
  pinMode(PUMP_PIN_2,OUTPUT);
  pinMode(TOGGLE_PIN,INPUT_PULLUP);
  pumpOn = 0;
}

void loop() {
  unsigned long timeSinceLastPressure = millis() - lastPressureTime;
  if (timeSinceLastPressure > 100) {
    pressureReading = getPressure();
    if (!digitalRead(TOGGLE_PIN)) {
      setMotorState(1);
    } else {
      setMotorState(0);
    }
    lastPressureTime = millis();
    Serial.println(pressureReading);
  }
}

double getPressure() {
  double Voltage = analogRead(VAC_SENSOR_PIN) / 1023.0 * 5.0;
  double Pressure = 1/0.018 * (Voltage/5.0 - 0.92);
  return Pressure;
}

void setMotorState(int motorDirection) {
  // -1 is reverse
  // 0 is stopped
  // 1 is forward
  switch(motorDirection) {
    case -1:
      digitalWrite(PUMP_PIN_1,HIGH);
      digitalWrite(PUMP_PIN_2,LOW);
      break;
    case 0:
      digitalWrite(PUMP_PIN_1,LOW);
      digitalWrite(PUMP_PIN_2,LOW);
      break;
    case 1:
      digitalWrite(PUMP_PIN_1,LOW);
      digitalWrite(PUMP_PIN_2,HIGH);
      break;
  } 
}
