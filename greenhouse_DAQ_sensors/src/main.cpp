#include <EEPROM.h>
#include <OneWire.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <Wire.h>
#include <SDL_Arduino_INA3221.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Sleep_n0m1.h>
#include <SoftwareSerial.h>
#include <BME280I2C.h>
#include <BH1750.h>

// Display arduino I2C address
#define DISPLAY_ARDUINO_I2C_ADDRESS 50
#define BUFFER_SIZE 70

// SigFox modem serial link
#define rxPin 10
#define txPin 7
SoftwareSerial sigfox = SoftwareSerial(rxPin, txPin);

// BME 280 sensor init
BME280I2C bme;

// BH 1750 sensor init
BH1750 lightMeter;

// create sleep object
Sleep sleep;

// serial debug output enabled
bool debug = true;

// DS1820 init (out)
#define DS_OUT_PIN  4
OneWire oneWireDS(DS_OUT_PIN);
DallasTemperature sens_DS_out(&oneWireDS);

// Current probe
SDL_Arduino_INA3221 ina3221;

// global variables
#define SIGFOX_INTERVAL 15   // in minutes
byte loops = 0;
int temperature_in[SIGFOX_INTERVAL];
int temperature_in_avg = 0;

int temperature_out[SIGFOX_INTERVAL];
int temperature_out_avg = 0;

byte humidity_in[SIGFOX_INTERVAL];
byte humidity_in_avg = 0;

int pressure[SIGFOX_INTERVAL];
int pressure_avg = 0;

unsigned int light[SIGFOX_INTERVAL];
unsigned int light_avg = 0;

int voltage[SIGFOX_INTERVAL];
int voltage_avg = 0;

int current[SIGFOX_INTERVAL];
int current_avg = 0;

int power = 0;
boolean abortSleep = false;
char buff[BUFFER_SIZE];

void readOutsideTemperature(bool debug) {
  // read outside temperature

  sens_DS_out.requestTemperatures();
  delay(1000);
  temperature_out[loops] = 10 * sens_DS_out.getTempCByIndex(0);

  if (debug) {
    Serial.print("T out: ");
    Serial.print(float(temperature_out[loops] / 10.0), 1);
    Serial.print(" °C ; ");
  }
}

void sendToDisplay(bool debug) {

  sprintf(buff, "%04X%04X%02X%04X%04X%04X%04X%04X", temperature_out[loops], temperature_in[loops], humidity_in[loops], pressure[loops], light[loops], voltage[loops], current[loops], power);
  Wire.beginTransmission(DISPLAY_ARDUINO_I2C_ADDRESS);
  Wire.write(buff);
  Wire.endTransmission();

    Serial.print(buff);
    if (debug) {
    Serial.println(" data sent to Display Arduino");
  }
}

void sendSigFoxData() {
  byte batt_volt;
  if (voltage_avg > 8){
    batt_volt = byte(float((voltage_avg - 8.0)) / 0.03);
  } else {
    batt_volt = 8;
  }

  sprintf(buff, "AT$SF=%04X%04X%02X%02X%04X%04X%04X\n", temperature_in_avg, temperature_out_avg, humidity_in_avg, batt_volt, current_avg, light_avg, pressure_avg);

  sigfox.print(buff);
  delay(10000);

  if (debug) {
    Serial.print(buff);
    Serial.println(" String sent to SigFox modem.");
  }
}

long calcAverage(int data[]) {
  long sum = 0;
  for (byte i = 0; i < SIGFOX_INTERVAL; i++) {
    sum += data[i];
  }
  return sum / SIGFOX_INTERVAL;
}

byte calcAverage(byte data[]) {
  long sum = 0;
  for (byte i = 0; i < SIGFOX_INTERVAL; i++) {
    sum += data[i];
  }
  return sum / SIGFOX_INTERVAL;
}

long calcAverage(unsigned int data[]) {
  long sum = 0;
  for (byte i = 0; i < SIGFOX_INTERVAL; i++) {
    sum += data[i];
  }
  return sum / SIGFOX_INTERVAL;
}

void calcPower (bool verb) {
  float tmp;
  tmp = float(voltage[loops]) * float(current[loops]) * 0.001;
  power = int(tmp);
  if (verb) {
    Serial.print(" ");
    Serial.print(power);
    Serial.print(" mW ");
  }
}

void readCurrent(bool verb) {
  current[loops] = ina3221.getCurrent_mA(2);
  if (verb) {
    Serial.print(" ");
    Serial.print(ina3221.getCurrent_mA(2), 1);
    Serial.print(" mA ");
  }
}

void readVoltage(bool verb) {
  voltage[loops] = int(1000 * ina3221.getBusVoltage_V(2));

  if (verb) {
    Serial.print(" ");
    Serial.print(ina3221.getBusVoltage_V(2), 2);
    Serial.print(" V ");
  }
}

void calcAverage() {
  temperature_in_avg = calcAverage(temperature_in);
  temperature_out_avg = calcAverage(temperature_out);
  voltage_avg = calcAverage(voltage);
  current_avg = calcAverage(current);
  humidity_in_avg = calcAverage(humidity_in);
  pressure_avg = calcAverage(pressure);
  light_avg = calcAverage(light);

  if (debug) {
    Serial.println();
    Serial.print(SIGFOX_INTERVAL);
    Serial.println(" min interval averages:");
    Serial.println("Inside: ");
    Serial.print("Temperature: ");
    Serial.print(temperature_in_avg / 10.0);
    Serial.print(" deg. C ; ");

    Serial.print("Humidity: ");
    Serial.print(humidity_in_avg);
    Serial.println(" %");

    Serial.println();
    Serial.println("Outside");
    Serial.print("Temperature: ");
    Serial.print(temperature_out_avg / 10.0);
    Serial.println(" deg. C");

    Serial.println();
    Serial.print("Air pressure: ");
    Serial.print(pressure_avg);
    Serial.println(" hPa");

    Serial.println();
    Serial.print("Light intensity: ");
    Serial.print(light_avg);
    Serial.println(" lux");

    Serial.println();
    Serial.println("Battery");
    Serial.print("Voltage: ");
    Serial.print(voltage_avg / 1000.0);
    Serial.print(" V ; ");

    Serial.print("Current: ");
    Serial.print(current_avg);
    Serial.print(" mA ; ");

    Serial.print("Power: ");
    Serial.print(voltage_avg * current_avg / 1000);
    Serial.println(" mW");
    Serial.println();
  }
}

int altitudeCompensation(float pressure) {
  return round(pressure / exp(-127.819679967 / (0.831432 * (2731.5 + temperature_in[loops]))));
}

void readInsideTemperature(bool debug) {

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  float temp(NAN), hum(NAN), pres(NAN);
  bme.read(pres, temp, hum, tempUnit, presUnit);

  temperature_in[loops] = 10 * temp;
  humidity_in[loops] = hum;
  pressure[loops] = altitudeCompensation(pres);

  if (debug) {
    Serial.print("T in: ");
    Serial.print(float(temperature_in[loops] / 10.0),1);
    Serial.print(" °C ; ");
    Serial.print("H in: ");
    Serial.print(humidity_in[loops]);
    Serial.print(" % ; ");
    Serial.print("Air pressure: ");
    Serial.print(pressure[loops]);
    Serial.print(" hPa ; ");
  }

}

void readLightIntensity(bool debug) {

  light[loops] = lightMeter.readLightLevel();
  lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);


  if (debug) {
    Serial.print("Light intensity: ");
    Serial.print(light[loops]);
    Serial.print(" lux ; ");
  }
}

void runMeasurement(bool debug) {

  readOutsideTemperature(debug);
  readInsideTemperature(debug);
  readLightIntensity(debug);
  readVoltage(debug);
  readCurrent(debug);
  calcPower(debug);

  if (debug) {
    Serial.println();
  }
}

void setup(void) {

  // reset DIO state
  for (byte i = 3; i < 14; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // SigFox modem link init

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);


  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  power_usart0_enable();// Serial (USART)

  // serial links init
  Serial.begin(19200);
  sigfox.begin(9600);
  Wire.begin();

  // init DS1820 sensor
  sens_DS_out.begin();
  sens_DS_out.setResolution(12); // set resolution to 12-bit

  // init BME 280 Sensor
  bme.begin();

  // BH1750 sensor init
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);

  // current probe init
  // INA 3221 datasheet Table 5
  // B0010010010010011 = 0x2493 (only Ch2 enabled, 4 sample AVG, 2.1 ms conv time bus,
  // 2.1 ms conv time shunt, single shot bus and shunt readout

  ina3221.wireWriteRegister(0x0, 0x2493);
  ina3221.begin();
  delay(500);
  }

void loop(void) {

  if (loops >= SIGFOX_INTERVAL) {
    calcAverage();
    sendSigFoxData();
    loops = 0;
  }
  runMeasurement(debug);
  sendToDisplay(debug);

  loops++;
  sleep.sleepDelay(55000, abortSleep); // 55000

}
