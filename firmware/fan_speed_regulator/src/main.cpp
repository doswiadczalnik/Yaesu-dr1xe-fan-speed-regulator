#include <Arduino.h>
#include <OneWire.h>

#define DS18B20_SKIP_ROM	0xCC
#define DS18B20_CONVERT_T 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

#define FAN_PIN PB0 // Analog output pin that the FAN is attached to
#define TEMP_SENSOR_PIN PB2 // Digital input pin that the DS18B20 sensor is attached to

#define PWM_MIN_VALUE 128
#define TEMP_START_TEMP 30


uint8_t sensorValue;       // value read from the potentiometer
int8_t temperature;         //value of read temperature
int8_t tempDiff;
int16_t outputValue;

OneWire ds(TEMP_SENSOR_PIN); 

int8_t readTemperature()
{
  ds.reset();
  ds.write(DS18B20_SKIP_ROM);
  ds.write(DS18B20_CONVERT_T);
  delay(1000);

  ds.reset();
  ds.write(DS18B20_SKIP_ROM);
  ds.write(DS18B20_READ_SCRATCHPAD);
  uint8_t lsb = ds.read();
  int8_t msb = ds.read();

  msb = msb << 4;
  lsb = lsb >> 4;

  return msb | lsb;
}

void initPwm()
{
  DDRB |= _BV(FAN_PIN); // set PWM pin as OUTPUT
  TCCR0A |= _BV(COM0A1)| _BV(WGM01)|_BV(WGM00); // set timer mode to FAST PWM and enable PWM signal on pin (AC0A => PB0)
  TCCR0B |= _BV(CS00); // start timer without prescaler
}

void setup() {
  cli();
  // put your setup code here, to run once:
  initPwm();
}

void loop() {
  // put your main code here, to run repeatedly:
  temperature = readTemperature();

  tempDiff = temperature - TEMP_START_TEMP;

  if (tempDiff >= 0)
  {
    outputValue = PWM_MIN_VALUE + tempDiff * 4;

    if (outputValue > 254)
    {
      outputValue = 254;
    }

    if (outputValue < 0)
    {
      outputValue = 0;
    }

    OCR0A = (uint8_t)outputValue;
  }
  else
  {
    OCR0A = 0;
  }
}