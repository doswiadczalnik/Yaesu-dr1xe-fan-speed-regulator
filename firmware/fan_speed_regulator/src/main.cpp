#include <Arduino.h>
#include <OneWire.h>
#include <avr/sleep.h>

#define DS18B20_SKIP_ROM	0xCC
#define DS18B20_CONVERT_T 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

#define FAN_PIN PB0 // Analog output pin that the FAN is attached to
#define TEMP_SENSOR_PIN PB2 // Digital input pin that the DS18B20 sensor is attached to

#define PWM_MIN_VALUE 50
#define TEMP_START_TEMP 30
#define PWM_START_VALUE 255

uint8_t sensorValue;       // value read from the DS18B20
int8_t temperature;         //value of read temperature
int8_t tempDiff;
int16_t outputValue;
boolean isPwmRunning = false;

OneWire ds(TEMP_SENSOR_PIN); 

int8_t readTemperature()
{
  ds.reset();
  ds.write(DS18B20_SKIP_ROM);
  ds.write(DS18B20_CONVERT_T);
  _delay_ms(1000);

  ds.reset();
  ds.write(DS18B20_SKIP_ROM);
  ds.write(DS18B20_READ_SCRATCHPAD);
  uint8_t lsb = ds.read();
  int8_t msb = ds.read();

  msb = msb << 4;
  lsb = lsb >> 4;

  return msb | lsb;
}

void startPwm()
{
  TCCR0B |= _BV(CS00); // start timer without prescaler
  TCCR0A |= _BV(COM0A1);
}

void stopPwm()
{
  TCCR0B &= ~(_BV(CS02)|_BV(CS01)|_BV(CS00)); // stop timer
  TCCR0A &= ~(_BV(COM0A1)|_BV(COM0A0));
}

void initPwm()
{
  DDRB |= _BV(FAN_PIN); // set PWM pin as OUTPUT
  TCCR0A |= _BV(WGM01)|_BV(WGM00); // set timer mode to FAST PWM and enable PWM signal on pin (AC0A => PB0)
}

void initSleep()
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
}

void initWatchdog()
{
  MCUSR &= ~_BV(WDRF);
  WDTCR |= _BV(WDTIE)|_BV(WDP3)|_BV(WDP0);
}

ISR(WDT_vect) //empty - do nothing, just wakeup
{
}

void setup()
{
  sei();
  initWatchdog();
  initPwm();
}

void loop()
{
  wdt_reset();

  temperature = readTemperature();

  tempDiff = temperature - TEMP_START_TEMP;

  if (tempDiff >= 0)
  {
    outputValue = PWM_MIN_VALUE + tempDiff * 10;

    if (outputValue > 255)
    {
      outputValue = 255;
    }

    if (outputValue < 0)
    {
      outputValue = 0;
    }

    if (!isPwmRunning)
    {
      OCR0A = PWM_START_VALUE;
      startPwm();
      isPwmRunning = true;
      _delay_ms(500);
      OCR0A = (uint8_t)outputValue;
    }
    else
      OCR0A = (uint8_t)outputValue;
  }
  else
  {
    stopPwm();
    isPwmRunning = false;
  }

  initSleep();
  sleep_cpu();
  sleep_disable();
}