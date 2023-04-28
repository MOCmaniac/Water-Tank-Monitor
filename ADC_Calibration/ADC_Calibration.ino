// Since the ADC2 module is also used by the Wi-Fi, adc2_get_raw() may get blocked until Wi-Fi stops, and vice versa.
// ADC1 pins : GPIO32 - GPIO39
// GPIO : 34 - 39 are input only (GPI)
// Only pins that support both input & output have integrated pull-up and pull-down resistors.
// Pin 32, 33 are the only one on ADC1 configurable as output
#define VOLTAGE_DIVIDER_GROUND_PIN GPIO_NUM_33  // Vbat - R2 - VOLTAGE_DIVIDER_PIN - R1 - GPIO33 and 100µF decoupling capacitor : VOLTAGE_DIVIDER_PIN - Ground
#define VOLTAGE_DIVIDER_PIN GPIO_NUM_32
#define SETTLING_TIME 50         // Time for the capacitor to be stable (measured at around 20ms)
#define VOLTAGE_OVERSAMPLING 32  // With an average over 32 readings, precision seems to be +- 1mV (precision is different from accuracy)

#define STEADY 2
#define EMA 0.95

float Vin = 0;
float lastADC = 0;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("\n\n-----------------\nSetting pins mode");
  pinMode(VOLTAGE_DIVIDER_PIN, INPUT);
  pinMode(VOLTAGE_DIVIDER_GROUND_PIN, OUTPUT);

  digitalWrite(VOLTAGE_DIVIDER_GROUND_PIN, LOW);
  Serial.println("Voltage divider ground pin set to LOW");

  lastADC = analogReadOversampling(VOLTAGE_DIVIDER_PIN, VOLTAGE_OVERSAMPLING);
}

void loop() {
  if (millis() > startTime + 100) {
    float adc = analogReadOversampling(VOLTAGE_DIVIDER_PIN, VOLTAGE_OVERSAMPLING);
    adc = EMA * lastADC + (1 - EMA) * adc;

    float Vin = adc * 3.3 / 4095;
    float adc_c = ADCCorrection(adc);
    float Vin_c = adc_c * 3.3 / 4095;

    Serial.printf("ADC : %.0f\tADC C : %.0f\t Vin : %.3f\t Vin C : %.3f", adc, round(adc_c), Vin, Vin_c);
    Serial.printf("\t%s\n", abs(lastADC - adc) < 2 ? "STEADY" : "");

    lastADC = adc;
    startTime = millis();
  }
}

float analogReadOversampling(int pin, int oversampling) {
  float adc = 0;

  for (int x = 0; x < oversampling; x++) {
    adc += analogRead(pin);  // Range : 0-4095
    delay(1);                // Reading stability
  }
  return adc / oversampling;
}

float ADCCorrection(int in) {
  return 1.0 * in + 0.0;
}