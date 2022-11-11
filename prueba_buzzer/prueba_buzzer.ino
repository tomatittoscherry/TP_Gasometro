#include<Wire.h>

int frecuencia = 2000;
int channel = 0;
int resolucion = 8;

void setup() {
  Serial.begin(115200);
  ledcSetup(channel, frecuencia, resolucion);
  ledcAttachPin(12, channel);
}

void loop() {

  ledcWriteTone(channel, 2000);

  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle = dutyCycle + 10) {
    Serial.println(dutyCycle);
    ledcWrite(channel, dutyCycle);
    delay(1000);
  }

  ledcWrite(channel, 125);

  for (int frecuencia =255; frecuencia <= 10000; frecuencia = frecuencia + 10) {
Serial.println(frecuencia);
ledcWriteTone(channel, frecuencia);
delay(1000);

  }
}
