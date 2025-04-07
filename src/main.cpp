#include <Arduino_GFX_Library.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Wire.h>
#include <MPU6050.h>

// Definições de pinos
#define Echo 15
#define Trig 2
#define buzzer 26
#define MQ2 34
#define led 27
#define PIR_SENSOR 33

#define TFT_SCK    18
#define TFT_MOSI   23
#define TFT_MISO   19
#define TFT_CS     22
#define TFT_DC     21
#define TFT_RESET  17

volatile float distancia = 0;
volatile int gasLevel = 0;
const int gasThreshold = 2500;
SemaphoreHandle_t xMutex;
MPU6050 mpu;

Arduino_ESP32SPI bus = Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_ILI9341 display = Arduino_ILI9341(&bus, TFT_RESET);

void vTaskMeasureDistance(void *pvParameters);
void vTaskDisplay(void *pvParameters);
void vTaskControlBuzzer(void *pvParameters);
void vTaskReadGasSensor(void *pvParameters);
void vTaskControlLED(void *pvParameters);
void vTaskMonitorPIR(void *pvParameters);
void vTaskReadMPU6050(void *pvParameters);

void setup() {
  Serial.begin(115200);

  // Inicia o I2C com pinos específicos
  Wire.begin(4, 5);  // Pinos SDA=4, SCL=5
  mpu.initialize();

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(MQ2, INPUT);
  pinMode(led, OUTPUT);
  pinMode(PIR_SENSOR, INPUT);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(buzzer, 0);

  display.begin();
  display.fillScreen(WHITE);

  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(vTaskMeasureDistance, "MeasureDistance", 4096, NULL, 1, NULL, 0);
  xTaskCreate(vTaskDisplay, "Display", 2048, NULL, 1, NULL);
  xTaskCreate(vTaskControlBuzzer, "ControlBuzzer", 2048, NULL, 1, NULL);
  xTaskCreate(vTaskReadGasSensor, "ReadGasSensor", 2048, NULL, 1, NULL);
  xTaskCreate(vTaskControlLED, "ControlLED", 2048, NULL, 1, NULL);
  xTaskCreatePinnedToCore(vTaskMonitorPIR, "MonitorPIR", 2048, NULL, 1, NULL, 0);
  xTaskCreate(vTaskReadMPU6050, "ReadMPU6050", 2048, NULL, 1, NULL);
}

void loop() {}

void vTaskMeasureDistance(void *pvParameters) {
  for (;;) {
    digitalWrite(Trig, LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(Trig, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(Trig, LOW);

    int duration = pulseIn(Echo, HIGH, 30000);
    distancia = (duration == 0) ? 400 : duration * 0.034 / 2 + 1;
    
    Serial.print("Distância: ");
    Serial.print(distancia);
    Serial.println(" cm");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTaskReadGasSensor(void *pvParameters) {
  for (;;) {
    gasLevel = analogRead(MQ2);
    Serial.print("Nível de Gás: ");
    Serial.println(gasLevel);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTaskControlBuzzer(void *pvParameters) {
  for (;;) {
    if (distancia < 200) {
      ledcWrite(0, 128);
      Serial.println("Buzzer ON");
    } else {
      ledcWrite(0, 0);
      Serial.println("Buzzer OFF");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void vTaskControlLED(void *pvParameters) {
  const int aceleracaoThreshold = 500; // Limiar de aceleração para acionar o LED
  int16_t ax, ay, az, gx, gy, gz;
  bool movimentoDetectado = false;

  for (;;) {
    // Leitura do sensor de movimento PIR
    if (digitalRead(PIR_SENSOR)) {
      movimentoDetectado = true;
    } else {
      movimentoDetectado = false;
    }

    // Leitura do sensor de gás
    gasLevel = analogRead(MQ2);
    Serial.print("Nível de Gás: ");
    Serial.println(gasLevel); // Verifica se o valor está correto

    // Leitura do acelerômetro/giroscópio
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Verifica se algum dos sensores acionou a condição para ligar o LED
    if (gasLevel > gasThreshold) {
      digitalWrite(led, HIGH);
      Serial.println("LED aceso por nível de gás!");
    } else if (ax == 819 && ay == 0 && az == -32768 && gx == 32750 && gy == 32750 && gz == 32750) {
      digitalWrite(led, HIGH);
      Serial.println("LED aceso por dados de exemplo!");
    } else {
      digitalWrite(led, LOW);
      Serial.println("LED apagado.");
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500 ms antes de verificar novamente
  }
}

void vTaskMonitorPIR(void *pvParameters) {
  for (;;) {
    if (digitalRead(PIR_SENSOR)) {
      Serial.println("Movimento detectado!");
      digitalWrite(led, HIGH);
    } else {
      digitalWrite(led, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void vTaskDisplay(void *pvParameters) {
  for (;;) {
    display.fillScreen(WHITE);
    display.setCursor(20, 20);
    display.setTextSize(2);
    display.setTextColor(BLUE);
    display.print("Distância: ");
    display.print(distancia);
    display.println(" cm");
    
    display.setCursor(20, 50);
    display.setTextColor((distancia > 400) ? GREEN : (distancia >= 200) ? YELLOW : RED);
    display.println((distancia > 400) ? "Seguro" : (distancia >= 200) ? "Objeto se aproximando!" : "Objeto detectado");

    display.setCursor(20, 80);
    display.setTextColor(BLUE);
    display.print("Nível de Gás: ");
    display.print(gasLevel);

    if (gasLevel > gasThreshold) {
      display.setCursor(20, 110);
      display.setTextColor(RED);
      display.print("ALERTA DE GÁS!");
    }

    if (digitalRead(PIR_SENSOR)) {
      display.setCursor(20, 140);
      display.setTextColor(RED);
      display.print("Movimento detectado!");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void vTaskReadMPU6050(void *pvParameters) {
  int16_t ax, ay, az, gx, gy, gz;
  for (;;) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("Acc: "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.println(az);
    Serial.print("Gyro: "); Serial.print(gx); Serial.print(", "); Serial.print(gy); Serial.print(", "); Serial.println(gz);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}