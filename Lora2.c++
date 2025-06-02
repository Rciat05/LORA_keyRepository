#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <SparkFun_MMA8452Q.h>

MMA8452Q accel;

float velZ = 0;
float altZ = 0;
unsigned long lastTime = 0;
float t = 0;
float az_offset = 0;

const int offsetSamples = 100;
const float threshold = 0.2;

void setup() {
  Serial.begin(9600);
  delay(1000); // Ensure Serial is ready

  Serial.println("[BOOT] Initializing I2C and sensor...");
  Wire.begin();

  if (!accel.begin()) {
    Serial.println("[ERROR] MMA8452Q not detected.");
    while (1);
  }
  Serial.println("[OK] MMA8452Q detected");

  accel.setScale(SCALE_2G);
  accel.setDataRate(ODR_100);
  delay(500);
  calibrarSensor();

  Serial.println("[BOOT] Initializing LoRa...");
  LoRa.setPins(10, 9, 2);  // NSS, RESET, DIO0
  if (!LoRa.begin(433E6)) {
    Serial.println("[ERROR] LoRa begin failed");
    while (1);
  }

  LoRa.setTxPower(20);  // Max TX power
  Serial.println("[OK] LoRa initialized at 915 MHz");

  Serial.println("t,ax,ay,az,az_m_s2,net_a,velZ,altZ");
  lastTime = millis();
}

void loop() {
  if (accel.available()) {
    accel.read();

    float ax = accel.getCalculatedX();
    float ay = accel.getCalculatedY();
    float az = accel.getCalculatedZ();
    float az_m_s2 = az * 9.81;
    float net_a = az_m_s2 - az_offset;

    if (abs(net_a) < threshold) {
      net_a = 0;
      velZ = 0;
    }

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    t += dt;

    velZ += net_a * dt;
    altZ += velZ * dt;

    String data = String(t, 3) + "," +
                  String(ax, 3) + "," +
                  String(ay, 3) + "," +
                  String(az, 3) + "," +
                  String(az_m_s2, 2) + "," +
                  String(net_a, 2) + "," +
                  String(velZ, 2) + "," +
                  String(altZ, 2);

    Serial.println("[DATA] " + data);

    int result = LoRa.beginPacket();
    if (result == 1) {
      Serial.println("[TX] Packet started");
      LoRa.print(data);
      LoRa.endPacket();  // Will block until complete
      Serial.println("[TX] Packet sent");
    } else {
      Serial.println("[WARN] Failed to begin LoRa packet");
    }
  }

  delay(200); // Limit to 5 packets/sec
}

void calibrarSensor() {
  Serial.println("[CAL] Calibrating Z-axis offset...");
  float suma = 0;
  int contador = 0;

  while (contador < offsetSamples) {
    if (accel.available()) {
      accel.read();
      suma += accel.getCalculatedZ() * 9.81;
      contador++;
      delay(10);
    }
  }

  az_offset = suma / offsetSamples;
  Serial.print("[CAL] Z offset: ");
  Serial.print(az_offset, 4);
  Serial.println(" m/s²");
}