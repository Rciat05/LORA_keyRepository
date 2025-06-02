#include <SPI.h>
#include <LoRa.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("[BOOT] Inicializando LoRa receptor...");
  LoRa.setPins(10, 9, 2); // NSS, RESET, DIO0 (mismos que el transmisor)

  if (!LoRa.begin(433E6)) {
    Serial.println("[ERROR] Falló inicializar LoRa");
    while (1);
  }

  Serial.println("[OK] LoRa iniciado en 433 MHz");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("[RX] Paquete recibido, tamaño ");
    Serial.println(packetSize);

    String received = "";
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      received += c;
    }

    Serial.print("[DATA] ");
    Serial.println(received);
  }
}