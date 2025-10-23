#include "esp_now.h"
#include "WiFi.h"

typedef struct struct_message {
  int id;
  float x;
  float y;
  float z;
} struct_message;

struct_message boardsStruct[9];

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  struct_message receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  int boardIndex = receivedData.id - 1;
  if (boardIndex >= 0 && boardIndex < 8) {
    boardsStruct[boardIndex].id = receivedData.id;
    boardsStruct[boardIndex].x = receivedData.x;
    boardsStruct[boardIndex].y = receivedData.y;
    boardsStruct[boardIndex].z = receivedData.z;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error de inicializacion del protocolo ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  printData();
  delay(10);
}

void printData() {
  // Enviar los datos en formato CSV (Comma Separated Values)
  for (int i = 0; i <= 8; i++) {
    if (boardsStruct[i].id >= 1 && boardsStruct[i].id <= 8) {
      // Si es una placa válida, enviar los datos en formato CSV
      Serial.print(boardsStruct[i].x);
      Serial.print(",");
      Serial.print(boardsStruct[i].y);
      Serial.print(",");
      Serial.print(boardsStruct[i].z);
      Serial.print(",");
      Serial.print(boardsStruct[i].id);
      
      Serial.print("\t"); // Agregar un tabulador como separador
      
    }
  }
  Serial.println(); // Imprimir una nueva línea al final de cada conjunto de datos
}
