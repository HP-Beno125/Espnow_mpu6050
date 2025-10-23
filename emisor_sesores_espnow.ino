//SENSOR-4_DISPOSITIVO-4_ID=4
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "esp_now.h"
#include "WiFi.h"
#define SDA_PIN 21
#define SCL_PIN 23
MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;

long f_ax, f_ay, f_az;
int16_t p_ax, p_ay, p_az;
long f_gx, f_gy, f_gz;
int16_t p_gx, p_gy, p_gz;
int counter = 0;
int16_t ax_o = 6094, ay_o = 4558, az_o = 9296;
int16_t gx_o = 220, gy_o = 34, gz_o = -66;

//Direcion MAC del receptor - 24:6F:28:81:DE:84
uint8_t broadcastAddress[] = {0x0C, 0x8B, 0x95, 0x75, 0xE2, 0x5C};

//Estructura del mensaje a enviar
typedef struct struct_message {
  int id;
  float x,y,z;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nEstado del ultimo paquete enviado\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Envio exitoso" : "Envio fallido");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  sensor.initialize();

  sensor.setXAccelOffset(ax_o);
  sensor.setYAccelOffset(ay_o);
  sensor.setZAccelOffset(az_o);

  sensor.setXGyroOffset(gx_o);
  sensor.setYGyroOffset(gy_o);
  sensor.setZGyroOffset(gz_o);

  Serial.println("Offsets asignados correctamente");

   WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error de inicializacion del protocolo ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("No se pudo agregar el par");
    return;
  }
}

void loop() {
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Convertir valores a unidades de aceleración (m/s^2) y velocidad angular (grados/s)
  float accelerationX = (float)ax / 16384.0 * 9.81;
  float accelerationY = (float)ay / 16384.0 * 9.81;
  float accelerationZ = (float)az / 16384.0 * 9.81;

  float gyroX = (float)gx / 131.0;
  float gyroY = (float)gy / 131.0;
  float gyroZ = (float)gz / 131.0;

 

   myData.id =7;
  myData.x=accelerationX  ;
  myData.y=  accelerationY;
   myData.z=  accelerationZ;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  

  delay(10); // Esperar un poco entre lecturas
}
