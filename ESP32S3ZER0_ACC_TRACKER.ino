/*
  https://www.waveshare.com/product/arduino/boards-kits/esp32-s3/esp32-s3-zero.htm?___SID=U

  Board: ESP32S3 Dev Module
  Flash Size: 16MB(128Mb)
  Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
  PSRAM: OPI PSRAM
  10:20:BA:31:43:94
*/

#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

//----------------------------------------------------------------
// MAIN DEF
//----------------------------------------------------------------

#define DEVICE_TRACKER 1  // MPU6500 | yaw/pitch/roll | ESP-NOW TX | lipo = A0:F2:62:F1:4C:68

#define DEVICE_DONGLE  2  // USB | ESP-NOW RX | OpenTrack interface A0:F2:62:F1:B3:24

#define DEVICE_TYPE DEVICE_DONGLE

//----------------------------------------------------------------
// ESP NOW
//----------------------------------------------------------------
typedef struct {
  int x;
  int y;
  int z;
  int yaw;
  int pitch;
  int roll;
} EspNowPacket;

uint8_t peerMAC[] = {0xA0, 0xF2, 0x62, 0xF1, 0xB3, 0x24}; // ESP PRIMAC

volatile EspNowPacket rxPacket;
volatile bool rxNewData = false;

//----------------------------------------------------------------
// WS-LED SETTINGS 
//----------------------------------------------------------------
#define NUMPIXELS 1
#define LED 48
Adafruit_NeoPixel pixels(NUMPIXELS, LED, NEO_GRB + NEO_KHZ800);

//----------------------------------------------------------------
// MPU SETTINGS
//----------------------------------------------------------------
#define MPU_SDA 11
#define MPU_SCL 12
#define CALIBRATION_TIME_MS 9000
#define DEADZONE_DEG 100 // deg*100
#define LOOP_HZ 200

#define MPU_ADDR         0x68
#define REG_SMPLRT_DIV   0x19 //Sample Rate Divider
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_PWR_MGMT_1   0x6B
struct MpuData{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};
MpuData mpu;

struct MpuCalibration
{
  int16_t ax_offset;
  int16_t ay_offset;
  int16_t az_offset;
  int16_t gx_offset;
  int16_t gy_offset;
  int16_t gz_offset;
};
MpuCalibration mpu_offset;

//----------------------------------------------------------------
// OPENTRACK
//----------------------------------------------------------------
#pragma pack(push, 1)
struct HatirePacket {
  uint8_t begin[2];   // 2 bajty namiesto int16_t
  uint16_t counter;   // 2 bajty
  float gyro[3];      // 12 bajtov
  float acc[3];       // 12 bajtov
  uint8_t end[2];     // 2 bajty namiesto int16_t
};
#pragma pack(pop)
HatirePacket hat;

//----------------------------------------------------------------
// ADC SETTINGS LIPO
//----------------------------------------------------------------
const int ADC_PIN = 1;
const float VCC = 3.305f;
const float R_FIXED = 1494.0f;     // 1.5k

//----------------------------------------------------------------
// ESP_NOW - Callback
//----------------------------------------------------------------
#if DEVICE_TYPE == DEVICE_DONGLE
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  // MAC odosielateľa
  //const uint8_t *mac = info->src_addr;

  if (len != sizeof(EspNowPacket)) {
    //Serial.println("[-] Wrong packet size");
    return;
  }

  /*
  EspNowPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  Serial.printf("[RX] from %02X:%02X:%02X:%02X:%02X:%02X -> a=%ld b=%ld\n",
                mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5],
                pkt.a, pkt.b);
  */
  
  memcpy((void*)&rxPacket, data, sizeof(EspNowPacket));
  rxNewData = true;
}
#endif

//----------------------------------------------------------------
bool mpu_calibrate_gyro(uint16_t samples = 500);
//----------------------------------------------------------------

void setup() {

  delay(250);
  pixels.begin();
  pixels.clear();
  Serial.begin(115200);

  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 55, 1));
  pixels.show();

  #if DEVICE_TYPE == DEVICE_TRACKER
  espnow_init_tracker();
  Wire.begin(MPU_SDA, MPU_SCL);
  if(mpu_init())
    mpu_calibrate_gyro(1000);
  #elif DEVICE_TYPE == DEVICE_DONGLE
  espnow_init_dongle();
  hatire_init();
  //Serial.print("Hat size: ");
  //Serial.println(sizeof(hat));
  pixels.setPixelColor(0, pixels.Color(0, 0, 55));
  pixels.show();
  delay(500);
  //sim_opentrack();
  #endif
}

int yaw = 0;
int pitch = 0;
int roll = 0;

unsigned long last_time = 0;

void loop() {

  #if DEVICE_TYPE == DEVICE_TRACKER
  //send_int(x, y, z, yaw, pitch, roll);
  unsigned long current_time = millis();
  int dt_ms = (int)(current_time - last_time);

  if (dt_ms >= 10) {
    last_time = current_time;

    if (mpu_read_motion()) {
      // 1. Gyroskop: Odčítanie offsetu
      int pure_gyro_x = mpu.gx - mpu_offset.gx_offset;
      int pure_gyro_y = mpu.gy - mpu_offset.gy_offset;
      int pure_gyro_z = mpu.gz - mpu_offset.gz_offset;

      // ---- MŔTVA ZÓNA (DEADZONE) ----
      // Ak je surová hodnota v pokoji malá (šum), natvrdo ju vynulujeme.
      // Hodnotu 15 môžete upraviť (ak Yaw stále driftuje, dajte 20 alebo 25).
      if (pure_gyro_z > -10 && pure_gyro_z < 10) pure_gyro_z = 0;
      // Voliteľne môžete pridať deadzone aj pre Pitch/Roll, ak jemne kmitajú:
      if (pure_gyro_x > -10 && pure_gyro_x < 10) pure_gyro_x = 0;
      if (pure_gyro_y > -10 && pure_gyro_y < 10) pure_gyro_y = 0;
      // -------------------------------

      // Prepočet na (°/s * 100)
      int gyro_x_rate = (pure_gyro_x * 100) / 131;
      int gyro_y_rate = (pure_gyro_y * 100) / 131;
      int gyro_z_rate = (pure_gyro_z * 100) / 131;
      // Zmena uhla za tento časový krok
      int delta_pitch = (gyro_x_rate * dt_ms) / 1000;
      int delta_roll  = (gyro_y_rate * dt_ms) / 1000;
      int delta_yaw   = (gyro_z_rate * dt_ms) / 1000;
      // 2. Akcelerometer: Výpočet uhlov (stupne * 100)
      int acc_pitch = (int)(atan2((float)mpu.ay, sqrt((float)mpu.ax * mpu.ax + (float)mpu.az * mpu.az)) * 5729.57795);
      int acc_roll  = (int)(atan2(-(float)mpu.ax, (float)mpu.az) * 5729.57795);
      // 3. Komplementárny filter (98% gyro / 2% acc)
      pitch = (98 * (pitch + delta_pitch) + 2 * acc_pitch) / 100;
      roll  = (98 * (roll + delta_roll) + 2 * acc_roll) / 100;
      // Yaw iba integrujeme z gyroskopu (vďaka offsetu už nebude driftovať tak rýchlo)
      yaw += delta_yaw;
      // 4. Výpis do vašej aplikácie
      Serial.print(yaw);   Serial.print(";");
      Serial.print(pitch); Serial.print(";");
      Serial.print(roll);  Serial.print(";");
      Serial.println(current_time);

      send_int(0,0,0,yaw,pitch,roll);

    }
  }
  #elif DEVICE_TYPE == DEVICE_DONGLE
  if (rxNewData) {
    rxNewData = false;
    static float t = 0.0f;
    hat.gyro[0] = rxPacket.yaw/100.00;  // Yaw
    hat.gyro[1] = rxPacket.roll/100.00;  // Pitch
    hat.gyro[2] = rxPacket.pitch/100.00;  // Roll

    hat.acc[0] = 0.0f;
    hat.acc[1] = 0.0f;
    hat.acc[2] = 1.0f;

    Serial.write((uint8_t*)&hat, sizeof(hat));
    Serial.flush();

    hat.counter++;
    if (hat.counter > 255) {
      hat.counter = 0;
    }
    t += 0.02f;
    delay(10); // 100 Hz
  }
  #endif

  delay(1);

}


//----------------------------------------------------------------
// DEVICE_DONGLE
//----------------------------------------------------------------
#if DEVICE_TYPE == DEVICE_DONGLE

void espnow_init_dongle() {

  //uint8_t defMAC[] = {0xAC, 0xEB, 0xE6, 0x1E, 0x1E, 0x68}; //AC:EB:E6:1E:1E:68
  WiFi.mode(WIFI_STA); // povinné
  //esp_wifi_set_mac(WIFI_IF_STA, defMAC);


  if (esp_now_init() != ESP_OK) {
    Serial.println("[-] ESP-NOW init FAILED");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  // MAC pre info
  Serial.println(WiFi.macAddress());
  uint8_t mac[6];
  WiFi.macAddress(mac);
  delay(100);
  Serial.println("[+] ESP-NOW RX ready");
  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5]);
}

void hatire_init() {
  // Explicitne definujeme bajty za sebou, endianita ich neotočí
  hat.begin[0] = 0xAA;
  hat.begin[1] = 0xAA;
  
  hat.end[0]   = 0x55;
  hat.end[1]   = 0x55;
  
  hat.counter  = 0;
}

void hatire_send_test(){
  static float t = 0.0f;

  hat.gyro[0] = sin(t) * 30.0f;  // Yaw
  hat.gyro[1] = sin(t * 0.7f) * 20.0f;  // Pitch
  hat.gyro[2] = sin(t * 1.2f) * 10.0f;  // Roll

  hat.acc[0] = 0.0f;
  hat.acc[1] = 0.0f;
  hat.acc[2] = 1.0f;

  Serial.write((uint8_t*)&hat, sizeof(hat));
  Serial.flush();

  hat.counter++;
  if (hat.counter > 255) {
    hat.counter = 0;
  }
  t += 0.02f;


}

void sim_opentrack(){
  while(true){

    while (Serial.available() > 0) {
      char c = Serial.read(); 
      // Hatire protokol niekedy posiela príkazy (napr. 'C' pre kalibráciu).
      // Ak chcete, môžete tu neskôr reagovať na stredový reset z OpenTracku.
    }

    hatire_send_test();
    delay(10); // 100 Hz
  }
}
#endif

//----------------------------------------------------------------
// DEVICE_TRACKER
//----------------------------------------------------------------
#if DEVICE_TYPE == DEVICE_TRACKER

void espnow_init_tracker() {

  WiFi.mode(WIFI_STA); // nutnE pre ESP-NOW

  if (esp_now_init() != ESP_OK) {
    Serial.println("[-] ESP-NOW init FAILED");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[-] Failed to add peer");
    return;
  }

  delay(100);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
              mac[0], mac[1], mac[2],
              mac[3], mac[4], mac[5]);

  Serial.println("[+] ESP-NOW ready");

}

void send_int(int x, int y, int z, int yaw, int pitch, int roll) {
  // Serial.print("[*] Sending... ");
  // Serial.print(x);
  // Serial.print(" : ");
  // Serial.print(y);
  // Serial.print(" : ");
  // Serial.print(z);
  // Serial.print(" \t ");
  // Serial.print(yaw);
  // Serial.print(" : ");
  // Serial.print(pitch);
  // Serial.print(" : ");
  // Serial.println(roll);

  EspNowPacket pkt;
  pkt.x = x;
  pkt.y = y;
  pkt.z = z;
  pkt.yaw = yaw;
  pkt.pitch = pitch;//;
  pkt.roll = roll;//;

  esp_err_t result = esp_now_send(peerMAC, (uint8_t*)&pkt, sizeof(pkt));

  if (result != ESP_OK) {
    Serial.println("[-] Send error");
  }
}

bool mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0){
    Serial.println("[-] MPU INIT FAIL");
    return false;
  }

  delay(100);
  Serial.println("[+] MPU INIT OK");
  return true;
}

bool mpu_read_registers(uint8_t reg, uint8_t* data, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
      return false;

  uint8_t received = Wire.requestFrom(MPU_ADDR, len);

  if (received != len)
      return false;

  for (uint8_t i = 0; i < len; i++)
      data[i] = Wire.read();

  return true;
}

bool mpu_read_motion(){
  uint8_t data[14];

  if (!mpu_read_registers(0x3B, data, 14))
      return false;

  mpu.ax   = (int16_t)((data[0]  << 8) | data[1]);
  mpu.ay   = (int16_t)((data[2]  << 8) | data[3]);
  mpu.az   = (int16_t)((data[4]  << 8) | data[5]);
  mpu.temp = (int16_t)((data[6]  << 8) | data[7]);
  mpu.gx   = (int16_t)((data[8]  << 8) | data[9]);
  mpu.gy   = (int16_t)((data[10] << 8) | data[11]);
  mpu.gz   = (int16_t)((data[12] << 8) | data[13]);

  return true;
}

void mpu_show_data(){
  Serial.print("Acc: ");
  Serial.print(mpu.ax); Serial.print(", ");
  Serial.print(mpu.ay); Serial.print(", ");
  Serial.print(mpu.az);
  
  Serial.print(" | Gyro: ");
  Serial.print(mpu.gx - mpu_offset.gx_offset); Serial.print(", ");
  Serial.print(mpu.gy - mpu_offset.gy_offset); Serial.print(", ");
  Serial.println(mpu.gz - mpu_offset.gz_offset);
}

bool mpu_calibrate_gyro(uint16_t samples){
  int sum_gx = 0;
  int sum_gy = 0;
  int sum_gz = 0;
  mpu_offset.gx_offset = 0;
  mpu_offset.gy_offset = 0;
  mpu_offset.gz_offset = 0;

  for (int i = 0; i < 5; i++){
    pixels.setPixelColor(0, pixels.Color(222, 0, 0));
    pixels.show();
    delay(250);
    pixels.setPixelColor(0, pixels.Color(55, 0, 0));
    pixels.show();
    delay(250);
  }


  for (uint16_t i = 0; i < samples; i++){
      if (!mpu_read_motion())
          return false;

      sum_gx += mpu.gx;
      sum_gy += mpu.gy;
      sum_gz += mpu.gz;

      delay(2);
  }

  mpu_offset.gx_offset = sum_gx / samples;
  mpu_offset.gy_offset = sum_gy / samples;
  mpu_offset.gz_offset = sum_gz / samples;

  for (int i = 0; i < 5; i++){
    pixels.setPixelColor(0, pixels.Color(0, 222, 0));
    pixels.show();
    delay(250);
    pixels.setPixelColor(0, pixels.Color(0, 55, 0));
    pixels.show();
    delay(250);
  }

  return true;
}

float mpu_get_temp(){
  return ((float)mpu.temp / 333.87f) + 21.0f;
}

bool mpu_detect() {

  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() == 0) {
    Serial.println("[+] MPU FOUND");
    return true;
  }

  Serial.println("[-] MPU NOT FOUND");
  return false;
}

#endif



