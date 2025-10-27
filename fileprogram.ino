/*
 NAMA LENGKAP : Dave Benaya Willianto
 NIM          : 25/569826/TK/64173

 Deskripsi: Program yang dibuat ini berfungsi untuk mengendalikan secara rotal 5 servo motor (S1-S5) berdasarkan data yang didapatkan program dari MPU6050 (Roll, Pitch, Yaw) dan PIR yang juga terhubung pada platform ESP32.
 */


#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050_light.h>

Servo s1, s2, s3, s4, s5;
MPU6050 mpu(Wire);

const int PIR = 13;

// Posisi awal servo
const int INIT_POS = 90;

// Posisi bebas  PIR
const int PIR_POS = 45;

float yawAngle = 0; // sudut yaw yg sudah terinterasi
unsigned long lastTime;

// Variabel Yaw non-blocking
unsigned long yawResetTimer = 0;
bool isYawResetting = false;

void moveAll(int pos) {
  pos = constrain(pos, 0, 180);
  s1.write(pos);
  s2.write(pos);
  s3.write(pos);
  s4.write(pos);
  s5.write(pos);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("status MPU6050: "));
  Serial.println(status);
  while(status != 0){ } // loop jika misal MPU6050 tidak terdeteksi
  
  Serial.println(F("menghitung offset gyro, janhan diapa-apain..."));
  mpu.calcGyroOffsets(); // Hitung offset gyro
  Serial.println(F("Selesai."));


  pinMode(PIR, INPUT);

  // =============================
  //   LABEL POSISI SERVO
  // =============================
  // s1 = Roll kiri
  // s2 = Roll kanan
  // s3 = Pitch kiri
  // s4 = Pitch kanan
  // s5 = Yaw

  s1.attach(18);
  s2.attach(19);
  s3.attach(23);
  s4.attach(25);
  s5.attach(26);

  moveAll(INIT_POS);
  delay(500);

  lastTime = millis();
}

void loop() {
  mpu.update();

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; //  loop dalam detik
  lastTime = currentTime;

  float roll  = mpu.getAngleX();  // rotasi sumbu X
  float pitch = mpu.getAngleY();  // rotasi sumbu Y
  float gyroZ = mpu.getGyroZ();   // speed  rotasi sumbu Z

  // Integrasi dari gyro untuk yaw
  // Hanya akan integrasi jika tidak dalam keadaan proses reset
  if (!isYawResetting) {
    yawAngle += gyroZ * dt;
  }

  // Pembatasan dari nilai supaya servo tidak over
  roll  = constrain(roll,  -90, 90);
  pitch = constrain(pitch, -90, 90);
  
  // biarkan yawAngle ini ada di luar -90/90 supaya visa integrasi, 
  // tetapi constrain nilai tersebut saat menuliskan ke servo
  float constrainedYaw = constrain(yawAngle, -90, 90);


  // =============================
  //   PIR MODE (Prioritas Utama)
  // =============================
  // Cek PIR. Dan jika nanti, aktif, akan diabaikan MPU dan melakukan aksi PIR.
  if (digitalRead(PIR) == HIGH) {
    moveAll(PIR_POS); // semua servo akan ke posisi bebas
    delay(500);       
    moveAll(INIT_POS); // kembali ke posisi awal
    
    // Reset juga status dari yaw jikalau misal PIR terdeteksi
    yawAngle = 0;
    isYawResetting = false;
    
    delay(500); // Beri jeda sebentar setelah mode PIR selesai
    lastTime = millis(); // Reset timer dt
    return; // Lewati  loop yang tersisa agar tidak bisa membaca MPU
  }


  // =============================
  //   APPLY ROTASI KE SERVO
  // =============================


  s1.write(INIT_POS - roll);
  s2.write(INIT_POS + roll); 


  s3.write(INIT_POS + pitch);
  s4.write(INIT_POS + pitch);

  s5.write(INIT_POS + constrainedYaw);

  // Cek apa rotasiv tersebut dari Z (gyroZ) sudah stop (mendekati 0)
  // Dan posisi yaw sedang tidak di tengah
  // DAN tidak sedang dalam prosesi reset
  if (abs(gyroZ) < 1.0 && abs(yawAngle) > 1.0 && !isYawResetting) {
    // Rotasi artinya telah berhenti, mtimer satu detik akan dimulai
    isYawResetting = true;
    yawResetTimer = millis();
  }

  // If timer 1 detik sudah selesai
  if (isYawResetting && (millis() - yawResetTimer > 1000)) {
    s5.write(INIT_POS); // akan kembali ke posisi awal
    yawAngle = 0;       // akan reset integrasi yaw
    isYawResetting = false; //  proses reset selesai
  }
  
  delay(15); // delay jeda yang singkat agar loop tidak terlalu cepat
}