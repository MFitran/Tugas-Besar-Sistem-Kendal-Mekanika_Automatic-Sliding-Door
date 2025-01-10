# Automatic Sliding Door System

Proyek ini adalah sistem kontrol pintu geser otomatis berbasis Arduino. Sistem ini memanfaatkan sensor VL53L0X untuk mendeteksi posisi pintu, sensor ultrasonik HC-SR04 untuk mendeteksi keberadaan objek, dan motor servo untuk menggerakkan pintu. Kontrol pergerakan pintu dilakukan menggunakan algoritma PID (Proportional-Integral-Derivative).

## Fitur
- Mengontrol pintu geser secara otomatis berdasarkan keberadaan objek.
- Algoritma PID digunakan untuk memastikan pergerakan pintu yang halus dan presisi.
- Dilengkapi mekanisme untuk membuka dan menutup pintu dengan akurasi tinggi.

## Komponen yang Digunakan
- **Arduino Uno**: Sebagai mikrokontroler utama.
- **VL53L0X**: Sensor pengukuran jarak berbasis waktu terbang (time-of-flight).
- **HC-SR04**: Sensor ultrasonik untuk mendeteksi keberadaan objek.
- **Motor Servo**: Untuk menggerakkan pintu.
- **Komponen Pendukung**: Kabel jumper, breadboard, dan catu daya.

## Cara Kerja
1. **Deteksi Objek**  
   Sensor HC-SR04 mendeteksi jarak objek di depan pintu. Jika jarak objek kurang dari 115 mm, pintu akan terbuka; jika tidak, pintu akan menutup.
   
2. **Kontrol Pintu**  
   VL53L0X digunakan untuk membaca posisi pintu secara real-time. Nilai ini dibandingkan dengan setpoint (posisi target) menggunakan algoritma PID, yang menghitung output untuk mengontrol motor servo.

3. **Tuning PID**  
   Konstanta PID digunakan untuk mengontrol respons sistem. Konstanta yang digunakan dapat dituning pada baris **49-51** di dalam kode:  
   ```cpp
   float kp = 0.8;   // Konstanta proporsional
   float ki = 0.015; // Konstanta integral
   float kd = 0.6;   // Konstanta derivatif
