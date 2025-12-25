# REBOT AGV (Automated Guided Vehicle) 
<img src="https://i.ibb.co.com/8LccPJ9q/image.png" alt="image" border="0" width=300>
AGV (Autonomous Guided Vehicle) beroda omnidirectional untuk riset dan pembelajaran.

## Fitur Utama
- Roda mecanum 4 buah sehingga robot bisa maju, mundur, geser kanan/kiri, dan rotasi di tempat.
- Rangka utama dari plat metal dengan cover atas tertutup, sehingga elektronik di dalam lebih aman dan rapi.
- Tersedia mounting di bagian atas untuk sensor utama seperti LiDAR, plus beberapa bukaan untuk tombol, konektor, dan indikator.

## Komponen Hardware
Sesuaikan dengan versi yang kamu pakai; berikut template yang umum:
- Main controller: ESP32 (pengembangan selanjutnya Raspberry Pi) sebagai otak utama robot.
- Motor: 4x stepper motor NEMA23 dengan roda omnidirectional.
- Motor driver: 4x TB6600.
- Sensor posisi: magnetic encoder pada tiap motor untuk perhitungan kecepatan dan posisi.
- Sensor navigasi utama:
  - RPLIDAR C1 2D di bagian atas untuk pemetaan dan obstacle detection
- Catu daya: Baterai + BMS + modul step-down  untuk 5 V dan 12 V.
- Emergency switch dan saklar utama untuk keamanan saat pengujian.

## Demo 
<img src="https://i.ibb.co.com/rGs1Dk8Z/Screenshot-2025-12-25-195426.png" alt="Screenshot-2025-12-25-195426" border="0" width=500>

