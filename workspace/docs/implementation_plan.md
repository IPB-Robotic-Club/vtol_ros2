# Rencana Perbaikan "Drop Mendadak" (Altitude Loss)

Terima kasih atas kesabarannya! Fakta bahwa *drone* Anda mengalami *drop* (jatuh mendadak) setiap kali nilai RC dinaikkan adalah petunjuk emas.

## Diagnosis: Altitude Drop akibat Pitch Agresif
Pada *drone* jenis VTOL (QuadPlane) dalam simulasi ArduPilot, komponen gaya angkat vertikal akan berkurang drastis jika *drone* miring (pitch/roll) terlalu tajam. 
1. Pada iterasi sebelumnya, `RC` mencapai angka `1535` hingga `1540`.
2. Nilai tersebut memerintahkan *drone* untuk miring tajam ke depan/belakang guna mengejar marker.
3. Karena kemiringan ekstrem ini, motor vertikal tidak sanggup mempertahankan gaya angkat (*lift*), sehingga ketinggian *drone* merosot tajam dari **1.70m -> 1.38m -> 1.02m -> 0.60m** dalam waktu kurang dari 2 detik!
4. Akibat jatuh ini, kamera kehilangan area pandang dan *marker* pun hilang dari layar.

## Solusi: Kombinasi Batas Kemiringan & Integral Gain
Untuk membuat pergerakan *drone* kuat merespons marker ("tidak terlalu kurang") namun tidak sampai miring ekstrem hingga jatuh ("tidak drop mendadak"), kita akan melakukan 3 hal:

1. **Batasi Kemiringan Maksimal (Max Override)**: Kita turunkan `max_override` ke angka **30**. Ini menjamin RC tidak akan pernah melebihi batas bahaya (1470 - 1530), sehingga *drone* mustahil menukik tajam dan kehilangan ketinggian.
2. **Kecilkan Zona Kedap (Deadzone Band)**: Karena batas maksimal kita potong, kita harus memastikan *error* kecil langsung ditanggapi dengan cukup bertenaga. Kita ubah `band` pada `apply_smooth_deadzone` dari `2.0` menjadi `0.5`.
3. **Tambahkan Komponen Integral (Ki)**: Kita kembalikan Kp ke `5.0`, tapi kita masukkan nilai `ki = 0.2`. Fungsi `Ki` adalah secara perlahan menambah daya dorong jika *drone* belum juga sampai ke tengah, menjamin *drone* pasti akan konvergen walau kemiringannya dibatasi.

## Proposed Changes

### [MODIFY] src/vtol_control/config/vtol_config.yaml
```yaml
pid_centering:
  kp_roll: 5.0
  ki_roll: 0.2          # Tambahan integral untuk memastikan drone sampai ke pusat
  kd_roll: 8.0
  kp_pitch: 5.0
  ki_pitch: 0.2         # Tambahan integral
  kd_pitch: 8.0
  max_override: 30      # Dibatasi ketat agar drone tidak menukik tajam dan jatuh
```

### [MODIFY] src/vtol_control/vtol_control/mission_centering.py
Mengubah fungsi kompensasi *deadzone* agar *drone* merespons kuat tanpa bantingan:
```python
# Kompensasi deadzone RC dengan smooth transition (band=0.5)
deadzone_bias = 25.0
u_roll = apply_smooth_deadzone(u_roll_raw, deadzone_bias, band=0.5)
u_pitch = apply_smooth_deadzone(u_pitch_raw, deadzone_bias, band=0.5)
```

## Verification Plan
1. Lakukan pengeditan pada kedua fail di atas.
2. Kompilasi ulang *workspace* dengan `colcon build --packages-select vtol_control`.
3. Anda dapat menguji kembali. Seharusnya *drone* kini akan bergerak ke arah *marker* dengan pasti (berkat Ki dan deadzone ketat) tanpa pernah miring ekstrem yang menyebabkannya *drop* ketinggian.
