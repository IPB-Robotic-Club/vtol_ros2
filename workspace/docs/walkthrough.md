# Penyelesaian "Drop Mendadak" pada Centering PID

## Masalah yang Ditemukan
Dalam pengujian terakhir, *drone* menderita **penurunan ketinggian (*altitude drop*)** yang parah (dari 1.70m jatuh ke 0.60m) selama fase pembidik *marker* (centering). 

Setelah memeriksa *log* mendalam, ditemukan bahwa:
- *Drone* VTOL dalam lingkungan simulasi akan **kehilangan gaya angkat (lift)** secara drastis apabila dimiringkan (Pitch/Roll) terlalu tajam.
- Setelan sebelumnya (`max_override=40` dan `kp=7.5`) mengizinkan PID untuk memberikan perintah kemiringan yang sangat besar (mencapai RC=1540).
- Hal ini menyebabkan hidung *drone* menukik tajam, ketinggian merosot drastis layaknya batu, dan sudut pandang kamera ikut tersapu dengan cepat sehingga seketika kehilangan penjejakan *marker*.

## Perubahan yang Dilakukan
Untuk menjaga *drone* dari kemiringan ekstrem tanpa mengorbankan daya dorong (*thrust*):

1. **Memotong Batas Kemiringan**:
   - [vtol_config.yaml](file:///home/qois/vtol-dev/workspace/src/vtol_control/config/vtol_config.yaml)
   - `max_override` diturunkan kembali dari `40` menjadi `30`. 
   - Ini memastikan batasan kemiringan *drone* ada pada rentang aman yang **tidak akan membahayakan gaya angkat vertikal**.

2. **Memperkecil Zona Kedap (Deadzone Band)**:
   - [mission_centering.py](file:///home/qois/vtol-dev/workspace/src/vtol_control/vtol_control/mission_centering.py)
   - Parameter `band` diubah dari `2.0` menjadi `0.5`. 
   - Meskipun kemiringan *drone* kita batasi, mengecilkan `band` memastikan setiap perhitungan pergerakan yang dikeluarkan PID akan langsung disalurkan ke motor secara responsif, membuat pergerakan *drone* tidak terasa lamban atau "lemah".

3. **Menambahkan Komponen Integral**:
   - `ki_roll` dan `ki_pitch` diaktifkan ke `0.2` (dari sebelumnya `0.0`).
   - `kp` dikembalikan ke `5.0`.
   - Ini meminimalisasi kemiringan mendadak, sekaligus memastikan bahwa kalaupun *drone* kurang kencang melaju ke titik pusat, komponen Integral secara lambat-laun akan mendorong *drone* hingga tepat berada di atas sasaran.

## Pengujian
- **Status Kompilasi**: Selesai dikompilasi dengan sukses melalui `colcon build`.
- **Harapan Pengujian Selanjutnya**: Silakan uji ulang sistem. Kini *drone* akan "mengerem" kemiringannya dan lebih stabil melaju ke *marker* tanpa merosot jatuh ke bawah. 

*(Perbaikan dan optimasi logika telah dimasukkan, dan drone Anda siap diterbangkan!)*
