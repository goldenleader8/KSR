<h1 align="center">UAJY ROBOTICS</h1>
<p align="center">
  Repository ini berisi <b>source code</b> proyek robotika yang dikembangkan oleh <b>KSR (Kelompok Studi Robotika) UAJY</b>.<br>
  Mencakup implementasi <b>Forward Kinematics</b>, <b>Inverse Kinematics</b>, simulasi, dan pengembangan sistem kontrol robot melalui ROS.
</p>
<p align="center">
  <img src="https://img.shields.io/badge/Language-C%20%7C%20C%2B%2B-blue?style=for-the-badge&logo=c" />
  <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Windows-lightgrey?style=for-the-badge" />
  <img src="https://img.shields.io/badge/University-UAJY-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" />
</p>

---

## Goals

Repository ini bertujuan untuk:
- Menjadi platform **riset dan eksperimen** algoritma robotika KSR UAJY
- Mengimplementasikan **Forward Kinematics** menggunakan konvensi Denavit-Hartenberg (D-H)
- Mengimplementasikan **Inverse Kinematics** robot lengan 3 DOF secara analitik
- Menjadi referensi belajar **kinematika robot** bagi anggota KSR

---

## Struktur Repository

```
KSR/
├── KSR_ARM_BIRU_FK.cpp  # Program Forward Kinematics Robot Arm 3 DOF (D-H)
├── KSR_ARM_BIRU_IK.c    # Program Inverse Kinematics Robot Arm 3 DOF
├── LICENSE
└── README.md
```

---

## Parameter Robot: **"BLUE"**

Parameter berikut diukur langsung dari model CAD (Autodesk Fusion 360):

| Parameter | Simbol | Nilai (paper) | Nilai (CAD aktual) | Keterangan |
|:--|:--|:--|:--|:--|
| **Link length 1** | a1 | 0 cm | 0 cm | Base — rotasi murni, tidak ada offset horizontal |
| **Link length 2** | a2 | 10.0 cm | **12.441 cm** | Pusat shoulder (M2) → pusat elbow (M3) |
| **Link length 3** | a3 | 10.0 cm | **10.688 cm** | Pusat elbow (M3) → pusat wrist (M4) |
| **Joint offset 1** | d1 | 0 cm | **10.296 cm** | Tinggi ground → pusat shoulder (M2) |
| **Joint offset 2** | d2 | 0 cm | 0 cm | — |
| **Joint offset 3** | d3 | 0 cm | 0 cm | — |
| **Twist angle 1** | α1 | 90° | 90° | — |
| **Twist angle 2** | α2 | 0° | 0° | — |
| **Twist angle 3** | α3 | 0° | 0° | — |
| **DOF** | — | 3 | 3 | — |

> **Catatan:** Nilai pada paper (Princewill et al., *Future Technology* Vol.03 Issue 01, 2024) menggunakan dimensi yang dibulatkan dan tidak menyertakan `d1`, sehingga terdapat ketidakcocokan antara tabel D-H dan posisi end-effector yang diklaim. Nilai CAD aktual di atas menghasilkan hasil perhitungan yang lebih akurat.

---

## FORWARD KINEMATICS: **`Forward_Kinematics_ARM_3DOF.cpp`**

### Deskripsi

Program **`KSR_ARM_BIRU_FK.cpp`** adalah implementasi **Forward Kinematics (FK)** menggunakan konvensi **Denavit-Hartenberg (D-H) Standard** untuk robot lengan 3 DOF.

Program ini menghitung posisi end-effector **(x, y, z)** berdasarkan sudut-sudut joint (θ1, θ2, θ3) yang diinputkan pengguna, menggunakan perkalian matriks transformasi homogen 4×4.

### Pendekatan Matematis

Setiap joint direpresentasikan oleh transformation matrix D-H:

$$T_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Posisi end-effector diperoleh dari:

$$T_{total} = T_1 \times T_2 \times T_3$$

### Cara Kompilasi & Menjalankan

```bash
# Kompilasi
g++ Forward_Kinematics_ARM_3DOF.cpp -o Forward_Kinematics_ARM_3DOF -lm

# Jalankan
./Forward_Kinematics_ARM_3DOF
```

### Cara Penggunaan

```
==[ROBOT ARM BIRU]==
  Robot ini memiliki 3 DOF

  ==[LINK 1]==
  Masukkan sudut theta joint ke-1 (-360 s/d 360): 45

  ==[LINK 2]==
  Masukkan sudut theta joint ke-2 (-360 s/d 360): 85

  ==[LINK 3]==
  Masukkan sudut theta joint ke-3 (-360 s/d 360): 110

  ==[POSISI END-EFFECTOR]==
  X = -6.5333 cm
  Y = -6.5333 cm
  Z = 19.4874 cm
```

---

## INVERSE KINEMATICS: **`Inverse_Kinematics_ARM_3DOF.c`**

### Deskripsi

Program **`Inverse_Kinematics_ARM_3DOF.c`** adalah implementasi **Inverse Kinematics (IK)** untuk robot lengan 3 DOF menggunakan bahasa C.

Program ini menghitung sudut-sudut joint (θ1, θ2, θ3) dari sebuah robot lengan berdasarkan posisi end-effector **(x, y, z)** yang diinputkan oleh pengguna.

### Pendekatan Matematis

Program menggunakan pendekatan **analitik geometris** dengan:
- Fungsi `clamp()` untuk memastikan nilai input `acos()` berada dalam range **-1 ≤ x ≤ 1** agar tidak menghasilkan `NaN`
- Perhitungan sudut joint menggunakan hukum cosinus dan trigonometri
- Output berupa sudut dalam **derajat (°)**

### Cara Kompilasi & Menjalankan

```bash
# Kompilasi
gcc Inverse_Kinematics_ARM_3DOF.c -o Inverse_Kinematics_ARM_3DOF -lm

# Jalankan
./Inverse_Kinematics_ARM_3DOF
```

### Cara Penggunaan

```
===[Inverse Kinematics]===
[1] Inverse Kinematics Robot 3 DOF
[0] Exit Program
Pilih Opsi Menu: 1

Masukkan koordinat X: 10.0
Masukkan koordinat Y: 5.0
Masukkan koordinat Z: 8.0

Hasil:
Theta 1: XX.XX°
Theta 2: XX.XX°
Theta 3: XX.XX°
```

---

## Tech Stack

| Komponen | Keterangan |
|:--|:--|
| **Language** | C (C99) / C++ (C++17) |
| **Libraries** | `stdio.h`, `stdlib.h`, `math.h`, `iostream`, `cmath`, `array`, `iomanip`, `vector` |
| **Compiler** | GCC / G++ |
| **OS Support** | Linux / Windows (MinGW) |

---

## Referensi

- Princewill, N.C., Steve, T., Omawumi, O.E. (2024). *Development and implementation of a wireless-controlled robotic arm for lifting applications with 6 DOF*. Future Technology, Vol. 03, Issue 01, pp. 25–31. DOI: 10.55670/fpll.futech.3.1.3
- Corke, P. (2011). *Robotics, Vision and Control: Fundamental Algorithms in MATLAB*. Springer-Verlag.
- Alavandar, S., & Nigam, M. J. (2008). Inverse kinematics solution of 3DOF planar robot using ANFIS. Int. J. of Computers, Communications & Control, 3, 150-155.
- Atique, M. M. U., & Ahad, M. A. R. (2014, May). Inverse Kinematics solution for a 3DOF robotic structure using Denavit-Hartenberg Convention. In 2014 international conference on informatics, electronics & vision (ICIEV) (pp. 1-5). IEEE.
- 

---

## Tim KSR UAJY

Dikembangkan oleh anggota **Kelompok Studi dan Riset Robotika (KSR)**  
Universitas Atma Jaya Yogyakarta (UAJY)

---

## License

Proyek ini dilisensikan di bawah **MIT License** – lihat file [LICENSE](LICENSE) untuk detail lebih lanjut.
