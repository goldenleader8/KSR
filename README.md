<h1 align="center"> UAJY ROBOTICS</h1>

<p align="center">
  Repository ini berisi <b>source code</b> proyek robotika yang dikembangkan oleh <b>KSR (Kelompok Studi Robotika) UAJY</b>.<br>
  Mencakup implementasi <b>Inverse Kinematics</b>, simulasi, dan pengembangan sistem kontrol robot melalui ROS.
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Language-C-blue?style=for-the-badge&logo=c" />
  <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Windows-lightgrey?style=for-the-badge" />
  <img src="https://img.shields.io/badge/University-UAJY-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" />
</p>

---

## Goals

Repository ini bertujuan untuk:
- Menjadi platform **riset dan eksperimen** algoritma robotika KSR UAJY
- Mengimplementasikan **Inverse Kinematics** robot lengan 3 DOF secara analitik
- Menjadi referensi belajar **kinematika robot** bagi anggota KSR

---

## Struktur Repository

```
KSR/
├── KSR_ARM_BIRU_IK.c    # Program Inverse Kinematics Robot Arm 3 DOF
├── LICENSE
└── README.md
```

---

## INVERSE KINEMATICS ROBOT ARM 3 DOF: "BLUE"

### Deskripsi

Program **`KSR_ARM_BIRU_IK.c`** adalah implementasi **Inverse Kinematics (IK)** untuk robot lengan 3 Degree of Freedom (DOF) menggunakan bahasa C.

Program ini menghitung sudut-sudut joint (θ1, θ2, θ3) dari sebuah robot lengan berdasarkan posisi end-effector **(x, y, z)** yang diinputkan oleh pengguna.

### Parameter Robot

| Parameter | Nilai |
|:--|:--|
| **Link 1 (a1)** | 9.087 cm |
| **Link 2 (a2)** | 12.441 cm |
| **Link 3 (a3)** | 10.688 cm |
| **DOF** | 3 |

### Pendekatan Matematis

Program menggunakan pendekatan **analitik geometris** dengan:
- Fungsi `clamp()` untuk memastikan nilai input `acos()` berada dalam range **-1 <= x <= 1** agar tidak menghasilkan `NaN`
- Perhitungan sudut joint menggunakan hukum cosinus dan trigonometri
- Output berupa sudut dalam **derajat (°)**

### Cara Kompilasi & Menjalankan

```bash
# Kompilasi
gcc KSR_ARM_BIRU_IK.c -o KSR_ARM_BIRU_IK -lm

# Jalankan
./KSR_ARM_BIRU_IK
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
| **Language** | C (C99) |
| **Libraries** | `stdio.h`, `stdlib.h`, `math.h` |
| **Compiler** | GCC |
| **OS Support** | Linux / Windows (MinGW) |

---

## Tim KSR UAJY

Dikembangkan oleh anggota **Kelompok Studi dan Riset Robotika (KSR)**  
Universitas Atma Jaya Yogyakarta (UAJY)

---

## License

Proyek ini dilisensikan di bawah **MIT License** – lihat file [LICENSE](LICENSE) untuk detail lebih lanjut.
