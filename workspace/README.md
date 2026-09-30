# DJI Tello ROS2 — Hand Vision Workspace

> **Platform:** Ubuntu 22.04 · ROS2 Humble  
> **Hardware:** DJI Tello (SDK v2.0) · WiFi: `TELLO-9B3FD0`

Workspace ini menggabungkan driver Tello dengan kontrol tangan berbasis MediaPipe.

---

## Struktur Workspace

```
workspace/
├── src/
│   ├── tello_msg/        # Custom messages (TelloStatus, dll)
│   ├── tello_driver/     # Driver utama UDP + video stream
│   ├── tello_control/    # Kontrol keyboard via OpenCV window
│   └── hand_vision/      # Kontrol tangan (MediaPipe C++ + solvePnP)
│       ├── src/
│       │   └── hand_control_node.cpp
│       ├── config/
│       │   └── hand_control.yaml
│       └── launch/
│           └── hand_control.launch.xml
├── build/
├── install/
└── src/launch.py         # Launch semua sekaligus
```

---

## Build

```bash
cd ~/Documents/tello/tello_ws/src/tello-ros2/workspace
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build ulang hanya `hand_vision`:
```bash
colcon build --packages-select hand_vision --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Cara Menjalankan

### 1. Koneksi WiFi

Hubungkan laptop ke WiFi Tello sebelum menjalankan apapun:
```
SSID : TELLO-9B3FD0
IP Tello  : 192.168.10.1
```

### 2. Source workspace

Lakukan ini di **setiap terminal** yang digunakan:
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/tello/tello_ws/src/tello-ros2/workspace/install/setup.bash
```

---

## Opsi A — Jalankan Semua Sekaligus

Satu perintah untuk menjalankan driver + hand_control + keyboard control:

```bash
ros2 launch ~/Documents/tello/tello_ws/src/tello-ros2/workspace/src/launch.py
```

---

## Opsi B — Jalankan Per Terminal

### Terminal 1 — Tello Driver

```bash
ros2 launch tello_driver tello.launch.xml
```

Driver akan publish `/image_raw` (video drone) dan subscribe ke `/takeoff`, `/land`, `/control`.

### Terminal 2 — Hand Control

```bash
ros2 launch hand_vision hand_control.launch.xml
```

Window **"Tello Hand Control"** akan muncul.  
- Selama driver belum aktif: tampil `Waiting for /image_raw ...`
- Setelah driver aktif: tampil video langsung dari kamera drone

---

## Kontrol Hand Vision

### Keyboard (window harus fokus)

| Tombol | Aksi | Kondisi |
|--------|------|---------|
| `Q` | Takeoff | State: IDLE |
| `E` | Land | State: TRACKING |
| `ESC` | Emergency land | Kapan saja |

### Alur State

```
IDLE → (tekan Q) → TAKING_OFF → (tunggu ~4.5s) → TRACKING → (tekan E) → LANDING → IDLE
```

Saat **TRACKING**, drone otomatis mengikuti tangan yang terdeteksi kamera:
- **LR / UD** — drone bergerak agar tangan tetap di tengah frame
- **FB** — drone menjaga jarak dengan tangan (default: 0.30m)

### Debug Image

Frame dengan overlay landmark dan tracking box dipublish ke:
```
/hand_vision/debug_image
```

Bisa dilihat dengan:
```bash
ros2 run rqt_image_view rqt_image_view /hand_vision/debug_image
```

---

## Konfigurasi (`config/hand_control.yaml`)

Parameter utama yang sering diubah:

| Parameter | Default | Keterangan |
|-----------|---------|------------|
| `target_distance` | `0.30` | Jarak target tangan dari drone (meter) |
| `box_px` | `130` | Ukuran kotak tracking di tengah frame (pixel) |
| `kp_lateral` | `35.0` | Kecepatan tracking LR/UD |
| `kp_depth` | `0.0` | Kecepatan tracking maju-mundur (`0` = nonaktif) |
| `lr_sign` | `-1` | Balik ke `1` jika drone bergerak berlawanan arah |
| `show_window` | `true` | Tampilkan window OpenCV |

---

## ROS2 Topics

| Topic | Tipe | Arah |
|-------|------|------|
| `/image_raw` | `sensor_msgs/Image` | Subscribe (dari driver) |
| `/takeoff` | `std_msgs/Empty` | Publish |
| `/land` | `std_msgs/Empty` | Publish |
| `/control` | `geometry_msgs/Twist` | Publish (tracking RC) |
| `/hand_vision/debug_image` | `sensor_msgs/Image` | Publish (debug) |

---

## Troubleshooting

**Window tidak muncul**
- Pastikan `DISPLAY` environment variable ter-set: `echo $DISPLAY` harus ada isinya (misal `:0`)
- Jalankan dari terminal dengan sesi grafis aktif, bukan SSH tanpa X forwarding

**`Waiting for /image_raw ...` tidak hilang**
- Tello driver belum jalan, atau
- Laptop belum konek ke WiFi `TELLO-9B3FD0`

**Drone bergerak berlawanan arah**
- Edit `config/hand_control.yaml`, ubah `lr_sign: -1` → `lr_sign: 1`

**Takeoff tidak merespons tombol Q**
- Pastikan window `"Tello Hand Control"` sedang fokus (klik window-nya dulu)
