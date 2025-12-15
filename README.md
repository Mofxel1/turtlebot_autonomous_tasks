# TurtleBot3 Otonom Görevler Projesi (ROS 1)

Bu proje, "Robotik Laboratuvarı" dersi kapsamında verilen 5 aşamalı otonom sürüş ve navigasyon görevlerini içermektedir. Proje **Jetson Nano** üzerinde **ROS 1** ve **TurtleBot3 Burger** kullanılarak geliştirilmiştir.

**Öğrenciler:**
* Orhan Yıldız
* Alperen Er
* Muammer Sönmez

**Son Teslim Tarihi:** 19.12.2025

## 🎯 Görev Listesi

### [Görev 1] Basit Otonom Sürüş ve Engelden Kaçma
Lidar verisi kullanılarak oluşturulan bir Durum Makinesi (FSM) ile robotun engellere çarpmadan parkuru tamamlaması sağlanmıştır.
* **Yöntem:** `/scan` verisi analizi (Yüzdelik dilim tarama).
* **Durumlar:** İLERİ, ENGEL_VAR (Dönüş).

### [Görev 2] SLAM ile Haritalama
Laboratuvar ortamının 2D haritası çıkarılmıştır.
* **Algoritma:** Cartographer / Gmapping.
* **Çıktı:** `maps/lab_map.pgm` ve `.yaml`.

### [Görev 3] Çoklu Hedef Navigasyonu
Çıkarılan harita üzerinde belirlenen 3-4 farklı noktaya (Masa, Kapı vb.) sırayla otonom navigasyon yapılması.
* **Yöntem:** `move_base` action client.

### [Görev 4] Dinamik Engeller ve Recovery
Hareketli engeller (insan, sandalye) karşısında robotun yeniden planlama yapması ve sıkışma durumunda kurtarma (recovery) davranışları sergilemesi.

### [Görev 5] Görev Yöneticisi (Mission Manager)
JSON tabanlı bir senaryo dosyasını okuyarak robotun kargo teslim/devriye görevlerini yerine getirmesi.

## 🚀 Kurulum ve Çalıştırma

### Gereksinimler
* ROS 1 (Melodic/Noetic)
* TurtleBot3 Paketleri
* Python 3

### Derleme
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
