# multi-tb3-frontier-explore

## Proje Tanımı

Bu proje, ROS Noetic ortamında dört adet TurtleBot3 (burger modeli) robot ile **çok robotlu frontier tabanlı keşif sistemi** geliştirmektedir. Her robot `gmapping` ile SLAM gerçekleştirir, `multirobot_map_merge` ile haritalar birleştirilir ve `move_base` altyapısı ile keşif yapılır.

---

##  Gereksinimler ve Kurulum

### 1. Gerekli Paketlerin Kurulumu

```bash
sudo apt-get install ros-noetic-multirobot-map-merge
```

### 2. Catkin workspace ve bağımlılıkların klonlanması

```bash
mkdir -p ~/robotlar_ws/src

git clone https://github.com/tastangh/ros-multi-turtlebot-frontier-exploration.git

cd ~/robotlar_ws/src/ros-multi-turtlebot-frontier-exploration

# PDF'te verilen resmi kaynaklardan klonlama
git clone https://gitlab.com/blm6191_2425b_tai/blm6191/turtlebot3.git

git clone https://gitlab.com/blm6191_2425b_tai/blm6191/turtlebot3_simulations.git
git clone https://gitlab.com/blm6191_2425b_tai/blm6191/micromouse_maze.git
```

### 3. Workspace Derleme

```bash
cd ~/robotlar_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
catkin_make
```

### 4. Ortam Değişkenleri

```bash
echo "source ~/robotlar_ws/devel/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

##  Simülasyonu Başlatma

## Önerilen Yöntem: Tek Komut ile Başlatma

Projenin tüm bileşenlerini (Gazebo simülasyonu, harita birleştirme, SLAM, navigasyon, RViz ve keşif düğümü) tek bir komutla başlatmak için bir ana launch dosyası oluşturulmuştur.

```bash
roslaunch multirobot_exploration_mapping tam_kesif_sistemi.launch
```

## Alternatif: Adım Adım Manuel Başlatma

```bash
roslaunch micromouse_maze micromouse_maze3_multi.launch
roslaunch turtlebot3_gazebo multi_map_merge.launch
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch
roslaunch turtlebot3_navigation multi_move_base.launch
roslaunch micromouse_maze multi_robot_rviz.launch
```

---

```bash
rosrun multirobot_exploration_mapping frontier_explorer_node
```

Her robot `/map` mesajına abone olur, frontier bölgelerini tespit eder, TF ile konumunu bulur ve en yakın keşfedilmemiş noktaya hedef atar. Hedefe ulaşan robota yeni hedef atanır.

---

## Çalışma Adımları

1. `/map` mesajı alındığında frontier sınırları belirlenir.
2. Bu sınırlar bölgelere ayrılır ve merkez noktaları hesaplanır.
3. Her robot TF ile kendi konumunu bulur.
4. En yakın hedefe `MoveBaseAction` kullanılarak komut gönderilir.
5. Robot ilerleyemezse veya hedefe ulaşırsa yeni hedef atanır.

---

## Örnek Çıktılar

### TF Ağaç Yapısı
![tf_tree](tf_tree.png)

### Keşif Başlangıcı
![start](exploration_start.png)

### Keşif İlerlemesi
![progress](exploration_progress.png)

### Keşif Sonu
![end](exploration_end.png)

---

##  Ek Bilgiler

- Rviz üzerinden her bir robota `/tb3_X/move_base_simple/goal` topic'ine hedef gönderilerek manuel test yapılabilir.
- Hedef atama işlemleri `move_base` üzerinden action tabanlı gerçekleşir.
- Sistem her robot için thread ile çalıştığından terminal çıktılarında `[tb3_0]`, `[tb3_1]` gibi etiketlemeler kullanılmıştır.

