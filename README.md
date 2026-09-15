# rtabmap_minimal

Минимальная рабочая демонстрация **RTAB-Map** как SLAM-бэкенда для **Intel RealSense D435i**
на Ubuntu/Xubuntu 22.04. Свой SLAM здесь не пишется: детектор, матчер, PnP, keyframes,
loop closure и pose graph целиком внутри RTAB-Map.

```
D435i (RGB + Depth, aligned + IMU)
   -> rtabmap::Odometry   (визуальная одометрия F2M)
   -> rtabmap::Rtabmap    (память, loop closure, оптимизация графа)
   -> живая 3D-карта (rtabmap::CloudViewer) + .db + облако точек (PCL) + траектория
```

Весь код — один файл `src/main.cpp` (~450 строк), из них собственно SLAM-вызовов пять:
`Odometry::create`, `Odometry::process`, `Rtabmap::init`, `Rtabmap::process`, `Rtabmap::getGraph`.

## Зависимости

| Компонент | Версия | Откуда |
|---|---|---|
| RTAB-Map | 0.23.7 | пакет `ros-humble-rtabmap` (это **standalone-библиотека**, ROS-узлы не нужны) |
| g2o / GTSAM | из того же пакета | оптимизаторы графа |
| librealsense2 | 2.58.3 | локальная сборка SDK |
| OpenCV | 4.5.4 | система |
| PCL | 1.12.1 | система |
| Eigen | 3.4 | система |

ROS 2 Humble используется **только как репозиторий пакетов**. Ни одного ROS-узла,
`rclcpp`, `rviz` или launch-файла в проекте нет.

Установка зависимостей (с root — предпочтительно):

```bash
sudo apt install ros-humble-rtabmap librealsense2-dev
```

Без root (именно так собрано здесь) те же `.deb` скачиваются и распаковываются в `./deps`:

```bash
./scripts/setup_deps.sh
```

## Сборка

```bash
./scripts/build.sh      # cmake + make -j1 под nice: на этом ноутбуке -j4 уводит в своп
```

## Запуск

```bash
./build/rtabmap_minimal                 # из корня проекта, чтобы нашёлся config/
./build/rtabmap_minimal --no-map        # без окна 3D-карты (быстрее)
./build/rtabmap_minimal --no-view       # без окна камеры, только карта
./build/rtabmap_minimal --no-gui        # без окон вообще, статус в консоль
./build/rtabmap_minimal --no-imu        # без IMU (и без gravity-констрейнтов)
./build/rtabmap_minimal --fps 6 --size 424 240   # ещё легче для CPU
```

Открываются два окна:

* **camera** — поток камеры, трекаемые точки, состояние одометрии (`TRACKING` / `LOST`),
  число features/inliers, обработанных кадров, частота, число узлов карты, loop closure, позиция;
* **map** — 3D-карта, которая строится **в реальном времени**: облако каждого нового узла
  добавляется сразу, а при loop closure позы уже показанных облаков пересчитываются
  (облака не пересобираются, двигается только их поза), плюс белая линия траектории.

Клавиши в любом окне: `q` — выход с сохранением, `s` — сохранить карту не выходя.
Закрытие обоих окон или `Ctrl+C` в консоли тоже корректно сохраняет карту.

Окна — это штатные виджеты RTAB-Map (`rtabmap::CloudViewer`, `rtabmap::ImageView`),
те же, что внутри `rtabmap-databaseViewer`. **PCLVisualizer в этой системе использовать
нельзя**: связка PCL 1.12 + VTK 9.1 из Ubuntu 22.04 падает с SIGSEGV в `_XEventsQueued`
на первом же `spinOnce()` — это воспроизводится и в 30-строчной программе без RTAB-Map.

## Потоки камеры

| Поток | Формат | Разрешение | Частота |
|---|---|---|---|
| COLOR | BGR8 | 640×480 | 15 FPS |
| DEPTH | Z16 | 640×480 | 15 FPS |
| ACCEL / GYRO | MOTION_XYZ32F | — | штатная |

Depth выравнивается на RGB через `rs2::align(RS2_STREAM_COLOR)`, интринсики берутся
из цветного потока. IMU проходит через `rtabmap::IMUFilter` (фильтр Мадгвика **из RTAB-Map**),
полученная ориентация кладётся в `SensorData::setIMU` вместе с extrinsics gyro→color.
Это даёт RTAB-Map gravity-констрейнты при оптимизации графа (`Optimizer/GravitySigma`).

## Optimizer / backend

Собранный `ros-humble-rtabmap` 0.23.7 содержит **TORO, g2o и GTSAM; Ceres не собран**.
Программа печатает это при старте:

```
RTAB-Map 0.23.7 | optimizers: TORO=1 g2o=1 GTSAM=1 Ceres=0 | using strategy 2
```

Используется **штатный для этой сборки бэкенд — GTSAM** (`Optimizer/Strategy = 2`,
это и есть скомпилированный default RTAB-Map). Чтобы сравнить с g2o, достаточно
поставить `Optimizer\Strategy = 1` в `config/rtabmap_minimal.ini`. Ceres не добавлялся:
его нет в сборке и он ничего не даёт проекту.

## Где используется PCL

Живая карта: `util3d::cloudRGBFromSensorData` строит `pcl::PointCloud<pcl::PointXYZRGB>`
для каждого нового узла (decimation 8, до 4 м), облако отдаётся в `CloudViewer::addCloud`.

В `exportMap()` (`src/main.cpp`), при сохранении карты:

* `util3d::cloudRGBFromSensorData` — RGB-D → `pcl::PointCloud<pcl::PointXYZRGB>`;
* `pcl::transformPointCloud` — перевод каждого узла в оптимизированную позу;
* `pcl::VoxelGrid` — прореживание собранного облака (3 см);
* `pcl::getMinMax3D` — статистика карты (bounding box);
* `pcl::io::savePCDFileBinary` — сохранение `.pcd`.

Собственной геометрии/SLAM на PCL нет.

## Сохранение результата

По `q`, `s` или `Ctrl+C` пишутся:

* `rtabmap_minimal.db` — полная база RTAB-Map (граф, позы, изображения, слова);
* `rtabmap_minimal_cloud.pcd` — собранное и прореженное облако;
* `rtabmap_minimal_trajectory.txt` — траектория (`timestamp x y z qx qy qz qw id`).

Посмотреть карту и loop closures штатными инструментами RTAB-Map:

```bash
deps/opt/ros/humble/bin/rtabmap-databaseViewer rtabmap_minimal.db   # или rtabmap-databaseViewer, если ставили через apt
deps/opt/ros/humble/bin/rtabmap-info rtabmap_minimal.db
```

## Снижение нагрузки на слабом железе

Всё крутится в `config/rtabmap_minimal.ini` (обычные параметры RTAB-Map, `rtabmap --params`):

| Параметр | Эффект |
|---|---|
| `Rtabmap\DetectionRate` | как часто создаётся узел карты (1 Гц; главный регулятор) |
| `Vis\MaxFeatures`, `Kp\MaxFeatures` | число фич для одометрии и для BoW |
| `OdomF2M\MaxSize` | размер локальной карты одометрии |
| `Rtabmap\TimeThr` | лимит времени на итерацию, лишнее уходит в long-term memory |
| `Mem\ImagePreDecimation` | уменьшение картинки до обработки |
| `--fps`, `--size` | меньше кадров и пикселей с камеры |
| `--no-map` | отключить живую 3D-карту: ~+1,5–2 Гц и ~150 МБ RAM |

Кадры не копятся: из потока камеры всегда берётся самый свежий frameset, остальные
отбрасываются, поэтому при просадке частоты система не уходит в лаг, а просто
обрабатывает реже.

Замеры на этом ноутбуке (статичная сцена, 640×480@15):

| режим | частота обработки | CPU (одно ядро) | RSS |
|---|---|---|---|
| `--no-gui` | 7,8–9,8 Гц | 69–75 % | ~205 МБ |
| оба окна, ~1 узел/с | 5,4–5,9 Гц | 58–69 % | ~350–370 МБ |

Окно карты перерисовывается не чаще 5 раз в секунду и обновляется только при появлении
нового узла, поэтому между узлами оно почти ничего не стоит.

## Что это такое

Это **reference-конфигурация RTAB-Map**, а не ещё один самописный SLAM. Всё, что можно
было отдать RTAB-Map, отдано RTAB-Map; в проекте остались только захват кадров с D435i,
передача их в RTAB-Map и минимальная визуализация/экспорт.
