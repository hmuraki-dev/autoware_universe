# CARLA 車両寸法一覧 (CARLA 0.9.15)

Vissim 車両タイプ → CARLA ブループリントの割り当て (`vissim_integration/data/vtypes.json`) を検討するための、CARLA 標準車両の寸法一覧。

## 測定方法

- 対象: `world.get_blueprint_library().filter('vehicle.*')` の全 41 ブループリント
- 各ブループリントを 1 台ずつスポーンし、`actor.bounding_box.extent` の 2 倍を寸法とした (単位: m)
  - バウンディングボックス値のため、実車諸元とは一致しない場合がある (特に二輪車・自転車)
- `base_type` / `wheels` はブループリント属性 `base_type` / `number_of_wheels` の値
- 画像は公式車両カタログ (https://carla.readthedocs.io/en/0.9.15/catalogue_vehicles/) の画像 (1920x1080) から車両部分をトリミングしたもの (`img/carla_vehicles/`, 高さ 336px に統一)
- 実行コマンド (`~/CARLA/Co-Simulation/PTV-Vissim/util` で実行)

```bash
python3 carla_vehicle_dimensions.py --host localhost --port 2000 --csv carla_vehicle_dims.csv
```

## 寸法一覧 (base_type 別・車長順)

「vtypes.json」列は、以前の選び方 (`vissim_carla_variants:=legacy_vtypes`) での割り当て先 Vissim 車両タイプ。
「LiDAR 車長・車幅」は、セマンティック LiDAR で測った外形 (`tools/carla_fleet/measure_blueprint_mesh.py`、2026-11-16。`data/carla_fleet/mesh_measurements_carla0.9.15.json`)。今の割り当て (`carla_variants_town01.yaml`) はこの値で決めている。
「carla_variants」列は、今の割り当て先 Vissim 2D/3D モデル番号 (`Vissim_車両寸法一覧.md` の最後の節)。

### car (乗用車)

| 画像 | ブループリント | base_type | 車輪数 | 車長 | 車幅 | 車高 | vtypes.json | LiDAR 車長 | LiDAR 車幅 | carla_variants |
|---|---|---|---:|---:|---:|---:|---|---:|---:|---|
| <img src="img/carla_vehicles/micro_microlino.webp" height="112"> | vehicle.micro.microlino | car | 4 | 2.21 | 1.48 | 1.38 | | 2.20 | 1.48 |  |
| <img src="img/carla_vehicles/nissan_micra.webp" height="112"> | vehicle.nissan.micra | car | 4 | 3.63 | 1.85 | 1.50 | 100 | 3.57 | 1.81 | 1 |
| <img src="img/carla_vehicles/audi_a2.webp" height="112"> | vehicle.audi.a2 | car | 4 | 3.71 | 1.79 | 1.55 | 100 | 3.69 | 1.75 |  |
| <img src="img/carla_vehicles/mini_cooper_s.webp" height="112"> | vehicle.mini.cooper_s | (未設定) | 4 | 3.81 | 1.97 | 1.48 | 100 | 3.80 | 1.96 |  |
| <img src="img/carla_vehicles/jeep_wrangler_rubicon.webp" height="112"> | vehicle.jeep.wrangler_rubicon | car | 4 | 3.87 | 1.91 | 1.88 | 100 | 3.59 | 1.88 |  |
| <img src="img/carla_vehicles/citroen_c3.webp" height="112"> | vehicle.citroen.c3 | car | 4 | 3.99 | 1.85 | 1.62 | 100 | 3.98 | 1.84 |  |
| <img src="img/carla_vehicles/audi_tt.webp" height="112"> | vehicle.audi.tt | car | 4 | 4.18 | 1.99 | 1.39 | 100 | 4.15 | 1.97 | 2, 3 |
| <img src="img/carla_vehicles/seat_leon.webp" height="112"> | vehicle.seat.leon | car | 4 | 4.19 | 1.82 | 1.47 | 100 | 4.13 | 1.80 |  |
| <img src="img/carla_vehicles/toyota_prius.webp" height="112"> | vehicle.toyota.prius | car | 4 | 4.51 | 2.01 | 1.52 | 100 | 4.47 | 1.95 |  |
| <img src="img/carla_vehicles/mini_cooper_s_2021.webp" height="112"> | vehicle.mini.cooper_s_2021 | car | 4 | 4.55 | 2.10 | 1.77 | | 4.50 | 2.16 | 4, 8, 9 |
| <img src="img/carla_vehicles/nissan_patrol.webp" height="112"> | vehicle.nissan.patrol | car | 4 | 4.60 | 1.93 | 1.85 | 100 | 4.56 | 1.91 | 10 |
| <img src="img/carla_vehicles/bmw_grandtourer.webp" height="112"> | vehicle.bmw.grandtourer | (未設定) | 4 | 4.61 | 2.24 | 1.67 | 100 | 4.56 | 2.24 |  |
| <img src="img/carla_vehicles/mercedes_coupe_2020.webp" height="112"> | vehicle.mercedes.coupe_2020 | car | 4 | 4.67 | 1.81 | 1.44 | | 4.64 | 2.00 |  |
| <img src="img/carla_vehicles/ford_mustang.webp" height="112"> | vehicle.ford.mustang | car | 4 | 4.72 | 1.89 | 1.30 | 100 | 4.65 | 1.85 |  |
| <img src="img/carla_vehicles/tesla_model3.webp" height="112"> | vehicle.tesla.model3 | car | 4 | 4.79 | 2.16 | 1.49 | 100 | 4.75 | 2.10 | 5, 6 |
| <img src="img/carla_vehicles/audi_etron.webp" height="112"> | vehicle.audi.etron | car | 4 | 4.86 | 2.03 | 1.65 | 100 | 4.84 | 2.01 |  |
| <img src="img/carla_vehicles/lincoln_mkz_2020.webp" height="112"> | vehicle.lincoln.mkz_2020 | car | 4 | 4.89 | 1.84 | 1.49 | | 4.88 | 1.83 |  |
| <img src="img/carla_vehicles/lincoln_mkz_2017.webp" height="112"> | vehicle.lincoln.mkz_2017 | car | 4 | 4.90 | 2.13 | 1.51 | 100 | 4.85 | 2.10 |  |
| <img src="img/carla_vehicles/dodge_charger_police.webp" height="112"> | vehicle.dodge.charger_police | car | 4 | 4.97 | 2.04 | 1.54 | | 4.95 | 1.99 |  |
| <img src="img/carla_vehicles/dodge_charger_2020.webp" height="112"> | vehicle.dodge.charger_2020 | car | 4 | 5.01 | 1.88 | 1.53 | | 5.00 | 1.90 | 7 |
| <img src="img/carla_vehicles/mercedes_coupe.webp" height="112"> | vehicle.mercedes.coupe | car | 4 | 5.03 | 2.15 | 1.65 | 100 | 5.01 | 2.15 |  |
| <img src="img/carla_vehicles/dodge_charger_police_2020.webp" height="112"> | vehicle.dodge.charger_police_2020 | car | 4 | 5.24 | 1.93 | 1.64 | | 5.20 | 1.90 |  |
| <img src="img/carla_vehicles/chevrolet_impala.webp" height="112"> | vehicle.chevrolet.impala | car | 4 | 5.36 | 2.03 | 1.41 | | 5.32 | 1.98 |  |
| <img src="img/carla_vehicles/ford_crown.webp" height="112"> | vehicle.ford.crown | car | 4 | 5.37 | 1.80 | 1.57 | | 5.27 | 1.77 |  |
| <img src="img/carla_vehicles/nissan_patrol_2021.webp" height="112"> | vehicle.nissan.patrol_2021 | car | 4 | 5.57 | 2.15 | 2.05 | | 5.55 | 2.42 |  |

### van (バン)

| 画像 | ブループリント | base_type | 車輪数 | 車長 | 車幅 | 車高 | vtypes.json | LiDAR 車長 | LiDAR 車幅 | carla_variants |
|---|---|---|---:|---:|---:|---:|---|---:|---:|---|
| <img src="img/carla_vehicles/volkswagen_t2_2021.webp" height="112"> | vehicle.volkswagen.t2_2021 | van | 4 | 4.44 | 1.77 | 1.99 | | 4.37 | 1.74 |  |
| <img src="img/carla_vehicles/volkswagen_t2.webp" height="112"> | vehicle.volkswagen.t2 | van | 4 | 4.48 | 2.07 | 2.04 | 100 | 4.47 | 2.06 |  |
| <img src="img/carla_vehicles/mercedes_sprinter.webp" height="112"> | vehicle.mercedes.sprinter | van | 4 | 5.92 | 1.99 | 2.56 | | 5.87 | 2.32 |  |
| <img src="img/carla_vehicles/ford_ambulance.webp" height="112"> | vehicle.ford.ambulance | van | 4 | 6.37 | 2.35 | 2.43 | | 6.31 | 2.62 |  |

### truck (トラック)

| 画像 | ブループリント | base_type | 車輪数 | 車長 | 車幅 | 車高 | vtypes.json | LiDAR 車長 | LiDAR 車幅 | carla_variants |
|---|---|---|---:|---:|---:|---:|---|---:|---:|---|
| <img src="img/carla_vehicles/carlamotors_carlacola.webp" height="112"> | vehicle.carlamotors.carlacola | truck | 4 | 5.20 | 2.63 | 2.47 | 200 | 5.11 | 2.56 | 22, 23, 31 (重ね置き) |
| <img src="img/carla_vehicles/tesla_cybertruck.webp" height="112"> | vehicle.tesla.cybertruck | truck | 4 | 6.27 | 2.39 | 2.10 | | 6.11 | 2.33 |  |
| <img src="img/carla_vehicles/carlamotors_european_hgv.webp" height="112"> | vehicle.carlamotors.european_hgv | truck | 6 | 7.94 | 2.89 | 3.46 | | 8.43 | 3.34 |  |
| <img src="img/carla_vehicles/carlamotors_firetruck.webp" height="112"> | vehicle.carlamotors.firetruck | truck | 4 | 8.47 | 2.89 | 3.83 | | 8.43 | 3.34 |  |

### Bus (バス)

| 画像 | ブループリント | base_type | 車輪数 | 車長 | 車幅 | 車高 | vtypes.json | LiDAR 車長 | LiDAR 車幅 | carla_variants |
|---|---|---|---:|---:|---:|---:|---|---:|---:|---|
| <img src="img/carla_vehicles/mitsubishi_fusorosa.webp" height="112"> | vehicle.mitsubishi.fusorosa | Bus | 4 | 10.27 | 3.94 | 4.25 | | 9.78 | 3.18 |  |

### motorcycle (自動二輪)

| 画像 | ブループリント | base_type | 車輪数 | 車長 | 車幅 | 車高 | vtypes.json | LiDAR 車長 | LiDAR 車幅 | carla_variants |
|---|---|---|---:|---:|---:|---:|---|---:|---:|---|
| <img src="img/carla_vehicles/vespa_zx125.webp" height="112"> | vehicle.vespa.zx125 | motorcycle | 2 | 1.82 | 0.87 | 1.59 | | 1.81 | 0.93 |  |
| <img src="img/carla_vehicles/kawasaki_ninja.webp" height="112"> | vehicle.kawasaki.ninja | motorcycle | 2 | 2.04 | 0.80 | 1.52 | 610, 620 | 2.02 | 0.87 | 70 |
| <img src="img/carla_vehicles/yamaha_yzf.webp" height="112"> | vehicle.yamaha.yzf | motorcycle | 2 | 2.19 | 0.87 | 1.53 | 610, 620 | 2.20 | 0.85 |  |
| <img src="img/carla_vehicles/harley-davidson_low_rider.webp" height="112"> | vehicle.harley-davidson.low_rider | motorcycle | 2 | 2.35 | 0.77 | 1.65 | 610, 620 | 2.34 | 0.87 |  |

### bicycle (自転車)

| 画像 | ブループリント | base_type | 車輪数 | 車長 | 車幅 | 車高 | vtypes.json | LiDAR 車長 | LiDAR 車幅 | carla_variants |
|---|---|---|---:|---:|---:|---:|---|---:|---:|---|
| <img src="img/carla_vehicles/bh_crossbike.webp" height="112"> | vehicle.bh.crossbike | bicycle | 2 | 1.51 | 0.87 | 1.61 | 610, 620 | 1.47 | 0.84 |  |
| <img src="img/carla_vehicles/diamondback_century.webp" height="112"> | vehicle.diamondback.century | bicycle | 2 | 1.66 | 0.58 | 1.62 | 610, 620 | 1.63 | 0.87 |  |
| <img src="img/carla_vehicles/gazelle_omafiets.webp" height="112"> | vehicle.gazelle.omafiets | bicycle | 2 | 1.84 | 0.66 | 1.78 | 610, 620 | 1.82 | 0.69 |  |

## 備考

- `vehicle.bmw.grandtourer` と `vehicle.mini.cooper_s` は `base_type` 属性が空。
- `vehicle.mitsubishi.fusorosa` の `base_type` は大文字始まりの `Bus` (他は小文字)。
- `vehicle.carlamotors.european_hgv` のみ 6 輪。
- LiDAR の外形とバウンディングボックスが大きく違うもの (2026-11-16): `european_hgv` は見た目がトラクターだが、LiDAR には `firetruck` と同じ形 (8.43 × 3.34 m) で映る。`fusorosa` は箱 (3.94 m) より細い (3.18 m)。`ford.ambulance`・`mercedes.sprinter`・`nissan.patrol_2021` は箱より 0.27〜0.34 m 広い。
- 「vtypes.json」列は、この資料を作った時点の値。今の `vtypes.json` とは一致しないところがある。
- 現在の `vtypes.json` の 610 / 620 には自動二輪と自転車が混在している。
