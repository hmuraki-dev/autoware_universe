# SUMO-CARLA-Autoware co-sim 起動手順

# 0. 前提条件
Autoware環境を構築済みであること。
環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。


# 1. リポジトリ

以下のブランチを使用する。
このブランチはAutoware Universe(0.52.0)のautoware_carla_interfaceをベースに、CARLA公式のSUMO-CARLAブリッジを盛り込み、fullstack(SUMO-CARLA-Autoware)のco-simを実装したものである。

- ARC: 
https://github.com/NTT-DATA-ARC/autoware_universe/tree/feature/autoware-carla-interface
- 個人: 
https://github.com/hmuraki-dev/autoware_universe/tree/feature/autoware-carla-interface

本ブランチは、Autoware Universe ワークスペース内の次のディレクトリに配置される。

```text
~/autoware.1.9.0/
└── src/
    └── universe/
        └── autoware_universe/      ← GitHubからcloneするリポジトリ
            └── simulator/
                └── autoware_carla_interface/
```

`autoware_carla_interface` は独立したリポジトリではなく、`autoware_universe` リポジトリ配下のパッケージである。

# 2. 実行コマンド

## 2.1 CARLAサーバー起動(ターミナル1)

```bash
cd ~/CARLA
./CarlaUE4.sh
```
コマンドを実行すると、CARLAサーバーが起動し、CARLA(Unreal Engine)の画面が表示される。


## 2.2 Town01ロード(ターミナル2)

```bash
cd ~/CARLA/PythonAPI/util
python3 config.py --map Town01
```
コマンドを実行すると、CARLAで読み込まれているマップが Town01 に切り替わる。CARLA画面が Town01 の地図に更新されたことを確認する。


### 2.3 ビルド(ターミナル3)

#### 通常のビルド(推奨)

```bash
source /opt/ros/humble/setup.bash
cd ~/autoware.1.9.0
colcon build --packages-select autoware_carla_interface --symlink-install
```

#### 初回ビルド・依存関係も含める場合

```bash
colcon build --packages-up-to autoware_carla_interface
```

#### ビルドオプション

| オプション | 内容 | 用途 |
|---|---|---|
| `--packages-select` | 指定パッケージのみビルド | 通常の開発 |
| `--packages-up-to` | 指定パッケージ＋依存パッケージをビルド | 初回・依存変更時 |
| `--symlink-install` | installへコピーせずシンボリックリンクを作成 | Python開発を効率化 |

#### Pythonファイル変更時

- `--symlink-install`あり：通常は**再ビルド不要**
- `--symlink-install`なし：**再ビルド必要**

#### 再ビルドが必要なケース

- setup.py
- setup.cfg
- package.xml
- CMakeLists.txt
- entry_points変更
- C++ソース（.cpp）

#### ビルド後

```bash
source ~/autoware.1.9.0/install/setup.bash
```

### 2.4 SUMO/CARLA/Autoware起動(ターミナル3)

```bash
source install/setup.bash
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_sumo:=true \
  sumo_cfg_file:=/home/divp/CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg \
  tls_manager:=sumo \
  sync_vehicle_lights:=true \
  sync_vehicle_color:=true
```

#### オプション一覧

| オプション | 説明 | 設定例 | デフォルト値(未指定時) |
|-----------|------|--------|--------------------------|
| `simulator_type` | 使用するシミュレータ | `carla` | なし(指定必須) |
| `map_path` | Lanelet2マップ | `$HOME/autoware_map/Town01` | なし(指定必須) |
| `vehicle_model` | 車両モデル | `sample_vehicle` | Launch既定値 |
| `sensor_model` | センサキット | `carla_sensor_kit` | Launch既定値 |
| `use_sumo` | SUMO連携 | `true` | `false` |
| `sumo_cfg_file` | SUMO設定 | `...Town01.sumocfg` | なし |
| `sumo_gui` | GUI表示 | `true` | `false(ヘッドレス)` |
| `sumo_host` | TraCIホスト | `localhost` | `localhost` |
| `sumo_port` | TraCIポート | `8813` | `8813` |
| `sumo_client_order` | クライアント順 | `1` | `1` |
| `tls_manager` | 信号管理 | `sumo` | `none` |
| `sync_vehicle_lights` | 灯火同期 | `true` | `false` |
| `sync_vehicle_color` | 車体色同期 | `true` | `false` |

### 2.5 [appendix] 処理時間計測

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  :
  :
  2>&1 | tee /tmp/autoware_carla.log
 ```
- `2>&1 | tee /tmp/autoware_carla.log` でターミナルログをautoware_carla.logに保存

```bash
grep "MAIN_LOOP_PERIOD" /tmp/autoware_carla.log
```
- 例えば、ログに[MAIN_LOOP_PERIOD]タグをつけている場合は、上記のコマンドで対象ログを抽出できます。