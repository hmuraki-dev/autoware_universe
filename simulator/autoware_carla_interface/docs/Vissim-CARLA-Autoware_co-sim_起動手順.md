# Vissim-CARLA-Autoware co-sim 起動手順

# 0. 前提条件
Autoware環境を構築済みであること。
環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。

PTV Vissim Kernel for Linux(`/opt/vissim_kernel_2026.00-10`)がインストール済みであり、
`libDrivingSimulatorProxy.so`が利用可能であること(ライセンス(CmDongle)が有効であること)。


# 1. リポジトリ

以下のブランチを使用する。
このブランチはAutoware Universe(0.52.0)のautoware_carla_interfaceをベースに、CARLA公式の
Vissim-CARLAブリッジ(PTV Vissim Driving Simulator Interface経由)を盛り込み、
fullstack(Vissim-CARLA-Autoware)のco-simを実装したものである。

- `autoware_universe`: `feature/vissim_co-sim`ブランチ
  (詳細は`autoware_carla_interface/docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`を参照)

本ブランチは、Autoware Universe ワークスペース内の次のディレクトリに配置される。

```text
~/autoware.1.9.0/
└── src/
    └── universe/
        └── autoware_universe/      ← GitHubからcloneするリポジトリ
            └── simulator/
                └── autoware_carla_interface/
```

`autoware_carla_interface` は独立したリポジトリではなく、`autoware_universe` リポジトリ配下の
パッケージである。

## 1.1 `autoware_launch` 側の対応(要確認)

SUMO版と同様、`ros2 launch autoware_launch e2e_simulator.launch.xml`経由でVissimパラメータを
渡せるようにするには、`autoware_launch`パッケージの`launch/e2e_simulator.launch.xml`に
Vissim関連の`<arg>`(`use_vissim`/`vissim_network`/`vissim_lib_path`/
`vissim_simulator_vehicles`/`sync_traffic_lights`)を追加し、`autoware_carla_interface.launch.xml`
の`<include>`へ転送する変更が必要(SUMO版の`use_sumo`等と同一パターン)。

本環境では`~/autoware.1.9.0/src/launcher/autoware_launch`にこの変更を適用済み(ローカルの
未コミット差分)。別環境やクリーンな`autoware_launch`checkoutで実行する場合は、同様の変更を
適用すること。


# 2. 実行コマンド

## 2.1 CARLAサーバー起動(ターミナル1)

```bash
cd ~/CARLA
./CarlaUE4.sh
```
コマンドを実行すると、CARLAサーバーが起動し、CARLA(Unreal Engine)の画面が表示される。


## 2.2 Town01ロード + 信号マッピング確認(ターミナル2)

```bash
cd ~/CARLA/PythonAPI/util
python3 config.py --map Town01
```
コマンドを実行すると、CARLAで読み込まれているマップが Town01 に切り替わる。CARLA画面が Town01 の
地図に更新されたことを確認する。

Vissim連携では、Vissimの信号グループとCARLA信号機(OpenDRIVE ID)を対応付けるマッピングファイル
`autoware_carla_interface/src/autoware_carla_interface/vissim_integration/data/signal_mapping.json`
が必要。Town01用のマッピングは作成・vendor化済みのため、Town01を使う限り追加作業は不要。
Town01以外のマップを使う場合は、`generate_signal_mapping.py`(参照元:
`~/CARLA/Co-Simulation/PTV-Vissim/util/generate_signal_mapping.py`)で対象マップ・対象`.inpx`
向けに再生成する必要がある。


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

### 2.4 Vissim/CARLA/Autoware起動(ターミナル3)

```bash
source install/setup.bash
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_vissim:=true \
  vissim_network:=/home/divp/CARLA/Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx \
  vissim_lib_path:=/opt/vissim_kernel_2026.00-10/lib/libDrivingSimulatorProxy.so \
  sync_traffic_lights:=true \
  spectator_follow:=true
```

#### オプション一覧

| オプション | 説明 | 設定例 | デフォルト値(未指定時) |
|-----------|------|--------|--------------------------|
| `simulator_type` | 使用するシミュレータ | `carla` | なし(指定必須) |
| `map_path` | Lanelet2マップ | `$HOME/autoware_map/Town01` | なし(指定必須) |
| `vehicle_model` | 車両モデル | `sample_vehicle` | Launch既定値 |
| `sensor_model` | センサキット | `carla_sensor_kit` | Launch既定値 |
| `use_vissim` | Vissim連携 | `true` | `false` |
| `vissim_network` | Vissimネットワークファイル(`.inpx`) | `.../Town01/Town01.inpx` | なし |
| `vissim_lib_path` | `libDrivingSimulatorProxy.so`の絶対パス | `/opt/vissim_kernel_.../libDrivingSimulatorProxy.so` | 空(`LD_LIBRARY_PATH`任せ) |
| `vissim_simulator_vehicles` | VissimにDriving Simulator車両として同時登録できる最大台数(既定1=EGOのみ) | `1` | `1` |
| `sync_traffic_lights` | 信号同期(Vissim→CARLA一方向のみ) | `true` | `false` |
| `spectator_follow` | EGO車両(role_name=`ego_vehicle_role_name`)にCARLAスペクテーターを自動追従させる | `true` | `false` |

**注意**:

- `use_vissim:=true`と`use_traffic_manager:=true`は**同時指定不可**
  (起動時に`ValueError`で即座に停止する。自動アダプト機構がTraffic Manager由来のNPCも
  無差別にVissimへ登録しようとするため)。
- SUMO版と異なり、Vissim連携は独立したサーバープロセスを別ターミナルで起動する必要が**ない**
  (`libDrivingSimulatorProxy.so`は`autoware_carla_interface`プロセス自身が`ctypes`経由で
  ロード・接続する)。
- `fixed_delta_seconds`(CARLA)と`vissim_network`(`.inpx`)側のシミュレーションステップ時間
  (`simRes`)は**必ず一致させること**(既定はいずれも0.05秒。SUMO-CARLA-Autoware連携の実績値。
  `.inpx`側は`simRes=20`に設定すること)。一致していないと、EGO等の
  Driving Simulator車両がVissim側に登録される際の確認(CreateIDハンドシェイク)が失敗し続ける
  現象が起こることが判明している(詳細は
  `autoware_carla_interface/docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md` 0.3節参照)。
- `spectator_follow:=true`はCARLAスペクテーター(自由視点カメラ)をEGO車両に自動追従させる
  (`ego_vehicle_role_name`と同じ`role_name`を持つアクターを検出)。RVizで初期位置・目的地を
  設定してEGOがスポーン(または再スポーン)された後も自動的に追従先を検出し直すため、
  起動タイミングを気にする必要はない。カメラの距離・高さ・俯角を調整したい場合は、
  この引数は使わず`ros2 run autoware_carla_interface spectator_follow --distance ... --height ...`
  を別ターミナルで手動実行すること(詳細は`autoware_carla_interface/README.md`参照)。


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
