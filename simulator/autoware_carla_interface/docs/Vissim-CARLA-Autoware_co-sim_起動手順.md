# Vissim-CARLA-Autoware co-sim 起動手順

## 0. 前提条件
- Autoware環境を構築済みであること。環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。
- PTV Vissim Kernel for Linux(`/opt/vissim_kernel_2026.00-10`)がインストール済みであり、`libDrivingSimulatorProxy.so`が利用可能であること(ライセンス(CmDongle)が有効であること)。
- .inpxネットワークの設定で「ドライブシミュレータ アクティブ(Driving Simulator active)」 が有効になっていること。この設定が無効だと実質的に何も同期しない。


## 1. リポジトリ
以下のブランチを使用する。
このブランチはAutoware Universe(0.52.0)のautoware_carla_interfaceをベースに、CARLA公式の
Vissim-CARLAブリッジ(PTV Vissim Driving Simulator Interface経由)を盛り込み、
fullstack(Vissim-CARLA-Autoware)のco-simを実装したものである。

- `autoware_universe`: `feature/vissim_co-sim`ブランチ
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

## 2. 実行コマンド
### 2.1 [ターミナル1] CARLAサーバーの起動
CARLAサーバーが起動します。

```bash
cd ~/CARLA
./CarlaUE4.sh
```

### 2.2 [ターミナル2] マップの設定 → 信号マッピングjsonの生成
Town01マップを読み込んだうえで、そのマップの信号機情報から`data/signal_mapping.json`を生成します。

#### 2.2.1 CARLAで使用するマップの設定

```bash
cd ~/CARLA/PythonAPI/util
source ~/carla310_env/bin/activate
python3 config.py --map Town01
```
| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `-m`, `--map <MAP>` | 稼働中のCARLAサーバーにロードするマップ名(例: `Town01`)を指定 | 省略可(省略時はマップ変更を行わない) | なし |

#### 2.2.2 信号マッピングjsonの生成
稼働中のCARLAサーバー(マップロード済みである必要あり)から信号機の位置を取得し、`.inpx`側の`signalHead`位置との幾何学的な最近傍マッチングにより`../data/signal_mapping.json`を生成します。

**注意**:
- **マップをロード済みである必要があります**(このコマンドの直前にマップ設定を行うこと)。
- **`.inpx`側の信号配置を変更した場合のみ再実行すればよく**、毎回の起動時に実行する必要はありません。

```bash
cd ~/CARLA/Co-Simulation/PTV-Vissim/util
python3 generate_signal_mapping.py ../examples/Town01/Town01.inpx
```
| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `vissim_network`(第1引数) | 対象の`.inpx`ネットワークファイルのパス | 省略不可 | なし |
| `--carla-host H` | CARLAホストサーバーのIPアドレス | 省略可 | `127.0.0.1` |
| `--carla-port P` | CARLAホストサーバーのTCPポート | 省略可 | `2000` |
| `--carla-timeout` | carlaクライアントの接続タイムアウト(秒) | 省略可 | `10.0` |
| `--max-distance` | この距離(メートル)より遠いマッチングは低信頼(low-confidence)として警告ログに出力され、手動確認が必要になる | 省略可 | `10.0` |
| `--output` | 生成するマッピングjsonの出力先パス | 省略可 | `../data/signal_mapping.json` |
| `--debug` | デバッグメッセージを有効化するフラグ | 省略可(フラグ指定のみ、値は取らない) | 指定なし(`False`、無効) |



### 2.3 [ターミナル3] ビルド
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

| オプション | 内容 | 用途 |
|---|---|---|
| `--packages-select` | 指定パッケージのみビルド | 通常の開発 |
| `--packages-up-to` | 指定パッケージ＋依存パッケージをビルド | 初回・依存変更時 |
| `--symlink-install` | installへコピーせずシンボリックリンクを作成 | Python開発を効率化 |

- Pythonファイル変更時
  - `--symlink-install`あり：通常は**再ビルド不要**
  - `--symlink-install`なし：**再ビルド必要**

- 再ビルドが必要なケース
  - setup.py
  - setup.cfg
  - package.xml
  - CMakeLists.txt
  - entry_points変更
  - C++ソース（.cpp）

#### ビルド後、環境設定を読み込む

```bash
source ~/autoware.1.9.0/install/setup.bash
```

### 2.4 [ターミナル3] Vissim/CARLA/Autoware起動
autoware_launch経由でAutoware本体とCARLAインタフェース(autoware_carla_interface)を起動し、CARLA上にEGO車両をスポーンさせた上でVissimとのco-simulation(車両同期・信号同期)を開始します。なお、Vissimはヘッドレス実行のため、画面表示はありません。

```bash
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
| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `simulator_type` | 使用するシミュレータ種別を指定する | 省略不可 | なし |
| `map_path` | Lanelet2マップのディレクトリパス(マップ名を抽出しCARLA側のマップロードにも使用) | 省略不可 | なし |
| `vehicle_model` | 車両モデル | 省略可 | `sample_vehicle` |
| `sensor_model` | センサキット。`autoware_launch`側で解決され、`autoware_carla_interface`には`sensor_kit_name`として渡る | 省略可 | Launch既定値(`sensor_kit_name`の既定値は`carla_sensor_kit_description`) |
| `use_vissim` | Vissim-CARLA co-simulationを有効化する | 省略可 | `false` |
| `vissim_network` | Vissimネットワークファイル(`.inpx`)へのパス | 省略可(`use_vissim:=true`時は指定必須) | 空文字列 |
| `vissim_lib_path` | `libDrivingSimulatorProxy.so`の絶対パス | 省略可 | 空文字列(`LD_LIBRARY_PATH`任せ) |
| `vissim_simulator_vehicles` | Vissim側で同時にトラッキングされるDriving Simulator(CARLA発生)車両の最大数(既定1=EGOのみ) | 省略可 | `1` |
| `sync_traffic_lights` | 信号機の状態をVissimからCARLAへ同期する(Vissim→CARLA方向のみ) | 省略可 | `false` |
| `spectator_follow` | CARLAスペクテーター(自由視点カメラ)をEGO車両のスポーンと同時に自動追従させる | 省略可 | `false` |

