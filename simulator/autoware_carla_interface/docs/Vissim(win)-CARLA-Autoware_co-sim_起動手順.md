# Vissim(windows)-CARLA-Autoware co-sim 起動手順

## 0. 前提条件
- Autoware環境を構築済みであること。環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。
- .inpxネットワークの設定で「ドライブシミュレータ アクティブ(Driving Simulator active)」 が有効になっていること。この設定が無効だと実質的に何も同期しない。

**Windows機**: 
- CARLAリポジトリ(`/home/divp/CARLA/Co-Simulation/PTV-Vissim_windows/`)**フォルダ式をそのままWindows機にコピー**し、`pyzmq`・`msgpack`をインストール済みであること(`Co-Simulation/PTV-Vissim_windows/requirements.txt`参照、  `pip install -r requirements.txt`で一括インストール可能)。

**Linux機(Autoware/CARLA側)**:
- Windows機と同一LAN内に接続できること(VPN/WAN跨ぎは未サポート)。`autoware_carla_interface`ノードを実行するPython環境(`carla`パッケージを`pip install --user`した環境と同じもの)に、`pyzmq`・`msgpack`を追加でインストールしておく必要がある:

  ```bash
  python3 -m pip install --user pyzmq msgpack
  ```
  - `carla`パッケージ同様、`package.xml`/`setup.py`には登録されていない(rosdep管理外)ため、手動でのインストールが必要な点に注意。


## 1. リポジトリ
以下のブランチを使用する。
このブランチはAutoware Universe(0.52.0)のautoware_carla_interfaceをベースに、CARLA公式のVissim-CARLAブリッジ(PTV Vissim Driving Simulator Interface経由)を盛り込み、fullstack(Vissim-CARLA-Autoware)のco-simを実装したものである。

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
### 2.1 【Windows側】ファイアウォールの受信規則
このコマンドは、Windowsファイアウォールに受信規則を追加し、指定したLinux機のIPアドレスからのみ、TCPポート5555(2項で起動するVissimアダプタ`server.py`のZeroMQ待受ポート)への接続を許可します。この規則を先に登録しておかないと、Linux側の`run_synchronization.py`からのZeroMQ接続がWindows側ファイアウォールでブロックされてしまいます。

管理者として PowerShell を開き、Linux機のIPを <LINUX_IP> に入れて実行します。

```bash
New-NetFirewallRule -DisplayName "Vissim adapter (ZeroMQ 5555)" `
  -Direction Inbound -Protocol TCP -LocalPort 5555 `
  -RemoteAddress <LINUX_IP> -Action Allow -Profile Private
```
Linux機のIPアドレスが`192.168.16.33`の場合
```bash
New-NetFirewallRule -DisplayName "Vissim adapter (ZeroMQ 5555)" `
  -Direction Inbound -Protocol TCP -LocalPort 5555 `
  -RemoteAddress 192.168.16.33 -Action Allow -Profile Public
```
-Profile は「イーサネット」のネットワークプロファイルに合わせてください（Get-NetConnectionProfile で確認できます。Public なら -Profile Public）。


### 2.2 【Windows側】アダプタ（RPCサーバー）起動
server.py実行で、dllをロード、ZeroMQのREPソケットをバインドし、 待機状態に入り、Linux側のrun_synchronization.pyからの最初のconnectリクエストが届くのを待つ。Linux側からconnectリクエストを受信して初めてVissimを起動し、inpxネットワークをロードする。

<GUI起動>
Linux機のIPアドレスが`192.168.16.33`の場合
```bash
python server.py `
  --bind tcp://192.168.16.56:5555 `
  --vissim-network "..\Vissim\work_muraki\CARLA\Town01.inpx" `
  --vissim-lib-path "C:\Program Files\PTV Vision\PTV Vissim 2026\API\DrivingSimulator_DLL\bin\x64\DrivingSimulatorProxy.dll" `
  --vissim-connect-mode gui `
  --vissim-version 2026 `
  --debug
  ```
<ヘッドレス起動>
```bash
非対応
  ```
| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `--bind ADDR` | ZeroMQのbindアドレス | 省略可 | `tcp://0.0.0.0:5555` |
| `--vissim-network PATH` | 対象の`.inpx`ネットワークファイルのパス(Windows側ローカルパス) | 省略不可 | なし |
| `--vissim-lib-path PATH` | `DrivingSimulatorProxy.dll`のパス | 省略可(省略時はPATH環境変数上から名前で解決) | なし |
| `--vissim-connect-mode {gui,console}` | `gui`: `VISSIM_Connect`でGUI版Vissimインスタンスを起動 / `console`: `VISSIM_ConnectToConsole`でヘッドレスのコンソール版Vissimインスタンスを起動(非対応) | 省略不可 | なし |
| `--vissim-version N` | `VISSIM_Connect`用のVissimバージョン番号(例: Vissim 10なら`1000`) | 省略可(ただし`--vissim-connect-mode gui`指定時は必須) | なし |
| `--vissim-console-path PATH` | Vissimコンソール実行ファイルのパス | 省略可(ただし`--vissim-connect-mode console`指定時は必須) | なし |
| `--debug` | デバッグメッセージを有効化するフラグ | 省略可(フラグ指定のみ、値は取らない) | 指定なし(`False`、無効) |

  --debug を付けておくと tick 毎の挙動が追えます。


### 2.3 【Linux側/ターミナル1】CARLAサーバー起動
CARLAサーバーを起動します。
```bash
cd ~/CARLA
./CarlaUE4.sh
```

### 2.4 【Linux側/ターミナル2】マップの設定 → 信号マッピングjsonの生成
Town01マップを読み込んだうえで、そのマップの信号機情報からdata/signal_mapping.jsonを生成します。

#### 2.4.1 CARLAで使用するマップの設定

```bash
cd ~/CARLA/PythonAPI/util
source ~/carla310_env/bin/activate
python3 config.py --map Town01
```
| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `-m`, `--map <MAP>` | 稼働中のCARLAサーバーにロードするマップ名(例: `Town01`)を指定 | 省略可(省略時はマップ変更を行わない) | なし |

#### 2.4.2 信号マッピングjsonの生成
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

### 2.5 【Linux側/ターミナル3】ビルド

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

### 2.6 【Linux側/ターミナル3】Vissim/CARLA/Autoware起動
autoware_launch経由でAutoware本体とCARLAインタフェース(autoware_carla_interface)を起動し、CARLA上にEGO車両をスポーンさせた上でVissimとのco-simulation(車両同期・信号同期)を開始します。

ROS_DOMAIN_IDは、使用するPCのIPアドレスの末尾の値を設定する運用とします。  
例：IPアドレスが 192.168.16.33 の場合  
`ROS_DOMAIN_ID=33`

```bash
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_vissim:=true \
  vissim_adapter_host:=192.168.16.56 \
  vissim_adapter_port:=5555 \
  vissim_connect_timeout_ms:=120000 \
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
| `vissim_adapter_host` | Windows側Vissimアダプタ(`server.py`)のIPアドレス | 省略可 | `127.0.0.1` |
| `vissim_adapter_port` | Windows側Vissimアダプタ(`server.py`)のTCPポート | 省略可 | `5555` |
| `vissim_connect_timeout_ms` | Vissimアダプタへの初回接続・再接続時のタイムアウト(ms)。Vissim GUI起動に数十秒かかりうるため長めに設定されている | 省略可 | `60000` |
| `vissim_rpc_timeout_ms` | 接続確立後、毎tickのVissimアダプタへのリクエストのタイムアウト(ms) | 省略可 | `2000` |
| `vissim_simulator_vehicles` | Vissim側で同時にトラッキングされるDriving Simulator(CARLA発生)車両の最大数(既定1=EGOのみ) | 省略可 | `1` |
| `sync_traffic_lights` | 信号機の状態をVissimからCARLAへ同期する(Vissim→CARLA方向のみ) | 省略可 | `false` |
| `spectator_follow` | CARLAスペクテーター(自由視点カメラ)をEGO車両のスポーンと同時に自動追従させる | 省略可 | `false` |

**注意**:
- `fixed_delta_seconds`(CARLA)とVissimネットワークファイル(`.inpx`)側のシミュレーションステップ
  時間(`simRes`)は**必ず一致させること**(既定はいずれも0.05秒。`.inpx`側は`simRes=20`に設定すること)。


