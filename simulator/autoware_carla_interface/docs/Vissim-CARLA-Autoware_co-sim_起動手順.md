# Vissim-CARLA-Autoware co-sim 起動手順

# 0. 前提条件
Autoware環境を構築済みであること。
環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。

PTV Vissim Kernel for Linux(`/opt/vissim_kernel_2026.00-10`)を使う場合は、
`libDrivingSimulatorProxy.so`が利用可能であること(ライセンス(CmDongle)が有効であること)。

**(`feature/vissim_windows_co-sim`ブランチ以降: Windowsリモート構成、以下は本手順の前提)**
Vissim(GUI/コンソール)をLinux Kernel版ではなく**Windows機上で動かし、同一LAN内の
ZeroMQ経由でリモート接続する構成**にも対応した(詳細な設計・実装は
`docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md`参照)。以下は本手順が前提と
するこの構成のセットアップ:

- **Windows機**: Vissim(バージョン2026、GUI版またはコンソール版)がインストール・ライセンス
  有効であること。移植元CARLAリポジトリ(`/home/divp/CARLA/Co-Simulation/PTV-Vissim_windows/`)
  の**フォルダ一式をそのままWindows機にコピー**する(このフォルダは自己完結しており、
  `autoware_universe`や他のCARLAコードへの依存を一切持たない。§2.4参照)。Python(推奨:
  Vissimと同じ実行ユーザーの環境)に`pyzmq`・`msgpack`をインストールしておくこと
  (`Co-Simulation/PTV-Vissim_windows/requirements.txt`参照、
  `pip install -r requirements.txt`で一括インストール可能)。
- **Linux機(Autoware/CARLA側)**: Windows機と同一LAN内に接続できること(VPN/WAN跨ぎは
  未サポート)。`autoware_carla_interface`ノードを実行するPython環境(`carla`パッケージを
  `pip install --user`した環境と同じもの)に、`pyzmq`・`msgpack`を追加でインストールして
  おく必要がある:

  ```bash
  python3 -m pip install --user pyzmq msgpack
  ```

  `carla`パッケージ同様、`package.xml`/`setup.py`には登録されていない(rosdep管理外)ため、
  手動でのインストールが必要な点に注意。


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
Vissim関連の`<arg>`(`use_vissim`/`vissim_adapter_host`/`vissim_adapter_port`/
`vissim_connect_timeout_ms`/`vissim_rpc_timeout_ms`/`vissim_simulator_vehicles`/
`sync_traffic_lights`)を追加し、`autoware_carla_interface.launch.xml`
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

### 2.4 Windows側Vissimアダプタ起動(Windows機、`feature/vissim_windows_co-sim`以降の必須手順)

Linux Kernel版(`vissim_lib_path`をローカルの`.so`に向ける旧構成)を使わない場合、Vissim本体は
Windows機で動かし、その隣で薄いZeroMQアダプタ(`Co-Simulation/PTV-Vissim_windows/server.py`)を
起動しておく必要がある。

1. 移植元CARLAリポジトリの`Co-Simulation/PTV-Vissim_windows/`フォルダ一式を、そのままWindows機に
   コピーする(このフォルダは自己完結しており、`autoware_universe`や他のCARLAコードへの依存を
   一切持たない)。
2. Windows機で`pip install -r requirements.txt`(`pyzmq`, `msgpack`)を実行。
3. Windows機で`server.py`を起動する(GUI版Vissimの例):

   ```powershell
   python server.py --bind tcp://0.0.0.0:5555 `
     --vissim-network C:\path\to\Town01.inpx `
     --vissim-connect-mode gui `
     --vissim-version 2026
   ```

   コンソール版(headless)Vissimを使う場合は`--vissim-connect-mode console
   --vissim-console-path <コンソール実行ファイルパス>`を指定する。`--vissim-lib-path`で
   `DrivingSimulatorProxy.dll`の絶対パスを明示することもできる(既定はDLL探索パス任せ)。

**事前チェック(トラブルシューティング、必読)**:

- 対象`.inpx`ネットワーク設定で「ドライブシミュレータ アクティブ(Driving Simulator active)」が
  **有効になっていること**を確認する。この設定が無効なままだと、DS Interfaceへの接続自体は
  成功するが、EGO/NPC双方向の同期が一切起こらないという紛らわしい挙動になる。
- Vissim機とLinux機は**同一LAN内**に置くこと(VPN/WAN越しは現状未サポート、RTT数ms以下が前提)。
- ファイアウォールで、`--bind`のポート(既定`5555`)へのアクセスをLinux機(オーケストレータ)の
  IPだけに絞ることを推奨する(認証・暗号化は同一LAN内前提のため見送り。詳細は
  `Co-Simulation/PTV-Vissim_windows/README.md`の「セキュリティ」節参照)。

### 2.5 Vissim/CARLA/Autoware起動(Linux機、ターミナル3)

```bash
source install/setup.bash
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_vissim:=true \
  vissim_adapter_host:=192.168.16.56 \
  vissim_adapter_port:=5555 \
  vissim_connect_timeout_ms:=60000 \
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
| `vissim_adapter_host` | Windows側Vissimアダプタ(`server.py`)のIPアドレス | `192.168.1.50` | `127.0.0.1` |
| `vissim_adapter_port` | Windows側Vissimアダプタのポート | `5555` | `5555` |
| `vissim_connect_timeout_ms` | 初回接続・再接続時のタイムアウト(ms)。Vissim GUI起動に数十秒かかりうるため長め | `60000` | `60000` |
| `vissim_rpc_timeout_ms` | 接続確立後、毎tickのタイムアウト(ms) | `2000` | `2000` |
| `vissim_simulator_vehicles` | VissimにDriving Simulator車両として同時登録できる最大台数(既定1=EGOのみ) | `1` | `1` |
| `sync_traffic_lights` | 信号同期(Vissim→CARLA一方向のみ) | `true` | `false` |
| `spectator_follow` | EGO車両(role_name=`ego_vehicle_role_name`)にCARLAスペクテーターを自動追従させる | `true` | `false` |

**注意**:

- `use_vissim:=true`と`use_traffic_manager:=true`は**同時指定不可**
  (起動時に`ValueError`で即座に停止する。自動アダプト機構がTraffic Manager由来のNPCも
  無差別にVissimへ登録しようとするため)。
- **(`feature/vissim_windows_co-sim`以降)** SUMO版と同様に、Vissim連携は**Windows側の
  `server.py`を事前に別マシン・別プロセスで起動しておく必要がある**(§2.4)。
  `vissim_connect_timeout_ms`(既定`60000`)以内にWindows側アダプタへの`connect`が成功しないと
  `autoware_carla_interface`起動時に`RuntimeError`で失敗する。
- `fixed_delta_seconds`(CARLA)とVissimネットワークファイル(`.inpx`)側のシミュレーションステップ
  時間(`simRes`)は**必ず一致させること**(既定はいずれも0.05秒。SUMO-CARLA-Autoware連携の実績値。
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


### 2.6 [appendix] 処理時間計測

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
