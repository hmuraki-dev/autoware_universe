# Vissim(windows)-CARLA-Autoware co-sim 起動手順

## 0. 前提条件
- Autoware環境を構築済みであること。環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。
- .inpxネットワークの設定で「ドライブシミュレータ アクティブ(Driving Simulator active)」 が有効になっていること。この設定が無効だと実質的に何も同期しない。
- .inpxのシミュレーション期間(`simPeriod`)・シミュレーション分解能(`simRes`)・実行回数(`numRuns`)は**設定不要**。これらはLinux側(Autoware側)の起動パラメータで管理され、Windows側アダプタが起動時に.inpxのコピーへ書き込む(2.2・2.6.1参照)。
- CARLAリポジトリ(Windows側アダプタ・`PTV-Vissim_windows/`)と本リポジトリ(`autoware_carla_interface`)は、**通信プロトコルの版(`PROTO_VERSION`)が一致する組み合わせで使うこと**。片方だけ新しくすると、接続時に「プロトコルバージョン不一致」などのエラーになる。

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

**シミュレーション期間・分解能について**:
- アダプタはLinux側からconnectリクエストを受け取ると、`--vissim-network`で指定した.inpxと**同じフォルダ**に`<元のファイル名>.cosim.inpx`(例: `Town01.inpx` → `Town01.cosim.inpx`)を作り、そのコピーでVissimを起動する。コピーでは`<simulation>`要素の次の3つの値だけが書き換わる(元の.inpxは変更されない)。
  - `simPeriod`: Linux側の`vissim_sim_period`(2.6.1参照) + 余裕10秒。ウォームアップを使う場合は、さらに`vissim_warmup_time`と`ego_spawn_wait_timeout`を足した値(2.6.3参照)
  - `simRes`: Linux側の`fixed_delta_seconds`から求めた値(`1 / fixed_delta_seconds`。既定の0.05秒なら20)
  - `numRuns`: 常に1
- このため、**.inpxのあるフォルダには書き込み権限が必要**(書き込めないとconnectがエラーになる)。`.cosim.inpx`は起動のたびに上書きされる。
- `--vissim-network`には元の.inpxを指定すること(`.cosim.inpx`を指定するとエラーになる)。
- Linux側の`vissim_sim_period`/`fixed_delta_seconds`(ウォームアップを使う場合は`vissim_warmup_time`/`ego_spawn_wait_timeout`も)を変えて起動し直すときは、**アダプタ(server.py)とVissimも起動し直すこと**。アダプタは起動中のVissimに新しい値を反映できないため、値が変わった状態で再接続するとconnectがエラーになる。


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
#### 2.6.1 起動コマンド
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
  vissim_sim_period:=600 \
  sync_traffic_lights:=true \
  vissim_warmup_time:=100 \
  spawn_point:="229.8,-2.0,0.3,0.0,0.0,180.0" \
  spectator_follow:=true
```

(`spawn_point`の値は書式の例。実際の地点は2.6.3「スポーン地点の選び方」に従って決めること)

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
| `vissim_sim_period` | co-simulationのシミュレーション期間(秒、1以上の整数)。経過すると、co-simulationを終了してVissimを閉じ、`e2e_simulator.launch.xml`で起動した全ノードを停止する | 省略可 | `600` |
| `vissim_max_consecutive_failures` | Vissimアダプタとのtickが連続でこの回数失敗したら(タイムアウト・再接続失敗・アダプタのエラー)、期間の途中でも同様に終了する | 省略可 | `3` |
| `spectator_follow` | CARLAスペクテーター(自由視点カメラ)をEGO車両のスポーンと同時に自動追従させる | 省略可 | `false` |
| `vissim_warmup_time` | Vissimだけを先に進める時間(秒、0以上の整数)。0でウォームアップ無効(従来どおり起動直後にEGOをスポーン) | 省略可 | `0` |
| `spawn_point` | EGOのスポーン地点。`x,y,z,roll,pitch,yaw`の6つの数値をカンマ区切りで指定(CARLAの座標、角度は度)。zには自動で+2 mされる | **ウォームアップ時は省略不可**(ランダムスポーンでは判定する位置が決まらないため、起動時エラー) | `None`(ランダム) |
| `ego_spawn_front_margin` | EGOの前端と前方車の後端の間に必要な距離(m) | 省略可 | `20.0` |
| `ego_spawn_rear_margin` | EGOの後端と後方車の前端の間に必要な距離(m) | 省略可 | `20.0` |
| `ego_spawn_wait_timeout` | ウォームアップ後、空きを待つ上限(秒、1以上の整数)。超えたら試験開始失敗として終了する | 省略可 | `60` |

**注意**:
- Vissim側のシミュレーション分解能(`simRes`)は`fixed_delta_seconds`から自動で決まり、.inpxのコピーに書き込まれる(2.2参照)。そのため、以前のように.inpx側の`simRes`を手で合わせる必要はない。
  ただし`use_vissim:=true`のときは、`fixed_delta_seconds`を**1/N秒(Nは1〜20の整数)**にすること(既定の0.05秒はN=20)。それ以外の値では起動時にエラーになる。
- Vissimを先に走らせて交通流を作ってからEGOを投入する場合は、ウォームアップの引数(`vissim_warmup_time`・`spawn_point`など)を追加する(2.6.3参照)。指定しなければ、従来どおり起動直後にEGOをスポーンする。
- `ego_spawn_front_margin`・`ego_spawn_rear_margin`は小数点付きで書くこと(`25`ではなく`25.0`)。整数で書くと型が合わず起動時にエラーになる。

#### 2.6.2 終了時の動作
`use_vissim:=true`のとき、次のいずれかで`autoware_carla_interface`ノードが終了し、それに伴って`e2e_simulator.launch.xml`で起動した**全ノードが停止する**(`autoware_carla_interface.launch.xml`の`on_exit="shutdown"`による)。

| きっかけ | ログ | Vissimの状態 |
| --- | --- | --- |
| `vissim_sim_period`秒が経過した | `Vissim co-simulation period elapsed (<N> ticks), stopping.` | Linux側が終了を指示し(`disconnect`)、Vissimが閉じる |
| Vissimアダプタとのtickが`vissim_max_consecutive_failures`回続けて失敗した | `Error: giving up after <N> consecutive failed vissim adapter tick(s), stopping. ...` | アダプタが応答しない状態のため、Vissimが閉じないことがある(下記参照) |
| ウォームアップを使う場合の試験開始失敗(空きが`ego_spawn_wait_timeout`秒以内にできなかった、など。2.6.3参照) | `Error: test start failed: ...` | Linux側が終了を指示し、Vissimが閉じる。終了コードは1 |
| Ctrl+C、その他の異常終了 | - | 通常はLinux側が終了を指示し、Vissimが閉じる |

期間は、Vissimとのtickが成功した回数で数える(`vissim_sim_period / fixed_delta_seconds`回。既定なら600秒 × 20 = 12000回)。tickに失敗した分は数えないので、実時間やCARLAの時刻とは一致しないことがある。ウォームアップを使う場合は、EGOをスポーンした時点から数える(ウォームアップと空き待ちの時間は含まない)。

`use_vissim:=false`(CARLA単体)の場合は、従来どおり、`autoware_carla_interface`ノードが終了しても他のノードは停止しない。

**連続失敗で停止した場合**: Vissimが異常終了したり、アダプタ(server.py)がDLL呼び出しの中で固まったりしている可能性がある。Windows側でserver.pyを止め(Ctrl+Cで止まらない場合はプロセスを終了する)、Vissimが残っていれば閉じてから、2.2の手順でアダプタを起動し直すこと。

#### 2.6.3 ウォームアップ・EGO安全スポーン
Vissimだけを先に走らせて交通流を作り、EGOのスポーン地点の前後が空いたときにEGOを投入する。設計は`docs/Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md`を参照。

`vissim_warmup_time`に1以上を指定すると有効になる(`use_vissim:=true`のときのみ)。起動後の流れは次のとおり。

```text
① EGOの外形の計測   : EGOの車種をスポーン地点の200 m上に一瞬だけスポーンして車長・車幅を読み、すぐ消す
② ウォームアップ    : Vissimだけをvissim_warmup_time秒進める(CARLAは止めたまま、待ち時間なしで可能な限り速く)
③ 一括スポーン      : その間にVissimに入ってきた車両・歩行者を、CARLAへまとめてスポーン
④ 空き待ち          : スポーン地点の前後の車間を判定。空いていなければ1ステップ進めて再判定
                      (ego_spawn_wait_timeout秒以内に空かなければ試験開始失敗)
⑤ EGOスポーン       : EGOをスポーンし、ここからvissim_sim_period秒の検証を開始(以降は通常のco-sim)
```

ROS(Autoware)の時刻は⑤から0で始まる。①〜④の間、Autowareにはセンサーデータが届かない。

起動例と引数は2.6.1を参照。

**注意**:
- Vissimに渡すシミュレーション期間は`vissim_warmup_time + ego_spawn_wait_timeout + vissim_sim_period + 10`秒になる(2.2参照)。これらを変えたら、アダプタとVissimを起動し直すこと。

**スポーン地点の選び方**:
- **直線区間**を選ぶ。前後車の判定は、スポーン地点の向きを基準にした縦・横の位置で行うため、カーブや交差点の中では隣の車線の車を前後車と取り違えたり、前後車を見落としたりすることがある。
- **車線の中心**に置き、向き(yaw)を車線の進行方向に合わせる。交差点の中、または最寄りの車線中心から車線幅の半分以上離れている場合は、起動時に警告が出る。
- 前後100 mの範囲で、同じ向き(±45°以内)・同じ車線(横のずれが車線幅の半分未満)の車両を前後車として判定する。車線に関係なく、EGOの外形と重なる車両がいる場合もスポーンしない。

**必要距離の決め方**:
- 判定はCARLA上の位置と車長で行う。Vissimの車両はCARLAとは車長が違うため、Vissim上の車間はCARLA上より短くなることがある。その差の分だけ必要距離を長めに設定する。
- Town01(2026-10-05変更版)では、前方車について最大+2.73 m(210: トラック)短くなりうる。前方の必要距離は、確保したい車間に3 m程度上乗せする(計画書§6 3-3参照)。
- CARLAにスポーンできなかったVissim車両(vtypes.json未登録など)は、Vissimの位置と仮の車長12.2 m・車幅2.6 mで判定する(安全側)。

**ログの見方**:

```text
[EGO SPAWN CHECK] config: warmup_time=100 s front_margin=23.0 m rear_margin=20.0 m wait_timeout=60 s ...   ← 判定の設定
[EGO SPAWN CHECK] ego vehicle.toyota.prius: length=4.51 m width=2.01 m lane_width=3.50 m (road 12, lane -1)   ← ①
[VISSIM WARMUP] start: warmup_time=100 s (2000 ticks)                                                        ← ②
[VISSIM WARMUP] t=10.0 s vehicles=12 pedestrians=0 (wall 2.1 s)                                             (10秒ごと)
[VISSIM WARMUP] completed: t=100.0 s vehicles=85 pedestrians=4 (wall 19.8 s)
[VISSIM WARMUP] caught up: carla_spawned=83 vissim_only=2 pedestrians=4 pedestrians_vissim_only=0 spawn_retries=3   ← ③
[EGO SPAWN CHECK] t=100.00 s front=vissim:123/carla:456 clearance=8.4 m rear=... overlap=none result=WAIT   ← ④
[EGO SPAWN CHECK] t=107.40 s front=vissim:125/carla:470 clearance=27.3 m rear=... overlap=none result=SAFE
[EGO SPAWN] t=107.40 s spawn_point=... location=(...) vehicles=87 end_tick=14148 front=... result=SAFE      ← ⑤
```

- `vissim_only`はCARLAにスポーンできなかったVissim車両の数。多い場合は、vtypes.jsonの登録漏れを疑う。
- ③では、信号待ちの車列のように前後が詰まった車両同士がスポーン位置で干渉することがある。その場合は高さをずらして最大10回スポーンし直す
  (`spawn_retries`はその回数)。このとき`ERROR:root:Spawn carla actor failed. Spawn failed because of collision at spawn position`が出るが、
  直後の`WARNING:root:[sync] catch-up spawn of ... needed N retries (spawned ... m higher)`が出ていれば、スポーンし直して成功している
  (`gave up`の場合は失敗で、その車両は`vissim_only`に数えられる)。
- `[EGO SPAWN CHECK]`の判定ログは、最初・結果や前後車が変わったとき・1秒(Vissim時間)ごと・SAFEのときに出る。
- `t=`はVissimの時刻(秒)。`end_tick`はco-simを終了するVissimのtick数。

**試験の記録として残すもの**:

| 項目 | どこにあるか |
| --- | --- |
| 判定の設定(ウォームアップ時間・必要距離・上限時間・車種・スポーン地点) | `[EGO SPAWN CHECK] config:`の行 |
| EGOの車長・車幅、スポーン地点の車線 | `[EGO SPAWN CHECK] ego`の行 |
| ウォームアップ終了時刻・ネットワーク内車両数 | `[VISSIM WARMUP] completed:`の行 |
| CARLAにスポーンできなかった車両 | `[VISSIM WARMUP] caught up:`の行(と、その次の行の車両ID) |
| EGOの実スポーン時刻・位置、スポーン時の前後車IDと車間、車両数 | `[EGO SPAWN]`の行 |
| VissimのRandom Seed | Windows側の`.inpx`の`<simulation randSeed="...">`(Linux側では分からないため、使用した`.inpx`ごと保存しておく) |

**試験開始失敗**(終了コード1。`Error: test start failed: ...`と表示され、Vissimを閉じて全ノードが停止する):

| 原因 | 表示 | 対処 |
| --- | --- | --- |
| `ego_spawn_wait_timeout`秒以内に空きができなかった | `no safe gap found within ego_spawn_wait_timeout=... s (t=...): <最後の判定>` | 交通量の少ない地点を選ぶ、上限時間を延ばす、必要距離を見直す |
| ウォームアップ中・空き待ち中に、Vissimアダプタとのtickが連続で失敗した | `giving up after <N> consecutive failed vissim adapter tick(s) during the warmup ...` など | 2.6.2の「連続失敗で停止した場合」と同じ |
| `vehicle_type`に一致する車種がない、またはEGOの外形の計測やスポーンに失敗した | `vehicle_type ... matches no blueprint`、`could not spawn ...`、`failed to spawn the EGO vehicle ...` | `vehicle_type`・`spawn_point`を見直す |

ウォームアップ中・空き待ち中にCtrl+Cを押した場合は、その時点で止まり、通常どおり終了する(試験開始失敗にはならない)。


