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

## 2. 起動・終了の手順
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
  - `simPeriod`: Linux側の`vissim_sim_period`(2.6.1参照) + 余裕10秒。ウォームアップを使う場合は、さらに`vissim_warmup_time`と`ego_spawn_wait_timeout`を足した値(3.1参照)
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

### 2.4 【Linux側/ターミナル2】マップの設定 → 信号マッピングjsonの生成 → 車両のブループリント表の生成
Town01マップを読み込んだうえで、そのマップの信号機情報からdata/signal_mapping.jsonを生成します。
Vissimの車両の設定などを変えたときは、Vissimの車両に割り当てるCARLAのブループリントの表も作り直します(2.4.3)。
2.4.2と2.4.3は、どちらも**対象の設定を変えたときだけ**実行する(毎回の起動では不要)。

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
- **マップをロード済みである必要があります**(このコマンドの直前にマップ設定を行うこと)。CARLAが同期モードで誰もtickしていないときは、1回だけtickしてから信号を読みます。
- **`.inpx`側の信号配置を変更した場合のみ再実行すればよく**、毎回の起動時に実行する必要はありません。今の`Town01.inpx`の信号の構成は3.4を参照。
- 横断歩道(歩行者エリアのlink)上の歩行者用signalHeadは、マッピングに入れません(CARLAには歩行者用の信号がないため)。1つのCARLA信号が2つ以上の信号グループに入るときは、ファイルを書き出さずにエラーで止まります。
- 出力は、Linux側で実際に読まれる`autoware_carla_interface`の`vissim_integration/data/signal_mapping.json`にもコピーして、2か所を同じ内容にします。

```bash
cd ~/CARLA/Co-Simulation/PTV-Vissim/util
python3 generate_signal_mapping.py ../examples/Town01/Town01.inpx   # ← 生成物(既定の出力先 ../data/signal_mapping.json)
```

> 生成物: ~/CARLA/Co-Simulation/PTV-Vissim/data/signal_mapping.json # Vissim信号グループ-CARLA信号の対応表

- 出力先は`--output`で変えられる。
- これを`~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface/src/autoware_carla_interface/vissim_integration/data/signal_mapping.json`にもコピーする(上の注意)。

| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `vissim_network`(第1引数) | 対象の`.inpx`ネットワークファイルのパス | 省略不可 | なし |
| `--carla-host H` | CARLAホストサーバーのIPアドレス | 省略可 | `127.0.0.1` |
| `--carla-port P` | CARLAホストサーバーのTCPポート | 省略可 | `2000` |
| `--carla-timeout` | carlaクライアントの接続タイムアウト(秒) | 省略可 | `10.0` |
| `--carla-json` | CARLAサーバーのかわりに、`carla_tl_to_lanelet2.py --dump-json`の出力(`~/autoware_map/Town01/carla_tl.json`)から信号の位置を読む | 省略可 | なし(CARLAサーバーから読む) |
| `--max-distance` | この距離(メートル)より遠いマッチングは低信頼(low-confidence)として警告ログに出力され、手動確認が必要になる | 省略可 | `10.0` |
| `--output` | 生成するマッピングjsonの出力先パス | 省略可 | `../data/signal_mapping.json` |
| `--debug` | デバッグメッセージを有効化するフラグ | 省略可(フラグ指定のみ、値は取らない) | 指定なし(`False`、無効) |

#### 2.4.3 Vissimの車両のCARLAブループリント表の生成
Vissimの車両1台ごとに、どのCARLAのブループリントを使うかを決める表(`vissim_integration/data/carla_variants_town01.yaml`)を作ります。
仕組みと今のTown01の割り当ては3.3を参照。

**注意**:
- **次のどれかを変えたときだけ実行すればよく**、毎回の起動時に実行する必要はありません。今の表はリポジトリに入っています。

  | 変えたもの | 実行する手順 |
  | --- | --- |
  | Vissimの車両の設定(`.inpx`の2D/3Dモデル・モデル分布・車両タイプ・車両構成) | (2)(3) |
  | 割り当ての規則(`src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml`) | (2)(3) |
  | CARLAの版(ブループリントの形が変わりうる) | (1)(2)(3) |

- 変えたのに作り直さないと、co-simの起動時(規則・CARLAの版)か、走行中に表にない車が来たとき(`.inpx`)に、`Error: CARLA blueprint assignment: ...`で止まります(3.3.2)。
- `.inpx`は、**Windows PCで実際に読まれるもの(2.2の`--vissim-network`)**をLinuxに写して使います((2)の1)。
- 作り直した表(と、(1)を実行したときは計測結果)は、autoware_universeにコミットします。`--symlink-install`でビルドしていれば、ビルドし直さなくても使われます。

**(1) CARLAのブループリントの外形の計測**(CARLAの版を変えたときだけ)

CARLAだけを起動した状態で行います(ブリッジ・Autowareは起動しない)。各ブループリントを地下に1台ずつ置き、LiDARで外形を測ってから消します。41ブループリントで1分ほどです。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/measure_blueprint_mesh.py \
    --out src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla<版>.json   # ← 生成物
```

> 生成物: src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla<版>.json # CARLAブループリントの外形(LiDARで測った長さ・幅)一覧

`<版>`は、計測するCARLAサーバーの版。CARLAを起動した状態で、次のコマンドで確かめられる。

```bash
python3 -c "import carla; c = carla.Client('localhost', 2000); c.set_timeout(10); print(c.get_server_version())"
```

CARLAの版が0.9.15のときのコマンド:

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/measure_blueprint_mesh.py \
    --out src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json   # ← 生成物
```

> 生成物: src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json # CARLAブループリントの外形(LiDARで測った長さ・幅)一覧

- 0.9.15の計測結果は、すでにリポジトリに入っている(2026-11-16に測ったもの)。出力先のファイルがあるとツールはエラーで止まるので、0.9.15で測り直すときは、先に古いファイルを消すか、日付を付けた別の名前(例: `mesh_measurements_carla0.9.15_20261201.json`)にする。別の名前にしたときは、(2)の`--geometry`もその名前にする。
- ブリッジが照合するのは、ファイル名ではなく、ファイルの中に記録された版(`carla_server_version`。ツールが接続先のCARLAから書き込む)。

| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `--out` | 計測結果(JSON)の出力先。すでにあるとエラー | 省略不可 | なし |
| `--host` / `--port` | CARLAサーバー | 省略可 | `localhost` / `2000` |
| `--only` | 測るブループリントを限る(例: `--only vehicle.lincoln.mkz_2017`) | 省略可 | すべて |
| `--exclude` | 測らないブループリント(fnmatchの形、複数可) | 省略可 | なし |

- CARLAが同期モードで誰もtickしていないときは、ツールが自分でtickする。ブリッジが動いているときは実行しない。
- 結果を使うには、(2)の`--geometry`にこのファイルを渡す。

**(2) 表の生成**

Windows PCで実際に読まれる`.inpx`(2.2の`--vissim-network`で指定しているもの)をLinuxに写し、それを入力にして表を作ります。

1. 【Windows側/PowerShell】`.inpx`をLinuxへコピーする(LinuxはSSHを受け付けている。IPアドレスとユーザー名は使うPCに合わせる)。

   ```powershell
   scp C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town01.inpx divp@192.168.16.33:/tmp/Town01.inpx   # ← 生成物(Linux側)
   ```

   > 生成物: /tmp/Town01.inpx # Windows PCのVissimネットワークファイルの写し(Linux側。2の入力)

   - USBメモリや共有フォルダで写してもよい。Linux側の置き場所は`/tmp/Town01.inpx`とする(下のコマンドに合わせる)。
   - Vissimで`.inpx`を開いている間に保存したばかりのときは、保存が終わってから写す。

2. 【Linux側/ターミナル2】表を生成する。

   ```bash
   cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
   python3 tools/carla_fleet/build_carla_variants.py \
       --inpx /tmp/Town01.inpx \
       --inpx-ref "Windows PC C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town01.inpx" \
       --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json \
       --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml \
       --out src/autoware_carla_interface/vissim_integration/data/carla_variants_town01.yaml   # ← 生成物
   ```

   > 生成物: src/autoware_carla_interface/vissim_integration/data/carla_variants_town01.yaml # Vissim車両-ブループリント割り当て一覧

   - `--inpx-ref`には、写す前のWindows側のパスを書く。表には`/tmp`のパスの代わりにこれが記録される(`.inpx`のsha256は自動で記録される)。
   - (1)で別の名前の計測結果を作ったときは、`--geometry`をその名前にする。

- Windowsの`.inpx`を変えたときは、CARLAリポジトリの`Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx`にもコミットしておく(リポジトリの版をWindowsと同じに保つため。表の生成そのものには不要)。
- 今の表は、sha256が`2265ff82…`の`.inpx`(2026-11-16。CARLAリポジトリ`feature/vissim_windows`の`4213552`の版と同じ)から作ったもの。`.inpx`・規則・外形のどれも変えていなければ、作り直しても割り当ては変わらない(`generated_at`と`sources.inpx`の記録だけが変わる)。

| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `--inpx` | Vissimのネットワークファイル | 省略不可 | なし |
| `--inpx-ref` | `.inpx`の出どころの記録(例: Windows側のパス)。渡すと、表には`--inpx`のパスの代わりにこれを書く(`/tmp`などに写したファイルを渡すときに使う) | 省略可 | なし(`--inpx`のパスを書く) |
| `--geometry` | (1)の計測結果 | 省略不可 | なし |
| `--rules` | 割り当ての規則 | 省略不可 | なし |
| `--out` | 表の出力先 | 省略不可 | なし |
| `--name` / `--generated-at` | 表の名前 / 生成日時(作り直しの差分を見るときに固定する) | 省略可 | `--out`のファイル名 / 今 |

- CARLAサーバーは要らない。
- 割り当てられないモデルがあると、`[build_carla_variants] 止めた: variant_model_empty: ...`と出て、ファイルを書かない。規則を見直すか、`.inpx`でそのモデルの割合を0にする。
- 歩行者の車両タイプと、自車(Driving Simulator)の車両タイプは表に入らない。割合0のモデルは`skipped`に入る。

**(3) 確認**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
git diff --stat src/autoware_carla_interface/vissim_integration/data/carla_variants_town01.yaml
python3 -m pytest test/test_carla_fleet.py test/test_build_carla_variants.py test/test_vissim_fleet_plan.py
```

- 表の`assignment`(モデルごとのブループリント・dL・dW)と`expected_bodies_per_100_vehicles`を見て、意図した割り当てか確かめる。
- テストは、今のTown01の割り当てを期待値にしている。割り当てが変わったら、テストの期待値(`test_carla_fleet.py`の`test_package_rules_on_town01_models`、`test_build_carla_variants.py`の`test_town01_variants_pass_the_startup_checks`)も直す。

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

(`spawn_point`の値は書式の例。実際の地点は3.1「スポーン地点の選び方」に従って決めること)

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
| `vissim_carla_variants` | Vissimの車両に割り当てるCARLAのブループリントの表(carla_variants)。相対パスは`vissim_integration/data/`から探す。`legacy_vtypes`を指定すると、以前の`vtypes.json`と乱数での選び方に戻る。3.3参照 | 省略可 | `carla_variants_town01.yaml` |
| `spectator_follow` | CARLAスペクテーター(自由視点カメラ)をEGO車両のスポーンと同時に自動追従させる | 省略可 | `false` |
| `vissim_warmup_time` | Vissimだけを先に進める時間(秒、0以上の整数)。0でウォームアップ無効(従来どおり起動直後にEGOをスポーン) | 省略可 | `0` |
| `spawn_point` | EGOのスポーン地点。`x,y,z,roll,pitch,yaw`の6つの数値をカンマ区切りで指定(CARLAの座標、角度は度)。zには自動で+2 mされる | **ウォームアップ時は省略不可**(ランダムスポーンでは判定する位置が決まらないため、起動時エラー) | `None`(ランダム) |
| `ego_spawn_front_margin` | EGOの前端と前方車の後端の間に必要な距離(m) | 省略可 | `20.0` |
| `ego_spawn_rear_margin` | EGOの後端と後方車の前端の間に必要な距離(m) | 省略可 | `20.0` |
| `ego_spawn_wait_timeout` | ウォームアップ後、空きを待つ上限(秒、1以上の整数)。超えたら試験開始失敗として終了する | 省略可 | `60` |
| `lanelet2_map_file` | Autowareが読み込むLanelet2地図のファイル名(`map_path`内)。信号を使うときは信号入りの地図(3.2.1) | 省略可 | `lanelet2_map.osm` |
| `traffic_light.publish` | CARLAの信号状態をAutowareへ直接通知する(カメラの信号認識は起動しない)。3.2参照 | 省略可 | `false` |
| `traffic_light.force_green` | 起動時に全信号を緑にして固定する。`sync_traffic_lights:=true`とは同時に使えない(起動時エラー)。3.2.3参照 | 省略可 | `false` |

**注意**:
- Vissim側のシミュレーション分解能(`simRes`)は`fixed_delta_seconds`から自動で決まり、.inpxのコピーに書き込まれる(2.2参照)。そのため、以前のように.inpx側の`simRes`を手で合わせる必要はない。
  ただし`use_vissim:=true`のときは、`fixed_delta_seconds`を**1/N秒(Nは1〜20の整数)**にすること(既定の0.05秒はN=20)。それ以外の値では起動時にエラーになる。
- Vissimを先に走らせて交通流を作ってからEGOを投入する場合は、ウォームアップの引数(`vissim_warmup_time`・`spawn_point`など)を追加する(3.1参照)。指定しなければ、従来どおり起動直後にEGOをスポーンする。
- `ego_spawn_front_margin`・`ego_spawn_rear_margin`は小数点付きで書くこと(`25`ではなく`25.0`)。整数で書くと型が合わず起動時にエラーになる。

### 2.7 【Linux側/ターミナル2】終了
#### 2.7.1 止め方
`vissim_sim_period`秒が経つと、自動で止まる(2.7.2)。途中で止めるときは、**別のターミナルで次を実行する**(2.6.1を起動したターミナルでCtrl+Cを押さない)。

```bash
pkill -INT -f "lib/autoware_carla_interface/autoware_carla_interface --ros-args"
```

- `autoware_carla_interface`が、CARLAの車両・センサーの片付けと、Vissimアダプタへの`disconnect`を済ませてから終了する。Windows側のVissimはここで閉じる。
- そのあと、`e2e_simulator.launch.xml`で起動した全ノードが自動で止まる(`autoware_carla_interface.launch.xml`の`on_exit="shutdown"`)。
- 止まったことは、2.6.1のターミナルのログで確かめる: `Cleanup complete.`と`process has finished cleanly`。
- 次に起動するときは、2.2の手順でWindows側のアダプタを起動し直す(Vissimを閉じると、アダプタも終了する)。

**注意(Ctrl+Cで止めた場合)**: `e2e_simulator.launch.xml`全体を止めると、launchは各ノードにSIGINTを送り、5秒後にSIGTERM、さらに10秒後にSIGKILLを送る。
`autoware_carla_interface`は、センサーなどの片付け(数秒)のあと、Vissimアダプタに`disconnect`を送ってVissimが閉じ終わるのを待つ。
そのため、この15秒に間に合わずにSIGKILLされることがある(2026-11-16に確認。ブループリント割り当ての計画書F8)。
その場合は`disconnect`が届かず、**Vissimが開いたまま残る**。Windows側でVissimを手で閉じてから、アダプタを起動し直すこと。

#### 2.7.2 自動で止まるとき
`use_vissim:=true`のとき、次のいずれかで`autoware_carla_interface`ノードが終了し、それに伴って`e2e_simulator.launch.xml`で起動した**全ノードが停止する**(`on_exit="shutdown"`による)。

| きっかけ | ログ | Vissimの状態 |
| --- | --- | --- |
| `vissim_sim_period`秒が経過した | `Vissim co-simulation period elapsed (<N> ticks), stopping.` | Linux側が終了を指示し(`disconnect`)、Vissimが閉じる |
| Vissimアダプタとのtickが`vissim_max_consecutive_failures`回続けて失敗した | `Error: giving up after <N> consecutive failed vissim adapter tick(s), stopping. ...` | アダプタが応答しない状態のため、Vissimが閉じないことがある(2.7.3) |
| ウォームアップを使う場合の試験開始失敗(空きが`ego_spawn_wait_timeout`秒以内にできなかった、など。3.1参照) | `Error: test start failed: ...` | Linux側が終了を指示し、Vissimが閉じる。終了コードは1 |
| ブループリントの表が使えない(起動時)。表がない、作ったときと規則・実メッシュが違う、など。3.3.2参照 | `Error: CARLA blueprint assignment: <code>: {...}` | **Vissimアダプタに接続する前に止まる**ので、Vissimは起動しない。終了コードは1 |
| 走行中に、表にないVissimのモデルが来た(.inpxを変えたのに表を作り直していない、など)。3.3.2参照 | `Error: CARLA blueprint assignment: npc_model_unknown: {...}` など | Linux側が終了を指示し、Vissimが閉じる。終了コードは1 |
| Ctrl+C、その他の異常終了 | - | 通常はLinux側が終了を指示し、Vissimが閉じる。ただし、2.7.1の注意を参照 |

期間は、Vissimとのtickが成功した回数で数える(`vissim_sim_period / fixed_delta_seconds`回。既定なら600秒 × 20 = 12000回)。tickに失敗した分は数えないので、実時間やCARLAの時刻とは一致しないことがある。ウォームアップを使う場合は、EGOをスポーンした時点から数える(ウォームアップと空き待ちの時間は含まない)。

`use_vissim:=false`(CARLA単体)の場合は、従来どおり、`autoware_carla_interface`ノードが終了しても他のノードは停止しない。

#### 2.7.3 連続失敗で止まったとき
Vissimが異常終了したり、アダプタ(server.py)がDLL呼び出しの中で固まったりしている可能性がある。Windows側でserver.pyを止め(Ctrl+Cで止まらない場合はプロセスを終了する)、Vissimが残っていれば閉じてから、2.2の手順でアダプタを起動し直すこと。

## 3. 参考

2章の手順で使う機能の説明と、今の設定の記録。起動のたびに読む必要はない。

### 3.1 ウォームアップ・EGO安全スポーン
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
| ウォームアップ中・空き待ち中に、Vissimアダプタとのtickが連続で失敗した | `giving up after <N> consecutive failed vissim adapter tick(s) during the warmup ...` など | 2.7.3「連続失敗で止まったとき」と同じ |
| `vehicle_type`に一致する車種がない、またはEGOの外形の計測やスポーンに失敗した | `vehicle_type ... matches no blueprint`、`could not spawn ...`、`failed to spawn the EGO vehicle ...` | `vehicle_type`・`spawn_point`を見直す |

ウォームアップ中・空き待ち中にCtrl+Cを押した場合は、その時点で止まり、通常どおり終了する(試験開始失敗にはならない)。

### 3.2 CARLAの信号状態をAutowareへ直接通知する

CARLAが管理している信号の状態(赤・黄・青)を、カメラ画像による信号認識を使わずに、
ブリッジ(`autoware_carla_interface`)から`/perception/traffic_light_recognition/traffic_signals`へ
毎tick publishする機能。設計と確認結果は`docs/Vissim_CARLA_Autoware_信号状態publish_実装計画_v1.0.md`を参照
(SUMO構成の`feature/sumo_co-sim`から移植したもの)。
`sync_traffic_lights:=true`のときは、Vissimが決めた信号状態がCARLA経由でそのままAutowareに届く
(Vissim→CARLA→Autoware。CARLAの状態をpublishするのは、Vissimの状態をCARLAに書き込んだ次のtickの後)。

#### 3.2.1 信号入りのLanelet2地図

標準のCARLA用Lanelet2地図(`~/autoware_map/Town01/lanelet2_map.osm`)には信号の情報がないため、
そのままではAutowareが信号で止まらない。CARLAから信号を読み取って地図に追加したものを使う。

- Town01は作成済み: `~/autoware_map/Town01/lanelet2_map_with_tl.osm`(36信号、停止線を各信号とも交差点の入口の1.0 m手前に置いたもの。2026-11-16)。
  同じフォルダの`carla_tl.json`は、地図を作ったときのCARLAの信号の記録(各信号の交差点`group`を含む)。
  Vissimの車用signalHeadも同じ停止線の位置にあるので(3.4)、EGOとVissimの車がほぼ同じ位置で止まる。
- 作り直すときは、CARLAサーバーでTown01を読み込んだ状態で次を実行する(出力は元の地図と同じフォルダ。すでにある場合は`--force`)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/Town01/lanelet2_map.osm --dump-json ~/autoware_map/Town01/carla_tl.json --stop-line-before-junction 1.0   # ← 生成物(2つ)
```

> 生成物: ~/autoware_map/Town01/lanelet2_map_with_tl.osm # 信号入りのLanelet2地図
>
> 生成物: ~/autoware_map/Town01/carla_tl.json # CARLAの信号の記録(位置・停止位置・交差点group)

- `lanelet2_map_with_tl.osm`は既定の出力先(入力の地図と同じフォルダ)で、`-o`/`--output`で変えられる。`carla_tl.json`は`--dump-json`の出力。

- CARLAなしで作り直すときは、`--dump-json ...`のかわりに`--from-json ~/autoware_map/Town01/carla_tl.json`を付ける。
- 停止線の位置を変えたときは、VissimのsignalHeadも`align_signal_heads.py`で移し直す(3.4)。

- ツールの詳細(停止線をずらす量、`--group-id-offset`、Town05・Town10HD)はREADMEの「Adding traffic lights to the CARLA Lanelet2 maps」を参照。

#### 3.2.2 起動とログの確認

2.6.1の起動コマンドに、次の2つを足す(`sync_traffic_lights:=true`はそのまま)。

```bash
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  traffic_light.publish:=true \
```

- ブリッジが信号を地図と対応付けるときの地図(`traffic_light.map_path`)は、既定でAutowareが読み込む地図と同じになる(指定不要)。
- EGOのスポーン後(ウォームアップを使う場合は`[EGO SPAWN] ... result=SAFE`の後)に、次のログが1回出る。

```text
Traffic-light matching: 36 matched, 0 ambiguous, 0 too far, 0 overridden (map has 36 heads / 36 groups)
```

- `ambiguous`・`too far`が0でないときは、その信号はpublishされない(ログの次の行に信号のIDが出る)。
- 確認: `ros2 topic info /perception/traffic_light_recognition/traffic_signals --verbose`のpublisherが
  `autoware_carla_interface`の1つだけであること(カメラの信号認識は起動しない)。
- ウォームアップ中はpublishしない(EGOのスポーン後のループから始まる)。

#### 3.2.3 オプション

| オプション | 説明 | 既定値 |
| --- | --- | --- |
| `traffic_light.publish` | 信号状態をpublishする。`true`のとき、カメラの信号認識(ブリッジ側の画像の中継ノードとAutoware側の信号認識)は起動しない | `false` |
| `traffic_light.force_green` | 起動時に全信号を緑にしてfreezeし、終了時に解除する。**`use_vissim:=true sync_traffic_lights:=true`とは同時に使えない**(Vissimが毎tick信号を上書きするため。起動時に`Error: invalid parameters: ...`で止まる) | `false` |
| `traffic_light.map_path` | 位置で対応付けるLanelet2地図。`""`にするとOpenDRIVEの信号IDをそのままグループIDに使う | Autowareが読む地図 |
| `traffic_light.match_distance` | 対応付けを認める信号灯どうしの最大距離(m) | `5.0` |
| `traffic_light.match_ratio` | 曖昧と判定するしきい値(小さいほど厳しい) | `0.6` |
| `traffic_light.id_map` | 手動の対応付け`opendrive_id:group_id[\|group_id...],...`。位置の対応付けより優先する(`\|`を含むので引用符で囲む) | `""` |

#### 3.2.4 注意点

- **信号の構成**: Vissimの信号は交差点ごとのSignal Controllerで制御する(3.4)。オフセットは全交差点0なので、全交差点が同じ時刻に切り替わる
  (交差点の中では、主道路と従道路が交互に青になる)。
- **黄はAMBERとして届く**: Vissimの`AMBER`/`FLASHING_AMBER`はCARLAの黄になり、そのまま黄としてpublishされる
  (SUMO構成では1つの信号に状態の違うリンクがあると赤になったが、Vissimでは起きない)。黄で止まるか通過するかはAutowareの判断
  (`traffic_light.param.yaml`の`enable_pass_judge`など)。
- **`sync_traffic_lights:=false`のとき**: Vissimの車はVissimの信号に、EGOはCARLAの信号に従うので、両者が食い違う。
  Autowareに信号を届けるときは`sync_traffic_lights:=true`を基本にする。
- **CycloneDDS**: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`で起動する(`~/.bashrc`で設定済みなら不要)。Fast DDSではノードの発見が一部しかできず、
  Autoを押せなかった(SUMO構成で確認)。
- **`carla310_env`を有効にしたターミナルで起動しない**: `transforms3d`がなく、ブリッジが起動直後に落ちる(`deactivate`してから起動する)。
- **確認用のツール**(どれもCARLA・ROSの読み取りだけ):
  - `tools/check_tl_topic_vs_carla.py --duration 300 --carla-json ~/autoware_map/Town01/carla_tl.json`: トピックとCARLAの信号状態を突き合わせ、
    交差点ごとに全信号が同時に青になっていないかも見る(ブリッジの実行中)。
    ブリッジだけを起動して確かめるとき(`autoware_carla_interface.launch.xml`)は、Autoware本体は起動しない。ログに`Traffic-light matching`が出たら、そのままこのスクリプトを実行する。
  - `tools/check_tl_topic_vs_carla.py --frozen-only`: 終了後にfreezeが残っていないか数える(ブリッジを止めてから。必要なら1回tickする)。
  - `tools/monitor_tl_stop.py --map <信号入りの地図> --group 372`: 1つの信号に対するEGOの停止・発進を記録する(Autoware全体の実行中)。
  - `tools/check_vissim_lanelet2_tl_ids.py <signal_mapping.json> <信号入りの地図> --carla-json <carla_tl.json> --inpx <Town01.inpx>`:
    Vissimのマッピング・Lanelet2地図・CARLAの信号がそろっているか、1つのSCが1つの交差点か、signalHeadとマッピングが合っているかを確かめる(CARLA・ROSなしで動く)。
    `--report-stop-offset`で、signalHeadとLanelet2の停止線の距離を出す。
- **確認済みの地点**(Town01): `spawn_point:="334.74,263.00,0.30,0.0,0.0,90.0"`(道路19、信号372の停止線の約54 m手前)。
  赤で停止線の0.4 m手前(交差点の入口の約1.4 m手前)に、Vissimの車とほぼ同じ位置で止まり、青の約3.5秒後に発進すること、
  停止線の約24 m手前・5.5 m/sで黄になったときに止まること、目的地 (374.92, -330.61) に着くことを確認した(2026-11-16)。

### 3.3 Vissimの車両のCARLAブループリント

Vissimの車両1台をCARLAに出すとき、どのブループリントを使うかを、**Vissimの2D/3Dモデルの寸法に最も近いもの**に決める。
設計と確認結果は`docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md`を参照(移植元: SBIR_simのブループリント割り当て処理)。

- 走行中には選ばない。生成ツールで事前に作った表(carla_variants)を、ブリッジは起動時に検査し、走行中は引くだけ。乱数は色とドライバーにだけ使う(車両IDで決まる)。
- 寸法の基準は、CARLAのブループリントを**LiDARで測った外形**(バウンディングボックスではない)。配置もその外形の中心で合わせる。
- 1体で表せない大型車(トラック・トラクター・バス)は、`carlacola`を2〜3体重ねて全長を合わせる(重ね置き)。
- 表にないモデルが来たら、黙って別のブループリントに替えずに止める。

#### 3.3.1 Town01の割り当て

`vissim_integration/data/carla_variants_town01.yaml`(Windows PCの`Town01.inpx`(sha256 `2265ff82…`。CARLAリポジトリ`4213552`の版と同じ)から作成)。

| Vissimのモデル | CARLAのブループリント | 長さの差 |
| --- | --- | --- |
| 1 Fiat 500 | `nissan.micra` | +0.01 m |
| 2 Tesla Model Y、3 VW Golf | `audi.tt` | −0.05 m、−0.06 m |
| 4 Mercedes B Class、8 Nissan Qashqai、9 VW Tiguan | `mini.cooper_s_2021` | +0.09 m、+0.18 m、0.00 m |
| 5 Mercedes C Class、6 Skoda Superb Combi | `tesla.model3` | 0.00 m、−0.03 m |
| 7 Audi A6 Avant | `dodge.charger_2020` | +0.01 m |
| 10 VW ID 4 | `nissan.patrol` | −0.04 m |
| 22 トラクター(Volvo E、5.96 m) | `carlamotors.carlacola` × 2 | 0.00 m |
| 23 トラック(DAF LF、7.93 m) | `carlamotors.carlacola` × 2 | 0.00 m |
| 31 バス(C2 Standard、12.14 m) | `carlamotors.carlacola` × 3 | 0.00 m |
| 70 バイク(Yamaha MT 07) | `kawasaki.ninja` | −0.08 m |

候補から外しているもの: パトカー・タクシー・消防車・救急車の塗装、見た目とLiDARの形が違う`european_hgv`、乗用車に合わないバン(`volkswagen.t2*`)、
自車と同じ`toyota.prius`(`data/carla_fleet/carla_variant_rules.yaml`)。

表の作り直し(Vissimの車両の設定・規則・CARLAの版を変えたときだけ)は、2.4.3の手順で行う。

#### 3.3.2 止まったとき(`Error: CARLA blueprint assignment: <code>: ...`)

| code | 意味 | 直し方 |
| --- | --- | --- |
| `variant_config_missing` | `vissim_carla_variants`のファイルがない | パスを確かめる |
| `variant_source_changed` | 表を作ったときと、規則か実メッシュが違う | 表を作り直す(2.4.3の(2)) |
| `variant_blueprint_missing` | 表のブループリントが、接続先のCARLAにない | CARLAの版を確かめ、外形を測り直して表を作り直す(2.4.3の(1)(2)) |
| `variant_mesh_carla_version_changed` | 外形を測ったCARLAの版と、接続先の版が違う | 外形を測り直して表を作り直す(2.4.3の(1)(2)) |
| `variant_articulated_unsupported` | 表に連結車(軌跡に沿う体)がある。このブリッジは扱わない | .inpxで連結車の割合を0にして作り直す(2.4.3の(2)) |
| `variant_model_ambiguous` | 同じ(車種, モデルファイル)が2つのモデルにある | .inpxのモデルを見直す |
| `npc_model_unknown`(走行中) | 表にない(車種, モデルファイル)の車が来た | Windowsで使っている.inpxから、表を作り直す(2.4.3の(2)) |
| `variant_model_empty`(走行中) | 表を作ったときに割合0だったモデルの車が来た | 同上 |

- 起動時の検査は、Vissimアダプタに接続する前に行う。そのため、起動時に止まったときはVissimは起動していない。
- 以前の選び方(車種ごとの候補から乱数で選ぶ。`vissim_integration/data/vtypes.json`)に戻すときは、`vissim_carla_variants:=legacy_vtypes`を付けて起動する。
- 終了時に、ブループリントごとのspawn回数がログに1行出る(`CARLA bodies spawned for Vissim vehicles: ...`)。

### 3.4 Vissimの信号の構成(Town01)

(参考情報。起動の手順ではない。信号の構成を変えるときは、ここのツールで`.inpx`を作り直し、2.4.2で信号マッピングjsonを作り直す。)

`Town01.inpx`は、CARLAの交差点ごとに1つの信号制御(Signal Controller、Fixed time)を持つ(2026-11-16)。
設計と確認結果は`docs/Vissim_CARLA_Autoware_信号設置_実装計画_v1.0.md`を参照。

| 項目 | 内容 |
| --- | --- |
| 信号制御 | 12個(`J1 360-362`〜`J12 393-395`)。SC番号は交差点の小さいlandmarkの順 |
| 信号グループ | 各SCにSG1(主道路、2信号)とSG2(従道路、1信号) |
| 時間 | 周期94秒。SG1の青0〜42秒、SG2の青47〜89秒。どちらも黄3秒、全赤2秒。オフセットは全SCとも0(全交差点が同じ時刻に切り替わる) |
| 歩行者用 | SC 5(372〜374)のSG3: 372の進入路を横切る横断歩道。青0〜34秒、青点滅34〜38秒。主道路(SG1)とは交差しない |
| signalHeadの位置 | 車用は、進入路のlinkの上で、Lanelet2の停止線(交差点の入口の1.0 m手前、3.2.1)と同じ位置。進入路のすべての進路(connector)の手前にある |
| 投入量 | 車両7か所: 各100台/h、歩行者2か所: 各100人/h |

作り直すときに使うツール(どれもCARLAリポジトリの`Co-Simulation/PTV-Vissim/util/`。`.inpx`は文字列のまま書き換え、改行(CRLF)も変えない):

- `split_signal_controllers.py`: 1つのSCを、CARLAの交差点(`carla_tl.json`の`group`)ごとのSCに複製し、signalHeadの`sg`を書き換える。信号プログラムは`--template-from`で、Vissim GUIで直したSCを雛形にできる。
- `align_signal_heads.py`: 車用signalHeadを、Lanelet2の停止線の位置の進入路linkに移す(`--lanelet2-map`)。
- `generate_signal_mapping.py`: 2.4.2(信号マッピングjsonの生成)。
- 確かめるとき: `autoware_carla_interface`の`tools/check_vissim_lanelet2_tl_ids.py`(3.2.4)。

注意:
- Vissim GUIで`.inpx`を開いてSimulation runを回すときは、Driving Simulatorを無効にする。そのまま保存すると`drivSimActive="false"`になり、co-simのconnectがタイムアウトする。**co-simで使うファイルは`drivSimActive="true"`にしておく**(Vissimが複数起動しているときもタイムアウトする)。
- VisSigでSGを足したときは、同時に青になってよい組(例: 主道路の車と、従道路を渡る歩行者)の中間時間を**空欄**にする(0は「交差する」の意味になり、`intergreen violated`になる)。interstageの終わりに青になるSGの最小青時間は0にする(`minimum green time violated`を避ける)。
