# Vissim(windows)-CARLA(0.9.16)-Autoware co-sim 起動手順

CARLA 0.9.15(`~/CARLA/`)用の「Vissim(win)-CARLA-Autoware_co-sim_起動手順.md」をもとに、CARLAを0.9.16(`~/CARLA_0.9.16_jp/`)に差し替えたときの手順をまとめたもの。
0.9.15版との違いは、下の表と3.1にまとめた。0.9.15版から変わらない機能の詳しい説明(ウォームアップのログの見方、信号の直接通知のオプションなど)は、0.9.15版の3章を参照。

| 項目 | 0.9.15版 | 0.9.16版(この手順書) |
| --- | --- | --- |
| CARLAサーバー | `~/CARLA/CarlaUE4.sh` | `~/CARLA_0.9.16_jp/LinuxNoEditor/CarlaUE4.sh` |
| `get_server_version()`の値 | `0.9.15` | `294096eb1-dirty`(独自ビルドのため、版の番号ではない) |
| CARLAのPythonクライアント | `carla==0.9.15`(`~/carla310_env`、`pip --user`) | `carla==0.9.16`(`pip --user`)。**0.9.15のクライアントは0.9.16のサーバーにつなぐと落ちる**(3.1.1) |
| マップ | Town01 | **Town10HD(`Town10HD_Opt`)**。0.9.16_jpに入っているマップはこれだけ(3.1.2) |
| Autowareの地図フォルダ | `~/autoware_map/Town01` | `~/autoware_map/Town10HD_Opt`(`Town10HD`へのシンボリックリンク。2.0.2) |
| Vissimのネットワーク | `Town01.inpx`(CARLAリポジトリ) | `Town10HD.inpx`(このリポジトリの`vissim/`。3.4) |
| 信号の対応表(`signal_mapping.json`) | Town01用(リポジトリに入っている) | Town10HD用に置き換える(2.4.2) |
| 車両のブループリント表 | `carla_variants_town01.yaml`(0.9.15で測った外形) | `carla_variants_town10hd.yaml`(0.9.16で外形を測り直して作る。2.4.3) |
| 車両 | CARLA標準 | CARLA標準 + 日本の車6台(`vehicle.jp.*`。3.1.4) |

## 0. 前提条件
- Autoware環境を構築済みであること。環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。
- CARLA 0.9.16を`~/CARLA_0.9.16_jp/LinuxNoEditor/`に展開済みであること。
- .inpxネットワークの設定で「ドライブシミュレータ アクティブ(Driving Simulator active)」 が有効になっていること。`Town10HD.inpx`は有効にしてある(Vissimで実行したあとに保存すると無効に戻ることがあるので、保存しない)。
- .inpxのシミュレーション期間(`simPeriod`)・シミュレーション分解能(`simRes`)・実行回数(`numRuns`)は**設定不要**。Linux側(Autoware側)の起動パラメータで管理され、Windows側アダプタが起動時に.inpxのコピーへ書き込む(2.2・2.6.1参照)。
- CARLAリポジトリ(Windows側アダプタ・`PTV-Vissim_windows/`)と本リポジトリ(`autoware_carla_interface`)は、**通信プロトコルの版(`PROTO_VERSION`)が一致する組み合わせで使うこと**。

**Windows機**:
- CARLAリポジトリ(`/home/divp/CARLA/Co-Simulation/PTV-Vissim_windows/`)**フォルダ式をそのままWindows機にコピー**し、`pyzmq`・`msgpack`をインストール済みであること(`pip install -r requirements.txt`)。0.9.15版と同じ(CARLAの版に関係しない)。

**Linux機(Autoware/CARLA側)**:
- Windows機と同一LAN内に接続できること。`autoware_carla_interface`ノードを実行するPython環境に、`pyzmq`・`msgpack`をインストールしておくこと。

  ```bash
  python3 -m pip install --user pyzmq msgpack
  ```


## 1. リポジトリ
以下のブランチを使用する。
このブランチはAutoware Universe(0.52.0)のautoware_carla_interfaceをベースに、CARLA公式のVissim-CARLAブリッジ(PTV Vissim Driving Simulator Interface経由)を盛り込み、fullstack(Vissim-CARLA-Autoware)のco-simを実装したものである。

- `autoware_universe`: `feature/vissim_windows_co-sim`ブランチ
- Town10HDのVissimネットワーク(`Town10HD.inpx`)と信号の対応表は、このブランチの`simulator/autoware_carla_interface/vissim/`にある。
  作り方(OpenDRIVEの取り込みとツール)は`vissim/README.md`。ツールは`feature/sumo_co-sim`ブランチにあり、作り直したものをこのブランチへ写す。

```text
~/autoware.1.9.0/
└── src/
    └── universe/
        └── autoware_universe/      ← GitHubからcloneするリポジトリ
            └── simulator/
                └── autoware_carla_interface/
                    └── vissim/     ← Vissimのネットワーク(Town10HD.inpx)と信号の対応表
```

CARLA関係のフォルダは次のとおり。

```text
~/
├── CARLA_0.9.16_jp/
│   └── LinuxNoEditor/          ← CARLA 0.9.16のサーバー(CarlaUE4.sh)・PythonAPI
├── CARLA_0.9.16_jp_incoming/   ← 受け渡し用のアーカイブと、日本の車の確認ツール
└── CARLA/                      ← CARLA 0.9.15のリポジトリ。Windows側アダプタ(Co-Simulation/PTV-Vissim_windows/)はここのものを使う
```

## 2. 起動・終了の手順

2.0から順に実行する。
- 2.0は、**CARLAを0.9.16に切り替えたときに1回だけ**実行する(毎回の起動では不要)。
- 2.2.0・2.4.2・2.4.3は、**対象を変えたときだけ**実行する(毎回の起動では不要)。

### 2.0 【Linux側/ターミナル2】CARLA 0.9.16に切り替えたときだけ行う準備

#### 2.0.1 CARLAのPythonクライアントを0.9.16にする

```bash
pip3 install --user carla==0.9.16
pip3 show carla | grep Version   # Version: 0.9.16 と出ること
```

- Autowareのブリッジ(`autoware_carla_interface`)・`config.py`・`tools/`のツールは、どれもこの`carla`を使う。
- **`~/carla310_env`は使わない**。中の`carla`は0.9.15で、0.9.16のサーバーにつなぐと落ちる(3.1.1)。0.9.15版の2.4.1は`carla310_env`で`config.py`を実行しているが、この手順書では使わない。
- `~/CARLA_0.9.16_jp/LinuxNoEditor/PythonAPI/carla/dist/`のwheelはPython 3.12用(`cp312`)だけで、Python 3.10では使えない。PyPIの`carla==0.9.16`にはPython 3.10用がある。
- 0.9.15に戻すときは3.5。

#### 2.0.2 Autowareの地図フォルダ`Town10HD_Opt`を作る

```bash
ln -s Town10HD ~/autoware_map/Town10HD_Opt   # ← 生成物
ls ~/autoware_map/Town10HD_Opt/
```

> 📄 **生成物:** `~/autoware_map/Town10HD`へのシンボリックリンク(2.6.1の`map_path`)\
> `~/autoware_map/Town10HD_Opt`

- ブリッジは、`map_path`のフォルダ名をCARLAのマップ名として読み込む。0.9.16_jpのマップは`Town10HD_Opt`だけなので、フォルダ名を`Town10HD_Opt`にする(3.1.2)。
- 中身(信号入りのLanelet2地図`lanelet2_map_with_tl.osm`・点群・`carla_tl.json`)は`Town10HD`のものをそのまま使う(3.2)。
- SUMO版の手順書(「SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md」の2.0.2)ですでに作ってあれば不要。

### 2.1 【Windows側】ファイアウォールの受信規則
0.9.15版の2.1と同じ。管理者として PowerShell を開き、Linux機のIPを <LINUX_IP> に入れて実行する(登録済みなら不要)。

```bash
New-NetFirewallRule -DisplayName "Vissim adapter (ZeroMQ 5555)" `
  -Direction Inbound -Protocol TCP -LocalPort 5555 `
  -RemoteAddress <LINUX_IP> -Action Allow -Profile Private
```

-Profile は「イーサネット」のネットワークプロファイルに合わせる（Get-NetConnectionProfile で確認できる。Public なら -Profile Public）。

### 2.2 【Windows側】アダプタ（RPCサーバー）起動

#### 2.2.0 `Town10HD.inpx`をWindows PCへ写す

`Town10HD.inpx`を変えたときだけ行う。

【Windows側/PowerShell】Linuxからこのリポジトリの`vissim/Town10HD.inpx`を写す(IPアドレスとユーザー名は使うPCに合わせる)。

```powershell
scp divp@192.168.16.33:/home/divp/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface/vissim/Town10HD.inpx C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town10HD.inpx   # ← 生成物(Windows側)
```

> 📄 **生成物:** Windows PCで読み込むVissimネットワーク(2.2.1の`--vissim-network`)\
> `C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town10HD.inpx`

- USBメモリや共有フォルダで写してもよい。置き場所は書き込みのできるフォルダにする(アダプタが同じフォルダに`Town10HD.cosim.inpx`を作るため)。
- Windows側で`.inpx`を直したときは、Linuxへ写し戻して`vissim/Town10HD.inpx`にコミットする(2.4.3の表の入力と同じものにするため)。
  ただし、`vissim/Town10HD.inpx`はツールで作り直すと上書きされる(`vissim/README.md`の2章)。

#### 2.2.1 アダプタの起動

0.9.15版の2.2と同じ。`--vissim-network`だけ`Town10HD.inpx`にする。

<GUI起動>

```bash
python server.py `
  --bind tcp://192.168.16.56:5555 `
  --vissim-network "..\Vissim\work_muraki\CARLA\Town10HD.inpx" `
  --vissim-lib-path "C:\Program Files\PTV Vision\PTV Vissim 2026\API\DrivingSimulator_DLL\bin\x64\DrivingSimulatorProxy.dll" `
  --vissim-connect-mode gui `
  --vissim-version 2026 `
  --debug
```

- オプションと、シミュレーション期間・分解能の扱い(`Town10HD.cosim.inpx`が作られる)は、0.9.15版の2.2を参照。

### 2.3 【Linux側/ターミナル1】CARLAサーバー起動

```bash
cd ~/CARLA_0.9.16_jp/LinuxNoEditor
./CarlaUE4.sh
```

- 0.9.15のサーバー(`~/CARLA/CarlaUE4.sh`)が動いていたら、先に止める(どちらもポート2000を使う)。

### 2.4 【Linux側/ターミナル2】マップの設定 → 信号の対応表の置き換え → 車両のブループリント表の生成

#### 2.4.1 CARLAで使用するマップの設定

```bash
cd ~/CARLA_0.9.16_jp/LinuxNoEditor/PythonAPI/util
python3 config.py --map Town10HD_Opt
```

- `carla310_env`は有効にしない(2.0.1)。有効になっていたら`deactivate`してから実行する。
- CARLAは起動直後から`Town10HD_Opt`を読み込んでいる。ほかのマップ(`OpenDriveMap`など)にしたときに、このコマンドで戻す。

#### 2.4.2 信号の対応表をTown10HD用にする

ブリッジは、`vissim_integration/data/signal_mapping.json`(場所は固定)を読む。Town10HD用の対応表で置き換える。

**注意**:
- **ブリッジは対応表を1つしか置けない**。置き換えると、Town01用の対応表は使えなくなる。Town01に戻すときは`git checkout`で戻す(3.5)。
- 置き換えたファイルはコミットしない(`git status`に変更として出たままになる)。
- 0.9.15版の2.4.2(`generate_signal_mapping.py`)は**使わない**。Town10HDでは誤った対応表になる(3.4)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
cp vissim/signal_mapping_town10hd.json src/autoware_carla_interface/vissim_integration/data/signal_mapping.json   # ← 生成物
cat src/autoware_carla_interface/vissim_integration/data/signal_mapping.json
```

> 📄 **生成物:** Town10HDのVissim信号グループ-CARLA信号の対応表(ブリッジが読むもの)\
> `src/autoware_carla_interface/vissim_integration/data/signal_mapping.json`

- 中身は、5つの信号制御(`"1"`〜`"5"`)ごとに、信号グループ`"1"`・`"2"`のCARLAの信号ID(943〜962の15個)。

#### 2.4.3 Vissimの車両のCARLAブループリント表の生成

Vissimの車両1台ごとに、どのCARLAのブループリントを使うかを決める表を作る。仕組みは3.3、0.9.15版の3.3を参照。

**注意**:
- **0.9.16に切り替えたときは、(1)(2)(3)をすべて行う**。0.9.15で作った表(`carla_variants_town01.yaml`)は、外形を測ったCARLAの版(`0.9.15`)が
  接続先(`294096eb1-dirty`)と違うので、起動時に`Error: CARLA blueprint assignment: variant_mesh_carla_version_changed: ...`で止まる。
- そのあとは、0.9.15版の2.4.3の表のとおり、変えたものに応じて実行する(`Town10HD.inpx`の車両の設定を変えたときは(2)(3))。
- 作った表と計測結果は、autoware_universeにコミットする。`--symlink-install`でビルドしていれば、ビルドし直さなくても使われる。
- 表を作らずに試すときは、2.6.1で`vissim_carla_variants:=legacy_vtypes`にする(以前の選び方。3.3)。

**(1) CARLAのブループリントの外形の計測**

CARLAだけを起動した状態で行う(ブリッジ・Autowareは起動しない)。各ブループリントを地下に1台ずつ置き、LiDARで外形を測ってから消す。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/measure_blueprint_mesh.py \
    --out src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json   # ← 生成物
```

> 📄 **生成物:** CARLAブループリントの外形(LiDARで測った長さ・幅)一覧(0.9.16_jp)\
> `src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json`

- ファイル名の`0.9.16_jp`は見分けるための名前。0.9.16_jpの`get_server_version()`は`294096eb1-dirty`を返し、版の番号にならないため。
  ブリッジが照合するのは、ファイルの中に記録された版(`carla_server_version`。0.9.16_jpでは`294096eb1-dirty`)。
- 日本の車6台(`vehicle.jp.*`)も測られ、割り当ての候補になる(3.1.4)。
- 出力先のファイルがあるとツールはエラーで止まる。SUMO構成(`feature/sumo_co-sim`)で同じファイルを測ってあれば、それを使ってもよい(同じCARLAなら同じ結果)。

**(2) 表の生成**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/build_carla_variants.py \
    --inpx vissim/Town10HD.inpx \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json \
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml \
    --out src/autoware_carla_interface/vissim_integration/data/carla_variants_town10hd.yaml   # ← 生成物
```

> 📄 **生成物:** Vissim車両-ブループリント割り当て一覧(Town10HD)\
> `src/autoware_carla_interface/vissim_integration/data/carla_variants_town10hd.yaml`

- `--inpx`は、Windows PCで読み込む`.inpx`と同じもの(2.2.0)。リポジトリ内のファイルなので`--inpx-ref`は要らない(表には、渡したパス`vissim/Town10HD.inpx`がそのまま記録される。`.inpx`のsha256も記録される)。
- `Town10HD.inpx`の車両は、日本の車のモデル(乗用車3・トラック1・バス2)にしてある(3.4)。モデルファイル名が`vehicle.jp.*`なので、0.9.16の外形では日本の車がそのまま候補に入る見込み。
- トレーラー(モデル22)は連結車にしない(トラクターだけ)。連結車があると、起動時に`Error: CARLA blueprint assignment: variant_articulated_unsupported`で止まる(このブリッジは連結車を扱わない)。

**(3) 確認**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 -m pytest test/test_carla_fleet.py test/test_build_carla_variants.py test/test_vissim_fleet_plan.py
```

- 表の`assignment`(モデルごとのブループリント・dL・dW)を見て、意図した割り当てか確かめる。タクシー(`vehicle.jp.taxi`)など、塗装に意味のある車が割り当たったときは、
  規則(`carla_variant_rules.yaml`)の`excluded`に足して(2)から作り直す。
- テストはTown01の表(`carla_variants_town01.yaml`)を期待値にしているので、Town10HDの表を足しても結果は変わらない。

### 2.5 【Linux側/ターミナル3】ビルド

0.9.15版の2.5と同じ。CARLAを差し替えただけなら、ビルドし直す必要はない(`carla`はPythonのパッケージとして実行時に読み込まれる)。

```bash
source /opt/ros/humble/setup.bash
cd ~/autoware.1.9.0
colcon build --packages-select autoware_carla_interface --symlink-install
source ~/autoware.1.9.0/install/setup.bash
```

### 2.6 【Linux側/ターミナル3】Vissim/CARLA/Autoware起動
#### 2.6.1 起動コマンド

```bash
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town10HD_Opt \
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_vissim:=true \
  vissim_adapter_host:=192.168.16.56 \
  vissim_adapter_port:=5555 \
  vissim_connect_timeout_ms:=120000 \
  vissim_sim_period:=600 \
  sync_traffic_lights:=true \
  traffic_light.publish:=true \
  vissim_carla_variants:=carla_variants_town10hd.yaml \
  vissim_warmup_time:=100 \
  spawn_point:="30.02,133.95,0.30,0.0,0.0,-179.7" \
  spectator_follow:=true
```

0.9.15版の起動コマンドとの違い:

| 引数 | 0.9.15版 | この手順書 | 理由 |
| --- | --- | --- | --- |
| `map_path` | `$HOME/autoware_map/Town01` | `$HOME/autoware_map/Town10HD_Opt` | フォルダ名がCARLAのマップ名になる(2.0.2) |
| `lanelet2_map_file`・`traffic_light.publish` | (3.2.2で足す) | 最初から付ける | Town10HDの信号入りの地図がある(3.2) |
| `vissim_carla_variants` | (既定の`carla_variants_town01.yaml`) | `carla_variants_town10hd.yaml` | 2.4.3で作った表。作っていないときは`legacy_vtypes` |
| `spawn_point` | Town01の地点 | 信号954の手前(CARLAのスポーン地点#80) | Town10HDの地点(3.4) |

- オプションの一覧と注意は、0.9.15版の2.6.1を参照(変わらない)。
- 起動ログで確かめること:
  - `[CARLA Interface] Map Path: '.../Town10HD_Opt' -> Loading CARLA Map: 'Town10HD_Opt'`
  - `CARLA blueprints of Vissim vehicles: .../carla_variants_town10hd.yaml ...`(表を使うとき)
  - `Traffic-light matching: 15 matched, 0 ambiguous, 0 too far, 0 overridden (map has 62 heads / 30 groups)`(EGOのスポーン後。SUMO構成のTown10HDで確認した値)

#### 2.6.2 目的地の設定と自動運転の開始

- ウォームアップを使うときは、ログに`[EGO SPAWN] ... result=SAFE`が出て、RVizにEGOが表示されてから操作する。
- RVizで目的地(2D Goal Pose)を置き、Autoを押す。2.6.1のスポーン地点からは、信号954の交差点を抜けた道路5の上、map座標で(-104, -99)付近(北北西向き)に置く
  (この車線は交差点を抜けると道路5へ曲がって続くだけ。SUMO構成のTown10HDで確認した目的地)。

### 2.7 【Linux側/ターミナル2】終了

0.9.15版の2.7と同じ。途中で止めるときは、**別のターミナルで次を実行する**(2.6.1を起動したターミナルでCtrl+Cを押さない)。

```bash
pkill -INT -f "lib/autoware_carla_interface/autoware_carla_interface --ros-args"
```

- 止まったことは、2.6.1のターミナルのログで確かめる: `Cleanup complete.`と`process has finished cleanly`。Windows側のVissimも閉じる。
- 次に起動するときは、2.2.1の手順でWindows側のアダプタを起動し直す。
- 自動で止まるとき(`vissim_sim_period`秒の経過、連続失敗、試験開始失敗、ブループリント表のエラー)と、連続失敗で止まったときの対処は、0.9.15版の2.7.2・2.7.3を参照。

## 3. 参考

2章の手順で使う機能の説明と、今の設定の記録。起動のたびに読む必要はない。

### 3.1 CARLA 0.9.16(0.9.16_jp)で変わったこと

2026-11-17に、`~/CARLA_0.9.16_jp/LinuxNoEditor`のファイルと、起動したサーバーで確かめた(SUMO版の手順書の3.1と同じ内容)。
**Vissim-CARLA-Autowareのco-simを0.9.16で動かす確認は、まだしていない**(3.1.5)。

#### 3.1.1 Pythonクライアント

- 0.9.15のクライアントで0.9.16_jpのサーバーにつなぐと、`WARNING: Version mismatch detected ... Simulator API version = 294096eb1-dirty`のあと`std::bad_alloc`で落ちる(コアダンプ)。
- サーバーの版(`get_server_version()`)は`294096eb1-dirty`。`VERSION`ファイルも同じ値。

#### 3.1.2 マップ

- `CarlaUE4/Content/Carla/Maps/`のマップは`Town10HD_Opt`・`OpenDriveMap`・`TestMaps/EmptyMap`だけ。Town01は入っていないので、0.9.15版のTown01の手順はそのままでは使えない。
- `Town10HD.xodr`と`Town10HD_Opt.xodr`は、0.9.15・0.9.16の4つとも同じファイル(md5 `570c77d196b918813e95750fcb980214`)。
  そのため、0.9.15で作ったTown10HDのLanelet2地図・信号の対応付け、OpenDRIVEから取り込んだ`Town10HD.inpx`を、そのまま使える。
- ブリッジは`map_path`のフォルダ名をCARLAのマップ名にし、今のマップと名前が違えば読み込み直す。0.9.16_jpには`Town10HD`という名前のマップがないので、フォルダ名を`Town10HD_Opt`にする(2.0.2)。

#### 3.1.3 Windows側

- Windows側(アダプタ`server.py`・Vissim)はCARLAを使わないので、CARLAの版によって変わらない。変わるのは`--vissim-network`の`.inpx`だけ。

#### 3.1.4 日本の車(`vehicle.jp.*`)

- 0.9.16_jpには、CARLA標準の車に加えて、日本の車6台が入っている(一覧は`~/CARLA_0.9.16_jp_incoming/jp_vehicles.json`)。
- Vissimの車は、ブループリント表(2.4.3)で寸法の最も近いブループリントに割り当たる。0.9.16で外形を測ると日本の車も候補に入るので、
  Vissimのモデルの寸法に近ければ日本の車が割り当たる。`legacy_vtypes`(`vtypes.json`)では使われない。

#### 3.1.5 まだ確かめていないこと

| 項目 | 確かめ方 |
| --- | --- |
| 2.6.1の起動コマンドで、Vissimとつながり、EGOがスポーンし、目的地まで自動運転できるか | 2.6.1・2.6.2 |
| 0.9.16で測った外形での割り当て(日本の車が入るか) | 2.4.3の(3) |
| Vissimの信号がCARLA・Autowareに届くか(`Traffic-light matching: 15 matched`) | 2.6.1の起動ログ、0.9.15版の3.2.4の確認用ツール |
| 2.6.1のスポーン地点の車線に、Vissimの車が通るか(ウォームアップの空き待ちの確認になるか) | 2.6.1の起動ログの`[EGO SPAWN CHECK]` |

### 3.2 信号入りのLanelet2地図(Town10HD)

- `~/autoware_map/Town10HD/lanelet2_map_with_tl.osm`(2026-10-21、SUMO構成で作成)。CARLAの15信号、停止位置ごとに30個のregulatory element(IDはOpenDRIVEの信号ID + 100000)。
  停止線はCARLAの停止位置のまま(前に出していない)。
- 同じフォルダの`carla_tl.json`は、地図を作ったときのCARLAの信号の記録(位置・停止位置)。交差点の`group`は入っていない(0.9.15版のTown01の手順の`split_signal_controllers.py`を使うときは、作り直す必要がある)。
- Vissimの信号灯(3.4)も同じCARLAの停止位置に置いてあるので、EGOとVissimの車がほぼ同じ位置で止まる。
- 作り直すときの手順は、SUMO版の手順書(「SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md」)の2.2.2。

### 3.3 Vissimの車両のCARLAブループリント

- 仕組みは0.9.15版の3.3と同じ(Vissimの2D/3Dモデルの寸法に最も近いブループリント。大型車は`carlacola`の重ね置き)。
- 表は、走行中にVissimの(車種, モデルファイル)で引く。`Town10HD.inpx`の車両は、日本の車のモデル(乗用車3・トラック1・バス2)、トレーラー(トラクターだけ)、バイク(3.4)。
- 止まったとき(`Error: CARLA blueprint assignment: <code>: ...`)の直し方は、0.9.15版の3.3.2を参照。0.9.15で作った表を0.9.16で使うと`variant_mesh_carla_version_changed`になる(2.4.3)。

### 3.4 Vissimのネットワーク(Town10HD)

`vissim/Town10HD.inpx`は、Windows PCのVissim 2026でTown10HDのOpenDRIVEを取り込んだ道路網(`vissim/Town10HD_road.inpx`)に、ツールで信号・交通需要・車両設定を足したもの(2026-11-17)。
作り方とSUMO構成との違いは`vissim/README.md`を参照(ツールは`feature/sumo_co-sim`ブランチ)。

| 項目 | 内容 |
| --- | --- |
| 道路網 | リンク111本(1車線54・2車線57)、コネクタ148本、conflictArea 228個(OpenDRIVEの取り込みのまま) |
| 信号制御 | 5個(SUMOの信号189・23・468・532・719に1つずつ)。中身はTown01.inpxと同じ固定時間の制御: 周期94秒、SG1の青0〜42秒、SG2の青47〜89秒、黄3秒、全赤2秒、オフセット0 |
| 信号灯 | 30個(CARLAの15信号の停止位置 × 2車線)。CARLAの信号は、SUMOの最初のフェーズで青ならSG1、赤ならSG2 |
| 信号の対応表 | `vissim/signal_mapping_town10hd.json`: `{"1": {"1": [949, 952], "2": [950, 951]}, "2": {"1": [943, 945], "2": [944]}, "3": {"1": [954], "2": [953]}, "4": {"1": [957, 958], "2": [959]}, "5": {"1": [961, 962], "2": [960]}}` |
| 車両入力 | 19か所、合計600台/h(SUMOの出発地点の割合で分けたもの)。経路の指定はない(Town01.inpxと同じ) |
| 車両設定 | 車両タイプ・車両構成はTown01.inpxと同じ(自車はEGO・車両タイプ101、ドライブシミュレータ有効)。2D/3Dモデル分布は、日本の車(Toyota Alphard・NOAH・JPN Taxi、ISUZU GIGA・Erga・GALA)に直してある。トレーラーはトラクターだけ(連結車にしない)(2026-11-17、利用者。`vissim/README.md`の2.1.1) |
| Vissimでの確認 | 開いたとき・RUNを止めたときとも、エラー・警告なし。RUNで信号が切り替わる(2026-11-17、利用者が確認) |

**Town10HDで使えない、0.9.15版のツール**:
- `~/CARLA`の`util/generate_signal_mapping.py`(0.9.15版の2.4.2)と、`tools/check_vissim_lanelet2_tl_ids.py`の`--inpx`の照合は、信号灯からいちばん近いCARLAの信号の柱で対応を決める。
  Town10HDの信号の柱は交差点の向こう側(停止線から30〜40 m先)にあり、いちばん近い柱が別の進入路の信号になるため、Town10HDでは誤った対応表・誤った`NG`になる。
  2026-11-17に`check_vissim_lanelet2_tl_ids.py`を`--inpx`付きで実行すると、`NG vehicle signal heads farther than 10.0 m`などが出た(誤り)。
- `--inpx`を付けない照合は使える。対応表・Lanelet2地図・CARLAの信号の15個がそろっていることを確かめた(`Vissim: 15 landmarks in 10 signal groups of 5 signal controllers`、
  `Lanelet2: 15 traffic lights in 30 regulatory elements`、`CARLA: 15 traffic lights`)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/check_vissim_lanelet2_tl_ids.py src/autoware_carla_interface/vissim_integration/data/signal_mapping.json \
    ~/autoware_map/Town10HD/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town10HD/carla_tl.json --group-id-offset 100000
```

- 最後に`OK`(終了コード0)が出ればよい(2026-11-17に確認)。
- `carla_tl.json`に`group`がないので`WARNING: --carla-json has no 'group' ...; intersection checks skipped`が出る(交差点ごとの照合だけ飛ばす)。

**スポーン地点**: 2.6.1の`30.02,133.95,0.30,0.0,0.0,-179.7`は、信号954の手前(CARLAのスポーン地点#80、道路4)。CARLAの停止位置まで51 m、手前の交差点から110 m(後ろ36 mから先はカーブ)。
SUMO構成で確かめた地点で、Vissimの車がこの車線を通るか、空き待ちが起きるかは未確認(3.1.5)。

### 3.5 CARLA 0.9.15(Town01)に戻すとき

```bash
pip3 install --user carla==0.9.15
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
git checkout src/autoware_carla_interface/vissim_integration/data/signal_mapping.json   # Town01の対応表に戻す
git status --short src/autoware_carla_interface/vissim_integration/data/                 # 何も出ないこと(表を足した場合は ?? が出る)
```

- サーバーは`~/CARLA/CarlaUE4.sh`で起動し、手順は0.9.15版の「Vissim(win)-CARLA-Autoware_co-sim_起動手順.md」に従う(Windows側は`Town01.inpx`)。
- `carla_variants_town10hd.yaml`と`mesh_measurements_carla0.9.16_jp.json`は残しておいてよい(Town01は`carla_variants_town01.yaml`を使う)。
- `~/autoware_map/Town10HD_Opt`(シンボリックリンク)も残しておいてよい。
