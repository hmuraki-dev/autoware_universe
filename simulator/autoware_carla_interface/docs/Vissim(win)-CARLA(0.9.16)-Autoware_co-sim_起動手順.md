# Vissim(windows)-CARLA(0.9.16)-Autoware co-sim 起動手順

CARLA 0.9.15(`~/CARLA/`)用の「Vissim(win)-CARLA-Autoware_co-sim_起動手順.md」をもとに、CARLAを0.9.16(`~/CARLA_0.9.16_jp/`)に差し替えたときの手順をまとめたもの。
0.9.15版との違いは、下の表と3.1にまとめた。0.9.15版から変わらない機能の詳しい説明(ウォームアップのログの見方、信号の直接通知のオプションなど)は、0.9.15版の3章を参照。

| 項目 | 0.9.15版 | 0.9.16版(この手順書) |
| --- | --- | --- |
| CARLAサーバー | `~/CARLA/CarlaUE4.sh` | `~/CARLA_0.9.16_jp/LinuxNoEditor/CarlaUE4.sh` |
| `get_server_version()`の値 | `0.9.15` | `294096eb1-dirty`(独自ビルドのため、版の番号ではない) |
| CARLAのPythonクライアント | `carla==0.9.15`(`pip --user`。`~/carla310_env`にも0.9.15) | `carla==0.9.16`(`~/venvs/carla0916`)。0.9.16を使うターミナルで`PYTHONPATH`の先頭に足す(2.0.1)。`pip --user`の0.9.15はそのまま。**0.9.15のクライアントは0.9.16のサーバーにつなぐと落ちる**(3.1.1) |
| マップ | Town01 | **Town10HD(`Town10HD_Opt`)**。0.9.16_jpに入っているマップはこれだけ(3.1.2) |
| Autowareの地図フォルダ | `~/autoware_map/Town01` | `~/autoware_map/Town10HD_Opt`(`Town10HD`へのシンボリックリンク。2.0.2) |
| Vissimのネットワーク | `Town01.inpx`(CARLAリポジトリ) | `Town10HD.inpx`(このリポジトリの`vissim/`。3.4) |
| 信号の対応表 | `signal_mapping_town01.json` | `signal_mapping_town10hd.json`。どちらもリポジトリに入っていて、ブリッジがマップ名から選ぶ(2.4.2) |
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
- 2.4・2.6の最初の`export PYTHONPATH=...`は、**0.9.16を使うターミナル(2と3)で毎回**実行する(2.0.1)。
- 2.2.0・2.4.2・2.4.3は、**対象を変えたときだけ**実行する(毎回の起動では不要)。

### 2.0 【Linux側/ターミナル2】CARLA 0.9.16に切り替えたときだけ行う準備

#### 2.0.1 CARLA 0.9.16のPythonクライアントを用意する

0.9.15の環境(`pip --user`で入れた`carla 0.9.15`)を変えずに、0.9.16のクライアントを別の場所に入れる。
`~/CARLA_0.9.16_jp_incoming/手順書.md`の「3. Python の環境を分ける」で作ったvenv(`~/venvs/carla0916`)を使う。作ってあれば、このコマンドは不要。

```bash
python3 -m venv ~/venvs/carla0916   # ← 生成物
~/venvs/carla0916/bin/pip install carla==0.9.16
~/venvs/carla0916/bin/python -c "import carla, importlib.metadata as m; print(m.version('carla'), carla.__file__)"   # 0.9.16 と出ること
```

> 📄 **生成物:** CARLA 0.9.16のPythonクライアントを入れたvenv\
> `~/venvs/carla0916`

**使い方**: 0.9.16を使うターミナル(2.4のターミナル2、2.6のターミナル3)で、最初に次を実行する。venvは有効にしない(`source .../activate`はしない)。

```bash
export PYTHONPATH=$HOME/venvs/carla0916/lib/python3.10/site-packages:$PYTHONPATH
python3 -c "import carla, importlib.metadata as m; print(m.version('carla'), carla.__file__)"   # 0.9.16 .../venvs/carla0916/... と出ること
```

- `PYTHONPATH`の先頭に足したフォルダは、`pip --user`のフォルダ(`~/.local`)より先に探される。そのため、このターミナルでは`carla`だけが0.9.16になり、
  ほかのパッケージ(`numpy`・`transforms3d`・`pyzmq`・`msgpack`・`rclpy`など)は今までどおり`~/.local`・システム・ROSのものが使われる。
  `export`しないターミナルは、今までどおり0.9.15(0.9.15版の手順書がそのまま使える)。
- **venvを有効にしない理由**: venvには`carla`しか入っていないので、有効にすると、Autowareのブリッジ(`transforms3d`・`numpy`・`pyzmq`・`msgpack`が要る)や
  表の生成ツール(`yaml`が要る)が起動できない(0.9.15版の手順書の`carla310_env`の注意と同じ)。ブリッジの起動スクリプトは`#!/usr/bin/env python3`なので、有効にしたvenvのPythonで動いてしまう。
- 2026-11-17に、この方法で、ブリッジのモジュールが読み込めること、起動中の0.9.16のサーバーにつながること(クライアント0.9.16、日本の車6台が見える)を確かめた。
- **`~/carla310_env`は使わない**。中の`carla`は0.9.15で、0.9.16のサーバーにつなぐと落ちる(3.1.1)。
- `~/CARLA_0.9.16_jp/LinuxNoEditor/PythonAPI/carla/dist/`のwheelはPython 3.12用(`cp312`)だけで、Python 3.10では使えない。PyPIの`carla==0.9.16`にはPython 3.10用がある。

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

#### 2.0.3 車線変更できる地図を作る

```bash
source /opt/ros/humble/setup.bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/add_lanelet2_line_types.py ~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl.osm \
    ~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl_lc.osm   # ← 生成物
```

> 📄 **生成物:** 車線の境界線に線の種類を付けた、信号入りのLanelet2地図(2.6.1の`lanelet2_map_file`)\
> `~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl_lc.osm`

- 出力の`line_thin dashed: 44`・`line_thin solid: 235`を確かめる(3.2)。
- すでに作ってあれば不要。`lanelet2_map_with_tl.osm`を作り直したときは、これも作り直す。

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
- Windows側で`.inpx`を直したときは、Linuxへ写し戻して`vissim/Town10HD.inpx`にコミットする(2.4.2の「作り直し」の(1))。
  信号の対応表(2.4.2)と車両のブループリント表(2.4.3)を、同じinpxから作るため。
  ただし、`vissim/Town10HD.inpx`は`vissim/README.md`の2章のツールで作り直すと上書きされる。

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

### 2.4 【Linux側/ターミナル2】マップの設定 → 信号の対応表の確認・作り直し → 車両のブループリント表の生成

#### 2.4.1 CARLAで使用するマップの設定

```bash
export PYTHONPATH=$HOME/venvs/carla0916/lib/python3.10/site-packages:$PYTHONPATH   # このターミナルを0.9.16にする(2.0.1)
cd ~/CARLA_0.9.16_jp/LinuxNoEditor/PythonAPI/util
python3 config.py --map Town10HD_Opt
```

- 2.4.2・2.4.3も、この`export`をしたターミナル2で実行する。
- venv(`~/venvs/carla0916`・`~/carla310_env`)は有効にしない(2.0.1)。有効になっていたら`deactivate`してから実行する。
- CARLAは起動直後から`Town10HD_Opt`を読み込んでいる。ほかのマップ(`OpenDriveMap`など)にしたときに、このコマンドで戻す。

#### 2.4.2 信号の対応表の確認・作り直し

Vissimの(信号制御, 信号グループ) → CARLAの信号IDの対応表。Town10HD用はリポジトリに入っていて、ブリッジが起動時に`map_path`のフォルダ名(`Town10HD_Opt`)から選ぶ(3.4.1)。

**確認(毎回の起動では不要。初めて使うときに1回)**

置き換えやコピーはしない。ファイルがあることだけ確かめる。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
cat src/autoware_carla_interface/vissim_integration/data/signal_mapping_town10hd.json
```

- 中身は、5つの信号制御(`"1"`〜`"5"`)ごとに、信号グループ`"1"`・`"2"`のCARLAの信号ID(943〜962の15個)。
- 以前の手順で`vissim_integration/data/signal_mapping.json`へコピーしたファイルが残っていても、ブリッジは読まない。消してよい。
- 0.9.15版の2.4.2(`generate_signal_mapping.py`)は**使わない**。Town10HDでは誤った対応表になる(3.4)。

**作り直し(Windows PCのVissimで`Town10HD.inpx`の信号灯・信号グループを変えたときだけ)**

対応表は、inpxの信号灯の名前(`CARLA <CARLAの信号ID>`)と、その信号灯の信号グループから作る。CARLA・Vissimは使わない。
信号灯・信号グループを変えていない(信号の秒数・車両の設定だけを変えた)ときは、(1)だけ行う。対応表は変わらない。

(1) 【Windows側/PowerShell】Vissimで保存した`Town10HD.inpx`を、Linuxの`vissim/Town10HD.inpx`へ写し戻す(IPアドレスとユーザー名は使うPCに合わせる)。

```powershell
scp C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town10HD.inpx divp@192.168.16.33:/home/divp/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface/vissim/Town10HD.inpx
```

- 写すのは`Town10HD.inpx`。アダプタが作る`Town10HD.cosim.inpx`(2.2.1)は写さない。
- USBメモリや共有フォルダで写してもよい。

(2) 【Linux側/ターミナル2】対応表を作る。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/make_vissim_signal_mapping.py --inpx vissim/Town10HD.inpx \
    --out src/autoware_carla_interface/vissim_integration/data/signal_mapping_town10hd.json \
    --out vissim/signal_mapping_town10hd.json   # ← 生成物
```

> 📄 **生成物:** Vissimの(信号制御, 信号グループ) → CARLAの信号IDの対応表(ブリッジが読むものと、`vissim/`の控え)\
> `src/autoware_carla_interface/vissim_integration/data/signal_mapping_town10hd.json`、`vissim/signal_mapping_town10hd.json`

- `信号制御 <番号>: SG 1 -> <CARLAの信号ID> SG 2 -> ...`が信号制御ごとに1行、最後に`信号制御 <数>、CARLA の信号 <数>`が出る。
  今のinpx(2026-11-17)では、信号制御5・CARLAの信号15。
- 2つの`--out`に同じものを書く(`vissim/`の控えは、inpxと対応表を一緒に置いておくため)。出力先は上書きする。
- 写せない信号灯があると`Error: <理由>`を出して、ファイルを書かずに終わる(3.4.2)。

(3) 【Linux側/ターミナル2】確かめる。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
git status --short vissim/ src/autoware_carla_interface/vissim_integration/data/
python3 tools/check_vissim_lanelet2_tl_ids.py src/autoware_carla_interface/vissim_integration/data/signal_mapping_town10hd.json \
    ~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town10HD_Opt/carla_tl.json --group-id-offset 100000
python3 -m pytest test/test_make_vissim_signal_mapping.py test/test_vissim_signal_mapping.py
```

- `git status`: 変えたinpxと、対応表が変わったときは2つの対応表が出る。
- 照合: 対応表のCARLAの信号が、信号入りのLanelet2地図(`lanelet2_map_with_tl.osm`)とCARLAの信号(`carla_tl.json`)にそろっているかを確かめる。
  最後に`OK`(終了コード0)が出ればよい。今の対応表では`Vissim: 15 landmarks in 10 signal groups of 5 signal controllers`・
  `Lanelet2: 15 traffic lights in 30 regulatory elements`・`CARLA: 15 traffic lights`が出る(2026-11-25)。
  `WARNING: --carla-json has no 'group' ...; intersection checks skipped`は出てよい。`--inpx`は付けない(3.4)。
- テスト: `test_town10hd_mapping_is_reproduced`は、今のinpxから作った対応表が`data/`・`vissim/`のものと同じかを確かめる((2)をしていないと失敗する)。
  `test_vissim_signal_mapping.py`は、`data/`と`vissim/`の対応表が同じか、ブリッジが選べるかを確かめる。
- 確かめたら、inpxと2つの対応表をautoware_universeにコミットする。`--symlink-install`でビルドしていれば、ビルドし直さなくても使われる。
- `vissim/README.md`の2章(`build_vissim_town.py`)でも対応表はできるが、`Town10HD.inpx`も作り直すので、Vissimで直した設定が消える。使わない。

#### 2.4.3 Vissimの車両のCARLAブループリント表の生成

Vissimの車両1台ごとに、どのCARLAのブループリントを使うかを決める表を作る。仕組みは3.3、0.9.15版の3.3を参照。

**注意**:
- **0.9.16に切り替えたときは、(1)(2)(3)をすべて行う**。0.9.15で作った表(`carla_variants_town01.yaml`)は、外形を測ったCARLAの版(`0.9.15`)が
  接続先(`294096eb1-dirty`)と違うので、起動時に`Error: CARLA blueprint assignment: variant_mesh_carla_version_changed: ...`で止まる。
- そのあとは、0.9.15版の2.4.3の表のとおり、変えたものに応じて実行する(`Town10HD.inpx`の車両の設定を変えたときは(2)(3))。
- 作った表と計測結果は、autoware_universeにコミットする。`--symlink-install`でビルドしていれば、**前からあるファイル**を作り直しただけなら、ビルドし直さなくても使われる。
  **新しいファイル**(初めて作った`carla_variants_town10hd.yaml`・`mesh_measurements_carla0.9.16_jp.json`・`carla_variant_rules_carla0.9.16_jp.yaml`など)は、
  2.5でビルドし直すまでインストール先にリンクが無く、ブリッジが読めない(`--symlink-install`のリンクはファイルごとに、ビルドのときに作られる)。
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
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules_carla0.9.16_jp.yaml \
    --out src/autoware_carla_interface/vissim_integration/data/carla_variants_town10hd.yaml   # ← 生成物
```

> 📄 **生成物:** Vissim車両-ブループリント割り当て一覧(Town10HD)\
> `src/autoware_carla_interface/vissim_integration/data/carla_variants_town10hd.yaml`

- `--inpx`は、Windows PCで読み込む`.inpx`と同じもの(2.2.0)。リポジトリ内のファイルなので`--inpx-ref`は要らない(表には、渡したパス`vissim/Town10HD.inpx`がそのまま記録される。`.inpx`のsha256も記録される)。
- `--rules`は0.9.16_jp用の規則(`carla_variant_rules_carla0.9.16_jp.yaml`)。0.9.15用の`carla_variant_rules.yaml`(Town01の表が使う)と違い、
  大型車(HGV・BUS)の幅の上限を0.45 mにし、ブループリントの分類(`base_type`)で候補を絞る(3.3)。
- `Town10HD.inpx`の車両は、日本の車のモデル(乗用車3・トラック1・バス2)にしてある(3.4)。2026-11-24に作った表では、6つとも同じ日本の車に割り当たった(3.3)。
- トレーラー(モデル22)は連結車にしない(トラクターだけ)。連結車があると、起動時に`Error: CARLA blueprint assignment: variant_articulated_unsupported`で止まる(このブリッジは連結車を扱わない)。

**(3) 確認**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 -m pytest test/test_carla_fleet.py test/test_build_carla_variants.py test/test_vissim_fleet_plan.py
```

- 表の`assignment`(モデルごとのブループリント・dL・dW)を見て、意図した割り当てか確かめる。タクシー(`vehicle.jp.taxi`)など、塗装に意味のある車が割り当たったときは、
  規則(`carla_variant_rules_carla0.9.16_jp.yaml`)の`excluded`に足して(2)から作り直す。
- テストは、Town01の割り当て(0.9.15の規則と外形)と、Town10HDの割り当て(0.9.16_jpの規則と`mesh_measurements_carla0.9.16_jp.json`。3.3の表)を期待値にしている。
  (1)で測り直して割り当てが変わると、`test_jp_rules_on_town10hd_models`が落ちる。意図した変化なら、テストの期待値を直す。

### 2.5 【Linux側/ターミナル3】ビルド

0.9.15版の2.5と同じ。CARLAを差し替えただけなら、ビルドし直す必要はない(`carla`はPythonのパッケージとして実行時に読み込まれる)。
2.4.3で新しいファイル(表・計測結果・規則)を作ったときは、ビルドし直す(2.4.3の注意)。

```bash
source /opt/ros/humble/setup.bash
cd ~/autoware.1.9.0
colcon build --packages-select autoware_carla_interface --symlink-install
source ~/autoware.1.9.0/install/setup.bash
```

### 2.6 【Linux側/ターミナル3】Vissim/CARLA/Autoware起動
#### 2.6.1 起動コマンド

```bash
export PYTHONPATH=$HOME/venvs/carla0916/lib/python3.10/site-packages:$PYTHONPATH   # ブリッジを0.9.16のクライアントで動かす(2.0.1)
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town10HD_Opt \
  lanelet2_map_file:=lanelet2_map_with_tl_lc.osm \
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
| `lanelet2_map_file`・`traffic_light.publish` | (3.2.2で足す) | 最初から付ける。地図は`lanelet2_map_with_tl_lc.osm` | Town10HDの信号入りの地図がある(3.2)。車線変更できるようにした地図を使う(2.0.3) |
| `vissim_carla_variants` | (既定の`carla_variants_town01.yaml`) | `carla_variants_town10hd.yaml` | 2.4.3で作った表。作っていないときは`legacy_vtypes` |
| `vissim_signal_mapping` | (既定の`auto`) | (既定の`auto`。指定しない) | `map_path`のフォルダ名から`signal_mapping_town10hd.json`が選ばれる(3.4.1) |
| `spawn_point` | Town01の地点 | 信号954の手前(CARLAのスポーン地点#80) | Town10HDの地点(3.4) |

- オプションの一覧と注意は、0.9.15版の2.6.1を参照(変わらない)。
- 起動ログで確かめること:
  - `[CARLA Interface] Map Path: '.../Town10HD_Opt' -> Loading CARLA Map: 'Town10HD_Opt'`
  - `Vissim signal mapping: .../vissim_integration/data/signal_mapping_town10hd.json`(信号の対応表。3.4.1)
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
- 0.9.16のクライアント(2.0.1の方法)では、`Version mismatch`の警告は出るが(サーバーの版が`294096eb1-dirty`のため)、つながる。
  `get_available_maps()`は`['Town10HD_Opt', 'AnnotationColorLandscape']`、`vehicle.jp.*`の6台が見えた(2026-11-17)。
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
| Vissimの信号がCARLA・Autowareに届くか(`Traffic-light matching: 15 matched`) | 2.6.1の起動ログ、0.9.15版の3.2.4の確認用ツール |
| 2.6.1のスポーン地点の車線に、Vissimの車が通るか(ウォームアップの空き待ちの確認になるか) | 2.6.1の起動ログの`[EGO SPAWN CHECK]` |

### 3.2 信号入りのLanelet2地図(Town10HD)

- `~/autoware_map/Town10HD/lanelet2_map_with_tl.osm`(2026-10-21、SUMO構成で作成)。CARLAの15信号、停止位置ごとに30個のregulatory element(IDはOpenDRIVEの信号ID + 100000)。
  停止線はCARLAの停止位置のまま(前に出していない)。
- 同じフォルダの`carla_tl.json`は、地図を作ったときのCARLAの信号の記録(位置・停止位置)。交差点の`group`は入っていない(0.9.15版のTown01の手順の`split_signal_controllers.py`を使うときは、作り直す必要がある)。
- Vissimの信号灯(3.4)も同じCARLAの停止位置に置いてあるので、EGOとVissimの車がほぼ同じ位置で止まる。
- 作り直すときの手順は、SUMO版の手順書(「SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md」)の2.2.2。
- **車線変更**: CARLAから変換した地図は、車線の境界線(279本)に線の種類(`type`・`subtype`)が無い。Lanelet2の交通ルールはこれを越えられない線として扱うので、
  `lanelet2_map_with_tl.osm`のままでは、どこでも車線変更できない。経路が隣の車線を使うと、EGOは自分の車線の終わりで止まる
  (2026-11-24。信号954の手前の左車線(lanelet 891)で、右折の経路(右車線 860 → 8218)に移れずに止まった。車線変更のモジュールは一度も動かなかった)。
- 2.0.3の`tools/add_lanelet2_line_types.py`は、種類の無い境界線に、同じ向きの車線どうしの間は`line_thin`/`dashed`(車線変更できる)、
  それ以外(外側・対向車線との間・交差点の中の車線)は`line_thin`/`solid`を付ける。交差点の中の車線は、分かれる・合わさる車線で見分ける(地図に`turn_direction`が無いため)。
  Town10HDでは44組の車線の間で車線変更できるようになる(891→860・569→692・2156→2179 を含む)。付けるのはタグだけで、点・車線・信号は変えない。
- 元の`lanelet2_map_with_tl.osm`は変えない(SUMO構成もこの地図を使う)。

### 3.3 Vissimの車両のCARLAブループリント

- 仕組みは0.9.15版の3.3と同じ(Vissimの2D/3Dモデルの寸法に最も近いブループリント。合うものが無い大型車は`carlacola`の重ね置き)。
- 0.9.16_jp用の規則(`carla_variant_rules_carla0.9.16_jp.yaml`、2026-11-24)は、0.9.15用の規則から次の2つを変えた。
  - 幅の上限(dW、太い側)を車種カテゴリごとにした。乗用車・二輪は0.35 m のまま、トラック・バス(HGV・BUS・TRAM)は0.45 m。
    大型車のミラーは乗用車より張り出す。日本の車のdW(ミラー込みの外形 − モデルの幅)は、乗用車 +0.18〜+0.34 m、トラック・バス +0.35〜+0.40 m だった。
  - 1体で表す候補を、ブループリントの分類(`base_type`)で絞る(乗用車: car・van・空、トラック: truck、バス: bus、二輪: motorcycle・bicycle)。
    絞らないと、トラック(GIGA)に長さの近いバス(`jp.tour_bus`)が割り当たる。`carlacola`の重ね置きは絞らない(バスにも使う)。
  - 分類の合う車も重ね置きも使えないモデルがあると、(2)は`variant_model_empty`で止まり、表を書かない。
- 2026-11-24に作ったTown10HDの表(修正版の日本の車を測った外形):

  | Vissimのモデル | ブループリント | dL / dW [m] |
  | --- | --- | --- |
  | 1001 Toyota Alphard | `vehicle.jp.alphard` | +0.021 / +0.310 |
  | 1002 Toyota NOAH | `vehicle.jp.minivan` | +0.015 / +0.344 |
  | 1003 Toyota JPN Taxi | `vehicle.jp.taxi` | −0.001 / +0.176 |
  | 1021 ISUZU GIGA | `vehicle.jp.large_truck` | +0.361 / +0.374 |
  | 22 Semi-Tractor Volvo E | `vehicle.carlamotors.carlacola` ×2(重ね置き) | 0.000 / +0.009 |
  | 1031 ISUZU Erga | `vehicle.jp.route_bus` | +0.269 / +0.396 |
  | 1032 ISUZU GALA | `vehicle.jp.tour_bus` | +0.119 / +0.350 |
  | 70 Yamaha MT 07 | `vehicle.kawasaki.ninja` | −0.081 / −0.032 |
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
| 車両設定 | 車両タイプはTown01.inpxと同じ(自車はEGO・車両タイプ101、ドライブシミュレータ有効)。車両構成の相対交通量は 乗用車0.5・トラック0.1・トレーラー0.1・バス0.2・バイク0.1(2026-11-17、利用者が変更)。2D/3Dモデル分布は、日本の車(Toyota Alphard・NOAH・JPN Taxi、ISUZU GIGA・Erga・GALA)に直してある。トレーラーはトラクターだけ(連結車にしない)(2026-11-17、利用者。`vissim/README.md`の2.1.1) |
| Vissimでの確認 | 開いたとき・RUNを止めたときとも、エラー・警告なし。RUNで信号が切り替わる(2026-11-17、利用者が確認) |

**Town10HDで使えない、0.9.15版のツール**:
- `~/CARLA`の`util/generate_signal_mapping.py`(0.9.15版の2.4.2)と、`tools/check_vissim_lanelet2_tl_ids.py`の`--inpx`の照合は、信号灯からいちばん近いCARLAの信号の柱で対応を決める。
  Town10HDの信号の柱は交差点の向こう側(停止線から30〜40 m先)にあり、いちばん近い柱が別の進入路の信号になるため、Town10HDでは誤った対応表・誤った`NG`になる。
  2026-11-17に`check_vissim_lanelet2_tl_ids.py`を`--inpx`付きで実行すると、`NG vehicle signal heads farther than 10.0 m`などが出た(誤り)。
- `--inpx`を付けない照合は使える。対応表・Lanelet2地図・CARLAの信号の15個がそろっていることを確かめた(`Vissim: 15 landmarks in 10 signal groups of 5 signal controllers`、
  `Lanelet2: 15 traffic lights in 30 regulatory elements`、`CARLA: 15 traffic lights`)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/check_vissim_lanelet2_tl_ids.py src/autoware_carla_interface/vissim_integration/data/signal_mapping_town10hd.json \
    ~/autoware_map/Town10HD/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town10HD/carla_tl.json --group-id-offset 100000
```

- 最後に`OK`(終了コード0)が出ればよい(2026-11-17に確認)。
- `carla_tl.json`に`group`がないので`WARNING: --carla-json has no 'group' ...; intersection checks skipped`が出る(交差点ごとの照合だけ飛ばす)。

**スポーン地点**: 2.6.1の`30.02,133.95,0.30,0.0,0.0,-179.7`は、信号954の手前(CARLAのスポーン地点#80、道路4)。CARLAの停止位置まで51 m、手前の交差点から110 m(後ろ36 mから先はカーブ)。
SUMO構成で確かめた地点で、Vissimの車がこの車線を通るか、空き待ちが起きるかは未確認(3.1.5)。

#### 3.4.1 信号の対応表の選び方(`vissim_signal_mapping`)

ブリッジは、Vissimの(信号制御, 信号グループ) → CARLAの信号IDの対応表を、起動の引数`vissim_signal_mapping`で選ぶ(2026-11-24から)。

| 値 | 読むファイル |
| --- | --- |
| `auto`(既定) | `vissim_integration/data/signal_mapping_<マップ名>.json`。マップ名は`map_path`のフォルダ名を小文字にし、末尾の`_Opt`を外したもの。`Town01` → `signal_mapping_town01.json`、`Town10HD_Opt` → `signal_mapping_town10hd.json` |
| ファイル名・相対パス | `vissim_integration/data/`から探す(例: `vissim_signal_mapping:=signal_mapping_town01.json`) |
| 絶対パス | そのまま |

- 読むのは`use_vissim:=true sync_traffic_lights:=true`のときだけ。ファイルが無いときは、Vissimにつなぐ前に`Error: invalid parameters: vissim_signal_mapping:=auto for CARLA map 'Town05': signal mapping file not found: ...`で止まる(`data/`にある対応表の一覧も出る)。
- `data/signal_mapping_town10hd.json`と`vissim/signal_mapping_town10hd.json`は同じもの(`test/test_vissim_signal_mapping.py`が確かめる)。
  2026-11-17に、`feature/sumo_co-sim`の`tools/vissim/build_vissim_town.py`が`Town10HD.inpx`と一緒に作った。
  Vissimで直したinpxからは、`tools/make_vissim_signal_mapping.py`で両方を作り直す(2.4.2の「作り直し」)。同じinpxからは、同じバイト列になる。
- SUMO構成(`feature/sumo_co-sim`)には、この対応表はない。SUMOの信号とCARLAの信号の対応は、SUMOの網(`sumo/net/Town10HD.net.xml`)の
  `tlLogic`の`linkSignalID`に書かれている(SUMO構成の起動手順書の2.2.2)。どちらもCARLAの信号IDでLanelet2地図(3.2)につながる。
- 以前は`data/signal_mapping.json`の1つだけを読み、Townを替えるたびに上書きしていた(このファイルは`data/signal_mapping_town01.json`に名前を変えた)。

#### 3.4.2 信号の対応表を作り直すときに止まったとき(2.4.2)

`tools/make_vissim_signal_mapping.py`は、写せない信号灯を見つけると`Error: <理由>`を出して、ファイルを書かずに終わる(終了コード1)。

| 出る理由 | 原因 | 対処 |
| --- | --- | --- |
| `信号灯 <番号> の名前 ... が「CARLA <信号 ID>」でない` / `... の CARLA の信号 ID が数でない` | 信号灯の名前がCARLAの信号IDの形でない(Vissimで足した信号灯など) | 名前を`CARLA <CARLAの信号ID>`にする |
| `CARLA の信号 <ID> の信号灯が別の信号グループにある` | 同じCARLAの信号の信号灯(車線ごとに1つ)が、違う信号グループにある | 同じ信号グループにする |
| `信号灯 <番号>(CARLA <ID>)の信号制御 ... が inpx に無い` / `信号グループ ... が信号制御に無い` | 信号灯の信号グループが、inpxの信号制御に無い | 信号灯の信号グループを確かめる |
| `信号灯 <番号> の信号グループ sg=... が「<信号制御> <信号グループ>」でない` | 信号灯に信号グループが付いていない | 信号灯に信号グループを付ける |
| `名前が「CARLA <信号 ID>」の信号灯が無い` | 信号灯が1つも無い | inpxを確かめる |

- 照合(2.4.2の(3))で`NG`が出たときは、対応表にあるCARLAの信号IDが、Lanelet2地図かCARLAに無い。信号灯の名前のIDを確かめる。
- 信号灯を、CARLAに信号の無い場所に足すことはできない(CARLAの信号機が無いので、色を書き込めない)。

### 3.5 CARLA 0.9.15(Town01)に戻すとき

0.9.15の環境は変えていないので、**`PYTHONPATH`を足していない新しいターミナル**を使えば、そのまま0.9.15になる(2.0.1)。

```bash
python3 -c "import carla, importlib.metadata as m; print(m.version('carla'))"   # 0.9.15 と出ること(新しいターミナルで)
```

- サーバーは`~/CARLA/CarlaUE4.sh`で起動し、手順は0.9.15版の「Vissim(win)-CARLA-Autoware_co-sim_起動手順.md」に従う(Windows側は`Town01.inpx`)。
- `carla_variants_town10hd.yaml`と`mesh_measurements_carla0.9.16_jp.json`は残しておいてよい(Town01は`carla_variants_town01.yaml`を使う)。
- `~/autoware_map/Town10HD_Opt`(シンボリックリンク)も残しておいてよい。
- 信号の対応表は戻さなくてよい(`map_path`が`Town01`なら、ブリッジが`signal_mapping_town01.json`を選ぶ。3.4.1)。
