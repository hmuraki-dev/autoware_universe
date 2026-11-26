# SUMO-CARLA(0.9.16)-Autoware co-sim 起動手順

CARLA 0.9.15(`~/CARLA/`)用の「SUMO-CARLA-Autoware_co-sim_起動手順.md」をもとに、CARLAを0.9.16(`~/CARLA_0.9.16_jp/`)に差し替えたときの手順をまとめたもの。
0.9.15版との違いは、下の表と3.1にまとめた。

| 項目 | 0.9.15版 | 0.9.16版(この手順書) |
| --- | --- | --- |
| CARLAサーバー | `~/CARLA/CarlaUE4.sh` | `~/CARLA_0.9.16_jp/LinuxNoEditor/CarlaUE4.sh` |
| `get_server_version()`の値 | `0.9.15` | `294096eb1-dirty`(独自ビルドのため、版の番号ではない) |
| CARLAのPythonクライアント | `carla==0.9.15`(`pip --user`) | `carla==0.9.16`(`~/venvs/carla0916`)。0.9.16を使うターミナルで`PYTHONPATH`の先頭に足す(2.0.1)。`pip --user`の0.9.15はそのまま。**0.9.15のクライアントは0.9.16のサーバーにつなぐと落ちる**(3.1.1) |
| 使えるマップ | Town01〜Town07・Town10HD など | **`Town10HD_Opt`のみ**(Town01・Town05は入っていない。3.1.2) |
| Autowareの地図フォルダ | `~/autoware_map/Town10HD` など | `~/autoware_map/Town10HD_Opt`(`Town10HD`へのシンボリックリンク。2.0.2) |
| SUMOのシナリオ(net・ルート・`.sumocfg`) | `sumo/`(autoware_universe) | 同じ(CARLAの版によらない。3.1.3) |
| 車両 | CARLA標準 | CARLA標準 + 日本の車6台(`vehicle.jp.*`。3.1.4) |

# 0. 前提条件
Autoware環境を構築済みであること。
環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。

CARLA 0.9.16を`~/CARLA_0.9.16_jp/LinuxNoEditor/`に展開済みであること(受け渡し用のアーカイブは`~/CARLA_0.9.16_jp_incoming/`)。

SUMOのシナリオ(`.sumocfg`・net・ルート)は、このリポジトリの`simulator/autoware_carla_interface/sumo/`のものを使う(0.9.15と同じ。3.1.3)。


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
                    └── sumo/  ← SUMOのシナリオ(.sumocfg・net/・rou/)
```

`autoware_carla_interface` は独立したリポジトリではなく、`autoware_universe` リポジトリ配下のパッケージである。

CARLA関係のフォルダは次のとおり。

```text
~/
├── CARLA_0.9.16_jp/
│   └── LinuxNoEditor/          ← CARLA 0.9.16のサーバー(CarlaUE4.sh)・PythonAPI
├── CARLA_0.9.16_jp_incoming/   ← 受け渡し用のアーカイブと、日本の車の確認ツール
└── CARLA/                      ← CARLA 0.9.15のリポジトリ。SUMOのnet・ルートを作るツールと、その手順書
```

# 2. 起動・終了の手順

2.0から順に実行する。
- 2.0は、**CARLAを0.9.16に切り替えたときに1回だけ**実行する(毎回の起動では不要)。
- 2.2・2.4の最初の`export PYTHONPATH=...`は、**0.9.16を使うターミナル(2と3)で毎回**実行する(2.0.1)。
- 2.2.2〜2.2.5は、**対象を変えたときだけ**実行する(毎回の起動では不要)。

## 2.0 【ターミナル2】CARLA 0.9.16に切り替えたときだけ行う準備

### 2.0.1 CARLA 0.9.16のPythonクライアントを用意する

0.9.15の環境(`pip --user`で入れた`carla 0.9.15`)を変えずに、0.9.16のクライアントを別の場所に入れる。
`~/CARLA_0.9.16_jp_incoming/手順書.md`の「3. Python の環境を分ける」で作ったvenv(`~/venvs/carla0916`)を使う。作ってあれば、このコマンドは不要。

```bash
python3 -m venv ~/venvs/carla0916   # ← 生成物
~/venvs/carla0916/bin/pip install carla==0.9.16
~/venvs/carla0916/bin/python -c "import carla, importlib.metadata as m; print(m.version('carla'), carla.__file__)"   # 0.9.16 と出ること
```

> 📄 **生成物:** CARLA 0.9.16のPythonクライアントを入れたvenv\
> `~/venvs/carla0916`

**使い方**: 0.9.16を使うターミナル(2.2のターミナル2、2.4のターミナル3)で、最初に次を実行する。venvは有効にしない(`source .../activate`はしない)。

```bash
export PYTHONPATH=$HOME/venvs/carla0916/lib/python3.10/site-packages:$PYTHONPATH
python3 -c "import carla, importlib.metadata as m; print(m.version('carla'), carla.__file__)"   # 0.9.16 .../venvs/carla0916/... と出ること
```

- `PYTHONPATH`の先頭に足したフォルダは、`pip --user`のフォルダ(`~/.local`)より先に探される。そのため、このターミナルでは`carla`だけが0.9.16になり、
  ほかのパッケージ(`numpy`・`traci`・`sumolib`・`rclpy`など)は今までどおり`~/.local`・システム・SUMO・ROSのものが使われる。
  `export`しないターミナルは、今までどおり0.9.15(0.9.15版の手順書がそのまま使える)。ROSの`setup.bash`の前後どちらで`export`しても0.9.16になる。
- **venvを有効にしない理由**: venvには`carla`しか入っていないので、有効にすると、Autowareのブリッジ(`transforms3d`・`numpy`などが要る)や表の生成ツール(`yaml`が要る)が起動できない。
  ブリッジの起動スクリプトは`#!/usr/bin/env python3`なので、有効にしたvenvのPythonで動いてしまう。
- 2026-11-17に、この方法で、ブリッジのモジュールが読み込めること、起動中の0.9.16のサーバーにつながること(クライアント0.9.16、日本の車6台が見える)を確かめた。
- `~/CARLA_0.9.16_jp/LinuxNoEditor/PythonAPI/carla/dist/`にあるwheelはPython 3.12用(`cp312`)だけで、
  Autoware(ROS 2 Humble)のPython 3.10では使えない。PyPIの`carla==0.9.16`にはPython 3.10用(`cp310`)がある。
- 0.9.15に戻すときは3.7。

### 2.0.2 Autowareの地図フォルダ`Town10HD_Opt`を作る

```bash
ln -s Town10HD ~/autoware_map/Town10HD_Opt   # ← 生成物
ls -l ~/autoware_map/Town10HD_Opt/
```

> 📄 **生成物:** `~/autoware_map/Town10HD`へのシンボリックリンク(2.4.1の`map_path`)\
> `~/autoware_map/Town10HD_Opt`

- ブリッジは、`map_path`のフォルダ名をCARLAのマップ名として読み込む(`autoware_carla_interface.launch.xml`の`carla_map`)。
  0.9.16に入っているマップは`Town10HD_Opt`だけなので、フォルダ名を`Town10HD_Opt`にする(3.1.2)。
- 中身(Lanelet2地図・点群・`map_projector_info.yaml`)は`Town10HD`と同じものを使う。
  CARLAのOpenDRIVE(`Town10HD.xodr`と`Town10HD_Opt.xodr`)は、0.9.15・0.9.16とも同じファイル(3.1.2)。
- シンボリックリンクなので、2.2.3で作る地図(`lanelet2_map_with_tl.osm`)も`Town10HD`と共通になる。

## 2.1 【ターミナル1】CARLAサーバー起動

```bash
cd ~/CARLA_0.9.16_jp/LinuxNoEditor
./CarlaUE4.sh
```
コマンドを実行すると、CARLAサーバーが起動し、CARLA(Unreal Engine)の画面が表示される。

- 0.9.15のサーバー(`~/CARLA/CarlaUE4.sh`)が動いていたら、先に止める(どちらもポート2000を使う)。


## 2.2 【ターミナル2】マップのロード → SUMOの信号とCARLAの信号の対応 → 信号入りの地図の作成 → inpxからの信号・需要の生成 → 車両のブループリント表の生成

### 2.2.1 マップのロード

```bash
export PYTHONPATH=$HOME/venvs/carla0916/lib/python3.10/site-packages:$PYTHONPATH   # このターミナルを0.9.16にする(2.0.1)
cd ~/CARLA_0.9.16_jp/LinuxNoEditor/PythonAPI/util
python3 config.py --map Town10HD_Opt
```
コマンドを実行すると、CARLAで読み込まれているマップが Town10HD_Opt に切り替わる(起動直後から Town10HD_Opt のときは、そのまま)。CARLA画面が Town10HD の地図になっていることを確認する。

- 2.2.2〜2.2.5も、この`export`をしたターミナル2で実行する。
- 読み込めるマップは、次のコマンドで確かめられる。0.9.16_jpでは`['Town10HD_Opt', 'AnnotationColorLandscape']`が出る(2026-11-17に確認)。

```bash
python3 -c "import carla; c = carla.Client('localhost', 2000); c.set_timeout(10); print(c.get_available_maps())"
```

### 2.2.2 SUMOの信号とCARLAの信号の対応(網の`linkSignalID`)の作成

SUMOの網(`sumo/net/Town10HD.net.xml`)の信号(`tlLogic`)に、CARLAの信号との対応を書く。
対応は、SUMOの信号のリンクごとの`<param key="linkSignalID:<linkIndex>" value="<CARLAの信号ID>"/>`で、
ブリッジ(2.4.1の`tls_manager:=sumo`)はこれを使って、SUMOの信号の色をCARLAの信号機に書き込む。
Vissim構成の信号の対応表(`signal_mapping_town10hd.json`)に当たるもの。仕組みは3.9を参照。

**注意**:
- **網はリポジトリに入っている(2026-10-21に作ったもの)。作り直すのは、CARLAの信号(OpenDRIVEの`Town10HD.xodr`)を変えたときだけ。**
  0.9.16_jpの`Town10HD.xodr`は0.9.15と同じ(md5 `570c77d196b918813e95750fcb980214`)なので、0.9.16に切り替えても作り直さなくてよい。
- 網を作り直したら、続けて2.2.3の「照合」と、2.2.4(inpxからの信号。網の`linkSignalID`を写す)を行う。
- CARLAサーバーは要らない(OpenDRIVEのファイルから作る)。`netconvert_carla.py`がCARLAのPythonクライアントを読み込むので、2.2.1の`export`をしたターミナル2で実行する。
- `SUMO_HOME`が設定されていること(`echo $SUMO_HOME`で確かめる。この環境では`/usr/share/sumo`)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
XODR=~/CARLA_0.9.16_jp/LinuxNoEditor/CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr
NETCONVERT_CARLA=~/CARLA_0.9.16_jp/LinuxNoEditor/Co-Simulation/Sumo/util/netconvert_carla.py
TMP=$(mktemp -d)
# OpenDRIVEから網を作る(CARLAの信号が付いた網。どのリンクがどのCARLAの信号かを、ここから読む)
python3 $NETCONVERT_CARLA $XODR -o $TMP/ref.net.xml
# 信号を外して、全交差点を優先道路の交差点(priority)にする
netconvert -s $TMP/ref.net.xml --tls.unset 23,189,468,532,719 --offset.disable-normalization true -o $TMP/unset.net.xml
sed 's/type="traffic_light"/type="priority"/' $TMP/unset.net.xml > $TMP/prio.net.xml
netconvert -s $TMP/prio.net.xml --offset.disable-normalization true -o $TMP/base.net.xml
# CARLAの信号のある交差点に、SUMOの信号を置き直す(linkSignalIDを付ける)
python3 tools/sumo_net/add_carla_tls.py $XODR $TMP/base.net.xml --edge-suffix '' \
    --netconvert-carla $NETCONVERT_CARLA -o sumo/net/Town10HD.net.xml   # ← 生成物
```

> 📄 **生成物:** CARLAの信号と対応付けたSUMOの信号の入った網(Town10HD)\
> `sumo/net/Town10HD.net.xml`

- 最後に`Added traffic lights at junctions 189,23,468,532,719 -> sumo/net/Town10HD.net.xml`が出る。
  その前に、交差点ごとに`junction <番号>: landmarks [<CARLAの信号ID>] [<フェーズの状態>]`が出る。
- `--tls.unset`の`23,189,468,532,719`は、`ref.net.xml`で信号の付いた交差点(`grep -o '<tlLogic id="[^"]*"' $TMP/ref.net.xml`で確かめる)。
  CARLAの信号を変えて交差点が変わったときは、ここも変える。
- `WARNING:root:Landmark  has not a valid name.`が出るが問題ない(名前のない信号955・956。CARLAに信号機がない。3.9)。
- `--edge-suffix ''`は、`netconvert_carla.py`の網のedgeのIDに接尾辞がないため(Town01の網は`.0.00`が付く)。
- 確かめ方: `git diff sumo/net/Town10HD.net.xml`。CARLAの信号を変えていなければ、変わるのは先頭のコメント
  (`generated on`の日時と、`sumo-net-file`・`tllogic-files`・`output-file`のパス)の4行だけ(2026-11-25に確かめた)。
  そのときは`git checkout sumo/net/Town10HD.net.xml`で元に戻す
  (2.2.4で作る`.tll.xml`・`.rou.xml`は網のsha256を記録しているので、コメントだけ変わった網をコミットすると、2.2.4のテストが失敗する)。
- 一時ファイルは`rm -r $TMP`で消す。

### 2.2.3 信号入りのLanelet2地図の作成

CARLAの信号状態をAutowareへ直接通知する(2.4.1の`traffic_light.publish:=true`)ときに使う、信号入りの地図を作る。仕組みと注意点は3.5を参照。

**注意**:
- **横断歩道の入った`~/autoware_map/Town10HD/lanelet2_map_with_tl.osm`(2026-11-26以降に作ったもの)があれば、作り直さなくてよい。** 下の照合だけ行う。
  `grep -c 'v="crosswalk"' ~/autoware_map/Town10HD/lanelet2_map_with_tl.osm`が`16`なら、横断歩道が入っている(`0`なら作り直す)。
- 作り直すのは、横断歩道が入っていないとき、またはCARLAの信号・横断歩道・元の地図・停止線をずらす量を変えたとき。
- 2.2.1で`Town10HD_Opt`を読み込んでおくこと。

**照合(毎回の起動では不要。0.9.16に切り替えたときに1回)**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/check_sumo_lanelet2_tl_ids.py sumo/net/Town10HD.net.xml \
    ~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town10HD_Opt/carla_tl.json \
    --report-mixed --group-id-offset 100000
```

- 最後に`OK`(終了コード0)が出ればよい。`--carla-json`は、0.9.15で`carla_tl_to_lanelet2.py`の`--dump-json`で保存したCARLAの信号(0.9.16のサーバーから読んだものではない)。

**作り直し(上の「注意」の対象を変えたときだけ)**

`lanelet2_map.osm`は、0.9.15のときに重複点を直してある(直す前の地図は`lanelet2_map.osm.orig`)。ここでは直さない。
先に横断歩道を足し、その地図に信号を足す(信号入りの地図の名前は今までどおり`lanelet2_map_with_tl.osm`)。
CARLAの横断歩道と信号は、0.9.16のサーバーから読み直す(`--dump-json`。`carla_crosswalks.json`・`carla_tl.json`を上書きする)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
# 0.9.15で読んだ信号を残しておく(0.9.16で読んだものと比べるため)
cp -n ~/autoware_map/Town10HD/carla_tl.json ~/autoware_map/Town10HD/carla_tl_carla0.9.15.json
python3 tools/carla_crosswalks_to_lanelet2.py ~/autoware_map/Town10HD_Opt/lanelet2_map.osm \
    --dump-json ~/autoware_map/Town10HD_Opt/carla_crosswalks.json --force   # ← 生成物
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/Town10HD_Opt/lanelet2_map_with_crosswalk.osm \
    --output ~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl.osm \
    --dump-json ~/autoware_map/Town10HD_Opt/carla_tl.json --stop-line-forward-offset 0 --group-id-offset 100000 --force   # ← 生成物
python3 tools/check_sumo_lanelet2_tl_ids.py sumo/net/Town10HD.net.xml \
    ~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town10HD_Opt/carla_tl.json \
    --report-mixed --group-id-offset 100000
```

> 📄 **生成物:** 横断歩道と信号の入ったLanelet2地図(Town10HD)、途中の横断歩道だけの地図、CARLAの横断歩道と信号の記録\
> `~/autoware_map/Town10HD_Opt/lanelet2_map_with_tl.osm`、`lanelet2_map_with_crosswalk.osm`、`carla_crosswalks.json`、`carla_tl.json`(実体は`~/autoware_map/Town10HD/`)

- `--force`は、すでにある地図を上書きする。元の地図(`lanelet2_map.osm`)は変更しない。
- 横断歩道のツールは`CARLA map Carla/Maps/Town10HD_Opt: 16 crosswalks`と、横断歩道ごとに重なる車線のlaneletを出す。
  `WARNING: crosswalk lanelet ... overlaps no road lanelet`が出たときは終了コードが2になる(3.5.1)。
- 信号のツールで`WARNING: ... not added`が出たときは終了コードが2になる(3.5.1)。
- 0.9.15で読んだ信号と比べるときは、`diff <(python3 -m json.tool --sort-keys ~/autoware_map/Town10HD/carla_tl_carla0.9.15.json) <(python3 -m json.tool --sort-keys ~/autoware_map/Town10HD/carla_tl.json)`。
  違うのが`map`(マップ名。`Town10HD`と`Town10HD_Opt`)だけなら、信号は同じ。

### 2.2.4 Vissimのinpxからの信号制御・交通需要・車両寸法の生成(`Town10HD_vissim`)

Vissim構成の`vissim/Town10HD.inpx`から、`Town10HD_vissim.sumocfg`(2.4.1の「Vissim構成と同じ条件で走らせるとき」)で使う、SUMOの信号制御・交通需要・車両寸法を作る。
作るものと仕組みは3.8を参照。

**注意**:
- **Windows PCのVissimで`Town10HD.inpx`を変えたときだけ実行する。** 今のinpxから作ったファイルはコミット済みなので、変えていなければ不要。
- inpxで何を変えたときも、(1)〜(6)を順に行う(どれも数秒で終わる)。どの設定がどのファイルになるかは3.8.1。
- **網(`sumo/net/Town10HD.net.xml`)は作り直さない。** 信号は、網に重ねる追加ファイル(`.tll.xml`)に書く(3.8.1)。
  inpxの道路網(リンク・コネクタ)を変えても、SUMOの網には入らない(SUMOの網はCARLAのOpenDRIVEから作ったもの)。
  ただし、inpxに歩行者リンク(横断歩道)があるときだけ、今の網に横断歩道と歩道を足した網`sumo/net/Town10HD_vissim.net.xml`を作る((2)の後半)。
- CARLAサーバーは要らない。

**(1) inpxを写す**

Windows PCのVissimで保存した`Town10HD.inpx`を、`simulator/autoware_carla_interface/vissim/Town10HD.inpx`に上書きで置く。
別の名前で保存したときも、この名前で置く(`.sumocfg`・テストがこの名前で読む)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
git status --short vissim/
```

- `M vissim/Town10HD.inpx`が出ればよい(何も出ないときは、inpxが変わっていない)。

**(2) inpxから読んだ内容の確認**

ファイルは書かない。inpxの流入・車両の設定を、SUMOでどうなるかの形で表示する。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/sumo_inpx/vissim_to_rou.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml --report
```

- 次の行が出る。Vissimで変えた値になっているかを確かめる。
  - `流入 <番号>: リンク <Vissimのリンク> -> edge <SUMOのedge> departPos <位置> <台/h> <台数>`(流入ごと)
  - `vissim_c1: <vTypeのID> <割合> <長さ> x <幅> m accel <加速度> decel <減速度> <希望速度>`(vTypeごと)
  - `vType <数>、車 <台数> 台、edge 数 <最小>〜<最大>(中央 <中央値>)`
- `Error: ...`が出たら、inpxにツールが扱えない設定がある(3.8.2)。

続けて、inpxの横断歩道(歩行者リンク)を確かめる。2.2.3の横断歩道の入った地図を使う。

```bash
python3 tools/sumo_inpx/vissim_to_crossings.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \
    --map ~/autoware_map/Town10HD/lanelet2_map_with_tl.osm --report
```

- `歩行者リンクが無い。横断歩道は足さない`が出たら、網はそのまま使う(今の`Town10HD.inpx`はこれ)。(3)へ進む。
- `地図の横断歩道 <id>: 歩行者リンク <番号> -> 交差点 <SUMOの交差点> の edge <edge> ...`が出たら、横断歩道入りの網を作る(下)。

**横断歩道入りの網の生成(上で横断歩道が出たときだけ)**

```bash
python3 tools/sumo_inpx/vissim_to_crossings.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \
    --map ~/autoware_map/Town10HD/lanelet2_map_with_tl.osm --out sumo/net/Town10HD_vissim.net.xml   # ← 生成物
```

> 📄 **生成物:** 今の網に、inpxの横断歩道(crossing)と待ち場所からの歩道を足した網と、その部品\
> `sumo/net/Town10HD_vissim.net.xml`、`sumo/net/Town10HD_vissim.crossings.{nod,edg,con}.xml`

- 作ったときは、(3)・(4)の`--net`を`sumo/net/Town10HD_vissim.net.xml`にし、`sumo/Town10HD_vissim.sumocfg`の`net-file`も
  `net/Town10HD_vissim.net.xml`にする。車のつながり・`linkSignalID`は今の網と同じ(3.8.1)。

**(3) 信号制御の生成**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/sumo_inpx/vissim_to_tll.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \
    --out sumo/tll/Town10HD_vissim.tll.xml   # ← 生成物
```

> 📄 **生成物:** inpxの信号制御から作ったSUMOの信号プログラム(`programID="vissim"`)\
> `sumo/tll/Town10HD_vissim.tll.xml`

- `tlLogic <SUMOの信号> <- 信号制御 <Vissimの信号制御>: <秒数>:<状態> ...`が信号ごとに1行出る。秒数がVissimの信号プログラムと同じかを確かめる。
- 出力先は上書きする。

**(4) 交通需要・車両寸法の生成**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/sumo_inpx/vissim_to_rou.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml \
    --out sumo/rou/Town10HD_vissim.rou.xml   # ← 生成物
```

> 📄 **生成物:** inpxの車両入力・車両構成・車両モデルから作ったSUMOの車両(vTypeと、経路付きの車1台ずつ)\
> `sumo/rou/Town10HD_vissim.rou.xml`

- `sumo/rou/Town10HD_vissim.rou.xml: 車 <台数> 台、vType <数>`が出る。
- 出力先は上書きする。オプション(経路の長さなど)は3.8.1。

**(5) ブループリント表の生成**

(4)で`.rou.xml`が変わったら、ブループリント表も作り直す(作り直さないと、co-simの起動時に`variant_source_changed`で止まる。3.6.3)。
表の作り方は2.2.5の(2)と同じ。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/build_carla_variants.py \
    --sumocfg sumo/Town10HD_vissim.sumocfg \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json \
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules_carla0.9.16_jp.yaml \
    --out src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd_vissim.yaml   # ← 生成物
```

> 📄 **生成物:** SUMO車両-ブループリント割り当て一覧(Town10HD_vissim)\
> `src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd_vissim.yaml`

- `割り当て <数> モデル`が出る。割り当ては2.2.5の(3)のスクリプトのファイル名を`carla_variants_sumo_town10hd_vissim.yaml`に変えて確かめる。
- (3)(4)で何も変わらなかった(`git status`で`.rou.xml`が出ない)ときも、作り直してよい。そのときは`generated_at`(作った日時)だけが変わる。

**(6) 確認**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
git status --short vissim/ sumo/ src/autoware_carla_interface/sumo_integration/data/
python3 -m pytest test/test_sumo_inpx.py test/test_sumo_fleet_plan.py
sumo -c sumo/Town10HD_vissim.sumocfg --end 600 --no-step-log
```

- `git status`: 変えたinpxと、(3)〜(5)で変わったファイルが出る。
- テスト:
  - `test_town10hd_tll_is_reproduced`・`test_town10hd_rou_is_reproduced`は、(3)(4)で作ったファイルが、今のinpxから作り直したものと同じかを確かめる。(3)(4)をしていないと失敗する。
  - 次のテストは、2026-11-24のinpxの値(周期94 s、流入19本、車種の割合と寸法、ブループリントの割り当て)を期待値にしている。
    inpxでその値を変えたときに失敗するのは正しい。Vissimで変えた値と合っていれば、テストの期待値を直す。

    | テスト | 期待値にしているもの |
    | --- | --- |
    | `test_sumo_inpx.py`の`test_town10hd_inputs_map_to_one_edge_each` | 流入の数(19)と、そのSUMOのedge |
    | `test_sumo_inpx.py`の`test_town10hd_controllers`・`test_town10hd_tll_values` | 信号制御の数(5)・周期(94 s)・青42・黄3・全赤2 s |
    | `test_sumo_inpx.py`の`test_town10hd_types` | 車種の割合・寸法・追従モデル・希望速度(30〜35 km/h) |
    | `test_sumo_fleet_plan.py`の`test_town10hd_vissim_plan_loads` | vTypeのIDと、そのブループリント(`TOWN10HD_VISSIM_ASSIGNMENT`) |

- SUMO単体: エラーなく`Simulation ended at time: 600.00`まで進めばよい。今のinpx(2026-11-24)では600 sまでにテレポートは出ない。
- 確かめたら、変えたinpxと(3)〜(5)で作ったファイルをautoware_universeにコミットする。
  `--symlink-install`でビルドしていれば、ビルドし直さなくても使われる。
- Vissim構成のブランチ(`feature/vissim_windows_co-sim`)でも、同じinpxを`vissim/Town10HD.inpx`に置く(3.8.1)。

### 2.2.5 SUMOの車両のCARLAブループリント表の生成

SUMOの車両1台ごとに、どのCARLAのブループリントを使うかを決める表(`sumo_integration/data/carla_variants_sumo_<.sumocfgの名前を小文字>.yaml`)を作る。
仕組みは3.6を参照。

ブリッジは、2.4.1の`sumo_cfg_file`の`.sumocfg`の名前から、使う表を決める(`sumo_carla_variants:=auto`の既定)。

| `sumo_cfg_file` | 使う表 | 表がないとき |
| --- | --- | --- |
| `.../Town10HD.sumocfg` | `carla_variants_sumo_town10hd.yaml` | 今までの選び方(vTypeのIDのブループリント、なければ`vtypes.json`から乱数) |
| `.../Town10HD_vissim.sumocfg` | `carla_variants_sumo_town10hd_vissim.yaml`(リポジトリに入っている。inpxを変えたら2.2.4の(5)で作り直す) | 同上 |
| `.../Town01.sumocfg` | `carla_variants_sumo_town01.yaml` | 同上 |

**注意**:
- **Town10HDの表は、作るかどうかを選べる。** 作らなければ、今までの選び方で動く。
  SUMOのvTypeを日本の車に合わせて変えるとき、日本の車を割り当てるには表を作る(3.6.1)。
- 表を作ったあとは、次のどれかを変えたら、作り直す。作り直さないと、co-simの起動時に`Error: CARLA blueprint assignment: ...`で止まる(3.6.3)。

  | 変えたもの | 実行する手順 |
  | --- | --- |
  | CARLAの版(0.9.15 → 0.9.16など) | (1)(2)(3) |
  | SUMOの車両の設定(`.sumocfg`の`route-files`の各ファイル。vTypeの寸法・割合など) | (2)(3) |
  | 割り当ての規則(0.9.16_jpでは`carla_variant_rules_carla0.9.16_jp.yaml`。(2)の`--rules`で渡したもの) | (2)(3) |

- Town01の表(0.9.15で作ったもの)は、0.9.16_jpにはTown01がないので使わない。Town01を読み込める0.9.16を使うときは、
  外形を測ったCARLAの版(`0.9.15`)が接続先(`294096eb1-dirty`)と違うので、`variant_mesh_carla_version_changed`で止まる。(1)(2)(3)で作り直す。
- (1)の計測結果は、Town10HD・Town01で共通(CARLAの版ごとに1つ)。0.9.16_jpの計測結果(修正版の日本の車を含む)はコミット済みなので、測り直すときだけ(1)を行う。
- 作った表と計測結果は、autoware_universeにコミットする。`--symlink-install`でビルドしていれば、ビルドし直さなくても使われる。

**(1) CARLAのブループリントの外形の計測**

CARLAだけを起動した状態で行う(ブリッジ・Autowareは起動しない)。各ブループリントを地下に1台ずつ置き、LiDARで外形を測ってから消す。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/measure_blueprint_mesh.py \
    --out src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla<版>.json   # ← 生成物
```

> 📄 **生成物:** CARLAブループリントの外形(LiDARで測った長さ・幅)一覧\
> `src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla<版>.json`

`<版>`は、ファイルを見分けるための名前。0.9.16_jpでは`get_server_version()`が`294096eb1-dirty`を返し版の番号にならないので、`0.9.16_jp`とする。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/measure_blueprint_mesh.py \
    --out src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json   # ← 生成物
```

> 📄 **生成物:** CARLAブループリントの外形(LiDARで測った長さ・幅)一覧(0.9.16_jp)\
> `src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json`

- ブリッジが照合するのは、ファイル名ではなく、ファイルの中に記録された版(`carla_server_version`。ツールが接続先のCARLAから書き込む。0.9.16_jpでは`294096eb1-dirty`)。
- 日本の車6台(`vehicle.jp.*`)も測られ、割り当ての候補になる。タクシー(`vehicle.jp.taxi`)・路線バスなど塗装に意味がある車を候補から外すときは、
  規則(`carla_variant_rules_carla0.9.16_jp.yaml`)の`excluded`に足す(0.9.15のパトカー・タクシーと同じ扱い。まだ決めていない)。
- 出力先のファイルがあるとツールはエラーで止まる。

| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `--out` | 計測結果(JSON)の出力先。すでにあるとエラー | 省略不可 | なし |
| `--host` / `--port` | CARLAサーバー | 省略可 | `localhost` / `2000` |
| `--only` | 測るブループリントを限る(例: `--only vehicle.lincoln.mkz_2017`) | 省略可 | すべて |
| `--exclude` | 測らないブループリント(fnmatchの形、複数可) | 省略可 | なし |

**(2) 表の生成**

co-simで使う`.sumocfg`(2.4.1の`sumo_cfg_file`)をそのまま入力にする。CARLAサーバーは要らない。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/build_carla_variants.py \
    --sumocfg sumo/<Town>.sumocfg \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla<版>.json \
    --rules src/autoware_carla_interface/data/carla_fleet/<規則>.yaml \
    --out src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_<townを小文字>.yaml   # ← 生成物
```

> 📄 **生成物:** SUMO車両-ブループリント割り当て一覧\
> `src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_<townを小文字>.yaml`

- 表には、`.sumocfg`のパスを、表のあるフォルダからの相対パスで記録する(`../../../../sumo/<Town>.sumocfg`)。
- `<Town>`は、2.4.1の`sumo_cfg_file`の`.sumocfg`の名前(`Town10HD`など)。`<townを小文字>`は、その小文字(`town10hd`)。
  この名前にしないと、ブリッジが表を見つけない(今までの選び方で動く)。
- `<版>`は、(1)で付けた名前(`0.9.16_jp`)。
- `<規則>`は、その版の割り当ての規則。0.9.16_jpでは`carla_variant_rules_carla0.9.16_jp.yaml`。
  0.9.15用の`carla_variant_rules.yaml`(0.9.15のTown01の表が使う)と違い、大型車(HGV・BUS)の幅の上限を0.45 mにし、
  ブループリントの分類(`base_type`)で候補を絞る(3.6.1)。

**Town10HD**(0.9.16_jp。2.4.1の起動コマンドで使う)

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/build_carla_variants.py \
    --sumocfg sumo/Town10HD.sumocfg \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json \
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules_carla0.9.16_jp.yaml \
    --out src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd.yaml   # ← 生成物
```

> 📄 **生成物:** SUMO車両-ブループリント割り当て一覧(Town10HD)\
> `src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd.yaml`

- 日本の車を割り当てたいvTypeは、`length`・`width`を、(1)で測ったその車の外形(計測結果の`mesh_length_m`・`mesh_width_m`)に合わせておく。
  規則は、除外していない、分類の合うブループリントから寸法(長さ、同じなら幅)が最も近いものを選ぶ(日本の車を優先するわけではない)。
  諸元の寸法のままだと、CARLA標準の車のほうが近くなったり、ミラーの分で幅の上限(乗用車+0.35 m、トラック・バス+0.45 m)を超えて外れたりすることがある。
  大型車(バス・トラック)が外れると、`carlacola`を重ねた形になる(3.6.1)。
- 作り直すときは、同じコマンドで上書きする。

**Town01**(Town01を読み込める0.9.16を使うときだけ)

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/build_carla_variants.py \
    --sumocfg sumo/Town01.sumocfg \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.16_jp.json \
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules_carla0.9.16_jp.yaml \
    --out src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml   # ← 生成物
```

> 📄 **生成物:** SUMO車両-ブループリント割り当て一覧(Town01)\
> `src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml`

- 表は0.9.15用のものを上書きする。0.9.15に戻すときは、`git checkout`で0.9.15用の表に戻す(3.7)。

共通:
- 割り当てられないモデルがあると、`[build_carla_variants] 止めた: variant_model_empty: ...`と出て、ファイルを書かない。
  規則を見直すか、`.rou.xml`でそのvTypeの割合を0にする。
- 歩行者のvTypeは表に入らない。割合0のvTypeは`skipped`に入る。

**(3) 確認**

**Town10HD**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 - <<'EOF'
import yaml
d = yaml.safe_load(open('src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town10hd.yaml'))
for key, m in d['assignment'].items():
    bodies = ' + '.join(a['blueprint'] for a in m['actors'])
    print(f"{','.join(m['vtypes']):45s} -> {bodies:40s} dL={m['dL']:+.3f} dW={m['dW']:+.3f} share={m['share']}")
print('skipped(割合0):', ', '.join(d['skipped']))
EOF
```

- vTypeごとに、割り当てたブループリント(重ね置きは`+`でつないだもの)・長さの差・幅の差・割合を1行ずつ出す。
  日本の車を割り当てたいvTypeに、意図した`vehicle.jp.*`が付いているかを確かめる。
- Town10HDの表のテストはない(下のテストはTown01の割り当てを期待値にしている)。

**Town01**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
git diff src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml
python3 -m pytest test/test_carla_fleet.py test/test_build_carla_variants.py test/test_sumo_fleet.py test/test_sumo_fleet_plan.py
```

- 表の`assignment`(モデルごとのブループリント・dL・dW)を見て、意図した割り当てか確かめる。
- テストは、0.9.15のTown01の割り当て(Vissim構成と同じ)を期待値にしている。0.9.16で割り当てが変わったら、テストの期待値も直す。

**起動して確かめる**(Town10HD・Town01共通)

2.4.1で起動し、起動ログに使った表が出ることを確かめる。

```bash
grep -n "CARLA blueprints of SUMO vehicles" /tmp/autoware_carla.log
```

- Town10HDの表を作ったときは`CARLA blueprints of SUMO vehicles: .../carla_variants_sumo_town10hd.yaml (<モデルの数> models, sumo_carla_variants=auto)`。
- 2.5.1で止めると、ブループリントごとのspawn回数が1行出る(`CARLA bodies spawned for SUMO vehicles: ...`)。


## 2.3 【ターミナル3】ビルド

0.9.15から変わらない。CARLAを差し替えただけなら、ビルドし直す必要はない(`carla`はPythonのパッケージとして実行時に読み込まれる)。

### 通常のビルド(推奨)

```bash
source /opt/ros/humble/setup.bash
cd ~/autoware.1.9.0
colcon build --packages-select autoware_carla_interface --symlink-install
```

### 初回ビルド・依存関係も含める場合

```bash
colcon build --packages-up-to autoware_carla_interface
```

### ビルドオプション

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

### ビルド後

```bash
source ~/autoware.1.9.0/install/setup.bash
```

## 2.4 【ターミナル3】SUMO/CARLA/Autoware起動

### 2.4.1 起動コマンド

```bash
source install/setup.bash
export PYTHONPATH=$HOME/venvs/carla0916/lib/python3.10/site-packages:$PYTHONPATH   # ブリッジを0.9.16のクライアントで動かす(2.0.1)
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town10HD_Opt \
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_sumo:=true \
  sumo_gui:=true \
  sumo_cfg_file:=Town10HD.sumocfg \
  tls_manager:=sumo \
  traffic_light.publish:=true \
  sync_vehicle_lights:=true \
  sync_vehicle_color:=true \
  sumo_warmup_time:=60 \
  spawn_point:="30.02,133.95,0.30,0.0,0.0,-179.7" \
  spectator_follow:=true \
  2>&1 | tee -i /tmp/autoware_carla.log
```

0.9.15版のTown10HDの起動コマンドとの違いは`map_path`の1行だけ(`Town10HD` → `Town10HD_Opt`)。

- 起動ログの`[CARLA Interface] Map Path: '.../Town10HD_Opt' -> Loading CARLA Map: 'Town10HD_Opt'`で、読み込むマップ名を確かめられる。
- `map_path`のフォルダ名(`Town10HD_Opt`)が、2.2.1でCARLAに読み込んだマップと同じであること。
  `map_path:=$HOME/autoware_map/Town10HD`のままにすると、ブリッジが`Town10HD`というマップを読み込もうとするが、0.9.16_jpにはない(3.1.2)。
- `sumo_cfg_file:=Town10HD.sumocfg`は、このリポジトリの`sumo/Town10HD.sumocfg`(インストール先の`share/autoware_carla_interface/sumo/`)を指す(0.9.15と同じ。3.1.3)。
  `--symlink-install`でビルドしていれば、`sumo/`のファイルを書き換えてもビルドし直さなくてよい。ファイルを足したときと、`--symlink-install`なしのときはビルドし直す。
- `sumo_warmup_time`は`60`にする。Town10HDのSUMOのルートは0〜199秒に計200台だけなので、`600`にするとEGOのスポーン時には車がいなくなっている。
- スポーン地点は信号954の手前(3.5.1の「Town10HD」)。
- 起動ログに、SUMOの車両のブループリントをどう選ぶかが1行出る(3.6)。
  - 2.2.5でTown10HDの表を作ったとき: `CARLA blueprints of SUMO vehicles: .../carla_variants_sumo_town10hd.yaml (<モデルの数> models, sumo_carla_variants=auto)`
  - 作っていないとき: `CARLA blueprints of SUMO vehicles: legacy (vType id, else vtypes.json) (sumo_carla_variants=auto: no carla_variants_sumo_town10hd.yaml ...)`(今までの選び方)

| オプション | 説明 | 設定例 | デフォルト値(未指定時) |
|-----------|------|--------|--------------------------|
| `simulator_type` | 使用するシミュレータ | `carla` | なし(指定必須) |
| `map_path` | Lanelet2マップ。フォルダ名がCARLAのマップ名になる | `$HOME/autoware_map/Town10HD_Opt` | なし(指定必須) |
| `vehicle_model` | 車両モデル | `sample_vehicle` | Launch既定値 |
| `sensor_model` | センサキット | `carla_sensor_kit` | Launch既定値 |
| `use_sumo` | SUMO連携 | `true` | `false` |
| `sumo_cfg_file` | SUMO設定。相対パス(`Town01.sumocfg`など)は、インストール先の`share/autoware_carla_interface/sumo/`(ソースの`sumo/`)から数える。絶対パスはそのまま使う。見つからなければ起動時に`Error: invalid parameters: sumo_cfg_file ... not found`で止まる。起動ログに`SUMO config: <使ったパス>`が出る | `Town10HD.sumocfg` | なし |
| `sumo_carla_variants` | SUMOの車両に割り当てるCARLAのブループリントの表。`auto`は`sumo_integration/data/carla_variants_sumo_<.sumocfgの名前を小文字>.yaml`があれば使い、なければ今までの選び方。`legacy_vtypes`は今までの選び方(vTypeのIDのブループリント、なければ`vtypes.json`から乱数)。ファイル名を渡すと、その表を使う(相対パスは`sumo_integration/data/`から)。3.6参照 | `auto` | `auto` |
| `sumo_gui` | GUI表示 | `true` | `false(ヘッドレス)` |
| `sumo_host` | TraCIホスト | `localhost` | `localhost` |
| `sumo_port` | TraCIポート | `8813` | `8813` |
| `sumo_client_order` | クライアント順 | `1` | `1` |
| `tls_manager` | 信号管理 | `sumo` | `none` |
| `sync_vehicle_lights` | 灯火同期 | `true` | `false` |
| `sync_vehicle_color` | 車体色同期 | `true` | `false` |
| `spectator_follow` | EGO車両(role_name=`ego_vehicle_role_name`)にCARLAスペクテーターを自動追従させる | `true` | `false` |
| `spawn_point` | EGOのスポーン位置(CARLA座標、`x,y,z,roll,pitch,yaw`) | `"30.02,133.95,0.30,0.0,0.0,-179.7"` | `None`(ランダム) |
| `vehicle_type` | EGOのCARLAブループリント | `vehicle.toyota.prius` | `vehicle.toyota.prius` |
| `sumo_warmup_time` | SUMOだけを先に進める秒数(整数)。0で無効。3.2参照 | `60` | `0`(無効) |
| `ego_spawn_front_margin` | EGOスポーン時に必要な前方車とのすき間 [m]。3.2参照 | `20.0` | `20.0` |
| `ego_spawn_rear_margin` | EGOスポーン時に必要な後方車とのすき間 [m]。3.2参照 | `20.0` | `20.0` |
| `ego_spawn_wait_timeout` | ウォームアップ後に空きを待つ上限秒数(整数)。3.2参照 | `60` | `60` |
| `traffic_light.publish` | CARLAの信号状態をAutowareへ直接通知する(カメラの信号認識は起動しない)。3.5参照 | `true` | `false` |
| `lanelet2_map_file` | Autowareが読み込むLanelet2地図のファイル名(`map_path`内)。信号を使うときは2.2.3の地図 | `lanelet2_map_with_tl.osm` | `lanelet2_map.osm` |
| `traffic_light.force_green` | 起動時に全信号を緑にして固定する。3.5参照 | `true` | `false` |

- `sumo_gui:=true`のsumo-guiは、起動後に自動でシミュレーションを開始する(`--start`)。Runボタンを押す必要はない。
  また、co-simの終了時にはダイアログを出さずに自動で閉じる(`--quit-on-end`)。
- `spectator_follow:=true`はCARLAスペクテーター(自由視点カメラ)をEGO車両に自動追従させる。
  カメラの距離・高さ・角度(`--distance`/`--height`/`--pitch`/`--rate`)はlaunch引数として
  公開されていないため、細かく調整したい場合はこの引数は使わず`ros2 run
  autoware_carla_interface spectator_follow --distance ... --height ...`のように別ターミナルで
  手動起動すること。
- `lanelet2_map_file`・`traffic_light.publish`の2行は、CARLAの信号状態をAutowareへ直接通知する設定(3.5参照)。
  2.2.3の信号入りの地図(`lanelet2_map_with_tl.osm`)を先に作っておくこと。使わない場合はこの2行を省く(カメラによる信号認識になる)。
- ウォームアップを使わない場合は、`sumo_warmup_time`と`spawn_point`の行を省く(EGOは起動直後にランダムな位置へスポーンされる)。
  `spawn_point`だけを指定すれば、ウォームアップなしで固定位置にスポーンする。スポーン地点の選び方は3.2.2。
- `2>&1 | tee -i /tmp/autoware_carla.log`はログ保存用(3.2.3のgrepで使う)。不要なら省いてよい。
  `-i`を付けないと、Ctrl+Cで`tee`も終了し、それ以降(終了処理)のログが残らない。

**Vissim構成と同じ条件で走らせるとき**(`Town10HD_vissim.sumocfg`。3.8)

`vissim/Town10HD.inpx`から作った信号(周期94 s)・交通需要(600台/h)・日本の車の寸法で走らせる。上のコマンドから`sumo_cfg_file`と`sumo_warmup_time`の2行を変える。

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town10HD_Opt \
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_sumo:=true \
  sumo_gui:=true \
  sumo_cfg_file:=Town10HD_vissim.sumocfg \
  tls_manager:=sumo \
  traffic_light.publish:=true \
  sync_vehicle_lights:=true \
  sync_vehicle_color:=true \
  sumo_warmup_time:=100 \
  spawn_point:="30.02,133.95,0.30,0.0,0.0,-179.7" \
  spectator_follow:=true \
  2>&1 | tee -i /tmp/autoware_carla.log
```

- `sumo_warmup_time:=100`は、Vissim構成の起動手順書のウォームアップ(`vissim_warmup_time:=100`)と同じ。
- 起動ログで確かめること:
  - `SUMO config: .../sumo/Town10HD_vissim.sumocfg`
  - `CARLA blueprints of SUMO vehicles: .../carla_variants_sumo_town10hd_vissim.yaml (8 models, sumo_carla_variants=auto)`
- 600 sを過ぎると、網が混んで車がほとんど動かなくなる(3.8)。試験は600 s以内に収める。

### 2.4.2 目的地の設定と自動運転の開始

- ウォームアップを使うときは、ログに`[EGO SPAWN]`が出て、RVizにEGOが表示されてから操作する(それまでCARLAにEGOはいない)。
- RVizで目的地(2D Goal Pose)を置き、Autoを押す。Town10HDでは、スポーン地点の先の交差点を抜けた道路5の上、map座標で(-104, -99)付近(北北西向き)に置く(3.5.1)。
- **EGOがスポーンしてから300秒以上、走り出さずに止まっていると**、SUMOがSUMOの中のEGOを消し、ブリッジが入れ直す(ログに`[sync] sumo removed CARLA-origin vehicle carla0 ...; adopting it into sumo again`)。co-simは続く(3.6.4)。

## 2.5 【ターミナル2】終了

### 2.5.1 止め方

途中で止めるときは、**別のターミナルで次を実行する**。

```bash
pkill -INT -f "lib/autoware_carla_interface/autoware_carla_interface --ros-args"
```

- `autoware_carla_interface`が、CARLAの車両・センサーの片付けと、SUMO(TraCI)の切断を済ませてから終了する。sumo-guiも閉じる。
- そのあと、`e2e_simulator.launch.xml`で起動した全ノードが自動で止まる(`autoware_carla_interface.launch.xml`の`on_exit="shutdown"`)。
- 止まったことは、2.4.1のターミナルのログで確かめる: `Cleanup complete.`と`process has finished cleanly`。
  車両のブループリント表を使っているときは、その前にブループリントごとのspawn回数が1行出る(`CARLA bodies spawned for SUMO vehicles: ...`)。
- 止めるときに出る、RViz(`Cannot create GL vertex buffer`など)・`map_hash_generator`のエラーや、`republish`の`exit code -11`は、Autowareのノードの止まり方によるもので、実害はない。
- CARLAサーバー(ターミナル1)は、ウィンドウを閉じるかCtrl+Cで止める。

**注意(2.4.1のターミナルでCtrl+Cを押した場合)**: これでも止まるが、`e2e_simulator.launch.xml`全体にSIGINTが送られ、launchは5秒後にSIGTERM、さらに10秒後にSIGKILLを送る。
`autoware_carla_interface`の片付けが間に合わないと、後片付けのログ(と要約)が途中で切れることがある。
Ctrl+Cで止めたときに、最後に`[ERROR] [launch]: Caught exception in launch ...: Cannot shutdown a ROS adapter that is not running`が1行出ることがあるが、launch側のメッセージで実害はない。

### 2.5.2 自動で止まるとき

次のいずれかで`autoware_carla_interface`ノードが終了し(終了コード1)、それに伴って`e2e_simulator.launch.xml`で起動した**全ノードが停止する**(`on_exit="shutdown"`による)。

| きっかけ | ログ | 対処 |
| --- | --- | --- |
| 起動パラメータの組み合わせが正しくない | `Error: invalid parameters: ...` | 3.2(ウォームアップの条件)などを見て直す |
| ウォームアップ後の試験開始失敗(空きが`ego_spawn_wait_timeout`秒以内にできなかった、EGOのスポーンに失敗した) | `Error: test start failed: ...` | 3.2.4 |
| ブループリントの表が使えない(起動時)。表がない、作ったときと`.rou.xml`・規則・実メッシュ・CARLAの版が違う、など | `Error: CARLA blueprint assignment: <code>: {...}` | **SUMOを起動する前に止まる**。3.6.3 |
| 走行中に、表にないvTypeの車、または寸法の違う車が来た | `Error: CARLA blueprint assignment: npc_model_unknown: {...}` など | 3.6.3 |

- ウォームアップ中・空き待ち中に止めると、EGOをスポーンせずに後始末して終了する。
- `sumo_gui:=true`のsumo-guiは、終了時にダイアログを出さずに自動で閉じる(`--quit-on-end`)。

# 3. 参考

2章の手順で使う機能の説明と、今の設定の記録。起動のたびに読む必要はない。
0.9.15から変わらない機能の詳しい記録(Town01・Town05の確認結果など)は、0.9.15版の「SUMO-CARLA-Autoware_co-sim_起動手順.md」の3章を参照。

## 3.1 CARLA 0.9.16(0.9.16_jp)で変わったこと

2026-11-17に、`~/CARLA_0.9.16_jp/LinuxNoEditor`のファイルと、起動したサーバーで確かめた。
**SUMO-CARLA-Autowareのco-simを0.9.16で通しで動かす確認は、まだしていない**(3.1.5)。

### 3.1.1 Pythonクライアント

- 0.9.15のクライアント(`pip`の`carla 0.9.15`)で0.9.16_jpのサーバーにつなぐと、
  `WARNING: Version mismatch detected ... Simulator API version = 294096eb1-dirty`のあと`std::bad_alloc`で落ちる(コアダンプ)。
  そのため、2.0.1で0.9.16のクライアントを用意し、0.9.16を使うターミナルではそちらを使う。
- 0.9.16のクライアント(2.0.1の方法)では、`Version mismatch`の警告は出るが(サーバーの版が`294096eb1-dirty`のため)、つながる。
  `get_available_maps()`は`['Town10HD_Opt', 'AnnotationColorLandscape']`、`vehicle.jp.*`の6台が見えた(2026-11-17)。
- サーバーの版(`get_server_version()`)は`294096eb1-dirty`(CARLAのgitのコミットを元にした独自ビルドのため)。`VERSION`ファイルも同じ値。
- 配布物の`PythonAPI/carla/dist/`には`carla-0.9.16-cp312-cp312-linux_x86_64.whl`だけがある。Python 3.10用は、PyPIの`carla==0.9.16`(`cp310`)を使う。

### 3.1.2 マップ

- `CarlaUE4/Content/Carla/Maps/`のマップは`Town10HD_Opt`・`OpenDriveMap`・`TestMaps/EmptyMap`だけ。
  Town01〜Town07・Town10HD(`_Opt`でないもの)は入っていない。OpenDRIVE(`Maps/OpenDrive/*.xodr`)はTown01〜Town10HDのすべてがある。
- `Town10HD_Opt`は、Town10HDの建物などを層(レイヤー)に分けた版で、道路は同じ。
  `Town10HD.xodr`と`Town10HD_Opt.xodr`は、0.9.15・0.9.16の4つとも同じファイル(md5 `570c77d196b918813e95750fcb980214`)。
  そのため、0.9.15で作ったTown10HDのSUMOのnet・Lanelet2地図・信号の対応付けを、そのまま使える。
- ブリッジは、`map_path`のフォルダ名をCARLAのマップ名にし(`autoware_carla_interface.launch.xml`の`carla_map`)、
  今のマップと名前が違えば`load_world`で読み込み直す。0.9.16_jpには`Town10HD`という名前のマップがないので、
  フォルダ名を`Town10HD_Opt`にする(2.0.2)。
- Town01・Town05を使うには、そのマップを含むCARLA 0.9.16(公式の`AdditionalMaps_0.9.16`など)が要る。
  0.9.16_jpは独自ビルドなので、公式の追加マップを`ImportAssets.sh`で入れられるかは確かめていない。

### 3.1.3 SUMOのシナリオ

- co-simのSUMOのシナリオは、CARLAの版によらず、このリポジトリの`simulator/autoware_carla_interface/sumo/`のものを使う
  (2026-11-17に`~/CARLA`(0.9.15)の`main`から移した。0.9.15版の手順書の1章)。
- `~/CARLA_0.9.16_jp/LinuxNoEditor/Co-Simulation/Sumo/examples/`は、CARLA公式の配布のまま(Town01・Town04・Town05だけ)で、co-simでは使わない。
  Town10HDのシナリオも、Town01・Town05の信号(CARLAの信号と紐づけたもの)も入っていない。
- `sumo/carlavtypes.rou.xml`(CARLAの車の寸法のvType)は、0.9.15のCARLAから作ったもの(2026-10-01、41種)。
  0.9.16の寸法(日本の車を含む)にするには、0.9.16のクライアントで`~/CARLA`の`util/create_sumo_vtypes.py`を使って作り直す(未実施)。
- co-simのSUMO-CARLAブリッジは`autoware_carla_interface`の中(`sumo_integration/`)にあり、`Co-Simulation/Sumo/`のPythonは使わない。
  使うのは`.sumocfg`とnet・ルートのファイルだけ。

### 3.1.4 日本の車(`vehicle.jp.*`)

- 0.9.16_jpには、CARLA標準の車に加えて、日本の車6台が入っている(一覧は`~/CARLA_0.9.16_jp_incoming/jp_vehicles.json`)。
  例: `vehicle.jp.taxi`(タクシー)、`vehicle.jp.minivan`(ミニバン)、`vehicle.jp.alphard`(大型ミニバン)など。
- 入っているかは、受け渡し用のツールで確かめられる(CARLAを起動した状態。2.0.1の0.9.16のクライアントを使う)。

```bash
cd ~/CARLA_0.9.16_jp_incoming
python3 check_jp_vehicles.py --out check_out   # ← 生成物
```

> 📄 **生成物:** 日本の車6台の確認結果(全部合格なら終了コード0)\
> `~/CARLA_0.9.16_jp_incoming/check_out/check_jp_vehicles.json`

- Town10HDの表(2.2.5)がないときは、日本の車は使われない。今までの選び方(3.6)は、vTypeのIDと同じ名前のCARLA標準の車か、
  `sumo_integration/data/vtypes.json`の車から選ぶため。
- 日本の車を使うには、SUMOのvTypeの寸法を日本の車の外形に合わせ、2.2.5の(1)(2)(3)でTown10HDの表を作る。

### 3.1.5 まだ確かめていないこと

| 項目 | 確かめ方 |
| --- | --- |
| 2.4.1の起動コマンドで、EGOがスポーンし、目的地まで自動運転できるか | 2.4.1・2.4.2 |
| Town10HDのSUMOのルートの車(`rou/Town10HD.rou.xml`のvType。`vehicle.audi.a2`などCARLAのブループリント名)が、0.9.16_jpにすべてあるか | 起動ログに`sumo vtype ... not supported`が出ないこと |
| センサー(カメラ・LiDAR・GNSS・IMU)のトピックが0.9.15と同じに出るか | RViz、`ros2 topic hz` |
| 信号の対応付け(`Traffic-light matching: 15 matched, ...`) | 3.5.2のgrep |
| 0.9.16のサーバーから読み直したCARLAの信号が、0.9.15の`carla_tl.json`と同じか | 2.2.3の「作り直し」のあとの`diff` |

## 3.2 ウォームアップ・EGO安全スポーン

SUMOだけを先に所定時間進めて交通流を作ってから、EGOのスポーン位置の前後が空いたタイミングでEGOをスポーンする機能。
`sumo_warmup_time`に1以上を指定すると有効になり、指定しない(既定の0)場合は従来と同じ動作になる。
設計は`docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md`を参照。

### 3.2.1 動作と条件

- 起動の流れ: Autoware起動 → SUMOだけを`sumo_warmup_time`秒進める(CARLAは止まったまま) →
  その間にSUMOに現れた車両をCARLAへまとめてスポーン → EGOのスポーン位置の前後の空きを判定 →
  空いていなければSUMO・CARLAを1ステップずつ進めて再判定 → 空いたらEGOをスポーン → 通常のco-sim。
- 空きの判定: 同じ車線の前方車・後方車とのすき間(バンパー間の距離)が、それぞれ`ego_spawn_front_margin`・
  `ego_spawn_rear_margin`以上で、かつEGOの予定位置に重なる車両・歩行者がいなければスポーンする。
  CARLAで複数の体を重ねて表す大型車(3.6)は、全体の外形(SUMOの前端から最後の体の後端まで)で判定する。
- EGOがスポーンされるまでは、CARLA上にEGOはおらず、RVizにもEGOは表示されない。
- ROSの時刻(`/clock`)はEGOのスポーン時点から0付近で始まる。ウォームアップ・空き待ちの時間は含まれない。

`sumo_warmup_time`に1以上を指定したときは、次を満たさないと起動時に
`Error: invalid parameters: ...`の1行を出して終了する。

- `use_sumo:=true`であること
- `spawn_point`を`x,y,z,roll,pitch,yaw`の6つの値で指定すること(ランダムスポーンでは判定する位置が決まらないため)
- `tls_manager`が`sumo`または`none`であること(`carla`ではウォームアップ中にSUMOの信号が消えたままになるため使えない)
- 必要距離が0以上、`ego_spawn_wait_timeout`が1以上であること

### 3.2.2 スポーン地点の選び方

空きの判定は「スポーン地点が直線区間にある」ことを前提にしている(曲線では前後車の判定がずれる)。
また、SUMO車両が通る車線でないと、空き待ちの確認にならない。

1. 直線区間のスポーン地点の候補を一覧にする(CARLAサーバーを起動し、`Town10HD_Opt`を読み込んでおく)。
   前後50 mに交差点がなく、向きの変化が3°未満のCARLAのスポーン地点を表示する。

```bash
python3 - <<'EOF'
import carla
c = carla.Client('localhost', 2000); c.set_timeout(10)
m = c.get_world().get_map()
def yaw_diff(a, b): return abs((a - b + 180) % 360 - 180)
for i, sp in enumerate(m.get_spawn_points()):
    wp = m.get_waypoint(sp.location)
    if wp.is_junction:
        continue
    pts = []
    for d in range(5, 55, 5):
        pts += wp.next(float(d)) + wp.previous(float(d))
    if len(pts) < 20 or any(p.is_junction or yaw_diff(p.transform.rotation.yaw, wp.transform.rotation.yaw) > 3 for p in pts):
        continue
    l, r = sp.location, sp.rotation
    print(f"#{i:3d} road={wp.road_id:3d} lane={wp.lane_id:2d}  spawn_point:=\"{l.x:.2f},{l.y:.2f},{l.z:.2f},{r.roll:.1f},{r.pitch:.1f},{r.yaw:.1f}\"")
EOF
```

2. 候補の中から、SUMOのルートが通り、かつSUMO車両と同じ向きの車線のものを選ぶ。
   初回は`spectator_follow:=true`で、スポーン後にSUMO車両と同じ向きに並んでいるかを目で確認する。

- 0.9.16では`waypoint.next`/`previous`の不具合(逆向きの車線が続く所でループする)が直っている(CHANGELOG)。
  上の一覧の結果が0.9.15と違うことがある。
- 2.4.1の地点(信号954の手前、`30.02,133.95`)は、Town10HDの0.9.15での確認済みの地点(3.5.1)。
- `spawn_point`のzにはCARLAの道路の高さを入れる。コード側で+2 mしてスポーンする。

### 3.2.3 ログの見方

```bash
grep -nE "SUMO WARMUP|EGO SPAWN|EGO spawn gate" /tmp/autoware_carla.log
```

| ログ | 意味 |
|---|---|
| `EGO spawn gate: vehicle.toyota.prius footprint 4.51 x 2.01 m (...), lane width 4.00 m` | 判定に使うEGOの寸法とその出所、車線幅 |
| `[SUMO WARMUP] start: warmup_time=60 s (1200 steps)` | ウォームアップ開始 |
| `[SUMO WARMUP] t=10.0 s vehicles=1 persons=0 (wall 0.3 s)` | 10秒(シミュレーション時間)ごとの進捗。`wall`は実時間 |
| `[SUMO WARMUP] completed: t=60.0 s vehicles=48 ... (wall 1.9 s)` | ウォームアップ終了時の車両数 |
| `[SUMO WARMUP] caught up: carla_spawned=48 sumo_only=0 ...` | CARLAへまとめてスポーンした台数。`sumo_only`はCARLAに対応する車種がなくSUMOにだけいる台数 |
| `[EGO SPAWN CHECK] start: spawn_point=(...) ego=... lane_width=... front_margin=... rear_margin=... wait_timeout=...` | 空き判定の条件(launch引数が届いているかの確認にも使う) |
| `[EGO SPAWN CHECK] t=60.05 front=... clearance=13.0 m rear=... clearance=-1.2 m overlap=... result=WAIT` | 判定結果。前方車・後方車とのすき間、EGOの位置に重なっている車両、`WAIT`/`SAFE` |
| `[EGO SPAWN] t=60.05 s waited=0.00 s spawn_point=(...) vehicles=48` | EGOをスポーンした時刻と空き待ちの時間 |

- 表の数値は例(Town10HDの車両数は、0.9.15での実績)。
- `sumo_only`が0でないときは、CARLAにないブループリントの車がある(3.1.5)。

### 3.2.4 試験開始失敗

- `ego_spawn_wait_timeout`秒待っても空かない場合や、空いた後にEGOのスポーンに失敗した場合は、
  `Error: test start failed: ...`(最後の判定結果を含む)を出して後始末し、`autoware_carla_interface`が終了コード1で終了する(2.5.2)。
  スポーン地点を変える、`sumo_warmup_time`を変える、`ego_spawn_wait_timeout`を延ばす、などで対処する。

## 3.3 処理時間計測

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town10HD_Opt \
  :
  :
  2>&1 | tee -i /tmp/autoware_carla.log
```
- `2>&1 | tee -i /tmp/autoware_carla.log` でターミナルログをautoware_carla.logに保存
- `-i`を付けること。付けないと、Ctrl+Cで`tee`も終了し、それ以降(終了処理)のログが残らない

```bash
grep "MAIN_LOOP_PERIOD" /tmp/autoware_carla.log
```
- 例えば、ログに[MAIN_LOOP_PERIOD]タグをつけている場合は、上記のコマンドで対象ログを抽出できます。

## 3.4 歩行者(Pedestrian)同期について

上記のco-sim起動(2.4)には、SUMO側の歩行者(`traci.person`)をCARLA側の`walker.pedestrian.*`
アクターとして反映する歩行者同期が組み込まれている。0.9.15から変わらない。

- **常時有効(CLI引数・launch引数は無い)**: SUMO設定(`sumo_cfg_file`)側に歩行者(`personFlow`/`person`)の
  ルート・ネットワークが定義されていれば、追加設定なしに自動的に同期される。
- **sumo→carla の一方向のみ**: CARLA側でspawnした歩行者をSUMOへ送り返す機能(carla→sumo)は実装していない。
- **Z座標補正**: SUMOの`VAR_HEIGHT`の半分(`sumo_person.extent.z`)をZ座標に加算する補正を行っている
  (`BridgeHelper.get_carla_pedestrian_transform()`)。
- 歩行者用のCARLA walkerブループリントは`sumo_integration/data/vtypes.json`の
  `carla_blueprints`に`walker.pedestrian.0001`〜`0051`(`vClass: "pedestrian"`)として登録済み。
- 詳しくは0.9.15版の3.3を参照。

## 3.5 CARLAの信号状態をAutowareへ直接通知する

CARLAが管理している信号の状態(赤・黄・青)を、カメラ画像による信号認識を使わずに、
ブリッジ(`autoware_carla_interface`)から`/perception/traffic_light_recognition/traffic_signals`へ
毎tick publishする機能。設計と確認結果は`docs/CARLA_Autoware_信号状態publish_実装計画_v1.0.md`を参照。
`tls_manager:=sumo`のときは、SUMOが決めた信号状態がCARLA経由でそのままAutowareに届く。
Town10HDのSUMOのnet(`sumo/net/Town10HD.net.xml`)には、CARLAの信号15個すべてに対応する信号がある
(設置方法と信号パターンは`~/CARLA/Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`の「SUMOの信号(Town10HD)」を参照)。

### 3.5.1 信号入りのLanelet2地図(Town10HD)

標準のCARLA用Lanelet2地図には信号の情報がないため、そのままではAutowareが信号で止まらない。
CARLAから信号を読み取って地図に追加するツールで、信号入りの地図を作る(コマンドは2.2.3)。
以下は0.9.15で2026-10-21に作ったときの記録。0.9.16でもOpenDRIVEが同じなので変わらない(3.1.2)。

- 地図・点群は2026-07-16に置いたもの(CARLA autoware-contents の vector_maps/lanelet2・point_cloud_maps)。
  配布元の地図は10本のlaneletで境界線に同じ位置の点が続くので、先に直す(2.2.3)。
- 信号は15個(landmark 943〜945・949〜954・957〜962)。どれも2車線で、計30個のregulatory elementができる
  (`Lanelet2: 15 traffic lights in 30 regulatory elements`)。照合の結果は`OK`、`INFO`(赤に置き換えられる信号)はなし。
- `--group-id-offset 100000`: 付けないと、地図の既存のID(943など。最大9690)とぶつかって止まる。
- `--stop-line-forward-offset 0`: CARLAの停止位置は、もともと交差点の入口の0.9〜6.9 m手前(中央値5.4 m)。
  1 mでも`WARNING: ... not added`が出る(1 mで4か所、3 mで8か所、6 mで24か所)ので、前に出さない。
- 出力は元の地図と同じフォルダに置く(CARLAの地図名は`map_path`のフォルダ名から決まるため)。
  0.9.16では`Town10HD_Opt`が`Town10HD`へのシンボリックリンクなので、どちらのパスで作っても同じファイルになる。
- `WARNING: ... not added`が出た信号は、停止位置に合うlaneletが見つからなかったもの、または前に出した停止線がlaneletの外に出たもの。終了コードは2になる。
- 照合ツール(`check_sumo_lanelet2_tl_ids.py`)は、最後に`OK`(終了コード0)が出れば、CARLAの全信号がSUMOと地図の両方にある。ずれがあると`NG ...`と、そのIDが出る。
- 横断歩道(2026-11-26に追加。`carla_crosswalks_to_lanelet2.py`):
  - CARLAの`Map.get_crosswalks()`が返す横断歩道の外形(OpenDRIVEの横断歩道。Town10HDは16個、どれも4つの角)を、
    `subtype=crosswalk`のlanelet(`participant:pedestrian=yes`、`one_way=no`)にする。長い2辺を左右の境界線(`type=pedestrian_marking`)にする。
  - 16個とも車線のlaneletと重なる(長さ14〜26 m、幅2.5〜2.6 m)。Autowareの横断歩道モジュールは、経路の車線と重なる横断歩道のlaneletを
    歩行者の経路グラフから見つけるので、横断歩道の規制要素(regulatory element)は足さない(停止線はモジュールが決める)。
  - 信号のツールは、停止位置のlaneletを探すとき、横断歩道のlaneletを飛ばす(車線だけに停止線を付ける)。
    横断歩道を足した地図に信号を足しても、信号の付く車線は足す前と同じ30か所(2026-11-26に確かめた)。
  - 横断歩道を足す前の地図は`lanelet2_map_with_tl_before_crosswalk.osm`として残してある。
  - 横断歩道の位置は、SUMOの横断歩道の位置にも使う(統一仕様`docs/SUMO_inpx変換_統一仕様_v1.0.md`のD2・4章)。
- 信号の手前に向かうスポーン地点: 信号954の手前(CARLAのスポーン地点#80、SUMOの`1_4`、ルートの34台が通る)
  `spawn_point:="30.02,133.95,0.30,0.0,0.0,-179.7"`。CARLAの停止位置まで51 m、手前の交差点から110 m
  (後ろ36 mから先はカーブ)。SUMOの車両の19台がこのedge(`1`)の始点から出発するが、スポーン地点から約110 m後ろ。
  信号954(junction 468)はSUMOで0〜42秒が青、42〜45秒が黄、45〜90秒が赤の繰り返し。
  この車線は交差点を抜けると道路5へ曲がって続くだけなので、ゴールはその先(map座標で(-104, -99)付近、北北西向き)に置く。
- 0.9.15での実機の起動ログ(2026-10-21): `[EGO SPAWN] t=60.05 s waited=0.00 s`(ウォームアップ60秒、空き待ちなし、そのときのSUMOの車両48台)と
  `Traffic-light matching: 15 matched, 0 ambiguous, 0 too far, 0 overridden (map has 62 heads / 30 groups)`。
  `Different SUMO traffic-light states`は出なかった。0.9.16でも同じになるかは未確認(3.1.5)。

### 3.5.2 起動とログの確認

2.4.1の起動コマンド例の次の2行で有効になる。

```bash
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  traffic_light.publish:=true \
```

- `traffic_light.publish:=true`のときは、カメラによる信号認識(traffic_light_recognition)を起動しない。
  同じトピックに2つのノードがpublishしないようにするため。
- ブリッジは、Autowareが読み込む地図(`map_path`/`lanelet2_map_file`)を読み、CARLAの信号を
  地図の信号に位置で対応付ける。

起動ログで対応付けの結果を確認する。

```bash
grep -nE "Traffic-light matching|Traffic lights|traffic-light|force_green" /tmp/autoware_carla.log
```

| ログ | 意味 |
| --- | --- |
| `Traffic-light matching: 15 matched, 0 ambiguous, 0 too far, 0 overridden (map has 62 heads / 30 groups)` | 正常。CARLAの15信号すべてが地図の信号に対応付いた |
| `the map has no traffic-light heads; 15 CARLA lights not published` | 地図に信号がない。`lanelet2_map_file:=lanelet2_map_with_tl.osm`を付け忘れている |
| `ambiguous CARLA light (opendrive_id=...)` / `unmatched CARLA light (...)` | その信号は対応付けできず、publishされない。下の`traffic_light.id_map`で指定できる |

### 3.5.3 オプション

| オプション | 説明 | デフォルト値 |
| --- | --- | --- |
| `traffic_light.publish` | CARLAの信号状態をpublishする | `false` |
| `traffic_light.force_green` | 起動時に全信号を緑にして固定し、終了時に元に戻す。`use_sumo:=true`で`tls_manager`が`sumo`/`carla`のときは指定できない(起動時にエラー) | `false` |
| `traffic_light.map_path` | 対応付けに使うLanelet2地図。空(`""`)にするとOpenDRIVEの信号IDをそのまま使う | `map_path`/`lanelet2_map_file` |
| `traffic_light.match_distance` | 対応付けを認める信号灯どうしの最大距離 [m] | `5.0` |
| `traffic_light.match_ratio` | 曖昧と判定するしきい値(小さいほど厳しい) | `0.6` |
| `traffic_light.id_map` | 手動の対応付け`opendrive_id:group_id[\|group_id...],...`。位置の対応付けより優先する。`\|`を含むので引用符で囲む | `""` |

- 状態の対応: CARLAのRed/Yellow/Green → RED/AMBER/GREEN(点灯)、Off → 消灯、それ以外 → UNKNOWN。
- 同じ信号IDに複数のCARLAの信号が対応して状態が違うときは、最も制限の強い状態(赤 > 黄 > 不明 > 消灯 > 青)を出す。

### 3.5.4 注意点

- **SUMO連携(`tls_manager:=sumo`)では、黄信号もAutowareには赤として届くことがある。**
  1つのCARLAの信号に、状態の違うSUMOの複数のリンクが対応している場合、CARLAの信号は赤になる
  (ログに`Different SUMO traffic-light states ... CARLA state is forced to red.`が出る)。Town10HDでは起きない(0.9.15で確認)。
- RMWは`~/.bashrc`と同じ`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`で起動すること。Fast DDSで起動したときは
  ノードどうしの通信が一部できず、Autoを押せる状態にならなかった。
- 別のTownで、SUMOとLanelet2の両方に信号を置く手順は、Claude Codeのスキル`/carla-town-signals`(`.claude/skills/carla-town-signals/SKILL.md`)にまとめてある。

## 3.6 SUMOの車両のCARLAブループリント

SUMOの車両1台をCARLAに出すとき、どのブループリントを使うかを、**SUMOのvTypeの寸法(長さ・幅)に最も近いもの**に決める。
設計と確認結果は`docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md`を参照。

- 生成ツールで事前に作った表(carla_variants)を、ブリッジは起動時に検査し、走行中はSUMOの車のvTypeのIDで引くだけ。
- どの表を使うかは`sumo_carla_variants`で決まる(2.4.1)。既定の`auto`は、`.sumocfg`の名前から`sumo_integration/data/carla_variants_sumo_<名前を小文字>.yaml`を探し、
  なければ今までの選び方(vTypeのIDと同じ名前のブループリント、なければ`sumo_integration/data/vtypes.json`からvClassで乱数)にする。
- リポジトリにある表はTown01だけ(0.9.15で作ったもの)。Town10HDは、2.2.5で表を作れば表で、作らなければ今までの選び方になる。

### 3.6.1 Town10HDの車両

**表を作らないとき(今までの選び方)**

- `rou/Town10HD.rou.xml`のvTypeのIDは、CARLAのブループリント名そのもの(`vehicle.audi.a2`、`vehicle.audi.tt`、`vehicle.carlamotors.carlacola`など)。
  ブリッジは、そのIDのブループリントをそのまま使う。
- 0.9.16_jpに、そのブループリントがないときは、`vtypes.json`から同じvClassのものを乱数で選ぶか、選べなければ
  `sumo vtype ... not supported`の警告を出して、その車をCARLAに出さない(3.1.5)。

**表を作るとき(2.2.5)**

- vTypeごとに、分類の合う、寸法が最も近いブループリントが割り当たる。vTypeのIDの名前は使わない。
- 0.9.16_jp用の規則(`carla_variant_rules_carla0.9.16_jp.yaml`、2026-11-24。Vissim構成と同じファイル)は、0.9.15用の規則から次の2つを変えた。
  - 幅の上限(dW、太い側)を車種カテゴリごとにした。乗用車・二輪は0.35 m のまま、トラック・バス(HGV・BUS・TRAM)は0.45 m。
    大型車のミラーは乗用車より張り出す。日本の車のdW(ミラー込みの外形 − Vissimのモデルの幅)は、乗用車 +0.18〜+0.34 m、トラック・バス +0.35〜+0.40 m だった。
  - 1体で表す候補を、ブループリントの分類(`base_type`)で絞る(乗用車: car・van・空、トラック: truck、バス: bus、二輪: motorcycle・bicycle)。
    絞らないと、トラックに長さの近いバスが割り当たることがある。`carlacola`の重ね置きは絞らない(バスにも使う)。
  - SUMOのvTypeの車種カテゴリはvClassから決める(passenger → 乗用車、truck・trailer・delivery・emergency → トラック、bus・coach → バス、
    motorcycle・bicycleなど → 二輪)。vClassが乗用車でも、長さが6 mを超えるvTypeはトラックとして扱う。
  - 分類の合う車も重ね置きも使えないモデルがあると、2.2.5の(2)は`variant_model_empty`で止まり、表を書かない。
- 今の`rou/Town10HD.rou.xml`(CARLA標準の車の名前のvType。日本の車のvTypeは無い)では、0.9.16_jp用の規則で作った表と、
  0.9.15用の規則で作った表は、27モデルすべてで同じ割り当てになる(2026-11-24に確認)。
  違いが出るのは、日本のトラック・バスに合わせたvTypeを足したとき。
- 今の(CARLA標準の車の名前の)vTypeのまま表を作ると、今までの選び方と違う車になるvTypeがある。
  2026-11-17に、0.9.15の計測結果で試しに作った結果では、27モデルすべてに割り当てがあり、生成は止まらなかった。
  ただし、27のvTypeのうち11は、IDと違うブループリントが割り当たった。
  - `vehicle.toyota.prius` → `vehicle.mini.cooper_s_2021`(`prius`は自車のブループリントなので、規則で候補から外している)
  - `vehicle.dodge.charger_police` → `vehicle.dodge.charger_2020`(パトカーの塗装は、規則で候補から外している)
  - ほかの9つ(`nissan.micra` → `jeep.wrangler_rubicon`、`ford.mustang` → `tesla.model3`など)は、
    vTypeの寸法(`carlavtypes.rou.xml`。CARLAのバウンディングボックスの値)と、規則が比べるLiDARで測った外形がずれているため。
  - これは、vTypeの寸法を測った外形に合わせていないと、IDの車が選ばれないことの例でもある(2.2.5の(2)の「Town10HD」)。
- SUMOのvTypeを日本の車に合わせたときの注意は、2.2.5の(2)の「Town10HD」。

### 3.6.2 Town01の表(0.9.16で使うとき)

- 0.9.15で作った`carla_variants_sumo_town01.yaml`(14モデル。Vissim構成と同じ割り当て)は、外形を0.9.15で測ったもの。
  0.9.16では使えないので、2.2.5の(1)(2)(3)で作り直す。
- 割り当て(0.9.15)の一覧は、0.9.15版の3.5.1を参照。

### 3.6.3 止まったとき(`Error: CARLA blueprint assignment: <code>: ...`)

| code | 意味 | 直し方 |
| --- | --- | --- |
| `variant_config_missing` | `sumo_carla_variants`のファイルがない | パスを確かめる |
| `variant_source_changed`(`source: route_files`) | 渡された`.sumocfg`の`route-files`の並びか中身(sha256)が、表を作ったときと違う | 表を作り直す(2.2.5の(2)) |
| `variant_source_changed`(`source: rules`・`geometry`) | 表を作ったときと、規則か実メッシュが違う | 表を作り直す(2.2.5の(2)) |
| `variant_source_changed`(`source: vehicle_dimensions`、走行中) | SUMOの車の寸法が、表のモデルと違う | `.rou.xml`を確かめ、表を作り直す(2.2.5の(2)) |
| `variant_blueprint_missing` | 表のブループリントが、接続先のCARLAにない | CARLAの版を確かめ、外形を測り直して表を作り直す(2.2.5の(1)(2)) |
| `variant_mesh_carla_version_changed` | 外形を測ったCARLAの版と、接続先の版が違う(0.9.15の表を0.9.16_jpで使うと、`measured: 0.9.15`・`server: 294096eb1-dirty`) | 外形を測り直して表を作り直す(2.2.5の(1)(2)) |
| `variant_model_ambiguous` | 同じvTypeが2つのモデルにある | 表を作り直す |
| `npc_model_unknown`(走行中) | 表にないvTypeの車が来た | 使っている`.sumocfg`から、表を作り直す(2.2.5の(2)) |
| `variant_model_empty`(起動時・走行中) | 割合 > 0のvTypeに割り当てがない、または表を作ったときに割合0だったvTypeの車が来た | 同上 |

- 起動時の検査は、SUMOを起動する前に行う。そのため、起動時に止まったときはSUMOは起動していない。

### 3.6.4 SUMOがEGOを消したとき

SUMOは、`time-to-teleport`(既定300秒)のあいだ止まったままの車を、渋滞の原因とみなしてteleportする(行き先の辺を越えると消す)。
EGOがスポーンしてから走り出さずに止まっていると、SUMOの中のEGO(`carla0`)がこれで消される。

- ブリッジは、SUMOが消したEGOを、同じステップのうちに新しいID(`carla1`、`carla2` …)でSUMOへ入れ直し、co-simを続ける。ログに次の行が出る。
  `[sync] sumo removed CARLA-origin vehicle carla0 (CARLA actor <id>), e.g. a teleport after standing still for time-to-teleport; adopting it into sumo again`
- EGOが止まっている間、後ろのSUMOの車が詰まるのは今までどおり。

## 3.7 CARLA 0.9.15に戻すとき

0.9.15の環境は変えていないので、**`PYTHONPATH`を足していない新しいターミナル**を使えば、そのまま0.9.15になる(2.0.1)。

```bash
python3 -c "import carla, importlib.metadata as m; print(m.version('carla'))"   # 0.9.15 と出ること(新しいターミナルで)
```

- サーバーは`~/CARLA/CarlaUE4.sh`で起動し、手順は0.9.15版の「SUMO-CARLA-Autoware_co-sim_起動手順.md」に従う
  (`map_path:=$HOME/autoware_map/Town10HD`など)。
- `~/autoware_map/Town10HD_Opt`(シンボリックリンク)は残しておいてよい(0.9.15でも`Town10HD_Opt`のマップはある)。
- 2.2.5でTown01の表を0.9.16用に作り直してコミットしたときは、0.9.15用の表と計測結果をgitで戻す。

## 3.8 Vissim構成と同じ条件のシナリオ(`Town10HD_vissim`)

`sumo/Town10HD_vissim.sumocfg`は、Vissim構成のTown10HD(`vissim/Town10HD.inpx`)から、信号・交通需要・車両寸法を作ったシナリオ(2026-11-24)。
網は今の`net/Town10HD.net.xml`のまま。設計と確認結果は`docs/SUMO_CARLA_Autoware_inpxからの信号・需要生成_実装計画_v1.0.md`、inpxを変えたときの作り直しの手順は2.2.4。

| 項目 | `Town10HD.sumocfg` | `Town10HD_vissim.sumocfg` |
| --- | --- | --- |
| 信号 | 周期90 s(青42・黄3 ×2) | 周期94 s(青42・黄3・全赤2 ×2)。Vissimの信号制御と同じ |
| 交通需要 | 0〜199 sに200台(決まった経路) | 流入19本・合計600台/h(ポアソン到着)で0〜3600 sに582台。経路は車1台ごとに分岐ごとに等分して選び、2000 m進んだら消える |
| 車両 | CARLAの車の名前のvType(27種) | inpxの3Dモデルの寸法のvType 8つ(アルファード・ノア・JPNタクシー・GIGA・Volvo E・エルガ・ガーラ・MT-07) |
| ブループリント表 | 作れば使う(2.2.5) | `carla_variants_sumo_town10hd_vissim.yaml`(リポジトリに入っている)。Vissim構成の表と同じ割り当て(日本の車6台・`carlacola` × 2・`kawasaki.ninja`) |
| 追従モデル | Krauss(SUMOの既定) | Wiedemann(`minGap` 2 m)。希望速度30〜35 km/h |

- 信号は追加ファイル(`tll/Town10HD_vissim.tll.xml`)の新しいプログラム`vissim`で、網のプログラムの代わりに使われる。
  `linkSignalID`は網と同じなので、`tls_manager:=sumo`でCARLAの信号機が同じように動く。
- Vissimの網もSUMOの網も閉じている(出口が無い)ので、流し続けると台数が増える。SUMO単体では次のとおり(2026-11-24):

  | 時刻 [s] | 100 | 300 | 500 | 700 |
  | -------- | --: | --: | --: | --: |
  | 網の中の台数 | 14 | 51 | 86 | 129 |
  | 平均速度 [m/s] | 6.6 | 5.3 | 1.8 | 0.8 |

  700 sまではテレポート・衝突なし。860 sごろからテレポートが出始める。Vissimの車は網から出ないので、Vissimでも同じように溜まる見込み(未確認)。
- 0.9.15でも使える。ただし表は0.9.16_jpで測った外形で作ったので、0.9.15では`variant_mesh_carla_version_changed`で止まる。
  0.9.15では`sumo_carla_variants:=legacy_vtypes`にするか、0.9.15の計測結果と規則で表を作る。

### 3.8.1 inpxから作るもの

2.2.4のツール(`tools/sumo_inpx/`)は、inpxの次の設定を読む。ここにない設定は、Vissimで変えてもSUMOには入らない。

| inpxの設定 | SUMOでの形 | ファイル(2.2.4の手順) |
| --- | --- | --- |
| 信号制御の信号プログラム(ステージ・信号グループ・各色の秒数) | `tlLogic`のプログラム`vissim`のフェーズ(秒数と状態)。オフセット0 | `sumo/tll/Town10HD_vissim.tll.xml`((3)) |
| 信号灯の名前(`CARLA <CARLAの信号ID>`)と信号グループ | どの`tlLogic`のどのlinkIndexを動かすか(網の`linkSignalID`で引く) | 同上 |
| 車両入力(リンク・台/h) | 流入のedge・`departPos`(リンクの位置から幾何で探す)と、ポアソン到着の車 | `sumo/rou/Town10HD_vissim.rou.xml`((4)) |
| 車両構成(車種・相対交通量・希望速度分布) | vTypeの割合(`vTypeDistribution`)・`speedFactor` | 同上 |
| 車両タイプ・2D/3Dモデル分布(モデルの長さ・幅) | vTypeの`length`・`width`・`vClass` | 同上 |
| 加速度・減速度の関数 | vTypeの`accel`・`decel` | 同上 |
| 運転挙動(Wiedemann 74の`ax`) | `carFollowModel="Wiedemann"`・`minGap` | 同上 |
| 静的経路の経路決定(車両入力のリンクにあるもの。経路のリンク列・行き先・割合) | 車の`route`(経路に沿うedge列)と`arrivalPos`(行き先) | 同上 |
| 希望速度決定(車両クラスごとの希望速度分布) | 決定点を通る車のvTypeの割合(変更後の分布の`vTypeDistribution`) | 同上 |
| 歩行者リンク(横断歩道)・待ち場所 | 網の横断歩道(crossing。位置と幅は2.2.3の地図の横断歩道)と、待ち場所からの歩道`ped_area<エリアの番号>` | `sumo/net/Town10HD_vissim.net.xml`((2)の後半) |
| 歩行者信号灯(歩行者リンクの上の信号灯)の信号グループ | 横断歩道のlinkIndexの色(青だけ`G`) | `sumo/tll/Town10HD_vissim.tll.xml`((3)) |

- 網(`.net.xml`)は変えない。信号は追加ファイル`.tll.xml`に、網と同じIDの`tlLogic`の新しいプログラム`vissim`として書く。
  SUMOは最後に読んだプログラムを使うので、`.sumocfg`の`additional-files`で読ませると、網のプログラム(周期90 s)の代わりにこちらが使われる。
  網を作り直さないので、`linkSignalID`(CARLAの信号との対応)と、Lanelet2地図の信号との対応(2.2.3)は変わらない。
- 信号は、**秒数・色の順は自由に変えられるが、一緒に青になるSUMOのリンクの組は、網の今のプログラムと同じでなければならない**
  (青の`G`/`g`を網のプログラムから写すため)。信号灯を足す・信号グループの組を変えると、(3)は止まる(3.8.2)。
  組を変えるときは、網の信号から作り直す(スキル`carla-town-signals`の手順)。
- 経路:
  - 経路決定の無い車両入力(今の`Town10HD.inpx`はすべてこれ。車はコネクタを選びながら走る)は、車1台ごとに、分岐ごとに次のedgeを
    等しい確率で選び、長さが`--route-length`(既定2000 m)に達したedgeの終わりで消える。
  - 車両入力のリンクに静的経路の経路決定があるときは、経路のリンク列(コネクタを含む)の近くを通るedge列を探し、行き先
    (行き先リンクの`destPos`)で消える。edge列が途中で分かれるときは分岐ごとに等分する。車1台ごとに、経路の割合×分岐の割合で選ぶ。
    行き先が交差点の中にあるときは、交差点を出たedgeの頭で消える。SUMOの網で着けない経路は、同じ車両入力の残りの経路に割合を寄せ、
    作ったファイルの先頭のコメントに書く。
- 希望速度決定: 決定点を通る車(経路決定の無い車も、その車のedge列で調べる)に、変更後の速度分布を**出発時から**使う
  (SUMOの車は走っている途中でvTypeを変えられない)。決定点を2つ以上通るときは、最初に通るものを使う。
- 横断歩道(2026-11-26に追加。`vissim_to_crossings.py`。統一仕様`docs/SUMO_inpx変換_統一仕様_v1.0.md`の4・5章):
  - 歩行者リンク(コネクタでないもの)ごとに、中ほどが地図の横断歩道の中心線の近く(半幅 + 1 m)にあり、向きが30度以内の横断歩道を対応付ける。
  - 渡る車の道は、横断歩道の中心線の近く(8 m以内)で交差点に入る・出る、横断歩道と45度以上交わる道。Town10HDの地図の横断歩道16個は、
    どれも1つの交差点の、入る道と出る道の2本になる(2026-11-26に確かめた)。
  - 横断歩道は地図の横断歩道の両端から1 mずつ延ばし、`priority="true"`(車が歩行者に譲る)にする。
  - 待ち場所(歩行者入力・歩行者の経路が使うエリア)が横断歩道の端の近く(15 m以内)にあれば、横断歩道の延長線上に歩道を足す。
    無い端は、網の今の歩道(OpenDRIVEから作った歩道)につながる。
  - 網は作り直さず、netconvertで今の網に部品を足す。変わるのは横断歩道のある交差点の信号プログラム(netconvertが横断歩道のlinkIndexを
    車の後ろに足す)と、歩行者の通り道(walkingarea)だけ。車のつながり・linkIndex・`linkSignalID`・ほかの信号は変わらない。
  - 信号のある交差点の横断歩道には、歩行者信号灯が要る。その色は信号グループが青のときだけ`G`(青点滅・黄・赤は`r`)。
    横断歩道が青の間、交わる車のリンクは`g`(譲る)。
- 写していないもの: 道路網(リンク・コネクタ)、Wiedemann 74の`bx`、加速度の速度による変化と車ごとのばらつき、歩行者の需要(歩行者入力)。
- 経路と希望速度決定は、`Town10HD.inpx`には無い。これらを足したinpxで作れることは、テスト(`test/test_sumo_inpx.py`の
  `test_town10hd_static_routes_follow_the_route`など。`Town10HD.inpx`に経路決定と希望速度決定を足したもので確かめる)で確かめた。
  Vissimで作ったinpxでは、まだ確かめていない。
- `vissim_to_rou.py`のオプション(既定値を変えると、`test_town10hd_rou_is_reproduced`は失敗する。変えたらテストも直す):

  | オプション | 説明 | 既定値 |
  | --- | --- | --- |
  | `--inpx` / `--net` | 入力のinpx / SUMOの網 | なし(省略不可) |
  | `--out` | 書き出す`.rou.xml` | なし(`--report`のときは不要) |
  | `--report` | 流入・車種・台数を表示する(書き出さない) | なし |
  | `--route-length` | 経路の長さ [m] | `2000` |
  | `--horizon` | 車を出す時間 [s] | `3600` |
  | `--seed` | 出発時刻と経路の乱数の種 | `1` |
  | `--prefix` | vType・車のIDの頭 | `vissim` |

- 作ったファイルの先頭のコメントに、inpx・網・ツールのsha256と、引数が書かれる(どのinpxから作ったかを確かめられる)。
- Vissim構成との関係:
  - Vissim構成のブランチ(`feature/vissim_windows_co-sim`)の`vissim/Town10HD.inpx`は、このブランチのものと同じ。
    Windows PCのVissimで直したinpxは、使う構成のブランチの`vissim/Town10HD.inpx`に置く。両方の構成で使うときは、両方のブランチにコミットする。
  - Vissim構成の信号の対応表(`signal_mapping_town10hd.json`)は、SUMO構成では使わない。
    信号灯の名前・信号グループを変えたときは、Vissim構成の起動手順書(`Vissim(win)-CARLA(0.9.16)-Autoware_co-sim_起動手順.md`)の2.4.2で作り直す。
  - `vissim/README.md`の2章(`build_vissim_town.py`)は、`Town10HD_road.inpx`から`Town10HD.inpx`を作り直す。**Vissimで直した設定は消える**ので、
    Vissimで直したinpxを使うときは実行しない。

### 3.8.2 止まったとき(2.2.4)

ツールは、写せない設定を見つけると`Error: <理由>`を出して、ファイルを書かずに終わる(終了コード1)。

| 出る理由(抜粋) | 原因 | 対処 |
| --- | --- | --- |
| `信号プログラムが 1 つでない` | 信号制御に信号プログラムが2つ以上ある | 使うプログラムを1つにする |
| `信号灯 <番号> の名前 ... が「CARLA <信号 ID>」でない` | 信号灯の名前がCARLAの信号IDの形でない | 名前を`CARLA <CARLAの信号ID>`にする |
| `CARLA の信号 <ID> の信号灯が別の信号グループにある` | 同じCARLAの信号の信号灯(2車線分)が、違う信号グループにある | 同じ信号グループにする |
| `... の青の組 ... が今のプログラムに無い` | 一緒に青になるリンクの組が、網のプログラムと違う(3.8.1) | 信号グループの組を元に戻すか、網の信号から作り直す |
| `CARLA の信号 ... が網の linkSignalID に無い` | 網にないCARLAの信号の信号灯がある | 信号灯の名前を確かめる |
| `リンク <番号>(<名前>)に合う SUMO の edge が 1 つでない` | 車両入力のリンクに当たるSUMOのedgeが見つからない(リンクの向き・位置で探す) | 車両入力を、OpenDRIVEから取り込んだリンクに置く |
| `流入 <番号> の時間帯が 1 つでない` / `が STOCHASTIC でない` | 車両入力に時間帯が2つ以上ある / 台数が確率的でない | 時間帯を1つ、台数をStochasticにする |
| `経路決定 <番号> がリンク <番号> にある。流入リンクにある経路決定だけを扱う` | 車両入力のリンク以外に経路決定がある(途中で経路を受け取る車は写せない) | 経路決定を車両入力のリンクに置く |
| `流入リンク <番号> に経路決定が 2 つ以上ある` | 1つの車両入力のリンクに経路決定が2つ以上ある | 1つにまとめる |
| `経路決定 <番号> が一部の車両クラスだけのもの` | 経路決定が「全車種」でない | 全車種にする |
| `経路決定 <番号> の経路 <番号> の割合の時間帯が 1 つでない` | 経路の割合が時間帯ごとに違う | 時間帯を1つにする |
| `流入 <番号>(リンク <番号>)の経路がどれも SUMO の網で着けない` | 経路のリンク列の近くを通るSUMOのedge列が無い | 経路のリンク列・行き先を確かめる |
| `歩行者リンク <番号>(...)に合う地図の横断歩道が 1 つでない` | 歩行者リンクの近くに地図の横断歩道が無い(または2つある) | 歩行者リンクを地図の横断歩道に合わせて置く。地図に横断歩道が入っているか確かめる(2.2.3) |
| `地図の横断歩道 <id> の渡る車道が 1 つの交差点に集まらない` | 横断歩道が交差点の近くに無い | 地図の横断歩道の位置を確かめる |
| `待ち場所 <番号> が地図の横断歩道 <id> の延長線上に無い` | 横断歩道の端の近くの待ち場所が、横断歩道の延長線上に無い | 待ち場所を横断歩道の延長線上に置く |
| `tlLogic <id> の横断歩道 ... に歩行者信号灯が無い` | 信号のある交差点の横断歩道に、歩行者信号灯が無い | 歩行者リンクに歩行者信号灯を置く |
| `tlLogic <id> の横断歩道 ... に別の信号グループの歩行者信号灯がある` | 1本の横断歩道の歩行者信号灯が、違う信号グループにある | 同じ信号グループにする |
| `希望速度決定 <番号> がコネクタ <番号> の上にある` | 希望速度決定がコネクタの上にある | リンクの上に置く |
| `リンク <番号> の希望速度決定が車線ごとに違う` | 同じリンクの車線ごとに、違う希望速度分布の決定がある | 全車線で同じ分布にする |
| `inpx に歩行者の流入が ... 本ある` | 歩行者の入力がある | 歩行者の入力を消す |
| `運転挙動の追従モデルが Wiedemann 74 でない` / `リンクの運転挙動が 1 つでない` | 追従モデルがWiedemann 99、またはリンクごとに運転挙動が違う | Wiedemann 74の運転挙動を1つにする |
| `... の vClass が決まっていない` | ツールが知らない車種・カテゴリがある | ツール(`vissim_to_rou.py`の`VCLASS`・`HGV_BY_NAME`)に対応を足す |
| `SUMO_HOME/tools が無い` | SUMOの環境変数が無い | `SUMO_HOME`を設定する |

## 3.9 SUMOの信号とCARLAの信号の対応

SUMO構成では、SUMOの信号とCARLAの信号の対応は、網(`sumo/net/Town10HD.net.xml`)の中に書かれている(対応表のファイルはない)。

| | SUMO構成 | Vissim構成 |
| --- | --- | --- |
| シミュレータの信号 → CARLAの信号 | 網の`tlLogic`の`<param key="linkSignalID:<linkIndex>" value="<CARLAの信号ID>"/>`(2.2.2) | `signal_mapping_town10hd.json`(Vissimの(信号制御, 信号グループ) → CARLAの信号ID) |
| CARLAの信号 → Autoware | 信号入りのLanelet2地図`lanelet2_map_with_tl.osm`(2.2.3。両方の構成で同じもの) | 同じ |

- どちらの構成も、Town10HDの同じ15個のCARLAの信号(943〜945・949〜954・957〜962)を指す。
  SUMOの信号5つ(交差点189・23・468・532・719)は、Vissimの信号制御5つ(1〜5)に1対1で当たる。

| SUMOの交差点 | CARLAの信号 | 形 |
| --- | --- | --- |
| 189 | 949〜952 | 十字路 |
| 23 | 943〜945 | T字路 |
| 468 | 953・954 | T字路(信号のない進入路が1本) |
| 532 | 957〜959 | T字路 |
| 719 | 960〜962 | T字路 |

- `tools/sumo_net/add_carla_tls.py`は、CARLAリポジトリ(`~/CARLA`の`main`、`Co-Simulation/Sumo/util/add_carla_tls.py`、c0cefa8)から写したもの(2026-11-25)。
  変えたのは、OpenDRIVEから網を作る`netconvert_carla.py`を`--netconvert-carla`で渡すようにしたことだけ
  (元は同じフォルダのものを使う。ここでは使うCARLA(0.9.16_jp)に入っているものを使う)。
- `add_carla_tls.py`が作る信号:
  1. `netconvert_carla.py`がOpenDRIVEから作る網で、どのリンク(接続)がどのCARLAの信号(landmark)に当たるかを読む。
  2. 入力の網の`priority`の交差点のうち、そのリンクがある交差点に信号を置く。進入路を向きで組にし(向かい合う2本で1組)、
     組ごとに青42秒→黄3秒を順に回す(T字路は主道路が先)。2本の組の左折・Uターンは`g`(対向の直進に譲る)。
  3. 1つのCARLAの信号のリンクは、いつも同じ色になる(CARLAで赤に置き換えられる信号はない)。
- xodrにある信号955・956は名前がなく、CARLAに信号機がないので対象外。ブリッジの起動時に`WARNING: Landmark 955 is not linked to any traffic light`が出ることがあるが問題ない。
- `Town10HD_vissim.sumocfg`(3.8)は、網の信号の時間の代わりに、inpxから作った`tlLogic`のプログラム`vissim`を使う。`linkSignalID`は網のものを写す。
- 2026-11-25に、上の手順(2.2.2)で作り直した網が、コミットしている網と、先頭のコメント(作った日時と一時ファイルのパス)以外で同じことを確かめた。
