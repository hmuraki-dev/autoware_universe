# SUMO-CARLA-Autoware co-sim 起動手順

# 0. 前提条件
Autoware環境を構築済みであること。
環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。

SUMOのシナリオ(`.sumocfg`・net・ルート)は、このリポジトリの`simulator/autoware_carla_interface/sumo/examples/`のものを使う(1章)。
CARLAリポジトリ(`~/CARLA`)の`Co-Simulation/Sumo/examples/`は、SUMOのnet・ルートを作る作業場所(作ったものを`sumo/examples/`へ写す)。


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
                    └── sumo/examples/  ← SUMOのシナリオ(.sumocfg・net/・rou/)
```

`autoware_carla_interface` は独立したリポジトリではなく、`autoware_universe` リポジトリ配下のパッケージである。

`sumo/examples/`には、Town01・Town05・Town10HDの`.sumocfg`・net・ルートと、それらが読む`carlavtypes.rou.xml`・`vissim_fleet_vtypes.rou.xml`・`viewsettings.xml`がある。
2026-11-17に、CARLAリポジトリ(`~/CARLA`)の`main`(`4e22c5e`)の`Co-Simulation/Sumo/examples/`から写した。
net・ルートを作り直すツール(`add_carla_tls.py`など)と、その手順書(`Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`)は`~/CARLA`にある。
`~/CARLA`で作り直したときは、`sumo/examples/`の同じ相対位置へ写す。

# 2. 起動・終了の手順

2.1から順に実行する。2.2.2・2.2.3は、**対象を変えたときだけ**実行する(毎回の起動では不要)。

## 2.1 【ターミナル1】CARLAサーバー起動

```bash
cd ~/CARLA
./CarlaUE4.sh
```
コマンドを実行すると、CARLAサーバーが起動し、CARLA(Unreal Engine)の画面が表示される。


## 2.2 【ターミナル2】マップのロード → 信号入りの地図の作成 → 車両のブループリント表の生成

### 2.2.1 マップのロード

```bash
cd ~/CARLA/PythonAPI/util
python3 config.py --map Town01
```
コマンドを実行すると、CARLAで読み込まれているマップが Town01 に切り替わる。CARLA画面が Town01 の地図に更新されたことを確認する。

Town05・Town10HDを使うときは、`--map`だけを変える。2.4.1の起動コマンドも、同じTownのものを使う。

```bash
# Town05
python3 config.py --map Town05
```

```bash
# Town10HD
python3 config.py --map Town10HD
```

### 2.2.2 信号入りのLanelet2地図の作成

CARLAの信号状態をAutowareへ直接通知する(2.4.1の`traffic_light.publish:=true`)ときに使う、信号入りの地図を作る。仕組みと注意点は3.4を参照。

**注意**: **そのTownで最初に1回だけ実行する**。CARLAの信号・元の地図・停止線をずらす量を変えたときは、作り直す。
2.2.1で、作る地図と同じTownを読み込んでおくこと。

**Town01**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/Town01/lanelet2_map.osm --stop-line-forward-offset 6   # ← 生成物
```

> 📄 **生成物:** 信号入りのLanelet2地図(Town01、2.4.1の`lanelet2_map_file`)\
> `~/autoware_map/Town01/lanelet2_map_with_tl.osm`

- すでにある場合は`--force`で上書きする。元の地図(`lanelet2_map.osm`)は変更しない。
- `WARNING: ... not added`が出たときは終了コードが2になる(3.4.1)。

SUMOのnetと作った地図が、同じCARLAの信号を指しているかを確かめる。

```bash
python3 tools/check_sumo_lanelet2_tl_ids.py sumo/examples/net/Town01.net.xml \
    ~/autoware_map/Town01/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town01/carla_tl.json --report-mixed
```

- 最後に`OK`(終了コード0)が出ればよい。`--carla-json`は、`carla_tl_to_lanelet2.py`の`--dump-json`で保存したCARLAの信号(省略できる)。

**Town05**(地図・点群の置き方は3.4.1の「Town05」)

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
# 配布元の地図は境界線に同じ位置の点が続くので、先に直す(元の地図は .orig に残す)。
cp ~/autoware_map/Town05/lanelet2_map.osm ~/autoware_map/Town05/lanelet2_map.osm.orig
python3 tools/fix_lanelet2_duplicate_points.py ~/autoware_map/Town05/lanelet2_map.osm.orig \
    ~/autoware_map/Town05/lanelet2_map.osm
# 2回目からは--dump-jsonの代わりに--from-json。
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/Town05/lanelet2_map.osm \
    --dump-json ~/autoware_map/Town05/carla_tl.json --stop-line-forward-offset 0 --group-id-offset 100000   # ← 生成物
python3 tools/check_sumo_lanelet2_tl_ids.py sumo/examples/net/Town05.net.xml \
    ~/autoware_map/Town05/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town05/carla_tl.json \
    --report-mixed --group-id-offset 100000
```

> 📄 **生成物:** 信号入りのLanelet2地図(Town05)と、CARLAの信号の記録\
> `~/autoware_map/Town05/lanelet2_map_with_tl.osm`、`~/autoware_map/Town05/carla_tl.json`

**Town10HD**(地図・点群は3.4.1の「Town10HD」)

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
cp ~/autoware_map/Town10HD/lanelet2_map.osm ~/autoware_map/Town10HD/lanelet2_map.osm.orig
python3 tools/fix_lanelet2_duplicate_points.py ~/autoware_map/Town10HD/lanelet2_map.osm.orig \
    ~/autoware_map/Town10HD/lanelet2_map.osm
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/Town10HD/lanelet2_map.osm \
    --dump-json ~/autoware_map/Town10HD/carla_tl.json --stop-line-forward-offset 0 --group-id-offset 100000   # ← 生成物
python3 tools/check_sumo_lanelet2_tl_ids.py sumo/examples/net/Town10HD.net.xml \
    ~/autoware_map/Town10HD/lanelet2_map_with_tl.osm --carla-json ~/autoware_map/Town10HD/carla_tl.json \
    --report-mixed --group-id-offset 100000
```

> 📄 **生成物:** 信号入りのLanelet2地図(Town10HD)と、CARLAの信号の記録\
> `~/autoware_map/Town10HD/lanelet2_map_with_tl.osm`、`~/autoware_map/Town10HD/carla_tl.json`

### 2.2.3 SUMOの車両のCARLAブループリント表の生成

SUMOの車両1台ごとに、どのCARLAのブループリントを使うかを決める表(`sumo_integration/data/carla_variants_sumo_town01.yaml`)を作る。
仕組みと今のTown01の割り当ては3.5を参照。表があるのはTown01だけで、Town05・Town10HDは表を使わない(今までの選び方)。

**注意**:
- **次のどれかを変えたときだけ実行すればよく**、毎回の起動時に実行する必要はない。今の表はリポジトリに入っている。

  | 変えたもの | 実行する手順 |
  | --- | --- |
  | SUMOの車両の設定(`Town01.sumocfg`の`route-files`の各ファイル。vType・車両グループ・flow) | (2)(3) |
  | Vissimの車両の設定に合わせて、SUMOのvTypeを作り直したとき(CARLAリポジトリの`Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`の「Town01の車両の構成」。作ったものを`sumo/examples/`へ写す) | (2)(3) |
  | 割り当ての規則(`src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml`) | (2)(3) |
  | CARLAの版(ブループリントの形が変わりうる) | (1)(2)(3) |

- 変えたのに作り直さないと、co-simの起動時に`Error: CARLA blueprint assignment: variant_source_changed: ...`で止まる(3.5.3)。
  ブリッジは起動時に、渡された`.sumocfg`の`route-files`の各ファイルのsha256を、表に記録した値と照合する。
- 作り直した表(と、(1)を実行したときは計測結果)は、autoware_universeにコミットする。`--symlink-install`でビルドしていれば、ビルドし直さなくても使われる。

**(1) CARLAのブループリントの外形の計測**(CARLAの版を変えたときだけ)

CARLAだけを起動した状態で行う(ブリッジ・Autowareは起動しない)。各ブループリントを地下に1台ずつ置き、LiDARで外形を測ってから消す。41ブループリントで1分ほど。
Vissim構成と同じファイルを使う(どちらかで測れば、もう一方で測り直す必要はない)。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/measure_blueprint_mesh.py \
    --out src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla<版>.json   # ← 生成物
```

> 📄 **生成物:** CARLAブループリントの外形(LiDARで測った長さ・幅)一覧\
> `src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla<版>.json`

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

> 📄 **生成物:** CARLAブループリントの外形(LiDARで測った長さ・幅)一覧\
> `src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json`

- 0.9.15の計測結果は、すでにリポジトリに入っている(2026-11-16に測ったもの)。出力先のファイルがあるとツールはエラーで止まるので、0.9.15で測り直すときは、先に古いファイルを消すか、日付を付けた別の名前(例: `mesh_measurements_carla0.9.15_20261201.json`)にする。別の名前にしたときは、(2)の`--geometry`もその名前にする。
- ブリッジが照合するのは、ファイル名ではなく、ファイルの中に記録された版(`carla_server_version`。ツールが接続先のCARLAから書き込む)。

| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `--out` | 計測結果(JSON)の出力先。すでにあるとエラー | 省略不可 | なし |
| `--host` / `--port` | CARLAサーバー | 省略可 | `localhost` / `2000` |
| `--only` | 測るブループリントを限る(例: `--only vehicle.lincoln.mkz_2017`) | 省略可 | すべて |
| `--exclude` | 測らないブループリント(fnmatchの形、複数可) | 省略可 | なし |

- CARLAが同期モードで誰もtickしていないときは、ツールが自分でtickする。ブリッジが動いているときは実行しない。

**(2) 表の生成**

co-simで使う`.sumocfg`(2.4.1の`sumo_cfg_file`)をそのまま入力にする。CARLAサーバーは要らない。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_fleet/build_carla_variants.py \
    --sumocfg "$PWD"/sumo/examples/Town01.sumocfg \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json \
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml \
    --out src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml   # ← 生成物
```

> 📄 **生成物:** SUMO車両-ブループリント割り当て一覧(Town01)\
> `src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml`

- 表には、`.sumocfg`のパスと、`route-files`の各ファイルのsha256が記録される。ブリッジは起動時にこのsha256を照合するので、どの版の`.rou.xml`から作った表かは、これで決まる。
- `--sumocfg`は`"$PWD"/sumo/examples/...`の絶対パスで渡す(表に記録されるパス。テスト`test_sumo_fleet.py`は、同じ絶対パスで表を作り直して比べる)。
- 出力先のファイル名は`carla_variants_sumo_<.sumocfgの名前を小文字>.yaml`にする。ブリッジは既定(`sumo_carla_variants:=auto`)で、この名前の表を探す(3.5)。
- `.sumocfg`・規則・外形のどれも変えていなければ、作り直しても割り当ては変わらない(`generated_at`の記録だけが変わる)。

| オプション | 説明 | 省略可/不可 | デフォルト値 |
| --- | --- | --- | --- |
| `--sumocfg` | SUMOの設定ファイル。`route-files`の全ファイルを読む | 省略不可(Vissimの`--inpx`とどちらか1つ) | なし |
| `--geometry` | (1)の計測結果 | 省略不可 | なし |
| `--rules` | 割り当ての規則 | 省略不可 | なし |
| `--out` | 表の出力先 | 省略不可 | なし |
| `--name` / `--generated-at` | 表の名前 / 生成日時(作り直しの差分を見るときに固定する) | 省略可 | `--out`のファイル名 / 今 |

- 割り当てられないモデルがあると、`[build_carla_variants] 止めた: variant_model_empty: ...`と出て、ファイルを書かない。規則を見直すか、`.rou.xml`でそのvTypeの割合を0にする。
- SUMOのvTypeは寸法(長さ・幅)でまとめて1つのモデルにする。歩行者のvTypeは表に入らない。割合0のvType(`carlavtypes.rou.xml`の41種など)は`skipped`に入る。

**(3) 確認**

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
git diff --stat src/autoware_carla_interface/sumo_integration/data/carla_variants_sumo_town01.yaml
python3 -m pytest test/test_carla_fleet.py test/test_build_carla_variants.py test/test_sumo_fleet.py test/test_sumo_fleet_plan.py
```

- 表の`assignment`(モデルごとのブループリント・dL・dW)と`expected_bodies_per_100_vehicles`を見て、意図した割り当てか確かめる。
- テストは、今のTown01の割り当て(Vissim構成と同じ)を期待値にしている。割り当てが変わったら、テストの期待値(`test_sumo_fleet.py`の`test_town01_sumo_variants_match_the_vissim_assignment`、`test_sumo_fleet_plan.py`の`test_same_assignment_as_vissim`など)も直す。


## 2.3 【ターミナル3】ビルド

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
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_sumo:=true \
  sumo_gui:=true \
  sumo_cfg_file:=/home/divp/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface/sumo/examples/Town01.sumocfg \
  tls_manager:=sumo \
  traffic_light.publish:=true \
  sync_vehicle_lights:=true \
  sync_vehicle_color:=true \
  sumo_warmup_time:=600 \
  spawn_point:="267.65,133.49,0.30,0.0,0.0,0.0" \
  spectator_follow:=true \
  2>&1 | tee -i /tmp/autoware_carla.log
```

- 起動ログに、SUMOの車両のブループリントをどう選ぶかが1行出る。Town01では`CARLA blueprints of SUMO vehicles: .../carla_variants_sumo_town01.yaml (14 models, sumo_carla_variants=auto)`(3.5)。

Town05・Town10HDの起動コマンド(Town01との違いは`map_path`・`sumo_cfg_file`・`sumo_warmup_time`・`spawn_point`の4行):

```bash
# Town05(信号2408の手前にスポーン。3.4.1「Town05」)
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town05 \
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_sumo:=true \
  sumo_gui:=true \
  sumo_cfg_file:=/home/divp/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface/sumo/examples/Town05.sumocfg \
  tls_manager:=sumo \
  traffic_light.publish:=true \
  sync_vehicle_lights:=true \
  sync_vehicle_color:=true \
  sumo_warmup_time:=60 \
  spawn_point:="24.71,-51.03,0.30,0.0,0.0,91.5" \
  spectator_follow:=true \
  2>&1 | tee -i /tmp/autoware_carla.log
```

```bash
# Town10HD(信号954の手前にスポーン。3.4.1「Town10HD」)
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town10HD \
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_sumo:=true \
  sumo_gui:=true \
  sumo_cfg_file:=/home/divp/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface/sumo/examples/Town10HD.sumocfg \
  tls_manager:=sumo \
  traffic_light.publish:=true \
  sync_vehicle_lights:=true \
  sync_vehicle_color:=true \
  sumo_warmup_time:=60 \
  spawn_point:="30.02,133.95,0.30,0.0,0.0,-179.7" \
  spectator_follow:=true \
  2>&1 | tee -i /tmp/autoware_carla.log
```

- `source install/setup.bash`と`export ROS_DOMAIN_ID=33`はTown01と同じく先に実行しておく。
- `map_path`のフォルダ名(Town05・Town10HD)が、2.2.1でCARLAに読み込んだマップと同じであること。
- `sumo_warmup_time`は`60`にする。Town05・Town10HDのSUMOのルートは0〜199秒に計200台だけなので、
  Town01と同じ`600`にすると、EGOのスポーン時には車がいなくなっている。
- `lanelet2_map_with_tl.osm`は、2.2.2の「Town05」「Town10HD」の手順で作っておく(重複点を直した地図から作ること)。
- Town05・Town10HDには車両のブループリント表がないので、起動ログに`CARLA blueprints of SUMO vehicles: legacy (vType id, else vtypes.json) (sumo_carla_variants=auto: no carla_variants_sumo_town05.yaml ...)`が出て、今までの選び方で動く(3.5)。
- Town05・Town10HDとも、この起動コマンドで実機確認済み(2026-10-21、3.4.1の「Town05」「Town10HD」)。

| オプション | 説明 | 設定例 | デフォルト値(未指定時) |
|-----------|------|--------|--------------------------|
| `simulator_type` | 使用するシミュレータ | `carla` | なし(指定必須) |
| `map_path` | Lanelet2マップ | `$HOME/autoware_map/Town01` | なし(指定必須) |
| `vehicle_model` | 車両モデル | `sample_vehicle` | Launch既定値 |
| `sensor_model` | センサキット | `carla_sensor_kit` | Launch既定値 |
| `use_sumo` | SUMO連携 | `true` | `false` |
| `sumo_cfg_file` | SUMO設定 | `...Town01.sumocfg` | なし |
| `sumo_carla_variants` | SUMOの車両に割り当てるCARLAのブループリントの表。`auto`は`sumo_integration/data/carla_variants_sumo_<.sumocfgの名前を小文字>.yaml`があれば使い、なければ今までの選び方。`legacy_vtypes`は今までの選び方(vTypeのIDのブループリント、なければ`vtypes.json`から乱数)。ファイル名を渡すと、その表を使う(相対パスは`sumo_integration/data/`から)。3.5参照 | `auto` | `auto` |
| `sumo_gui` | GUI表示 | `true` | `false(ヘッドレス)` |
| `sumo_host` | TraCIホスト | `localhost` | `localhost` |
| `sumo_port` | TraCIポート | `8813` | `8813` |
| `sumo_client_order` | クライアント順 | `1` | `1` |
| `tls_manager` | 信号管理 | `sumo` | `none` |
| `sync_vehicle_lights` | 灯火同期 | `true` | `false` |
| `sync_vehicle_color` | 車体色同期 | `true` | `false` |
| `spectator_follow` | EGO車両(role_name=`ego_vehicle_role_name`)にCARLAスペクテーターを自動追従させる | `true` | `false` |
| `spawn_point` | EGOのスポーン位置(CARLA座標、`x,y,z,roll,pitch,yaw`) | `"267.65,133.49,0.30,0.0,0.0,0.0"` | `None`(ランダム) |
| `vehicle_type` | EGOのCARLAブループリント | `vehicle.toyota.prius` | `vehicle.toyota.prius` |
| `sumo_warmup_time` | SUMOだけを先に進める秒数(整数)。0で無効。3.1参照 | `600` | `0`(無効) |
| `ego_spawn_front_margin` | EGOスポーン時に必要な前方車とのすき間 [m]。3.1参照 | `20.0` | `20.0` |
| `ego_spawn_rear_margin` | EGOスポーン時に必要な後方車とのすき間 [m]。3.1参照 | `20.0` | `20.0` |
| `ego_spawn_wait_timeout` | ウォームアップ後に空きを待つ上限秒数(整数)。3.1参照 | `60` | `60` |
| `traffic_light.publish` | CARLAの信号状態をAutowareへ直接通知する(カメラの信号認識は起動しない)。3.4参照 | `true` | `false` |
| `lanelet2_map_file` | Autowareが読み込むLanelet2地図のファイル名(`map_path`内)。信号を使うときは2.2.2で作った地図 | `lanelet2_map_with_tl.osm` | `lanelet2_map.osm` |
| `traffic_light.force_green` | 起動時に全信号を緑にして固定する。3.4参照 | `true` | `false` |

- `sumo_gui:=true`のsumo-guiは、起動後に自動でシミュレーションを開始する(`--start`)。Runボタンを押す必要はない。
  また、co-simの終了時にはダイアログを出さずに自動で閉じる(`--quit-on-end`)。
- `spectator_follow:=true`はCARLAスペクテーター(自由視点カメラ)をEGO車両に自動追従させる。
  カメラの距離・高さ・角度(`--distance`/`--height`/`--pitch`/`--rate`)はlaunch引数として
  公開されていないため、細かく調整したい場合はこの引数は使わず`ros2 run
  autoware_carla_interface spectator_follow --distance ... --height ...`のように別ターミナルで
  手動起動すること。
- `lanelet2_map_file`・`traffic_light.publish`の2行は、CARLAの信号状態をAutowareへ直接通知する設定(3.4参照)。
  2.2.2の信号入りの地図(`lanelet2_map_with_tl.osm`)を先に作っておくこと。使わない場合はこの2行を省く(カメラによる信号認識になる)。
- ウォームアップを使わない場合は、`sumo_warmup_time`と`spawn_point`の行を省く(EGOは起動直後にランダムな位置へスポーンされる)。
  `spawn_point`だけを指定すれば、ウォームアップなしで固定位置にスポーンする。スポーン地点の選び方は3.1.2。
- `2>&1 | tee -i /tmp/autoware_carla.log`はログ保存用(3.1.3のgrepで使う)。不要なら省いてよい。
  `-i`を付けないと、Ctrl+Cで`tee`も終了し、それ以降(終了処理)のログが残らない。

### 2.4.2 目的地の設定と自動運転の開始

- ウォームアップを使うときは、ログに`[EGO SPAWN]`が出て、RVizにEGOが表示されてから操作する(それまでCARLAにEGOはいない)。
- RVizで目的地(2D Goal Pose)を置き、Autoを押す。目的地の置き方はTownごとに3.4.1を参照
  (Town05は同じ道路を2回通る場所に置くと経路ができない。Town10HDはスポーン地点の先の交差点を抜けた道路5の上、map座標で(-104, -99)付近)。
- **EGOがスポーンしてから300秒以上、走り出さずに止まっていると**、SUMOがSUMOの中のEGOを消し、ブリッジが入れ直す(ログに`[sync] sumo removed CARLA-origin vehicle carla0 ...; adopting it into sumo again`)。co-simは続く(3.5.4)。

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

**注意(2.4.1のターミナルでCtrl+Cを押した場合)**: これでも止まるが、`e2e_simulator.launch.xml`全体にSIGINTが送られ、launchは5秒後にSIGTERM、さらに10秒後にSIGKILLを送る。
`autoware_carla_interface`の片付けが間に合わないと、後片付けのログ(と要約)が途中で切れることがある(Vissim構成で確認。SUMO構成では、上の`pkill`の止め方で確かめている)。
Ctrl+Cで止めたときに、最後に`[ERROR] [launch]: Caught exception in launch ...: Cannot shutdown a ROS adapter that is not running`が1行出ることがあるが、launch側のメッセージで実害はない。

### 2.5.2 自動で止まるとき

次のいずれかで`autoware_carla_interface`ノードが終了し(終了コード1)、それに伴って`e2e_simulator.launch.xml`で起動した**全ノードが停止する**(`on_exit="shutdown"`による)。

| きっかけ | ログ | 対処 |
| --- | --- | --- |
| 起動パラメータの組み合わせが正しくない | `Error: invalid parameters: ...` | 3.1(ウォームアップの条件)などを見て直す |
| ウォームアップ後の試験開始失敗(空きが`ego_spawn_wait_timeout`秒以内にできなかった、EGOのスポーンに失敗した) | `Error: test start failed: ...` | 3.1.4 |
| ブループリントの表が使えない(起動時)。表がない、作ったときと`.rou.xml`・規則・実メッシュが違う、など | `Error: CARLA blueprint assignment: <code>: {...}` | **SUMOを起動する前に止まる**。3.5.3 |
| 走行中に、表にないvTypeの車、または寸法の違う車が来た | `Error: CARLA blueprint assignment: npc_model_unknown: {...}` など | 3.5.3 |

- ウォームアップ中・空き待ち中に止めると、EGOをスポーンせずに後始末して終了する。
- `sumo_gui:=true`のsumo-guiは、終了時にダイアログを出さずに自動で閉じる(`--quit-on-end`)。

# 3. 参考

2章の手順で使う機能の説明と、今の設定の記録。起動のたびに読む必要はない。

## 3.1 ウォームアップ・EGO安全スポーン

SUMOだけを先に所定時間進めて交通流を作ってから、EGOのスポーン位置の前後が空いたタイミングでEGOをスポーンする機能。
`sumo_warmup_time`に1以上を指定すると有効になり、指定しない(既定の0)場合は従来と同じ動作になる。
設計は`docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md`を参照。

### 3.1.1 動作と条件

- 起動の流れ: Autoware起動 → SUMOだけを`sumo_warmup_time`秒進める(CARLAは止まったまま) →
  その間にSUMOに現れた車両をCARLAへまとめてスポーン → EGOのスポーン位置の前後の空きを判定 →
  空いていなければSUMO・CARLAを1ステップずつ進めて再判定 → 空いたらEGOをスポーン → 通常のco-sim。
- 空きの判定: 同じ車線の前方車・後方車とのすき間(バンパー間の距離)が、それぞれ`ego_spawn_front_margin`・
  `ego_spawn_rear_margin`以上で、かつEGOの予定位置に重なる車両・歩行者がいなければスポーンする。
  CARLAで複数の体を重ねて表す大型車(3.5)は、全体の外形(SUMOの前端から最後の体の後端まで)で判定する。
- EGOがスポーンされるまでは、CARLA上にEGOはおらず、RVizにもEGOは表示されない。
- ROSの時刻(`/clock`)はEGOのスポーン時点から0付近で始まる。ウォームアップ・空き待ちの時間は含まれない。

`sumo_warmup_time`に1以上を指定したときは、次を満たさないと起動時に
`Error: invalid parameters: ...`の1行を出して終了する。

- `use_sumo:=true`であること
- `spawn_point`を`x,y,z,roll,pitch,yaw`の6つの値で指定すること(ランダムスポーンでは判定する位置が決まらないため)
- `tls_manager`が`sumo`または`none`であること(`carla`ではウォームアップ中にSUMOの信号が消えたままになるため使えない)
- 必要距離が0以上、`ego_spawn_wait_timeout`が1以上であること

`ego_spawn_front_margin`・`ego_spawn_rear_margin`・`ego_spawn_wait_timeout`は、Vissim-CARLA-Autoware co-simの
ウォームアップ(`vissim_warmup_time`)と共通のパラメータである。
`autoware_launch`の`e2e_simulator.launch.xml`には、`sumo_warmup_time`を含む4つの引数の宣言・受け渡しを追加している
(`ros2 launch autoware_launch e2e_simulator.launch.xml --show-args`で確認できる)。
なお、宣言がなくてもコマンドラインで指定した値はノードまで届く。届いているかは3.1.3の`[EGO SPAWN CHECK] start:`の行の値で確認できる。

### 3.1.2 スポーン地点の選び方

空きの判定は「スポーン地点が直線区間にある」ことを前提にしている(曲線では前後車の判定がずれる)。
また、SUMO車両が通る車線でないと、空き待ちの確認にならない。

1. 直線区間のスポーン地点の候補を一覧にする(CARLAサーバーを起動し、Town01を読み込んでおく)。
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
   road番号が同じでも、対向車線(SUMO車両が通らない)のことがあるので、初回は`spectator_follow:=true`で
   スポーン後にSUMO車両と同じ向きに並んでいるかを目で確認する。
   SUMO車両が出発する道路(Town01では`-19.0.00`・`5.0.00`・`-9.0.00`・`19.0.00`・`2.0.00`・`10.0.00`・`-18.0.00`・`-8.0.00`・`-1.0.00`・`24.0.00`)の
   始点付近は、車両が突然現れるので避ける。

Town01(`rou/Town01.rou.xml`、2026-10-19以降: 入力1・2が各300台/h、多様な経路の入力3〜10が各40台/h、計920台/h)で確認済みの地点:

| 地点 | 用途 | SUMOの車線 | 交通量 | `spawn_point` |
|---|---|---|---|---|
| #37 | 交通量が多い(空き待ちが起きる) | `6.0.00` | 309台/h | `199.95,326.97,0.30,0.0,0.0,180.0` |
| #36 | SUMO車両が来ない(すぐスポーン) | `-6.0.00`(#37の対向車線) | 0台/h | `199.95,330.46,0.30,0.0,0.0,-0.0` |
| #144 | 別の道路で交通量が多い | `1.0.00` | 282台/h | `256.55,2.02,0.30,0.0,0.0,-0.0` |
| #249 | 交通量が少ない | `-4.0.00` | 101台/h | `220.14,133.24,0.30,0.0,0.0,-0.0` |
| — | 信号372の手前(信号状態の直接通知の確認用、3.4参照) | `-19.0.00` | 349台/h | `334.74,263.00,0.30,0.0,0.0,90.0` |

- `sumo_warmup_time:=600`・既定の必要距離での実績: #37は空き待ち13.75秒(シミュレーション時間)でスポーン、#36は待ちなし。
- 上の交通量と空き待ちの実績は、SUMOの信号が2交差点(CARLAの信号360〜362・372〜374)だけだったときの値。
  2026-10-16に全12交差点(CARLAの信号36個)へSUMOの信号を設置した後は信号待ちの渋滞が起きるため、値が変わる。
- 交通量の列は2026-10-19の`rou/Town01.rou.xml`の需要(flowの台数×経路の確率)。SUMO単体で1時間回した通過台数もほぼ同じ
  (`6.0.00`: 304台、`1.0.00`: 273台、`-4.0.00`: 99台)。変更前(入力2か所×600台/h、行き先はすべて`-18.0.00`)は
  1時間後に414台が出発待ちになるほど詰まっていたが、変更後は出発待ち10台・テレポート0回。
  #37は交通量が約半分になったので、空き待ちが起きるかは未確認。`-6.0.00`(#36)は変更後もSUMO車両が通らない。
- 2026-11-16に、Town01の車両グループの中身をVissimのモデルにした(3.5.2。大型車が長くなった)。そのあとの実績(`sumo_warmup_time:=600`、既定の必要距離):
  2.4.1の地点(`267.65,133.49`)は待ちなし、信号372の手前は空き待ち3.20秒(ウォームアップ820秒では1.70秒)。
- `spawn_point`のzにはCARLAの道路の高さ(Town01ではほぼ0〜0.3)を入れる。コード側で+2 mしてスポーンする。

### 3.1.3 ログの見方

```bash
grep -nE "SUMO WARMUP|EGO SPAWN|EGO spawn gate" /tmp/autoware_carla.log
```

| ログ | 意味 |
|---|---|
| `EGO spawn gate: vehicle.toyota.prius footprint 4.51 x 2.01 m (...), lane width 4.00 m` | 判定に使うEGOの寸法とその出所、車線幅 |
| `[SUMO WARMUP] start: warmup_time=600 s (12000 steps)` | ウォームアップ開始 |
| `[SUMO WARMUP] t=10.0 s vehicles=1 persons=0 (wall 0.3 s)` | 10秒(シミュレーション時間)ごとの進捗。`wall`は実時間 |
| `[SUMO WARMUP] completed: t=600.0 s vehicles=40 ... (wall 19.3 s)` | ウォームアップ終了時の車両数 |
| `[SUMO WARMUP] caught up: carla_spawned=40 sumo_only=0 ...` | CARLAへまとめてスポーンした台数。`sumo_only`はCARLAに対応する車種がなくSUMOにだけいる台数 |
| `[EGO SPAWN CHECK] start: spawn_point=(...) ego=... lane_width=... front_margin=... rear_margin=... wait_timeout=...` | 空き判定の条件(launch引数が届いているかの確認にも使う) |
| `[EGO SPAWN CHECK] t=600.05 front=sumo:in1_A.52/carla:245 clearance=13.0 m rear=... clearance=-1.2 m overlap=... result=WAIT` | 判定結果。前方車・後方車とのすき間、EGOの位置に重なっている車両、`WAIT`/`SAFE`。結果や前後の車両が変わったときと、それ以外は1秒ごとに出る。重ね置きの大型車は`sumo:<id>/carla:<id>+<2体目以降の数>` |
| `[EGO SPAWN] t=613.80 s waited=13.75 s spawn_point=(...) vehicles=42` | EGOをスポーンした時刻と空き待ちの時間 |

記録しておく項目(再現性のため): `.sumocfg`とルートファイル(SUMOの乱数シードを含む)、`sumo_warmup_time`・必要距離・
待ち時間上限、`spawn_point`、上の`[SUMO WARMUP] completed`・`[EGO SPAWN CHECK]`(最後のSAFE)・`[EGO SPAWN]`の行。
同じ`.sumocfg`・同じパラメータなら、SUMOの車両の動きは同じになる(開発用PCとLinux機で一致を確認済み)。

### 3.1.4 試験開始失敗

- `ego_spawn_wait_timeout`秒待っても空かない場合や、空いた後にEGOのスポーンに失敗した場合は、
  `Error: test start failed: ...`(最後の判定結果を含む)を出して後始末し、`autoware_carla_interface`が終了コード1で終了する(2.5.2)。
  スポーン地点を変える、`sumo_warmup_time`を変える、`ego_spawn_wait_timeout`を延ばす、などで対処する。

## 3.2 処理時間計測

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
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

## 3.3 歩行者(Pedestrian)同期について

上記のco-sim起動(2.4)には、SUMO側の歩行者(`traci.person`)をCARLA側の`walker.pedestrian.*`
アクターとして反映する歩行者同期が組み込まれている。

- **常時有効(CLI引数・launch引数は無い)**: 車両同期(`sync_vehicle_lights`等)と異なり、
  歩行者同期のON/OFFを切り替える起動オプションは存在しない。SUMO設定(`sumo_cfg_file`)側に
  歩行者(`personFlow`/`person`)のルート・ネットワークが定義されていれば、追加設定なしに
  自動的に同期される。
- **sumo→carla の一方向のみ**: CARLA側でspawnした歩行者をSUMOへ送り返す機能(carla→sumo)は
  実装していない。あくまでSUMOが管理する歩行者をCARLA上に可視化・追従させるための機能である。
- **Z座標補正**: `carla.Walker`アクターのtransform原点はbounding boxの垂直中心にあり、
  SUMOが返す座標は地面(足元)基準のため、そのまま反映すると歩行者が地面に埋まって見える。
  この差分を吸収するため、SUMOの`VAR_HEIGHT`の半分(`sumo_person.extent.z`)をZ座標に
  加算する補正を行っている(`BridgeHelper.get_carla_pedestrian_transform()`)。
- **ログについて**: 歩行者のspawn/update/destroyは`logging.debug()`で出力しているが、本パッケージは
  Pythonの`logging`モジュールに対して`basicConfig`等でレベル設定を行っていないため、
  デフォルト状態では表示されない(ターミナルにはWARNING以上、例えば未対応vclassのため
  blueprintが見つからなかった場合の警告のみが表示される)。spawn/update/destroyの詳細を
  確認したい場合は、`autoware_carla_interface`起動前に`python3 -c "import logging;
  logging.basicConfig(level=logging.DEBUG)"`相当の設定を追加する、または該当箇所に
  一時的なデバッグ出力を追加すること。
- 歩行者用のCARLA walkerブレンプリントは`sumo_integration/data/vtypes.json`の
  `carla_blueprints`に`walker.pedestrian.0001`〜`0051`(`vClass: "pedestrian"`)として
  登録済みであり、追加設定は不要。

## 3.4 CARLAの信号状態をAutowareへ直接通知する

CARLAが管理している信号の状態(赤・黄・青)を、カメラ画像による信号認識を使わずに、
ブリッジ(`autoware_carla_interface`)から`/perception/traffic_light_recognition/traffic_signals`へ
毎tick publishする機能。設計と確認結果は`docs/CARLA_Autoware_信号状態publish_実装計画_v1.0.md`を参照。
`tls_manager:=sumo`のときは、SUMOが決めた信号状態がCARLA経由でそのままAutowareに届く。
Town01のSUMOのnet(`sumo/examples/net/Town01.net.xml`)には、CARLAの信号36個すべてに対応する信号がある
(設置方法と信号パターンは`~/CARLA/Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`の「SUMOの信号(Town01)」を参照)。
Town05のnetにも、CARLAの信号54個すべてに対応する信号がある(同「SUMOの信号(Town05)」、地図は3.4.1の「Town05」)。
Town10HDは、netとルートを作り、CARLAの信号15個すべてに対応する信号を置いた(同「SUMOの信号(Town10HD)」、地図は3.4.1の「Town10HD」)。

### 3.4.1 信号入りのLanelet2地図

標準のCARLA用Lanelet2地図(`~/autoware_map/Town01/lanelet2_map.osm`)には信号の情報がないため、
そのままではAutowareが信号で止まらない。CARLAから信号を読み取って地図に追加するツールで、
信号入りの地図を作る(コマンドは2.2.2)。

**Town01**

- `--stop-line-forward-offset 6`は、停止線をCARLAの停止位置から車線に沿って6 m前に出す。
  CARLAの停止位置は信号灯の約7 m手前(Town01では6.7〜8.9 m)にあり、そのままだとEGOが交差点のかなり手前で止まるため。
  Town01では、6 m前に出すと停止線から交差点の入口(laneletの終わり)までが0.96〜7.08 m(信号372では2.72 m)になる。
  前に出した停止線がlaneletの外(交差点の中)に出る信号は追加せず、`WARNING: ... not added`を出す。
  Town01でこうならないのは約6.9 mまで。
- 出力は`~/autoware_map/Town01/lanelet2_map_with_tl.osm`(元の地図は変更しない)。すでにある場合は`--force`で上書きする。
- 信号ごとに、信号灯・停止線・信号のregulatory element(IDはOpenDRIVEの信号ID)を追加し、
  交差点手前のlaneletに付ける。Town01では36信号すべてが追加される。
- 出力は元の地図と同じフォルダに置く(CARLAの地図名は`map_path`のフォルダ名から決まるため、
  別のフォルダにすると別の地図を読もうとする)。
- `WARNING: ... not added`が出た信号は、停止位置に合うlaneletが見つからなかったもの、または前に出した停止線がlaneletの外に出たもの。終了コードは2になる。
- 照合ツール(`check_sumo_lanelet2_tl_ids.py`)は、最後に`OK`(終了コード0)が出れば、CARLAの全信号がSUMOと地図の両方にある。ずれがあると`NG ...`と、そのIDが出る。
- `--report-mixed`の`INFO ... -> CARLA shows red`は、SUMOの状態の違うリンクが対応していて、CARLAで赤に置き換えられる信号(3.4.4)。
  Town01では360と374。
- Town01以外のTownで、SUMOとLanelet2の両方に信号を置く手順(停止線をずらす量の決め方、交通量の見直し、記録まで)は、
  Claude Codeのスキル`/carla-town-signals`(`.claude/skills/carla-town-signals/SKILL.md`)にまとめてある。
- 2026-11-16の確認(ブループリント割り当ての計画書S6): 信号372の手前(`334.74,263.00`)からスポーンして地図座標(374.92, −330.61)へ向かう経路で、
  EGOは信号372の停止線の手前((334.6, −313.6)付近。CARLAの停止位置は(334.71, −308.73))で止まり、信号が変わってから走り出した。

**Town05**(2026-10-19に作成)

- 地図はTown01と同じ配布元(CARLA autoware-contents の vector_maps/lanelet2/Town05.osm)を`~/autoware_map/Town05/lanelet2_map.osm`に置く。
  点群も同じ配布元の point_cloud_maps/Town05.pcd(y軸反転版)を`pointcloud_map.pcd`に置く。`map_projector_info.yaml`は「projector_type: Local」。
- 信号は54個(landmark 2376〜2434)。多車線のため、停止位置ごとに1つ、計111個のregulatory elementができる。
  OpenDRIVEのID(+100000)を持つのは各信号の最初の1つだけなので、3.4.2の位置の対応付け(既定)で使う。
  照合ツールは、同じ信号灯を共有するregulatory elementを1つの信号として数える
  (`Lanelet2: 54 traffic lights in 111 regulatory elements`)。結果は`OK`、`INFO`(赤に置き換えられる信号)はなし。
- `--group-id-offset 100000`: 付けないと、地図の既存のID(2376など)とぶつかって止まる。
- `--stop-line-forward-offset 0`: Town05では、CARLAの停止位置がもともと交差点の入口の1.3〜8.1 m手前(中央値6.8 m、
  CARLAの道路で測った値)にあり、Town01で6 m前に出した後と同じくらいなので、前に出さない。
  laneletがTown01より手前で終わるため、1 mでも`WARNING: ... not added`が出る(2395。3 mで4か所、6 mで45か所)。
- 点群に`~/CARLA/HDMaps/Town05.pcd`を使ってはいけない。CARLAの座標のまま(y軸が反転していない)なので、
  NDTの自己位置がずれ、RVizでEGOが車線をまたいで表示される(2026-10-19に発生)。Town01の`pointcloud_map.pcd`も配布元の版。
- 信号の手前に向かうスポーン地点: 信号2408の手前(SUMOの`-16.0.00`、ルートの17台が通る)
  `spawn_point:="24.71,-51.03,0.30,0.0,0.0,91.5"`。手前の交差点から26 m、CARLAの停止位置まで31 m。
  Town05は交差点の間隔が短く(約60 m)、前後とも30 m以上取れる直線のスポーン地点はない。
  `rou/Town05.rou.xml`は0〜199秒に計200台なので、`sumo_warmup_time`は`60`程度にする(600だと車がいなくなる)。
- **重複点**: 配布元のTown05の地図は、48本のlaneletで境界線の最初の2点が同じ位置にある(Town10HDは10本、Town01はなし)。
  そのままだと、map_loaderが作り直す中心線の先頭がNaNになり、そこを通る経路の長さもNaNになる。
  mission_plannerは空の経路を受け取り、`component_container_mt`(mission_planner_container)が終了コード-11で落ちる。
  RVizで2D Goal Poseを置いても経路ができず、以降のゴールも受け付けない(2026-10-21に発生。信号の有無に関係ない)。
  `tools/fix_lanelet2_duplicate_points.py`で直す。`--check`を付けると直さずに数だけ表示する(重複があれば終了コード1)。
  信号入りの地図を直す前の地図から作った場合は、`lanelet2_map_with_tl.osm`にも同じツールをかければよい。
- ゴールは、同じ道路を2回通らずに行ける場所に置く。スポーン地点(`24.71,-51.03`、南向き)から西側の
  (-153, -3)や(-129, 24)へは約1.6 kmの遠回りになり、途中で同じ道路の隣の車線を通るため、
  `Loop detected within route!`(`The planned route is empty.`)で経路ができない。
  (-44, 50)北向きや(0, 84)東向きは経路ができる(座標はmap座標。mission_plannerだけ起動して確認)。
- 実機(3.4.2)で確認済み(2026-10-21、利用者が確認。2.4.1のTown05の起動コマンド)。

**Town10HD**(2026-10-21に作成)

- 地図・点群は2026-07-16に置いたもの(Town05と同じ配布元。点群の座標が地図と合うことを確かめた)。
  配布元の地図は10本のlaneletで境界線に同じ位置の点が続くので、先に直す(2.2.2)。
- 信号は15個(landmark 943〜945・949〜954・957〜962)。どれも2車線で、計30個のregulatory elementができる
  (`Lanelet2: 15 traffic lights in 30 regulatory elements`)。結果は`OK`、`INFO`(赤に置き換えられる信号)はなし。
- `--group-id-offset 100000`: 付けないと、地図の既存のID(943など。最大9690)とぶつかって止まる。
- `--stop-line-forward-offset 0`: CARLAの停止位置は、もともと交差点の入口の0.9〜6.9 m手前(中央値5.4 m、CARLAの道路で測った値)。
  Town05と同じく1 mでも`WARNING: ... not added`が出る(1 mで4か所、3 mで8か所、6 mで24か所)ので、前に出さない。
- SUMOは`examples/Town10HD.sumocfg`を使う(2.4.1の`map_path`・`sumo_cfg_file`のTown01をTown10HDに置き換える)。
  ルートは0〜199秒に計200台なので、Town05と同じく`sumo_warmup_time`は`60`程度にする。
- 信号の手前に向かうスポーン地点: 信号954の手前(CARLAのスポーン地点#80、SUMOの`1_4`、ルートの34台が通る)
  `spawn_point:="30.02,133.95,0.30,0.0,0.0,-179.7"`。CARLAの停止位置まで51 m、手前の交差点から110 m
  (後ろ36 mから先はカーブ)。SUMOの車両の19台がこのedge(`1`)の始点から出発するが、スポーン地点から約110 m後ろ。
  信号954(junction 468)はSUMOで0〜42秒が青、42〜45秒が黄、45〜90秒が赤の繰り返し。
  この車線は交差点を抜けると道路5へ曲がって続くだけなので、ゴールはその先(map座標で(-104, -99)付近、北北西向き)に置く。
- 実機(3.4.2)で確認済み(2026-10-21、利用者が確認。2.4.1のTown10HDの起動コマンド)。
  起動ログは`[EGO SPAWN] t=60.05 s waited=0.00 s`(ウォームアップ60秒、空き待ちなし、そのときのSUMOの車両48台)と
  `Traffic-light matching: 15 matched, 0 ambiguous, 0 too far, 0 overridden (map has 62 heads / 30 groups)`。
  `Different SUMO traffic-light states`は出なかった。

### 3.4.2 起動とログの確認

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

| ログ                                                                                                      | 意味                                                                              |
| --------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| `Traffic-light matching: 36 matched, 0 ambiguous, 0 too far, 0 overridden (map has 36 heads / 36 groups)` | 正常。CARLAの36信号すべてが地図の信号に対応付いた                                 |
| `the map has no traffic-light heads; 36 CARLA lights not published`                                       | 地図に信号がない。`lanelet2_map_file:=lanelet2_map_with_tl.osm`を付け忘れている   |
| `ambiguous CARLA light (opendrive_id=...)` / `unmatched CARLA light (...)`                                | その信号は対応付けできず、publishされない。下の`traffic_light.id_map`で指定できる |

### 3.4.3 オプション

| オプション                     | 説明                                                                                                                                  | デフォルト値                   |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------ |
| `traffic_light.publish`        | CARLAの信号状態をpublishする                                                                                                          | `false`                        |
| `traffic_light.force_green`    | 起動時に全信号を緑にして固定し、終了時に元に戻す。`use_sumo:=true`で`tls_manager`が`sumo`/`carla`のときは指定できない(起動時にエラー) | `false`                        |
| `traffic_light.map_path`       | 対応付けに使うLanelet2地図。空(`""`)にするとOpenDRIVEの信号IDをそのまま使う                                                           | `map_path`/`lanelet2_map_file` |
| `traffic_light.match_distance` | 対応付けを認める信号灯どうしの最大距離 [m]                                                                                            | `5.0`                          |
| `traffic_light.match_ratio`    | 曖昧と判定するしきい値(小さいほど厳しい)                                                                                              | `0.6`                          |
| `traffic_light.id_map`         | 手動の対応付け`opendrive_id:group_id[\|group_id...],...`。位置の対応付けより優先する。`\|`を含むので引用符で囲む                      | `""`                           |

- 状態の対応: CARLAのRed/Yellow/Green → RED/AMBER/GREEN(点灯)、Off → 消灯、それ以外 → UNKNOWN。
- 同じ信号IDに複数のCARLAの信号が対応して状態が違うときは、最も制限の強い状態(赤 > 黄 > 不明 > 消灯 > 青)を出す。

### 3.4.4 注意点

- **SUMO連携(`tls_manager:=sumo`)では、黄信号もAutowareには赤として届くことがある。**
  1つのCARLAの信号に、状態の違うSUMOの複数のリンク(例: 直進は`G`、右左折は`y`)が対応している場合、
  CARLAの信号は赤になる(既存のSUMO連携の仕様。ログに`Different SUMO traffic-light states ... CARLA state is forced to red.`が出る)。
  Town01で起きるのは、信号パターンを手で設定した信号360と374だけ。後から設置した30個の信号は、対応するリンクが常に同じ色になる。
- RMWは`~/.bashrc`と同じ`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`で起動すること。Fast DDSで起動したときは
  ノードどうしの通信が一部できず、Autoを押せる状態にならなかった。
- 実機で確認済みの地図はTown01・Town05・Town10HD。別の地図では、2.2.2のツールが既存のIDとぶつかって止まることがある。
  その場合は`--group-id-offset`(例: `--group-id-offset 100000`)を付け、3.4.2の位置の対応付け(既定)で使う。

## 3.5 SUMOの車両のCARLAブループリント

SUMOの車両1台をCARLAに出すとき、どのブループリントを使うかを、**SUMOのvTypeの寸法(長さ・幅)に最も近いもの**に決める。
Vissim構成と同じ仕組み・同じ規則・同じ外形の計測結果を使う。
設計と確認結果は`docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md`を参照(移植元: SBIR_simのブループリント割り当て処理)。

- 走行中には選ばない。生成ツールで事前に作った表(carla_variants)を、ブリッジは起動時に検査し、走行中はSUMOの車のvTypeのIDで引くだけ。
  乱数は色とドライバーにだけ使う(SUMOの車両IDで決まる。`sync_vehicle_color:=true`ならSUMOの色)。
- 寸法の基準は、CARLAのブループリントを**LiDARで測った外形**(バウンディングボックスではない)。配置もその外形の中心で合わせる(SUMOの前端の位置から置く)。
- 1体で表せない大型車(トラック・トラクター・バス)は、`carlacola`を2〜3体重ねて全長を合わせる(重ね置き)。同じ車の体はすべて同じ色。
- 表にないvTypeの車や、表と寸法の違う車が来たら、黙って別のブループリントに替えずに止める。
- どの表を使うかは`sumo_carla_variants`で決まる(2.4.1)。既定の`auto`は、`.sumocfg`の名前から`sumo_integration/data/carla_variants_sumo_<名前を小文字>.yaml`を探し、
  なければ今までの選び方(vTypeのIDと同じ名前のブループリント、なければ`sumo_integration/data/vtypes.json`からvClassで乱数)にする。表があるのはTown01だけ。

### 3.5.1 Town01の割り当て

`sumo_integration/data/carla_variants_sumo_town01.yaml`(`sumo/examples/Town01.sumocfg`から作成。route-filesの中身は、CARLAリポジトリ`main`の`6a1c918`と同じ)。
14モデルの割り当ては、Vissim構成の`carla_variants_town01.yaml`と同じ。

| SUMOのvType(Vissimのモデル) | CARLAのブループリント | 長さの差 |
| --- | --- | --- |
| `vissim.1.fiat_500` | `nissan.micra` | +0.01 m |
| `vissim.2.tesla_model_y`、`vissim.3.volkswagen_golf` | `audi.tt` | −0.05 m、−0.06 m |
| `vissim.4.mercedes_b_class`、`vissim.8.nissan_qashqai`、`vissim.9.volkswagen_tiguan` | `mini.cooper_s_2021` | +0.09 m、+0.18 m、0.00 m |
| `vissim.5.mercedes_c_class`、`vissim.6.skoda_superb_combi` | `tesla.model3` | 0.00 m、−0.03 m |
| `vissim.7.audi_a6_avant` | `dodge.charger_2020` | +0.01 m |
| `vissim.10.volkswagen_id_4` | `nissan.patrol` | −0.04 m |
| `vissim.22.semi_tractor_volvo_e`(トラクター、5.96 m) | `carlamotors.carlacola` × 2 | 0.00 m |
| `vissim.23.delivery_daf_lf`(トラック、7.93 m) | `carlamotors.carlacola` × 2 | 0.00 m |
| `vissim.31.c2_standard`(バス、12.14 m) | `carlamotors.carlacola` × 3 | 0.00 m |
| `vissim.70.motorbike_yamaha_mt_07`(バイク) | `kawasaki.ninja` | −0.08 m |

候補から外しているもの: パトカー・タクシー・消防車・救急車の塗装、見た目とLiDARの形が違う`european_hgv`、乗用車に合わないバン(`volkswagen.t2*`)、
自車と同じ`toyota.prius`(`data/carla_fleet/carla_variant_rules.yaml`)。

表の作り直し(SUMOの車両の設定・規則・CARLAの版を変えたときだけ)は、2.2.3の手順で行う。

### 3.5.2 Town01の車両の構成(2026-11-16に変更)

Vissim構成と同じ車両の構成で比べられるように、`rou/Town01.rou.xml`の5つの車両グループの中身を、Vissimの`Town01.inpx`の2D/3Dモデル(寸法と、車両タイプ内の割合)にした。
グループどうしの比率と、flow・経路・信号は変えていない。作り方はCARLAリポジトリの`Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`の「Town01の車両の構成」。

| 車両グループ | 比率(flowの台数) | 中身(以前) | 中身(今) |
| --- | ---: | --- | --- |
| `A_car` | 70% | 乗用車16種(vTypeの名前がCARLAのブループリント名) | `vissim.1`〜`vissim.10`の乗用車10モデル |
| `B_truck` | 10% | `carlacola` | `vissim.23.delivery_daf_lf`(7.93 × 2.55 m) |
| `C_trailer` | 5% | `european_hgv` | `vissim.22.semi_tractor_volvo_e`(5.96 × 2.55 m) |
| `D_bus` | 5% | `fusorosa` | `vissim.31.c2_standard`(12.135 × 2.55 m) |
| `E_bike` | 10% | バイク3種 | `vissim.70.motorbike_yamaha_mt_07`(2.098 × 0.907 m) |

- vTypeは`sumo/examples/vissim_fleet_vtypes.rou.xml`にあり、`Town01.sumocfg`の`route-files`は`carlavtypes.rou.xml,vissim_fleet_vtypes.rou.xml,rou/Town01.rou.xml`。
  `carlavtypes.rou.xml`は、自車(CARLA→SUMO)のvTypeのために残している。
- 大型車が長くなったので、SUMO単体の1時間の比較で、平均の所要時間が4〜22%延びた(乱数のシードによる)。既定のシードでは、teleportが3560秒に1件だけだった。
- `sumo_carla_variants:=legacy_vtypes`(今までの選び方)で起動すると、vTypeの名前がブループリント名ではないので、乗用車は`vtypes.json`の14種から乱数で選ばれ、
  トラック・トラクターは`carlacola` 1体になり、**バスはCARLAに出ない**(`vtypes.json`にvClass `bus`の候補がない。ログに`sumo vtype vissim.31.c2_standard not supported`)。
  Town01では比べるためだけに使う。

### 3.5.3 止まったとき(`Error: CARLA blueprint assignment: <code>: ...`)

| code | 意味 | 直し方 |
| --- | --- | --- |
| `variant_config_missing` | `sumo_carla_variants`のファイルがない | パスを確かめる |
| `variant_source_changed`(`source: route_files`) | 渡された`.sumocfg`の`route-files`の並びか中身(sha256)が、表を作ったときと違う | 表を作り直す(2.2.3の(2)) |
| `variant_source_changed`(`source: rules`・`geometry`) | 表を作ったときと、規則か実メッシュが違う | 表を作り直す(2.2.3の(2)) |
| `variant_source_changed`(`source: vehicle_dimensions`、走行中) | SUMOの車の寸法が、表のモデルと違う | `.rou.xml`を確かめ、表を作り直す(2.2.3の(2)) |
| `variant_blueprint_missing` | 表のブループリントが、接続先のCARLAにない | CARLAの版を確かめ、外形を測り直して表を作り直す(2.2.3の(1)(2)) |
| `variant_mesh_carla_version_changed` | 外形を測ったCARLAの版と、接続先の版が違う | 外形を測り直して表を作り直す(2.2.3の(1)(2)) |
| `variant_model_ambiguous` | 同じvTypeが2つのモデルにある | 表を作り直す |
| `npc_model_unknown`(走行中) | 表にないvTypeの車が来た | 使っている`.sumocfg`から、表を作り直す(2.2.3の(2)) |
| `variant_model_empty`(起動時・走行中) | 割合 > 0のvTypeに割り当てがない、または表を作ったときに割合0だったvTypeの車が来た | 同上 |

- 起動時の検査は、SUMOを起動する前に行う。そのため、起動時に止まったときはSUMOは起動していない。
- 終了時に、ブループリントごとのspawn回数がログに1行出る(`CARLA bodies spawned for SUMO vehicles: ...`。括弧内は実際の比率と、表から見込んだ比率)。

### 3.5.4 SUMOがEGOを消したとき

SUMOは、`time-to-teleport`(既定300秒)のあいだ止まったままの車を、渋滞の原因とみなしてteleportする(行き先の辺を越えると消す)。
EGOがスポーンしてから走り出さずに止まっていると、SUMOの中のEGO(`carla0`)がこれで消される。

- ブリッジは、SUMOが消したEGOを、同じステップのうちに新しいID(`carla1`、`carla2` …)でSUMOへ入れ直し、co-simを続ける。ログに次の行が出る。
  `[sync] sumo removed CARLA-origin vehicle carla0 (CARLA actor <id>), e.g. a teleport after standing still for time-to-teleport; adopting it into sumo again`
- 2026-11-16より前は、ここでブリッジが`KeyError: 79`で落ちていた(ブループリント割り当ての計画書S5の結果の7)。
- EGOが止まっている間、後ろのSUMOの車が詰まるのは今までどおり。
