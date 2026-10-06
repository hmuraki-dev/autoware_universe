# SUMO-CARLA-Autoware co-sim 起動手順

# 0. 前提条件
Autoware環境を構築済みであること。
環境構築については、「Autoware1.9.0環境構築ガイド_v1.0.md」を参照。


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
```

`autoware_carla_interface` は独立したリポジトリではなく、`autoware_universe` リポジトリ配下のパッケージである。

# 2. 実行コマンド

## 2.1 CARLAサーバー起動(ターミナル1)

```bash
cd ~/CARLA
./CarlaUE4.sh
```
コマンドを実行すると、CARLAサーバーが起動し、CARLA(Unreal Engine)の画面が表示される。


## 2.2 Town01ロード(ターミナル2)

```bash
cd ~/CARLA/PythonAPI/util
python3 config.py --map Town01
```
コマンドを実行すると、CARLAで読み込まれているマップが Town01 に切り替わる。CARLA画面が Town01 の地図に更新されたことを確認する。


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

### 2.4 SUMO/CARLA/Autoware起動(ターミナル3)

#### 2.4.1 起動コマンド

```bash
source install/setup.bash
export ROS_DOMAIN_ID=33

ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_sumo:=true \
  sumo_gui:=true \
  sumo_cfg_file:=/home/divp/CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg \
  tls_manager:=sumo \
  sync_vehicle_lights:=true \
  sync_vehicle_color:=true \
  spectator_follow:=true \
  sumo_warmup_time:=600 \
  spawn_point:="199.95,326.97,0.30,0.0,0.0,180.0" \
  2>&1 | tee -i /tmp/autoware_carla.log
```

- 最後の3行は、ウォームアップ・EGO安全スポーン(2.4.2)を使う場合の指定である。
  - ウォームアップを使わない場合は、`sumo_warmup_time`と`spawn_point`の行を省く(EGOは起動直後にランダムな位置へスポーンされる)。
    `spawn_point`だけを指定すれば、ウォームアップなしで固定位置にスポーンする。
  - `2>&1 | tee -i /tmp/autoware_carla.log`はログ保存用(2.4.4のgrepで使う)。不要なら省いてよい。
    `-i`を付けないと、Ctrl+Cで`tee`も終了し、それ以降(終了処理)のログが残らない。

##### オプション一覧

| オプション | 説明 | 設定例 | デフォルト値(未指定時) |
|-----------|------|--------|--------------------------|
| `simulator_type` | 使用するシミュレータ | `carla` | なし(指定必須) |
| `map_path` | Lanelet2マップ | `$HOME/autoware_map/Town01` | なし(指定必須) |
| `vehicle_model` | 車両モデル | `sample_vehicle` | Launch既定値 |
| `sensor_model` | センサキット | `carla_sensor_kit` | Launch既定値 |
| `use_sumo` | SUMO連携 | `true` | `false` |
| `sumo_cfg_file` | SUMO設定 | `...Town01.sumocfg` | なし |
| `sumo_gui` | GUI表示 | `true` | `false(ヘッドレス)` |
| `sumo_host` | TraCIホスト | `localhost` | `localhost` |
| `sumo_port` | TraCIポート | `8813` | `8813` |
| `sumo_client_order` | クライアント順 | `1` | `1` |
| `tls_manager` | 信号管理 | `sumo` | `none` |
| `sync_vehicle_lights` | 灯火同期 | `true` | `false` |
| `sync_vehicle_color` | 車体色同期 | `true` | `false` |
| `spectator_follow` | EGO車両(role_name=`ego_vehicle_role_name`)にCARLAスペクテーターを自動追従させる | `true` | `false` |
| `spawn_point` | EGOのスポーン位置(CARLA座標、`x,y,z,roll,pitch,yaw`) | `"199.95,326.97,0.30,0.0,0.0,180.0"` | `None`(ランダム) |
| `vehicle_type` | EGOのCARLAブループリント | `vehicle.toyota.prius` | `vehicle.toyota.prius` |
| `sumo_warmup_time` | SUMOだけを先に進める秒数(整数)。0で無効。2.4.2参照 | `600` | `0`(無効) |
| `ego_spawn_front_margin` | EGOスポーン時に必要な前方車とのすき間 [m]。2.4.2参照 | `20.0` | `20.0` |
| `ego_spawn_rear_margin` | EGOスポーン時に必要な後方車とのすき間 [m]。2.4.2参照 | `20.0` | `20.0` |
| `ego_spawn_wait_timeout` | ウォームアップ後に空きを待つ上限秒数(整数)。2.4.2参照 | `60` | `60` |

- `sumo_gui:=true`のsumo-guiは、起動後に自動でシミュレーションを開始する(`--start`)。Runボタンを押す必要はない。
  また、co-simの終了時にはダイアログを出さずに自動で閉じる(`--quit-on-end`)。
- `spectator_follow:=true`はCARLAスペクテーター(自由視点カメラ)をEGO車両に自動追従させる。
  カメラの距離・高さ・角度(`--distance`/`--height`/`--pitch`/`--rate`)はlaunch引数として
  公開されていないため、細かく調整したい場合はこの引数は使わず`ros2 run
  autoware_carla_interface spectator_follow --distance ... --height ...`のように別ターミナルで
  手動起動すること。

#### 2.4.2 ウォームアップ・EGO安全スポーン

SUMOだけを先に所定時間進めて交通流を作ってから、EGOのスポーン位置の前後が空いたタイミングでEGOをスポーンする機能。
`sumo_warmup_time`に1以上を指定すると有効になり、指定しない(既定の0)場合は従来と同じ動作になる。
設計は`docs/SUMO_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md`を参照。

- 起動の流れ: Autoware起動 → SUMOだけを`sumo_warmup_time`秒進める(CARLAは止まったまま) →
  その間にSUMOに現れた車両をCARLAへまとめてスポーン → EGOのスポーン位置の前後の空きを判定 →
  空いていなければSUMO・CARLAを1ステップずつ進めて再判定 → 空いたらEGOをスポーン → 通常のco-sim。
- 空きの判定: 同じ車線の前方車・後方車とのすき間(バンパー間の距離)が、それぞれ`ego_spawn_front_margin`・
  `ego_spawn_rear_margin`以上で、かつEGOの予定位置に重なる車両・歩行者がいなければスポーンする。
- EGOがスポーンされるまでは、CARLA上にEGOはおらず、RVizにもEGOは表示されない。
  ログに`[EGO SPAWN]`が出て、RVizにEGOが表示されてから、目的地を設定してAutoを押す(目的地の設定方法は従来と同じ)。
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
なお、宣言がなくてもコマンドラインで指定した値はノードまで届く。届いているかは2.4.4の`[EGO SPAWN CHECK] start:`の行の値で確認できる。

#### 2.4.3 スポーン地点の選び方

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
   SUMO車両が出発する道路(Town01では`-19.0.00`・`5.0.00`)の始点付近は、車両が突然現れるので避ける。

Town01(`rou/Town01.rou.xml`、入力2か所×600台/h)で確認済みの地点:

| 地点 | 用途 | SUMOの車線 | 交通量 | `spawn_point` |
|---|---|---|---|---|
| #37 | 交通量が多い(空き待ちが起きる) | `6.0.00` | 600台/h | `199.95,326.97,0.30,0.0,0.0,180.0` |
| #36 | SUMO車両が来ない(すぐスポーン) | `-6.0.00`(#37の対向車線) | 0台/h | `199.95,330.46,0.30,0.0,0.0,-0.0` |
| #144 | 別の道路で交通量が多い | `1.0.00` | 525台/h | `256.55,2.02,0.30,0.0,0.0,-0.0` |
| #249 | 交通量が少ない | `-4.0.00` | 189台/h | `220.14,133.24,0.30,0.0,0.0,-0.0` |

- `sumo_warmup_time:=600`・既定の必要距離での実績: #37は空き待ち13.75秒(シミュレーション時間)でスポーン、#36は待ちなし。
- `spawn_point`のzにはCARLAの道路の高さ(Town01ではほぼ0〜0.3)を入れる。コード側で+2 mしてスポーンする。

#### 2.4.4 ログの見方

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
| `[EGO SPAWN CHECK] t=600.05 front=sumo:in1_A.52/carla:245 clearance=13.0 m rear=... clearance=-1.2 m overlap=... result=WAIT` | 判定結果。前方車・後方車とのすき間、EGOの位置に重なっている車両、`WAIT`/`SAFE`。結果や前後の車両が変わったときと、それ以外は1秒ごとに出る |
| `[EGO SPAWN] t=613.80 s waited=13.75 s spawn_point=(...) vehicles=42` | EGOをスポーンした時刻と空き待ちの時間 |

記録しておく項目(再現性のため): `.sumocfg`とルートファイル(SUMOの乱数シードを含む)、`sumo_warmup_time`・必要距離・
待ち時間上限、`spawn_point`、上の`[SUMO WARMUP] completed`・`[EGO SPAWN CHECK]`(最後のSAFE)・`[EGO SPAWN]`の行。
同じ`.sumocfg`・同じパラメータなら、SUMOの車両の動きは同じになる(開発用PCとLinux機で一致を確認済み)。

#### 2.4.5 試験開始失敗と終了

- `ego_spawn_wait_timeout`秒待っても空かない場合や、空いた後にEGOのスポーンに失敗した場合は、
  `Error: test start failed: ...`(最後の判定結果を含む)を出して後始末し、`autoware_carla_interface`が終了コード1で終了する。
  `autoware_carla_interface`ノードには`on_exit="shutdown"`が付いているので、launch全体(RViz・Autoware)も終了する。
  スポーン地点を変える、`sumo_warmup_time`を変える、`ego_spawn_wait_timeout`を延ばす、などで対処する。
- `sumo_gui:=true`のsumo-guiは、終了時にダイアログを出さずに自動で閉じる(`--quit-on-end`)。
- ウォームアップ中・空き待ち中にCtrl+Cを押すと、EGOをスポーンせずに後始末して終了する。
- Ctrl+Cで終了したときに、最後に`[ERROR] [launch]: Caught exception in launch ...: Cannot shutdown a ROS adapter that is not running`
  が1行出ることがあるが、launch側のメッセージで実害はない。

### 2.5 [appendix] 処理時間計測

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

### 2.6 歩行者(Pedestrian)同期について

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
