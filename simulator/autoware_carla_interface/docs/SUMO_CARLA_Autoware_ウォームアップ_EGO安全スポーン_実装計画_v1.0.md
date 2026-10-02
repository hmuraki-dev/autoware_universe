# SUMO–CARLA–Autoware ウォームアップ・EGO安全スポーン 実装計画

作成日: 2026-09-30
版: v1.0
対象ブランチ:
- 本リポジトリ(`autoware_universe`): **`feat/sumo-warmup`**(作業ブランチ)
  - `feature/sumo_co-sim`の最新版から作成し、本計画の実装はすべてこのブランチで行う。
  - Step S9(実機検証)の完了後、`feature/sumo_co-sim`へマージする。
  - Vissim版(`feat/vissim-warmup`、`Vissim_CARLA_Autoware_ウォームアップ_EGO安全スポーン_実装計画_v1.0.md`)とは
    **ブランチもコードも分ける**。Vissim版のコード・ドキュメントはこのブランチに持ち込まない。
- CARLAリポジトリ(`/home/divp/CARLA`): 無改修の見込み。

前提ドキュメント:
- `docs/SUMO_CARLA_Autoware_統合修正項目_v0.5.md`(同期ループ・auto-adaptの元設計)
- `docs/SUMO_CARLA_Autoware_統合_実装ステップ計画_v1.1.md`(Step 1〜7の実装内容)
- `docs/SUMO_CARLA_Autoware_歩行者同期_実装計画_v1.0.md`(歩行者同期)
- 要件の出典はVissim版と同じ(OneDrive `doc/20260929_ウームアップ/`の引き継ぎ資料・チーム共有資料)。

---

## 0. 背景と要件

### 0.1 現状の問題

- SUMOは開始直後、ルートファイルの`depart`に従って車両が順次流入するため、しばらくは想定した交通流になっていない。
- 現状はco-sim開始と同時にEGOをスポーンするため、交通流が形成される前から検証が始まってしまう。
- 先にSUMOだけを進めて交通流を作ると、今度はEGOのスポーン予定位置付近にSUMO車両がいる可能性があり、
  スポーン直後の衝突・重複や、SUMO側の追従の不自然さが起こりうる。

### 0.2 要件(Vissim版と同じ。SUMO向けに読み替え)

| # | 項目 | 決定 |
|---|---|---|
| 1 | 全体の流れ | `WARMUP → WAIT_FOR_SAFE_GAP → SPAWN_EGO → NORMAL_COSIM` の4状態 |
| 2 | ウォームアップ | SUMOだけを所定時間進める。CARLA/Autowareとの同期は行わない。可能な限り高速に進める |
| 3 | ギャップ判定の基準 | **CARLA基準**。位置はCARLA上の車両中心、車長はCARLA車両の`bounding_box`から求める |
| 4 | 車間距離の式 | `中心間距離 − 前の車の車長/2 − 後ろの車の車長/2`(前方・後方それぞれ) |
| 5 | SUMOとの車長差 | 前方・後方の必要距離(パラメータ)を長めに設定して吸収する。より安全にする場合は両者の長いほうの車長を使う(オプション、§2.6.4) |
| 6 | 空きがない場合 | SUMOを1ステップ進めて再判定する。交通流には手を加えない |
| 7 | タイムアウト | 待ち時間の上限を超えたら**試験開始失敗**として終了する。無理にスポーンしない |
| 8 | 外部指定する設定値 | ウォームアップ時間・前方必要距離・後方必要距離・待ち時間上限 |
| 9 | ログ | 判定のたびに結果を出す。再現性のための情報を記録する |
| 10 | 既存処理 | 通常のSUMO–CARLA同期処理(STEP周期、同期モード、信号同期、車両・歩行者同期、Autoware起動、timestamp)は変えない。**ウォームアップ無効時(既定)は現状と完全に同じ動作** |

### 0.3 スコープ外

- ルートファイル(交通需要)の作成・変更(§1.7のとおり、Town01の例ではウォームアップの効果が出ない)
- ウォームアップ中からスポーン地点を空ける方式、周辺車両を削除する方式
- 速度に応じて必要距離を変える方式(将来の拡張候補、§7)
- Vissim版との共通化(ブランチが別のため。共通化はマージ方針が決まってから検討する)

---

## 1. 前提調査で判明した事実

### 1.1 現在の起動・ループ構造(`carla_autoware.py`)

- `InitializeInterface.load_world()`(`carla_autoware.py:287`)で、CARLAのsync mode設定 →
  `_init_sumo_integration()`(:317)→ **EGOスポーン**(`CarlaDataProvider.request_new_actor()`、:320)→
  センサー設定 → Traffic Manager設定、の順に実行する。つまり**EGOはループ開始前にスポーン済み**。
- `run_bridge()`(:332)のループは`max_real_delta_seconds`でwall-clockのペースを合わせ、毎回
  `SensorLoop._tick_sensor()`(:98)を呼ぶ。1回の中身は次のとおり。
  1. `GameTime.on_carla_tick()` / `CarlaDataProvider.on_carla_tick()`
  2. `self.sensor()`(ROS publish。`/clock`もここで出る)→ EGOへ制御を適用
  3. `sumo_sync.sync_sumo_to_carla()`(内部で`sumo.tick()` = `traci.simulationStep()`)
  4. `world.tick()`(CARLAが1ステップ進む)
  5. `sumo_sync.sync_carla_to_sumo()`(EGOを含むCARLA車両をSUMOへ反映。auto-adapt)
- Vissim版と違い、**シミュレーション期間の管理(`end_tick`等)はない**。SUMO側の終了は`.sumocfg`の`<end>`次第で、
  Town01の`.sumocfg`には`<end>`がない(§1.7)。

### 1.2 シミュレーション時刻

- ROSの`/clock`とセンサーのtimestampは`GameTime.get_time()`(`carla_ros.py:331`)。
  `GameTime._current_game_time`は**最初に`on_carla_tick()`が呼ばれたときに`delta_seconds`1回分**から始まり、
  以後はフレーム差分×`delta_seconds`で積算する(`modules/carla_data_provider.py:842-868`)。
- SUMOの時刻はROS側へは一切流れていない。
- → **SUMOだけ先に進めても、また空き待ち中にCARLAを`world.tick()`しても、`GameTime.on_carla_tick()`を
  呼ばなければROS時刻は通常co-sim開始時点から0付近で始まる**。

### 1.3 SUMOの進み方とウォームアップの高速化

- SUMOはco-simのtickごとに`traci.simulationStep()`で1ステップ(`--step-length` = `fixed_delta_seconds`)進むロックステップ。
- SUMOはLinux機でローカル起動される(`traci.start()`、`sumo_host`/`sumo_port`が`None`の場合)。
  Vissim版のようなRPC往復はなく、1ステップはTraCIのローカル呼び出しだけ。
- ペース合わせ(`max_real_delta_seconds`のsleep)は`run_bridge()`のループにだけある。
  → ウォームアップを`run_bridge()`の外の専用ループで`sumo.tick()`だけ回せば、sleepもCARLA tickも挟まず、
  **SUMOの計算速度そのままで進む**。
- Town01で計測した結果、100秒ぶんのウォームアップはLinux機(DIVP-WS03)で**約1.6秒**、開発用PC(Windows)で約0.9秒(§6 #2)。
  `traci.simulationStep(目標時刻)`で一気に進める方式も試したが、ステップごとに回す方式と速さはほぼ同じだったため、
  **ステップごとに`sumo.tick()`を回す方式**にする(進捗ログ・停止フラグの確認・信号マネージャの更新が既存どおり行えるため)。

### 1.4 SUMO車両のCARLAへの反映(`simulation_synchronization.py`)

- `sync_sumo_to_carla()`はSUMO車両を**「このステップで出発した車両」(`sumo.spawned_actors` =
  `traci.simulation.getDepartedIDList()`)だけ**CARLAへスポーンする(:128-142)。`spawned_actors`は
  `sumo.tick()`のたびに上書きされる(`sumo_simulation.py:586`)。
  → **ウォームアップ中に`sumo.tick()`だけ回すと、その間に出発した車両は二度とCARLAにスポーンされない**。
  ウォームアップ終了時に「今いる全車両」をCARLAへスポーンする処理(キャッチアップ)が必要(§2.4)。
  歩行者(`spawned_persons` = `getDepartedPersonIDList()`)も同じ。
- CARLAへの反映にはTraCIのsubscribeが前提(`sumo.get_actor()`はsubscription結果を読む)。
  ウォームアップ中に出発した車両はsubscribeされていないので、キャッチアップでsubscribeする。
- CARLAにスポーンできなかった車両(対応するブループリントがない、スポーン失敗)は`sumo2carla_ids`に入らず再試行もされない。
  ブループリントがない車両は`unsubscribe`される(:141-142)。
  → ギャップ判定では、**CARLAにいないSUMO車両も見落とさない**ようにする(§2.6.3)。
- SUMO車両の位置は前端中央基準。ブリッジは**SUMOの車長**(`VAR_LENGTH`/2 = `extent.x`)で中心基準へ変換して
  CARLAに置く(`bridge_helper.py:53-78`)。そのため**CARLA上の中心はSUMO上の中心と一致し**、
  SUMOとCARLAの車長差は前後に半分ずつのずれとして現れる(Vissim版は前端一致で、後端に全部ずれる)。

### 1.5 車長

- Vissim版と違い、**SUMOはすべての車両の車長を返す**(`traci.vehicle`の`VAR_LENGTH`。subscribe済み)。
  CARLAにいない車両も、仮の車長ではなくSUMOの車長で判定できる。
- Town01の例では、SUMOの車両タイプ(`carlavtypes.rou.xml`)は**CARLAのブループリントIDそのもの**で、
  車長もCARLAの`bounding_box`から作られている(例: `vehicle.toyota.prius`のSUMO車長4.54 m)。
  `BridgeHelper.get_carla_blueprint()`はIDが一致するブループリントをそのまま使うので、
  **Town01ではSUMOとCARLAの車長差はほぼない**(S0 #4で確認済み。最大でford.mustangの0.19 m)。
- 車両タイプIDがブループリントにない場合は、vClassから`vtypes.json`の候補をランダムに選ぶため、車長差が出る。

### 1.6 信号(`tls_manager`)

- `tls_manager=sumo`(起動手順書の推奨): SUMOの信号をCARLAへ反映。CARLA側の信号は凍結される。
  ウォームアップ中はSUMOの信号が通常どおり動くので問題ない。ウォームアップ後の最初の同期ステップでCARLAへ反映される。
- `tls_manager=carla`: SUMOの信号は`switch_off`され、CARLAの信号をSUMOへ反映する。
  ウォームアップ中はCARLAを進めないので**SUMOの信号が消えたまま**になり、交通流が実際と異なる。
  → **`tls_manager=carla`ではウォームアップを使えないようにする**(起動時エラー、§2.2)。
- `tls_manager=none`: 各自独立。問題ない。

### 1.7 Town01の交通需要(S0 #6の計測結果)

**検証に使う需要(2026-10-01に変更)**: `CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg`(`rou/Town01.rou.xml`を変更したもの)。

- 2つの入力(`-19.0.00`→`-18.0.00`、`5.0.00`→`-18.0.00`)それぞれ600台/hを、`<flow>`(`period="exp(...)"`のポアソン発生、0〜3600秒)で流入させる。
  車両グループの割合はA(乗用車)70%・B(トラック)10%・C(トレーラー`european_hgv`)5%・D(バス`fusorosa`)5%・E(二輪)10%。歩行者はなし。
- 計測(開発用PC、1800秒)では、出発は1800秒で591台(約1180台/h)。車両数は120秒で約30台、400秒で約40台に達するが、
  **その後もゆっくり増え続け、30分では一定にならない**(600〜900秒の平均44台 → 1500〜1800秒の平均65台、最大81台)。下記§6-4。
  → ウォームアップ時間は「完全に安定するまで」ではなく、目的の交通量に達するまでの時間で決める(例: 400〜600秒で約40台)。
  増え続ける原因(信号での滞留の蓄積など)はsumo-guiで確認する。
- 新しく出発する車種のうち、`vehicle.carlamotors.european_hgv`(7.94 m)と`vehicle.mitsubishi.fusorosa`(10.27 m)は
  `vtypes.json`に登録されていないが、`BridgeHelper.get_carla_blueprint()`はブループリントIDの完全一致を先に探すため、
  CARLAにこのブループリントがあればそのままスポーンされる(S4で確認)。

**変更前の需要(参考、2026-09-30の計測)**:

- `rou/Town01.rou.xml`は**車両100台を`depart`=0〜99秒で1台ずつ出発させるだけ**で、flowによる継続的な流入はない。
  歩行者は`personFlow`(3600秒間)。
- 計測では、車両数は100秒付近で最大(72〜73台)になり、その後は到着で減り続け、**400秒以降は0台**になる。
  → **Town01の例のままでは「交通流が安定するまで待つ」ことにならない**。ウォームアップ時間を長くするほど車両が減る。
  ウォームアップの効果を得るには、継続的な需要(`<flow>`や`randomTrips.py --period`)を持つルートファイルが必要(スコープ外)。
- 車両がすべて到着した後も、SUMOは`simulationStep()`を受け付けて進み続けた(1500秒まで確認)。
  `.sumocfg`に`<end>`がないため、ウォームアップや空き待ちが長くなってもSUMOが先に終わることはない。

### 1.8 CARLA側のEGOスポーンとauto-adapt

- EGOは`CarlaDataProvider.request_new_actor()`でスポーンされ、次の`sync_carla_to_sumo()`でauto-adaptにより
  SUMOへ登録される(`get_sumo_vtype()` → `sumo.spawn_actor()` → 以後毎ステップ`moveToXY`)。
- 空き待ち中(EGOなし)に通常の同期ステップを回しても、CARLA側にSUMO以外の車両はいないので、
  auto-adaptでSUMOへ登録されるものはない(`use_traffic_manager`との併用は起動時に禁止済み)。

---

## 2. 設計

### 2.1 全体の流れ

```text
load_world()
  ├─ CARLA設定 / _init_sumo_integration()          (既存)
  ├─ [warmup無効] EGOスポーン + センサー設定         (既存のまま)
  └─ [warmup有効] EGOスポーンは後回し

[warmup有効時のみ] run_ego_spawn_gate()   ← 新規。run_bridge()の前に呼ぶ
  ├─ WARMUP            : sumo.tick() のみを sumo_warmup_time 秒ぶん。sleepなし
  ├─ (キャッチアップ)  : 今いるSUMO車両・歩行者をCARLAへ一括スポーン → world.tick()
  ├─ WAIT_FOR_SAFE_GAP : ギャップ判定 → NGなら「通常の同期1ステップ(EGOなし)」→ 再判定
  │                      上限時間を超えたら「試験開始失敗」で終了
  └─ SPAWN_EGO         : EGOスポーン + センサー設定

run_bridge()                                      (既存のまま = NORMAL_COSIM)
```

- 状態遷移はco-simのループ本体(`SensorLoop._tick_sensor()`)に入れず、**`run_bridge()`の前段の独立した
  処理**にする。通常co-simのコードパスには手を入れない(要件10)。
- 「通常の同期1ステップ(EGOなし)」は、既存の`sync_sumo_to_carla()` → `world.tick()` →
  `sync_carla_to_sumo()`をそのまま呼ぶ。`GameTime`/`CarlaDataProvider.on_carla_tick()`と
  `sensor()`は呼ばない(ROS時刻を進めない・publishしない)。

### 2.2 パラメータ

launch arg / ROS paramとして追加する(`launch/autoware_carla_interface.launch.xml`のarg・param、
`carla_ros.py`のパラメータ定義の両方)。

| 名前 | 型 | 既定値 | 説明 |
|---|---|---|---|
| `sumo_warmup_time` | int(秒) | `0` | SUMOだけを先に進める時間。**0 = 無効(現状と同じ動作)**。0以上必須 |
| `ego_spawn_front_margin` | double(m) | `20.0` | EGO前端と前方車後端の必要距離 |
| `ego_spawn_rear_margin` | double(m) | `20.0` | EGO後端と後方車前端の必要距離 |
| `ego_spawn_wait_timeout` | int(秒) | `60` | 空き待ちの上限。超えたら試験開始失敗。1以上必須 |

- 必要距離・待ち時間上限の名前はVissim版と同じにする(起動手順書を揃えるため)。ウォームアップ時間だけ`sumo_`を付ける。
- 必要距離のparamは`$(eval "float('...')")`で渡す。`ego_spawn_front_margin:=25`のように整数で指定しても、
  DOUBLEで宣言したROS paramと型が合うようにするため(rclpyは宣言と異なる型の値を受け付けない)。
- `ros2 launch autoware_launch e2e_simulator.launch.xml`から指定するには、`autoware_launch`側
  (Linux機の`~/autoware.1.9.0/src/launcher/autoware_launch/autoware_launch/launch/e2e_simulator.launch.xml`、別リポジトリ)にも
  既存の`sumo_*`引数と同じように4つの引数の受け渡しを追加する必要がある。追加していない場合は既定値(ウォームアップ無効)で動作する。

起動時の検査(`_check_sumo_warmup_params()`を追加し、`__init__`で`_check_sumo_traffic_manager_exclusivity()`の後に呼ぶ):
- `sumo_warmup_time > 0`は`use_sumo=True`のときだけ有効。
- `sumo_warmup_time > 0`のときは`spawn_point`の指定を必須にする(ランダムスポーンでは判定する位置が決まらない)。
- `sumo_warmup_time > 0`かつ`tls_manager=carla`はエラー(§1.6)。
- 各値の範囲(`sumo_warmup_time ≥ 0`、`ego_spawn_wait_timeout ≥ 1`、必要距離 ≥ 0)。

判定用の固定値(`sumo_integration/ego_spawn_gate.py`に置く。launch argにはしない。
`sumo_integration/constants.py`は公式ブリッジ由来の無改修ファイルなので触らない):

| 名前 | 値(案) | 用途 |
|---|---|---|
| `EGO_SPAWN_SEARCH_RANGE_M` | `100.0` | 前後車を探す縦方向の範囲 |
| `EGO_SPAWN_HEADING_TOLERANCE_DEG` | `45.0` | 同一進行方向とみなす向きの差 |

### 2.3 WARMUP

- `sumo.tick()`だけを`sumo_warmup_time / fixed_delta_seconds`回呼ぶ。CARLAはtickしない。sleepしない。
- 停止フラグ(SIGINT/SIGTERM)を毎ステップ確認する。
- 進捗ログを10秒(SUMO時間)ごとに出す(車両数・歩行者数・経過・wall-clock)。
- `traci.exceptions.FatalTraCIError`(SUMOの異常終了)は試験開始失敗として終了する。

### 2.4 キャッチアップ(WARMUP → WAIT_FOR_SAFE_GAP)

- `SimulationSynchronization`に`spawn_all_sumo_actors_in_carla()`を追加する。
  - `traci.vehicle.getIDList()`のうち、`sumo2carla_ids`にも`carla2sumo_ids`の値にもない車両を、
    既存の`sync_sumo_to_carla()`のスポーン処理と同じ手順(`subscribe` → `get_actor` → `get_carla_blueprint` →
    `get_carla_transform` → `carla.spawn_actor`、ブループリントがなければ`unsubscribe`)でスポーンする。
  - 歩行者も同様(`traci.person.getIDList()`、`subscribe_person` → … → `sumo2carla_ped_ids`)。
  - `SumoSimulation`にIDの一覧を返すメソッド(`get_vehicle_ids()`/`get_person_ids()`)を追加する。
- 既存の`sync_sumo_to_carla()`のスポーン条件(差分のみ)は**変えない**。変えると、スポーンに失敗した車両の
  再試行が毎ステップ発生し、通常co-simの挙動とログが変わるため。
- 呼んだ後に`world.tick()` → `carla.update_actor_diff()`を1回行い、CARLA側の差分管理を最新にする
  (キャッチアップでスポーンした車両がEGOと誤認されてSUMOへauto-adaptされないよう、
  `sumo2carla_ids`登録後に差分更新する)。
- 歩行者が多い(Town01では約600人)ため、キャッチアップにかかる時間をS4で計測する。
  既存のスポーンは1台ずつ`apply_batch_sync`するので、遅い場合はバッチ化を検討する(通常co-simのコードは変えない)。

### 2.5 シミュレーション期間

- 本ブランチには期間管理がないため、変更しない。
- `.sumocfg`に`<end>`を設定する運用にする場合は、`<end>` ≥ ウォームアップ時間 + 空き待ち上限 + 検証時間 にすること
  (起動手順書に記載)。`<end>`に達した後のTraCIの挙動はS0 #8で確認する。

### 2.6 ギャップ判定(WAIT_FOR_SAFE_GAP)

CARLA・TraCIに依存しない純粋関数`evaluate_spawn_gap()`として`sumo_integration/ego_spawn_gate.py`に実装し、
単体テストする。入力は「EGO予定位置・向き・車長」と「周辺車両のリスト(ID、中心位置、向き、車長、出所)」。
周辺車両のリストを作る処理(CARLA/TraCIを読む部分)は別関数に分ける。

#### 2.6.1 前後車の抽出(同一車線)

EGO予定位置を原点、EGOの向きを前方とする座標で、各車両の縦位置`s`と横位置`d`を求める。

```text
s = (p_vehicle − p_ego) · forward      # 前方が正
d = (p_vehicle − p_ego) · right
```

次をすべて満たす車両を「同一車線上の車両」とする。

- `|d| < 車線幅/2`(車線幅はEGO予定位置の`map.get_waypoint()`の`lane_width`)
- `|s| ≤ EGO_SPAWN_SEARCH_RANGE_M`
- 向きの差 ≤ `EGO_SPAWN_HEADING_TOLERANCE_DEG`(対向車線を除く)

`s ≥ 0`で最も近い車両を前方車、`s < 0`で最も近い車両を後方車とする。

- この方式は、**スポーン地点が直線区間であること**を前提にする(曲線では横位置の判定がずれる)。
  スポーン地点の選び方の注意として起動手順書に記載する。
- SUMOの車線情報(`traci.simulation.convertRoad()`、`traci.lane.getLastStepVehicleIDs()`)を使えば曲線でも判定できるが、
  要件3(CARLA基準)に合わせてCARLA座標で判定する。SUMO車線での判定は将来の拡張候補とする(§7)。

#### 2.6.2 クリアランスと判定

```text
front_clearance = s_front − L_ego/2 − L_front/2
rear_clearance  = |s_rear| − L_ego/2 − L_rear/2
```

- `L`はCARLA上の車長(`bounding_box.extent.x × 2`)。
- `L_ego`はスポーン前には`bounding_box`が取れないため、次の順で決める。
  1. SUMOに`vehicle_type`と同じIDの車両タイプがあれば、その車長(`traci.vehicletype.getLength()`。
     Town01では`carlavtypes.rou.xml`がCARLAの寸法から作られている)
  2. なければ、S0 #4で確認した`vehicle_type`の車長を`ego_spawn_gate.py`の表から引く
     (`vehicle.toyota.prius`はCARLA 4.51 m、SUMO 4.54 mで、1.の方法でも差は0.03 m)
- `front_clearance ≥ ego_spawn_front_margin` かつ `rear_clearance ≥ ego_spawn_rear_margin` ならSAFE。
- 前方車(後方車)がいない側は条件を満たすとみなす。
- **重複チェック**: 車線によらず、EGO予定位置の外形と重なる車両が1台でもあればUNSAFE
  (隣の車線にはみ出している車両や、交差点内の車両の見落とし防止)。歩行者も重複チェックの対象にする。

#### 2.6.3 CARLAにいないSUMO車両

- ブループリントがない・スポーン失敗でCARLAにいないSUMO車両も、SUMOの位置から判定に含める。
- 中心位置はSUMOの前端位置を`BridgeHelper.get_carla_transform()`でCARLA座標の中心へ変換したもの、
  車長はSUMOの車長(`traci.vehicle.getLength()`)を使う(Vissim版のような仮の車長は不要)。
- 対象は`traci.vehicle.getIDList()`のうち`sumo2carla_ids`にないもの。subscribeされていないので`traci.vehicle`の
  getterで直接読む(判定1回あたり数十台程度なので問題ない見込み)。
- ログでは`source=sumo_only`として区別する。

#### 2.6.4 SUMOとの車長差(要件5)

- 基本は**必要距離の設定で吸収**する。§1.4のとおり中心がSUMOとCARLAで一致するため、前方・後方とも
  車長差の半分ずつが効く。Town01の例では車長差は最大0.19 mで、前後それぞれ0.1 m程度にしかならない(§6-3)。
- オプション(Step S8、必要になったら): `L = max(CARLAの車長, SUMOの車長)`で計算する。
  SUMOの車長はsubscription結果にすでにあるため、Vissim版と違って対応表は不要で、実装は小さい。

#### 2.6.5 待ちの1ステップ

- NGなら「通常の同期1ステップ(EGOなし)」を1回実行し、再判定する。
- ペース合わせのsleepは入れない(ウォームアップと同様、できるだけ速く)。
- `ego_spawn_wait_timeout / fixed_delta_seconds`ステップ待ってもSAFEにならなければ、試験開始失敗として終了する(§2.8)。

### 2.7 SPAWN_EGO → NORMAL_COSIM

- 既存の`load_world()`にあるEGOスポーン・センサー設定・Traffic Manager設定を`_spawn_ego_and_sensors()`に切り出し、
  - ウォームアップ無効時: 現状どおり`load_world()`から呼ぶ
  - ウォームアップ有効時: SAFE判定の直後に呼ぶ
- EGOのSUMOへの登録は、既存のauto-adapt(次の`sync_carla_to_sumo()`)にそのまま任せる。
- `run_bridge()`へ進む。`GameTime`はここで初めて`on_carla_tick()`されるので、ROS時刻は0付近から始まる。

### 2.8 終了とエラー

| 事象 | 動作 |
|---|---|
| 空き待ちの上限超過 | `Error: no safe gap found within ego_spawn_wait_timeout ...`を出し、試験開始失敗として終了(終了コード非0)。既存の`_cleanup()`で後始末する |
| ウォームアップ中・空き待ち中にSUMOが異常終了 | 試験開始失敗として終了 |
| EGOスポーン失敗(`request_new_actor`がNone) | 試験開始失敗として終了(待ち直しはしない) |
| SIGINT/SIGTERM | ウォームアップ・空き待ちのループも停止フラグを見て抜ける |
| SIGINT後のTraCI切断(`FatalTraCIError`) | Ctrl+CのSIGINTはSUMOにも届き、SUMOが先に終了する(S2の実機確認で判明、従来からの挙動)。停止フラグが立った後のTraCI切断は異常ではなく正常な停止として扱い、トレースバックを出さずに後始末して終了コード0で終える。停止要求なしのTraCI切断は従来どおり異常終了 |

- 現状の`main()`はシグナルハンドラを`load_world()`の後に登録し、`_stop_loop()`は`self.bridge_loop`を前提にしている。
  ゲート処理を`load_world()`と`run_bridge()`の間に入れるため、停止フラグを`InitializeInterface`側に持たせ、
  ゲート処理の前にハンドラが有効になるようにする(ウォームアップ無効時の動作は変えない)。
- launch側の`on_exit`の扱い(試験開始失敗でlaunch全体を止めるか)はS6で確認して決める。

### 2.9 ログ・記録(要件9)

```text
[SUMO WARMUP] start: warmup_time=100 s (2000 steps)
[SUMO WARMUP] t=10.0 s vehicles=9 persons=600 (wall 0.1 s)
[SUMO WARMUP] completed: t=100.0 s vehicles=72 persons=594 (wall 0.9 s)
[SUMO WARMUP] caught up: carla_spawned=72 sumo_only=0 pedestrians=594 (wall 3.2 s)
[EGO SPAWN CHECK] t=100.05 front=sumo:veh12/carla:456 clearance=8.4 m rear=sumo:veh3/carla:431 clearance=31.2 m overlap=none result=WAIT
[EGO SPAWN CHECK] t=107.40 front=sumo:veh12/carla:456 clearance=27.3 m rear=- overlap=none result=SAFE
[EGO SPAWN] t=107.40 spawn_point=(x, y, z, yaw) vehicles=70
```

- 判定ログは毎回出すと多いので、結果が変わったとき + 1秒(SUMO時間)ごとにINFO、それ以外はDEBUG。
- 記録項目: ウォームアップ時間、ウォームアップ終了時刻・車両数、EGO実スポーン時刻・位置、前後車ID・クリアランス、
  ネットワーク内車両数、SUMOの乱数シード(`.sumocfg`の`--seed`。未指定ならSUMOの既定値)、`.sumocfg`・ルートファイル。

---

## 3. 変更ファイル一覧

| ファイル | 変更内容 |
|---|---|
| `launch/autoware_carla_interface.launch.xml` | パラメータ4つ追加(argとparam) |
| `src/autoware_carla_interface/carla_ros.py` | パラメータ定義4つ追加 |
| `src/autoware_carla_interface/carla_autoware.py` | パラメータ読込・検査、`_spawn_ego_and_sensors()`切り出し、`run_ego_spawn_gate()`呼び出し、停止フラグ |
| `src/autoware_carla_interface/sumo_integration/ego_spawn_gate.py` | **新規**。状態遷移(WARMUP/WAIT/SPAWN)と`evaluate_spawn_gap()`、判定用の固定値 |
| `src/autoware_carla_interface/sumo_integration/simulation_synchronization.py` | `spawn_all_sumo_actors_in_carla()`追加(既存メソッドは無変更) |
| `src/autoware_carla_interface/sumo_integration/sumo_simulation.py` | `get_vehicle_ids()`/`get_person_ids()`等の読み出しメソッド追加(既存メソッドは無変更) |
| `test/sumo_warmup_params_test.py` | **新規**(S1)。起動時検査と、launch・`carla_ros.py`のパラメータ定義の一致 |
| `test/sumo_ego_spawn_gate_test.py` | **新規**。ギャップ判定の単体テスト(既存の`pedestrian_sync_stub_test.py`と同じく手動実行のスクリプト) |
| `test/sumo_warmup_catchup_stub_test.py` | **新規**(S3)。キャッチアップとゲートの順序のスタブテスト(偽のSUMO/CARLAで`spawn_all_sumo_actors_in_carla()`・`EgoSpawnGate`を検査) |
| `docs/SUMO-CARLA-Autoware_co-sim_起動手順.md` | パラメータ・スポーン地点の選び方・ルートファイルの条件・記録項目 |
| `sumo_integration/NOTICE.md` | vendorファイル(`simulation_synchronization.py`・`sumo_simulation.py`)への変更点を追記 |

---

## 4. 実装ステップ

各ステップ完了時に、ウォームアップ無効(既定)で既存の動作が変わっていないことを確認する。

### ブランチ運用

```text
feature/sumo_co-sim (最新版)
  └─ feat/sumo-warmup  ← Step S0〜S9 をここで実装・検証
        └─ (S9完了後) feature/sumo_co-sim へマージ
```

- コミットはステップ単位で分ける(コミットメッセージにStep番号を入れる。例: `feat(sumo-carla-autoware co-sim): ウォームアップ・EGO安全スポーン(Step S1)`)。
- 作業中に`feature/sumo_co-sim`が更新された場合は、`feat/sumo-warmup`へ取り込んでから検証を続ける。
- Vissim版(`feat/vissim-warmup`)の変更はこのブランチに取り込まない。
- マージ条件: Step S9の確認項目がすべて完了し、`test/`のスクリプトがすべて通ること。

### Step S0: 事前確認(コード変更なし)

1. CARLA側の車両なしでSUMOを連続して進めて、正常に進むか。
2. 1ステップの処理時間を計測し、ウォームアップにかかるwall-clockを見積もる。ステップごと/一気に進める方式を比較する。
3. 実際に使うルートファイルの車両タイプが、CARLAのブループリントまたは`vtypes.json`のvClassに対応しているか。
4. EGOのCARLA車種(`vehicle_type`)と主なNPC車種の`bounding_box`(車長・中心のずれ)と、SUMO車長との差。
5. 出発する車両タイプと、そのSUMO車長・vClass。
6. ウォームアップに使う時間の目安(車両数が安定するまでの時間)。
7. Autowareが、センサーデータが遅れて届き始めても正常に起動・初期化できるか → **Step S4の確認項目へ移す**
   (コード変更なしではEGOスポーンを遅らせられないため)。
8. `.sumocfg`に`<end>`を設定した場合、`<end>`到達後のTraCIの挙動(例外の種類)。

計測用ツール(パッケージにはインストールしない。ソースツリーから実行する):

| ツール | 対象 | 実行場所 |
|---|---|---|
| `tools/sumo_warmup_probe.py` | 1・2・5・6(8は`<end>`付きの`.sumocfg`で実行) | SUMOが動く機械(CARLA・Autowareは不要) |
| `tools/carla_bbox_probe.py` | 3・4 | Linux機(CARLAサーバーを起動しておく) |

```bash
# 1・2・5・6: SUMOを300秒ぶん進め、ステップ時間と車両数の推移、出発した車両タイプを記録
python3 tools/sumo_warmup_probe.py \
  --sumo-cfg /home/divp/CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg \
  --duration 300 --csv sumo_warmup_probe.csv

# 2: 一気に進める方式(simulationStep(300))の所要時間
python3 tools/sumo_warmup_probe.py \
  --sumo-cfg /home/divp/CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg \
  --duration 300 --mode jump

# 3・4: carlavtypes.rou.xmlの全車種について、CARLAの寸法とSUMO車長の差
python3 tools/carla_bbox_probe.py --all-from-vtypes \
  --sumo-vtypes /home/divp/CARLA/Co-Simulation/Sumo/examples/carlavtypes.rou.xml
```

- 交通流の安定時間の目安(`warmup hint`)は「最後の60秒の平均±10%に収まり続ける最初の時刻」という簡易な指標なので、CSVも確認する。

**完了条件**: 1〜6・8の結果を本計画書§6に記録し、方針に影響があれば計画を更新する。

### Step S1: パラメータ追加(動作変更なし) — **実装済み**(2026-10-01)

- launch arg / ROS param 4つを追加し、`carla_autoware.py`で読み込み、§2.2の起動時検査を追加する。
- `sumo_warmup_time=0`では何もしない。
- テスト: 起動時検査の異常系(検査関数を切り出してスクリプトで確認)。
- 実装内容:
  - 検査は`sumo_integration/ego_spawn_gate.py`の`validate_warmup_params()`(traci/carla非依存の純粋関数)。
    `InitializeInterface._check_sumo_warmup_params()`から呼ぶ。ウォームアップ無効時は`sumo_warmup_time ≥ 0`だけを検査する。
  - `sumo_warmup_time > 0`を指定した場合、S3で実装するまでは「未実装のため従来どおりすぐにEGOをスポーンする」旨のWARNINGを出す。
  - `test/sumo_warmup_params_test.py`: 検査の正常系・異常系、launch arg・param・`carla_ros.py`のパラメータ定義の一致
    (既定値・型、整数で指定したときに渡る型)。`python3 test/sumo_warmup_params_test.py`で実行。
- 実機確認(Linux機、2026-10-01): (1) 引数なしで従来どおり起動した。(2) `sumo_warmup_time:=100`のみ(`spawn_point`未指定)で、
  `ValueError: sumo_warmup_time > 0 requires spawn_point ...`によりSUMO接続前に停止した(期待どおり)。
- 補足(実機確認を受けて): 検査エラーがトレースバックに埋もれて分かりにくかったため、`main()`で`ValueError`を受けて
  `Error: invalid parameters: ...`の1行を出し、終了コード1で終わるようにした。ROSノードは終了前に後始末する。
  `use_sumo`と`use_traffic_manager`の併用エラーも同じ表示になる。

### Step S2: EGOスポーン処理の切り出し・停止フラグ(動作変更なし) — **実装済み**(2026-10-01)

- `load_world()`のEGOスポーン・センサー設定・Traffic Manager設定を`_spawn_ego_and_sensors()`へ移す。
- 停止フラグとシグナルハンドラの登録位置を§2.8のとおり変える。
- ウォームアップ無効時は`load_world()`から呼び、現状と同じ順序・同じ動作にする。
- 確認: ウォームアップ無効で実機co-simが従来どおり動くこと。Ctrl+Cで従来どおり終了すること。
- 実装内容:
  - `_spawn_ego_and_sensors(client)`: `load_world()`末尾の処理をそのまま移した。S2では常に`load_world()`の最後から呼ぶ。
  - 停止フラグ`InitializeInterface.stop_requested`: `_stop_loop()`(SIGINT/SIGTERMハンドラ)が立てる。
    `bridge_loop`がまだない場合(従来は`AttributeError`になっていた)はフラグだけを立てる。
    `run_bridge()`は開始前にフラグが立っていれば何もせずに戻る(その後`finally`の`_cleanup()`が従来どおり走る)。
  - シグナルハンドラの登録位置は変えない(`load_world()`の後、`try`の前)。S3のゲート処理はハンドラ登録後・`try`の中で
    `run_bridge()`の前に呼ぶので、ゲート中のCtrl+Cもフラグで止められ、`_cleanup()`も走る。
  - 単体テストは追加していない(`carla_autoware.py`はcarla・rclpyに依存し、切り出しのみで判定ロジックがないため)。実機で確認する。
- 実機確認(Linux機、2026-10-01):
  - 引数なしで従来どおり起動・co-simが動いた。`sumo_warmup_time:=100`のみで`Error: invalid parameters: ...`の1行で終了した。
    `sumo_warmup_time:=100 spawn_point:=...`で未実装のWARNINGが出て従来どおり起動した。
  - Ctrl+Cで`Cleaning up CARLA resources...`→`Cleanup complete.`まで走り、SUMOプロセス・CARLA上のアクター(vehicle/walker/sensor)は0になった。
  - 判明した点1: Ctrl+C後の`print()`がros2 launch下で失われていた → `main()`で標準出力を行バッファにした(`b23fe45d4`)。
    また`ros2 launch ... | tee`ではCtrl+Cで`tee`も終了してログが切れるため、ログ保存は`tee -i`を使う(S7で起動手順書に反映)。
  - 判明した点2(従来からの挙動): Ctrl+CのSIGINTはSUMOのプロセスにも届くため、SUMOが先に終了し、次の`traci.simulationStep()`で
    `FatalTraCIError: Connection closed by SUMO.` → `Error during bridge operation` → 後始末 → トレースバック・終了コード1になる。
    EGOのSUMO側の車両(`carla0`)の削除も`Connection already closed.`の警告になる(SUMOごと終了しているので実害はない)。
    ゲート処理中のCtrl+Cでも同じことが起きるため、S3で「停止要求後のTraCI切断は正常な停止として扱う」対処を入れる(§2.8)。

### Step S3: WARMUPとキャッチアップ — **実装済み**(2026-10-02)

- `ego_spawn_gate.py`にWARMUPループを実装する(§2.3)。
- `spawn_all_sumo_actors_in_carla()`と`SumoSimulation`の読み出しメソッドを実装する(§2.4)。
- この段階では、ウォームアップ後すぐにEGOをスポーンする(ギャップ判定なし)。
- テスト: `sumo_warmup_catchup_stub_test.py`(未登録の車両・歩行者だけがスポーンされる、CARLA由来の車両は対象外、
  ブループリントがない車両はunsubscribeされる、二重にスポーンしない)。
- 実装内容:
  - `SumoSimulation.get_vehicle_ids()`/`get_person_ids()`/`get_time()`(traciの薄いラッパー)。
  - `SimulationSynchronization.spawn_all_sumo_actors_in_carla()`: 戻り値は
    `{vehicles, vehicles_not_in_carla, pedestrians, pedestrians_not_in_carla}`の件数。
  - `ego_spawn_gate.EgoSpawnGate`: WARMUP(`sumo.tick()`のみ、10秒ごとに進捗ログ)→ キャッチアップ →
    **同期1ステップ(EGOなし)** → EGOスポーン。同期1ステップを挟むのは、キャッチアップでスポーンした車両は
    (既存のスポーンと同じく)`SPAWN_OFFSET_Z`=25 m上空に物理なしで置かれ、次の同期で道路上へ移されるため。
    挟まないと、CARLAの`try_spawn_actor`の重なり判定が効かないままEGOがスポーンされる。各段階の前に停止フラグを確認する。
  - `carla_autoware.py`: `sumo_warmup_time > 0`のときは`load_world()`でEGOをスポーンせず、`main()`で`run_bridge()`の前に
    `run_ego_spawn_gate()`を呼ぶ(シグナルハンドラ登録後・`try`の中)。S1の「未実装」WARNINGは削除。
  - EGOスポーン失敗(`request_new_actor()`が`None`)は`RuntimeError: failed to spawn EGO ...`にした
    (従来は次の行の`AttributeError`。ウォームアップ無効時も、失敗時のメッセージだけが変わる)。
  - §2.8のSIGINT後のTraCI切断: `is_sumo_disconnect_after_stop()`。停止要求後の`FatalTraCIError`は
    `Stopped: SUMO closed the TraCI connection after the stop request (...)`を出して後始末し、終了コード0。
    ウォームアップ中・通常ループ中のどちらにも効く。
  - テスト(`python3 test/sumo_warmup_catchup_stub_test.py`): キャッチアップ(未登録の車両・歩行者だけ、CARLA由来は対象外、
    ブループリントなしはunsubscribe、2回呼んでも二重にスポーンしない、vtypes.json未登録でもブループリントIDが一致すればスポーン)、
    ゲートの順序(WARMUP中はCARLAをtickしない、2000ステップ後に同期1ステップ→EGOスポーン、進捗ログ9行)、
    WARMUP中の停止要求。carla・lxmlがない環境では最小限の代用モジュールを入れて実行する(Linux機では本物を使う)。
  - 開発用PCでの確認(実物のSUMO + 偽のCARLA、Town01・変更後の需要、`sumo_warmup_time=600`): WARMUPのwall-clockは約6秒、
    キャッチアップで40台すべてをCARLA側に登録、その後10秒間の通常同期で出発・到着も反映され、未登録の車両は0台。

### Step S4: S3の実機確認 — **一部確認済み**(2026-10-02)

- 確認: ウォームアップ後、CARLA上にSUMO車両・歩行者が揃って現れること。EGOがSUMOへ登録され、
  通常co-simに移ること。ROS時刻が0付近から始まること。キャッチアップにかかる時間。
  Autowareが、センサーデータがウォームアップ分遅れて届き始めても正常に起動・初期化できること(S0 #7から移動)。
- 実機確認結果(Linux機、Town01・変更後の需要、`sumo_warmup_time:=600`、`sumo_gui:=true`):
  - ログ: start → 10秒ごとの進捗 → `completed: t=600.0 s vehicles=40 (wall 18.4 s)` →
    `caught up: carla_spawned=40 sumo_only=0` → `[EGO SPAWN] t=600.05 s` の順に出た。車両数の推移は開発用PC(実物のSUMO)と1台単位で一致
    (SUMOの再現性を確認)。wall-clockが計測ツールの見積もり(約10秒)より長いのは`sumo_gui:=true`の描画のためと思われる。
  - スポーン地点#36(`199.95,330.46,0.30,0.0,0.0,-0.0`、#37の対向車線でSUMO車両が通らない): ウォームアップありでも
    従来どおりRVizにEGOが表示され、目的地設定・Autoで走行した → **センサー開始がウォームアップ分遅れてもAutowareは正常に起動・初期化できる**(S0 #7)。
  - スポーン地点#37(`199.95,326.97,0.30,0.0,0.0,180.0`、`6.0.00`、600台/h): EGOが目的地なしで約3 km/hで動き続け、
    RVizではEGOが実際と違う位置に表示されて走行できなかった。ウォームアップなしの#37は従来どおり走行できる。
    開発用PCの実物のSUMOで再現すると、600秒時点のスポーン直後にトラック(`in1_B.14`、carlacola)がEGOの真後ろ(SUMOの最小車間2.5 m)にいて、
    後続も含めて停止して並ぶ。物理なしで毎ステップ位置を書き換えられるSUMO車両がEGOに接触して押し続け、
    EGOが止まらないためAutowareの自己位置推定の初期化(停止が条件)がうまくいかなかったと考えられる。
    → ギャップ判定(S5/S6)で防ぐ対象そのもの。S5/S6の実機確認で#37を「空き待ちが必要な地点」として使う。
  - 未確認: バス・トレーラー(`fusorosa`/`european_hgv`)のCARLAスポーン、ROS時刻(`/clock`)、ウォームアップ中・通常走行中のCtrl+C。


### Step S5: ギャップ判定ロジック(純粋関数 + 単体テスト)

- `evaluate_spawn_gap()`を実装する(§2.6.1〜2.6.3)。
- テストケース(`sumo_ego_spawn_gate_test.py`):
  - 前後に車両なし → SAFE
  - 前方車のクリアランスが必要距離未満 / ちょうど / 超過
  - 後方車も同様
  - 隣の車線の車両は無視される
  - 対向車線の車両は無視される
  - EGO予定位置と重なる車両(隣の車線にはみ出し)・歩行者 → UNSAFE
  - CARLAにいないSUMO車両(SUMOの車長で判定)
  - 探索範囲外の車両は無視される

### Step S6: WAIT_FOR_SAFE_GAPとタイムアウト

- S3の「すぐにスポーン」を、判定 → NGなら同期1ステップ → 再判定、に置き換える(§2.6.5)。
- タイムアウト・SIGINT・SUMO異常終了時の終了処理(§2.8)。
- 確認: 交通量の多い地点で「待つ → 空いたらスポーン」、上限を短くして「タイムアウトで終了」。

### Step S7: ログ・記録・起動手順書

- §2.9のログを実装する。
- 起動手順書に、パラメータ、スポーン地点の選び方(直線区間)、ルートファイルの条件(継続的な需要、`<end>`)、
  `tls_manager=carla`では使えないこと、記録項目を追記する。

### Step S8(オプション): 長いほうの車長で判定

- 運用して、必要距離の上乗せでは不十分な場合に実装する(§2.6.4)。

### Step S9: 実機検証

| # | 確認内容 | 期待結果 |
|---|---|---|
| 1 | ウォームアップ無効(既定) | 現状と完全に同じ動作(EGOスポーン時刻、ログ) |
| 2 | ウォームアップ100秒 | wall-clockがウォームアップ時間より十分短い。終了時の車両数がS0の計測どおり |
| 3 | キャッチアップ | CARLA上の車両数 ≒ SUMO車両数(差分は`sumo_only`としてログに出る)。歩行者も同様 |
| 4 | 空きありの地点 | ウォームアップ直後にSAFEでスポーン |
| 5 | 空きなしの地点 | WAITが続き、空いた時点でスポーン。スポーン直後に衝突しない(`--collision.check-junctions`のSUMO側の衝突警告も確認) |
| 6 | タイムアウト | 上限でエラー終了する。SUMOも閉じる |
| 7 | ROS時刻 | `/clock`とセンサーtimestampが0付近から単調増加。Autowareが正常に動く |
| 8 | 信号同期(`tls_manager=sumo`) | ウォームアップ後もSUMOとCARLAの信号が一致している |
| 9 | `tls_manager=carla` + ウォームアップ | 起動時エラーになる |
| 10 | 再現性 | 同じ`.sumocfg`・同じパラメータで2回実行し、EGOスポーン時刻・前後車IDが一致するか記録する |

---

## 5. 影響を受けない(変えない)もの

- 通常co-simのループ(`SensorLoop._tick_sensor()`、`run_bridge()`)
- `sync_sumo_to_carla()`/`sync_carla_to_sumo()`の既存処理
- SUMO/CARLAのSTEP周期(`fixed_delta_seconds`)、同期モード、信号同期、歩行者同期
- ROS timestamp・`/clock`の計算方法
- SUMOの起動オプション(`SumoSimulation.__init__()`)

---

## 6. 未確認事項・S0の記録欄

| # | 項目 | 状態 | 結果 |
|---|---|---|---|
| 1 | CARLA側の車両なしでSUMOが進むか | **確認済み**(開発用PC、2026-09-30) | 1500秒まで正常に進んだ。車両が0台になった後も`simulationStep()`を受け付ける |
| 2 | 1ステップの処理時間 | **確認済み**(Linux機・開発用PC、2026-09-30) | 下記6-1。一気に進める方式(`--mode jump`)はLinux機では未計測(開発用PCで差がなかったため方針は変えない) |
| 3 | 車両タイプとブループリントの対応 | **確認済み**(Town01) | 変更後の需要で出発する22車種はすべてCARLAのブループリントID。`european_hgv`・`fusorosa`は`vtypes.json`に未登録だが、ブループリントIDの完全一致でスポーンされる見込み(§1.7、S4で確認) |
| 4 | EGO・NPC車種の`bounding_box`とSUMO車長の差 | **確認済み**(Linux機、2026-09-30) | 下記6-3。車長差は最大0.19 m、`bounding_box`中心のずれは最大0.057 mで、判定式の補正は不要 |
| 5 | 出発する車両タイプ | **確認済み**(Town01) | 下記6-2 |
| 6 | 交通流が安定するまでの時間 | **確認済み**(Town01、変更後の需要は開発用PCのみ) | 変更後の需要では400秒で約40台に達した後もゆっくり増え続ける(§1.7、下記6-4)。変更前の需要は安定しない(下記6-1) |
| 7 | Autowareがセンサー開始の遅れに耐えるか | Step S4へ移動 | |
| 8 | `<end>`到達後のTraCIの挙動 | 未確認 | Town01の`.sumocfg`には`<end>`がないため、現状の運用では起きない |

**6-1. Town01・変更前の需要(`CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg`、2026-09-30時点)の計測結果**
(`--step-length 0.05`、ヘッドレス。車両数・歩行者数はLinux機と開発用PCで完全に一致した)

| SUMO時刻 | 20 s | 60 s | 100 s | 140 s | 200 s | 300 s | 400 s以降 |
|---|---|---|---|---|---|---|---|
| 車両数 | 19 | 54 | 72 | 60 | 18 | 4 | 0 |
| 歩行者数 | 600 | 596 | 594 | 581 | 579 | 556 | 減少(1500 sで355) |

| 計測した機械 | 300秒ぶんのwall-clock | 実時間比 | `simulationStep()` 平均 / p50 / p95 / 最大 | ウォームアップ100秒の見積もり |
|---|---|---|---|---|
| Linux機(DIVP-WS03、実機) | 4.82 s | 62倍 | 0.61 / 0.47 / 1.08 / 8.19 ms | 1.6 s |
| 開発用PC(Windows 11、SUMO 1.26.0) | 2.66 s | 113倍 | 0.35 / 0.28 / 0.74 / 3.94 ms | 0.9 s |

- → ウォームアップ100秒は実機で約1.6秒。1ステップは車両数が多いほど重く、最大の100秒付近で平均1.3 ms。
- `simulationStep(300)`で一気に進めた場合は1.7秒で、ステップごとの方式と大差ない(§1.3)。
- 車両の出発は0〜99秒の100台だけ。歩行者は開始20秒で約600人に達し、その後ゆっくり減る。

**6-2. Town01で出発する車両タイプ(100台、27車種)**

| vClass | 車種(SUMO車長) |
|---|---|
| passenger | audi.a2 (3.72)、audi.tt (4.15)、bmw.grandtourer (4.64)、chevrolet.impala (5.37)、citroen.c3 (3.98)、ford.mustang (4.90)、jeep.wrangler_rubicon (3.87)、lincoln.mkz_2017 (4.90)、mercedes.coupe (5.04)、mini.cooper_s (3.80)、nissan.micra (3.67)、nissan.patrol (4.52)、seat.leon (4.21)、volkswagen.t2 (4.47) |
| evehicle | audi.etron (4.89)、micro.microlino (2.20)、tesla.cybertruck (6.36)、tesla.model3 (4.81)、toyota.prius (4.54) |
| authority | dodge.charger_police (5.01) |
| truck | carlamotors.carlacola (5.20) |
| motorcycle | harley-davidson.low_rider (2.35)、kawasaki.ninja (2.04)、yamaha.yzf (2.19) |
| bicycle | bh.crossbike (1.51)、diamondback.century (1.66)、gazelle.omafiets (1.84) |

- 最長はtesla.cybertruck(6.36 m)。Vissim版のトレーラー(16.5 m)のような大きな車長差の原因はない。

**6-3. CARLAの`bounding_box`とSUMO車長の差**(Linux機、`carla_bbox_probe.py --all-from-vtypes`、`carlavtypes.rou.xml`の27車種)

| 項目 | 結果 |
|---|---|
| EGO(`vehicle.toyota.prius`) | CARLA 長さ4.51 m・幅2.01 m・高さ1.52 m、中心のずれ(x) 0.002 m、SUMO車長4.54 m(差 −0.02 m) |
| 車長差(CARLA − SUMO)が大きい車種 | ford.mustang −0.19 m、tesla.cybertruck −0.09 m、nissan.patrol +0.08 m。ほかの24車種は±0.04 m以内 |
| `bounding_box`中心のx方向のずれ | 最大でnissan.patrolの−0.057 m、次いでford.mustang 0.032 m・tesla.model3 0.029 m。ほかは0.025 m以内 |
| `bounding_box`中心のz方向のずれ | 車高の約半分(箱の中心が地面から車高/2の位置にある)。平面の判定には影響しない |

- → **判定式に中心のずれの補正は入れない**。車長差も必要距離(既定20 m)に比べて無視できるため、
  Town01ではStep S8(長いほうの車長で判定)は不要。

---

**6-4. Town01・変更後の需要の計測結果**(開発用PC、`--duration 1800`、2026-10-01)

| SUMO時刻の区間 | 0〜120 s | 120〜240 s | 240〜360 s | 360〜600 s | 600〜900 s | 900〜1200 s | 1200〜1500 s | 1500〜1800 s |
|---|---|---|---|---|---|---|---|---|
| 車両数 平均(最小〜最大) | 13(0〜26) | 30(26〜35) | 28(22〜35) | 39(33〜46) | 44(39〜50) | 51(41〜60) | 50(43〜60) | 65(53〜81) |

- 1800秒ぶんのwall-clockは18.8秒(実時間の約96倍)。ウォームアップ600秒なら開発用PCで約4秒(Linux機ではその約1.8倍の見込み、6-1の比)。
- 車種別の出発台数(900秒時点、283台): carlacola 39、fusorosa 19、seat.leon 20、audi.etron 18、lincoln.mkz 15 ほか。
  `carlavtypes.rou.xml`の車長はCARLAの寸法に合わせて更新されている(例: audi.a2 3.71 m、ford.mustang 4.72 m)。

---

## 7. 将来の拡張候補

- 速度に応じた必要距離(`後方車速度 × 時間ギャップ + 最小距離`)
- SUMOの車線情報(`convertRoad`・レーン上の車両一覧)を使った判定(曲線区間でのスポーン)
- 複数のスポーン候補地点から、最初に空いた地点を選ぶ
- Vissim版との共通化(判定の純粋関数は同じ形にしておく)
