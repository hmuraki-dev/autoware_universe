# Vissim(Windows)–CARLA–Autoware ウォームアップ・EGO安全スポーン 実装計画

作成日: 2026-09-30
版: v1.0
対象ブランチ:
- 本リポジトリ(`autoware_universe`): **`feat/vissim-warmup`**(作業ブランチ)
  - `feature/vissim_windows_co-sim`の最新版から作成し、本計画の実装はすべてこのブランチで行う。
  - Step V9(実機検証)の完了後、`feature/vissim_windows_co-sim`へマージする。
- CARLAリポジトリ(`C:\Users\hirokazu.muraki.bp\src\CARLA`、Linux機では`/home/divp/CARLA`):
  `feature/vissim_windows`(本計画では**無改修**の見込み、§1.6)

前提ドキュメント:
- `Vissim_CARLA_ウォームアップ_EGO安全スポーン_引き継ぎ.md`(OneDrive `doc/20260929_ウームアップ/`。要件の出典)
- `20260929_VissimウォームアップとEGO安全スポーン.pptx`(同フォルダ。チーム共有資料)
- `docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`(同期ループ・auto-adoptの元設計)
- `docs/Vissim_CARLA_Autoware_シミュレーション期間管理_実装計画_v1.0.md`(`end_tick`/`simPeriod`。本計画で拡張する)
- `docs/CARLA_車両寸法一覧.md`(CARLA側の車長の根拠)
- `docs/Vissim_車両寸法一覧.md`(お台場`odaiba_v015.inpx`の値。参考扱い。Town01の値は§6 3-3)

車両タイプ・車長は、**現状Town01(2026-10-05変更版の`.inpx`)を正とする**(§6 3-2・3-4)。

---

## 0. 背景と要件

### 0.1 現状の問題

- Vissimは開始直後、Vehicle Inputから車両が順次流入するため、しばらくは想定した交通流になっていない。
- 現状はco-sim開始と同時にEGOをスポーンするため、交通流が形成される前から検証が始まってしまう。
- 先にVissimだけを進めて交通流を作ると、今度はEGOのスポーン予定位置付近にVissim車両がいる可能性があり、
  スポーン直後の衝突・重複や、Vissim側の追従の不自然さが起こりうる。

### 0.2 要件(引き継ぎ資料・チーム共有資料・2026-09-29〜30の検討より)

| # | 項目 | 決定 |
|---|---|---|
| 1 | 全体の流れ | `WARMUP → WAIT_FOR_SAFE_GAP → SPAWN_EGO → NORMAL_COSIM` の4状態 |
| 2 | ウォームアップ | Vissimだけを所定時間進める。CARLA/Autowareとの同期は行わない。可能な限り高速に進める |
| 3 | ギャップ判定の基準 | **CARLA基準**。位置はCARLA上の車両中心、車長はCARLA車両の`bounding_box`から求める |
| 4 | 車間距離の式 | `中心間距離 − 前の車の車長/2 − 後ろの車の車長/2`(前方・後方それぞれ) |
| 5 | Vissimとの車長差 | 前方・後方の必要距離(パラメータ)を長めに設定して吸収する。より安全にする場合は両者の長いほうの車長を使う(オプション、§2.6.4) |
| 6 | 空きがない場合 | Vissimを1ステップ進めて再判定する。交通流には手を加えない(引き継ぎ資料の案A) |
| 7 | タイムアウト | 待ち時間の上限を超えたら**試験開始失敗**として終了する。無理にスポーンしない |
| 8 | 外部指定する設定値 | ウォームアップ時間・前方必要距離・後方必要距離・待ち時間上限 |
| 9 | ログ | 判定のたびに結果を出す。再現性のための情報を記録する |
| 10 | 既存処理 | 通常のVissim–CARLA同期処理(STEP周期、同期モード、信号同期、車両同期、Autoware起動、timestamp)は変えない。**ウォームアップ無効時(既定)は現状と完全に同じ動作** |

### 0.3 スコープ外

- 車両クラスによるブループリント変換(検討のみ、実装予定なし)
- トレーラー連結車のCARLAモデル化(CARLA標準に存在しない。§1.5)
- ウォームアップ中からスポーン地点を空ける方式(案B)、周辺車両を削除する方式(案C)
- 速度に応じて必要距離を変える方式(将来の拡張候補、§7)

---

## 1. 前提調査で判明した事実

### 1.1 現在の起動・ループ構造(`carla_autoware.py`)

- `InitializeInterface.load_world()`(`carla_autoware.py:288`)で、CARLAのsync mode設定 →
  `_init_vissim_integration()` → **EGOスポーン**(`CarlaDataProvider.request_new_actor()`、:318)→
  センサー設定、の順に実行する。つまり**EGOはループ開始前にスポーン済み**。
- `run_bridge()`(:330)のループは`max_real_delta_seconds`でwall-clockのペースを合わせ、毎回
  `SensorLoop._tick_sensor()`(:77)を呼ぶ。1回の中身は次のとおり。
  1. `GameTime.on_carla_tick()` / `CarlaDataProvider.on_carla_tick()`
  2. `self.sensor()`(ROS publish。`/clock`もここで出る)→ EGOへ制御を適用
  3. `vissim_sync.sync_vissim_to_carla()`(内部で`vissim.tick()` = Vissimが1ステップ進む)
  4. `world.tick()`(CARLAが1ステップ進む)
  5. `vissim_sync.sync_carla_to_vissim()`(EGOを含むCARLA車両をVissimへ反映。auto-adopt)
  6. `_check_vissim_stop_conditions()`(期間経過・連続失敗で停止)

### 1.2 シミュレーション時刻

- ROSの`/clock`とセンサーのtimestampは`GameTime.get_time()`(`carla_ros.py:872-878`)。
  `GameTime`は**最初に`on_carla_tick()`が呼ばれた時点を0**として、そこからの経過を積算する
  (`modules/carla_data_provider.py:842`)。
- Vissimの時刻はROS側へは一切流れていない。Vissim側の経過は`PTVVissimSimulation.tick_count`で数えるだけ。
- → **Vissimだけ先に100秒進めても、ROS/Autowareの時刻には影響しない**。ウォームアップ・空き待ち中に
  `GameTime.on_carla_tick()`を呼ばなければ、ROS時刻は通常co-sim開始時点から0で始まる
  (引き継ぎ資料§18の懸念は、この構造では問題にならない)。
- ただし**期間管理(`end_tick`)はVissimのtick数で判定している**ため、ウォームアップ分を考慮しないと
  検証時間が短くなる(§2.5で対処)。

### 1.3 Vissimの進み方とウォームアップの高速化

- Vissimはco-simのtickごとに1ステップ進むロックステップ(期間管理計画§1.1)。
- 現在のペース合わせ(`max_real_delta_seconds`のsleep)は`run_bridge()`のループにだけある。
  → ウォームアップを`run_bridge()`の外の専用ループで`vissim.tick()`だけ回せば、sleepもCARLA tickも
  挟まないので、**アダプタとのRPC往復の速さでVissimが進む**。
- 実際の速さ(1 tickあたりの往復時間)は未計測。V0で計測する。

### 1.4 Vissim車両のCARLAへの反映(`simulation_synchronization.py`)

- `sync_vissim_to_carla()`はVissim車両を**「このtickで新たに現れた車両」(`vissim.spawned_vehicles`)だけ**
  CARLAへスポーンする(:196-206)。`spawned_vehicles`は`vissim.tick()`のたびに前回との差分で上書きされる
  (`vissim_simulation.py:668-672`)。
  → **ウォームアップ中に`vissim.tick()`だけ回すと、その間に現れた車両は二度とCARLAにスポーンされない**。
  ウォームアップ終了時に「今いる全車両」をCARLAへスポーンする処理(キャッチアップ)が必要(§2.4)。
  歩行者(`spawned_pedestrians`)も同じ。
- CARLAへのスポーンに失敗した車両(vtypes.jsonに未登録の車両タイプ、スポーン位置の干渉)は、
  `vissim2carla_ids`に入らず再試行もされない(:200-206、`bridge_helper.py:215-218`)。
  → ギャップ判定では、**CARLAにいないVissim車両も見落とさない**ようにする(§2.6.3)。
- Vissim車両の位置は前端基準。ブリッジはCARLA車両の`extent.x`を使って中心基準へ変換している
  (`bridge_helper.py:83-87`)。そのため**前端はVissimとCARLAで一致し、車長差は後端のずれとして現れる**。

### 1.5 車長

- DSIの`VISSIM_Veh_Data`に車長の項目はない(`DrivingSimulatorProxy_windows.h:77-101`)。
  CARLA上の車長は`actor.bounding_box.extent.x × 2`で車両ごとに取得できる。
- CARLA車種はvtypes.jsonの候補からランダムに選ばれるため、同じVissim車両タイプでも車長が変わる。
- Town01で最長のVissim車両は300(バス、12.14 m)。220(トレーラー)の3Dモデルはトラクターのみ(5.96 m)。
  CARLAで最長の車両はバス(10.27 m)で、トレーラー連結モデルはCARLA標準にない。
- 車長差が最も大きいのは210(Vissim 7.93 m、CARLA 5.20 m、差+2.73 m)。詳細は§6 3-4。
- vtypes.jsonは2026-10-05に変更され(コミット`7438613f6`)、Town01の車両入力で使う車両タイプ
  (100 / 210 / 220 / 300 / 700)はすべて登録済み。未登録タイプはCARLAにスポーンされない(§6 3-1)。

### 1.6 Windows側アダプタ

- ウォームアップ中のtickは「spawn/update/destroyが空のtick」であり、既存のプロトコルでそのまま送れる。
  現状でもループ最初のtick(EGOのauto-adopt前)は空のtickであり、動作実績がある。
- DSIの可視半径は0(無制限、`PTV-Vissim_windows/constants.py:27`)なので、EGOがいなくても全車両が返る。
- → **Windows側・RPCプロトコルは無改修**で実現できる見込み。

---

## 2. 設計

### 2.1 全体の流れ

```text
load_world()
  ├─ CARLA設定 / _init_vissim_integration()           (既存)
  ├─ [warmup無効] EGOスポーン + センサー設定            (既存のまま)
  └─ [warmup有効] EGOスポーンは後回し

[warmup有効時のみ] run_ego_spawn_gate()   ← 新規。run_bridge()の前に呼ぶ
  ├─ WARMUP            : vissim.tick() のみを vissim_warmup_time 秒ぶん。sleepなし
  ├─ (キャッチアップ)  : 今いるVissim車両・歩行者をCARLAへ一括スポーン → world.tick()
  ├─ WAIT_FOR_SAFE_GAP : ギャップ判定 → NGなら「通常の同期1ステップ(EGOなし)」→ 再判定
  │                      上限時間を超えたら「試験開始失敗」で終了
  └─ SPAWN_EGO         : EGOスポーン + センサー設定 + end_tick確定

run_bridge()                                       (既存のまま = NORMAL_COSIM)
```

- 状態遷移はco-simのループ本体(`SensorLoop._tick_sensor()`)に入れず、**`run_bridge()`の前段の独立した
  処理**にする。通常co-simのコードパスには手を入れない(要件10)。
- 「通常の同期1ステップ(EGOなし)」は、既存の`sync_vissim_to_carla()` → `world.tick()` →
  `sync_carla_to_vissim()`をそのまま呼ぶ。`GameTime`/`CarlaDataProvider.on_carla_tick()`と
  `sensor()`は呼ばない(ROS時刻を進めない・publishしない)。

### 2.2 パラメータ

launch arg / ROS paramとして追加する(`autoware_carla_interface.launch.xml`の2つのnode定義の両方。
`test/vissim_launch_params_test.py`で一致を検査)。

| 名前 | 型 | 既定値 | 説明 |
|---|---|---|---|
| `vissim_warmup_time` | int(秒) | `0` | Vissimだけを先に進める時間。**0 = 無効(現状と同じ動作)**。0以上必須 |
| `ego_spawn_front_margin` | double(m) | `20.0` | EGO前端と前方車後端の必要距離 |
| `ego_spawn_rear_margin` | double(m) | `20.0` | EGO後端と後方車前端の必要距離 |
| `ego_spawn_wait_timeout` | int(秒) | `60` | 空き待ちの上限。超えたら試験開始失敗。1以上必須 |

起動時の検査(`_check_vissim_sim_period_params()`に追加):
- `vissim_warmup_time > 0`は`use_vissim=True`のときだけ有効。
- `vissim_warmup_time > 0`のときは`spawn_point`の指定を必須にする(ランダムスポーンでは判定する位置が決まらない)。
- `vissim_warmup_time + ego_spawn_wait_timeout + vissim_sim_period + 余裕10秒` がVissimの最大期間以下であること(§2.5)。

判定用の固定値(`constants.py`に置く。launch argにはしない):

| 名前 | 値(案) | 用途 |
|---|---|---|
| `EGO_SPAWN_SEARCH_RANGE_M` | `100.0` | 前後車を探す縦方向の範囲 |
| `EGO_SPAWN_HEADING_TOLERANCE_DEG` | `45.0` | 同一進行方向とみなす向きの差 |
| `EGO_SPAWN_UNKNOWN_VEHICLE_LENGTH_M` | `12.2` | CARLAにいないVissim車両の仮の車長(Town01で最長のVissim車両、300: バス 12.14 m) |

### 2.3 WARMUP

- `vissim.tick()`だけを`vissim_warmup_time × sim_res`回呼ぶ。CARLAはtickしない。sleepしない。
- tickの失敗は既存と同じく`consecutive_failures`で数え、`vissim_max_consecutive_failures`に達したら終了する。
- 進捗ログを10秒(Vissim時間)ごとに出す(車両数・経過・wall-clock)。

### 2.4 キャッチアップ(WARMUP → WAIT_FOR_SAFE_GAP)

- `SimulationSynchronization`に`spawn_all_vissim_actors_in_carla()`を追加する。
  - 今いる全Vissim車両のうち、`vissim2carla_ids`にも`carla2vissim_ids`の値にもない車両を、
    既存の`sync_vissim_to_carla()`のスポーン処理と同じ手順(`get_carla_blueprint` → `get_carla_transform` →
    `carla.spawn_actor`)でスポーンする。歩行者も同様。
  - 位置は前端基準のままスポーンされるが、次の同期ステップで既存の更新処理が中心基準へ補正する(既存と同じ挙動)。
- 既存の`sync_vissim_to_carla()`のスポーン条件(差分のみ)は**変えない**。変えると、スポーンに失敗した車両の
  再試行が毎tick発生し、通常co-simの挙動とログが変わるため。
- 呼んだ後に`world.tick()` → `carla.update_actor_diff()`を1回行い、CARLA側の差分管理を最新にする
  (キャッチアップでスポーンした車両がEGOと誤認されてVissimへauto-adoptされないよう、
  `vissim2carla_ids`登録後に差分更新する)。

### 2.5 シミュレーション期間

- `.inpx`に書く`simPeriod`: 現状の`vissim_sim_period + 10`を
  **`vissim_warmup_time + ego_spawn_wait_timeout + vissim_sim_period + 10`** に変える
  (`get_vissim_sim_params()`の引数を追加)。空き待ちが上限まで伸びてもVissimが先に終わらないようにする。
- `end_tick`: 現状は起動時に`vissim_sim_period × sim_res`で固定。これを、
  **EGOスポーン時点の`tick_count` + `vissim_sim_period × sim_res`** に変える(SPAWN_EGOで確定)。
  ウォームアップ無効時はスポーン時点の`tick_count`が0なので、現状と同じ値になる。
- 検証時間(EGOが走る時間)は、ウォームアップ・空き待ちの長さによらず常に`vissim_sim_period`秒になる。

### 2.6 ギャップ判定(WAIT_FOR_SAFE_GAP)

CARLAに依存しない純粋関数`evaluate_spawn_gap()`として`vissim_integration/ego_spawn_gate.py`に実装し、
単体テストする。入力は「EGO予定位置・向き・車長」と「周辺車両のリスト(ID、中心位置、向き、車長、出所)」。

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
- CARLAのroad_id/lane_idによる判定は、道路の区切りをまたぐ車両を取りこぼすため採用しない。

#### 2.6.2 クリアランスと判定

```text
front_clearance = s_front − L_ego/2 − L_front/2
rear_clearance  = |s_rear| − L_ego/2 − L_rear/2
```

- `L`はCARLA上の車長(`bounding_box.extent.x × 2`)。`L_ego`はスポーン予定のブループリントの寸法
  (スポーン前に取得できないため、V0で車種ごとの値を確認し、`vehicle_type`から引く。§6)。
- `front_clearance ≥ ego_spawn_front_margin` かつ `rear_clearance ≥ ego_spawn_rear_margin` ならSAFE。
- 前方車(後方車)がいない側は条件を満たすとみなす。
- **重複チェック**: 車線によらず、EGO予定位置の外形と重なる車両が1台でもあればUNSAFE
  (隣の車線にはみ出している車両や、交差点内の車両の見落とし防止)。

#### 2.6.3 CARLAにいないVissim車両

- vtypes.json未登録やスポーン失敗でCARLAにいないVissim車両も、Vissimの位置(前端)から判定に含める。
- 中心位置は「前端 − 仮の車長/2」とし、車長は`EGO_SPAWN_UNKNOWN_VEHICLE_LENGTH_M`(12.2 m)を使う(安全側)。
  ネットワークを変えて、より長いVissim車両が加わった場合は値を見直す。
- ログでは`source=vissim_only`として区別する。

#### 2.6.4 Vissimとの車長差(要件5)

- 基本は**必要距離の設定で吸収**する(運用で`ego_spawn_front_margin`/`ego_spawn_rear_margin`に上乗せする)。
  前方で効くのは前方車の車長差、後方で効くのはEGOの車長差(前端がVissimとCARLAで一致するため)。
- オプション(Step V8、必要になったら): Vissim車両タイプ → 車長の対応表`data/vissim_vehicle_lengths.json`を用意し、
  `L = max(CARLAの車長, 表の車長)`で計算する。表にないタイプはCARLAの車長を使う。

#### 2.6.5 待ちの1ステップ

- NGなら「通常の同期1ステップ(EGOなし)」を1回実行し、再判定する。
- ペース合わせのsleepは入れない(ウォームアップと同様、できるだけ速く)。
- `ego_spawn_wait_timeout × sim_res`ステップ待ってもSAFEにならなければ、試験開始失敗として終了する
  (§2.8)。

### 2.7 SPAWN_EGO → NORMAL_COSIM

- 既存の`load_world()`にあるEGOスポーン・センサー設定・Traffic Manager設定を`_spawn_ego_and_sensors()`に切り出し、
  - ウォームアップ無効時: 現状どおり`load_world()`から呼ぶ
  - ウォームアップ有効時: SAFE判定の直後に呼ぶ
- EGOのVissimへの登録は、既存のauto-adopt(次の`sync_carla_to_vissim()`)にそのまま任せる。
- `end_tick`を確定する(§2.5)。
- `run_bridge()`へ進む。`GameTime`はここで初めて`on_carla_tick()`されるので、ROS時刻は0付近から始まる。

### 2.8 終了とエラー

| 事象 | 動作 |
|---|---|
| 空き待ちの上限超過 | `Error: no safe gap found within ego_spawn_wait_timeout ...`を出し、試験開始失敗として終了(終了コード非0)。既存の`_cleanup()`で後始末し、launchは`on_exit="shutdown"`で全体停止 |
| ウォームアップ中・空き待ち中のtick連続失敗 | 既存と同じ上限で終了 |
| EGOスポーン失敗(`request_new_actor`がNone) | 試験開始失敗として終了(待ち直しはしない) |
| SIGINT/SIGTERM | ウォームアップ・空き待ちのループも停止フラグを見て抜ける |

### 2.9 ログ・記録(要件9)

```text
[VISSIM WARMUP] start: warmup_time=100 s (2000 ticks)
[VISSIM WARMUP] t=10.0 s vehicles=12 (wall 2.1 s)
[VISSIM WARMUP] completed: t=100.0 s vehicles=85 (wall 19.8 s)
[VISSIM WARMUP] caught up: carla_spawned=83 vissim_only=2 pedestrians=4
[EGO SPAWN CHECK] t=100.05 front=vissim:123/carla:456 clearance=8.4 m rear=vissim:98/carla:431 clearance=31.2 m overlap=none result=WAIT
[EGO SPAWN CHECK] t=107.40 front=vissim:125/carla:470 clearance=27.3 m rear=vissim:98/carla:431 clearance=24.1 m overlap=none result=SAFE
[EGO SPAWN] t=107.40 spawn_point=(x, y, z, yaw) vehicles=87 end_tick=14148
```

- 判定ログは毎回出すと多いので、結果が変わったとき + 1秒(Vissim時間)ごとにINFO、それ以外はDEBUG。
- 記録項目: ウォームアップ時間、ウォームアップ終了時刻・車両数、EGO実スポーン時刻・位置、前後車ID・クリアランス、
  ネットワーク内車両数。Vissim Random Seedは`.inpx`の値(Windows側)なので、Linux側では記録できない
  → 起動手順書で`.inpx`を記録対象にする。

---

## 3. 変更ファイル一覧

| ファイル | 変更内容 |
|---|---|
| `launch/autoware_carla_interface.launch.xml` | パラメータ4つ追加(2つのnode定義の両方) |
| `src/autoware_carla_interface/carla_autoware.py` | パラメータ読込・検査、`_spawn_ego_and_sensors()`切り出し、`run_ego_spawn_gate()`呼び出し |
| `src/autoware_carla_interface/vissim_integration/ego_spawn_gate.py` | **新規**。状態遷移(WARMUP/WAIT/SPAWN)と`evaluate_spawn_gap()` |
| `src/autoware_carla_interface/vissim_integration/simulation_synchronization.py` | `spawn_all_vissim_actors_in_carla()`追加(既存メソッドは無変更) |
| `src/autoware_carla_interface/vissim_integration/vissim_simulation.py` | `get_vissim_sim_params()`の期間計算、`end_tick`の確定を後から行えるようにする |
| `src/autoware_carla_interface/vissim_integration/constants.py` | 判定用の固定値 |
| `test/vissim_ego_spawn_gate_test.py` | **新規**。ギャップ判定の単体テスト |
| `test/vissim_sim_period_test.py` / `test/vissim_launch_params_test.py` | 期間計算・パラメータ追加に追従 |
| `docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md` | パラメータ・スポーン地点の選び方・記録項目 |
| `vissim_integration/NOTICE.md` | vendorファイル(`simulation_synchronization.py`等)への変更点を追記 |

---

## 4. 実装ステップ

各ステップ完了時に、ウォームアップ無効(既定)で既存の動作が変わっていないことを確認する。

### ブランチ運用

```text
feature/vissim_windows_co-sim (最新版)
  └─ feat/vissim-warmup  ← Step V0〜V9 をここで実装・検証
        └─ (V9完了後) feature/vissim_windows_co-sim へマージ
```

- 作業開始時に`feature/vissim_windows_co-sim`を最新化してから`feat/vissim-warmup`を作成する。
- コミットはステップ単位で分ける(コミットメッセージにStep番号を入れる)。
- 作業中に`feature/vissim_windows_co-sim`が更新された場合は、`feat/vissim-warmup`へ取り込んでから検証を続ける。
- マージ条件: Step V9の確認項目がすべて完了し、単体テスト(`test/`)がすべて通ること。

### Step V0: 事前確認(実機、コード変更なし)

1. 空のtick(spawn/update/destroyなし)を連続で送ってVissimが正常に進むか。
2. 1 tickあたりのRPC往復時間を計測し、ウォームアップ100秒(2000 tick)にかかるwall-clockを見積もる。
3. 実際に使う`.inpx`の車両タイプがvtypes.jsonにすべて登録されているか(210 / 220 / 700)。
4. EGOのCARLA車種(`vehicle_type`)の`bounding_box`(車長・中心のずれ)を確認する。
5. Autowareが、センサーデータが数十秒〜数分遅れて届き始めても正常に起動・初期化できるか。
   → コード変更なしではEGOスポーンを遅らせられないため、**Step V4の確認項目へ移す**(V4ではウォームアップ分だけ自然に遅れる)。
6. ウォームアップに使う時間の目安(車両数が安定するまでの時間)をVissim単体で計測する。

計測は実機構成で行う。Vissimを起動するのは開発用PCとは**別のWindows PC(PTV Vissim 2026)**であり、
開発用PC(Vissim 2025のみインストール)では計測しない。

計測用ツール(パッケージにはインストールしない。ソースツリーから実行する):

| ツール | 対象 | 実行場所 |
|---|---|---|
| `tools/vissim_warmup_probe.py` | 1・2・6 | Linux機(CARLA・Autowareは不要。Windows側でアダプタ`server.py`を起動しておく) |
| `tools/carla_bbox_probe.py` | 4 | Linux機(CARLAサーバーを起動しておく) |

```bash
# 1・2・6: Vissimを300秒ぶん空のtickで進め、往復時間と車両数の推移を記録(終了時にVissimを閉じる)
python3 tools/vissim_warmup_probe.py --host <WindowsのIP> --duration 300 --csv warmup_probe.csv

# 4: EGO車種の車長とbounding_box中心のずれ
python3 tools/carla_bbox_probe.py vehicle.toyota.prius
```

- `vissim_warmup_probe.py`は実行のたびにアダプタとVissimを起動し直すこと(アダプタは最初のconnectの期間・分解能でVissimを起動し、
  異なる値の再connectを拒否するため)。
- 交通流の安定時間の目安(`warmup hint`)は「最後の60秒の平均±10%に収まり続ける最初の時刻」という簡易な指標なので、CSVも確認する。

**完了条件**: 1〜4・6の結果を本計画書§6に記録し、方針に影響があれば計画を更新する。

### Step V1: パラメータ追加(動作変更なし)

- launch arg / ROS param 4つを追加し、`carla_autoware.py`で読み込み、§2.2の起動時検査を追加する。
- `vissim_warmup_time=0`では何もしない。
- テスト: `vissim_launch_params_test.py`(2つのnode定義の一致)、起動時検査の異常系。

### Step V2: シミュレーション期間の拡張

- `get_vissim_sim_params()`に`warmup_time`/`wait_timeout`を追加し、`simPeriod`を§2.5のとおり計算する。
- `PTVVissimSimulation`の`end_tick`を「確定メソッド(`start_measurement(tick_count)`など)で後から設定」できるようにする。
  ウォームアップ無効時は従来どおり起動時に確定する。
- テスト: `vissim_sim_period_test.py`に、warmup/timeoutあり・なし、最大期間超過のケースを追加。

### Step V3: EGOスポーン処理の切り出し(動作変更なし)

- `load_world()`のEGOスポーン・センサー設定・Traffic Manager設定を`_spawn_ego_and_sensors()`へ移す。
- ウォームアップ無効時は`load_world()`から呼び、現状と同じ順序・同じ動作にする。
- 確認: ウォームアップ無効で実機co-simが従来どおり動くこと。

### Step V4: WARMUPとキャッチアップ

- `ego_spawn_gate.py`にWARMUPループを実装する(§2.3)。
- `spawn_all_vissim_actors_in_carla()`を実装する(§2.4)。
- この段階では、ウォームアップ後すぐにEGOをスポーンする(ギャップ判定なし)。
- 確認: ウォームアップ後、CARLA上にVissim車両・歩行者が揃って現れること。EGOがVissimへ登録され、
  通常co-simに移ること。ROS時刻が0付近から始まること。検証時間が`vissim_sim_period`どおりであること。
  Autowareが、センサーデータがウォームアップ分遅れて届き始めても正常に起動・初期化できること(V0 #5から移動)。

### Step V5: ギャップ判定ロジック(純粋関数 + 単体テスト)

- `evaluate_spawn_gap()`を実装する(§2.6.1〜2.6.3)。
- テストケース:
  - 前後に車両なし → SAFE
  - 前方車のクリアランスが必要距離未満 / ちょうど / 超過
  - 後方車も同様
  - 隣の車線の車両は無視される
  - 対向車線の車両は無視される
  - EGO予定位置と重なる車両(隣の車線にはみ出し) → UNSAFE
  - CARLAにいないVissim車両(仮の車長で判定)
  - 探索範囲外の車両は無視される

### Step V6: WAIT_FOR_SAFE_GAPとタイムアウト

- V4の「すぐにスポーン」を、判定 → NGなら同期1ステップ → 再判定、に置き換える(§2.6.5)。
- タイムアウト・SIGINT・連続失敗時の終了処理(§2.8)。
- 確認: 交通量の多い地点で「待つ → 空いたらスポーン」、交通量の非常に多い設定で「タイムアウトで終了」。

### Step V7: ログ・記録

- §2.9のログを実装する。
- 起動手順書に、パラメータ、スポーン地点の選び方(直線区間)、記録項目を追記する。

### Step V8(オプション): Vissim車長の対応表

- 運用して、必要距離の上乗せでは待ち時間が長くなりすぎる場合に実装する(§2.6.4)。
- `data/vissim_vehicle_lengths.json`(Vissim車両タイプ → 車長)を、使用する`.inpx`の2D/3Dモデル分布(Town01は§6 3-4)から作成し、
  `max(CARLA, Vissim)`で計算する。

### Step V9: 実機検証

| # | 確認内容 | 期待結果 |
|---|---|---|
| 1 | ウォームアップ無効(既定) | 現状と完全に同じ動作(EGOスポーン時刻、ログ、期間) |
| 2 | ウォームアップ100秒 | wall-clockがウォームアップ時間より十分短い。終了時の車両数が想定どおり |
| 3 | キャッチアップ | CARLA上の車両数 ≒ Vissim車両数(差分は`vissim_only`としてログに出る) |
| 4 | 空きありの地点 | ウォームアップ直後にSAFEでスポーン |
| 5 | 空きなしの地点 | WAITが続き、空いた時点でスポーン。スポーン直後に衝突しない |
| 6 | タイムアウト | 上限でエラー終了し、launch全体が停止。Vissimも閉じる |
| 7 | ROS時刻 | `/clock`とセンサーtimestampが0付近から単調増加。Autowareが正常に動く |
| 8 | 期間 | EGOスポーン後、ちょうど`vissim_sim_period`秒で停止する |
| 9 | 信号同期 | ウォームアップ後もVissimとCARLAの信号が一致している |
| 10 | 再現性 | 同じ`.inpx`・同じパラメータで2回実行し、EGOスポーン時刻・前後車IDが一致するか記録する |

---

## 5. 影響を受けない(変えない)もの

- 通常co-simのループ(`SensorLoop._tick_sensor()`、`run_bridge()`)
- `sync_vissim_to_carla()`/`sync_carla_to_vissim()`の既存処理
- Vissim/CARLAのSTEP周期(`fixed_delta_seconds`)、同期モード、信号同期、歩行者同期
- ROS timestamp・`/clock`の計算方法
- Windows側アダプタ・RPCプロトコル

---

## 6. 未確認事項・V0の記録欄

| # | 項目 | 状態 | 結果 |
|---|---|---|---|
| 1 | 空のtickの連続送信でVissimが進むか | 実機計測待ち | 計測ツールはダミーアダプタで動作確認済み(2026-09-30) |
| 2 | 1 tickのRPC往復時間 | 実機計測待ち | 同上 |
| 3 | vtypes.jsonの登録状況 | **確認済み**(2026-09-30、vtypes.json変更後に2026-10-05更新) | 下記3-1〜3-4 |
| 4 | EGO車種の車長・`bounding_box`中心のずれ | 一部確認 | `vehicle.toyota.prius`: 車長4.51 m・幅2.01 m・高さ1.52 m(`CARLA_車両寸法一覧.md`)。中心のずれは実機計測待ち |
| 5 | Autowareがセンサー開始の遅れに耐えるか | Step V4へ移動 | |
| 6 | 交通流が安定するまでの時間 | 実機計測待ち | |
| 7 | DSIでEGOを登録する際の重複チェック・位置補正の有無 | 未確認(公開情報なし) | ギャップ判定で事前に防ぐので、実装の前提にはしない |

**3-1. vtypes.json(コミット`7438613f6`、2026-10-05変更後)**

| Vissim車両タイプ | CARLAブループリント |
|---|---|
| 100: 乗用車 | 乗用車16車種(変更なし) |
| 210: トラック | `vehicle.carlamotors.carlacola` |
| 220: トレーラー | `vehicle.carlamotors.european_hgv`(トラクターのみ。CARLA標準に連結車はない) |
| 300: バス | `vehicle.mitsubishi.fusorosa` |
| 700: バイク | `vehicle.yamaha.yzf`, `vehicle.harley-davidson.low_rider`, `vehicle.kawasaki.ninja` |

- 変更前にあった200 / 400 / 510 / 520 / 610 / 620は削除された。これらの車両タイプを車両入力で使うネットワーク
  (変更前のTown01の車両構成1にある610など)では、該当車両がCARLAにスポーンされない(`vissim type N unknown`)。
- 変更途中の版では、220・300のブループリント名の末尾にゼロ幅スペース(U+200B)が混入し、CARLAで車種が見つからずスポーンされなかった。
  コミット版では除去済み(ASCII以外の文字なし)。
- ブリッジが読み込むのは、実行中のモジュールと同じ場所にある`data/vtypes.json`。別のコピー(CARLAリポジトリ側の
  `Co-Simulation/PTV-Vissim/data/vtypes.json`など)を編集しても反映されない(2026-10-05に実際に発生)。

**3-2. Town01(2026-10-05変更版。実機のVissim PCにある`.inpx`)**
- 車両入力7か所はすべて車両構成1(各50台/時)。構成比は100: 60%、210・220・300・700: 各10%。
  → **すべてvtypes.jsonに登録済み**。未登録の車両タイプによるCARLA未スポーンは起きない。
- `randSeed="42"`。
- **要確認**: Driving Simulatorが無効(`drivSimActive="false"`)になっている(変更前は`true`)。起動手順書では
  「無効だと実質的に何も同期しない」とされており、EGOがVissimへ登録されない可能性がある。
- **要確認**: EGOの車両タイプが`drivSimVehType="101"`(EGO)に変わった。一方、アダプタはEGO登録時に車両タイプ0を
  指定している(`PTV-Vissim_windows/constants.py`の`VISSIM_DEFAULT_VEHICLE_TYPE = 0`)ため、101が実際に使われるかは未確認。
  EGOのVissim上の車長(§2.6.4の後方の車長差)に影響する。

**3-3. 車長差への影響(§2.6.4、Town01基準)**

差 = Vissimの車長 − CARLAの車長。正の値ほど、Vissim上の前方の車間がCARLA上より短くなる。

| 車両タイプ | CARLA車長 | Vissim車長(Town01の3Dモデル) | 差 |
|---|---|---|---|
| 100 | 3.63〜5.03 m(16車種) | 3.75〜4.76 m(7モデル) | −1.28〜+1.13 m |
| 210 | 5.20 m(carlacola) | 7.93 m(HGV - Delivery DAF LF) | **+2.73 m** |
| 220 | 7.94 m(european_hgv) | 5.96 m(HGV - Semi-Tractor Volvo E、トレーラーなし) | −1.98 m |
| 300 | 10.27 m(fusorosa) | 12.14 m(Bus - C2 Standard) | +1.87 m |
| 700 | 2.04〜2.35 m(3車種) | 2.10 m(Bike - Motorbike Yamaha MT 07) | −0.25〜+0.06 m |

- 最大は+2.73 m(210)。前方の必要距離(`ego_spawn_front_margin`)に3 m程度上乗せすれば吸収できるため、
  **Town01ではStep V8(Vissim車長の対応表)は不要**の見込み。
- Vissimの車長は、2026-10-05変更版の`.inpx`の2D/3Dモデル分布から求めた。CARLAは`CARLA_車両寸法一覧.md`の値。
- 差の範囲は「Vissimの最短/最長モデル」と「CARLAの最短/最長車種」の組み合わせで求めた最小・最大値(実際は車両ごとにランダムに組み合わさる)。

**3-4. 参考: お台場(`odaiba_v015.inpx`、`Vissim_車両寸法一覧.md`の対象)**

現状の検討対象はTown01であり、以下は将来お台場でco-simする場合の参考。
- 車両入力で使う車両タイプは100 / 210 / 220 / 300 / 700で、vtypes.json変更後はすべて登録済み。
- 同じ車両タイプでも3DモデルがTown01と異なり、車長差が大きい(210: +5.06 m、220: トレーラー連結16.50 mで+8.56 m)。
  お台場でco-simする場合は、Step V8の実施と`EGO_SPAWN_UNKNOWN_VEHICLE_LENGTH_M`の見直しが必要。
- Driving Simulatorは無効(`drivSimActive="false"`)のまま。

---

## 7. 将来の拡張候補

- 速度に応じた必要距離(`後方車速度 × 時間ギャップ + 最小距離`)
- 曲線区間でのスポーン(CARLAの車線中心線に沿った距離で判定)
- 複数のスポーン候補地点から、最初に空いた地点を選ぶ
