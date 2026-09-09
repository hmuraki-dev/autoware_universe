# SUMO-CARLA-Autoware co-sim: 歩行者(Pedestrian)同期 実装計画 v1.0

対象ブランチ: `feature/sumo_co-sim`

移植元(修正記録): `/home/divp/CARLA/Co-Simulation/Sumo/docs/PEDESTRIAN_TODO.md`
移植元(実装コード): `/home/divp/CARLA/Co-Simulation/Sumo/sumo_integration/`, `run_synchronization.py`

移植先(本リポジトリ、vendor先):
`simulator/autoware_carla_interface/src/autoware_carla_interface/sumo_integration/`
`simulator/autoware_carla_interface/src/autoware_carla_interface/carla_autoware.py`

---

## 0. 前提・スコープ(移植元の確定事項を踏襲)

移植元の `PEDESTRIAN_TODO.md` で確定した以下の方針は、本リポジトリへの移植でもそのまま踏襲する。

- **sumo→carla の一方向のみ**。carla→sumo方向(CARLA側でspawnした歩行者をSUMOの`person`
  として送り返す機能)は実装しない。
- **常時有効**(CLI引数・launch引数を新設しない)。車両同期と同様、`sumo2carla_ids`まわりと
  全く同じく無条件で毎tick実行する。
- 歩行者タイプ→carla walkerブレンプリントのマッピングは既存の`data/vtypes.json`を拡張する
  形で行う(専用ファイルは作らない)。
- Z座標補正(carla.Walkerのtransform原点がbounding box垂直中心である点への対応)が必要。
  移植元で実機検証済み・確定した補正式(`transform.location.z += sumo_person.extent.z`)を
  そのまま使う。

## 0.1 本リポジトリ固有の前提(移植元との差分)

移植元(`run_synchronization.py`)は `SimulationSynchronization.tick()` が
「sumo tick → sumo→carla同期 → carla tick → carla→sumo同期」を1メソッドに直列実行するが、
本リポジトリの vendor版 `simulation_synchronization.py` は Step 4 の変更で
`sync_sumo_to_carla()` / `sync_carla_to_sumo()` に分割済みであり、`carla_autoware.py`
の `SensorLoop._tick_sensor()` がこの2つを **CARLAの単一`world.tick()`を挟んで**個別に
呼び出している(`tick()`自体は後方互換ラッパーとしてのみ残存)。

歩行者同期は sumo→carla 一方向のみのため、移植元 `tick()` 内の「sumo-->carla pedestrian
sync」ブロック全体は、本リポジトリでは **`sync_sumo_to_carla()` の中に**(既存の車両
「sumo-->carla sync」ブロックの直後、TLS(`tls_manager == 'sumo'`)ブロックの前)挿入する。
これにより移植元と同じ相対順序を保ちつつ、単一Tick原則(v0.5 3.1-3.7)を崩さない。

また、終了処理も移植元と本リポジトリで構造が異なる点に注意する:

- 移植元: `SimulationSynchronization.close()` が同期アクター破棄・TraCI切断・CARLA
  world設定復元を1メソッドで行う。
- 本リポジトリ: `carla_autoware.py` の `InitializeInterface._cleanup_sumo()` が
  `close()`相当の処理を**ステップごとにtry/exceptで分離再実装**しており(v0.5 2.13/3.11/3.12)、
  `SimulationSynchronization.close()` 自体は直接呼ばれない(標準/後方互換用途のみに残存)。
  したがって歩行者用アクターの破棄処理は **両方**(`simulation_synchronization.py`の
  `close()` と `carla_autoware.py`の`_cleanup_sumo()`)に追加する必要がある。

---

## 進捗状況

| # | 項目 | 状態 |
|---|---|---|
| 1 | `sumo_integration/sumo_simulation.py`: 歩行者データ取得ロジック追加 | 未着手 |
| 2 | `sumo_integration/carla_simulation.py`: walker用差分集合・synchronize_pedestrian追加 | 未着手 |
| 3 | `sumo_integration/bridge_helper.py`: `get_carla_pedestrian_transform()`追加 | 未着手 |
| 4 | `sumo_integration/data/vtypes.json`: walkerブレンプリント追加 | 未着手 |
| 5 | `sumo_integration/simulation_synchronization.py`: `sync_sumo_to_carla()`/`close()`統合 | 未着手 |
| 6 | `carla_autoware.py`: `_cleanup_sumo()`に歩行者破棄処理を追加 | 未着手 |
| 7 | `sumo_integration/NOTICE.md`: 変更ファイル一覧の更新 | 未着手 |
| 8 | スタブ回帰テスト作成 | 未着手 |
| 9 | 実機検証(SUMO + CARLA + Autoware fullstack) | 未着手 |
| 10 | ドキュメント更新(起動手順.md) | 未着手 |

---

## 1. `sumo_integration/sumo_simulation.py`: 歩行者データ取得ロジック

現状NOTICE.mdでは本ファイルは「vendored as-is (no logic changes)」と記載されているが、
本タスクで初めてロジック変更が入るため、NOTICE.mdの更新(タスク7)も必要になる。

`SumoSimulation`クラスに以下を追加する(移植元と同一内容、パス調整等は不要):

- `__init__`: 既存の`self.spawned_actors = set()` / `self.destroyed_actors = set()`の
  直後に、
  ```python
  self.spawned_persons = set()
  self.destroyed_persons = set()
  ```
  を追加。
- `subscribe_person(person_id)`(`@staticmethod`): 既存の`subscribe()`(車両)と対になる
  歩行者版。`traci.person.subscribe(person_id, [VAR_TYPE, VAR_VEHICLECLASS, VAR_COLOR,
  VAR_LENGTH, VAR_WIDTH, VAR_HEIGHT, VAR_POSITION3D, VAR_ANGLE, VAR_SLOPE, VAR_SPEED])`。
  `VAR_SIGNALS`/`VAR_SPEED_LAT`は歩行者に該当概念が無いため除外。
- `unsubscribe_person(person_id)`(`@staticmethod`): `traci.person.unsubscribe(person_id)`。
- `get_person(person_id)`(`@staticmethod`): `traci.person.getSubscriptionResults(person_id)`
  から値を取り出し、既存の`SumoActor`namedtuple(`type_id vclass transform signals extent
  color`)をそのまま再利用して返す(`signals=None`固定)。専用namedtupleは新設しない。
- `tick()`: 既存の
  ```python
  self.spawned_actors = set(traci.simulation.getDepartedIDList())
  self.destroyed_actors = set(traci.simulation.getArrivedIDList())
  ```
  の直後に、
  ```python
  self.spawned_persons = set(traci.simulation.getDepartedPersonIDList())
  self.destroyed_persons = set(traci.simulation.getArrivedPersonIDList())
  ```
  を追加。

`spawn_person`/`destroy_person`(carla→sumo方向専用)は本スコープでは実装しない。

---

## 2. `sumo_integration/carla_simulation.py`: walker spawn/update/destroy

`CarlaSimulation`クラスに以下を追加する。

- `__init__`: 既存の`self._active_actors` / `self.spawned_actors` /
  `self.destroyed_actors`の直後に、
  ```python
  self._active_walkers = set()
  self.spawned_walkers = set()
  self.destroyed_walkers = set()
  ```
  を追加。
- `synchronize_pedestrian(walker_id, transform)`: `synchronize_vehicle()`に倣った歩行者版。
  `lights`引数に相当するものが無いため省略。
  ```python
  def synchronize_pedestrian(self, walker_id, transform):
      walker = self.world.get_actor(walker_id)
      if walker is None:
          return False
      walker.set_transform(transform)
      return True
  ```
- `update_actor_diff()`: **本リポジトリ固有**。移植元では`tick()`内の
  `world.tick()`直後にvehicle差分計算とwalker差分計算が両方書かれているが、本リポジトリでは
  `world.tick()`自体が呼び出し元(`carla_autoware.py`)にあるため、このメソッド(vehicle差分の
  み計算する既存実装)に、既存の`vehicle.*`フィルタブロックの直後、以下のwalker差分計算を追加する:
  ```python
  current_walkers = set(
      [walker.id for walker in self.world.get_actors().filter('walker.pedestrian.*')])
  self.spawned_walkers = current_walkers.difference(self._active_walkers)
  self.destroyed_walkers = self._active_walkers.difference(current_walkers)
  self._active_walkers = current_walkers
  ```
  `tick()`(後方互換ラッパー)は`update_actor_diff()`を呼ぶだけなので追加変更不要。
- `spawn_actor(blueprint, transform)` / `destroy_actor(actor_id)`は修正不要(車両・歩行者
  共用の汎用実装のまま流用)。

---

## 3. `sumo_integration/bridge_helper.py`: 歩行者用transform変換

現状NOTICE.mdでは本ファイルは「vendored as-is (vtypes.jsonパス調整のみ)」と記載されているが、
本タスクでロジック追加が入るため、NOTICE.mdの更新(タスク7)が必要になる。

`BridgeHelper`クラスに、既存の`get_carla_transform()`直後、以下を追加する(移植元と同一):

```python
@staticmethod
def get_carla_pedestrian_transform(sumo_person):
    """
    歩行者はcarla.Walkerのtransform原点がbounding box垂直中心であり、
    地面(足元)基準のsumo座標とはZ軸がずれるため、sumo_person.extent.z
    (VAR_HEIGHTの半分)を加算して補正する。extent=(0,0,0)を渡すことで、
    車両向けのfront-center-bumper補正(extent.x分のオフセット)はスキップされる。
    """
    transform = BridgeHelper.get_carla_transform(sumo_person.transform, carla.Vector3D(0, 0, 0))
    transform.location.z += sumo_person.extent.z
    return transform
```

`get_carla_blueprint()`/`_get_recommended_carla_blueprint()`はロジック変更不要
(`vClass: "pedestrian"`が付与されたwalkerブレンプリントに対して既存のフォールバック経路が
そのまま機能する)。

---

## 4. `sumo_integration/data/vtypes.json`: walkerブレンプリント追加

移植元(`/home/divp/CARLA/Co-Simulation/Sumo/data/vtypes.json`)で既に実機確認済みの
`walker.pedestrian.0001`〜`0051`(vClass: "pedestrian")のエントリ(51件)を、本リポジトリの
`carla_blueprints`にそのまま追加する。

- 同一の`~/CARLA`インストール(同一バージョン)を使用しているため、ブレンプリートIDの再列挙
  (`world.get_blueprint_library().filter('walker.pedestrian.*')`)は不要と想定されるが、
  念のため追加後にCARLAサーバーへ接続して同じ51件が存在することを確認する
  (バージョン差異があれば再列挙して差し替える)。
- JSONの妥当性(パース可否・キー重複無し)を確認する。
- `util/create_sumo_vtypes.py`相当のスクリプトがこのリポジトリに存在する場合も
  修正不要(carla→sumo方向専用、`vehicle.*`のみを処理するため)。

---

## 5. `sumo_integration/simulation_synchronization.py`: `sync_sumo_to_carla()`/`close()`統合

- `__init__`: 既存の`self.sumo2carla_ids = {}` / `self.carla2sumo_ids = {}`の直後に、
  ```python
  self.sumo2carla_ped_ids = {}  # sumo person_id -> carla walker actor_id
  self._ped_tick_count = 0      # デバッグログの間引き用(20tick毎)
  ```
  を追加。CLI引数(`--sync-pedestrians`等)は追加しない(常時有効の確定方針)。
- `sync_sumo_to_carla()`: 既存の車両「sumo-->carla sync」ブロック(vehicle spawned/
  destroyed/updateループ)の直後、`tls_manager == 'sumo'`ブロックの**前**に、
  「sumo-->carla pedestrian sync」ブロックを新設し、無条件(`if`ガード無し)で毎回実行する:
  1. `self.sumo.spawned_persons`をイテレートし、`subscribe_person` →
     `get_person` → `BridgeHelper.get_carla_blueprint(sumo_person)` → 成功すれば
     `BridgeHelper.get_carla_pedestrian_transform(sumo_person)` で変換したtransformで
     `self.carla.spawn_actor(blueprint, transform)` → `self.sumo2carla_ped_ids[person_id]
     = walker_actor_id`。ブレンプリント取得/spawn失敗時は`unsubscribe_person`する。
  2. `self.sumo.destroyed_persons`をイテレートし、対応する`sumo2carla_ped_ids`の
     エントリを`self.carla.destroy_actor(...)`で破棄(`pop`で辞書からも削除)。
  3. `self.sumo2carla_ped_ids`の全エントリについて`get_person` →
     `get_carla_pedestrian_transform` → `self.carla.synchronize_pedestrian(walker_id,
     transform)`。
  4. 20tick毎のサンプルデバッグログ(移植元と同じ間引き方式)を追加する。
     spawn/destroy時は都度debugログを出す(移植元の教訓: 「ログが無いとトラブルシュートし
     づらい」ため)。
- `close()`(後方互換/standalone用途のみに残存): 既存の
  ```python
  for carla_actor_id in self.sumo2carla_ids.values():
      self.carla.destroy_actor(carla_actor_id)
  ```
  の直後に、
  ```python
  for carla_walker_id in self.sumo2carla_ped_ids.values():
      self.carla.destroy_actor(carla_walker_id)
  ```
  を追加する。
- `tick()`(後方互換ラッパー)は`sync_sumo_to_carla()`/`sync_carla_to_sumo()`を順に呼ぶ
  だけなので変更不要。

---

## 6. `carla_autoware.py`: `_cleanup_sumo()`への歩行者破棄処理追加(本リポジトリ固有)

`InitializeInterface._cleanup_sumo()`は`SimulationSynchronization.close()`を直接呼ばず、
各ステップをtry/exceptで分離再実装しているため、歩行者の破棄処理もここに追加する必要がある
(タスク5だけでは実運用パス上で歩行者アクターが破棄されずリークする)。

既存の「Destroy SUMO-origin actors mirrored into CARLA」ブロック
(`for carla_actor_id in list(self.sumo_sync.sumo2carla_ids.values()):`)の直後に、
同じtry/exceptパターンで以下を追加する:

```python
# Destroy SUMO-origin pedestrians mirrored into CARLA (walkers spawned via
# sync_sumo_to_carla()'s pedestrian block).
for carla_walker_id in list(self.sumo_sync.sumo2carla_ped_ids.values()):
    try:
        self.sumo_carla_sim.destroy_actor(carla_walker_id)
    except Exception as e:
        print(f"Warning: failed to destroy SUMO-origin CARLA walker {carla_walker_id}: {e}")
```

歩行者は一方向同期(sumo→carla)のみのため、`carla2sumo_ids`側の処理には歩行者関連の
追加は不要。

---

## 7. `sumo_integration/NOTICE.md`: 変更ファイル一覧の更新

- `sumo_simulation.py`を「Files vendored as-is」セクションから外し、「Files vendored with
  modifications」セクションに移動。変更理由(歩行者`traci.person`用の
  subscribe/unsubscribe/get_person追加、`tick()`での`spawned_persons`/`destroyed_persons`
  追加)を記載。
- `bridge_helper.py`の記載も同様に更新し、`get_carla_pedestrian_transform()`追加を明記。
- `carla_simulation.py`の既存の変更理由に、walker差分集合・`synchronize_pedestrian()`・
  `update_actor_diff()`拡張を追記。
- `simulation_synchronization.py`の既存の変更理由に、`sumo2carla_ped_ids`および歩行者
  同期ブロックの追加を追記。

---

## 8. スタブ回帰テスト作成

移植元の`util/pedestrian_sync_stub_test.py`と同じアプローチ(`traci`をモックせず、
`SimulationSynchronization`へ渡す`sumo_simulation`/`carla_simulation`引数を単純な
Pythonクラス`FakeSumoSimulation`/`FakeCarlaSimulation`で差し替える)を本リポジトリにも
移植する。

- 配置場所: 本パッケージには現状pytest等のテスト基盤が無いため、`colcon build`の対象に
  含まれない`simulator/autoware_carla_interface/test/pedestrian_sync_stub_test.py`
  として新設する(手動実行: `PYTHONPATH=src`を通した上で
  `python3 test/pedestrian_sync_stub_test.py`)。
- 本リポジトリ固有の差異に対応する:
  - `SimulationSynchronization.__init__`はCARLAの同期モード設定
    (`world.apply_settings()`/`traffic_manager.set_synchronous_mode()`)を呼ばない
    (Step4で削除済み)ため、`FakeCarlaSimulation`側にこれらのメソッドが無くてもエラーに
    ならないことを確認する(モック不足による見かけ上のテスト成功に注意)。
  - 本番の呼び出し順(`carla_autoware.py`の`SensorLoop._tick_sensor()`)に合わせ、
    `synchronization.tick()`を呼ぶのではなく、`sync_sumo_to_carla()` →
    (fakeのworld tick相当処理) → `carla.update_actor_diff()`/`sync_carla_to_sumo()`
    を個別に呼ぶ形でテストする(`tick()`をそのまま呼んでも動作はするはずだが、実運用パスと
    異なるため、実運用パスに合わせたテストの方が回帰検知の実効性が高い)。
- 実際の`data/vtypes.json`(タスク4で更新後のもの)を読み込み、`walker.pedestrian.*`/
  `vClass:"pedestrian"`エントリからfakeブレンプリントライブラリを構築し、タスク4の内容自体の
  回帰テストも兼ねる。
- カバレッジ: 新規spawn(vtypes.jsonのwalkerエントリから選ばれること、Z補正
  (`extent.z`分の引き上げ)が正しいこと)、未対応vclass(spawnされないこと)、毎tickの位置更新、
  destroy時に`sumo2carla_ped_ids`と対応walkerが削除されること、`_cleanup_sumo()`相当の
  破棄処理(タスク6)が`sumo2carla_ped_ids`の全エントリを漏れなく破棄すること。

---

## 9. 実機検証(SUMO + CARLA + Autoware fullstack)

移植元で完了済みの検証(歩行者spawn/update/destroy、Z補正)に加え、本リポジトリ固有の
統合ポイントを重点的に確認する。

- `docs/SUMO-CARLA-Autoware_co-sim_起動手順.md`の手順に従い、CARLA + Town01ロード +
  Autoware(SUMO co-sim有効)を起動し、`examples/Town01.sumocfg`の`personFlow`
  から生成される歩行者が正しくspawn/update/destroyされることを確認する。
- 歩行者がEGO車両(Autoware)のオートアダプト機構(v0.5 2.10)や既存の車両同期
  (`sumo2carla_ids`/`carla2sumo_ids`)と干渉しないこと(特に`update_actor_diff()`の
  vehicle差分計算がwalker追加によって影響を受けないこと)を確認する。
- Autoware側の停止(Ctrl+C等)経由で`InitializeInterface._cleanup_sumo()`が呼ばれた際、
  spawn済みのwalkerアクターが全てCARLAから破棄され、リークしないことを確認する
  (タスク6の検証)。
- Z補正の値が移植元と同じ(`z=0.000000`→`z≈0.8595`相当)になることを確認する。

---

## 10. ドキュメント更新

- `docs/SUMO-CARLA-Autoware_co-sim_起動手順.md`に「歩行者(Pedestrian)同期について」の
  節を追加し、常時有効(CLI引数不要)であること、sumo→carla一方向のみであること、
  `--debug`相当のログの見方、Z補正の概要を記載する(移植元の`docs/SUMO-CARLA_co-sim_
  起動手順.md`の記載内容をベースに、本リポジトリの`_tick_sensor()`/`_cleanup_sumo()`
  構成に合わせて調整する)。
- 本ファイル(実装計画)の進捗表・各タスクの内容を実装の進行に合わせて更新する。
