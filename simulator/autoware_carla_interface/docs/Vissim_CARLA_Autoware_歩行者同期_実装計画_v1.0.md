# Vissim–CARLA–Autoware統合 歩行者同期 実装計画

本ドキュメントは、`docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`(以下「本編計画」)に基づいて実装済みの
Vissim–CARLA–Autoware co-simulationに対し、アップストリームのVissim–CARLAブリッジに新たに追加された
「歩行者(pedestrian)同期処理」を移植するための調査結果とタスク一覧をまとめたものである。

参照した記録: `/home/divp/CARLA/Co-Simulation/PTV-Vissim/docs/PEDESTRIAN_TODO.md`
(タスク0〜9完了、タスク10〜11(実機検証・ドキュメント更新)はアップストリームでも未完了)

**本ドキュメントの位置づけ**: 調査・計画のみ。本編計画のStep 0〜8と同様、実装は本ドキュメントの内容を
ユーザーが確認した上で、個別に着手を指示された段階(例:「Step P1を進めて」)で行う。

---

## 0. 前提調査で判明した事実

### 0.1 アップストリームの変更規模

アップストリーム(`/home/divp/CARLA/Co-Simulation/PTV-Vissim/`)は、本リポジトリへのvendor化(本編計画
Step 1)以降に以下のファイルが変更・追加されている。

| ファイル | 変更前後の行数 | 変更内容 |
|---|---|---|
| `vissim_integration/vissim_simulation.py` | 610 → 798 (+188) | 歩行者関連の enum/struct/クラス/DS Interface呼び出し追加 |
| `vissim_integration/carla_simulation.py` | 203 → 242 (+39) | walker(歩行者アクター)のスポーン/破棄追跡・同期メソッド追加 |
| `vissim_integration/bridge_helper.py` | 185 → 252 (+67) | 歩行者向けBlueprint/Transform変換の追加 |
| `run_synchronization.py` | 355 → 452 (+97) | `SimulationSynchronization`への歩行者同期組み込み |
| `data/ptypes.json` | 新規 (48行) | vissim pedestrianType → CARLA walker blueprint のマッピング |
| `util/pedestrian_sync_stub_test.py` | 新規 | fakeオブジェクトによるスタブ検証(実CARLA/Vissim不要) |

本リポジトリの現行vendor版(`simulator/autoware_carla_interface/src/autoware_carla_interface/vissim_integration/`)
は上記のいずれの歩行者関連変更も含んでいない(本編計画Step 1時点のコードのまま)。

### 0.2 設計方針(アップストリームPEDESTRIAN_TODO.mdで確定済み)

- **同期方向はvissim→carlaのみ**。carla→vissim方向の歩行者同期は範囲外(DS Interfaceの
  `Simulator_Ped_Data`にはID/Create/Delete/ControlledByVissimに相当するフィールドが無く、
  carla側で生成した歩行者をvissimへ登録する手段が無いため)。
- **CLIフラグなし・常時有効**。当初 `--sync-pedestrians`(デフォルトOff)というフラグが実装されたが、
  「vissimはログに歩行者を出すのにCARLA上には何も見えない」という、過去の `--sync-traffic-lights`
  と同種の混乱を招くとして2026-09-03に削除され、車両同期と全く同じ「常時有効」方式に統一された。
  → **本リポジトリへの移植でも、`use_vissim` が有効な限り歩行者同期は常時有効とし、独立したROSパラメータ
  やlaunch引数は追加しない**方針を踏襲する。
- `data/ptypes.json`: vissim pedestrianType (100=Man, 200=Woman, 300=Wheelchair User) →
  CARLA `walker.pedestrian.*` blueprint IDのリスト。300(車椅子)は対応するCARLA walkerが存在しないため
  空配列 `[]`(vtypes.jsonの「非対応type」表現と同じ慣習)。
- 歩行者ID対応表 `vissim2carla_ped_ids`(vissim PedestrianID → carla walker actor id)は、車両用の
  `vissim2carla_ids`/`carla2vissim_ids` とは独立した辞書として新設(carla2vissim相当は存在しない)。
- 歩行者アクターのCARLA上の移動は `carla.WalkerControl` ではなく、車両と同じ **`set_transform()` +
  `set_target_velocity()`** による純粋な運動学的更新(理由:車両同期との一貫性・実装の単純さを優先。
  トレードオフとして歩行アニメーションが実速度と一致しない場合がある、とアップストリームで明記)。
- CARLAのwalkerアクターは**バウンディングボックス中心が transform 原点**になる(車両は接地面が原点)ため、
  vissimの `Position_Z`(接地面)をそのまま使うと歩行者が地面に半分めり込む。これを補正するため、
  `BridgeHelper.get_carla_pedestrian_transform()` という**車両用 `get_carla_transform()` とは別の新規関数**
  が追加されている(既存の `get_carla_transform()`/`get_carla_velocity()` 自体は無変更)。
  → 会話サマリ時点の認識(「get_carla_transform/get_carla_velocityは無変更で済む」)は不正確だったため、
  ここで訂正する。**新規関数が1つ追加される。**

### 0.3 スコープ外・要確認の差分(重要・ユーザー確認事項)

アップストリームの `run_synchronization.py` の `SimulationSynchronization.__init__` に、
PEDESTRIAN_TODO.mdのタスク一覧には記載のない以下の1行が追加されているのを確認した。

```python
# weather setting
self.carla.world.set_weather(carla.WeatherParameters.ClearNoon)
```

これは歩行者同期のタスクとは無関係な、アップストリーム作業者自身の検証用途(歩行者を見やすくする天候固定
など)の可能性がある変更であり、PEDESTRIAN_TODO.mdのどのタスクにも対応しない。**本リポジトリへの移植では
この天候設定変更は自動的には持ち込まない**(CARLA天候はautoware_carla_interface側の既存責務であり、
歩行者同期ロジックの必須要素ではないため)。もし移植を希望する場合は別途指示すること。

---

## 1. 現行vendor版コードへの適用方針

本リポジトリの `vissim_integration` パッケージは本編計画Step 1〜8で以下の構造的な差分をアップストリームに
対して持つ(`NOTICE.md` に記録済み)。歩行者同期の移植は、アップストリームを単純に上書きコピーするのでは
なく、**この差分を維持したまま該当ロジックのみを合流させる**必要がある。

| 項目 | アップストリーム | 本リポジトリ |
|---|---|---|
| `CarlaSimulation.__init__` | 独自に `carla.Client()` 生成 | `client`/`world` を注入 |
| `CarlaSimulation.tick()` | `world.tick()` + actor diff更新が一体 | `tick()`(後方互換)と `update_actor_diff()`(diffのみ)に分割 |
| `SimulationSynchronization.__init__` | CARLA同期モード設定を自前で実施 | 同設定は行わない(`InitializeInterface.load_world()` に一元化) |
| `SimulationSynchronization.tick()` | vissim→carla / carla→vissim が一体 | `sync_vissim_to_carla()` / `sync_carla_to_vissim()` に分割、`tick()` は後方互換ラッパ |
| エラー処理 | 素朴(例外を投げっぱなし) | Step 8で各アクター単位のtry/except・整合性チェックを追加(`feat/vissim_co-sim_step8` ブランチ) |

歩行者同期はvissim→carla方向のみのため、**`sync_vissim_to_carla()` 側にのみ**組み込めばよく、
`sync_carla_to_vissim()` 側の変更は不要である。

**ブランチについての確認事項**: Step 8のtry/except強化は現在 `feature/vissim_co-sim` にはマージされておらず、
別ブランチ `feat/vissim_co-sim_step8` にのみ存在する。歩行者同期の移植を「Step 8の例外処理込みで」行うか
「`feature/vissim_co-sim` 上でStep 8とは独立に」行うかは、実装着手前にユーザーに確認する
(下記タスク一覧はどちらの土台でも適用できるよう記述するが、Step 8ブランチを土台にする場合は
歩行者スポーン/破棄/同期の各呼び出しにも同種のtry/exceptを追加するのが自然な流れになる)。

---

## 2. 修正が必要なファイル一覧とタスク

### タスクP1: `data/ptypes.json` の新規vendor化

- アップストリームの `/home/divp/CARLA/Co-Simulation/PTV-Vissim/data/ptypes.json`(48行)を、
  `simulator/autoware_carla_interface/src/autoware_carla_interface/vissim_integration/data/ptypes.json`
  としてそのままvendor化(バイト同一、`vtypes.json`/`signal_mapping.json` と同じ扱い)。
- 内容: `"100"`(Man)/`"200"`(Woman) に各20種の `walker.pedestrian.NNNN` blueprint、`"300"`(Wheelchair User)
  は空配列 `[]`。

### タスクP2: `vissim_simulation.py` への歩行者取得ロジック追加

現行vendor版(599行)に以下を追加する。アップストリームでの追加位置はすべて特定済み。

1. **新規enum** `VissimPedestrianMotionState`(19値: `APPROACHING_PT_VEHICLE=1` 〜 `END=19`)と
   `VissimPedestrianConstructionElementType`(`NONE=0`/`AREA=1`/`RAMP=2`/`ELEVATOR_GROUP=3`/`PED_LINK=4`)
   を、既存の `VissimSignalState` 等のenum定義群と同じ場所に追加。
2. **新規ctypes構造体** `VISSIM_Ped_Data`(18フィールド。`PedestrianID`, `PedestrianType`,
   `ModelFileName[NAME_MAX_LENGTH]`, `Length/Width/Height`, `Position_X/Y/Z`,
   `Orient_Heading/Orient_Pitch`, `DistanceSinceBirth`, `Speed`, `MotionState`,
   `ConstructionElementType`, `ConstructionElementID`, `ConstructionElementName[NAME_MAX_LENGTH]`,
   `PreviousIndex`)。`ControlledByVissim` 相当のフィールドは無い
   (`VISSIM_GetTrafficPedestrians` がシミュレータ由来の歩行者を最初から除外して返すため不要)。
3. **新規クラス** `VissimPedestrian`(`VissimVehicle` に似るが、「自アクターの記録」ロジックは不要。
   `__init__(self, pedestrian_id, type_id, model_filename, extent, location, rotation, velocity,
   motion_state=None)`。`extent` は `(length, width, height)` タプル。`get_velocity()`/`get_transform()`
   を提供)。
4. **`_declare_prototypes()` への追加**:
   ```python
   self.ds_proxy.VISSIM_GetTrafficPedestrians.argtypes = [
       POINTER(c_int), POINTER(POINTER(VISSIM_Ped_Data))
   ]
   self.ds_proxy.VISSIM_GetTrafficPedestrians.restype = None
   ```
5. **新規アクセサ**: `get_pedestrian(self, pedestrian_id): return self._vissim_pedestrians[pedestrian_id]`
   (`get_actor()` と同じパターン)。
6. **`PTVVissimSimulation.__init__` への初期化追加**: `self._vissim_pedestrians = {}`,
   `self.spawned_pedestrians = set()`, `self.destroyed_pedestrians = set()`
   (現行 `self._vissim_vehicles = {}` 等と同じ並びに追加)。
7. **`tick()` への追加**(車両の `spawned_vehicles`/`destroyed_vehicles` 計算の直後、signal取得の直前に
   挿入する):
   - `VISSIM_GetTrafficPedestrians` を呼び出し、`VissimPedestrian` オブジェクトを構築。
   - `MotionState` が未知の値の場合は `logging.warning` して `motion_state=None` のまま保持
     (例外にはしない)。
   - 車両と同じ「IDセット差分」方式で `spawned_pedestrians`/`destroyed_pedestrians` を計算
     (`VISSIM_GetPedestrianLists`/`PreviousIndex` のような専用APIは使わない)。
   - 20 tick毎にサンプル歩行者1体の位置/MotionStateをdebugログ出力(既存の車両サンプルログと同じ様式)。

  **注意**: 現行vendor版の `vissim_simulation.py` はNOTICE.md記載の通り「module-levelのデバッグブロック削除」
  という唯一の逸脱がある。歩行者ロジックの合流はこの逸脱と無関係な箇所への追加なので、コンフリクトは
  発生しない見込み。

### タスクP3: `carla_simulation.py` へのwalker管理追加

現行vendor版(220行、`update_actor_diff()`/`tick()` 分割済み)に以下を追加する。

1. **`__init__` への追加**(既存の `self._active_actors`/`spawned_actors`/`destroyed_actors` の直後):
   ```python
   self._active_walkers = set()
   self.spawned_walkers = set()
   self.destroyed_walkers = set()
   ```
2. **新規メソッド** `synchronize_pedestrian(self, walker_id, transform, velocity=None)`:
   `synchronize_vehicle()` とほぼ同一のロジック(`get_actor()` → `None` チェック → `set_transform()` →
   `velocity is not None` なら `set_target_velocity()`)。light_state相当は無し。
3. **walker用の actor diff 計算**: アップストリームでは `tick()` 内で `vehicle.*` フィルタの直後に
   `walker.pedestrian.*` フィルタで同様の diff を計算している。
   **本リポジトリでは `tick()` ではなく `update_actor_diff()` 側に追加する**
   (`update_actor_diff()` が「world.tick()を呼ばずに diff だけ更新する」現行版の責務そのものであるため。
   `tick()` は `world.tick()` の後に `update_actor_diff()` を呼ぶだけの後方互換ラッパなので、
   `update_actor_diff()` 側に置けば両方から恩恵を受ける)。
   ```python
   current_walkers = set(
       [walker.id for walker in self.world.get_actors().filter('walker.pedestrian.*')])
   self.spawned_walkers = current_walkers.difference(self._active_walkers)
   self.destroyed_walkers = self._active_walkers.difference(current_walkers)
   self._active_walkers = current_walkers
   ```

### タスクP4: `bridge_helper.py` への歩行者変換関数追加

現行vendor版(188行、無変更でvendor化されている)に以下を追加する。

1. **クラス変数追加**: `ptypes = {}`(既存の `vtypes = {}` の直後)。
2. **新規メソッド** `get_carla_pedestrian_blueprint(vissim_pedestrian)`:
   `get_carla_blueprint()` とほぼ同じロジック(`BridgeHelper.ptypes` を参照、候補からランダム選択、
   blueprint存在チェック)。ただし walker blueprintには `color`/`driver_id` 属性が無いため、
   `role_name` のみ `has_attribute()` チェック付きで設定する点が異なる。
3. **新規メソッド** `get_carla_pedestrian_transform(vissim_pedestrian)`:
   `get_carla_transform(vissim_pedestrian.get_transform())` を呼んだ後、
   `carla_transform.location.z += vissim_pedestrian.extent[2] / 2.0` で高さ補正を加える
   (0.2節で述べたバウンディングボックス中心原点への補正。**既存の `get_carla_transform()`/
   `get_carla_velocity()` 自体は無変更**、これらはそのまま歩行者にも流用できる)。

### タスクP5: `simulation_synchronization.py` への同期処理組み込み

現行vendor版(284行、`sync_vissim_to_carla()`/`sync_carla_to_vissim()`/`close()` に分割済み)に
以下を追加する。

1. **`__init__` への追加**(`vtypes.json` 読み込みの直後、signal_mapping関連コードの前に挿入):
   ```python
   self.vissim2carla_ped_ids = {}

   BridgeHelper.ptypes = self._load_ptypes(os.path.join(dir_path, 'data', 'ptypes.json'))
   if BridgeHelper.ptypes:
       logging.info('Vissim pedestrian type(s) mapped : %s', sorted(BridgeHelper.ptypes.keys()))
   else:
       logging.warning(
           'No usable ptypes.json was found - no pedestrians will be synchronized.')
   ```
   CLIフラグは無いため(0.2節)、`sync_traffic_lights` のような有効/無効の分岐は不要
   (`ptypes.json` が読めなければ辞書が空になり、自然に同期がno-opになる)。
2. **新規static method** `_load_ptypes(path)`(`_load_signal_mapping()` と同じ try/except パターンで
   IOError/OSError/ValueErrorを警告ログに変換して `{}` を返す)。
3. **`sync_vissim_to_carla()` への追加**: 車両の「Updating vissim controlled vehicles in carla」ループの
   直後、`# vissim-->carla signal sync` ブロックの**前**に、以下3ループを追加する(アップストリームの
   `tick()` 内の配置と同じ相対位置)。
   - spawnループ: `self.vissim.spawned_pedestrians` を走査し、
     `BridgeHelper.get_carla_pedestrian_blueprint()` → `get_carla_pedestrian_transform()` →
     `self.carla.spawn_actor()` → 成功時 `self.vissim2carla_ped_ids[vissim_pedestrian_id] = carla_walker_id`。
     車両と異なり `carla2vissim_ids` との差分計算は不要(vissim側は元々simulator歩行者を含まないため)。
   - destroyループ: `self.vissim.destroyed_pedestrians` を走査し、`vissim2carla_ped_ids` にあれば
     `self.carla.destroy_actor()` して pop。
   - updateループ: `self.vissim2carla_ped_ids` の全エントリについて `get_carla_pedestrian_transform()`/
     `get_carla_velocity()` を計算し `self.carla.synchronize_pedestrian()` を呼ぶ。
   - 20 tick毎に `self.vissim2carla_ped_ids` のサイズと中身をdebugログ出力(既存の signal sync ログと
     同じ様式)。
4. **`close()` への追加**: 「Destroying synchronized actors」ループの直後に
   ```python
   for carla_walker_id in self.vissim2carla_ped_ids.values():
       self.carla.destroy_actor(carla_walker_id)
   ```
   を追加。

### タスクP6: `carla_autoware.py` の `_cleanup_vissim()` への歩行者cleanup追加

現行の `_cleanup_vissim()`([carla_autoware.py](../src/autoware_carla_interface/carla_autoware.py#L323)、
本編計画Step 7でtry/except方式に書き換え済み)は `SimulationSynchronization.close()` の内容を
「1ステップずつ独立したtry/exceptで再実装」したものであるため、`close()` 側にタスクP5で追加した
歩行者destroyループも、同じ方式でこの関数に追加する必要がある。

- 「Destroy CARLA actors mirrored from vissim (`vissim2carla_ids`)」ループの直後に、
  `self.vissim_sync.vissim2carla_ped_ids.values()` に対する同型のtry/exceptループを追加
  (1体ずつ独立させる方針を踏襲。1体の破棄失敗が他のクリーンアップを止めないようにする)。
- 関数のdocstring内の「Order:」一覧にも歩行者destroyの記載を追加する。

### タスクP7: `NOTICE.md` の更新

- 冒頭のファイル一覧に `data/ptypes.json` を追加。
- 「Deviations from the upstream files」に以下を追記:
  - `vissim_simulation.py`: 歩行者関連の追加はアップストリームと同一内容でvendor化(既存の
    「module-levelデバッグブロック削除」という逸脱以外に新規の逸脱は無し)。
  - `carla_simulation.py`: walkerの actor diff 計算をアップストリームの `tick()` ではなく
    `update_actor_diff()` 側に配置(既存の分割方針(タスクP3参照)に合わせるための逸脱)。
  - `simulation_synchronization.py`: 歩行者同期を `tick()` ではなく `sync_vissim_to_carla()` 側に配置
    (vissim→carla方向のみのため、既存の分割方針にそのまま合致。逸脱というより自然な帰結)。
  - `bridge_helper.py`: 歩行者関連メソッド追加は無変更vendor化の対象から「歩行者メソッドを除く」に更新。
- 「Not vendored」一覧に `run_synchronization.py` の
  `self.carla.world.set_weather(carla.WeatherParameters.ClearNoon)`(0.3節、要ユーザー確認)を追記。

### タスクP8: 動作検証

- **スタブテスト**: アップストリームの `util/pedestrian_sync_stub_test.py`
  (fakeVissimSimulation/fakeCarlaSimulation による実サーバ不要の検証)を参考に、本リポジトリの
  `vissim_integration` パッケージ構成(`SimulationSynchronization` のimportパス、
  `CarlaSimulation.__init__(client, world)` のシグネチャ差)に合わせて移植する。
  本リポジトリには現状 `vissim_integration` 用のテストが無いため、これが最初のテストになる。
- **実機テスト**: 本編計画Step 6と同じ要領で、CARLA + Vissim Kernelを実際に起動し、
  歩行者を含む `.inpx` ネットワークで以下を確認する:
  - vissimの歩行者がCARLA上にwalkerとしてスポーンされること。
  - 歩行者の消滅(destroy)がCARLA側でも反映されること。
  - `_cleanup_vissim()` 実行後、CARLA上にwalkerアクターが残留しないこと。
  - 既存の車両同期・signal同期が歩行者追加によって壊れていないこと(回帰確認)。

### タスクP9: ドキュメント更新

- `README.md` に歩行者同期の説明を追加(「vissim→carla方向のみ・常時有効・`ptypes.json` 依存」の3点)。
- `docs/Vissim-CARLA-Autoware_co-sim_起動手順.md`(canonical起動手順)に、歩行者を含むネットワークを
  使う場合の注意点があれば追記。
- 本編計画 `Vissim_CARLA_Autoware_統合_実装計画_v1.0.md` からも本ドキュメントへの相互参照を追加するかは
  ユーザー判断(本編計画のスコープに含めるか、独立ドキュメントのままにするか)。

---

## 3. 実装ステップ案(本編計画のStep分けスタイルに合わせる場合)

本編計画のStep 0〜8に続ける形で番号を振るなら、以下のような分割が自然である
(実装着手はユーザーの個別指示を待つ)。

- **Step P1**: `data/ptypes.json` vendor化 + `vissim_simulation.py` 歩行者取得ロジック追加(タスクP1・P2)
  — **完了(2026-09-03、`feature/vissim_co-sim`ブランチ)**
- **Step P2**: `carla_simulation.py` / `bridge_helper.py` への歩行者変換・walker管理追加(タスクP3・P4)
  — **完了(2026-09-03、`feature/vissim_co-sim`ブランチ)**
- **Step P3**: `simulation_synchronization.py` への同期組み込み(タスクP5)— ここでvissim→carla方向の
  歩行者同期が一通り動作するようになる想定
- **Step P4**: `carla_autoware.py` `_cleanup_vissim()` への歩行者cleanup追加(タスクP6)
- **Step P5**: NOTICE.md更新 + スタブテスト移植(タスクP7・P8前半)
- **Step P6**: 実機検証 + ドキュメント更新(タスクP8後半・P9)

### Step P1実施内容(2026-09-03)

- `data/ptypes.json` をアップストリームからバイト同一でvendor化(`diff` で無差異を確認済み)。
- `vissim_simulation.py` に以下を追加(いずれもアップストリームと同一内容。差分は既存の逸脱
  「module-levelデバッグブロック削除」のみで、歩行者関連コードそのものに新規の逸脱は無い):
  - `VissimPedestrianMotionState`(19値)/`VissimPedestrianConstructionElementType`(5値)enum。
  - `VISSIM_Ped_Data` ctypes構造体(18フィールド)。
  - `VissimPedestrian` クラス(`get_velocity()`/`get_transform()`)。
  - `PTVVissimSimulation.__init__` に `self._vissim_pedestrians = {}` /
    `self.spawned_pedestrians = set()` / `self.destroyed_pedestrians = set()` を追加。
  - `_declare_prototypes()` に `VISSIM_GetTrafficPedestrians` のargtypes/restype宣言を追加。
  - `get_pedestrian(pedestrian_id)` アクセサを追加。
  - `tick()` に、車両の spawned/destroyed 計算の直後・signal取得の直前として、
    `VISSIM_GetTrafficPedestrians` 呼び出し・`VissimPedestrian` 構築・IDセット差分による
    `spawned_pedestrians`/`destroyed_pedestrians` 計算・20 tick毎のサンプルログを追加。
- `NOTICE.md` のファイル一覧・deviations節を更新(`data/ptypes.json` を対象ファイルに追加、
  `vissim_simulation.py` の歩行者追加が無変更vendorであることを明記)。
- 検証: `python3 -m py_compile vissim_simulation.py` で構文確認、`get_errors` でlint確認、
  いずれも問題なし。CARLA/Vissim Kernel未接続のため実行時動作確認は次StepまたはStep P6で実施。

### Step P2実施内容(2026-09-03)

- `carla_simulation.py` に以下を追加(アップストリームと同一内容。唯一の差分は、walkerのactor diff
  計算をアップストリームの `tick()` ではなく本リポジトリの `update_actor_diff()` 側に配置した点
  — Step 4で確立済みの `tick()`/`update_actor_diff()` 分割方針に合わせるための意図的な逸脱):
  - `__init__` に `self._active_walkers`/`self.spawned_walkers`/`self.destroyed_walkers` を追加。
  - 新規メソッド `synchronize_pedestrian(walker_id, transform, velocity=None)` を追加
    (`synchronize_vehicle()` と同じ `set_transform()`/`set_target_velocity()` 方式)。
  - `update_actor_diff()` に `walker.pedestrian.*` フィルタによるwalker diff計算を追加。
- `bridge_helper.py` に以下を追加(アップストリームと同一内容):
  - クラス変数 `ptypes = {}`。
  - 新規メソッド `get_carla_pedestrian_transform(vissim_pedestrian)`(walkerのバウンディングボックス
    中心原点補正、`extent[2] / 2.0` のZ加算)。
  - 新規メソッド `get_carla_pedestrian_blueprint(vissim_pedestrian)`(`ptypes` からのblueprint選択、
    `role_name` 設定)。
- `NOTICE.md` を更新: `carla_simulation.py` の逸脱に walker diff 配置の説明を追記、`bridge_helper.py`
  を「無変更vendor」グループから分離し、追加されたメソッドを明記。
- 検証: `python3 -m py_compile carla_simulation.py bridge_helper.py` で構文確認、`get_errors` で
  lint確認、いずれも問題なし。両ファイルをアップストリームと `diff` し、既知の逸脱(client/world注入、
  tick()/update_actor_diff()分割、provenanceヘッダ)以外の差分が無いことを確認済み。

---

## 4. ユーザー確認が必要な事項

1. **ブランチ方針**: `feature/vissim_co-sim`(Step 8のtry/except強化を含まない現行版)と
   `feat/vissim_co-sim_step8`(Step 8込み)のどちらを土台に歩行者同期を実装するか。
2. **天候設定の移植可否**: アップストリームに追加されている
   `self.carla.world.set_weather(carla.WeatherParameters.ClearNoon)`(PEDESTRIAN_TODO.mdに記載のない
   変更、0.3節参照)を一緒に移植するか、見送るか。
3. **Step番号の割り振り**: 上記Step P1〜P6を本編計画の追加Stepとして本編ドキュメントに統合するか、
   本ドキュメントを独立したStep計画として管理するか。
