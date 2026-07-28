# SUMO–CARLA–Autoware統合 修正項目一覧

作成日: 2026-07-28
版: v0.5(検討済み全体フロー図(初期化・メインループ)のレビューを反映し、v0.4の記述を更新)
対象方針: `autoware_carla_interface` を主処理とし、CARLA公式SUMO–CARLAブリッジの同期処理をライブラリ化して組み込む。

## 0. v0.3 → v0.4 での変更点

**訂正**: v0.3「3. その他の細かい指摘」で「リポジトリ内を確認した限りSUMO/TraCI関連の既存コードは存在しない」としたのは誤り。`/home/divp/CARLA/Co-Simulation/Sumo` に**動作確認済みのSUMO-CARLA連携コード一式**が存在する(公式ブリッジをベースに、EGOトラッキング等のカスタマイズが加えられている)。以下、実コードを確認して判明した事実に基づき、v0.3の想定・要確認事項の多くを「確認済み」に格上げし、内容を訂正した。

### 確認できたリポジトリ構成

```text
/home/divp/CARLA/Co-Simulation/Sumo/
├── run_synchronization.py       # 公式ブリッジの同期ループ本体(CLIエントリポイント)
├── spawn_ego_carla.py           # 【カスタム追加】EGOをCARLA側にのみ生成するスクリプト
├── dbg_mon_tls.py               # 【カスタム追加】信号機デバッグ用定点カメラ
├── spawn_npc_sumo.py            # 公式: SUMO管理NPCの生成
├── sumo_integration/
│   ├── carla_simulation.py      # CarlaSimulationクラス(CARLA側ラッパー)
│   ├── sumo_simulation.py       # SumoSimulationクラス(TraCIラッパー)
│   ├── bridge_helper.py         # BridgeHelperクラス(座標変換・vtype変換)
│   └── constants.py
├── data/vtypes.json             # CARLA blueprint ⇔ SUMO vtype 対応表
├── examples/                    # Town01/04/05用の.sumocfg一式
└── SUMO-CARLA_Co-Simulation実行手順.md  # 動作確認済み手順書(カスタム追加)
```

`SUMO-CARLA_Co-Simulation実行手順.md` によれば、実際の動作確認は次の4〜5プロセス構成で行われている(**重要**: 現状は1プロセスではなく、複数プロセスがそれぞれ独立して`carla.Client`を生成するマルチクライアント構成)。

1. `CarlaUE4.sh`(CARLAサーバー)
2. `config.py --map Town01`(マップ設定)
3. `run_synchronization.py examples/Town01.sumocfg --sumo-gui --step-length 0.1 --tls-manager sumo --sync-vehicle-lights --sync-vehicle-color --track-ego`(SUMO同期)
4. `spawn_ego_carla.py --spawn-point-index 0 --follow`(EGO生成、CARLA側のみ・Traffic Manager自動運転)
5. (任意)`dbg_mon_tls.py`(信号機デバッグカメラ)

> **v0.4での方針転換**: 統合後は上記2〜4のプロセスが行っていることを、すべて `autoware_carla_interface` の単一プロセス・単一 `carla.Client` の中に集約する必要がある。これは既存コードの「移植」ではなく「マルチプロセス構成からライブラリ組み込みへの再構築」である点を明確にした。

### 主な訂正・新規判明事項(詳細は各章)

1. **【訂正】2.3**: SUMO起動・TraCI接続処理(`SumoSimulation.__init__`)は実装済み・動作確認済み。移植ではなく「流用」。
2. **【訂正】2.4/2.5**: `CarlaSimulation.__init__` は独自に `carla.Client(host, port)` を生成している(コード確認済み)。既存Client/World注入への改修が**必須**であることが確定した(想定ではなく事実)。
3. **【訂正】2.6**: `SimulationSynchronization.__init__` が `world.apply_settings()`(同期モード・`fixed_delta_seconds`)と `traffic_manager.set_synchronous_mode(True)` を**実際に呼んでいる**ことをコードで確認。この重複排除は必須。
4. **【重要・新規】2.7/3.2-1**: `CarlaSimulation.spawn_actor()` は `client.apply_batch_sync(batch, False)` で**明示的にtickしない**実装になっている。v0.3で懸念した「暗黙tick」問題は、`autoware_carla_interface` 側の `CarlaDataProvider.request_new_actor()` を使わず、**この既存 `CarlaSimulation.spawn_actor()`/`destroy_actor()` をそのまま流用すれば発生しない**ことが判明。対応方針を「注意する」から「どちらの経路を使うか設計判断する」に更新。
5. **【重要・新規】2.10/3.4/3.7/3.8**: `SimulationSynchronization.tick()` の「carla-->sumo sync」処理には、**未登録のCARLA車両アクターを自動検出してSUMO側に対応車両を生成し、`traci.vehicle.moveToXY(..., keepRoute=2)` で毎tick位置を強制反映する「自動アダプト」機構が既に実装されている**ことを確認。これはEGOをSUMOへ登録する処理(2.10、新規処理としていた項目)の大部分を代替できる可能性が高い。ただし副作用(2.11参照)がある。
6. **【新規】2.11の補強**: 上記の自動アダプト機構は `vehicle.*` 型の**あらゆる**CARLAアクターに対して無差別に働く。そのため `use_traffic_manager=True` でTraffic Manager経由のNPCが存在する場合、それらも意図せず全てSUMOへ登録されてしまう。2.11の排他制御は「望ましい」ではなく「自動アダプトを正しく機能させるための必須前提」に格上げする。
7. **【訂正】2.8**: 座標変換(`BridgeHelper.get_carla_transform`/`get_sumo_transform`、SUMO-CARLA間offsetおよび座標系変換)は実装済み・動作確認済み。「移植」というより「そのまま流用可能」。
8. **【新規】2.10**: `spawn_ego_carla.py` の現状のEGOは **CARLA側のみに存在しTraffic Manager自動運転で走る車両**であり、SUMOには登録されていない(スクリプトのコメントに明記)。`run_synchronization.py --track-ego` のSUMO GUIカメラ追従は、この自動アダプト機構によって色一致した車両をSUMO側で検出しているものと推定され、EGO⇔SUMO双方向同期の「配管」自体は機能している傍証と考えられる。ただし、現状はAutowareではなくTraffic Managerが運転しているため、「Autoware制御のEGOに対しても同じ経路で正しく動作するか」は改めてエンドツーエンドで検証が必要。
9. **【新規】4章/5章**: 上記を反映し、区分表・優先度を更新。

---

## 0.5 v0.4 → v0.5 での変更点

検討済みの全体フロー図(初期化処理・メインループ処理、2026-07-28レビュー分)を確認し、以下を追記・更新した。

1. **【決定事項・2.2を更新】ステップ時間**: 7/27デイリーにて、`fixed_delta_seconds`/`step_length` は**0.1秒に統一する**ことが決定した。SUMO側ではなく**`autoware_carla_interface` 側の `fixed_delta_seconds` をデフォルト0.05秒→0.1秒に変更する**(動作確認済みの実行手順書がSUMO側 `--step-length 0.1` を使っていたことに合わせる形)。2.2の「どちらの値を採用するか決定する」は解消。
2. **【確認・2.0を新設】初期化処理の実行順序が確定**: レビュー済みフロー図により、初期化処理の順序は次の通り確定した(詳細は新設した2.0参照): CARLA Clientに接続 → CARLAマップ読み込み → SUMO起動・TraCI接続 → SUMO同期エンジン作成(CARLA Client/World設定・ID対応表/座標変換初期化) → CARLA EGOスポーン → EGOをSUMOへ登録 → CARLAセンサースポーン。
3. **【要決定事項として2.10に追記】EGOをSUMOへ登録するタイミング**: レビュー済みフロー図では「EGOをSUMOへ登録」が**初期化処理内の明示的な単独ステップ**として描かれている。一方、本ドキュメント2.10で確認した通り、実コード上でEGOがSUMOへ反映される主経路は**メインループの「CARLA→SUMO同期」内の自動アダプト機構(毎tick実行)**である。両者は役割が重複しうるため、「初期化時に明示的にTraCI APIで登録する処理を新設するのか」「自動アダプトに任せ、初期化フロー図上の当該ステップは概念的な表現(実体はメインループ1回目のtickで発生)に留めるのか」を設計決定として明確にする必要がある。
4. **【要確認・新規】3.4/3.7とのラベル対応**: メインループ図の「SUMO→CARLA同期」「CARLA→SUMO同期」それぞれに付随する注記(NPC状態更新・信号状態更新・EGO状態更新)は、`SimulationSynchronization.tick()`の実装(本ドキュメント3.4/3.5/3.7)と次のように対応させること: **SUMO→CARLA同期(sumo-->carla sync)** = CARLA上のNPC状態を更新 + (`tls_manager=='sumo'`の場合)CARLA側信号状態を更新。**CARLA→SUMO同期(carla-->sumo sync)** = SUMO上のEGO/車両状態を更新 + (`tls_manager=='carla'`の場合)SUMO側信号状態を更新。フロー図上でこの対応が入れ替わっていないか要確認。

---

## 1. 結論

修正対象は大きく次の2領域である。

1. **初期化処理**
   SUMOの起動・TraCI接続・同期エンジン生成(いずれも実装済みコードを流用)を組み込み、CARLA側で生成したEGOをSUMOへ登録する。
2. **メインループ処理**
   `autoware_carla_interface` のループを唯一の主ループとして残し、`SUMO Tick → SUMO→CARLA同期 → CARLA Tick → CARLA→SUMO同期` を組み込む。

最重要事項は、**CARLA Tickを1か所に限定すること**である。

> `sumo_integration` 側の実装(`SimulationSynchronization.tick()`)は、内部で `self.carla.tick()`(= `world.tick()`)を1回だけ呼ぶ設計になっており、単体では「1tick/step」の原則を守っている。この原則が崩れうるのは、**この既存実装をそのまま使わず、`autoware_carla_interface` 側の別のアクター生成手段(`CarlaDataProvider.request_new_actor()` 等、デフォルトで暗黙tickする)に置き換えてしまった場合**である。したがって設計判断としては「既存の `CarlaSimulation`/`SumoSimulation`/`BridgeHelper` の生成・同期メソッド群をできる限りそのまま流用し、`autoware_carla_interface` 独自のアクター生成APIとは混在させない」ことを原則とする。

---

## 2. 初期化処理の修正項目

### 2.0 初期化処理の実行順序(確定・2026-07-28)

レビュー済みの全体フロー図により、初期化処理の実行順序は次の通り確定した(3.1の主ループ順序に相当する、初期化版のシーケンス)。

```text
CARLA Clientに接続
  ↓
CARLAマップ読み込み
  ↓
SUMO起動・TraCI接続
  ↓
SUMO同期エンジン作成(CARLA Client/Worldを設定・ID対応表/座標変換を初期化)
  ↓
CARLA EGOスポーン(spawn_pointの座標に従ってEGOをスポーン)
  ↓
EGOをSUMOへ登録
  ↓
CARLAセンサースポーン
```

CARLA接続・マップ読み込みを先に行い、その後にSUMO起動・TraCI接続・同期エンジン作成(2.3/2.4)を行う点、EGOスポーン(2.9)の後に「EGOをSUMOへ登録」(2.10)を独立ステップとして置く点は本ドキュメントの記述順序と整合する。ただし「EGOをSUMOへ登録」を初期化時の明示ステップとして扱うか、メインループの自動アダプト任せにするかは2.10の追記の通り要決定。

### 2.1 SUMO関連パラメータを追加する

`autoware_carla_interface.launch.xml` などから、少なくとも以下を指定できるようにする。

- SUMO設定ファイル(`.sumocfg`)
- SUMO実行ファイルまたはSUMO-GUI使用有無(`--sumo-gui`)
- SUMOホスト／TraCIポート(`--sumo-host`/`--sumo-port`)
- `step_length`(動作確認済み実行例では `0.1` を使用。コード側デフォルトは `0.05`)
- 信号制御元(`--tls-manager {none, sumo, carla}`。動作確認済み実行例では `sumo`)
- 車両ライト同期有無(`--sync-vehicle-lights`。動作確認済み実行例では有効)
- 車体色同期有無(`--sync-vehicle-color`。動作確認済み実行例では有効)
- SUMO機能の有効／無効
- TraCIクライアント順序(`--client-order`。複数TraCIクライアントが同時接続する場合に必要)

これらは全て `run_synchronization.py` の既存CLI引数としてそのまま対応関係があるため、launchファイルの `<arg>` は極力この既存引数名・デフォルト値に合わせて追加する。

### 2.2 ステップ時間を統一する

`autoware_carla_interface` の `fixed_delta_seconds` を、SUMOの `step_length` に合わせる。

- `autoware_carla_interface` launchファイルの `fixed_delta_seconds` デフォルト: **0.05秒**
- `sumo_integration` コード側の `--step-length` デフォルト: **0.05秒**
- ただし動作確認済みの実行手順書では **0.1秒** を明示指定して検証している

> **【決定・2026-07-27デイリー】** `autoware_carla_interface` 側の `fixed_delta_seconds` を**デフォルト0.05秒→0.1秒に変更する**ことで統一する(SUMO側の `step_length` は動作確認済み手順書の値である0.1秒のまま変更しない)。launchファイルのデフォルト値変更、および既存パラメータドキュメント(README等)への反映が必要。

### 2.3 SUMO起動・TraCI接続処理を組み込む

`SumoSimulation.__init__`(`sumo_integration/sumo_simulation.py`)には以下がすでに実装・動作確認済みである。

- `sumolib.checkBinary()` によるSUMO/SUMO-GUIバイナリ検出
- `host`/`port` 未指定時は `traci.start([...])` でSUMOプロセス自体を起動、指定時は `traci.init(host, port)` で既存SUMOサーバーへ接続
- `traci.setOrder(client_order)` によるTraCIクライアント順序設定
- `.sumocfg` からのnetファイル読み込み(`_get_sumo_net()`)
- 信号機管理用 `SumoTLManager` の初期化

これは「移植」ではなく、`SumoSimulation` クラスを `autoware_carla_interface` から**そのままインポートして利用する**形で組み込むのが現実的である。接続失敗時の例外処理(`traci.exceptions.TraCIException` 等)は呼び出し側で追加する。

### 2.4 SUMO同期エンジン生成処理を組み込む

`SimulationSynchronization`(`run_synchronization.py` 内に定義)は独立したクラスとして実装済みで、以下を担う。

- `SumoSimulation`
- `CarlaSimulation`
- `BridgeHelper`

いずれも既存モジュールとして流用できるが、**`CarlaSimulation.__init__` が独自に `carla.Client(host, port)` を生成し、`SimulationSynchronization.__init__` が独自に同期モード設定を行っている**(2.5/2.6参照)ため、そのままでは `autoware_carla_interface` のCARLA接続一元化と衝突する。移植時は以下の改修が必須:

- `CarlaSimulation.__init__` を、外部から `client`/`world` を注入できるコンストラクタ(または `classmethod`)に変更する
- `SimulationSynchronization.__init__` 内の `world.apply_settings()` / `traffic_manager.set_synchronous_mode()` 呼び出しを削除する(2.6)

### 2.5 CARLA接続を一元化する

CARLAの以下のオブジェクトは、`autoware_carla_interface` が生成・管理する。

- `carla.Client`
- `carla.World`
- 必要に応じて `TrafficManager`
- CARLA同期モード
- `fixed_delta_seconds`

現行 `autoware_carla_interface` では `InitializeInterface.load_world()` がこれらを一括管理している。一方、確認済みSUMOブリッジ側の `CarlaSimulation.__init__` は次の通り**独自に接続を生成している**(コード確認済み)。

```python
class CarlaSimulation(object):
    def __init__(self, host, port, step_length):
        self.client = carla.Client(host, port)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        ...
```

このため統合時は、`CarlaSimulation` のコンストラクタを改修し、`InitializeInterface.load_world()` で生成済みの `client`/`world`(`CarlaDataProvider.get_client()`/`get_world()`)を注入する形に変更する。

### 2.6 CARLA同期モード設定の重複を削除する

`SimulationSynchronization.__init__` は実際に以下を実行している(コード確認済み)。

```python
settings = self.carla.world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = self.carla.step_length
self.carla.world.apply_settings(settings)

traffic_manager = self.carla.client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)
```

これは `autoware_carla_interface`(`InitializeInterface.load_world()`)側で既に行っている設定と完全に重複する。統合時はこのブロックを丸ごと削除し、`autoware_carla_interface` 側の設定のみを有効にする。

### 2.7 ID対応表を初期化する

`SimulationSynchronization` は既に以下のID対応表を保持している(コード確認済み、実装移植は不要でそのまま流用可能)。

- `self.sumo2carla_ids`: SUMO車両ID → CARLA Actor ID(SUMO正本の車両)
- `self.carla2sumo_ids`: CARLA Actor ID → SUMO車両ID(CARLA正本の車両。EGOはここに入る想定)
- 信号ID対応は `BridgeHelper`/`SumoTLManager` の `traffic_light_ids`(landmark_id基準の共通集合)で管理

追加が必要なのは、Lanelet2交通信号IDとの対応表のみ(公式ブリッジには存在しない、Autoware固有の追加要素)。

> **アクター生成時の注意**: `self.carla.spawn_actor()`(= `CarlaSimulation.spawn_actor()`)は内部で `client.apply_batch_sync(batch, False)` を使っており、**明示的に暗黙tickをしない**実装になっている。SUMO由来車両の生成・削除は、この既存メソッドをそのまま使う(= `CarlaDataProvider.request_new_actor()` 等の別経路を新たに使わない)ことで、3.2の「CARLA Tickを1か所に限定する」との整合を保つ。

### 2.8 座標変換情報を初期化する

`BridgeHelper.get_carla_transform()` / `get_sumo_transform()`(`sumo_integration/bridge_helper.py`)に、以下が実装・動作確認済みである。

- SUMO⇔CARLA間のネット座標オフセット補正(`BridgeHelper.offset`、`SumoSimulation.get_net_offset()` から取得)
- 前方バンパー基準⇔車両中心基準の補正(`extent.x` を用いた位置シフト)
- 左手系(CARLA)⇔右手系(SUMO)の座標変換、Y軸反転
- yaw角の座標系変換(SUMO角度からCARLA角度への `-yaw + 90` 変換など)

これは移植ではなく**そのまま流用可能**。`autoware_carla_interface` に既存の `modules/coordinate_transformer.py`(Autoware/CARLA間の変換)とは別レイヤーであり、両者を混同しないよう役割を明確にドキュメント化すること(SUMO⇔CARLAは`BridgeHelper`、CARLA⇔Autowareは`coordinate_transformer.py`)。

### 2.9 EGOをCARLAへスポーンする

この処理は既存の `autoware_carla_interface` の処理を基本的に維持する。

- `spawn_point` に基づくEGO生成
- EGO Actor IDの取得
- Autoware制御対象として登録

現行実装(`InitializeInterface.load_world()` → `_parse_spawn_point()`)では、`spawn_point` パラメータのパースに失敗した場合はランダム配置にフォールバックする。2.10でSUMO側に初期位置を反映する際は、パラメータ上の `spawn_point` 文字列ではなく、実際にスポーンされた `ego_actor.get_transform()` の値を使用すること。

### 2.10 EGOをSUMOへ登録する

CARLA側で生成したEGOに対応する車両を、SUMO側へ登録する。

> **重要な新規判明事項**: `SimulationSynchronization.tick()` の「carla-->sumo sync」処理(後述3.7参照)には、**未登録のCARLA車両アクターを自動検出し、SUMO側へ対応車両を自動生成・自動同期する仕組みが既に実装されている**。具体的には:
>
> 1. `CarlaSimulation.tick()` が `world.get_actors().filter('vehicle.*')` の差分から新規車両を検出する
> 2. `self.carla2sumo_ids` に未登録の車両に対し、`BridgeHelper.get_sumo_vtype(carla_actor)` でSUMO vtypeを解決する(`data/vtypes.json` に未登録のCARLA blueprintでも、`_create_sumo_vtype()` がバウンディングボックス等からその場でvtypeを自動生成するフォールバックを持つ)
> 3. `self.sumo.spawn_actor(type_id, color)` でSUMO側に車両を生成(`traci.vehicle.add()` + ダミールート)
> 4. 以降毎tick、`self.sumo.synchronize_vehicle()` → `traci.vehicle.moveToXY(vehicle_id, "", 0, x, y, angle=yaw, keepRoute=2)` でCARLA側の実際のtransformをSUMO側へ強制反映する(`keepRoute=2` によりSUMOの通常の経路追従・レーン制約を無視して自由配置できる)
>
> これは、2.10で「新規処理」としていた「EGOをSUMOへ登録し、SUMO側では外部制御下に置く」という要件の**大部分を、既存コードのままカバーできる可能性が高い**ことを意味する。実際、動作確認済みの `run_synchronization.py --track-ego` オプションは、この自動アダプト機構によってSUMO側に生成されたEGO相当車両を色一致で検出し、SUMO GUIカメラを追従させるものと推定される(「EGOはCARLA側のみに存在しSUMOには登録されない」という `spawn_ego_carla.py` のコメントと矛盾するように見えるが、これは「明示的なTraCI登録コードを書いていない」という意味であり、実体は上記の自動アダプト機構経由でSUMOに反映されていると考えられる)。

必要な追加確認・処理(新規処理として残るもの):

- EGOがAutowareによって制御される場合も、上記の自動アダプト経路が同様に機能するかをエンドツーエンドで検証する(現状の動作確認はTraffic Manager運転のEGOでのみ行われている)
- 自動アダプトによって生成されるSUMO車両の**初期出現タイミング**(EGOスポーン直後の最初の `tick()` で拾われるか)を確認する
- 自動アダプトに任せず明示的にTraCI APIで登録したい場合(例: 特定のSUMO vTypeを強制したい、出現タイミングを制御したい等)は、`self.sumo.spawn_actor()` を直接呼び出す形で明示登録するオプションも用意する

> **【要決定・2026-07-28追記】** レビュー済みの全体フロー図(2.0参照)では「EGOをSUMOへ登録」が**初期化処理内の独立した明示ステップ**として描かれている。これは、本項で確認した「メインループの`CARLA→SUMO同期`内の自動アダプト機構が毎tick自動的にEGOをSUMOへ反映する」という実装済みの仕組みと役割が重複する。フロー図の当該ステップが (a) 自動アダプトに頼らず`self.sumo.spawn_actor()`等で明示的に登録する処理を指しているのか、(b) 自動アダプトによって初回tick時に登録される様子を初期化フロー上の概念的な位置に置いているだけなのか、を設計者に確認し、どちらの方針かをこのドキュメントに明記すること。

### 2.11 CARLA側のランダムNPC生成を見直す【重要度: 必須に格上げ】

NPCをSUMO正本とする場合、`autoware_carla_interface` 側のランダムNPC生成は原則無効化する。

現行実装では `use_traffic_manager` パラメータ(デフォルト `False`)がTraffic Manager経由のNPC自動生成(`_setup_traffic_manager()`)を制御しており、デフォルトでは既に無効化されている。

> **2.10で判明した自動アダプト機構により、この排他制御は「望ましい」ではなく「必須」である。** 自動アダプトは `vehicle.*` 型の**あらゆる**CARLAアクターを無差別にSUMOへ登録する。したがって `use_traffic_manager=True` でTraffic Manager経由のNPCが存在すると、それらも全て意図せずSUMOへ登録され、SUMO側のNPC(正本)とCARLA Traffic Manager側のNPC(意図せず複製)が二重に存在することになる。

対応:

- SUMO使用時に `use_traffic_manager=True` が同時指定された場合、起動時にエラーで停止する排他チェックを追加する

### 2.12 センサー生成処理を維持する

既存の以下の処理は基本的に維持する。

- `sensor_kit_calibration.yaml` の読み込み
- `sensor_mapping.yaml` の読み込み
- Camera／LiDAR／IMU／GNSSの生成
- EGOへのアタッチ
- センサーコールバック登録

### 2.13 初期化失敗時の終了処理を追加する

途中で初期化に失敗した場合、生成済みリソースを解放する。

- SUMOプロセス終了(`SumoSimulation.close()` = `traci.close()`)
- TraCI切断
- 生成済みCARLAアクター削除
- ROS 2ノード終了
- CARLA設定の復元

`SimulationSynchronization.close()` には、同期モード解除・`sumo2carla_ids`/`carla2sumo_ids` に登録された全アクターの破棄・`carla.close()`/`sumo.close()` が既に実装済みである。統合時はこの `close()` を `autoware_carla_interface` 側の `InitializeInterface._cleanup()` の枠組みに組み込み、二重のクリーンアップ経路を作らないこと。

### 2.14 SUMO由来アクターの管理統合

SUMO→CARLA同期(3.4)で生成するNPCアクターの破棄は、`SimulationSynchronization.close()` の `sumo2carla_ids`/`carla2sumo_ids` 一括破棄ロジックをそのまま利用できる見込みが高い。`CarlaDataProvider` 側の管理(`_carla_actor_pool` 等)に二重登録する必要があるかどうかは、`autoware_carla_interface` の他の機能(センサーアタッチ判定等)がこれらのNPCアクターを参照するかどうかに依存するため、実装時に確認する(6章参照)。

---

## 3. メインループ処理の修正項目

### 3.1 主ループを `autoware_carla_interface` に一本化する

`run_synchronization.py` の独立した `while True:` ループ(`synchronization_loop()` 内)は使用しない。この関数は「1ステップあたりの実行時間を計測し `step_length` に満たない分だけ `time.sleep()` する」という、`autoware_carla_interface` の `max_real_delta_seconds` ペーシングと類似した仕組みを持っている(3.2-2参照)。

統合後の基本順序:

```text
センサーデータ取得・配信
  ↓
ROS 2コールバック処理
  ↓
Autoware制御指令をCARLA EGOへ適用
  ↓
SUMO Tick
  ↓
SUMO→CARLA同期
  ↓
CARLA Tick
  ↓
CARLA→SUMO同期
  ↓
次ループ
```

現行 `autoware_carla_interface` の「制御適用→Tick」の順序は既にこの想定と一致している。

### 3.2 CARLA Tickを1か所に限定する

残す処理:

- `autoware_carla_interface` 側のCARLA Tick

削除または無効化する処理:

- `SimulationSynchronization.tick()` 内で呼ばれる `self.carla.tick()`(= `CarlaSimulation.tick()` 内の `world.tick()`)を、独立した呼び出しにせず、`autoware_carla_interface` の主ループが呼ぶ唯一の `world.tick()` に置き換える(`CarlaSimulation.tick()` を「world.tick()を呼ぶ版」と「呼ばない版(既にtick済みの前提でアクター差分更新だけ行う版)」に分離するなどの改修が必要)

1ループにつきCARLA Tickは1回だけ実行する。

#### 3.2-1 既存ヘルパーの暗黙tickの扱い(v0.4で判明した事実に基づき更新)

- `sumo_integration.carla_simulation.CarlaSimulation.spawn_actor()` は `client.apply_batch_sync(batch, False)` を使っており、**暗黙tickをしない**(コード確認済み)。SUMO由来車両の生成・削除はこのメソッドをそのまま流用する限り、暗黙tick問題は発生しない。
- 一方、`autoware_carla_interface` 側の `modules/carla_data_provider.py` の `CarlaDataProvider.request_new_actor()` / `handle_actor_batch()` は、デフォルト引数 `tick=True` の場合に暗黙的に `world.tick()` を呼ぶ(v0.3で指摘済み、引き続き有効な注意点)。
- **設計判断**: SUMO由来アクターの生成・削除には `sumo_integration.CarlaSimulation.spawn_actor()`/`destroy_actor()` を使用し、`CarlaDataProvider.request_new_actor()` 系のAPIとは混在させない。これにより暗黙tickのリスクを設計上回避する。

#### 3.2-2 実時間ペーシングとの相互作用を確認する

`autoware_carla_interface` の `run_bridge()` (`max_real_delta_seconds` によるペーシング)と、`run_synchronization.py` の `synchronization_loop()` (`step_length` に基づく同様のペーシング)は、**類似目的の重複した仕組み**である。統合時は `autoware_carla_interface` 側のペーシングのみを残し、`run_synchronization.py` 由来のペーシングコードは移植しない。

TraCI通信(SUMO Tick・同期処理)による実処理時間の増加が、`max_real_delta_seconds` ベースのペーシングに与える影響は、統合後に実測して確認する(優先度Bとして扱う)。

### 3.3 SUMO Tick処理を追加する

`SumoSimulation.tick()` は以下を実行する(コード確認済み、そのまま流用可能)。

```python
def tick(self):
    traci.simulationStep()
    self.traffic_light_manager.tick()
    self.spawned_actors = set(traci.simulation.getDepartedIDList())
    self.destroyed_actors = set(traci.simulation.getArrivedIDList())
```

SUMO Tick失敗時の例外処理(`traci.exceptions.FatalTraCIError`等)と終了判定を呼び出し側に追加する。

### 3.4 SUMO→CARLA同期処理を追加する

`SimulationSynchronization.tick()` の前半(sumo-->carla sync)には、以下が実装済みである(そのまま流用可能)。

- SUMOで新規生成された車両をCARLAへ生成(`BridgeHelper.get_carla_blueprint()` → `self.carla.spawn_actor()`。**暗黙tickなし**、3.2-1参照)
- SUMOで削除された車両をCARLAから削除(`self.carla.destroy_actor()`)
- SUMO車両の位置・姿勢をCARLAへ反映(`self.carla.synchronize_vehicle()` = `actor.set_transform()`)
- 車両ライト状態を反映(`sync_vehicle_lights` オプション時)
- SUMO信号状態をCARLA信号へ反映(`tls_manager == 'sumo'` の場合)
- ID対応表(`sumo2carla_ids`)を更新

車体色反映は `BridgeHelper.get_carla_blueprint()` 内、`sync_vehicle_color` オプションに応じて生成時に設定される。

**まとめ**: SUMO→CARLA同期は「CARLA上のNPC状態を更新」+「(`tls_manager=='sumo'`の場合)CARLA側信号状態を更新」を担う処理である。

### 3.5 SUMO由来NPCをCARLA Engineへ直接反映する

3.4に同じ(`self.carla.synchronize_vehicle()` が Transform / LightState を更新。Velocityの直接設定は行っておらず、`set_transform()` による位置強制が実質的な速度表現になっている点に留意)。

### 3.6 CARLA Tickを実行する

`autoware_carla_interface` 側の `world.tick()` を1回実行した後、`CarlaSimulation.tick()` が行っている「`world.get_actors().filter('vehicle.*')` による車両アクター差分検出」のロジックを、tick呼び出しと分離して実行できるよう改修する(3.2参照)。

`fixed_delta_seconds` はSUMOの `step_length`(2.2参照)と一致させる。

### 3.7 CARLA→SUMO同期処理を追加する

`SimulationSynchronization.tick()` の後半(carla-->sumo sync)には、以下が実装済みである(そのまま流用可能)。

- CARLA側で新規生成された(=まだ `sumo2carla_ids`/`carla2sumo_ids` どちらにも未登録の)車両アクターを検出し、SUMOへ自動生成(2.10参照。**EGOもこの経路で自動的に拾われる想定**)
- CARLA側で削除された車両をSUMOから削除(`self.sumo.destroy_actor()`)
- CARLA車両の位置・姿勢をSUMOへ反映(`self.sumo.synchronize_vehicle()` → `traci.vehicle.moveToXY(..., keepRoute=2)`)
- CARLA側で削除された同期対象車両の反映
- 灯火状態の反映(`sync_vehicle_lights` オプション時)
- 信号状態のSUMOへの反映(`tls_manager == 'carla'` の場合)
- ID対応表(`carla2sumo_ids`)の更新

少なくともEGOは毎ステップ同期される(自動アダプト経路に乗っている限り)。

**まとめ**: CARLA→SUMO同期は「SUMO上のEGO/車両状態を更新」+「(`tls_manager=='carla'`の場合)SUMO側信号状態を更新」を担う処理である。

### 3.8 EGOをSUMO側で外部制御する

**確認済み**: `SumoSimulation.synchronize_vehicle()` が使う `traci.vehicle.moveToXY(vehicle_id, "", 0, loc_x, loc_y, angle=yaw, keepRoute=2)` の `keepRoute=2` が、まさに「SUMOの通常の経路追従・車線変更・速度制御を無視して、外部から与えた位置に強制配置する」ためのAPIである。この項目は「要確認事項」から「実装方法確定」に格上げできる。

目的(達成方法が確定):

- SUMO NPCがEGOの存在・位置・速度を認識できるようにする → 自動アダプトにより `carla2sumo_ids` に登録されたSUMO車両として存在するため、他のSUMO車両からは通常のSUMO車両として認識される
- SUMOの車両モデルがEGOを独自制御しないようにする → `moveToXY(..., keepRoute=2)` により実現

### 3.9 センサーデータ処理との時系列を維持する

既存のpublishワーカースレッド(`modules/sensor_publish_worker.py`)は基本的に維持する。変更なし(v0.3から継続)。

### 3.10 EGO車両ステータス配信との整合を確認する

現行実装では `run_step()` が毎tick `/clock` トピックと `self.timestamp` を更新しており、SUMO同期追加後もこの仕組みは変更しない。変更なし(v0.3から継続)。

### 3.11 終了条件を統合する

以下を同一ループ内で判定する。

- ROS 2 shutdown
- CARLA切断
- SUMO終了(TraCI例外、`traci.exceptions.FatalTraCIError`)
- シミュレーション終了ステップ到達
- ユーザー中断

#### 3.11-1 SUMO由来アクターの終了時破棄

`SimulationSynchronization.close()` の一括破棄ロジック(`sumo2carla_ids`/`carla2sumo_ids` に登録された全アクターの破棄)をそのまま流用できる見込みが高い。

### 3.12 終了処理を一元化する

`InitializeInterface._cleanup()`(センサー→ROSインターフェース→EGOアクター→CarlaDataProvider の順)に、`SimulationSynchronization.close()` 相当の処理(同期モード解除・SUMO/CARLA双方の同期アクター破棄・`traci.close()`)を追加する形で統合する。元の `run_synchronization.py` の `finally: synchronization.close()` と `autoware_carla_interface` の終了処理が二重実行されないようにする。

### 3.13 スレッドモデルを明記する

SUMO同期処理(TraCI呼び出しを含む)は **メインスレッド上で、tickループの一部として逐次実行する**こととし、新たなスレッドを追加しない。`autoware_carla_interface` のROS 2 `spin_thread` とは独立させる。変更なし(v0.3から継続)。

---

## 4. 既存処理と新規処理の区分

| 処理 | 区分 | 主な対応 |
|---|---|---|
| SUMO起動・TraCI接続 | **流用**(実装・動作確認済み) | `sumo_integration.SumoSimulation` をそのままインポート |
| SUMO同期エンジン作成 | 流用＋改修 | `SimulationSynchronization`。CARLA再接続・同期モード重複設定を除去 |
| ID対応表 | **流用**(実装済み) | `sumo2carla_ids`/`carla2sumo_ids` |
| 座標変換 | **流用**(実装・動作確認済み) | `BridgeHelper.get_carla_transform`/`get_sumo_transform` |
| CARLA Client／World生成 | 既存維持 | `InitializeInterface.load_world()` が管理。`CarlaSimulation`側は注入方式に改修 |
| CARLA同期モード設定 | 既存維持 | `InitializeInterface.load_world()` が管理。`SimulationSynchronization`側の重複設定を削除 |
| CARLA EGOスポーン | 既存維持 | `InitializeInterface.load_world()` の処理。フォールバック(ランダム配置)挙動に注意 |
| EGOをSUMOへ登録 | **既存の自動アダプト機構を流用できる可能性が高い**(タイミング・明示登録の要否は要決定、2.10参照) | `carla2sumo_ids` への自動登録+`moveToXY(keepRoute=2)` |
| SUMO Tick | **流用**(実装済み) | `SumoSimulation.tick()` を主ループへ組み込み |
| SUMO→CARLA同期 | **流用**(実装済み) | `SimulationSynchronization.tick()` 前半をCARLA Tick前に実行(NPC状態・信号状態(sumo管理時)をCARLAへ反映) |
| CARLA Tick | 既存維持＋改修 | 1か所だけ残す。`CarlaSimulation.tick()` からtick呼び出し部分を分離。`fixed_delta_seconds`は0.1秒に統一(決定済み) |
| CARLA→SUMO同期 | **流用**(実装済み) | `SimulationSynchronization.tick()` 後半をCARLA Tick後に実行(EGO/車両状態・信号状態(carla管理時)をSUMOへ反映) |
| センサー取得・publish | 既存維持 | ワーカースレッドを維持 |
| CARLA側ランダムNPC生成 | 設定変更(**必須**) | `use_traffic_manager` はSUMO使用時に強制OFF・排他エラー化 |
| SUMO由来アクターのライフサイクル管理 | **流用できる可能性が高い** | `SimulationSynchronization.close()` の一括破棄ロジック |
| マルチプロセス構成→単一プロセス化 | **新規(構造変更)** | `spawn_ego_carla.py`/`run_synchronization.py` 相当を`autoware_carla_interface`の1プロセス・1クライアントに集約 |

---

## 5. 優先度

### 優先度A: 必須

- CARLA Tickの一元化(`CarlaSimulation.tick()` からtick呼び出しを分離)
- SUMO／CARLAステップ時間の一致 **【決定済み: 0.1秒に統一、`autoware_carla_interface`側を0.05→0.1秒へ変更(2026-07-27デイリー)】**
- SUMO起動・TraCI接続の組み込み(`SumoSimulation`流用)
- SUMO→CARLA同期の組み込み(`SimulationSynchronization.tick()`前半の流用)
- CARLA Client/World の外部注入(`CarlaSimulation`改修)・同期モード重複設定の削除
- EGOのSUMO自動アダプト経路の動作検証(Autoware制御下での実機検証)、および初期化時の明示登録が必要か否かの設計決定(2.10参照)
- CARLA→SUMOによるEGO状態同期(`moveToXY(keepRoute=2)`、実装方法確定済み)
- 正常終了・異常終了処理の統合(`SimulationSynchronization.close()`との統合)
- `use_traffic_manager` とSUMO NPCの排他制御(自動アダプトの副作用を防ぐため必須)
- スレッドモデルの明記(SUMO同期処理はメインスレッドで実行)

### 優先度B: 動作安定化

- ID対応表の整合性チェック
- 車両生成／削除の例外処理
- SUMOとCARLAの時刻ずれ検出
- ROS 2タイムスタンプとの整合確認
- 実時間ペーシング(`max_real_delta_seconds`)とTraCI通信の相互作用検証(`run_synchronization.py`側の重複ペーシングコードの除去含む)
- SUMO由来アクターを`CarlaDataProvider`の管理下に統合するか independent に扱うかの決定

### 優先度C: 機能拡張

- 信号状態のAutoware向けメッセージ変換(Lanelet2交通信号IDとの対応表を新規追加)
- 車両ライト・車体色同期の設定化
- CARLA生成車両の双方向同期
- 同期状態・遅延・車両数の診断トピック追加
- 自動アダプトに頼らない明示的EGO登録オプションの追加(タイミング制御・vType強制指定用)

---

## 6. 実装時に確認が必要な事項

- `SimulationSynchronization.tick()` を「SUMO Tick+sumo→carla同期」部分と「carla tick+carla→sumo同期」部分の2メソッドに分割改修する際の、既存コードへの影響範囲
- EGOがAutoware制御下にある状態で、自動アダプト機構(2.10/3.7)が意図通り機能するか(特に、EGOスポーン直後の最初の `CarlaSimulation` 差分検出タイミングと、`autoware_carla_interface` 側のEGOスポーンタイミングの前後関係)
- 自動アダプトで生成されたSUMO車両のvType(`data/vtypes.json` に `vehicle.toyota.prius` 等が未登録の場合、`_create_sumo_vtype()` によるフォールバック生成で十分か、専用vTypeを事前定義すべきか)
- SUMO由来アクターを `CarlaDataProvider._carla_actor_pool` に二重登録すべきか(既存の `autoware_carla_interface` の他機能がこれらのNPCを参照する可能性の有無)
- 信号ID対応をOpenDRIVE、SUMO net.xml、Lanelet2のどの情報で管理するか
- `client_order`(TraCI複数クライアント接続時の順序)を、統合後の単一プロセス構成でどう扱うか(単一プロセスなら不要になる可能性が高い)
- CARLA 0.9.15(`autoware_carla_interface`が前提とするバージョン)と、確認済み`sumo_integration`コードの対応バージョンとの互換性
- **【v0.5追加】** 「EGOをSUMOへ登録」を初期化処理の明示ステップとするか、メインループの自動アダプトに一任するか(2.0/2.10参照)
- **【v0.5追加】** 全体フロー図の「SUMO→CARLA同期」「CARLA→SUMO同期」に付随する注記(NPC状態更新／信号状態更新／EGO状態更新)が、3.4/3.7で確認した実装内容と正しく対応しているか(0.5節の4参照)

これらの詳細は、実際の統合実装・実機検証を通じて確定する。
