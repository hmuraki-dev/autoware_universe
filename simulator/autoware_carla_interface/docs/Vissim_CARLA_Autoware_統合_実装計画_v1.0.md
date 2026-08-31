# Vissim–CARLA–Autoware統合 実装計画

作成日: 2026-08-31
版: v1.0
対象ブランチ: `feature/vissim_co-sim`(ベースコミット: `chore: bump version to 0.52.0 (#12902)`)
対象方針: `autoware_carla_interface` を主処理とし、CARLA公式Vissim–CARLAブリッジ(PTV Vissim Driving
Simulator Interface経由)の同期処理をライブラリ化して組み込む。

参考: `feature/sumo_co-sim` ブランチで実施した同種の統合作業(`SUMO_CARLA_Autoware_統合修正項目_v0.5.md`
/ `SUMO_CARLA_Autoware_統合_実装ステップ計画_v1.1.md`、いずれもgit historyにコミット済み)。本ドキュメントは
その設計判断・実装ステップの分け方を踏襲しつつ、Vissim特有の差分を反映する。

---

## 0. 前提調査で判明した事実

### 0.1 参照元コードの構成

```text
/home/divp/CARLA/Co-Simulation/PTV-Vissim/
├── run_synchronization.py            # 同期ループ本体(CLIエントリポイント)。SimulationSynchronization定義
├── test_carla_spawn_autopilot.py     # 【カスタム追加】EGO相当のテスト車両をCARLA側にのみ生成するスクリプト
├── vissim_integration/
│   ├── carla_simulation.py           # CarlaSimulationクラス(CARLA側ラッパー。信号freeze/unfreeze含む)
│   ├── vissim_simulation.py          # PTVVissimSimulationクラス(ctypes経由のDS Interfaceラッパー)
│   ├── bridge_helper.py              # BridgeHelperクラス(座標変換・車両型変換・信号状態変換)
│   └── constants.py
├── data/
│   ├── vtypes.json                   # Vissim vehicle type ⇔ CARLA blueprint 対応表
│   └── signal_mapping.json           # Town01用。(ControllerID, SignalGroupID) ⇔ CARLA OpenDRIVE ID
├── util/
│   ├── generate_signal_mapping.py    # 信号マッピング生成(幾何学的最近傍マッチング)
│   ├── export_traffic_lights.py      # 信号機位置のPNG/CSVエクスポート(可視化用)
│   └── signal_sync_stub_test.py      # 信号同期ロジックのスタブ検証(実機不要)
├── examples/Town01/, Town03/          # .inpxネットワークファイル一式
├── Vissim-CARLA_Co-Simulation実行手順.md
├── TRAFFIC_SIGNAL_TODO.md            # 信号同期の実装記録(完了済みタスク+未解決の実機検証項目)
└── LINUX_KERNEL_TODO.md              # Linux Kernel対応の実装記録(★重要な未解決の不具合を含む、1.2参照)
```

動作確認済みの実行手順(`Vissim-CARLA_Co-Simulation実行手順.md`)は次の4プロセス構成(SUMO版と同様、
現状はマルチクライアント構成):

1. `CarlaUE4.sh`(CARLAサーバー)
2. `config.py --map Town01` → `generate_signal_mapping.py`(初回のみ・マップ変更時のみ)
3. `run_synchronization.py examples/Town01/Town01.inpx --vissim-lib-path ... --step-length 0.1
   --sync-traffic-lights --debug`(Vissim同期)
4. `test_carla_spawn_autopilot.py`(EGO相当のテスト車両生成、CARLA側のみ・オートパイロット)

統合後は、SUMO版と同様に2〜4のプロセスが行っていることを、すべて `autoware_carla_interface` の
単一プロセス・単一 `carla.Client` の中に集約する。

### 0.2 SUMO版との構造的な違い(重要)

| 観点 | SUMO版 | Vissim版 |
|---|---|---|
| 通信方式 | TraCI(`sumolib`/`traci`、TCPソケット) | `ctypes.CDLL` で `libDrivingSimulatorProxy.so` をロードし、C関数を直接呼ぶ(PTV Vissim Kernel for Linux) |
| 起動プロセス | SUMO自体を `traci.start()` で起動、または既存プロセスへ`traci.init()`で接続 | 事前に起動済みのVissim Kernelへ `VISSIM_ConnectToKernel()` で接続(スクリプト側からVissimプロセスは起動しない) |
| CARLA車両→他Sim登録 | 自動アダプト(`vehicle.*`型を無差別に検出し即座にSUMOへ`traci.vehicle.add()`) | 同様に`carla.spawned_actors`を無差別に検出するが、**登録はCreateID方式の2段階ハンドシェイク**(`Create=True`送信→数tick後にVissim側が実VehicleIDをエコーバックして初めて`active`化)であり、即時ではない |
| 登録可能台数の上限 | 実質上限なし(`VISSIM_MAX_VISSIM_VEH`相当の概念がSUMO側に無い) | **`--simulator-vehicles`(既定1)で明示的に上限がある**。上限に達すると新規車両は警告ログのみでサイレントに登録されない |
| 信号同期の方向 | `--tls-manager {none, sumo, carla}` で双方向選択可 | **Vissim→CARLA一方向のみ**(DS InterfaceにCARLA→Vissim方向のAPIが存在しないため、`--sync-traffic-lights`は有効/無効のフラグのみ) |
| 信号ID対応表の生成 | 位置情報を含むSUMO net.xmlから自動導出可能 | **位置情報を含まないため手動生成必須**(`generate_signal_mapping.py`による幾何学的最近傍マッチング。マップ変更・信号配置変更のたびに再生成が必要) |
| CARLA接続の一元化要否 | 必要(`CarlaSimulation.__init__`が独自Client生成) | **同様に必要**(`vissim_integration.carla_simulation.CarlaSimulation.__init__(args)`も独自に`carla.Client`を生成) |
| 同期モード設定の重複 | `SimulationSynchronization.__init__`が重複設定 | **同様に重複**(`run_synchronization.py`の`SimulationSynchronization.__init__`が`world.apply_settings()`を呼ぶ。ただし`CarlaSimulation`自体は同期モード設定を行わない) |
| CARLA Tick分離要否 | 必要(`CarlaSimulation.tick()`がworld.tick()+差分検出を同時に行う) | **同様に必要**(構造はSUMO版とほぼ同一) |
| 明示的な既知の不具合 | 無し(実機検証は概ね成功) | **1件は解決済み・1件は未解決(1.2章参照)**: CreateID確認が低確率で失敗する問題(解決済み)、Simulation Period跨ぎでCARLA車両が消失する問題(未解決) |

### 0.3 Vissim側の不具合(必読・リスクとして計画に反映)

`LINUX_KERNEL_TODO.md` の記録によれば、以下2点が課題として挙がっていた。1.は原因判明・解決済み、
2.は現時点で未解決である。Autoware統合の実装順序・優先度に影響するため、着手前に必ず把握しておくこと。

1. **【解決済み・2026-08-31】CreateID確認の低確率失敗**
   `VISSIM_SetDriverVehicles()`が`True`を返しても、Vissim側がサイレントに車両生成に失敗し、
   `VISSIM_GetTrafficVehicles()`に`CreateID`が一切エコーバックされないケースが実機で再現していた。

   **根本原因が判明した**: `.inpx`ネットワーク側のステップ時間(0.1秒)と、co-simスクリプト側の
   ステップ時間(既定0.05秒)が**不一致**だったことが原因だった。両者のステップ時間を0.1秒に
   統一したところ、本現象は発生しなくなることを確認済み。

   → **本統合作業への反映**: 2.2節で述べる「`fixed_delta_seconds`を0.1秒に統一する」という
   方針は、単なる推奨(SUMO版に合わせる)ではなく、**このCreateID確認問題を回避するために
   必須の設定**であることが確定した。Vissimネットワーク(`.inpx`)側のステップ時間と
   `autoware_carla_interface`側の`fixed_delta_seconds`・Vissim接続時に渡す`step_length`は
   **必ず一致させること**(片方だけ0.1秒に変更して他方が異なる値のままだと、本現象が再発する
   おそれがある)。既存のリトライ機構(`_pending_retry_ticks`)は保険として残してよいが、
   本来の対策は「ステップ時間を一致させること」である点をコードコメント・ドキュメント双方に
   明記する。EGO登録(2.10相当)は、ステップ時間さえ一致していれば通常通り成功する前提で
   設計してよい(3.5節のベストエフォート方針は保険として維持するが、リトライが頻発すること
   自体は「異常」として扱ってよい)。
2. **【未解決】Simulation Period(run境界)跨ぎでCARLA車両が消失**
   `.inpx`のSimulation periodの経過により自動的に次のrunへ遷移するタイミングで、
   CARLA側の車両アクター(手動spawn分含む)が消失する現象が複数回再現している。
   対応案として「`.inpx`のSimulation periodを十分長く設定する」ことが優先案として挙げられている。

2.については引き続き**本統合作業側で新たに作り込む不具合ではなく、Vissim連携コード自体に存在する
既知の課題**として扱う。Autoware実機検証(本計画のStep6相当)に着手する前に、Simulation Periodを
どこまで延ばすかを再確認・再検証することを優先度Aに含める(3.5節参照)。

---

## 1. 結論

修正対象は大きく次の2領域であり、SUMO版と同一の構造を踏襲する。

1. **初期化処理**
   Vissim Kernelへの接続・DS Interface経由の同期エンジン生成(実装済みコードを流用)を組み込み、
   CARLA側で生成したEGOをVissim側へ登録する。
2. **メインループ処理**
   `autoware_carla_interface` のループを唯一の主ループとして残し、
   `Vissim Tick(Set+Get) → Vissim→CARLA同期 → CARLA Tick → CARLA→Vissim同期` を組み込む。

最重要事項は、SUMO版と同様に**CARLA Tickを1か所に限定すること**である。

> `vissim_integration`側の`CarlaSimulation.tick()`は、内部で`world.tick()`を呼んだ直後に
> `vehicle.*`アクターの差分検出を行う一体型の実装になっている。`autoware_carla_interface`の
> 別のアクター生成手段(`CarlaDataProvider.request_new_actor()`等、デフォルトで暗黙tickする)
> と混在させないという設計原則は、SUMO版と全く同じ理由でVissim版にも適用する。

---

## 2. 初期化処理の修正項目

### 2.0 初期化処理の実行順序(SUMO版を踏襲)

```text
CARLA Clientに接続
  ↓
CARLAマップ読み込み
  ↓
Vissim Kernelへ接続(VISSIM_ConnectToKernel)
  ↓
Vissim同期エンジン作成(CARLA Client/World設定・ID対応表/座標変換初期化・信号マッピング読み込み・
  対象信号機のfreeze)
  ↓
CARLA EGOスポーン(spawn_pointの座標に従ってEGOをスポーン)
  ↓
EGOをVissimへ登録(spawn_actor()によるCreateリクエスト送信。ただし2.10参照、非同期ハンドシェイクの
  ため即時確定ではない)
  ↓
CARLAセンサースポーン
```

SUMO版で確定した順序(2.0)と基本的に同一だが、「EGOをVissimへ登録」はCreateID確認が
数tickの遅延を伴う非同期処理である点がSUMO版(TraCIの同期API呼び出しで即時完了)と異なる。

### 2.1 Vissim関連パラメータを追加する

`autoware_carla_interface.launch.xml` から、少なくとも以下を指定できるようにする
(`run_synchronization.py`の既存CLI引数に極力対応させる)。

- Vissimネットワークファイル(`.inpx`)のパス
- `libDrivingSimulatorProxy.so`の絶対パス(`--vissim-lib-path`。未指定時は`LD_LIBRARY_PATH`任せ)
- `step_length`(SUMO版同様0.1秒を既定値に採用予定。2.2参照)
- Driving Simulator車両の最大同時登録数(`--simulator-vehicles`。**既定1のまま、EGO 1台分のみを
  想定**。2.11のNPC排他制御と合わせて、この値を安易に増やさない方針とする)
- 信号同期の有効/無効(`--sync-traffic-lights`。SUMO版と異なり方向選択は不要、フラグのみ)
- Vissim機能の有効/無効(`use_vissim`。SUMO版の`use_sumo`に相当)

`--carla-host`/`--carla-port`はCARLA接続一元化(2.5)により不要になるため、launchパラメータとして
追加しない(既存の`host`/`port`パラメータをそのまま使う)。

### 2.2 ステップ時間を統一する

Vissimの`simulatorFrequency`は`VISSIM_ConnectToKernel()`に`c_ushort(int(1./step_length))`として
渡される(`.inpx`側には直接設定しない値で、接続時にKernelへ通知する形)。動作確認済みの実行例は
いずれも`--step-length 0.1`を使用している。

> **【必須・実機検証で確定】** SUMO版で決定した「`fixed_delta_seconds`を0.1秒に統一する」という
> 方針をVissim版でも踏襲する(現ブランチのベースコミットでは`fixed_delta_seconds`既定値は0.05秒の
> ままなので、0.1秒への変更が必要)。単なる横並びの推奨ではなく、**`.inpx`ネットワーク側のステップ
> 時間(0.1秒)と`autoware_carla_interface`側の`fixed_delta_seconds`/Vissim接続時の`step_length`
> が一致していないと、0.3節1.で述べたCreateID確認の失敗が発生することが実機検証で確認済み**である。
> 両者を必ず0.1秒に統一すること(片方だけ変更して他方が異なる値のままだと、本現象が再発する
> おそれがある)。`int(1./0.1)`が意図通り`10`になることも合わせて確認しておくこと(浮動小数点誤差で
> `9`に丸まらないかの確認、優先度B)。

### 2.3 Vissim接続処理を組み込む

`PTVVissimSimulation.__init__`(`vissim_integration/vissim_simulation.py`)には以下が実装済み。

- `cdll.LoadLibrary(lib_path)` によるDS Interfaceライブラリのロード
- `_declare_prototypes()` によるctypes関数シグネチャの明示宣言
- `VISSIM_ConnectToKernel()` による接続(失敗時は`VISSIM_GetLastErrorMessage()`を使った
  `RuntimeError`)
- Create/CreateIDベースの車両登録状態機械(`_simulator_vehicles`辞書、`pending`/`active`状態)

これは「移植」ではなく`PTVVissimSimulation`クラスをそのままインポートして利用する形で組み込む。
ただし`__init__`の引数が現状`args`(argparseの`Namespace`)を直接受け取る設計になっているため、
`autoware_carla_interface`のROS 2パラメータから必要な値だけを渡す薄いアダプタ(`argparse.Namespace`
相当のオブジェクトを組み立てて渡す、または引数を分解して受け取れるようリファクタする)が必要になる。

### 2.4 Vissim同期エンジン生成処理を組み込む

`SimulationSynchronization`(`run_synchronization.py`内に定義)は独立したクラスとして実装済みで、
以下を担う。

- `PTVVissimSimulation`
- `CarlaSimulation`
- `BridgeHelper`
- 信号マッピングの読み込み(`_load_signal_mapping()`)と対象信号機のfreeze

SUMO版と同様、`CarlaSimulation.__init__`が独自に`carla.Client`を生成し、
`SimulationSynchronization.__init__`が独自に同期モード設定を行っているため、移植時は以下の改修が
必須。

- `CarlaSimulation.__init__(args)` を、外部から `client`/`world` を注入できるコンストラクタに変更する
- `SimulationSynchronization.__init__` 内の `world.apply_settings()` 呼び出しを削除する(2.6)

### 2.5 CARLA接続を一元化する

SUMO版(v0.5 2.5)と同一の方針。現状の`vissim_integration.CarlaSimulation.__init__`は次の通り
独自に接続を生成している(コード確認済み)。

```python
class CarlaSimulation(object):
    def __init__(self, args):
        self.client = carla.Client(args.carla_host, args.carla_port)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        ...
```

統合時は、`InitializeInterface.load_world()`で生成済みの`client`/`world`
(`CarlaDataProvider.get_client()`/`get_world()`)を注入する形に変更する。

### 2.6 CARLA同期モード設定の重複を削除する

`SimulationSynchronization.__init__`(`run_synchronization.py`)は以下を実行している
(コード確認済み)。

```python
settings = self.carla.world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = args.step_length
self.carla.world.apply_settings(settings)
```

これは`InitializeInterface.load_world()`側の設定と完全に重複するため、統合時はこのブロックを
丸ごと削除する。なお、Vissim版はSUMO版と異なり`traffic_manager.set_synchronous_mode()`の
呼び出しはそもそも行っていない(Traffic Manager自体を使わない前提のため)。

### 2.7 ID対応表を初期化する

`SimulationSynchronization`は既に以下のID対応表を保持している(そのまま流用可能)。

- `self.vissim2carla_ids`: Vissim車両ID → CARLA Actor ID(Vissim正本の車両)
- `self.carla2vissim_ids`: CARLA Actor ID → Vissim車両ID(CARLA正本の車両。EGOはここに入る想定)
- 信号ID対応は`self.signal_mapping`(`{(controller_id, signal_group_id): [opendrive_id, ...]}`)で管理

> **アクター生成時の注意**(SUMO版2.7と同様): `CarlaSimulation.spawn_actor()`は内部で
> `client.apply_batch_sync(batch, False)`を使っており、暗黙的にtickしない実装になっている。
> Vissim由来車両の生成・削除は、この既存メソッドをそのまま使う。

### 2.8 座標変換情報を初期化する

`BridgeHelper.get_carla_transform()`/`get_vissim_transform()`(`vissim_integration/bridge_helper.py`)
に、SUMO版と同様の変換が実装済みである。

- 前方バンパー基準⇔車両中心基準の補正(`extent.x`を用いた位置シフト)
- 左手系(CARLA)⇔右手系(Vissim)の座標変換、Y軸反転
- 信号状態変換(`_VISSIM_TO_CARLA_SIGNAL_STATE`、11種類のVissim信号状態→CARLAの5状態への縮退マップ)

> **SUMO版との差異**: SUMO版の`BridgeHelper`にあった「SUMO⇔CARLA間のネットワーク座標オフセット
> 補正(`self.offset`)」に相当する処理がVissim版には存在しない。これは意図的な省略か、
> `.inpx`のネットワーク座標系がそもそもCARLA座標系とオフセット無しで一致する前提を置いているためか、
> 実装時に要確認(3.8「実装時に確認が必要な事項」参照)。Autowareが使うマップ(既定Town01)で
> 座標がずれずに一致するか、初回接続時に実測で確認すること。

### 2.9 EGOをCARLAへスポーンする

既存の`autoware_carla_interface`の処理(`InitializeInterface.load_world()` →
`_parse_spawn_point()`)を維持する。SUMO版2.9と同一の注意点(スポーン後の実際の
`ego_actor.get_transform()`を使うこと)がそのまま当てはまる。

### 2.10 EGOをVissimへ登録する

CARLA側で生成したEGOに対応する車両を、Vissim側へ登録する。

> **SUMO版との重要な違い**: SUMO版の「自動アダプト」は`traci.vehicle.add()`が**その場で完了する
> 同期API**だったため、EGOスポーン直後の最初の`tick()`で確実に登録が完了していた。Vissim版は
> `spawn_actor()`が返すのは内部管理用の仮ID(`actor_id`)のみで、実際にVissim側の`VehicleID`が
> 確定する(`active`状態になる)のは**数ティック後、`VISSIM_GetTrafficVehicles()`が
> `CreateID`をエコーバックした時点**である。したがって:
>
> - EGOのVissim側登録は「初期化直後に完了している」ことを前提にできない。メインループの序盤
>   数ティックは「Vissim側にEGOがまだ存在しない」状態がありうる設計として扱う必要がある。
> - 0.3節1.で述べた通り、以前はこの確認が最終的に来ないまま失敗し続けるケースが実機で確認されて
>   いたが、**原因は`.inpx`側とco-simスクリプト側のステップ時間の不一致であり、両者を0.1秒に
>   統一することで解消済み**である。ただし、ステップ時間を一致させても「数ティックの遅延を伴う
>   非同期ハンドシェイクである」という設計上の性質自体は変わらないため、EGO登録が確定しない場合の
>   フォールバック・ログ・アラートは引き続き用意しておく(優先度A、3.5節参照)。

必要な追加確認・処理:

- EGOがAutowareによって制御される場合も、CreateID方式の登録が同様に機能するかをエンドツーエンド
  で検証する(現状の動作確認は`test_carla_spawn_autopilot.py`によるオートパイロット車両のみ)
- `_pending_retry_ticks`(既定・約2秒)がAutowareの実運用速度感で妥当か確認する
- 登録が長時間`pending`のまま解決しない場合の扱い(ログレベル、診断トピック化、等)を設計する

### 2.11 CARLA側のランダムNPC生成を見直す【必須】

SUMO版2.11と全く同じ理由で必須。`self.carla.spawned_actors`の差分検出は`vehicle.*`型の
**あらゆる**CARLAアクターに対して無差別に働く(EGOもTraffic Manager由来のNPCも区別しない)。

> **Vissim版特有の追加リスク**: `use_traffic_manager=True`のままVissimを使うと、TM由来のNPCが
> 次々とVissimへの登録を試み、`--simulator-vehicles`の上限(既定1)に達した時点で
> **エラーにならずログ警告のみで登録が打ち切られる**。SUMO版は上限という概念自体が無かったため
> 「全部登録されて破綻する」問題だったが、Vissim版は「一部だけ登録され、残りは静かに無視される」
> というより気づきにくい形の副作用になる。排他チェックの必要性はSUMO版以上に高い。

対応: `use_vissim=True`かつ`use_traffic_manager=True`が同時指定された場合、起動時にエラーで停止する
排他チェックを追加する(SUMO版2.11・Step5と同一パターン)。

### 2.12 センサー生成処理を維持する

変更なし(SUMO版2.12と同一)。

### 2.13 初期化失敗時の終了処理を追加する

途中で初期化に失敗した場合、生成済みリソースを解放する。

- Vissim接続の切断(`PTVVissimSimulation`には現状 `close()` に相当するメソッドが無い可能性がある
  ため、`VISSIM_Disconnect()`を呼ぶ経路を実装時に確認・整備する。`run_synchronization.py`の
  `SimulationSynchronization.close()`は`self.vissim.close()`を呼んでいるため、
  `PTVVissimSimulation`側に`close()`メソッドが存在する前提で書かれている可能性が高いが、
  本ドキュメント執筆時点では`vissim_simulation.py`の該当箇所を全文確認できていない。実装着手時に
  必ず確認すること)
- freeze済み信号機の解凍(`unfreeze_traffic_lights()`)
- 生成済みCARLAアクター削除
- ROS 2ノード終了
- CARLA設定の復元

### 2.14 Vissim由来アクターの管理統合

SUMO版2.14と同一方針。`SimulationSynchronization.close()`の`vissim2carla_ids`/`carla2vissim_ids`
一括破棄ロジックをそのまま利用できる見込みが高い。

---

## 3. メインループ処理の修正項目

### 3.1 主ループを `autoware_carla_interface` に一本化する

`run_synchronization.py`の独立した`while True:`ループ(`synchronization_loop()`内)は使用しない。
`max_real_delta_seconds`ベースのペーシングと類似の`time.sleep()`ペーシングを持っている点もSUMO版と
同じ(3.2-2相当の重複排除が必要)。

統合後の基本順序(SUMO版3.1と同一パターン):

```text
センサーデータ取得・配信
  ↓
ROS 2コールバック処理
  ↓
Autoware制御指令をCARLA EGOへ適用
  ↓
Vissim Tick(VISSIM_SetDriverVehicles + VISSIM_GetTrafficVehicles + VISSIM_GetSignalStates)
  ↓
Vissim→CARLA同期(NPC生成/削除/位置反映、信号状態反映)
  ↓
CARLA Tick
  ↓
CARLA→Vissim同期(EGO/車両状態のVissimへの登録・位置反映)
  ↓
次ループ
```

### 3.2 CARLA Tickを1か所に限定する

残す処理: `autoware_carla_interface`側のCARLA Tick。

削除・分離する処理: `CarlaSimulation.tick()`(`world.tick()` + `vehicle.*`差分検出が一体化)を、
SUMO版と同じパターンで「`world.tick()`を呼ぶ版」と「呼ばない版(`update_actor_diff()`相当、
差分検出のみ)」に分離する。

#### 3.2-1 `PTVVissimSimulation.tick()` は分割不要(SUMO版との差異・要設計判断)

SUMO版では`SimulationSynchronization.tick()`を「SUMO Tick+sumo→carla同期」と
「carla tick+carla→sumo同期」の2メソッドに分割する際、`SumoSimulation.tick()`自体は
分割不要だった(`traci.simulationStep()`は純粋なpull処理のため)。

Vissim版の`PTVVissimSimulation.tick()`は、**CARLA車両状態の送信(push、
`VISSIM_SetDriverVehicles`)とVissim車両/信号状態の取得(pull、`VISSIM_GetTrafficVehicles`
/`VISSIM_GetSignalStates`)が1メソッド内で連続実行される一体型**である。ただし、この
push内容は「前回ループの`carla-->vissim同期`で`synchronize_vehicle()`により記録された、
CARLA側の直近の位置」であり、pull結果は「そのタイミングでのVissim側の最新状態」を返す。
すなわち、`vissim.tick()`をメインループの先頭(CARLA Tickより前)で1回呼ぶという構造は
**そのまま維持してよく、追加の分割は不要**と判断できる(=SUMO版の`SumoSimulation.tick()`と
同じ位置づけで良い)。分割・改修が必要なのはあくまで`CarlaSimulation.tick()`側のみである。

> ただし、この判断は静的なコード解析に基づくものであり、`VISSIM_GetTrafficVehicles()`が
> 内部でVissimの1シミュレーションステップの完了を待つ(ブロックする)ことを前提にしている。
> 実機接続後、最初に確認すべき項目としてStep3〜4の確認方法に明記する。

### 3.3 Vissim Tick処理を追加する

`PTVVissimSimulation.tick()`をそのまま流用可能(2.3参照)。Vissim側の異常時(ライブラリ呼び出し
失敗、`VISSIM_GetLastErrorMessage()`が有意なメッセージを返す場合等)の例外処理を呼び出し側に
追加する。

### 3.4 Vissim→CARLA同期処理を追加する

`SimulationSynchronization.tick()`の前半(vissim-->carla sync)に実装済みの以下を流用する。

- Vissimで新規生成された車両をCARLAへ生成(`BridgeHelper.get_carla_blueprint()` →
  `self.carla.spawn_actor()`。暗黙tickなし)
- Vissimで削除された車両をCARLAから削除
- Vissim車両の位置・姿勢・速度をCARLAへ反映(`self.carla.synchronize_vehicle()`)
- Vissim信号状態をCARLA信号へ反映(`--sync-traffic-lights`有効時、`signal_mapping`経由)
- ID対応表(`vissim2carla_ids`)を更新

### 3.5 EGOのVissim登録の非同期ハンドシェイクを考慮した設計にする【優先度A】(方針決定済み・Step0)

SUMO版には存在しなかった、Vissim固有の検討事項。

- `carla-->vissim同期`(3.7相当)で`self.vissim.spawn_actor(transform)`を呼んだ直後は、
  戻り値の`actor_id`が`pending`状態であり、まだVissim側車両として実体化していない。
  この間に`synchronize_vehicle()`が呼ばれても位置更新のみで実際にはVissimへ反映されない
  (2.10参照)。
- **前提条件(必須)**: 2.2節の通り、`.inpx`側のステップ時間と`autoware_carla_interface`側の
  `fixed_delta_seconds`/`step_length`を0.1秒に一致させること。これが崩れるとCreateID確認が
  失敗し続ける現象が再発するため、実装・デプロイの両方で必ず確認する(単なる推奨ではなく前提条件)。
- ステップ時間を一致させた上でも、登録確定までに数ティックのラグがあるという設計上の性質は
  残るため、失敗時の扱いについてStep0で以下の通り決定済み。
  - **採用: (a) ベストエフォート方針**: 登録が(何らかの理由で)確定しなくても致命的エラーとはせず、
    ログ警告のみとしてAutoware側の走行自体は継続させる(EGOがVissim側のNPCから見えないだけで、
    CARLA単体の挙動には影響しない)
  - 不採用: (b) 検証ゲート方針(EGO登録の`active`化を初期化完了の条件に含め、一定時間内に確定
    しない場合は起動を失敗させる)。ステップ時間を一致させることで既知の主要因は解消したが、
    それ以外の未知の要因で失敗する可能性を完全には排除できないため、(b)のように起動失敗に
    直結させるとかえって可用性を下げるリスクがあると判断した(Step0の判断理由と同一)。

### 3.6 CARLA Tickを実行する

`autoware_carla_interface`側の`world.tick()`を1回実行した後、`CarlaSimulation.tick()`が行っている
「`world.get_actors().filter('vehicle.*')`による車両アクター差分検出」のロジックを、tick呼び出しと
分離して実行できるよう改修する(3.2参照、SUMO版3.6と同一パターン)。

### 3.7 CARLA→Vissim同期処理を追加する

`SimulationSynchronization.tick()`の後半(carla-->vissim sync)に実装済みの以下を流用する。

- CARLA側で新規生成された(まだ`vissim2carla_ids`/`carla2vissim_ids`どちらにも未登録の)
  車両アクターを検出し、Vissimへ登録要求(`self.vissim.spawn_actor()`。2.10/3.5参照、
  即時確定ではない)
- 前回ループで登録要求が失敗した(`INVALID_ACTOR_ID`のままの)車両についても再登録要求を
  試みる(`carla_spawned_actors.update([...])`の部分、既存ロジックで対応済み)
- CARLA側で削除された車両をVissimから削除要求
- CARLA車両の位置・姿勢・速度をVissimへ反映(`self.vissim.synchronize_vehicle()`。ただし
  実際にVissim側へpushされるのは次回ループの`vissim.tick()`内、3.2-1参照)
- ID対応表(`carla2vissim_ids`)の更新

### 3.8 センサーデータ処理との時系列を維持する / EGO車両ステータス配信との整合

変更なし(SUMO版3.9/3.10と同一)。

### 3.9 終了条件を統合する

SUMO版3.11と同一方針。加えてVissim固有の終了条件として、0.3節の「Simulation Period跨ぎでの
車両消失」を検知した場合の扱い(ログ警告に留めるか、再同期を試みるか)を優先度Bで検討する。

### 3.10 終了処理を一元化する

`InitializeInterface._cleanup()`に、`SimulationSynchronization.close()`相当の処理
(freeze済み信号機の解凍・Vissim/CARLA双方の同期アクター破棄・Vissim切断)を追加する形で統合する。
SUMO版Step7の「各ステップをtry/exceptで分離し、1箇所の失敗が他のクリーンアップをブロックしない
ようにする」という設計方針をそのまま踏襲する。

### 3.11 スレッドモデルを明記する

変更なし(SUMO版3.13と同一。Vissim同期処理もメインスレッド上で逐次実行し、新たなスレッドは
追加しない)。

---

## 4. 既存処理と新規処理の区分

| 処理 | 区分 | 主な対応 |
|---|---|---|
| Vissim Kernelへの接続 | **流用**(実装・動作確認済み) | `vissim_integration.PTVVissimSimulation` をそのままインポート |
| Vissim同期エンジン作成 | 流用＋改修 | `SimulationSynchronization`。CARLA再接続・同期モード重複設定を除去 |
| ID対応表 | **流用**(実装済み) | `vissim2carla_ids`/`carla2vissim_ids` |
| 座標変換 | **流用**(実装済み) | `BridgeHelper.get_carla_transform`/`get_vissim_transform`。オフセット補正の要否は要確認(2.8) |
| CARLA Client／World生成 | 既存維持 | `InitializeInterface.load_world()` が管理。`CarlaSimulation`側は注入方式に改修 |
| CARLA同期モード設定 | 既存維持 | `InitializeInterface.load_world()` が管理。`SimulationSynchronization`側の重複設定を削除 |
| CARLA EGOスポーン | 既存維持 | `InitializeInterface.load_world()` の処理 |
| EGOをVissimへ登録 | **流用だが非同期・低確率失敗リスクあり(新規リスク管理が必要)** | `spawn_actor()`のCreateIDハンドシェイク(2.10/3.5参照) |
| Vissim Tick | **流用**(実装済み、分割不要) | `PTVVissimSimulation.tick()` を主ループへ組み込み |
| Vissim→CARLA同期 | **流用**(実装済み) | `SimulationSynchronization.tick()` 前半をCARLA Tick前に実行 |
| CARLA Tick | 既存維持＋改修 | 1か所だけ残す。`CarlaSimulation.tick()` からtick呼び出し部分を分離 |
| CARLA→Vissim同期 | **流用**(実装済み) | `SimulationSynchronization.tick()` 後半をCARLA Tick後に実行 |
| 信号同期 | **流用(一方向のみ)** | `signal_mapping.json` + `switch_off_traffic_lights`/`synchronize_traffic_light`/`unfreeze_traffic_lights` |
| センサー取得・publish | 既存維持 | ワーカースレッドを維持 |
| CARLA側ランダムNPC生成 | 設定変更(**必須**) | `use_traffic_manager` はVissim使用時に強制OFF・排他エラー化(2.11) |
| Vissim由来アクターのライフサイクル管理 | **流用できる可能性が高い** | `SimulationSynchronization.close()` の一括破棄ロジック |
| マルチプロセス構成→単一プロセス化 | **新規(構造変更)** | `test_carla_spawn_autopilot.py`/`run_synchronization.py` 相当を`autoware_carla_interface`の1プロセス・1クライアントに集約 |
| 信号マッピングデータ | **既存資産流用(Town01のみ)** | `data/signal_mapping.json`。Autowareが別マップを使う場合は`generate_signal_mapping.py`で再生成が必要 |

---

## 5. 優先度

### 優先度A: 必須

- CARLA Tickの一元化(`CarlaSimulation.tick()` からtick呼び出しを分離)
- Vissim Kernelへの接続組み込み(`PTVVissimSimulation`流用)、`--vissim-lib-path`のlaunch化
- Vissim→CARLA同期の組み込み(`SimulationSynchronization.tick()`前半の流用)
- CARLA Client/World の外部注入(`CarlaSimulation`改修)・同期モード重複設定の削除
- CARLA→VissimによるEGO状態同期の組み込み、および**非同期ハンドシェイクへの対応方針決定**
  (3.5節、ベストエフォート方針(a)採用決定済み)
- `use_traffic_manager` とVissim使用の排他制御(`--simulator-vehicles`上限の副作用を防ぐため必須、
  2.11)
- 正常終了・異常終了処理の統合(`SimulationSynchronization.close()`との統合、Vissim切断APIの実装
  詳細確認含む)
- **ステップ時間の統一(0.1秒への変更、2.2)** — `.inpx`側と`autoware_carla_interface`側の
  両方を0.1秒に一致させること。CreateID確認失敗の既知原因(0.3節1.、解決済み)の再発防止のため、
  単なる既定値変更ではなく**必須の前提条件**として扱う
- スレッドモデルの明記(Vissim同期処理はメインスレッドで実行)
- **0.3節2.(Simulation Period跨ぎでのCARLA車両消失)への対応方針の再確認**
  (現時点で未解決。`.inpx`のSimulation periodを想定テスト時間より十分長く設定することを最低限の
  回避策として採用するかどうかを決定する)

### 優先度B: 動作安定化

- ID対応表の整合性チェック
- 車両生成/削除の例外処理
- VissimとCARLAの時刻ずれ検出
- 実時間ペーシング(`max_real_delta_seconds`)とDS Interface呼び出しの相互作用検証
- `int(1./step_length)`の丸め誤差確認(2.2)
- Simulation Period跨ぎの検知・再同期(3.9)
- 座標オフセット補正の要否確認(2.8)

### 優先度C: 機能拡張

- 信号状態のAutoware向けメッセージ変換(Lanelet2交通信号IDとの対応表を新規追加。SUMO版と共通化
  できないか検討)
- Autowareが使う可能性のある他マップ向けの`signal_mapping.json`生成(`generate_signal_mapping.py`
  の実行手順をAutoware側ドキュメント化)
- 同期状態・遅延・車両数・EGO登録状態(pending/active)の診断トピック追加
- 車両ライト・車体色同期(現状Vissim版のBridgeHelperには未実装の可能性があり、実装時に確認)

---

## 6. 実装ステップ計画(SUMO版 v1.1 と同じ段階分けスタイル)

### Step 0: 設計決定の確定(コード変更なし) — 完了(2026-08-31)

#### ① EGO登録失敗時の方針(3.5参照)

**選択肢**
- 案(a): ベストエフォート方針(登録に失敗してもログ警告のみとし、Autoware側の走行自体は継続させる)
- 案(b): 検証ゲート方針(EGO登録の`active`化を初期化完了の条件に含め、一定時間内に確定しない場合は
  起動を失敗させる)

**採用案: 案(a)(ベストエフォート方針)**

**判断理由**
- 0.3節1.の通りCreateID確認失敗の主要因(ステップ時間不一致)は解決済みだが、それ以外の未知の
  要因で稀に失敗する可能性は完全には排除できない
- 案(b)を採ると、その稀な失敗が原因で起動そのものが失敗するようになり、可用性を大きく下げる
  リスクがある
- EGOがVissim側から見えないだけであれば、CARLA単体・Autoware側の走行には影響しないため、
  致命的エラーにする必要性が薄い
- 3.5節・Step6の実装方針として反映済み(EGO登録は非同期ハンドシェイクとして扱い、`pending`が
  長時間続く場合はログ警告に留める)

#### ② `vissim_integration` パッケージの取り込み方法

**選択肢**
- 案A: 外部パス参照(`/home/divp/CARLA/Co-Simulation/PTV-Vissim`を直接参照)
- 案B: `autoware_carla_interface`配下へvendor化

**採用案: 案B(vendor化)**

**判断理由**
- `feature/sumo_co-sim`ブランチでの先行事例(SUMO版Step0)と同一の判断理由がそのまま当てはまる:
  外部パス依存を排除できる、Gitで一元管理できる、統合時の改修を同一リポジトリで管理できる、
  環境依存が少なく保守性・再現性が高い

**③ 取り込み対象**(`/home/divp/CARLA/Co-Simulation/PTV-Vissim` からvendor化するもの)

- `carla_simulation.py`(`vissim_integration/`。ただし2.5/2.6の改修=外部注入方式への変更を前提に
  取り込む)
- `vissim_simulation.py`(`PTVVissimSimulation`。2.3で述べた通り、`args`引数の受け取り方を
  `autoware_carla_interface`のROS 2パラメータに合わせて薄くリファクタする)
- `bridge_helper.py`(`BridgeHelper`)
- `constants.py`
- `data/vtypes.json`
- `data/signal_mapping.json`(Town01用。他マップを使う場合は別途生成が必要、7章参照)
- `run_synchronization.py`から`SimulationSynchronization`クラスのみを抽出した新規ファイル
  (`simulation_synchronization.py`。SUMO版と同じ抽出方針)

**④ 取り込まない対象**(vendor化しないもの。3.1/3.2で「重複のため削除」とした処理と対応)

- `run_synchronization.py`のCLIエントリポイント(`argparse`部分)・独立した`while True:`ループ
- `CarlaSimulation.__init__`内の独自の`carla.Client`生成処理(注入方式に置き換えるため)
- `SimulationSynchronization.__init__`内の同期モード設定(`world.apply_settings()`。
  `autoware_carla_interface`側の設定と重複するため)
- `CarlaSimulation.tick()`内の`world.tick()`呼び出し(CARLA Tickの一元化のため、tickの実行自体は
  行わずアクター差分検出のみ利用する)
- `test_carla_spawn_autopilot.py`(マルチプロセス構成の名残であり、統合後は不要)

### Step 1: CARLA接続の一元化(2.5 / 2.6)

- `vissim_integration`をvendor化し、`CarlaSimulation.__init__`を外部注入方式に改修
- `SimulationSynchronization.__init__`内の重複した同期モード設定を削除
- この時点ではVissimはまだ繋がない。既存のCARLA単体動作に影響がないことを確認する

### Step 1実施内容(2026-08-31)

**vendor化(Step 0 ③の選定対象)** を `src/autoware_carla_interface/vissim_integration/` 以下に作成した
(`/home/divp/CARLA/Co-Simulation/PTV-Vissim/`から取得、MITライセンス表記は保持し、経緯は
`vissim_integration/NOTICE.md`に記載)。

- `constants.py` / `bridge_helper.py` / `data/vtypes.json` / `data/signal_mapping.json`:
  **本体無修正でvendor化**(`constants.py`/`bridge_helper.py`はヘッダーに由来コメントのみ追加。
  data系2ファイルはoriginalと`diff`でバイト単位の完全一致を確認済み)
- `vissim_simulation.py`: vendor化。ただし**upstream側に存在した、モジュールレベルで
  `/opt/vissim_kernel_2026.00-10/lib/libDrivingSimulatorProxy.so`を固定パスでロードし、
  かつどこからも呼ばれていない未使用のデバッグコード(`dsi = ctypes.CDLL(...)` /
  `print_vissim_last_error()`)を削除した**。これを残したままだと、そのパスが存在しない環境では
  このモジュールを`import`しただけで無条件に失敗する(`PTVVissimSimulation.__init__`が本来
  行っている、`args.vissim_lib_path`によるレイジー・可変パスでのライブラリロードとは別に、
  import時点で即座に固定パスを読みに行ってしまうため)。この1点のみが差分であることを`diff`で
  確認済み
- `carla_simulation.py`: **修正してvendor化**。`CarlaSimulation.__init__`が自前で
  `carla.Client`/`World`を生成しないようにし(`args.carla_host`/`args.carla_port`削除)、
  外部から接続済みの`client`/`world`を注入する形に変更(2.5)。他のメソッドは無修正
- `simulation_synchronization.py`: **新規ファイル**。`run_synchronization.py`から
  `SimulationSynchronization`クラスのみを抽出(CLI/独立ループは非vendor化)。`__init__`内の
  CARLA同期モード設定ブロック(`world.apply_settings()`)を削除(2.6)。`close()`内の
  非同期モード復元処理は、Step1時点ではSUMO版の前例に倣いあえて手を付けず、そのまま残した
  (実際の重複回避策はStep7でまとめて設計する)

**実施した検証**(単体レベル、mock使用):

1. `python3 -m py_compile` で全ベンダーファイルの構文エラーが無いことを確認
2. `constants.py`/`bridge_helper.py`/`vissim_simulation.py`/`carla_simulation.py` をオリジナルと
   `diff`し、意図した差分(由来コメントの追加、デバッグブロックの削除、`__init__`のクライアント
   注入化)以外に差分がないことを確認
3. `CarlaSimulation(mock_client, mock_world)` をmockで生成し、(a)コンストラクタが`client`/
   `world`のみを受け取ること(host/port引数が存在しないこと、`self.args`を保持しないこと)、
   (b)注入したclient/worldがそのまま保持され、`carla.Client(...)`自体は一切呼ばれないことを確認
4. `SimulationSynchronization(mock_vissim, mock_carla, mock_args)` をmockで生成し、
   `world.get_settings()`/`world.apply_settings()`が**一切呼ばれない**ことを確認(2.6の重複削除を
   検証)
5. `data/vtypes.json`/`data/signal_mapping.json`が原本とバイト単位で完全一致することを`diff`で
   確認

**未実施(次回以降で実施推奨)**: 実際にCARLAサーバーを起動しての end-to-end 回帰確認
(colcon build → `ros2 launch` でCARLA単体シミュレーションが従来通り動作すること)。新規ファイルは
既存コードから一切importされていない(`carla_autoware.py`/`carla_ros.py`は未変更、`__init__.py`は
空のまま)ため、既存の実行パスに影響が無いことは構造的に保証されるが、実機確認は未実施。

### Step 2: パラメータ追加・ステップ時間統一(2.1 / 2.2)

- launchファイルにVissim関連パラメータを追加(まだ未使用でOK)
- `fixed_delta_seconds`のデフォルトを0.05→0.1秒に変更
- **対象の`.inpx`のシミュレーションステップ時間が0.1秒に設定されていることを確認する**(不一致だと
  Step6でCreateID確認失敗が再発することが判明済みのため、この段階で先に確認しておく)

### Step 2実施内容(2026-08-31)

**`launch/autoware_carla_interface.launch.xml`**

- `fixed_delta_seconds`のデフォルトを`0.05`から`0.1`に変更
- `max_real_delta_seconds`のデフォルトも`0.05`から`0.1`に変更(SUMO版と同じ理由:
  `fixed_delta_seconds`だけを0.1に上げて`max_real_delta_seconds`を0.05のままにすると、実時間ペーシング
  の速度倍率上限が崩れるため、揃えて更新した)
- Vissim関連パラメータを`<arg>`として新規追加(まだどのノードの`<param>`にも渡していない。ノードへの
  配線・Python側`declare_parameter`はStep3で実施): `use_vissim`(既定`False`)、`vissim_network`
  (既定空文字、`.inpx`パス)、`vissim_lib_path`(既定空文字、`libDrivingSimulatorProxy.so`の絶対パス。
  空文字は`PTVVissimSimulation.__init__`内の`args.vissim_lib_path or 'libDrivingSimulatorProxy.so'`が
  そのままフォールバックとして機能するため、SUMO版の`"None"`センチネル文字列のような特別扱いは不要)、
  `vissim_simulator_vehicles`(既定`1`)、`sync_traffic_lights`(既定`False`)
- **意図的に追加していないもの**: Vissim側の`step_length`(=`VISSIM_ConnectToKernel`に渡す値)専用の
  `<arg>`は追加しなかった。0.3節1.で判明した「`.inpx`側とco-simスクリプト側のステップ時間の不一致が
  CreateID確認失敗の原因だった」という教訓を踏まえ、Step3の配線時に`fixed_delta_seconds`の値を
  そのままVissim接続にも流用する設計とし、独立して設定できるパラメータを最初から作らないことで
  ステップ時間の乖離が構造的に起こり得ないようにする方針とした
- `run_synchronization.py`の`--carla-host`/`--carla-port`は、CARLA接続一元化(2.5、Step1で対応済み)
  により不要なため追加していない

**`.inpxのステップ時間確認`**

- `examples/Town01/Town01.inpx`・`examples/Town03/*.inpx`の`<simulation>`要素を確認したところ、
  いずれも`simRes="10"`(1秒あたり10ステップ = ステップ時間0.1秒)であり、**既に目標の0.1秒と
  一致していることを確認した**(`.inpx`側の変更は不要)
- 併せて`simPeriod`(0.3節2.のSimulation Period跨ぎ問題に関連)も確認: Town01は`simPeriod="300"`
  (5分)、Town03は`simPeriod="3600"`(1時間)。Town01は短時間テストでも跨ぐ可能性があるため、
  Step6での実機検証時にテスト時間と比較して要注意(この値の変更自体はStep2のスコープ外)

**`README.md`**

- `fixed_delta_seconds`/`max_real_delta_seconds`のデフォルト値表記・Tips記載を`0.05`から`0.1`に更新

**実施した検証**:

1. `python3 -m xml.dom.minidom`でlaunch XMLの構文妥当性を確認
2. `ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml --show-args`
   (`install/`がsymlink-installのため再ビルド不要で編集内容がそのまま反映される)で、
   `fixed_delta_seconds`/`max_real_delta_seconds`が`0.1`、新規Vissim引数がすべて意図した
   説明文・デフォルト値で認識されることを確認
3. `grep`で`.inpx`の`<simulation>`要素の`simRes`/`simPeriod`を確認(上記の通り)

**未実施(次回以降で実施推奨)**: 実際にCARLAサーバーを起動しての end-to-end 回帰確認(0.1秒
ステップでCARLA単体シミュレーションが従来通り動作すること)。追加したVissim関連`<arg>`はどの
`<param>`にも渡していないため、既存ノードの動作には影響しない構造になっている。

### Step 3: Vissim Kernel接続 + 同期エンジン生成(2.3 / 2.4 / 2.7 / 2.8)

- `PTVVissimSimulation`のインポート・接続を`InitializeInterface`に組み込む
- `SimulationSynchronization`(ID対応表・座標変換・信号マッピング込み)を生成する
- `tick()`はまだ呼ばない。接続確認のみ
- **確認方法**: CARLA・Vissim Kernelの両方が起動し、正常に接続・切断できることを確認。
  併せて3.2-1で述べた「`VISSIM_GetTrafficVehicles()`がVissimの1ステップ完了を待つ」という
  想定が実機で成立するかをこの段階で最初に確認する

### Step 3実施内容(2026-08-31)

**`carla_ros.py`**: `_initialize_parameters()`に新規ROS 2パラメータを追加(`use_vissim`/
`vissim_network`/`vissim_lib_path`/`vissim_simulator_vehicles`/`sync_traffic_lights`)。すべて
Python側にデフォルト値を持たせ(`use_vissim`は`False`)、launchファイルが未対応でも既存ノードが
起動できるようにした。

**`launch/autoware_carla_interface.launch.xml`**: Step2で追加した`<arg>`をノードの`<param>`として
配線。

**`carla_autoware.py`(`InitializeInterface`)**:

- `__init__`で上記パラメータを読み込み、`self.vissim_carla_sim`/`self.vissim_sim`/
  `self.vissim_sync`を`None`で初期化
- 新規メソッド`_init_vissim_integration(client)`を追加。`use_vissim=False`(デフォルト)なら即
  `return`(既存動作に影響なし)。`True`の場合のみ、vendor化した`CarlaSimulation`(client/world
  注入版)・`PTVVissimSimulation`・`SimulationSynchronization`を遅延import(将来的な依存関係の
  分離のため関数内import。現時点では`vissim_integration`側にimport時点で外部ライブラリを読みに
  行くコードは無いためSUMO版の`traci`/`sumolib`ほど厳密な理由はないが、同じパターンを踏襲した)
  して構築。`vissim_args`は`types.SimpleNamespace`で組み立て、`step_length`は独立パラメータに
  せず**`self.fixed_delta_seconds`をそのまま流用**(2.2/0.3節の教訓を設計で担保するため)。
  `vissim_lib_path`の空文字列は`None`に変換し、`PTVVissimSimulation`側の
  `args.vissim_lib_path or 'libDrivingSimulatorProxy.so'`フォールバックが機能するようにした
- `load_world()`内、`CarlaDataProvider.set_client(client)`の直後・EGOスポーンの直前に
  `self._init_vissim_integration(client)`を呼び出し(v0.5 2.0で確定した初期化順序と一致)
- `_cleanup()`に`_cleanup_vissim()`を追加(EGOアクター破棄後・`CarlaDataProvider`破棄前)。
  **`PTVVissimSimulation.close()`(`VISSIM_Disconnect()`)のみ呼び出し**、
  `SimulationSynchronization.close()`(信号解凍・同期アクター破棄・非同期モード復元を含む)は
  Step7で統合予定のため今回は呼ばない

**実施した検証**(mockベースの単体テスト・ROS 2環境上で実施):

1. `use_vissim=False`(デフォルト)で`_init_vissim_integration()`が完全なno-opであること
   (`vissim_carla_sim`/`vissim_sim`/`vissim_sync`が`None`のまま、`CarlaSimulation`/
   `PTVVissimSimulation`/`SimulationSynchronization`が一切呼ばれないこと)を確認
2. `use_vissim=True`で、vendor化した`CarlaSimulation`/`PTVVissimSimulation`/
   `SimulationSynchronization`が期待した引数(注入された`client`/`world`、`vissim_network`、
   `simulator_vehicles`、`sync_traffic_lights`)で正しく1回だけ呼ばれることを確認
3. `vissim_lib_path=""` → `vissim_args.vissim_lib_path is None`への変換、
   `vissim_args.step_length == self.fixed_delta_seconds`(0.1)であることを確認
4. `_cleanup_vissim()`が`vissim_sim`が`None`のときno-op、設定されていれば`close()`を1回呼ぶこと、
   `close()`が例外を送出しても`_cleanup_vissim()`自体は例外を伝播させないことを確認
5. ROS 2環境(`/opt/ros/humble` + ワークスペースoverlay、symlink-installのため再ビルド不要)を
   実際にsourceし、`carla_ros2_interface._initialize_parameters()`を実ノードで呼び出して新規
   パラメータが期待したデフォルト値(`use_vissim=False`/`vissim_network=""`/
   `vissim_lib_path=""`/`vissim_simulator_vehicles=1`/`sync_traffic_lights=False`)で宣言される
   ことを確認

**未実施(次回以降で実施推奨)**: 実際にCARLAサーバー・Vissim Kernelを両方起動してのend-to-end
接続確認(`use_vissim=True`でノードを実行し、`VISSIM_ConnectToKernel()`が成立すること、および
`VISSIM_GetTrafficVehicles()`がVissimの1ステップ完了を待つという3.2-1の想定が実機で成立するか)。
`use_vissim=False`時は新規コードパスに一切入らないため、既存のCARLA単体動作への影響は構造的に
無いが、実機での最終確認は未実施。

### Step 4: メインループへの同期処理組み込み + CARLA Tick一元化(3.1〜3.7)

- `Vissim Tick → Vissim→CARLA同期 → CARLA Tick → CARLA→Vissim同期`を主ループに組み込む
- `CarlaSimulation.tick()`からtick呼び出し部分を分離し、`world.tick()`は1ループ1回のみに限定
- **確認方法**: Vissim側で生成した車両がCARLAに、CARLA側(テスト用オートパイロット車両)の車両が
  Vissimに、それぞれ反映されることを確認。ログでtick回数が1ループ1回であることを検証

### Step 4実施内容(2026-08-31)

**`vissim_integration/carla_simulation.py`**: `tick()`を分割。

- `update_actor_diff()`(新規): `world.tick()`を呼ばずに`spawned_actors`/`destroyed_actors`/
  `_active_actors`のみ更新
- `tick()`: `world.tick()` → `update_actor_diff()`。単体利用時の後方互換のために残すが、配線後の
  メインループからは呼ばれない

**`vissim_integration/simulation_synchronization.py`**: `tick()`を分割。

- `sync_vissim_to_carla()`(新規、旧`tick()`前半 "vissim-->carla sync"+信号sync相当):
  `self.vissim.tick()` → Vissim車両のCARLAへの生成/削除/位置反映 → (`sync_traffic_lights`なら)
  信号反映。**CARLA側は一切tick・差分更新しない**
- `sync_carla_to_vissim()`(新規、旧`tick()`後半 "carla-->vissim sync"相当): 冒頭で
  `self.carla.update_actor_diff()`を呼んでから、CARLA車両(EGO含む、自動アダプト経由)のVissimへの
  登録要求/削除要求/位置反映
- `tick()`: `sync_vissim_to_carla()` → `self.carla.tick()` → `sync_carla_to_vissim()`。単体利用時の
  後方互換のために残すが、配線後のメインループからは呼ばれない
- 3.2-1で述べた通り、`PTVVissimSimulation.tick()`自体(push+pull一体型)は分割不要と判断し、
  そのまま`sync_vissim_to_carla()`冒頭で呼び出している

**`carla_autoware.py`**:

- `SensorLoop.__init__`に`self.vissim_sync = None`を追加(デフォルトはVissim無効と同じ挙動)
- `SensorLoop._tick_sensor()`を修正。`self.ego_actor.apply_control(ego_action)`の直後に
  `vissim_sync`があれば`sync_vissim_to_carla()`を呼び、既存の`CarlaDataProvider.get_world().tick()`
  (**唯一のCARLA Tick呼び出し、変更なし**)の直後に`vissim_sync`があれば`sync_carla_to_vissim()`を
  呼ぶ。`vissim_sync is None`(デフォルト)の場合は追加コードパスに一切入らない
- `InitializeInterface.run_bridge()`で`self.bridge_loop.vissim_sync = self.vissim_sync`を設定

**`vissim_integration/NOTICE.md`**: 上記2ファイルのtick分割を「vendor化時の逸脱」として追記。

**実施した検証**(mockベースの単体テスト):

1. `CarlaSimulation.update_actor_diff()`が`world.tick()`を呼ばないこと、`tick()`は`world.tick()`後に
   差分更新することを確認
2. `SimulationSynchronization.sync_vissim_to_carla()`が`self.carla.tick()`/`update_actor_diff()`の
   どちらも呼ばないこと、`sync_carla_to_vissim()`は`update_actor_diff()`のみ呼び`world.tick()`は
   呼ばないことを確認
3. `SimulationSynchronization.tick()`(後方互換ラッパー)が
   `sync_vissim_to_carla → carla.tick → sync_carla_to_vissim`の順で呼ばれることを確認
4. **`SensorLoop._tick_sensor()`の回帰確認**: `vissim_sync=None`(デフォルト)時、Step4適用前と
   同じ呼び出し(`sensor()` → `apply_control()` → `world.tick()`が正確に1回)になることを確認
5. `vissim_sync`設定時、呼び出し順序が`sensor() → apply_control() → sync_vissim_to_carla() →
   world.tick()(1回) → sync_carla_to_vissim()`であることを確認(3.1の確定順序と一致)
6. タイムスタンプゲートが閉じている(未経過)ケースでも、`world.tick()`と(設定時は)
   `sync_carla_to_vissim()`は毎ループ実行され、`sensor()`/`apply_control()`/
   `sync_vissim_to_carla()`はスキップされることを確認(既存のゲート挙動を維持)

**未実施(次回以降で実施推奨)**: 実際にCARLA・Vissim Kernel両サーバーを起動してのend-to-end動作
確認(Vissim車両がCARLAに、CARLA車両(EGO含む)がVissimに実際に反映されること)。優先度A項目
「EGOのVissim自動登録経路の動作検証(Autoware制御下)」は引き続き未実施。

### Step 5: NPC排他制御(2.11)

- `use_traffic_manager=True`とVissim使用が同時指定された場合にエラー停止する排他チェックを追加

### Step 6: EGO登録・双方向同期の検証(2.9 / 2.10 / 3.5)

- Step0で決めた方針でEGOをVissimへ登録
- **Autoware制御下のEGO**でエンドツーエンド検証
- **ステップ時間(`.inpx`と`autoware_carla_interface`側)が0.1秒で一致していることを再確認した上で
  検証する**: 0.3節1.の通り、ここが不一致だとCreateID確認が失敗する現象が再現するため、
  Step2で確認済みであっても本Stepの実機検証開始時にもう一度確認する
- Simulation Periodを十分に長く設定した`.inpx`で、想定するテスト時間内にrun境界を跨がないことを
  確認する(0.3節2.、現時点で未解決)

### Step 7: 終了処理の統合(2.13 / 3.9 / 3.10)

- `InitializeInterface._cleanup()`に`SimulationSynchronization.close()`相当の処理を統合
- 各クリーンアップステップをtry/exceptで分離し、1箇所の失敗が他をブロックしないようにする
  (SUMO版Step7と同一方針)

### Step 8: 動作安定化(優先度B)

- ID対応表の整合性チェック、車両生成/削除の例外処理、Vissim/CARLAの時刻ずれ検出、
  `int(1./step_length)`丸め誤差確認、Simulation Period跨ぎの検知など

### Step 9以降: 機能拡張(優先度C、別タスクとして後回し推奨)

- 信号状態のLanelet2/Autowareメッセージ変換、他マップ向け信号マッピング生成手順の整備、
  診断トピック追加など

---

## 7. 実装時に確認が必要な事項

- (解決済み)`PTVVissimSimulation`に`close()`メソッドが実際に存在するかは、vendor化時に原本
  (`vissim_simulation.py`)を確認して**存在することを確認済み**(`VISSIM_Disconnect()`を呼ぶのみ)。
  `SimulationSynchronization.close()`が`self.vissim.close()`を呼ぶ前提は成立している
- `BridgeHelper`に座標オフセット補正(SUMO版の`self.offset`相当)が本当に存在しない/不要なのか、
  Autowareが使うマップで実測して確認する(2.8)
- `--simulator-vehicles`の既定値1のまま運用して問題ないか(EGO 1台のみを想定する限り問題ないが、
  将来的にCARLA側から複数の実験車両を出す計画がある場合は要再検討)
- (解決済み)EGO登録のCreateID確認が失敗し続ける現象は、`.inpx`とco-simスクリプトのステップ時間
  不一致が原因と判明し、両者を0.1秒に統一することで解消した。ステップ時間の一致を実装・運用の両方で
  維持できる仕組み(launchデフォルト値の固定、起動時の不一致検知ログ等)を用意できないか検討する
  (優先度B)
- Simulation Periodをどの程度まで延ばせば、想定するAutoware走行テストの最大時間を安全にカバー
  できるか(0.3節2.、現時点で未解決)
- Autowareが使用する可能性のあるマップ(Town01以外)について、対応する`.inpx`と
  `signal_mapping.json`が用意されているか(用意されていない場合は`generate_signal_mapping.py`を
  そのマップに対して再実行する必要がある)
- 車両ライト・車体色の同期がVissim版のBridgeHelper/CarlaSimulationに実装されているか
  (SUMO版にあった`sync_vehicle_lights`/`sync_vehicle_color`相当の機能の有無を確認)

これらの詳細は、実際の統合実装・実機検証を通じて確定する。
