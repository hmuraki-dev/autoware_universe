# Vissim(Windows)–CARLA(Linux)–Autoware リモートCo-Simulation化 実装計画

対象ブランチ: `feature/vissim_windows_co-sim`(`feature/vissim_co-sim`の最新からブランチ済み)

移植元(取り込み元):
- ベース: `/home/divp/CARLA/Co-Simulation/PTV-Vissim`(ブランチ`main`)
- 変更版: `/home/divp/CARLA/Co-Simulation/PTV-Vissim`(ブランチ`feature/vissim_windows`)
- 設計ドキュメント:
  - `docs/WINDOWS_VISSIM_REMOTE_TODO.md`(設計課題・確定事項一覧)
  - `docs/WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md`(実装タスクの洗い出し・進捗)

前提ドキュメント(本リポジトリ側、正本一覧は`/memories/repo/vissim_co-sim_docs.md`参照):
- `docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`(Vissim-CARLA-Autoware統合の元計画)
- `docs/Vissim_CARLA_Autoware_歩行者同期_実装計画_v1.0.md`(歩行者同期の追加計画)
- `docs/Vissim-CARLA-Autoware_co-sim_起動手順.md`(起動手順、正本)

---

## 0. 前提調査で判明した事実

### 0.1 移植元(CARLA公式リポジトリ)側の変更内容

`main`→`feature/vissim_windows`の差分を確認した結果、変更は大きく2つに分かれる。

1. **Linux側オーケストレータの改修**(`Co-Simulation/PTV-Vissim/`):
   - `vissim_integration/vissim_simulation.py`: `PTVVissimSimulation`が、ctypes直接呼び出し
     (`libDrivingSimulatorProxy.so`をロードして`VISSIM_ConnectToKernel`等を直接呼ぶ)から、
     **ZeroMQ(REQ)クライアント**に全面改修されている。**公開インターフェース
     (`__init__(args)`, `tick()`, `spawn_actor()`, `destroy_actor()`, `synchronize_vehicle()`,
     `get_actor()`, `get_pedestrian()`, `get_signal_state()`, `signal_ids`, `tick_count`,
     `spawned_vehicles`/`destroyed_vehicles`/`spawned_pedestrians`/`destroyed_pedestrians`,
     `close()`)は一切変更されていない**ため、これを呼び出す側
     (`SimulationSynchronization`/`run_synchronization.py`)は無改修。
   - `vissim_integration/rpc_protocol.py`(新規): ZeroMQ+msgpackのメッセージエンベロープ定義。
     `carla`にも`ctypes`にも非依存。
   - `vissim_integration/constants.py`: 内容は無変更、Windows側との「byte-identicalな
     ベンダーコピー」であることを明記するdocstringのみ追加。
   - `run_synchronization.py`: CLI引数変更のみ。
     - 削除: 位置引数`vissim_network`、`--vissim-lib-path`(Windows側のローカルな事実になったため)。
     - 追加: `--vissim-adapter-host`(既定`127.0.0.1`)、`--vissim-adapter-port`(既定`5555`)、
       `--vissim-connect-timeout-ms`(既定`60000`、`connect`/再接続専用)、
       `--vissim-rpc-timeout-ms`(既定`2000`、毎tick用)。
     - `SimulationSynchronization`本体・`--step-length`・`--simulator-vehicles`は無変更。
2. **Windows側アダプタの新規追加**(`Co-Simulation/PTV-Vissim_windows/`、**自己完結フォルダ**):
   `constants.py`/`rpc_protocol.py`(Linux側とbyte-identicalなベンダーコピー)、
   `vissim_kernel_session.py`(ctypes構造体・DLLロード・Create/CreateID状態機械。現行Linux
   Kernel版の`vissim_simulation.py`から`carla`依存を除去して移植)、`server.py`
   (ZeroMQ REPループ、CLI引数)、`README.md`、`requirements.txt`(`pyzmq`, `msgpack`)。
   パッケージ化(`__init__.py`)されておらず、**このフォルダごとWindows機にコピーして
   `python server.py ...`でそのまま実行**できるように作られている。

### 0.2 通信プロトコル概要(詳細は移植元`WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md` §3)

- ZeroMQ **REQ/REP** + **msgpack**。1 simulation stepにつき1回のREQ/REP往復
  (`tick`メッセージにspawn/destroy/updateコマンドと現在状態取得を両方載せる)。
- `local_id`(Linux側で採番する不透明な整数)でcarla起源車両を識別。実VehicleIDへの解決・
  `CreateID`確認待ち・リトライは全てWindowsアダプタ内に閉じ込め、プロトコル上に出てこない
  (Linux側の実装・責務は変わらない)。
- `connect`(起動時/再接続時)・`tick`(毎ステップ)・`disconnect`(終了時)の3種類のメッセージ。
- タイムアウト時はREQソケットをclose&再生成し、次回`tick()`冒頭で`connect`を再送してから
  本処理に進む(`_needs_reconnect`フラグ)。タイムアウトしても`carla.tick()`を止めない
  (差分集合を空にして正常returnする)設計。
- `vissim_network`(`.inpx`パス)・DLLパスはWindows側`server.py`のローカルなCLI引数となり、
  Linux側からは一切渡さない。`step_length`・`simulator_vehicles`は引き続きLinux側
  (Autoware/ROS側)を単一の情報源とし、`connect`メッセージで伝える。

### 0.3 本リポジトリ(`autoware_carla_interface`)固有の事情との整合性確認

移植元`WINDOWS_VISSIM_REMOTE_TODO.md` §10には「Autoware統合時は`autoware_carla_interface`側の
`world.tick()`/`synchronous_mode`制御を無効化・改修する必要がある」という論点が記載されている
(移植元は`run_synchronization.py`が唯一のtickマスターであることを前提に設計されているため)。

**→ この論点は本リポジトリでは既に解消済みで、影響なし。** 本リポジトリでは
`docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`の設計により、`SimulationSynchronization`は
最初から`run_synchronization.py`のような独立オーケストレータとしてではなく、
`vissim_integration/simulation_synchronization.py`として**ライブラリ化・組み込み**されており、
tick()は`sync_vissim_to_carla()`/`sync_carla_to_vissim()`に分割され、
`autoware_carla_interface`(`SensorLoop._tick_sensor()`)の中で
`CarlaDataProvider.get_world().tick()`(唯一のCARLA tick呼び出し)を挟む形で直接呼び出されている
(`carla_autoware.py`参照)。つまり**tickマスターは最初から`autoware_carla_interface`一択であり、
今回のWindowsリモート化でこの構造を変える必要はない**。

このため、今回移植すべき範囲は**`vissim_simulation.py`(ctypes→ZeroMQクライアント化)と
`rpc_protocol.py`(新規vendor)のみ**であり、`simulation_synchronization.py`/
`carla_simulation.py`/`bridge_helper.py`は無改修で済む。

### 0.4 現状(本リポジトリ)の該当コード確認結果

- `src/autoware_carla_interface/vissim_integration/vissim_simulation.py`: 現在は移植元の
  **Linux Kernel版**(ctypes直接呼び出し)がそのままvendorされている(`NOTICE.md`に記載の
  デビエーションのみ)。
- `carla_autoware.py`の`InitializeInterface._init_vissim_integration()`が、ROSパラメータから
  組み立てた`SimpleNamespace`(`vissim_args`)を`PTVVissimSimulation(vissim_args)`に渡している。
  現在渡している属性: `simulator_vehicles`, `vissim_lib_path`, `vissim_network`, `step_length`,
  `sync_traffic_lights`。
- ROSパラメータ宣言は`carla_ros.py`(`use_vissim`, `vissim_network`, `vissim_lib_path`,
  `vissim_simulator_vehicles`, `sync_traffic_lights`)。
- `launch/autoware_carla_interface.launch.xml`にも同名の`<arg>`/`<param>`が存在。
- 外部の`autoware_launch`(`~/autoware.1.9.0/src/launcher/autoware_launch`)の
  `e2e_simulator.launch.xml`にも、同じ5個の引数をノードまで転送するローカル未コミット差分が
  適用済み(別リポジトリのため本ブランチのコミット対象外、`/memories/repo/vissim_co-sim_docs.md`
  参照)。

---

## 1. 結論

`vissim_integration/vissim_simulation.py`を移植元と同様に**ZeroMQ REQクライアント化**し、
`rpc_protocol.py`を新規vendorする。それに伴い、Linux側で不要になった`vissim_network`/
`vissim_lib_path`パラメータを、Windowsアダプタへの接続情報(`vissim_adapter_host`/
`vissim_adapter_port`/タイムアウト2種)に置き換える変更を、ROSパラメータ・launchファイル
(本リポジトリ側・`autoware_launch`側の両方)・`carla_autoware.py`の全レイヤーに通す。

Windows側(`Co-Simulation/PTV-Vissim_windows/`)は**そのまま流用**し、本リポジトリには
vendorしない(自己完結フォルダとして、移植元リポジトリから直接コピーして使う運用を維持する。
理由は§8参照)。

```
[Linux: autoware_carla_interface プロセス]                    [Windows]
SensorLoop._tick_sensor()                                      PTV-Vissim_windows/server.py
  sensor()/apply_control()                                       - ZeroMQ REPソケットでlisten
  vissim_sync.sync_vissim_to_carla()  <--- ZeroMQ REQ --->        - VissimKernelSession
  world.tick()  (唯一のCARLA tick)                                  (ctypes, DrivingSimulatorProxy)
  vissim_sync.sync_carla_to_vissim() <--- ZeroMQ REQ --->            |
    (内部で PTVVissimSimulation.tick() が                            v (ローカル)
     ZeroMQ REQ/REP 1往復を行う)                                Vissim (GUI or Console)
```

---

## 2. 変更ファイル一覧

| ファイル | 変更内容 |
|---|---|
| `src/autoware_carla_interface/vissim_integration/vissim_simulation.py` | **全面改修**: ctypes直接呼び出し→ZeroMQ REQクライアント化。移植元の実装をベースに、本リポジトリ固有のデビエーション(NOTICE.md記載分、歩行者同期含む)を再適用しながら移植する |
| `src/autoware_carla_interface/vissim_integration/rpc_protocol.py` | **新規vendor**(移植元からbyte-identicalに近い形でコピー、carla非依存) |
| `src/autoware_carla_interface/vissim_integration/constants.py` | 変更なし(既存のまま。`INVALID_ACTOR_ID`等、新版でも引き続き使用される) |
| `src/autoware_carla_interface/vissim_integration/simulation_synchronization.py` | **無改修**(§0.3参照。`PTVVissimSimulation`の公開インターフェースが変わらないため) |
| `src/autoware_carla_interface/vissim_integration/carla_simulation.py` / `bridge_helper.py` | **無改修** |
| `src/autoware_carla_interface/vissim_integration/NOTICE.md` | `vissim_simulation.py`のデビエーション記述を更新、`rpc_protocol.py`の vendor 元情報を追記 |
| `src/autoware_carla_interface/carla_ros.py` | ROSパラメータ変更: `vissim_network`/`vissim_lib_path`を削除、`vissim_adapter_host`/`vissim_adapter_port`/`vissim_connect_timeout_ms`/`vissim_rpc_timeout_ms`を追加 |
| `src/autoware_carla_interface/carla_autoware.py` | `InitializeInterface.__init__`/`_init_vissim_integration()`のパラメータ読み出し・`vissim_args`組み立てを変更 |
| `launch/autoware_carla_interface.launch.xml` | `<arg>`/`<param>`を同様に変更 |
| `~/autoware.1.9.0/src/launcher/autoware_launch/launch/e2e_simulator.launch.xml` | (別リポジトリ、ローカル未コミット)同様に引数転送を変更 |
| `package.xml` / `setup.py` | Python依存(`pyzmq`, `msgpack`)の扱いを明記(下記§4.0参照。`carla`パッケージ同様、package.xml/setup.pyには追加せず、起動手順ドキュメントに手動インストール手順を追記する方針) |
| `test/vissim_pedestrian_sync_stub_test.py` | 影響確認のみ(`FakeVissimSimulation`を使うため無改修で通るはず、§6で回帰確認) |
| `docs/Vissim-CARLA-Autoware_co-sim_起動手順.md` | Windowsアダプタの起動手順、新CLI引数・ROSパラメータへの反映 |
| `/memories/repo/vissim_co-sim_docs.md` | 本計画doc・関連ファイルの追記(ユーザー記憶) |

---

## 3. パラメータ変更一覧(Before/After)

| 旧パラメータ(ROS param / launch arg) | 新パラメータ | 備考 |
|---|---|---|
| `vissim_network`(`.inpx`パス) | **削除**(Windows側`server.py`のCLI引数`--vissim-network`に移動、Linux/Autoware側では持たない) | Windows機のローカルな事実になったため |
| `vissim_lib_path`(`.so`パス) | **削除**(Windows側`server.py`のCLI引数`--vissim-lib-path`に移動) | 同上。かつWindowsでは`.dll`なので意味も変わる |
| (新規) | `vissim_adapter_host`(既定`127.0.0.1`) | Windowsアダプタ(`server.py`)のIPアドレス |
| (新規) | `vissim_adapter_port`(既定`5555`) | Windowsアダプタのポート |
| (新規) | `vissim_connect_timeout_ms`(既定`60000`) | `connect`(初回接続・再接続)専用タイムアウト。実機ではVissim GUI起動に数十秒かかるため長め |
| (新規) | `vissim_rpc_timeout_ms`(既定`2000`) | 毎tickの`tick`メッセージ用タイムアウト |
| `vissim_simulator_vehicles` | 変更なし | Linux側(Autoware側)が単一の情報源のまま、`connect`メッセージ経由でアダプタに伝える |
| `sync_traffic_lights` | 変更なし | Vissim→CARLA一方向、プロトコルの`tick`応答`signals`から取得(既存ロジックのまま) |
| `use_vissim` | 変更なし | |

---

## 4. 実装ステップ

### Step W0: 事前準備

- [x] `feature/vissim_windows_co-sim`ブランチが`feature/vissim_co-sim`の最新から分岐済みで
      あることを確認(完了済み、§0.4参照)。
- [x] 移植元`Co-Simulation/PTV-Vissim`(`feature/vissim_windows`ブランチ)の
      `vissim_integration/vissim_simulation.py`(647行)・`rpc_protocol.py`の全文と、本リポジトリの
      現行`vissim_simulation.py`(ctypes版、歩行者同期込み)を1メソッド単位で突き合わせた
      (2026-09-15実施)。**結論: 両者はロジック的にほぼ1対1対応しており、機械的な置き換えで
      移植できることを確認した。** 主な突き合わせ結果:
  - `__init__`/`tick()`/`close()`の制御フロー・ログ出力(20tickごとのサンプルログ等)は
    移植元・本リポジトリ現行版でほぼ同一の構造(移植元は本リポジトリのCARLA公式ベースと
    同世代のロジックをそのままZeroMQ化しただけで、本リポジトリ独自の歩行者同期/信号同期/
    ウインカー(`TurningIndicator`)伝播ロジックは移植元にも**同じ形で既に存在する**
    (`WINDOWS_VISSIM_REMOTE_TODO.md`確認済み事項の通り、Windows版DrivingSimulatorProxy.hの
    構造体は現行Linux Kernel版と完全一致のため、CARLA公式側で機能追従済みだった)。
  - 削除される部分: ctypes `Structure`定義4つ(`Simulator_Veh_Data`/`VISSIM_Veh_Data`/
    `VISSIM_Ped_Data`/`VISSIM_Sig_Data`)、DLLロード・`_declare_prototypes()`、
    `_get_simulator_veh_data()`/`_allocate_create_id()`等のpending/active状態機械 — これらは
    全てWindowsアダプタ側(`vissim_kernel_session.py`、非vendor対象)に移る。
  - 維持される部分: `VissimPedestrianMotionState`/`VissimPedestrianConstructionElementType`/
    `VissimLightState`/`VissimSignalState`の4 enum、`VissimVehicle`/`VissimPedestrian`データ
    クラス(`carla.Transform`/`carla.Vector3D`生成含め無変更)、`get_actor()`/`get_pedestrian()`/
    `signal_ids`/`get_signal_state()`/`tick_count`等のアクセサ、`tick()`内のvehicles/
    pedestrians/signalsの3ブロック構成とサンプルログ。
  - 置き換えられる引数: `args.vissim_lib_path`/`args.vissim_network` →
    `args.vissim_adapter_host`/`args.vissim_adapter_port`/`args.vissim_connect_timeout_ms`/
    `args.vissim_rpc_timeout_ms`(移植元`__init__`で実際に読まれている属性名を確認済み、
    §3の対応表と一致)。
  - `import os`・`from ctypes import *`は不要になり削除。`constants`モジュールは
    `INVALID_ACTOR_ID`参照のためimportを維持(他の定数は移植元でも未使用になる)。
  - `NOTICE.md`の`vissim_simulation.py`デビエーション記述は、Step W6で「ctypes版からの
    デビエーション」ではなく「移植元ZeroMQ版そのままの構造」に書き換える必要がある
    (突き合わせの結果、追加のデビエーションは発生しない見込み)。
- [x] Autoware側の実行環境(`autoware_carla_interface`ノードを動かすシステムPython、
      `/usr/bin/python3` 3.10.12、`carla`パッケージが`~/.local/lib/python3.10/site-packages`に
      `pip install --user`済みの環境)で、`pyzmq`・`msgpack`を同じ方式
      (`/usr/bin/python3 -m pip install --user pyzmq msgpack`)でインストールし、
      `import zmq, msgpack, carla`が同一インタプリタで揃って成功することを実機確認済み
      (2026-09-15、`pyzmq 27.2.0` / `msgpack 1.2.2` / `carla 0.9.15`)。
      `package.xml`/`setup.py`には追加せず、`carla`と同様に手動pipインストールする現行方針を
      踏襲する(§8/Step W5参照)。

### Step W1: `rpc_protocol.py`をvendor化 — ✅ 完了(2026-09-15)

- [x] 移植元`vissim_integration/rpc_protocol.py`(`feature/vissim_windows`ブランチ、173行)を
      本リポジトリの`src/autoware_carla_interface/vissim_integration/rpc_protocol.py`として
      作成した。`PROTO_VERSION`/`MSG_CONNECT`/`MSG_TICK`/`MSG_DISCONNECT`/`ProtocolError`/
      `encode_request`/`decode_request`/`encode_response`/`decode_response`/`_unpack`/
      `_check_common_envelope_fields`は**全て移植元とbyte-identical**(`diff`で確認済み、
      差分はモジュール先頭のヘッダコメント/docstringのみ)。`carla`/`ctypes`に非依存。
- [x] `NOTICE.md`にこのファイルを vendor 対象として追記し、vendor元(`feature/vissim_windows`
      ブランチ)・デビエーション(ヘッダコメントのみ、コード本体は無変更)を記載した。
      本リポジトリでは`Co-Simulation/PTV-Vissim_windows/`をvendorしない方針(§8)のため、
      移植元の「Linux/Windows間でbyte-identicalに保つ」という注意書きは、「本リポジトリの
      `rpc_protocol.py`と移植元`Co-Simulation/PTV-Vissim_windows/rpc_protocol.py`(Windows実機に
      コピーされる側)の間で、プロトコルバージョン(`PROTO_VERSION`)を含めて内容を
      一致させ続ける必要がある」という形に読み替えて明記した(一致しないとバージョン
      不一致エラーになる、または最悪サイレントに壊れる)。
- [x] 検証済み: `ast.parse`による構文チェック、パッケージ経由(`autoware_carla_interface.
      vissim_integration.rpc_protocol`)でのimport、3種別(connect/tick/disconnect)の
      request/responseラウンドトリップ、エラー応答(`ok=False`)、不明なメッセージ種別・
      `seq`不一致・不正なmsgpackバイト列がいずれも`ProtocolError`になることを実際に実行して
      確認した(移植元`util/rpc_protocol_test.py`のカバレッジ相当)。

### Step W2: `vissim_simulation.py`をZeroMQクライアント化 — ✅ 完了(2026-09-15)

- [x] `PTVVissimSimulation.__init__`: ctypesロード・DLL接続処理(`Simulator_Veh_Data`/
      `VISSIM_Veh_Data`/`VISSIM_Ped_Data`/`VISSIM_Sig_Data`の4 ctypes `Structure`定義、
      `_declare_prototypes()`、`_get_next_actor_id()`/`_allocate_create_id()`含む)を全て削除し、
      `zmq.Context()`・`REQ`ソケット生成・`connect`メッセージ送受信に置き換えた。
      `args.vissim_adapter_host`/`args.vissim_adapter_port`/`args.vissim_connect_timeout_ms`/
      `args.vissim_rpc_timeout_ms`を読む形にした(§3のパラメータ変更に対応。`import os`/
      `from ctypes import *`も不要になったため削除、`constants`は`INVALID_ACTOR_ID`参照のため
      importを維持)。
- [x] `_connect_socket()`/`_request()`/`_reconnect()`を移植元通り実装した(タイムアウト時の
      ソケット再生成、`_needs_reconnect`フラグによる次回`tick()`冒頭での`connect`再送)。
- [x] `spawn_actor()`/`destroy_actor()`/`synchronize_vehicle()`: ローカル`local_id`采番・
      バッファリング(`_pending_spawn`/`_pending_destroy`/`_pending_update`)のみを行うように
      変更した(ネットワーク往復なし)。`_max_simulator_vehicles`上限チェックも
      `_active_local_ids`集合を使ってクライアント側で完結させた。
- [x] `tick()`: バッファ済みのspawn/destroy/updateコマンドを`tick`メッセージのpayloadに詰めて
      送信し、応答の`vehicles`/`pedestrians`/`signals`から`VissimVehicle`/`VissimPedestrian`/
      `VissimSignalState`オブジェクトを再構築するように実装した。歩行者同期
      (`get_pedestrian()`, `VissimPedestrianMotionState`変換)・信号同期(`get_signal_state()`,
      `VissimSignalState`)・ウインカー伝播(`row['turn_indicator']`)は、Step W0の突き合わせで
      確認済みの通り移植元にも同一ロジックが既に存在したため、そのまま引き継いだ(追加の
      デビエーションは発生しなかった)。20tickごとのサンプルログ出力も現行のまま維持。
- [x] `close()`: `disconnect`メッセージを送ってからソケットをcloseするように実装した
      (`try/except zmq.ZMQError`/`finally`でソケット・コンテキストの解放を保証)。
- [x] タイムアウト発生時、`tick()`は差分集合(`spawned_vehicles`等)を空にして正常returnする
      (`_clear_diff_sets()`)、CARLA側の`world.tick()`を詰まらせない設計をそのまま踏襲した。
- [x] 受信データの値検証(NaN/Infinity・型不正の無視)は**Windowsアダプタ側
      (`vissim_kernel_session.py`)の責務**であり、本リポジトリ側での対応は不要と判断した
      (§8で扱う対象外事項)。`tick`応答の必須キー欠落等に対する追加の型チェックは、
      過剰実装(YAGNI)を避けるため今回は追加していない(現状のフェイクアダプタ/実アダプタは
      いずれも仕様通りのレスポンスを返す前提のため。実機検証(Step W9)で問題が出れば
      追加検討する)。
- [x] **検証済み(2026-09-15)**: `ast.parse`構文チェック、`get_errors`でエラーなしを確認。
      さらにフェイクのZeroMQ REPサーバ(バックグラウンドスレッド)を用いたループバックテストを
      実行し、`connect`→`spawn_actor`(容量上限チェック含む)→`synchronize_vehicle`→`tick()`
      (vehicles/pedestrians/signalsの復元、`spawned_vehicles`等の差分計算)→`destroy_actor`→
      2回目の`tick()`→`close()`まで一連の流れが実際に動作することを確認した
      (移植元`util/vissim_adapter_stub_test.py`相当のカバレッジ)。

### Step W3: ROSパラメータ・`carla_autoware.py`の変更 — ✅ 完了(2026-09-15)

- [x] `carla_ros.py`: パラメータ宣言テーブルから`vissim_network`/`vissim_lib_path`を削除し、
      `vissim_adapter_host`(既定`"127.0.0.1"`)・`vissim_adapter_port`(既定`5555`)・
      `vissim_connect_timeout_ms`(既定`60000`)・`vissim_rpc_timeout_ms`(既定`2000`)を追加した。
- [x] `carla_autoware.py`の`InitializeInterface.__init__`: 対応する`self.vissim_*`属性
      (`vissim_adapter_host`/`vissim_adapter_port`/`vissim_connect_timeout_ms`/
      `vissim_rpc_timeout_ms`)を読み替えた。
- [x] `InitializeInterface._init_vissim_integration()`: `vissim_args`(`SimpleNamespace`)から
      `vissim_lib_path`/`vissim_network`を削除し、`vissim_adapter_host`/`vissim_adapter_port`/
      `vissim_connect_timeout_ms`/`vissim_rpc_timeout_ms`を追加した(Step W2で書き換えた
      `PTVVissimSimulation.__init__`が読む属性名と一致することを、Step W2のループバックテストで
      使用したのと同じ属性名であることから確認済み)。`_check_vissim_traffic_manager_exclusivity()`
      等、`vissim_network`/`vissim_lib_path`に依存しないロジックは無改修のまま。
- [x] 検証済み: `ast.parse`構文チェック・`get_errors`で両ファイルともエラーなしを確認。
      `self.vissim_network`/`self.vissim_lib_path`への参照が両ファイルに一切残っていないことを
      grepで確認。`launch/autoware_carla_interface.launch.xml`側の対応する`<arg>`/`<param>`は
      **意図的に未変更のまま**残している(Step W4のスコープ)。

### Step W4: launchファイルの変更 — ✅ 完了(2026-09-15)

- [x] 本リポジトリ`launch/autoware_carla_interface.launch.xml`: `<arg>`/`<param>`を
      §3のBefore/After通りに変更した(`vissim_network`/`vissim_lib_path`を削除し、
      `vissim_adapter_host`/`vissim_adapter_port`/`vissim_connect_timeout_ms`/
      `vissim_rpc_timeout_ms`を追加)。
- [x] `~/autoware.1.9.0/src/launcher/autoware_launch`の`e2e_simulator.launch.xml`
      (別リポジトリ、ローカル未コミット差分)にも同様の変更を適用した
      (`/memories/repo/vissim_co-sim_docs.md`に記載の通り、このファイルは別リポジトリのため
      本ブランチのコミット対象ではないが、動作確認のためには必須。同メモリファイルの記述も
      更新済み)。
- [x] 検証済み: 両ファイルとも`xml.dom.minidom.parse()`でwell-formedなXMLであることを確認し、
      mojibake無しを確認した。

### Step W5: 依存関係の確認・明記

- [ ] `package.xml`/`setup.py`は変更しない方針(`carla`パッケージが現状登録されていないのと
      同じ理由で、`pyzmq`/`msgpack`もrosdep管理外のPython実行環境に手動インストールする形を
      踏襲する)。
- [ ] `docs/Vissim-CARLA-Autoware_co-sim_起動手順.md`の前提条件セクションに、
      `pyzmq`/`msgpack`のインストール手順(どのPython環境に対して`pip install pyzmq msgpack`
      するか)を明記する。

### Step W6: `NOTICE.md`の更新 — ✅ 完了(Step W1/W2内で前倒し実施済み、2026-09-15)

- [x] `vissim_simulation.py`のデビエーション記述を、「ctypes版からのデビエーション」から
      「移植元のZeroMQ版(`feature/vissim_windows`ブランチ)からのデビエーション」に更新した
      (Step W2実施時に合わせて更新済み。歩行者同期・信号同期・ウインカー伝播は移植元と同一
      ロジックのため追加デビエーションなし、と明記)。
- [x] `rpc_protocol.py`の vendor 元情報(移植元パス・ブランチ`feature/vissim_windows`)を
      追記した(Step W1で実施済み)。

### Step W7: テスト

- [ ] 既存`test/vissim_pedestrian_sync_stub_test.py`が無改修で通ることを確認する
      (`FakeVissimSimulation`を使っており`PTVVissimSimulation`を直接テストしていないため、
      影響を受けないはず)。
- [ ] `PTVVissimSimulation`自体の単体テストが本リポジトリに存在しない場合、移植元の
      `util/vissim_adapter_stub_test.py`(フェイクの`VissimKernelSession`をZeroMQ REPサーバに
      注入したループバックテスト)に相当するテストを、本リポジトリの歩行者同期ロジックも
      含めて`test/`配下に追加することを検討する(実Windows機・実Vissimなしで
      ZeroMQ層・spawn/destroy/update/tick往復・タイムアウト再接続を検証できる)。
- [ ] `ast.parse`等による構文チェック、全シンボルのimport確認(移植元のStep 3検証内容を踏襲)。

### Step W8: ドキュメント更新

- [ ] `docs/Vissim-CARLA-Autoware_co-sim_起動手順.md`:
  - 「0. 前提条件」に、Windows機の準備(`Co-Simulation/PTV-Vissim_windows/`一式のコピー、
    `pyzmq`/`msgpack`インストール)を追加する。
  - 「2.4 Vissim/CARLA/Autoware起動」のコマンド例・オプション一覧表を、新パラメータ
    (`vissim_adapter_host`/`vissim_adapter_port`/`vissim_connect_timeout_ms`/
    `vissim_rpc_timeout_ms`)に更新し、`vissim_network`/`vissim_lib_path`の記載を削除する。
  - Windows側`server.py`の起動コマンド例(ターミナル追加、例:
    `python server.py --vissim-network <.inpx> --vissim-connect-mode gui --vissim-version 2026`)
    を新設のセクションとして追記する。
  - 移植元`WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md`のフェーズ8(実機検証)で判明した
    運用上の注意点(Vissim`.inpx`側の「ドライブシミュレータ アクティブ」設定必須、
    `connect`タイムアウトを長めに取る必要がある等)を「トラブルシューティング」節として
    引き継ぐ。
- [ ] `/memories/repo/vissim_co-sim_docs.md`に本計画docへのリンクを追記する。

### Step W9: 実機検証

- [ ] 同一LAN内のWindows実機(Vissim 2026)にVissim側アダプタフォルダを配置し、
      本リポジトリの`autoware_carla_interface`(Linux)から接続してtick同期を確認する。
- [ ] 移植元フェーズ8で未完了だった項目(実測RTT・20Hzでの安定動作確認、run境界跨ぎの挙動、
      座標系再確認)を、本リポジトリのAutoware統合環境でも再確認する。
- [ ] EGO(Autoware制御下)・NPC(Vissim側)双方向の同期、歩行者同期、信号同期
      (`sync_traffic_lights`)がリモート構成でも従来(同一Linuxプロセス内construction)と
      同じ結果になることを確認する。

---

## 5. 実装しない/対象外

- **`Co-Simulation/PTV-Vissim_windows/`フォルダを本リポジトリ(autoware_universe)にvendorする
  こと**: このフォルダは元々「Windows機にコピーして単体で動かす」ことを目的に自己完結・
  非パッケージ化で作られており(移植元`WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md`冒頭の
  経緯を参照)、`autoware_universe`(ROS2ワークスペース)に含める意義が薄い
  (colconビルド対象にする必要がない、Windows上ではAutoware/ROS2環境を必要としない)。
  移植元リポジトリ(`/home/divp/CARLA/Co-Simulation/PTV-Vissim_windows/`)から都度コピーする
  運用を維持し、本リポジトリのドキュメントではその参照・コピー手順のみ記載する。
- Windows側アダプタのコード自体の変更(ユーザーの想定通り、Windows側実装はそのまま流用する。
  §0.1で確認済みの通り、Linux側`vissim_simulation.py`との協調に必要な変更は既に移植元で
  完結している)。
- ZeroMQ通信のTLS化/CurveZMQ導入(移植元でもユーザー判断により見送り済み。同一LAN内前提)。
- VPN/WAN越しの構成対応(移植元と同じくスコープ外)。
- `simulation_synchronization.py`/`carla_simulation.py`/`bridge_helper.py`の変更
  (§0.3の通り不要)。

---

## 6. タスクチェックリスト(サマリ)

- [x] W0: 事前準備(差分突き合わせ、pyzmq/msgpack環境確認)
- [x] W1: `rpc_protocol.py`のvendor化
- [x] W2: `vissim_simulation.py`のZeroMQクライアント化(歩行者同期含む)
- [x] W3: ROSパラメータ・`carla_autoware.py`の変更
- [x] W4: launchファイル変更(本リポジトリ + `autoware_launch`側)
- [ ] W5: 依存関係の明記(ドキュメントのみ、package.xml/setup.pyは変更なし)
- [x] W6: `NOTICE.md`更新
- [ ] W7: テスト(既存回帰確認 + 新規ループバックテスト検討)
- [ ] W8: ドキュメント更新(起動手順、repo memory)
- [ ] W9: 実機検証(Windows実機 + 本リポジトリのAutoware統合環境)
