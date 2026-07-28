# SUMO–CARLA–Autoware統合 実装ステップ計画

作成日: 2026-07-28
版: v1.0
対象ドキュメント: [SUMO_CARLA_Autoware_統合修正項目_v0.5.md](./SUMO_CARLA_Autoware_統合修正項目_v0.5.md)

`SUMO_CARLA_Autoware_統合修正項目_v0.5.md` に記載された修正項目(2章・3章・優先度A/B/C)を、依存関係の少ない順に段階分けした実装計画。各ステップは独立して動作確認・切り戻しがしやすい単位にしている。

---

## Step 0: 設計決定の確定(コード変更なし)

実装前に決めておかないと手戻りが大きい2点を先に確定させる。

- **EGO登録方式**(v0.5 2.10): 自動アダプト任せにするか、明示的に `self.sumo.spawn_actor()` で登録するか
- **`sumo_integration` パッケージの取り込み方**: `autoware_carla_interface` にコードをコピー(vendor化)するか、外部パスとして参照するか(ライセンス・保守性の観点も含め決定)

---

### Step 0での設計決定（2026-07-28）

#### ① EGO登録方式
**選択肢**
- 案A：既存のCARLA→SUMO自動アダプト機構を利用する
- 案B：初期化時に `self.sumo.spawn_actor()` を呼び出して明示登録する

**採用案：案A（自動アダプト）**

**判断理由**
- 既存実装を最大限流用できる。
- `SimulationSynchronization` の自動アダプト機構をそのまま利用できる。
- 追加実装が最小で保守性が高い。
- Autoware制御下での動作はStep6で実機検証する。

#### ② sumo_integration の取り込み方法
**選択肢**
- 案A：外部パス参照
- 案B：autoware_carla_interface配下へvendor化

**採用案：案B（vendor化）**

**判断理由**
- 外部パスへの依存を排除できる。
- Gitで一元管理できる。
- 統合時の改修を同一リポジトリで管理できる。
- 環境依存が少なく保守性・再現性が高い。

**③ 取り込み対象**(`/home/divp/CARLA/Co-Simulation/Sumo` から vendor化するもの)

- `SumoSimulation`(`sumo_integration/sumo_simulation.py`)
- `CarlaSimulation`(`sumo_integration/carla_simulation.py`。ただし2.5/2.6の改修を前提に取り込む)
- `BridgeHelper`(`sumo_integration/bridge_helper.py`)
- `constants.py`
- `vtypes.json`(`data/vtypes.json`)
- `SimulationSynchronization` の同期処理(`run_synchronization.py` 内のクラス定義から、tick()の同期ロジック部分のみを抽出)

**④ 取り込まない対象**(vendor化しないもの。v0.5 3.1/3.2/3.2-2で「重複のため削除」とした処理と対応)

- `run_synchronization.py` のCLIエントリポイント(`argparse`部分)
- `synchronization_loop()` の独立した `while True:` メインループ
- `CarlaSimulation.__init__` 内の独自の `carla.Client` 生成処理(注入方式に置き換えるため)
- `SimulationSynchronization.__init__` 内の同期モード設定(`world.apply_settings()`/`traffic_manager.set_synchronous_mode()`。`autoware_carla_interface`側の設定と重複するため)
- `CarlaSimulation.tick()` 内の `world.tick()` 呼び出し(CARLA Tickの一元化のため、tickの実行自体は行わずアクター差分検出のみ利用する)
- `synchronization_loop()` の実時間ペーシング処理(`max_real_delta_seconds` と重複するため)

## Step 1: CARLA接続の一元化(v0.5 2.5 / 2.6 / 3.2-1)

> **前提**: 本ステップの改修対象である `CarlaSimulation` は、Step 0の決定に基づき `autoware_carla_interface` 配下へvendor化した後のコードを対象とする(vendor化そのものはStep 1着手時に併せて実施する)。

- `CarlaSimulation.__init__` を改修し、外部から `client`/`world` を注入できるようにする
- `SimulationSynchronization.__init__` 内の `world.apply_settings()`/`traffic_manager.set_synchronous_mode()` の重複呼び出しを削除
- この時点ではSUMOはまだ繋がない。既存の `autoware_carla_interface`(CARLA単体)の動作に影響がないことだけを確認する

**確認方法**: 既存のCARLA単体シミュレーションが従来通り動くことを回帰確認

### Step 1実施内容（2026-07-28）

**vendor化(Step 0 ③の選定対象)** を `src/autoware_carla_interface/sumo_integration/` 以下に作成した（一式を0.4/0.5参照で`/home/divp/CARLA/Co-Simulation/Sumo/`から取得、MITライセンス表記は保持し、経緯は`sumo_integration/NOTICE.md`に記載）。

- `constants.py` / `sumo_simulation.py` / `data/vtypes.json`: **本体無修正でvendor化**
- `bridge_helper.py`: vendor化。`vtypes.json`の相対パスのみ調整(data/の配置場所が変わったため)
- `carla_simulation.py`: **修正してvendor化**。`CarlaSimulation.__init__`が自前で`carla.Client`/`World`を生成しないようにし、外部から接続済みの`client`/`world`を注入する形に変更(2.5)
- `simulation_synchronization.py`: **新規ファイル**。`run_synchronization.py`から`SimulationSynchronization`クラスのみを抽出(CLI/独立ループは非 vendor化)。`__init__`内の`world.apply_settings()`/`traffic_manager.set_synchronous_mode()`の重複呼び出しを削除(2.6)

**実施した検証**（単体レベル、mock使用）:

1. `python3 -m py_compile` で全ベンダーファイルの構文エラーが無いことを確認
2. `constants.py`/`sumo_simulation.py`/`bridge_helper.py` をオリジナルと`diff`し、ヘッダーコメントと意図した`vtypes.json`パス変更以外に差分がないことを確認
3. `CarlaSimulation(mock_client, mock_world, step_length=0.1)` をmockで生成し、（a）コンストラクタが `client`/`world`/`step_length` のみを受け取ること（host/port引数が存在しないこと）、（b）注入したclient/worldがそのまま保持されることを確認
4. `SimulationSynchronization(mock_sumo, mock_carla)` をmockで生成し、`world.apply_settings()`/`world.get_settings()`/`client.get_trafficmanager()` が**一切呼ばれない**ことを確認(2.6の重複削除を検証)
5. `data/vtypes.json` が原本とバイト単位で完全一致することを`diff`で確認

**未実施(次回以降で実施推奨)**: 実際にCARLAサーバーを起動しての end-to-end 回帰確認(colcon build → `ros2 launch` でCARLA単体シミュレーションが従来通り動作すること)。新規ファイルは既存コードから一切importされていない(`__init__.py`は空のまま)ため、既存の実行パスに影響が無いことは構造的に保証されるが、実機確認は未実施。

---

## Step 2: パラメータ追加・ステップ時間統一(v0.5 2.1 / 2.2)

- launchファイルにSUMO関連パラメータ(`.sumocfg`パス、`--sumo-gui`、`tls-manager`等)を追加(まだ未使用でOK)
- `fixed_delta_seconds` のデフォルトを0.05→0.1秒に変更(決定済み事項)

**確認方法**: CARLA単体で0.1秒ステップでも従来通り動作することを確認

### Step 2実施内容(2026-07-28)

**`launch/autoware_carla_interface.launch.xml`**

- `fixed_delta_seconds` のデフォルトを `0.05` から `0.1` に変更
- `max_real_delta_seconds` のデフォルトも `0.05` から `0.1` に変更(補足: `fixed_delta_seconds` だけを0.1に上げ`max_real_delta_seconds`を0.05のままにすると、速度倍率の上限が2倍になり実時間より速く進めなくなるという既存の1倍速上限が崩れてしまうため、揃えて更新した)
- SUMO関連パラメータを`<arg>`として新規追加(まだどのノードの`<param>`にも渡していない。ノードへの配線・Python側`declare_parameter`はStep 3で実施): `use_sumo`(既定False)、`sumo_cfg_file`(既定空文字)、`sumo_gui`(既定False)、`sumo_host`/`sumo_port`(既定"None")、`sumo_client_order`(既定1)、`sync_vehicle_lights`/`sync_vehicle_color`(既定False)、`tls_manager`(既定"none")
- `run_synchronization.py` の `--track-ego`/`--debug`/`--sync-vehicle-all` はコアの同期処理と直接関係しないため追加していない

**`README.md`**

- `fixed_delta_seconds`/`max_real_delta_seconds` のデフォルト値表記・Tips記載を `0.05` から `0.1` に更新

**実施した検証**: `python3 -m xml.dom.minidom` でlaunch XMLの構文妥当性を確認

**未実施(次回以降で実施推奨)**: 実際にCARLAサーバーを起動しての end-to-end 回帰確認(0.1秒ステップでCARLA単体シミュレーションが従来通り動作すること)。追加したSUMO関連`<arg>`はどの`<param>`にも渡していないため、既存ノードの動作には影響しない構造になっている。

---

## Step 3: SUMO起動・TraCI接続 + 同期エンジン生成(v0.5 2.3 / 2.4 / 2.7 / 2.8)

- `SumoSimulation` のインポート・起動・TraCI接続を `InitializeInterface` に組み込む
- `SimulationSynchronization`(ID対応表・座標変換込み)を生成する
- `tick()` はまだ呼ばない。接続確認のみ

**確認方法**: CARLA・SUMOの両方が起動し、正常に接続・切断(2.13相当のクリーンアップ)できることを確認

### Step 3実施内容(2026-07-28)

**`carla_ros.py`**: `_initialize_parameters()` に新規ROS 2パラメータを追加(`use_sumo`/`sumo_cfg_file`/`sumo_gui`/`sumo_host`/`sumo_port`/`sumo_client_order`/`sync_vehicle_lights`/`sync_vehicle_color`/`tls_manager`)。すべてPython側にデフォルト値を持たせ、launchファイルが未対応でも既存ノードが起動できるようにした。

**`launch/autoware_carla_interface.launch.xml`**: Step 2で追加した`<arg>`をノードの`<param>`として配線。

**`carla_autoware.py`(`InitializeInterface`)**:

- `__init__`で上記パラメータを読み込み、`self.sumo_carla_sim`/`self.sumo_sim`/`self.sumo_sync`を`None`で初期化
- 新規メソッド`_init_sumo_integration(client)`を追加。`use_sumo=False`(デフォルト)なら即return(既存動作に影響なし)。`True`の場合のみ、vendor化した`CarlaSimulation`(client/world注入版)・`SumoSimulation`・`SimulationSynchronization`を遅延import(`traci`/`sumolib`はSUMO_HOME未設定時に存在しないため、モジュールトップレベルではなく関数内でimportすることで`use_sumo=False`時の既存動作に影響しないようにしている)して構築。`sumo_host`/`sumo_port`の`"None"`センチネル文字列は実際の`None`に変換
- `load_world()`内、`CarlaDataProvider.set_client(client)`の直後・EGOスポーンの直前に`self._init_sumo_integration(client)`を呼び出し(v0.5 2.0で確定した初期化順序と一致)
- `_cleanup()`に`_cleanup_sumo()`を追加(EGOアクター破棄後・`CarlaDataProvider`破棄前)。**`SumoSimulation.close()`(`traci.close()`)のみ呼び出し**、`SimulationSynchronization.close()`(CARLA world設定のasyncモード復元・同期アクター破棄を含む)はStep 7で統合予定のため今回は呼ばない

**実施した検証**(mockベースの単体テスト・ROS 2環境上で実施):

1. `use_sumo=False`(デフォルト)で`_init_sumo_integration()`が完全なno-opであること(`sumo_carla_sim`/`sumo_sim`/`sumo_sync`が`None`のまま)を確認
2. `use_sumo=True`で、vendor化した`CarlaSimulation`/`SumoSimulation`/`SimulationSynchronization`が期待した引数(注入された`client`/`world`、`sumo_cfg_file`、`tls_manager`等)で正しく1回だけ呼ばれることを確認(`traci`/`sumolib`/`lxml.etree`はテスト側でスタブモジュールを注入)
3. `sumo_host`/`sumo_port`の`"None"`→実`None`変換、および非`"None"`文字列(`"127.0.0.1"`/`"8813"`)がそのまま渡る(`sumo_port`は`int`変換される)ことを確認
4. `_cleanup_sumo()`が`SumoSimulation.close()`を呼ぶことを確認
5. ROS 2環境(`/opt/ros/humble` + ワークスペースoverlay)を実際にsourceし、`carla_ros2_interface._initialize_parameters()`を実ノードで呼び出して新規パラメータが期待したデフォルト値で宣言されることを確認
6. `ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml --show-args` で、`fixed_delta_seconds`/`max_real_delta_seconds`が`0.1`、新規SUMO引数がすべて意図した説明文・デフォルト値で認識されることを確認(launchファイルの構文・変数参照が正しいことをROS 2自身の解析で検証)

**未実施(次回以降で実施推奨)**: 実際にCARLAサーバー・SUMOサーバーを両方起動してのend-to-end接続確認(`use_sumo=True`でノードを実行し、TraCI接続が成立すること)。`use_sumo=False`時は新規コードパスに一切入らないため、既存のCARLA単体動作への影響は構造的にないが、実機での最終確認は未実施。

---

## Step 4: メインループへの同期処理組み込み + CARLA Tick一元化(v0.5 3.1〜3.7)

- `SUMO Tick → SUMO→CARLA同期 → CARLA Tick → CARLA→SUMO同期` を主ループに組み込む
- `CarlaSimulation.tick()` からtick呼び出し部分を分離し、`world.tick()` は1ループ1回のみに限定
- ここが最も重要かつ最も大きな変更なので、他のステップより慎重にレビューする

**確認方法**: SUMO側で生成した車両がCARLAに、CARLA側(Traffic Managerでのテスト用NPC)の車両がSUMOに、それぞれ反映されることを確認。ログでtick回数が1ループ1回であることを検証

### Step 4実施内容(2026-07-28)

**`sumo_integration/carla_simulation.py`**: `tick()`を分割。

- `update_actor_diff()`(新規): `world.tick()`を呼ばずに`spawned_actors`/`destroyed_actors`/`_active_actors`のみ更新
- `tick()`: `world.tick()` → `update_actor_diff()`。単体利用時の後方互換のために残すが、配線後のメインループからは呼ばれない

**`sumo_integration/simulation_synchronization.py`**: `tick()`を分割。

- `sync_sumo_to_carla()`(新規、旧`tick()`前半 "sumo-->carla sync"相当): `self.sumo.tick()` → SUMO車両のCARLAへの生成/削除/位置反映 → (`tls_manager=='sumo'`なら)信号反映。**CARLA側は一切tickしない**
- `sync_carla_to_sumo()`(新規、旧`tick()`後半 "carla-->sumo sync"相当): 冒頭で`self.carla.update_actor_diff()`を呼んでから、CARLA車両(EGO含む、自動アダプト経由)のSUMOへの生成/削除/位置反映 → (`tls_manager=='carla'`なら)信号反映
- `tick()`: `sync_sumo_to_carla()` → `self.carla.tick()` → `sync_carla_to_sumo()`。単体利用時の後方互換のために残すが、配線後のメインループからは呼ばれない

**`carla_autoware.py`**:

- `SensorLoop.__init__`に`self.sumo_sync = None`を追加(デフォルトはSUMO無効と同じ挙動)
- `SensorLoop._tick_sensor()`を修正。`self.ego_actor.apply_control(ego_action)`の直後に`sumo_sync`があれば`sync_sumo_to_carla()`を呼び、既存の`CarlaDataProvider.get_world().tick()`(**唯一のCARLA Tick呼び出し、変更なし**)の直後に`sumo_sync`があれば`sync_carla_to_sumo()`を呼ぶ。`sumo_sync is None`(デフォルト)の場合は追加コードパスに一切入らない
- `InitializeInterface.run_bridge()`で`self.bridge_loop.sumo_sync = self.sumo_sync`を設定

**実施した検証**(mockベースの単体テスト):

1. `CarlaSimulation.update_actor_diff()`が`world.tick()`を呼ばないこと、`tick()`は`world.tick()`後に差分更新することを確認
2. `SimulationSynchronization.sync_sumo_to_carla()`が`self.carla.tick()`/`update_actor_diff()`のどちらも呼ばないこと、`sync_carla_to_sumo()`は`update_actor_diff()`のみ呼び`world.tick()`は呼ばないことを確認
3. `SimulationSynchronization.tick()`(後方互換ラッパー)が`sync_sumo_to_carla → carla.tick → update_actor_diff`の順で呼ばれることを確認
4. **`SensorLoop._tick_sensor()`の回帰確認**: `sumo_sync=None`(デフォルト)時、Step 4適用前と同じ呼び出し(`sensor()` → `apply_control()` → `world.tick()`が正確に1回)になることを確認
5. `sumo_sync`設定時、呼び出し順序が`sensor() → apply_control() → sync_sumo_to_carla() → world.tick()(1回) → sync_carla_to_sumo()`であることを確認(v0.5 3.1の確定順序と一致)
6. タイムスタンプゲートが閉じている(未経過)ケースでも、`world.tick()`と(設定時は)`sync_carla_to_sumo()`は毎ループ実行され、`sensor()`/`apply_control()`/`sync_sumo_to_carla()`はスキップされることを確認(既存のゲート挙動を維持)

**未実施(次回以降で実施推奨)**: 実際にCARLA・SUMO両サーバーを起動してのend-to-end動作確認(SUMO車両がCARLAに、CARLA車両(EGO含む)がSUMOに実際に反映されること)。優先度A項目「EGOのSUMO自動アダプト経路の動作検証(Autoware制御下)」は引き続き未実施。

---

## Step 5: NPC排他制御(v0.5 2.11)

- `use_traffic_manager=True` とSUMO使用が同時指定された場合にエラー停止する排他チェックを追加
- Step 4より前に組み込んでも良いが、自動アダプトの副作用を実際に確認してから追加した方が説得力のあるテストになるため、ここに配置

**確認方法**: 両方同時指定時に意図通りエラーになることを確認

### Step 5実施内容(2026-07-28)

**`carla_autoware.py`(`InitializeInterface`)**:

- 新規メソッド`_check_sumo_traffic_manager_exclusivity()`を追加し、`__init__`の最後(パラメータ読み込み直後、CARLA/SUMOへの接続処理より前)で呼び出す
- `use_sumo`と`use_traffic_manager`が両方`True`の場合、`ValueError`を送出して起動を停止する。それ以外の組み合わせ(片方のみ`True`、両方`False`)ではエラーにならない

**実施した検証**(mockベースの単体テスト):

1. `use_sumo=True`かつ`use_traffic_manager=True` → `ValueError`が送出されることを確認
2. `use_sumo=True`かつ`use_traffic_manager=False` → エラーにならないことを確認
3. `use_sumo=False`かつ`use_traffic_manager=True` → エラーにならないことを確認
4. 両方`False`(デフォルト) → エラーにならないことを確認

チェックは`__init__`の最後に配置しているため、CARLAへの接続やSUMOプロセスの起動が試みられる前に、不正な組み合わせを検出して即座に停止する。

---

## Step 6: EGO登録・双方向同期の検証(v0.5 2.9 / 2.10 / 3.8)

- Step 0で決めた方式(自動アダプト or 明示登録)でEGOをSUMOへ反映
- **Autoware制御下のEGO**でエンドツーエンド検証(これまでの動作確認はTraffic Manager運転のEGOのみだったため、ここが新規検証ポイント)

**確認方法**: SUMO GUI上でAutoware運転中のEGOが正しい位置・向きで追従表示されること

### Step 6着手メモ(2026-07-28、ブロック中)

このリポジトリの開発環境に**実際に起動中のCARLA 0.9.15サーバー(Town01、PID 8710/8717)**と、実行可能な`sumo`/`sumo-gui`(`SUMO_HOME=/usr/share/sumo`)が存在することを確認したため、mockではなく実サーバーに対するend-to-endテストを試みた。

**テスト方針**: Autoware実機を起動する代わりに、「Traffic Managerではなく外部から`apply_control()`される車両」をEGOの代理として使い、自動アダプト機構がAutoware(相当)制御下でも機能するかを検証する(自動アダプトはアクターの`type_id`とtransformだけを見ており、誰が`apply_control()`しているかは区別しないため、この代理は技術的に妥当)。

**発生した問題**: テストスクリプトが`world.apply_settings()`で`synchronous_mode=True`に設定した直後、CARLAサーバーが**`get_server_version()`のような軽量なRPC呼び出しにも応答しなくなり、ハングした**。`world.tick()`を挟まずに同期モードへ切り替えたことが原因と推測される。`finally`節での設定復元・アクター破棄・SUMO側のクリーンアップもすべて同じタイムアウトで失敗した。複数回再接続を試みたが、現時点でサーバーは応答しないまま。

**現在の状態**:

- CARLAサーバープロセス(PID 8710/8717)自体は稼働中(CPU使用率105%)だが、RPC応答なし
- **プロセスの再起動は行っていない**(このCARLAサーバーが他の用途で使われている可能性があるため、無断で`kill`はしない方針)
- `/memories/debugging.md`に本事象を再発防止のため記録済み(同期モード切り替え直後に`tick()`を挟まないとRPCがハングし得るという注意点)

**未実施**: 上記の理由により、Step 6本来の目的である「Autoware制御下のEGOがSUMO側に正しく追従表示されること」の実機確認はまだ完了していない。CARLAサーバーの復旧(再起動)方法についてユーザーの指示を待っている。

### Step 6実施内容(2026-07-28、CARLAサーバー再起動後に完了)

ユーザー許可のもと、ハングしたCARLAサーバー(PID 8710/8717)を`kill -9`で停止し、`/home/divp/CARLA`から`./CarlaUE4.sh`で再起動(元と同じ起動コマンド・引数なし)。再起動後、`/memories/debugging.md`に記録した教訓通り、**`synchronous_mode=True`を設定した直後に即座に`world.tick()`を呼ぶ**ことでハングを回避できることを確認した。

**テスト内容**: CARLA(Town01)に`vehicle.toyota.prius`を`world.spawn_actor()`で直接スポーン(Traffic Manager不使用)し、毎ループ`apply_control(throttle=0.5)`で直接制御(Autoware実機の代わりに「Traffic Manager以外の外部制御」を模擬)。vendor化済みの`CarlaSimulation`/`SumoSimulation`/`SimulationSynchronization`を実際に構築し、Step 4で配線した順序(`sync_sumo_to_carla → world.tick() → sync_carla_to_sumo`)を20回実行した。

**結果**:

- EGOは**1回目のループ(iteration 0)で即座に`carla2sumo_ids`へ自動登録**された(`{200: 'carla0'}`)。Traffic Manager運転ではない外部制御下の車両でも、自動アダプト機構(v0.5 2.10)が想定通り機能することを実機で確認
- CARLA側でEGOが実際に移動(20ステップで約2.19m)しており、制御が効いていることを確認
- SUMO側(`traci.vehicle.getPosition()`)でも位置が更新され、最終位置(335.55, 50.66)はCARLA側のEGO位置と整合する値になった(初回読み取り値はTraCIの「未初期化」センチネル値(`-1073741824.0`)を含んでおり、これは`subscribe()`直後・`simulationStep()`前の既知のTraCI挙動であって、本実装の不具合ではない。以降のフレームでは正常値に収束した)
- **副次的な確認**: テスト中、`sync_sumo_to_carla()`がTown01の`.rou.xml`に定義されたSUMO側のバックグラウンド交通(自転車・バイク等)を正しくCARLAへミラーリングしていることも確認できた(v0.5 3.4の記載通り)。また、自動アダプトが`vehicle.*`型の**あらゆる**CARLAアクター(前回のテストで残っていた車両含む)を無差別に拾うことも実機で再確認でき、Step 5で必須化した排他制御の妥当性が実証された

**既知の制約**: 実際のAutowareスタック(認識・計画・制御)は起動しておらず、「Autoware制御下のEGO」は`apply_control()`による直接制御で代替した。自動アダプト機構はアクターの`type_id`とtransformのみを見て制御主体を区別しないため、この代替は技術的に妥当と考えるが、Autoware実機での最終確認は別途推奨。

**後片付け**: テスト後、EGOおよびSUMO由来のミラー車両(前回の失敗テストの残骸含む、計4台)をすべて`destroy()`し、CARLAワールド設定を元の非同期モードに復元。最終確認で総アクター数173(車両0台)・同期モードFalseとなり、CARLAサーバーはテスト前の状態に復帰していることを確認した。

---

## Step 7: 終了処理の統合(v0.5 2.13 / 3.11 / 3.12)

- `InitializeInterface._cleanup()` に `SimulationSynchronization.close()` 相当の処理を統合し、二重クリーンアップを防止

**確認方法**: 異常終了(Ctrl+C、例外発生)を含む複数パターンでリソースリーク・ゾンビプロセスがないことを確認

---

## Step 8: 動作安定化(優先度B)

- ID対応表の整合性チェック、車両生成/削除の例外処理、SUMO/CARLAの時刻ずれ検出、`max_real_delta_seconds` とTraCI通信の相互作用検証など
- ここは複数の小粒PRに分けても良い

---

## Step 9以降: 機能拡張(優先度C、別タスクとして後回し推奨)

- 信号状態のLanelet2/Autowareメッセージ変換、診断トピック追加など
- 統合の主目的(EGO・NPC同期)とは独立しているため、Step 1〜8が安定してから着手するのが安全

---

## ステップ一覧(サマリ)

| Step | 内容 | 対応するSUMO Tick/CARLA Tickの実動作 | 進捗状況 |
|---|---|---|---|
| 0 | 設計決定の確定 | なし | 完了(2026-07-28) |
| 1 | CARLA接続の一元化 | なし(CARLA単体のまま) | 完了（コード・単体検証まで、2026-07-28。実機エンドツーエンド回帰確認は未実施） |
| 2 | パラメータ追加・ステップ時間統一 | なし(CARLA単体のまま) | 完了(launch/README更新まで、2026-07-28。実機回帰確認は未実施) |
| 3 | SUMO起動・TraCI接続 + 同期エンジン生成 | なし(接続確認のみ) | 完了(コード配線・単体検証まで、2026-07-28。実SUMOサーバーとのend-to-end接続確認は未実施) |
| 4 | メインループへの同期処理組み込み | **あり(最重要ステップ)** | 完了(コード配線・単体検証まで、2026-07-28。実SUMOサーバーとのend-to-end動作確認は未実施) |
| 5 | NPC排他制御 | あり | 完了(2026-07-28) |
| 6 | EGO登録・双方向同期の検証 | あり | 完了(実機end-to-end検証済み、2026-07-28。Autoware実機自体は未起動、apply_control()による代替検証) |
| 7 | 終了処理の統合 | あり(終了時) | 未着手 |
| 8 | 動作安定化 | あり | 未着手 |
| 9〜 | 機能拡張 | あり | 未着手 |
