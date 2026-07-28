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

---

## Step 5: NPC排他制御(v0.5 2.11)

- `use_traffic_manager=True` とSUMO使用が同時指定された場合にエラー停止する排他チェックを追加
- Step 4より前に組み込んでも良いが、自動アダプトの副作用を実際に確認してから追加した方が説得力のあるテストになるため、ここに配置

**確認方法**: 両方同時指定時に意図通りエラーになることを確認

---

## Step 6: EGO登録・双方向同期の検証(v0.5 2.9 / 2.10 / 3.8)

- Step 0で決めた方式(自動アダプト or 明示登録)でEGOをSUMOへ反映
- **Autoware制御下のEGO**でエンドツーエンド検証(これまでの動作確認はTraffic Manager運転のEGOのみだったため、ここが新規検証ポイント)

**確認方法**: SUMO GUI上でAutoware運転中のEGOが正しい位置・向きで追従表示されること

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
| 4 | メインループへの同期処理組み込み | **あり(最重要ステップ)** | 未着手 |
| 5 | NPC排他制御 | あり | 未着手 |
| 6 | EGO登録・双方向同期の検証 | あり | 未着手 |
| 7 | 終了処理の統合 | あり(終了時) | 未着手 |
| 8 | 動作安定化 | あり | 未着手 |
| 9〜 | 機能拡張 | あり | 未着手 |
