# Vissim-CARLA-Autoware: 信号状態の直接publish 実装計画 v1.0

作成日: 2026-11-12

対象ブランチ: `feature/vissim_windows_co-sim`(autoware_universe)

移植元: `feature/sumo_co-sim` の
`simulator/autoware_carla_interface/docs/CARLA_Autoware_信号状態publish_実装計画_v1.0.md`
(以下「SUMO版計画書」)。設計の根拠と確認結果はSUMO版計画書にある。本書には、Vissim構成へ
移すときの差分と作業手順だけを書く。

参考: `docs/Vissim移植_信号状態publish_信号設置_引き継ぎ.md`(2026-10-21、未コミット)

---

## 0. 目的・スコープ

CARLAが管理している信号機の状態(赤・黄・青)を、カメラ画像による認識処理を介さずに、
Autowareへ直接通知する。SUMO版と同じ機能を、Vissim構成でも使えるようにする。

- 出力トピック: `/perception/traffic_light_recognition/traffic_signals`
- 型: `autoware_perception_msgs/msg/TrafficLightGroupArray`
- 周期: 毎tick
- 状態の変換・グループIDの決め方・パラメータは、SUMO版計画書の0章と同じ(変えない)

Vissim構成では、Vissimが決めた信号状態は既存の信号同期(`sync_traffic_lights=true`、
Vissim→CARLAの一方向)でCARLAの信号に書き込まれる。本機能はCARLAの状態を読むだけなので、
Vissimの信号状態もそのままAutowareに届く。

```text
Vissim (Windows) --RPC--> sync_vissim_to_carla() --> CARLAの信号 --> carla_ros.run_step()
  VISSIM_GetSignalStates    signal_mapping.json         (freeze済み)     _publish_traffic_lights()
                                                                          --> /perception/.../traffic_signals
```

### 0.1 スコープ外

- **Vissimの信号設置の作り直し**(引き継ぎメモの「機能B」: 交差点ごとのSignal Controller、
  `.inpx` と `signal_mapping.json` の作り直し、照合ツール `check_vissim_lanelet2_tl_ids.py`)。
  別の計画書で扱う。本計画は、今の `signal_mapping.json`(1 SC・2 SGに36信号すべてを割り当て、
  全交差点が同時に切り替わる試験用の構成)のままで、機能が動くことを確かめる。
- `map_origin_x/y`(SUMO版と同じく入れない)。
- Windows側アダプタ(`~/CARLA` の `PTV-Vissim_windows/server.py`)の変更。不要の見込み。
- `_cleanup_vissim()` で信号のfreezeを解除したあとの `reset_group()`(既存の動作。必要なら別タスク)。

---

## 1. 事前調査で分かったこと(2026-11-12)

| #   | 項目                         | 内容                                                                                                                                                                                                                                                   |
| --- | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 1   | ブランチの関係               | 2つのブランチは `main` の `6e477c645` から分かれている。SUMO版のコミット(`82819fc7b`・`4246b257d`)は `carla_autoware.py`・launchで**そのままでは当たらない**(`git apply --check` で失敗)。新規ファイルは持ってこられるが、既存ファイルは手でマージする。 |
| 2   | `carla_ros.py`               | Vissim用パラメータが追加されているが、`run_step()` の最後は `self.ego_status()` で、SUMO版と同じ位置にpublishを足せる。                                                                                                                             |
| 3   | `carla_autoware.py`          | `load_world()` の順番は「ワールド設定 → `CarlaDataProvider.set_world()` → `_init_vissim_integration()` → (ウォームアップなしなら)EGOのスポーン」。`force_green` はSUMO版と同じく `_init_vissim_integration()` の直前に入れられる。                  |
| 4   | 終了処理                     | `_cleanup()` は `sensors → ros_interface → ego → vissim → carla_provider`。`_cleanup_vissim()` はマッピング済みの信号のfreezeを解除する(`reset_group()` はしない)。                                                                                 |
| 5   | tickの順番                   | `_tick_sensor()`: `run_step()`(センサー・publish)→ `sync_vissim_to_carla()`(信号の書き込み)→ `world.tick()` → `sync_carla_to_vissim()`。SUMO版と同じく、Vissimの状態は次のtickの後にpublishされる(1 tick遅れ)。                                 |
| 6   | ウォームアップ               | `_run_vissim_warmup()` の間は `run_step()` が呼ばれないので、信号もpublishされない。EGOのスポーン後のループからpublishが始まる。                                                                                                                     |
| 7   | launch                       | `autoware_carla_interface.launch.xml` は**ノードを2回宣言している**(`use_vissim` で切り替え。Vissim側だけ `on_exit="shutdown"`)。`<param>` は両方に足す。`test/vissim_launch_params_test.py` が2つの一覧の一致と、ノードの宣言との一致を検査する。 |
| 8   | テスト                       | Vissim側のテストは `*_test.py` をスクリプトとして実行する形。`test/conftest.py` はまだない。SUMO版の信号テストは `test_*.py` でpytestを使う。                                                                                                        |
| 9   | autoware_launch              | 作業ツリーを共有しているため、`e2e_simulator.launch.xml` に `traffic_light.*` の6引数、`use_traffic_light_recognition` の切り替え、CARLA用includeへの受け渡しが**すでに入っている**。Vissimは同じincludeを使うので、追加の変更は不要。           |
| 10  | Lanelet2地図                 | `~/autoware_map/Town01/lanelet2_map_with_tl.osm`(36信号、`--stop-line-forward-offset 6`)と `carla_tl.json` が作成済み。CARLAのTown01は同じなので、そのまま使える。                                                                                |
| 11  | `signal_mapping.json`        | `{"1": {"1": [24個], "2": [12個]}}`。Town01の36信号すべてが同期対象なので、`sync_traffic_lights=true` ならEGOが見る信号もVissimの状態になる。                                                                                                        |
| 12  | Vissimの黄                   | `bridge_helper.py` で `AMBER`/`FLASHING_AMBER` → Yellow → `AMBER/SOLID_ON`。SUMOでは黄が赤で届いたが、Vissimでは黄として届く(SUMO版で確認できなかった項目)。                                                                                      |

---

## 2. Vissim用に変えるところ

SUMO版の仕様(グループIDの決め方、最も制限の強い状態を出す、対応付けできないときはpublishしない、
infoとwarningのログの呼び出し箇所を分ける)は変えない。変えるのは次の点だけ。

1. **`force_green` の排他チェック**:
   SUMO版の `validate_traffic_light_params()` は `use_sumo` と `tls_manager in ("sumo", "carla")`
   で `force_green` を拒否する。Vissim版では **`use_vissim=true` かつ `sync_traffic_lights=true`**
   のときに拒否する(Vissimが毎tick信号を上書きするので緑に固定できない)。
   引数を `use_vissim`・`sync_traffic_lights` に替える。`sync_traffic_lights=false` なら指定できるが、
   Vissimの車はVissimの信号に従い、CARLAの信号だけが緑になる(README・手順書に書く)。
2. **`force_green` の呼び出し位置**: `load_world()` で `CarlaDataProvider.set_client()` の直後、
   `_init_vissim_integration()` より前。ウォームアップありでもなしでも、EGOのスポーンより前になる。
3. **終了処理**: `_cleanup()` の `_cleanup_vissim()` の後に `_cleanup_forced_green()` を足す。
   `force_green` と `sync_traffic_lights` は排他なので、実際に動くのはどちらか一方。
4. **launch**: `traffic_light.*` の6つの `<param>` を、2つのノード宣言の**両方**に足す。
   カメラ信号認識向け3ノードの `unless` 条件はSUMO版と同じ。
5. **テストの実行形式**: SUMO版の信号テスト(`test_*.py`、pytest)と `conftest.py` はそのまま持ってくる。
   `conftest.py` を置いても、既存のVissimのテスト(`*_test.py`)の結果が変わらないことを確かめる。

---

## 3. タスク一覧

| #   | タスク                                                  | 主な変更先                                                                                                                                    |
| --- | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| V0  | 計画書の保存                                            | `docs/`                                                                                                                                       |
| V1  | ROSに依存しないロジックとテストの移植                   | `modules/traffic_light_matcher.py`・`modules/traffic_light_publisher.py`、`test/test_traffic_light_*.py`、`test/conftest.py`(新規)        |
| V2  | 地図ツールとテストの移植                                | `tools/carla_tl_to_lanelet2.py`・`tools/fix_lanelet2_duplicate_points.py`、`test/test_carla_tl_to_lanelet2.py`・`test/test_fix_lanelet2_duplicate_points.py`(新規) |
| V3  | `force_green` の排他条件をVissim用に変える              | `modules/traffic_light_publisher.py`、`test/test_traffic_light_publisher.py`                                                                  |
| V4  | ROSパラメータ、publisher、毎tickのpublish               | `carla_ros.py`                                                                                                                                |
| V5  | 起動時チェック、`force_green`、終了時の解除             | `carla_autoware.py`                                                                                                                           |
| V6  | launchの引数とカメラ信号認識ノードの停止                | `launch/autoware_carla_interface.launch.xml`、`test/vissim_launch_params_test.py`(必要なら)                                                   |
| V7  | ブリッジ単体での実機確認                                | `tools/check_tl_topic_vs_carla.py`(新規、確認用)                                                                                                                                             |
| V8  | Autoware全体での実機確認                                | `tools/monitor_tl_stop.py`(新規、確認用)                                                                                                                                             |
| V9  | ドキュメント                                            | `docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md`、`README.md`、`~/CARLA` の `TRAFFIC_SIGNAL_TODO.md`                                      |

進める順番: V0 → V1+V2 → V3 → V4 → V5 → V6 → V7 → V8 → V9

コミットはタスクごとに、`feat(autoware_carla_interface): 信号状態publish(Vissim版 Vx: ...)` /
`docs(vissim-carla-autoware co-sim): ...` の形で、日本語で書く。

---

## 4. 各タスクの内容

### V1: ロジックとテストの移植

SUMOに依存しないので、移植元の最新版をファイルごと持ってくる。

```bash
git checkout feature/sumo_co-sim -- \
  simulator/autoware_carla_interface/src/autoware_carla_interface/modules/traffic_light_matcher.py \
  simulator/autoware_carla_interface/src/autoware_carla_interface/modules/traffic_light_publisher.py \
  simulator/autoware_carla_interface/test/test_traffic_light_matcher.py \
  simulator/autoware_carla_interface/test/test_traffic_light_publisher.py \
  simulator/autoware_carla_interface/test/conftest.py
```

- `traffic_light_matcher.py` はupstream(#13327)から変更せずに移植したもの。ここでも変更しない。
- `conftest.py` は信号のテストに必要な部分だけか確かめる(移植元の版は42行で、`stand_ins.py` には依存しない)。
- 完了条件: `python3 -m pytest test/test_traffic_light_matcher.py test/test_traffic_light_publisher.py`
  が、ROSなし・ROSをsourceした環境の両方で通る(V3でテストを書き換える前の状態)。
  既存の `python3 test/vissim_*_test.py` もすべて通る。

### V2: 地図ツールとテストの移植

```bash
git checkout feature/sumo_co-sim -- \
  simulator/autoware_carla_interface/tools/carla_tl_to_lanelet2.py \
  simulator/autoware_carla_interface/tools/fix_lanelet2_duplicate_points.py \
  simulator/autoware_carla_interface/test/test_carla_tl_to_lanelet2.py \
  simulator/autoware_carla_interface/test/test_fix_lanelet2_duplicate_points.py
```

- `check_sumo_lanelet2_tl_ids.py` はSUMOのnet用なので持ってこない(Vissim版は機能Bで作る)。
- Town01の地図は作成済みのものを使うので、本計画ではツールを実行しない。Town05・Town10HDで
  使うときのために持ってくる。
- 完了条件: `python3 -m pytest test/test_carla_tl_to_lanelet2.py test/test_fix_lanelet2_duplicate_points.py` が通る。

### V3: `force_green` の排他条件

- `validate_traffic_light_params(publish, map_path, match_distance, match_ratio, force_green=False,
  use_vissim=False, sync_traffic_lights=False)` にする。
  `force_green and use_vissim and sync_traffic_lights` のとき `ValueError`。
  メッセージは「Vissimと信号を同期しているので使えない。`sync_traffic_lights:=false` にするか、
  `force_green` を外す」。
- `_TLS_MANAGERS_WITHOUT_FORCE_GREEN` などSUMO用の定数・引数は削除する(このブランチにSUMO連携はない)。
- `test_traffic_light_publisher.py` の排他テストを書き換える:
  - `use_vissim=True, sync_traffic_lights=True, force_green=True` → エラー
  - `use_vissim=True, sync_traffic_lights=False, force_green=True` → OK
  - `use_vissim=False, sync_traffic_lights=True, force_green=True` → OK(同期は動かない)
  - `force_green=False` なら組み合わせに関係なくOK

### V4: `carla_ros.py`

移植元の `git diff main...feature/sumo_co-sim -- .../carla_ros.py` のうち、信号に関係する部分だけを足す。

- `_initialize_parameters()` に `traffic_light.publish`(False)・`force_green`(False)・`map_path`("")・
  `match_distance`(5.0)・`match_ratio`(0.6)・`id_map`("") を追加する。Python側にも既定値を持たせ、
  launchが渡さなくても今までどおり動くようにする(Vissim用パラメータと同じ方針)。
- `traffic_light.publish=true` のときだけ、`TrafficLightGroupArray` のpublisher(depth 1、reliable)を作る。
- `run_step()` の最後(`self.ego_status()` の後)で `_publish_traffic_lights()` を呼ぶ。
- `_resolve_traffic_light_groups()`(最初にpublishするtickで1回だけ)、`_actor_opendrive_id()`、
  `_carla_light_map_point()` をそのまま移す。
- 完了条件: ROSをsourceした環境で `import` が通り、`traffic_light.publish=false` で従来と動作が変わらない(V7で確認)。

### V5: `carla_autoware.py`

- `__init__()` に `self._forced_green_lights = []`。
- `_check_traffic_light_params()` を追加し、既存の起動時チェック(`_check_vissim_*`)と並べて呼ぶ。
  `use_vissim=self.use_vissim`、`sync_traffic_lights=self.sync_traffic_lights` を渡す。
- `_force_green_traffic_lights()` を追加し、`load_world()` の `CarlaDataProvider.set_client(client)` の直後、
  `_init_vissim_integration(client)` の前で呼ぶ(2章の2)。
- `_cleanup_forced_green()` を追加し、`_cleanup()` の `_cleanup_vissim()` の後で呼ぶ。
  freezeの解除の後、`get_group_traffic_lights()` のまとまりごとに1回 `reset_group()` を呼ぶ。

### V6: launch

- `autoware_carla_interface.launch.xml`
  - `traffic_light.*` の6つの `<arg>` を追加する(既定値はSUMO版と同じ)。
  - 2つのノード宣言の**両方**に `<param>` を足す。`match_distance`・`match_ratio` は `float()` を通す
    (整数で指定されてもDOUBLEになるように)。
  - `traffic_light.publish=true` のときは、`traffic_light_image_relay`・`traffic_light_camera_info_relay`・
    `traffic_light_republish` を起動しない(SUMO版 `8b28a44e2` と同じ条件)。
- `python3 test/vissim_launch_params_test.py` が通ることを確かめる。通らなければテスト側の想定
  (例えば `float()` 付きの値の扱い)を直す。
- autoware_launch: 変更不要(1章の9)。`ros2 launch autoware_launch e2e_simulator.launch.xml --show-args`
  で `traffic_light.*` が出ることだけ確かめる。

### V7: ブリッジ単体での実機確認(Autowareなし)

Autowareは起動せず、`autoware_carla_interface.launch.xml` だけを起動する。信号の状態が正しく
トピックに出ているかを、確認用スクリプト `tools/check_tl_topic_vs_carla.py` でCARLAの状態と突き合わせる。

#### V7.0 準備

| 端末                   | やること                                                                                                     |
| ---------------------- | ------------------------------------------------------------------------------------------------------------ |
| Windows                | 起動手順書2.2のとおりアダプタ(`server.py`)を起動しておく。`--debug` を付けると、Vissimへのtickが追える       |
| Linux ターミナル1      | 起動手順書2.3のとおりCARLAを起動する(マップはブリッジが `map_path` の名前からTown01を読み込む)               |
| Linux ターミナル2      | ブリッジ(下の各ケースのコマンド)                                                                           |
| Linux ターミナル3      | 確認用スクリプト・`ros2` コマンド                                                                            |

ターミナル2・3では、最初に次を実行する。

```bash
source /opt/ros/humble/setup.bash
source ~/autoware.1.9.0/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=33        # 起動手順書2.6.1と同じ(Linux機のIPの末尾)
VISSIM_HOST=192.168.16.56      # Windows PCのIP
MAP_DIR=$HOME/autoware_map/Town01
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
```

- ビルドは不要(`--symlink-install` のinstallが、追加したモジュールのソースを指している)。
- 各ケースのブリッジは、確認が済んだらCtrl+Cで止める。Linux側の終了でVissimが閉じる。次のケースで
  接続できない(connectがエラーになる)ときは、起動手順書2.6.2のとおりアダプタを起動し直す。
- `vissim_sim_period` などVissimへ渡す値は全ケースで同じにしてある(値を変えるとアダプタの起動し直しが要る)。
- Vissim連携の `logging.info`/`logging.debug`(`Carla traffic light(s) mapped`・`[sync] signal ...`)は、
  ログレベルの設定がないため表示されない。Vissimの状態がCARLAに入ったかは、CARLAの画面と確認用スクリプトで見る。

ブリッジの共通の引数(ケースごとに、これに引数を足す):

```bash
COMMON="map_path:=$MAP_DIR use_vissim:=true vissim_adapter_host:=$VISSIM_HOST vissim_adapter_port:=5555 \
  vissim_connect_timeout_ms:=120000 vissim_sim_period:=600 spectator_follow:=true \
  spawn_point:=334.74,263.00,0.30,0.0,0.0,90.0"
```

(`spawn_point` はSUMO版で確認したTown01の地点。信号372の停止線の約46 m手前。EGOはAutowareなしでは動かない)

#### V7.1 従来どおり動く(publishなし)

```bash
# ターミナル2
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml $COMMON sync_traffic_lights:=true
```

```bash
# ターミナル3
ros2 node list | grep -E "traffic_light_(image_relay|camera_info_relay|republish)"
ros2 topic info /perception/traffic_light_recognition/traffic_signals
```

- 期待: 3ノードが出る。トピックのPublisher countは0(ブリッジはpublishしない)。Vissimの車がCARLAに出て、
  CARLAの信号がVissimに合わせて切り替わる(今の `signal_mapping.json` では全交差点が同時に切り替わる)。

#### V7.2 publishと位置マッチング(メインのケース)

```bash
# ターミナル2
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml $COMMON sync_traffic_lights:=true \
  traffic_light.publish:=true traffic_light.map_path:=$MAP_DIR/lanelet2_map_with_tl.osm
```

ブリッジのログに次が出ることを確かめる。

```text
Traffic-light matching: 36 matched, 0 ambiguous, 0 too far, 0 overridden (map has 36 heads / 36 groups)
```

```bash
# ターミナル3
ros2 node list | grep -E "traffic_light_(image_relay|camera_info_relay|republish)"   # 何も出ない
ros2 topic hz /perception/traffic_light_recognition/traffic_signals                  # Ctrl+Cで止める
python3 tools/check_tl_topic_vs_carla.py --duration 180
```

- `ros2 topic hz` はtickの周期と同じ程度(Vissim連携では実時間より遅くなることがある)。
- `--duration` は、Vissimの信号が少なくとも1周(青→黄→赤→青)するように決める。Vissimの信号プログラムの
  周期が分からなければ、まず180秒で実行し、`color transitions` に変化が出ているかを見る。
- 期待(確認用スクリプトの出力):

| 項目                     | 期待                                                                                                    |
| ------------------------ | ------------------------------------------------------------------------------------------------------- |
| `publishers on the topic` | `1 ['/autoware_carla_interface']`                                                                       |
| `groups per message`      | `{36: N}`(すべてのメッセージに36グループ)                                                              |
| `match (same tick)`       | ほぼ全件。`1 tick behind` は少数ならよい(tickの途中で読んだもの)。`mismatch` は0                       |
| `stamp vs CARLA elapsed`  | `spread` が0.05 s(1 tick)程度以下                                                                       |
| `colors published`        | `RED`・`AMBER`・`GREEN` が出る(**Vissimの黄がAMBERで届くこと**。SUMO版では確認できなかった項目)        |
| `color transitions`       | `GREEN->AMBER`・`AMBER->RED`・`RED->GREEN`(Vissimの `RED_AMBER` を使う信号プログラムなら、赤のまま) |
| 最後の行                  | `OK`                                                                                                   |

- あわせて、CARLAの画面(スペクテーターはEGOを追従)で信号372の色とトピックの色が同じことを目で見る:
  `ros2 topic echo /perception/traffic_light_recognition/traffic_signals --field traffic_light_groups | grep -A6 "traffic_light_group_id: 372"`
- Ctrl+Cで止めた後、V7.6を行う。

#### V7.3 地図なし(OpenDRIVE IDのまま)と id_map

```bash
# ターミナル2
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml $COMMON sync_traffic_lights:=true \
  traffic_light.publish:=true "traffic_light.id_map:=360:9000|9001"
```

- 期待: ログ `Traffic lights: publishing 35 CARLA lights using the OpenDRIVE signal id as the group id (no traffic_light.map_path), 1 overridden by traffic_light.id_map`。
  トピックに37グループ(9000・9001があり、360がない)。
- 確認用スクリプトは使わない(9000・9001はCARLAのIDではないため `group ids without a CARLA light` でNGになる)。
  `ros2 topic echo ... --once | grep traffic_light_group_id | sort | uniq | wc -l` で37を確かめる。

#### V7.4 force_green と sync_traffic_lights の排他

```bash
# ターミナル2
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml $COMMON sync_traffic_lights:=true \
  traffic_light.publish:=true traffic_light.force_green:=true
```

- 期待: Vissimに接続する前(Windows側のアダプタにconnectが届く前)に、
  `traffic_light.force_green cannot be used with use_vissim and sync_traffic_lights` のエラーで止まり、
  launch全体が終了する(`on_exit="shutdown"`)。

#### V7.5 force_green(信号同期なし)

```bash
# ターミナル2
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml $COMMON sync_traffic_lights:=false \
  traffic_light.publish:=true traffic_light.map_path:=$MAP_DIR/lanelet2_map_with_tl.osm traffic_light.force_green:=true
```

```bash
# ターミナル3
python3 tools/check_tl_topic_vs_carla.py --duration 30
```

- 期待: ブリッジのログ `traffic_light.force_green: forced 36 traffic lights to green and froze them.`。
  確認用スクリプトで `colors published` が `GREEN` だけ、`color transitions` が空、`OK`。
- Vissimの車はVissimの信号に従うので、CARLAの画面では青信号の前でVissimの車が止まることがある(想定どおり)。
- Ctrl+Cで止めたとき、ブリッジのログに `Un-froze 36 traffic lights (traffic_light.force_green) and reset N groups.` が出る
  (Town01はSUMO版の確認で12グループ)。続けてV7.6を行う。

#### V7.6 終了後にfreezeが残っていない(V7.2・V7.5の後)

```bash
# ターミナル3(ブリッジを止めた後)
python3 tools/check_tl_topic_vs_carla.py --frozen-only
```

- 期待: `CARLA traffic lights: 36, frozen: 0` と `OK`。
  V7.2の後は `_cleanup_vissim()`(マッピング済みの信号の解除)、V7.5の後は `_cleanup_forced_green()` による。

#### V7.7 ウォームアップ中はpublishしない

```bash
# ターミナル2
ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml $COMMON sync_traffic_lights:=true \
  traffic_light.publish:=true traffic_light.map_path:=$MAP_DIR/lanelet2_map_with_tl.osm vissim_warmup_time:=60
```

```bash
# ターミナル3(ブリッジを起動した直後から)
ros2 topic hz /perception/traffic_light_recognition/traffic_signals
```

- 期待: ウォームアップ中は `ros2 topic hz` に何も出ない。ブリッジのログに `[EGO SPAWN] ... result=SAFE` が出た後から
  publishが始まり、`Traffic-light matching: 36 matched, ...` のログもそのときに出る。
- `spawn_point` がVissimのリンク上にないなど、ウォームアップの条件を満たさずに起動が失敗した場合は、起動手順書2.6.3の
  「スポーン地点の選び方」で地点を選び直す(`COMMON` の `spawn_point` を置き換える)。

#### V7.8 記録

各ケースの結果(ログ、確認用スクリプトの出力)を、この節の後に「結果(V7)」として書く。

#### 結果(V7、2026-11-12、CARLA 0.9.15、Town01、Vissim 2026(Windows)、ブリッジ単体)

| ケース                 | 結果                                                                                                                                                                                                                                                                                       |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| V7.1                   | ✅ カメラ信号認識の3ノードが起動。トピックのpublisherなし(`Unknown topic`)                                                                                                                                                                                                                 |
| V7.6(V7.1の後)         | ✅ `CARLA traffic lights: 36, frozen: 0`(信号同期でfreezeした信号は `_cleanup_vissim()` で解除されている)                                                                                                                                                                                  |
| V7.2                   | ✅ ログ `Traffic-light matching: 36 matched, 0 ambiguous, 0 too far, 0 overridden (map has 36 heads / 36 groups)`。3ノードは起動しない。確認用スクリプト(180秒): CARLAのtick 1344回、メッセージ1341件、publisherはブリッジ1つ、全メッセージ36グループ、**1341件すべて同じtickで一致**(1 tick遅れ0、不一致0)、stampのずれのばらつき0.000 s。色は `GREEN`・`RED`・**`AMBER`** が届き、変化は `GREEN->AMBER` 60・`AMBER->RED` 36・`RED->GREEN` 36(SG1の24信号が青から1周してもう一度黄、SG2の12信号が赤から1周、で数が合う) |
| V7.6(V7.2の後)         | ✅ `CARLA traffic lights: 36, frozen: 0`                                                                                                                                                                                                                                                    |
| V7.3                   | ✅ トピックに37グループ(361〜395と9000・9001。360なし)。90秒間(704件)、9000・9001の色はつねにCARLAの信号360と一致(赤・黄・青すべて観測)                                                                                                                                                       |
| V7.4                   | ✅(修正後) 起動から約2秒で `Error: invalid parameters: traffic_light.force_green cannot be used with use_vissim and sync_traffic_lights: ...` を出して終了コード1で終わり、launch全体が止まった。Windows側アダプタのログに新しい接続(`Wrote co-simulation network file` / `Establishing a connection`)は出ず、Vissimに接続する前に止まったことを確認した。**修正前は、エラーの後もブリッジのプロセスが終わらなかった**(下記)                                         |
| V7.5                   | ✅ ログ `traffic_light.force_green: forced 36 traffic lights to green and froze them.`。CARLA側は36信号とも緑・freeze。確認用スクリプト(30秒): 243件すべて36グループで `GREEN` のみ、色の変化なし、不一致0、`OK`。停止時に `Un-froze 36 traffic lights (traffic_light.force_green) and reset 12 groups.` |
| V7.6(V7.5の後)         | ✅ `CARLA traffic lights: 36, frozen: 0`。1 tick進めた時点で赤24・緑12(まとまりごとの周期に戻っている)                                                                              |
| V7.7                   | ✅ `vissim_warmup_time:=60`。起動から約29秒(実時間)はトピックのメッセージ0件(publisherは起動時から1つある)。CARLAの車両0台・EGOなし。ウォームアップ後にVissimの車29台とEGOが一括でスポーンし、その直前後からpublishが始まった(以後約6.7件/秒) |

- 実時間180秒でtickは1344回(約7.5 Hz、シミュレーション時間で約67秒)。Vissim連携ではシミュレーションが実時間より遅い。
- `stamp vs CARLA elapsed` の `offset`(572 s)は、CARLAの経過時刻(以前の実行の分を含む)とブリッジのゲーム時刻の差で、一定なら問題ない。
- 準備で分かったこと(手順に関するもの):
  - `carla310_env` を有効にしたターミナルでブリッジを起動すると、`transforms3d` がなく起動直後に落ちる。`deactivate` してから起動する。
  - `--frozen-only` は、ブリッジの終了後に同期モードのまま誰もtickしないため信号が0個に見えた。スクリプトを、信号が見えなければ1回tickしてから数えるように直した。
  - CARLAに新しく接続したクライアントは、最初のtickを受け取るまで信号が見えないことがある。比較モードでも、最初に `wait_for_tick()` するようにした。
  - CARLAの起動前に、NVIDIAドライバーの自動更新(11/4)による `Driver/library version mismatch` でCARLAが起動できず、PCを再起動した(本機能とは無関係)。

- V7.5の停止時に `Timed out waiting for a reply from the vissim adapter ... (request type='disconnect')` が出た(2000 ms)。後始末は続行して正常に終わった。Vissimを閉じる処理が `vissim_rpc_timeout_ms` より長くかかった可能性がある(本機能とは無関係。V7.1〜V7.3では出ていない)。
- アダプタのログで、接続し直すたびに `spawn requested for already-tracked local_id=1, ignoring` が出る。前回のEGOの追跡が残っている(本機能とは無関係)。
- **V7.4で見つかった不具合と修正**: 起動時のチェック(`_check_vissim_*`・`_check_traffic_light_params()`)の
  `ValueError` は `InitializeInterface()` の中で出るが、その時点で `carla_ros2_interface` がROSのspinスレッド
  (daemonでない)を動かしているため、メインの処理が例外で終わってもプロセスが残り、`on_exit="shutdown"` も働かなかった
  (既存のVissim用のチェックでも同じ)。チェックで失敗したら `self.interface.shutdown()` してから例外を出し、`main()` で
  `Error: invalid parameters: ...` を1行出して終了コード1で終わるようにした(SUMO版の `main()` と同じ形)。
  - 終了時に `Spin thread did not terminate within timeout` の警告が出て2秒待つ(`shutdown()` は `destroy_node()` の後に
    spinスレッドを待ち、`rclpy.shutdown()` はその後のため)。プロセスは終わるので、そのままにしている。
  - launchの停止時に `republish` が終了コード-11で終わるのは、V7.1の失敗時にも出ている既存の動き(本機能とは無関係)。

### V8: Autoware全体での実機確認

Autoware全体(`e2e_simulator.launch.xml`)を起動し、Vissimが決めた信号でEGOが止まり・発進することを確かめる。
目的地の設定とAutoはAD API(`ros2 service call`)で行う(RVizのボタンでもよい)。走行中の様子は
監視スクリプト `tools/monitor_tl_stop.py` で記録する。

#### V8.0 シナリオと準備

- スポーン地点: `spawn_point:=334.74,263.00,0.30,0.0,0.0,90.0`(CARLA座標。道路19、V7と同じ)。
- 信号: 372(`signal_mapping.json` のSC1・SG2。SG2の12信号は同時に切り替わる)。
  CARLAの停止位置は y=308.73、Lanelet2の停止線は6 m前に出して y=314.73(地図座標では y=-314.73)。
  スポーン地点から停止線まで約52 m。
- 道路19の先はT字路。右折して道路7に入った先を目的地にする:
  CARLA (374.92, 330.61)、向き0度 → 地図座標 (374.92, -330.61)、向き0度。
- 端末はV7と同じ(Windows: アダプタ、Linux 1: CARLA、2: Autoware、3: AD APIと監視)。
  ターミナル2・3の最初の設定はV7.0と同じ(`carla310_env` を有効にしたターミナルは使わない)。
- `vissim_sim_period` はV7と同じ600(変えるとアダプタの起動し直しが要る)。Autowareの起動に
  数分かかっても、Vissim連携は実時間より遅く進むので600秒(シミュレーション時間)で足りる。

#### V8.1 起動(ターミナル2)

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$MAP_DIR \
  lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  use_vissim:=true \
  vissim_adapter_host:=$VISSIM_HOST \
  vissim_adapter_port:=5555 \
  vissim_connect_timeout_ms:=120000 \
  vissim_sim_period:=600 \
  sync_traffic_lights:=true \
  traffic_light.publish:=true \
  spawn_point:=334.74,263.00,0.30,0.0,0.0,90.0 \
  spectator_follow:=true
```

- `traffic_light.map_path` は指定しない(既定値がAutowareの読む地図 `$MAP_DIR/lanelet2_map_with_tl.osm` になる)。
- ブリッジのログに `Traffic-light matching: 36 matched, 0 ambiguous, 0 too far, ...` が出ることを確認する。

#### V8.2 publisherの確認(ターミナル3)

```bash
ros2 topic info /perception/traffic_light_recognition/traffic_signals --verbose | grep -E "Node name|Publisher count"
```

- 期待: `Publisher count: 1`、Node nameは `autoware_carla_interface`(カメラの信号認識が起動していない)。

#### V8.3 監視を始める(ターミナル3、Autoの前から最後まで動かしておく)

```bash
python3 tools/monitor_tl_stop.py --map $MAP_DIR/lanelet2_map_with_tl.osm --group 372 | tee /tmp/v8_monitor.log
```

1秒ごと(と変化したとき)に、信号372の色、`traffic-signal` の停止理由(`/api/planning/velocity_factors`)、
速度、EGOの前端から停止線までの距離を出す。Ctrl+Cで止めると、停止線の近くで止まった位置・そのときの色・
時間と、赤で停止線を越えた回数をまとめて出す。

#### V8.4 自己位置・目的地・Auto(ターミナル4、新しく開いてV7.0の設定をする)

```bash
ros2 service call /api/localization/initialize autoware_adapi_v1_msgs/srv/InitializeLocalization "{pose: []}"
```

```bash
ros2 topic echo --once /api/localization/initialization_state
```

(`state: 3`(INITIALIZED)になるまで待つ)

```bash
ros2 service call /api/routing/set_route_points autoware_adapi_v1_msgs/srv/SetRoutePoints "{header: {frame_id: map}, option: {allow_goal_modification: true}, goal: {position: {x: 374.92, y: -330.61, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, waypoints: []}"
```

- `The route is already set.` で失敗したら、次を実行してから設定し直す(SUMO版でも起きた):

```bash
ros2 service call /api/routing/clear_route autoware_adapi_v1_msgs/srv/ClearRoute "{}"
```

```bash
ros2 topic echo --once /api/operation_mode/state
```

(`is_autonomous_mode_available: true` になってから)

```bash
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}"
```

- Autoの後しばらく発進しないことがある(MPCの操舵の収束待ち。SUMO版で23〜60秒以上)。本機能とは無関係。

#### V8.5 確認すること

| #   | 確認                                     | 見るもの                                                                                                                                                     |
| --- | ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 1   | publisherが1つ                           | V8.2                                                                                                                                                         |
| 2   | 赤で止まり、青で発進する                 | 監視ログ: 赤の間に `traffic-signal factor=yes` で減速・停止し、青で `no` になって発進する。停止位置(前端から停止線までの距離)。まとめの `Crossings of the stop line on RED: 0` |
| 3   | 黄の扱い                                 | 監視ログで `AMBER` になったときの動き(止まれる距離なら止まり、近すぎれば通過する)。黄の間に停止線を越えたときは、そのときの距離と速度を記録する             |
| 4   | Vissimの車とEGOが同じ信号に従う          | CARLAの画面で、同じ進入路のVissimの車がEGOと同じ赤で止まっているか。停止位置のずれ(Vissimの信号設置は機能Bで揃える予定なので、ずれは記録のみ)           |
| 5   | 目的地に着く                             | `ros2 topic echo --once /api/routing/state` が `state: 3`(ARRIVED)                                                                                         |

- 1回の走行で赤に当たるとは限らない(SG2の周期しだい)。赤で止まる場面がなかったときは、Ctrl+Cで止めて
  V8.1からやり直す(タイミングがずれて別の色で着く)。
- 終わったら、ターミナル2をCtrl+Cで止め、`python3 tools/check_tl_topic_vs_carla.py --frozen-only` で
  freezeが残っていないことを確かめる。

#### V8.6 force_green(任意)

V8.1のコマンドを `sync_traffic_lights:=false traffic_light.force_green:=true` に替えて起動し、V8.3〜V8.4を行う。
停止理由 `traffic-signal` が一度も出ずに交差点を通過して目的地に着くことを確かめる。

#### V8.7 記録

監視ログ(`/tmp/v8_monitor.log`)のまとめ部分と、目立った場面(赤での停止、黄の扱い)を「結果(V8)」として書く。

#### 結果(V8、2026-11-12、CARLA 0.9.15、Town01、Vissim 2026(Windows)、Autoware全体)

| #   | 確認                         | 結果                                                                                                                                                                                                                                                                                                                                         |
| --- | ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | publisherが1つ               | ✅ `Publisher count: 1`(`autoware_carla_interface`)。カメラの信号認識はない                                                                                                                                                                                                                                                                 |
| 2   | 赤で止まり、青で発進する     | ✅ Autoは信号372が赤の間(監視ログ206 s)。赤のまま停止線へ近づき(最高約5.4 m/s)、`traffic-signal` で減速、前端が停止線の2.0 m手前でいったん止まったあと0.5 m手前まで詰めて停止(256.7 s〜)。約34秒停止し、青(290.5 s)の約3秒後に発進して交差点に入った。**赤で停止線を越えたことはない**(利用者もCARLAの画面で信号の遵守を確認) |
| 3   | 黄の扱い                     | 未確認。黄(AMBER)はトピックに届いた(190.5〜198.4 s、EGOはスポーン地点で停止中)が、走行中に黄に当たる場面はなかった **追記(2026-11-16)**: 機能BのB7で確認した。停止線の約24 m手前・5.5 m/sで黄になり、EGOは止まった(`Vissim_CARLA_Autoware_信号設置_実装計画_v1.0.md`) |
| 4   | Vissimの車とEGOが同じ信号に従う | ✅ 利用者がCARLAの画面で、Vissimの車もEGOと一緒に赤で止まっていたことを確認(停止位置のずれは未計測)                                                                                                                                                                                                                                                                                                                                       |
| 5   | 目的地に着く                 | 未達(本機能とは無関係)。経路は右折ではなく左折して西側を大回りするものになり、地図座標 (92.5, -139.6) で後ろからVissimのバイクに衝突されて動けなくなり、その後は左折するVissimのバスに前をふさがれた(停止理由 `route-obstacle`)                                                                                                                                                                                                                                                                              |

- 目的地に着かなかったのは、Vissimの車とEGOのやりとり(Vissim側がEGOを避けない・ふさぐ)によるもので、信号の機能とは関係しない。
  経路が大回りになった理由(目的地の向き・車線の向き)は未確認。
- 監視スクリプトの `traffic-signal factor` は信号372に限らず、経路上のどの信号の停止理由でも `yes` になる(交差点を通過した後も
  `yes` が続いたのは、先の信号のため)。また停止線から離れた場所の `front->stop line` の値は意味を持たない。

### V9: ドキュメント

- `docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md`
  - 2.6.1のオプション一覧に `traffic_light.publish`・`lanelet2_map_file`・`traffic_light.force_green` を追加する。
  - 新しい節に、信号入り地図(Town01は作成済み。作り方はREADMEへのリンク)、起動とログの確認、
    `force_green` と `sync_traffic_lights` の排他、黄の扱い、CycloneDDS、今の `signal_mapping.json` の制約を書く。
- `README.md`: パラメータ表の6行と「Publishing CARLA Traffic-Light States」節をSUMO版から持ってきて、
  排他条件をVissim用に書き換える。Known Issuesに、Vissimの信号設置が試験用の構成であることを書く。
- 本書に、V1〜V8の結果を「結果」として追記する(SUMO版計画書と同じ書き方)。
- `~/CARLA`(`feature/vissim_windows`)の `Co-Simulation/PTV-Vissim/docs/TRAFFIC_SIGNAL_TODO.md` の
  タスク9(実機検証)・10(ドキュメント)を、V7・V8の結果で更新する。
- 引き継ぎメモ(`Vissim移植_信号状態publish_信号設置_引き継ぎ.md`)は、機能Aが終わったことを書いて、
  機能Bの計画書を作るときの入力にする。

#### 結果(V9、2026-11-12)

- `docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md`
  - 2.6.1のオプション一覧に `lanelet2_map_file`・`traffic_light.publish`・`traffic_light.force_green` を追加。
  - 2.7(新規)に、信号入りの地図(2.7.1)、起動とログの確認(2.7.2)、オプション(2.7.3)、注意点(2.7.4:
    試験用の `signal_mapping.json`、黄の扱い、`sync_traffic_lights:=false` のとき、CycloneDDS、`carla310_env`、
    確認用ツール、確認済みの地点)を書いた。
- `README.md`: パラメータ表に `traffic_light.*` の6行、「Publishing CARLA Traffic-Light States」節(SUMO版から移し、
  排他条件・ウォームアップ・終了コード・確認用ツールをVissim用に書き換え)、Known Issuesに追記。
- CARLAリポジトリ `feature/vissim_windows` の `Co-Simulation/PTV-Vissim/docs/TRAFFIC_SIGNAL_TODO.md`:
  タスク9に今回の実機確認を追記(run境界は未確認のまま)、タスク10を完了にした。
- 黄の走行中の扱い(V8.5の3)は未確認のまま。ドキュメントには「黄はAMBERとして届き、止まるか通過するかはAutowareの判断」とだけ書いた。

---

## 5. リスクと対応

| リスク                                                                 | 対応                                                                                                     |
| ---------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `conftest.py` が既存のVissimのテストの読み込みを変える                 | V1で、置く前と後で `python3 test/vissim_*_test.py` と `pytest test/` の結果を比べる                       |
| launchの2つのノード宣言の片方にだけ `<param>` を足してしまう            | `test/vissim_launch_params_test.py` で検出する                                                           |
| `sync_traffic_lights=false` のとき、EGOとVissimの車が別の信号に従う    | 手順書・READMEに書く。Autowareに信号を届けるときは `sync_traffic_lights:=true` を基本にする               |
| 全交差点が同時に切り替わる構成のため、交差道路と同時に青になる          | 本計画では動作確認のみ。**機能Bで対応済み(2026-11-16)**: 交差点ごとのSignal Controllerにした(`Vissim_CARLA_Autoware_信号設置_実装計画_v1.0.md`) |
| Vissimの信号同期とpublishの間の1 tick遅れ                               | SUMO版と同じ構造。V7の3で、ずれが1 tick以内であることを確かめる                                          |
