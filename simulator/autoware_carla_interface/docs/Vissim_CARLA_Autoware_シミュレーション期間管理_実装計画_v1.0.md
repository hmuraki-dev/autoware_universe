# Vissim(Windows)–CARLA–Autoware シミュレーション期間管理 実装計画

作成日: 2026-09-28
版: v1.0
対象ブランチ:
- 本リポジトリ(`autoware_universe`): `feature/vissim_windows_co-sim`
- CARLAリポジトリ(`C:\Users\hirokazu.muraki.bp\src\CARLA`、Linux機では`/home/divp/CARLA`):
  `feature/vissim_windows`

前提ドキュメント:
- `docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`(統合の元計画。0.3節2.「Simulation Period
  跨ぎでCARLA車両が消失」が本計画で解消される)
- `docs/Vissim_CARLA_Autoware_Windowsリモート化_実装計画_v1.0.md`(Windowsアダプタ構成。本計画で
  「Windows側は無改修で流用」という同計画§5の方針を変更する)
- `docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md`(起動手順、正本)

---

## 0. 背景と要件

### 0.1 現状の問題

- Vissimは`.inpx`の`<simulation simPeriod="...">`で設定されたシミュレーション期間が経過すると、
  シミュレーションを終了する。
- co-sim側(`autoware_carla_interface`)はシミュレーション期間を知らないため、Vissim終了後も
  応答を待ち続ける。

### 0.2 要件(ユーザー確定事項、2026-09-28)

| # | 項目 | 決定 |
|---|---|---|
| 1 | Vissimへの設定方式 | **案A**: Windowsアダプタが`.inpx`のコピーを作って値を書き換え、それを`VISSIM_Connect`に渡す |
| 2 | シミュレーション分解能 | 独立パラメータにせず、`fixed_delta_seconds`から求める(`simRes = 1 / fixed_delta_seconds`) |
| 3 | 期間経過時の停止範囲 | `ros2 launch autoware_launch e2e_simulator.launch.xml`で起動した**全ノード**を止める |
| 4 | `simPeriod`に足す余裕 | 推奨値(§2.3で**10秒**と決定)。あわせて`numRuns=1`を強制する |
| 5 | 安全策 | 入れる(連続失敗回数の上限で停止、§2.5) |
| 6 | Windows側コード | 変更する(`C:\Users\hirokazu.muraki.bp\src\CARLA`に実装) |

---

## 1. 前提調査で判明した事実

### 1.1 DS Interface(`DrivingSimulatorProxy.h`、PTV Vissim 2025で確認)

- **シミュレーション期間・分解能を設定する関数は存在しない。** 提供される関数は
  `VISSIM_Connect`/`VISSIM_ConnectToConsole`/`VISSIM_ConnectToKernel`/`VISSIM_Disconnect`/
  `VISSIM_SetDriverVehicles`(ほか車両・歩行者の設定系)/`VISSIM_DataReady`/
  `VISSIM_GetTrafficVehicles`(ほか取得系)/`VISSIM_GetSignalStates`/`VISSIM_GetLastErrorMessage`
  のみ。
- `VISSIM_Connect`の`simulatorFrequency`は「シミュレータ側のフレームレート」であり、`.inpx`の
  `simRes`を上書きしない(統合計画0.3節1.で、両者の不一致がCreateID確認失敗の原因だった実績がある)。
  → **Vissimへ期間・分解能を「セット」するには、`.inpx`そのものを書き換えるしかない**(案A採用の根拠)。
- `VISSIM_SetDriverVehicles`: 「受信直後に次フレームの計算を開始する」。つまりVissimは
  co-simのtickごとに1フレーム進む(ロックステップ)。
  → **Vissim側の経過時間 = co-simのtick成功回数 × `step_length`** で決まる。co-sim側で
  tick数を数えれば、Vissimの期間終了タイミングを正確に予測できる。
- `VISSIM_GetTrafficVehicles`: 「Vissimの計算が終わるまでブロックする」。
  → 期間経過でVissimが止まると、アダプタ(`server.py`)はDLL呼び出しの中でブロックしたまま戻らず、
  REP応答を返せない。Linux側は「2秒でtickタイムアウト → 次のtickで`connect`再送(60秒待ち)→
  タイムアウト → …」を永久に繰り返す。**これが0.1の「待ち続ける」の実体**。アダプタが固まって
  いるので、再接続も成功しない。
- `VISSIM_Disconnect`: 「シミュレーション実行を停止し、Vissimを閉じ、DLLを切断する」。
  → **「co-simからVissimへ終了を指示する」は、既存の`PTVVissimSimulation.close()`が送る
  `disconnect`メッセージでそのまま実現できる**。新しいプロトコルメッセージは不要。

### 1.2 `.inpx`の形式

平文XML。`<simulation>`要素が1つあり、対象の属性はすべてここにある(Town01で確認)。

```xml
<simulation comment="" numCores="1" numRuns="1" randSeed="42" randSeedIncr="1" retroSync="false"
  simMode="MICRO" simPeriod="300" simRes="20" simSpeed="3" startTm="0" useAllCores="true"
  useMaxSimSpeed="false" volumeIncrDynAssign="0"/>
```

### 1.3 Windowsアダプタ(`Co-Simulation/PTV-Vissim_windows/`)の現状

- `server.py`: `_handle_connect()`が`payload['step_length']`/`payload['simulator_vehicles']`のみを
  `VissimKernelSession.connect()`へ渡す。
- `vissim_kernel_session.py`: `__init__`で`network_path`(`--vissim-network`)を保持し、`connect()`で
  そのまま`VISSIM_Connect`/`VISSIM_ConnectToConsole`に渡す。**2回目以降の`connect()`は何もしない**
  (接続済みの状態で再度DLLを呼ぶとVissim GUIが無期限にハングする実績があるため)。
- `rpc_protocol.py`/`constants.py`: Linux側(`Co-Simulation/PTV-Vissim/vissim_integration/`)と
  **byte-identical**に保つ運用(`util/rpc_protocol_test.py`で一致を検査)。本リポジトリの
  `rpc_protocol.py`もプロトコル内容(`PROTO_VERSION`含む)を一致させ続ける必要がある。
- `PROTO_VERSION = 1`。バージョンが違うメッセージは`ProtocolError`になる。

### 1.4 プロトコル変更の波及範囲(重要)

`connect`メッセージの中身を変えるため、**同じプロトコルを話す3つのクライアント/サーバーすべて**を
同時に更新する必要がある。

| 実装 | 場所 | 役割 |
|---|---|---|
| Windowsアダプタ | CARLA `Co-Simulation/PTV-Vissim_windows/` | サーバー |
| CARLA公式側のLinuxオーケストレータ | CARLA `Co-Simulation/PTV-Vissim/`(`run_synchronization.py`+`vissim_integration/`) | クライアント(Autowareなしの単体co-sim用) |
| 本リポジトリ | `autoware_carla_interface/vissim_integration/` | クライアント(Autoware統合) |

CARLA公式側のLinuxオーケストレータは要件6の「Windows側」には含まれないが、アダプタとプロトコルを
共有しているため、**更新しないとアダプタと通信できなくなる**。本計画では同じCARLAリポジトリ内の
変更として一緒に更新する(Step V3)。

### 1.5 launchでの全体停止

- `autoware_carla_interface.launch.xml`の`<node>`には、現在`on_exit`/`required`の指定がない。
  そのため`autoware_carla_interface`ノードが終了しても、他のAutowareノードは動き続ける。
- ROS 2 launch(Humble)のXMLフロントエンドは`<node ... on_exit="shutdown"/>`をサポートする
  (`ExecuteProcess`の`on_exit`属性)。`Shutdown`アクションはLaunchService全体に効くため、
  `e2e_simulator.launch.xml`からincludeされていても、**`ros2 launch`で起動した全ノードが止まる**。

---

## 2. 設計

### 2.1 全体の流れ

```text
[起動時]
Linux: carla_ros.py / carla_autoware.py
  vissim_sim_period(秒) と fixed_delta_seconds を検証
  sim_res = round(1 / fixed_delta_seconds)
  end_tick = round(vissim_sim_period / fixed_delta_seconds)
  connect payload = {step_length, simulator_vehicles,
                     sim_period = vissim_sim_period + 余裕10秒, sim_res}
        │  ZeroMQ connect
        ▼
Windows: server.py → VissimKernelSession.connect()
  <network>.inpx を読み、<simulation> の simPeriod / simRes / numRuns を書き換えて
  同じフォルダの <network>.cosim.inpx に保存
  VISSIM_Connect(..., <network>.cosim.inpx, ...)

[毎tick]
SensorLoop._tick_sensor():
  sync_vissim_to_carla() → world.tick() → sync_carla_to_vissim()
  (a) vissim.tick_count >= end_tick                     → 正常終了(期間経過)
  (b) vissim.consecutive_failures >= 上限                → 異常終了(安全策)
  どちらかで running = False

[終了時]
既存 _cleanup() → _cleanup_vissim() → PTVVissimSimulation.close()
  → disconnect → VISSIM_Disconnect()(Vissimの実行停止・終了)
プロセス終了 → launch の on_exit="shutdown" → e2e_simulator の全ノード停止
```

### 2.2 パラメータ

| 名前(ROS param / launch arg) | 型 | 既定値 | 説明 |
|---|---|---|---|
| `vissim_sim_period` | int(秒) | `600` | co-simのシミュレーション期間。経過したらco-sim全体を終了する。**1以上必須**(0以下は起動時エラー) |
| `vissim_max_consecutive_failures` | int | `3` | Vissimアダプタとのtickが連続で何回失敗したら停止するか(§2.5)。**1以上必須** |
| `fixed_delta_seconds` | double | `0.05`(既存) | ここから`simRes`を求める。独立した分解能パラメータは作らない(要件2) |

- `vissim_sim_period`を整数秒にするのは、Vissimの`simPeriod`が秒単位の整数属性であるため。
- 既定値`600`(10分)は、Town01.inpxの`300`秒では短時間のAutowareテストでも足りなくなる
  可能性があるため、少し長めにした。実機運用に合わせて見直してよい。
- 余裕(10秒)と`numRuns=1`はパラメータにせず固定する(§2.3)。

**起動時の検証**(`InitializeInterface.__init__`、`use_vissim=True`の場合のみ):

1. `vissim_sim_period >= 1`
2. `vissim_max_consecutive_failures >= 1`
3. `1 / fixed_delta_seconds`が整数である(許容誤差`1e-6`)。例: `0.05`→`20`はOK、`0.03`→`33.33…`はNG
4. `sim_res`がVissimの分解能として有効な範囲(**1〜20**、Step V0で確認済み)である
5. Vissimに書き込む`simPeriod`(`vissim_sim_period + 10`)が、Vissimの上限`2678400`秒以下である
   (Step V0で確認済み)

いずれかに違反した場合は`ValueError`で起動を止める(既存の`_check_vissim_traffic_manager_exclusivity()`
と同じ方式)。

### 2.3 `simPeriod`の余裕を10秒にする理由

Vissimはロックステップで進む(§1.1)ので、原理的にはVissimの経過時間とco-simのtick数は一致する。
それでも余裕を取るのは、次の「co-simが数えるtick数より、Vissimのほうが先に進む」ケースがあるため。

- tickリクエストがLinux側でタイムアウトしたが、アダプタ側では実際には処理されてVissimが1フレーム
  進んでいた場合。Linux側は失敗として数えない(`tick_count`は成功時しか増えない)ので、Vissimの
  ほうが先行する。
- 安全策(§2.5)により連続失敗は最大`vissim_max_consecutive_failures`回で停止するので、先行量は
  数tick程度に収まる。

`fixed_delta_seconds=0.05`なら10秒は200tick分に当たり、上記の先行量に対して十分な余裕がある。
Vissimはco-simからtickされない限り進まないため、余裕を大きく取ることによるコスト(待ち時間など)は
発生しない。したがって**余裕は固定値10秒**とし、`constants.py`に定数として置く
(`VISSIM_SIM_PERIOD_MARGIN_S = 10`)。

`numRuns=1`を強制するのは、複数runの設定だと期間経過時に次のrunへ遷移し、統合計画0.3節2.の
「run境界でCARLA車両が消える」問題が起きるため。co-simが期間内に終了する本計画の設計と合わせて、
この問題は発生しなくなる。

### 2.4 `.inpx`の書き換え(Windowsアダプタ)

`VissimKernelSession`に新しいメソッド`_prepare_network_file(sim_period, sim_res)`を追加する。

- 元の`.inpx`を読み、`<simulation ...>`要素の`simPeriod`/`simRes`/`numRuns`の3属性だけを
  書き換える。
  - **XML全体をパースして書き直すことはしない**(ElementTreeで書き直すと、属性順・空白・名前空間
    表記などが変わり、Vissimが読めるか保証できないため)。`<simulation\b[^>]*>`を正規表現で
    特定し、その中の3属性の値だけを置き換える。
  - `<simulation`要素がちょうど1つでない場合、または3属性のどれかが無い場合は`RuntimeError`
    (`connect`の失敗としてLinux側に返る)。
- 書き換えたものを**元と同じフォルダ**に`<元のファイル名>.cosim.inpx`として保存する
  (例: `Town01.inpx` → `Town01.cosim.inpx`)。
  - 同じフォルダに置くのは、`.inpx`内の相対パス参照(信号制御ファイル、3Dモデルなど)を壊さない
    ため。
  - 既に存在する場合は上書きする。元の`.inpx`は一切変更しない。
  - 文字コードはUTF-8(`.inpx`の宣言通り)。バイナリで読み書きし、改行コードなどを保つ。
- `connect()`では、このコピーのパスを`VISSIM_Connect`/`VISSIM_ConnectToConsole`に渡す。
- 書き換え内容(元の値 → 新しい値)をINFOログに出す。

**2回目以降の`connect`**(既存の「何もしない」処理の拡張):

- `sim_period`/`sim_res`が初回と**同じ**なら、今まで通り何もしない。
- **違う**なら`RuntimeError`を返す(Vissimは起動済みで、`.inpx`の変更は反映できないため)。
  エラーメッセージで「アダプタ(とVissim)を再起動すること」を案内する。

### 2.5 安全策(連続失敗で停止)

`PTVVissimSimulation`に`consecutive_failures`(読み取り専用プロパティ)を追加する。

- `tick()`が最後まで完了しなかったとき(tickタイムアウト、再接続の失敗、アダプタからの`ok=False`
  応答)に1増やし、成功したら0に戻す。
- `SensorLoop`は`sync_vissim_to_carla()`の後にこの値を確認し、上限以上になったらエラーログを出して
  ループを止める。以降は正常終了と同じ終了処理(`_cleanup()`)を通る。

既定値3の場合、最悪の停止までの時間は「tickタイムアウト2秒 + 再接続タイムアウト60秒 × 2」で
約2分。Vissimのクラッシュなど、期間とは関係なくアダプタが応答しなくなった場合でも、無期限に
待ち続けることはなくなる。

> 注: アダプタがDLL呼び出しの中でブロックしたまま固まった場合、終了処理の`disconnect`も
> タイムアウトする(2秒)。`close()`は`finally`でソケットを必ず閉じるので、Linux側の終了は
> 妨げられない。固まったアダプタとVissimは手動で再起動する必要がある(起動手順書に記載する)。

### 2.6 期間経過の判定

- 判定には`PTVVissimSimulation.tick_count`を使う。成功したtickだけが数えられるので、Vissim自身の
  経過時間と一致する。CARLAの時刻(`GameTime`/snapshot)は、タイムアウトしたtickの分だけVissimと
  ずれるので使わない。
- `end_tick = round(vissim_sim_period / fixed_delta_seconds)`を起動時に1回だけ計算する。
- `tick_count >= end_tick`になったら、INFOログ(期間、tick数)を出してループを止める。

### 2.7 launchでの全体停止

`autoware_carla_interface.launch.xml`の`autoware_carla_interface`ノードに`on_exit="shutdown"`を
付ける。

- ただし`use_vissim=False`(CARLA単体)のときの挙動は変えない(これまでの「`use_vissim=False`なら
  既存動作に影響しない」という方針を維持)。そのため、ノード定義を`use_vissim`で2つに分ける
  (`if="$(var use_vissim)"`側だけに`on_exit="shutdown"`を付ける)。
  - `on_exit`は置換(`$(var ...)`)を解釈しないため(Step V0で確認済み)、1つの定義にはまとめられない。
- `on_exit`はノードがどんな理由で終了しても効く(期間経過、安全策、例外、Ctrl+C)。Vissim使用時は
  ブリッジが止まればco-sim全体が成り立たないので、どの場合も全体停止でよい。

### 2.8 プロトコル変更

- `connect`リクエストのpayloadに`sim_period`(int、余裕を足した後の値)と`sim_res`(int)を追加する。
  余裕を足すのはLinux側(アダプタは受け取った値をそのまま書くだけ)。「Linux側が単一の情報源、
  アダプタは受け取った値を使うだけ」という既存の設計に合わせる。
- `PROTO_VERSION`を`1`→`2`に上げる。古いアダプタと新しいクライアント(またはその逆)を組み合わせたとき、
  黙って動いて期間が効かないのではなく、はっきりエラーにするため。
- `rpc_protocol.py`はCARLAリポジトリの2つのコピーをbyte-identicalに保ち、本リポジトリのコピーも
  コード本体を一致させる。
- `constants.py`に`VISSIM_SIM_PERIOD_MARGIN_S = 10`を追加する(CARLAリポジトリの2つのコピー、
  本リポジトリのコピー)。

---

## 3. 変更ファイル一覧

### 3.1 CARLAリポジトリ(`C:\Users\hirokazu.muraki.bp\src\CARLA`、ブランチ`feature/vissim_windows`)

| ファイル | 変更内容 |
|---|---|
| `Co-Simulation/PTV-Vissim_windows/rpc_protocol.py` | `PROTO_VERSION`を2に上げる。docstringに`connect`の新キーを記載 |
| `Co-Simulation/PTV-Vissim_windows/constants.py` | `VISSIM_SIM_PERIOD_MARGIN_S = 10`を追加 |
| `Co-Simulation/PTV-Vissim_windows/vissim_kernel_session.py` | `connect(step_length, simulator_vehicles, sim_period, sim_res)`に変更、`_prepare_network_file()`追加、2回目以降の`connect`の値比較 |
| `Co-Simulation/PTV-Vissim_windows/server.py` | `_handle_connect()`で新キーを渡す |
| `Co-Simulation/PTV-Vissim_windows/README.md` | `.cosim.inpx`が生成されること、期間・分解能はLinux側で管理することを追記 |
| `Co-Simulation/PTV-Vissim/vissim_integration/rpc_protocol.py` / `constants.py` | Windows側とbyte-identicalになるよう同じ変更 |
| `Co-Simulation/PTV-Vissim/vissim_integration/vissim_simulation.py` | `connect` payloadに`sim_period`/`sim_res`追加、`consecutive_failures`追加 |
| `Co-Simulation/PTV-Vissim/run_synchronization.py` | `--sim-period`/`--max-consecutive-failures`引数追加、ループに期間経過・連続失敗の判定を追加 |
| `Co-Simulation/PTV-Vissim/util/vissim_kernel_session_test.py` | `.inpx`書き換え・`connect`値比較のテストを追加 |
| `Co-Simulation/PTV-Vissim/util/vissim_adapter_stub_test.py` / `rpc_protocol_test.py` | 新しい`connect` payloadに合わせて更新 |
| `Co-Simulation/PTV-Vissim/docs/WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md` / `Vissim(win)-CARLA_co-sim_起動手順.md` | 仕様変更を追記 |

### 3.2 本リポジトリ(`autoware_universe`、ブランチ`feature/vissim_windows_co-sim`)

| ファイル | 変更内容 |
|---|---|
| `vissim_integration/rpc_protocol.py` / `constants.py` | CARLAリポジトリと同じ変更 |
| `vissim_integration/vissim_simulation.py` | CARLAリポジトリと同じ変更(`connect` payload、`consecutive_failures`) |
| `vissim_integration/NOTICE.md` | vendor元の更新を記載 |
| `carla_ros.py` | `vissim_sim_period`/`vissim_max_consecutive_failures`を宣言 |
| `carla_autoware.py` | パラメータ読み込み・検証、`vissim_args`への追加、`SensorLoop`に期間経過・連続失敗の判定を追加 |
| `launch/autoware_carla_interface.launch.xml` | `<arg>`/`<param>`追加、`use_vissim`時のみ`on_exit="shutdown"` |
| `test/vissim_adapter_stub_test.py` / `test/vissim_rpc_protocol_test.py` | 新しい`connect` payload・`consecutive_failures`に合わせて更新 |
| `test/vissim_sim_period_test.py`(新規) | `SensorLoop`の期間経過・連続失敗の判定、起動時検証のテスト |
| `docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md` | 新パラメータ、`.cosim.inpx`、全体停止の挙動を追記 |
| `docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md` | 0.3節2.が本計画で解消されたことを追記 |

### 3.3 別リポジトリ(ローカル未コミット)

| ファイル | 変更内容 |
|---|---|
| `~/autoware.1.9.0/src/launcher/autoware_launch/autoware_launch/launch/e2e_simulator.launch.xml`(Linux機) | `vissim_sim_period`/`vissim_max_consecutive_failures`の引数転送を追加。このPCには無いため、変更内容を起動手順書に記載し、ユーザーがLinux機で適用する |

---

## 4. 実装ステップ

### Step V0: 事前確認(コード変更なし)

- [x] Vissimの`simRes`(1秒あたりのタイムステップ数)の有効範囲を確認する。
- [x] launch XMLの`on_exit`属性に置換(`$(var ...)`)が使えるかを確認する。
- [x] 書き換えた`.cosim.inpx`をVissim GUIで開けること、`simPeriod`/`simRes`/`numRuns`が
      反映されることを手動で確認する(2026-09-28、ユーザーがVissim 2026で確認済み)。
- [ ] (任意)現状の「期間経過後に`VISSIM_GetTrafficVehicles`がブロックする」挙動を実機で観察し、
      §1.1の分析と一致するか確認する(未実施。Step V9の実機検証で、変更後の挙動と合わせて確認する)。

### Step V0実施内容(2026-09-28)

**1. Vissimの属性の値域**

PTV Vissim 2025付属の属性一覧(`C:\Program Files\PTV Vision\PTV Vissim 2025\Doc\Eng\attribute.xlsx`、
`Attributes`シート、Object=`Simulation`)で確認した。

| 属性 | 型 | 最小 | 最大 | 既定 | 備考 |
|---|---|---|---|---|---|
| `SimRes` | unsigned int32 | 1 | 1000 | 10 | 説明文: 「値域 1〜20。値域はライセンスに依存し、既定は1〜20、アドオンモジュール'Automotive'のライセンスがある場合は1〜1000」 |
| `SimPeriod` | durationInSeconds | 1 | 2678400(31日) | 3600 | シミュレーション秒 |
| `NumRuns` | unsigned int32 | 1 | (なし) | 1 | 連続して実行するrun数 |

→ 計画への反映:
- §2.2の検証4は**`sim_res`が1〜20**とする(想定通り)。'Automotive'ライセンスがあれば20を超えても
  よいが、ライセンスの有無はLinux側から分からないため、標準の範囲で検証する。`fixed_delta_seconds`の
  既定値`0.05`(=20)はこの範囲の上限にあたる。
- §2.2の検証に**5. `vissim_sim_period + 10`(余裕込み)が2678400以下**を追加する。

**2. launchの`on_exit`属性**

ROS 2 Humbleのlaunchのソース(`ros2/launch`の`humble`ブランチ、
`launch/launch/actions/execute_process.py`の`ExecuteProcess.parse()`)を確認した。

- `on_exit`属性は文字列をそのまま`'shutdown'`と比較しており、**置換(`$(var ...)`)は解釈されない**。
  `'shutdown'`以外の値はエラーになる(対照的に`respawn`は`parser.parse_substitution()`を通すので
  置換が使える)。
- `launch_ros`の`Node.parse()`(`humble`ブランチ)は`super().parse(entity, parser, ignore=['cmd'])`で
  `ExecuteProcess.parse()`を呼ぶので、`<node>`でも同じ扱いになる。

→ 計画への反映: §2.7の通り、**ノード定義を`use_vissim`で2つに分ける**
(`if="$(var use_vissim)"`側だけに`on_exit="shutdown"`を付け、`unless="$(var use_vissim)"`側は今のまま)。
`<param>`の並びが2つに重複するため、差分が出ないよう注意して実装する(Step V6)。

**3. 確認用の`.cosim.inpx`**

Step V2で実装する予定と同じ方法(`<simulation\b[^>]*>`を正規表現で1つだけ特定し、3属性の値だけを
バイト列のまま置き換える)で、CARLAリポジトリの
`Co-Simulation/PTV-Vissim/examples/Town01/Town01.cosim.inpx`を作成した
(`simPeriod` 300→70、`simRes` 20→20、`numRuns` 1→1)。

- 元ファイルとの差分は`<simulation>`の1行だけであること、行数(22237行)が同じであることを`diff`で
  確認済み。
- このファイルはgit管理外(未追跡)。Step V2で`.gitignore`に`*.cosim.inpx`を追加する。

**GUIでの確認結果(2026-09-28、ユーザーがVissim 2026で実施)**: `Town01.cosim.inpx`について、
(a)エラーなく読み込めること、(b)「シミュレーションパラメータ」で期間が70秒・分解能が20になって
いること、(c)シミュレーションを実行して70秒で止まること、をすべて確認した。
→ §2.4の「`<simulation>`要素の3属性だけを正規表現で置き換える」方式で問題ないことが確定した。

**注意(バージョン差)**: 実運用のVissimは**2026**。1.の値域はこのPCに入っている**2025**の
`attribute.xlsx`で確認したもので、2026の資料では未確認。ただし2026のGUIで`simRes=20`・
`simPeriod=70`が期待通りに動いたことは(c)で確認できている。§1.1のDS Interfaceの内容も2025の
`DrivingSimulatorProxy.h`で確認したものだが、Windowsアダプタ(Vissim 2026で稼働実績あり)が
使っている関数・構造体と一致している。

### Step V1: プロトコル・定数の更新(CARLAリポジトリ)

- [x] `PTV-Vissim_windows/rpc_protocol.py`: `PROTO_VERSION = 2`、docstringに`connect`の新キー
      (`sim_period`、`sim_res`)を追記。
- [x] `PTV-Vissim_windows/constants.py`: `VISSIM_SIM_PERIOD_MARGIN_S = 10`を追加。
- [x] 上記2ファイルを`PTV-Vissim/vissim_integration/`へコピーし、byte-identicalであることを
      `diff`で確認。

### Step V1実施内容(2026-09-28)

- `PTV-Vissim_windows/rpc_protocol.py`: `PROTO_VERSION`を`1`→`2`に変更。`PROTO_VERSION`直前の
  コメントにバージョン履歴を追加し、v2で`connect`リクエストのpayloadに`sim_period`(int、秒、
  余裕込み)と`sim_res`(int)が加わったこと、余裕はクライアント側で足すことを記載した。
  エンベロープの形・検証処理は変更していない(新キーの検証はアダプタ側`connect()`の責務、Step V2)。
- `PTV-Vissim_windows/constants.py`: 次の定数を追加した。
  - `VISSIM_SIM_PERIOD_MARGIN_S = 10`(§2.3)
  - **計画外の追加**: `VISSIM_MIN_SIM_RES = 1`/`VISSIM_MAX_SIM_RES = 20`/
    `VISSIM_MIN_SIM_PERIOD_S = 1`/`VISSIM_MAX_SIM_PERIOD_S = 2678400`(Step V0で確認した値域)。
    §2.2の起動時検証はLinux側2か所(Step V3・V5)とアダプタ側(Step V2)の3か所で行うため、
    値域を1か所にまとめておくことにした。
- 上記2ファイルを`PTV-Vissim/vissim_integration/`へコピーし、`cmp`でbyte-identicalであることを
  確認した。改行コードはLFのまま(元と同じ)。
- 検証: `ast.parse`で構文確認、`constants`をimportして追加した定数の値を確認した。
  `rpc_protocol.py`のimport確認・`util/rpc_protocol_test.py`の実行は、このPCに`msgpack`が
  無いため未実施(Step V7でまとめて実行する)。既存テストは`rpc.PROTO_VERSION`を参照しており、
  バージョン番号を決め打ちしていないため、この変更で壊れる箇所は無いことをgrepで確認した。
- 注意: この時点では、CARLAリポジトリのアダプタとLinux側クライアントは`PROTO_VERSION=2`を名乗るが、
  まだ新キーを送受信しない(Step V2・V3で対応)。本リポジトリのコピーは`PROTO_VERSION=1`のまま
  (Step V4で対応)なので、**Step V4が終わるまで本リポジトリとアダプタは通信できない**。

### Step V2: Windowsアダプタの変更(CARLAリポジトリ)

- [x] `vissim_kernel_session.py`:
  - [x] `_prepare_network_file(sim_period, sim_res)`を追加(§2.4)。
  - [x] `connect(step_length, simulator_vehicles, sim_period, sim_res)`に変更。初回は
        `_prepare_network_file()`の結果を`VISSIM_Connect`/`VISSIM_ConnectToConsole`に渡す。
        2回目以降は値を比較し、違えば`RuntimeError`。
  - [x] 受信値の検証(`sim_period`/`sim_res`が正の整数か)。既存の`_is_valid_number()`と同じ方針。
- [x] `server.py`: `_handle_connect()`で`payload['sim_period']`/`payload['sim_res']`を渡す。
- [x] `README.md`を更新。
- [x] `.gitignore`に`*.cosim.inpx`を追加(自動生成されるコピーをコミットしないため)。

### Step V2実施内容(2026-09-28)

**`PTV-Vissim_windows/vissim_kernel_session.py`**

- モジュール関数`patch_simulation_attributes(data, attributes)`を新規追加。`.inpx`のバイト列から
  `<simulation\b[^>]*>`を1つだけ探し、指定した属性の値だけを置き換えたバイト列と、元の値を返す。
  `<simulation>`要素が1つでない場合、または属性がちょうど1つでない場合は`RuntimeError`。
  属性名は`\s`の直後から一致させるので、名前の一部が一致する別の属性(例: `xsimPeriod`)を
  誤って書き換えることはない。
  - `_prepare_network_file()`のメソッド内に書かず関数に分けたのは、ファイル入出力なしで
    単体テストできるようにするため(Step V7)。
- メソッド`_prepare_network_file(sim_period, sim_res)`を新規追加。元の`.inpx`を読み、
  `simPeriod`/`simRes`/`numRuns=1`を書き換えて`<元の名前>.cosim.inpx`(定数
  `COSIM_NETWORK_SUFFIX`)に書き出し、そのパスを返す。元の値→新しい値をINFOログに出す。
- `connect()`を`connect(step_length, simulator_vehicles, sim_period, sim_res)`に変更した。
  - 受信値の検証: `sim_period`が`[1, 2678400]`、`sim_res`が`[1, 20]`の整数(`bool`は除く)で
    あること(`constants.py`の値域定数を使用)。さらに**`sim_res == round(1 / step_length)`で
    あること**も検証する(計画外の追加。両者の不一致はCreateID確認が失敗する既知の原因なので、
    Linux側の検証をすり抜けた場合の最後の防壁としてアダプタ側でも止める)。違反は`ValueError`。
  - 接続済みで`(sim_period, sim_res)`が初回と違う場合は`RuntimeError`(アダプタとVissimの
    再起動を案内するメッセージ)。この判定は`_max_simulator_vehicles`などを更新する**前**に
    行い、拒否したリクエストで状態が変わらないようにした。
  - 初回は`_prepare_network_file()`の戻り値を`VISSIM_Connect`/`VISSIM_ConnectToConsole`に渡す。
  - 接続に成功したら`self._connected_sim_params = (sim_period, sim_res)`を記録し、
    `disconnect()`で`None`に戻す(`__init__`で`None`に初期化)。
- `__init__`: `network_path`が`.cosim.inpx`で終わる場合は`ValueError`(計画外の追加。生成した
  コピーを`--vissim-network`に指定すると、コピーを自分自身の上に書き出してしまうため)。
  DLLのロードより前に判定する。

**`PTV-Vissim_windows/server.py`**

- `_handle_connect()`で`sim_period`/`sim_res`を`session.connect()`に渡す。必須キーが欠けている
  場合は、`KeyError`ではなく欠けているキー名を示す`ValueError`にした(`ok=False`としてLinux側に返る)。

**その他**

- `PTV-Vissim_windows/README.md`: 「シミュレーション期間・分解能(`PROTO_VERSION` 2以降)」の節を
  追加(`.cosim.inpx`の生成、3属性の書き換え内容、元ファイルは変更しないこと、値を変えて再接続すると
  エラーになるのでアダプタを再起動すること)。
- リポジトリ直下の`.gitignore`に`*.cosim.inpx`を追加。Step V0で作った確認用の
  `Town01.cosim.inpx`が無視されることを`git check-ignore`で確認した。
- 既存テストの追従(新しいテストの追加はStep V7):
  - `util/vissim_kernel_session_test.py`の`check_connect_is_idempotent`: 存在しないパスの代わりに
    一時フォルダに最小の`.inpx`を作り、新しい引数で`connect()`を呼ぶように変更。`_make_session()`で
    `_connected_sim_params`も初期化。
  - `util/vissim_adapter_stub_test.py`: アダプタの`connect`を置き換えるlambdaを4引数に変更
    (クライアント側の`args`の更新はStep V3)。

**実施した検証**

1. `python util/vissim_kernel_session_test.py`(このPCで実行可能): 全チェック合格。
2. 使い捨てスクリプト(`zmq`/`msgpack`はダミーに差し替え)で次を確認した。
   - 実際の`Town01.inpx`に`patch_simulation_attributes()`を適用した結果が、Step V0で作りVissim 2026で
     動作確認した`Town01.cosim.inpx`とバイト単位で一致する。
   - `<simulation>`が0個/2個、属性が無い、名前の一部だけが一致する属性しかない、の各場合に
     `RuntimeError`になる。
   - `connect()`が元と同じフォルダの`.cosim.inpx`を`VISSIM_Connect`に渡し、元のファイルは変わらない。
     `numRuns="3"`の入力が`1`に書き換わる。
   - 同じ値での再接続は何もしない、違う値での再接続は`RuntimeError`で状態も変わらない、
     `disconnect()`後は違う値で接続できる。
   - `sim_res`と`step_length`の不一致、`sim_res`=25、`sim_period`=0/2678401/`True`が`ValueError`になる。
   - `__init__`に`.cosim.inpx`を渡すと`ValueError`になる。
   - `server._handle_connect()`が欠けたキーを名前付きで報告し、正常時は4引数で`connect()`を呼ぶ。
3. `util/vissim_adapter_stub_test.py`は`zmq`/`msgpack`/`carla`が必要なため未実行(Step V7でLinux機で実行)。

**この時点の状態**: アダプタは`sim_period`/`sim_res`を必須として受け付けるが、Linux側クライアント
(CARLA公式側・本リポジトリとも)はまだ送らない。Step V3・V4が終わるまで、どちらのクライアントも
新しいアダプタには接続できない。CARLA公式側のクライアントは`PROTO_VERSION=2`を名乗っているので
「必須キーが無い」エラー、本リポジトリのクライアントは`PROTO_VERSION=1`のままなので
「プロトコルバージョン不一致」エラーになる。

### Step V3: CARLA公式側Linuxオーケストレータの変更(CARLAリポジトリ)

- [x] `vissim_integration/vissim_simulation.py`: `connect` payloadに`sim_period`(=
      `args.sim_period + VISSIM_SIM_PERIOD_MARGIN_S`)・`sim_res`(=`round(1/step_length)`)を追加。
      `consecutive_failures`プロパティを追加(§2.5)。
- [x] `run_synchronization.py`: `--sim-period`(秒、既定600)・`--max-consecutive-failures`(既定3)を
      追加。`synchronization_loop()`の`while True:`を、期間経過・連続失敗で抜けるように変更。
      起動時検証(§2.2の1〜4)を追加。

### Step V3実施内容(2026-09-28)

**`vissim_integration/vissim_simulation.py`**

- モジュール関数`get_vissim_sim_params(step_length, sim_period)`を新規追加(計画外の設計判断)。
  §2.2の検証1・3・4・5をここに1か所でまとめ、`connect`で送る`(vissim_sim_period, sim_res)`
  (=`(sim_period + 10, round(1 / step_length))`)を返す。違反は`ValueError`。
  - 検証内容: `sim_period`が1以上の`int`(`bool`・`float`は不可)、余裕込みの期間が2678400秒以下、
    `1 / step_length`が整数(許容誤差`1e-6`)、`sim_res`が1〜20。
  - クラスの中だけでなく関数として公開したのは、呼び出し側(`run_synchronization.py`、Step V5の
    `carla_autoware.py`)がCARLAやアダプタに接続する**前**に同じ検証を呼べるようにするため。
    検証ロジックを呼び出し側にそれぞれ書くと、本リポジトリとCARLAリポジトリで食い違うおそれがある。
    Step V4でこの関数ごと本リポジトリへ持ち込み、Step V5の`_check_vissim_sim_period_params()`は
    これを呼ぶだけにする。
- `PTVVissimSimulation.__init__`:
  - 冒頭(ソケット作成より前)で`get_vissim_sim_params()`を呼び、設定が不正なら即座に失敗する。
  - `args.sim_period`を新たに読む。`connect` payloadに`sim_period`(余裕込み)・`sim_res`を追加。
  - `self._end_tick = args.sim_period * sim_res`。`sim_res = 1 / step_length`なので
    `round(sim_period / step_length)`と同じ値だが、整数の掛け算で求まるので丸め誤差が無い。
- 新しいプロパティ:
  - `end_tick`: 期間が経過する`tick_count`(§2.6)。
  - `consecutive_failures`: 連続で完了しなかった`tick()`の回数(§2.5)。
- `tick()`: 完了しなかった3か所(再接続の失敗、tickのタイムアウト、アダプタの`ok=False`)を、
  差分を空にして失敗回数を1増やす新メソッド`_record_failed_tick()`に置き換えた。成功時
  (`_tick_count += 1`の直後)に`_consecutive_failures = 0`。

**`run_synchronization.py`**

- CLI引数`--sim-period`(int、既定600)・`--max-consecutive-failures`(int、既定3)を追加。
  `--step-length`のヘルプに「1/N秒、Nは1〜20」の制約を追記。
- 引数の解析直後(CARLA・アダプタへの接続より前)に`get_vissim_sim_params()`と
  `--max-consecutive-failures >= 1`を検証し、違反は`argparser.error()`で終了する。
- `synchronization_loop()`の`while True:`内、`synchronization.tick()`の直後に次の判定を追加:
  - `tick_count >= end_tick` → INFOログ(期間・tick数)を出して`break`。
  - `consecutive_failures >= --max-consecutive-failures` → ERRORログ(アダプタとVissimの
    再起動が必要かもしれない旨)を出して`break`。
  - どちらも既存の`finally`の`synchronization.close()`を通るので、`disconnect`が送られる。

**`util/vissim_adapter_stub_test.py`**: クライアントの`args`(2か所)に`sim_period=600`を追加。

**実施した検証**(使い捨てスクリプト、`carla`/`zmq`/`msgpack`はダミー):

1. `get_vissim_sim_params()`: `(0.05, 600)`→`(610, 20)`、`(0.1, 60)`→`(70, 10)`、`(1.0, 1)`→`(11, 1)`、
   上限ちょうど`(0.05, 2678390)`→`(2678400, 20)`。`sim_period`=0/-5/60.0/`True`/2678391、
   `step_length`=0.03(1/Nでない)/0.04・0.02(範囲外)/2.0(1/Nでない)がすべて`ValueError`。
2. `PTVVissimSimulation`(`_request`をダミーに差し替え): `connect` payloadが
   `{step_length: 0.05, simulator_vehicles: 1, sim_period: 70, sim_res: 20}`、`end_tick`=1200。
   tickのタイムアウト→1、再接続のタイムアウト→2、再接続成功後のアダプタエラー→3、成功で0に戻り
   `tick_count`=1。不正な設定ではリクエストを1つも送らずに`ValueError`。
3. `synchronization_loop()`(各クラスをダミーに差し替え): 期間経過(`end_tick`=5)で5tick後に、
   連続失敗(上限3)で3回目の失敗後に、それぞれループを抜け、どちらも`close()`が呼ばれる。
4. `python run_synchronization.py`をダミーのモジュール入りで`__main__`として実行し、
   `--step-length 0.03`/`--sim-period 0`/`--max-consecutive-failures 0`がそれぞれ
   `argparser.error()`で終了すること、`--help`に新しい引数が出ることを確認。

**気づいた既存の問題(本Stepの対象外。Step V3とは別のコミットで修正済み、2026-09-28)**:

- `run_synchronization.py:214`に、本リポジトリで`2c6e850f0`として修正した不具合と同じもの
  (Vissim側で消えた車両をCARLAから消す処理が`self.vissim.destroy_actor()`を呼んでいる)が
  残っている。
- `synchronization_loop()`の`finally`は`synchronization.close()`を呼ぶが、
  `SimulationSynchronization(...)`の生成自体が例外で失敗した場合は`synchronization`が未定義のため
  `NameError`になり、元の例外が見えにくくなる。

修正内容: 1件目は`self.carla.destroy_actor()`に変更。2件目は`synchronization = None`で初期化し、
`finally`では`None`でなければ`close()`、`None`なら(アダプタには接続済みなので)
`vissim_simulation.close()`だけを呼ぶようにした。どちらも使い捨てスクリプト(ダミーのモジュール)で、
消えた車両がCARLA側で1回だけ破棄されること、`SimulationSynchronization()`の例外が`NameError`に
置き換わらずにそのまま伝わり、アダプタとの接続も閉じられることを確認した。

### Step V4: 本リポジトリのvendorファイル更新

- [x] `vissim_integration/rpc_protocol.py`/`constants.py`/`vissim_simulation.py`にStep V1/V3と
      同じ変更を適用する(本リポジトリ固有の既存デビエーション、例えば`lights_state`のenum変換は
      維持する)。
- [x] `NOTICE.md`を更新。

### Step V4実施内容(2026-09-28)

**取り込み方法**: 3ファイルとも、本リポジトリ独自の既存の違い(ヘッダーのコメント、docstringの
参照先、`lights_state`のenum変換)はそのまま残し、CARLAリポジトリのStep V1(`1d5b08e`)・
Step V3(`93fcf84`)で入った変更だけを適用した。

- `rpc_protocol.py`: `PROTO_VERSION`を`2`に変更し、バージョン履歴のコメントを追加(CARLA側と同じ文面)。
- `constants.py`: `VISSIM_SIM_PERIOD_MARGIN_S`と値域の定数4つを追加(CARLA側と同じ文面)。
  `INVALID_ACTOR_ID`以降の本文はCARLA側と完全に一致することを`diff`で確認。
- `vissim_simulation.py`: `get_vissim_sim_params()`、`__init__`冒頭の検証・`_end_tick`・
  `_consecutive_failures`、`connect` payloadの2キー、`end_tick`/`consecutive_failures`プロパティ、
  `_record_failed_tick()`とその3か所の呼び出し・成功時のリセットを適用。関数・メソッドの本文は
  手で書き写さず、CARLA側のファイルからスクリプトでそのまま切り出して挿入した(2つのリポジトリで
  内容がずれないようにするため)。
  - 確認: CARLA側との差分は、Step V3より前と同じ100行(以前からの本リポジトリ独自の違いのみ)に
    戻った。Step V3の変更を取り込む前は206行だった。

**`NOTICE.md`**: `vissim_simulation.py`/`constants.py`/`rpc_protocol.py`の各項目に、どのCARLA側
コミットの内容を取り込んだか、新たな独自の違いは無いことを追記。`rpc_protocol.py`の項目には、
同じ版以降のWindowsアダプタと組み合わせる必要があることも追記した。`simulation_synchronization.py`の
`destroy_actor`の修正は、CARLA側でも`8b90182`で同じ修正が入ったので、動作上の違いではなくなった旨を追記。

**`test/vissim_adapter_stub_test.py`**: クライアントに渡す`args`に`sim_period=600`を追加(新しい
必須の引数に追従するための最低限の修正。テストの追加はStep V7)。`test/vissim_rpc_protocol_test.py`の
`connect`のpayloadは古い形のままだが、エンベロープ層はpayloadの中身を検証しないので影響は無い
(Step V7で新しい形に合わせる)。

**実施した検証**(使い捨てスクリプト、`carla`/`zmq`/`msgpack`はダミー):

1. 変更した4ファイルの構文確認(`ast.parse`)。
2. `PROTO_VERSION == 2`、`get_vissim_sim_params(0.05, 600) == (610, 20)`。
3. `PTVVissimSimulation`(`_request`をダミーに差し替え): `connect` payloadが
   `{step_length: 0.05, simulator_vehicles: 1, sim_period: 70, sim_res: 20}`、`end_tick`=1200、
   失敗2回で`consecutive_failures`=2、再接続と成功で0に戻る。
4. 以前からの独自の違い(`turn_indicator`の変換)が壊れていない: `1`→`LEFT`、未知の`7`→警告を出して`NONE`。

**この時点の状態(重要)**: `PTVVissimSimulation`は`args.sim_period`を必須として読むが、
`carla_autoware.py`の`_init_vissim_integration()`はまだ`vissim_args`に`sim_period`を入れていない
(Step V5で対応)。そのため**Step V5が終わるまで、`use_vissim=True`で起動すると`AttributeError`で
失敗する**。`use_vissim=False`(CARLA単体)は`vissim_integration`をimportしないので影響を受けない。

### Step V5: 本リポジトリのパラメータ・メインループ変更

- [x] `carla_ros.py`: `vissim_sim_period`(INTEGER、既定600)・`vissim_max_consecutive_failures`
      (INTEGER、既定3)を宣言。
- [x] `carla_autoware.py`:
  - [x] `InitializeInterface.__init__`でパラメータを読み込み、新メソッド
        `_check_vissim_sim_period_params()`で検証(§2.2、`use_vissim=True`時のみ)。検証本体は
        Step V3で追加した`get_vissim_sim_params()`を呼ぶ(`vissim_max_consecutive_failures >= 1`だけは
        このメソッドで確認する)。
  - [x] `_init_vissim_integration()`で`vissim_args`に`sim_period`を追加(`sim_res`は
        `PTVVissimSimulation`側で`step_length`から求める)。
  - [x] `SensorLoop`に`vissim_max_consecutive_failures`を追加し、
        `_tick_sensor()`内で判定(§2.1)。`run_bridge()`で値を設定。
- [x] `use_vissim=False`時に新しいコードパスへ一切入らないことを確認。

### Step V5実施内容(2026-09-28)

**`carla_ros.py`**: パラメータの宣言表に`vissim_sim_period`(INTEGER、既定`600`)・
`vissim_max_consecutive_failures`(INTEGER、既定`3`)を追加。既存のVissim系パラメータと同じく
Python側に既定値を持たせたので、まだ渡していないlaunchファイルでもノードは起動できる。

**`carla_autoware.py`**

- `InitializeInterface.__init__`: 2つのパラメータを読み込み、既存の
  `_check_vissim_traffic_manager_exclusivity()`の直後に新メソッド`_check_vissim_sim_period_params()`を
  呼ぶ(CARLA・アダプタへの接続より前)。
- `_check_vissim_sim_period_params()`: `use_vissim=False`なら何もしない。`True`なら、取り込み済みの
  `get_vissim_sim_params(fixed_delta_seconds, vissim_sim_period)`で期間・分解能を検証し(CARLA側の
  `run_synchronization.py`と同じ関数なので検証内容が一致する)、`vissim_max_consecutive_failures >= 1`を
  確認する。違反は`ValueError`。`vissim_integration`は既存の`_init_vissim_integration()`と同じく関数内で
  importする(`use_vissim=False`のときに`zmq`などを読み込まないため)。
- `_init_vissim_integration()`: `vissim_args`に`sim_period=self.vissim_sim_period`を追加。
- `SensorLoop`:
  - 属性`vissim_max_consecutive_failures`を追加し、`run_bridge()`で設定する。
  - 新メソッド`_check_vissim_stop_conditions()`: `vissim.tick_count >= vissim.end_tick`なら期間経過、
    `vissim.consecutive_failures >= 上限`なら連続失敗として、メッセージを出して`running = False`にする
    (このファイルの既存の出力に合わせて`print`を使用)。
  - `_tick_sensor()`: `sync_carla_to_vissim()`の直後(`vissim_sync`があるときだけ)に呼ぶ。
    `sync_carla_to_vissim()`はタイムスタンプのゲートが閉じているループでも毎回実行される位置なので、
    判定も毎ループ行われる。
  - 計画からの変更: 計画では`SensorLoop`に`vissim_end_tick`も持たせる予定だったが、
    `PTVVissimSimulation.end_tick`(Step V3)を`vissim_sync.vissim`経由で直接読めばよいので、
    `SensorLoop`には上限回数だけを持たせた。
- 停止後の流れは既存のまま: `run_bridge()`のループが終わる → `main()`の`finally`で`_cleanup()` →
  `_cleanup_vissim()`が`PTVVissimSimulation.close()`を呼んで`disconnect`を送る(Vissimが終了する) →
  プロセスが終了する(→ Step V6の`on_exit="shutdown"`でlaunch全体が止まる)。

**実施した検証**(使い捨てスクリプト。`carla`/`zmq`/`msgpack`と、ROS 2に依存する`carla_ros`・
`modules.*`はダミーに差し替え):

1. `use_vissim=True`の起動時検証: `vissim_sim_period`=0、`fixed_delta_seconds`=0.03(1/Nでない)/
   0.02(分解能50で範囲外)、`vissim_max_consecutive_failures`=0、`vissim_sim_period`=2678391(余裕込みで
   上限超え)がすべて`InitializeInterface()`の時点で`ValueError`になる。
2. `use_vissim=False`なら、上記の不正な値をすべて同時に与えてもエラーにならない(従来動作のまま)。
3. `_init_vissim_integration()`が`PTVVissimSimulation`に渡す`args`の`sim_period`が60、`step_length`が0.05。
4. `SensorLoop`: `end_tick`=3なら3ループで、連続失敗の上限3なら3回目の失敗で`running`が`False`になる。
   `vissim_sync=None`なら49ループ回しても止まらない。
5. `run_bridge()`が`vissim_max_consecutive_failures`(7を指定)を`SensorLoop`に設定する。

**この時点の状態**: 本リポジトリの`autoware_carla_interface`は、新しいWindowsアダプタ(CARLA側
`2e92b25`以降)と通信できる状態になった。ただしlaunchファイルはまだ新しいパラメータを渡していない
(既定値の600秒・3回が使われる)ので、期間経過でノードは止まるが、launch全体はまだ止まらない(Step V6)。

### Step V6: launchファイルの変更

- [x] `launch/autoware_carla_interface.launch.xml`: `<arg>`/`<param>`追加、`use_vissim`時のみ
      `on_exit="shutdown"`(§2.7、Step V0の結果に従う)。
- [x] `e2e_simulator.launch.xml`(Linux機、別リポジトリ)の変更内容を記載する(本計画書の下記。
      起動手順書への反映はStep V8)。

### Step V6実施内容(2026-09-28)

**`launch/autoware_carla_interface.launch.xml`**

- `<arg>`を2つ追加: `vissim_sim_period`(既定`600`)、`vissim_max_consecutive_failures`(既定`3`)。
  説明文に「`use_vissim`のときのみ有効」「期間経過でlaunch全体が止まる」ことを書いた。
- `autoware_carla_interface`ノードを2つに分けた。
  - `unless="$(var use_vissim)"`: 従来と同じ(`on_exit`なし)。
  - `if="$(var use_vissim)"`: `on_exit="shutdown"`付き。
  - どちらも同じ22個の`<param>`(既存20個 + 新しい2個)を持つ。「2つの一覧を同じに保つこと」と、
    その理由を定義の直前のコメントに書いた。

**`<set_parameter>`で重複を避ける案を検討し、不採用にした**(計画外の検討):

`<param>`の一覧を2回書く代わりに、専用の`<group>`の中で`<set_parameter>`を並べ、その中に`if`/`unless`の
ノード定義を置けば重複を無くせる。ROS 2 Humbleのソース(`launch_ros`の`humble`ブランチ、
`launch_ros/actions/set_parameter.py`・`node.py`、`launch`の`group_action.py`)を確認した結果、
次の理由で採用しなかった。

- `SetParameter.execute()`は、`context.launch_configurations`の`global_params`リストを取り出して
  **その場で`extend()`する**。
- `<group>`(`scoped=True`)は`launch_configurations`の辞書をコピーするが、中のリストは上の階層と
  共有されたままになる。
- そのため、includeする側(`e2e_simulator.launch.xml`など)が既に`<set_parameter>`を使っていると、
  `host`/`port`/`timeout`などが、後から起動される他のAutowareノードにも付いてしまう
  (`Node`は`global_params`を全て`-p name:=value`としてコマンドラインに付ける)。

**`test/vissim_launch_params_test.py`(新規、Step V7の一部を前倒し)**

launchファイルのコメントから参照しているため、このStepで追加した。ROS 2もCARLAも不要で、XMLと
`carla_ros.py`のソースを読むだけなので、このPCでも実行できる。

1. `autoware_carla_interface`ノードがちょうど2つあり、`unless`/`if`が`use_vissim`で、`if`側だけが
   `on_exit="shutdown"`を持ち、それ以外の属性は同じ。
2. 2つの`<param>`の一覧(名前と値、順序も含む)が同じで、名前の重複も無い。
3. launchが渡す`<param>`の名前の集合が、`carla_ros.py`のパラメータ宣言表と一致する(ソースを
   `ast`で読む。`rclpy`は不要)。
4. `<param>`の値の`$(var X)`が、すべて定義済みの`<arg>`/`<let>`を参照している。

**`e2e_simulator.launch.xml`(`autoware_launch`、Linux機のローカル未コミット差分)に加える変更**

このPCには無いため、変更内容のみを記載する(Step V8で起動手順書にも反映する)。既存の`vissim_*`引数と
同じ2か所に、それぞれ2行ずつ追加する。

```xml
<!-- (1) 引数の宣言(既存の vissim_* の <arg> と同じ場所) -->
<arg name="vissim_sim_period" default="600" description="Co-simulation period (s)"/>
<arg name="vissim_max_consecutive_failures" default="3" description="Stop after this many consecutive failed vissim adapter ticks"/>

<!-- (2) autoware_carla_interface.launch.xml を include している箇所の中(既存の vissim_* の <arg> と同じ場所) -->
<arg name="vissim_sim_period" value="$(var vissim_sim_period)"/>
<arg name="vissim_max_consecutive_failures" value="$(var vissim_max_consecutive_failures)"/>
```

この変更は**値をコマンドラインで変えたい場合にのみ必要**。転送しなくても
`autoware_carla_interface.launch.xml`の既定値(600秒・3回)が使われ、`on_exit="shutdown"`はinclude先の
ノードでもLaunchService全体に効く(`Shutdown`アクションはincludeの深さに関係しない)ので、期間経過で
`e2e_simulator.launch.xml`の全ノードが止まる。

**実施した検証**

1. `xml.dom.minidom`でlaunchファイルがwell-formedであることを確認。
2. `python test/vissim_launch_params_test.py`: 全チェック合格。
3. 一時コピーの`if`側だけ`vissim_sim_period`の値を`60`に書き換えると、チェック2が差分を検出して
   失敗することを確認(テストが実際に効いていることの確認)。
4. 未実施: `ros2 launch ... --show-args`や実際の起動での確認(このPCにROS 2が無いため)。Linux機で
   Step V7・V9の際に確認する。

### Step V7: テスト

- CARLAリポジトリ:
  - [x] `util/vissim_kernel_session_test.py`: 一時フォルダに最小の`.inpx`を作り、
        `_prepare_network_file()`が3属性だけを書き換えること、元ファイルが変わらないこと、
        `<simulation>`が無い/複数ある場合にエラーになること、2回目の`connect`で値が違えば
        エラー・同じなら何もしないことを確認。
  - [x] `util/rpc_protocol_test.py`: 新しい`connect` payloadのラウンドトリップ、
        `PROTO_VERSION`不一致がエラーになること、2つのコピーがbyte-identicalであること。
  - [x] `util/vissim_adapter_stub_test.py`: 新しい`connect` payloadに合わせて更新。
  - [x] `util/run_synchronization_loop_test.py`(新規、計画外の追加): 下記参照。
- 本リポジトリ:
  - [x] `test/vissim_rpc_protocol_test.py`/`test/vissim_adapter_stub_test.py`: 同様に更新し、
        `consecutive_failures`がタイムアウトで増え、成功で0に戻ることを確認。
  - [x] `test/vissim_launch_params_test.py`(新規、Step V6で前倒しして追加済み): launchファイルの
        2つのノード定義の整合性。
  - [x] `test/vissim_sim_period_test.py`(新規、mock使用): `tick_count`が`end_tick`に達したら
        `SensorLoop`が止まること、`consecutive_failures`が上限に達したら止まること、
        `vissim_sync=None`(Vissim未使用)なら判定しないこと、起動時検証の各エラーケース。
- 実行環境の注意: このWindows PCには`carla`/`msgpack`/`zmq`が入っていないため、本リポジトリの
  テストとCARLAリポジトリのLinux側テストはLinux機で実行する。Windowsアダプタ側のテスト
  (`vissim_kernel_session_test.py`)は`carla`不要なので、このPCでも実行できる見込み。

### Step V7実施内容(2026-09-28)

**CARLAリポジトリ**

- `util/vissim_kernel_session_test.py`: 6つのチェックを追加。
  - `check_patch_simulation_attributes_only_touches_requested_attributes`: 最小の`.inpx`(タブ・他の属性・
    他の要素を含む)で、3属性の値だけが変わり他のバイトはすべて同じであること。名前の末尾だけが一致する
    属性(`xsimPeriod`)は書き換えないこと。
  - `check_patch_simulation_attributes_rejects_unexpected_files`: `<simulation>`が0個・2個、属性が無い、
    名前の一部だけ一致する属性しかない、の各場合に`RuntimeError`。
  - `check_connect_starts_vissim_with_patched_copy`: `gui`/`console`の両方で、元と同じフォルダの
    `.cosim.inpx`が`VISSIM_Connect`/`VISSIM_ConnectToConsole`に渡され、`numRuns="3"`が`1`になり、
    元のファイルが変わらないこと。
  - `check_connect_rejects_different_sim_params_when_connected`: 接続済みで値が違うと`RuntimeError`
    (メッセージに再起動の案内)、状態(`simulator_vehicles`など)も変わらず、DLLも呼ばれないこと。
    `disconnect()`後は新しい値で接続できること。
  - `check_connect_validates_sim_params`: 期間0/上限超え/`True`/`70.0`、分解能0/25、
    `step_length`との不一致がすべて`ValueError`で、DLLも呼ばれず`.cosim.inpx`も作られないこと。
  - `check_init_rejects_generated_copy_as_network_path`: `.cosim.inpx`(大文字小文字を問わない)を
    渡すと、DLLのロード前に`ValueError`。
- `util/rpc_protocol_test.py`: `check_connect_roundtrip`のpayloadを新しい形
  (`sim_period`/`sim_res`入り)にし、`PROTO_VERSION == 2`を確認。既存の
  `check_version_mismatch_rejected`・`check_windows_vendored_copy_is_identical`はそのまま有効。
- `util/vissim_adapter_stub_test.py`:
  - 偽のセッションの`connect`が受け取った引数を`connect_calls`に記録するようにした。
  - `check_timeout_triggers_reconnect`に、タイムアウト後`consecutive_failures == 1`・`tick_count == 0`、
    再接続・成功後`consecutive_failures == 0`・`tick_count == 1`、再接続で同じ`connect`の値が
    送られること、を追加。
  - `check_connect_sends_sim_period_and_resolution`(新規): `sim_period=60`のクライアントが
    `connect(0.05, 5, 70, 20)`を送り、`end_tick == 1200`であること。
- `util/run_synchronization_loop_test.py`(新規、計画外の追加): Step V3で入れた`run_synchronization.py`の
  ループ終了を直接確かめるテストが無かったため追加した。`get_vissim_sim_params()`の正常値・境界値・
  エラーの各ケース、期間経過・連続失敗でループを抜けて`close()`が呼ばれること、
  `SimulationSynchronization()`の生成が失敗したときに元の例外がそのまま伝わりアダプタとの接続も
  閉じられること(`8b90182`の回帰テスト)。

**本リポジトリ**

- `test/vissim_rpc_protocol_test.py`: CARLA側と同じく`connect`のpayloadを新しい形にし、
  `PROTO_VERSION == 2`を確認。
- `test/vissim_adapter_stub_test.py`: 偽のアダプタが受け取った`connect`のpayloadを`connect_requests`に
  記録するようにし、CARLA側と同じ内容を`check_timeout_triggers_reconnect`に追加、
  `check_connect_sends_sim_period_and_resolution`(新規)でpayloadが
  `{step_length: 0.05, simulator_vehicles: 5, sim_period: 70, sim_res: 20}`・`end_tick == 1200`で
  あることを確認。
- `test/vissim_sim_period_test.py`(新規): 7つのチェック(起動時検証のエラー6ケース、`use_vissim=False`
  では不正な値も無視、`sim_period`が`PTVVissimSimulation`まで渡る、期間経過で5ループ目に停止、
  連続失敗3回で停止、Vissim未使用なら止まらない、`run_bridge()`が上限回数を`SensorLoop`に渡す)。
  `carla`/`zmq`/`msgpack`とROS 2依存の`carla_ros`/`modules.*`は、`run()`の間だけ
  `mock.patch.dict(sys.modules)`でダミーに差し替える(テストランナーが他のテストと同じプロセスで
  このファイルを読み込んでも、ダミーが漏れないようにするため)。そのため**ROS 2もCARLAも無い環境で
  実行できる**。

**このPCで実行した結果**

| テスト | 結果 | 備考 |
|---|---|---|
| CARLA `util/vissim_kernel_session_test.py` | 合格 | 追加の依存なし |
| CARLA `util/run_synchronization_loop_test.py` | 合格 | このPCに無い`carla`/`zmq`/`msgpack`だけダミーに差し替えて実行 |
| 本リポジトリ `test/vissim_sim_period_test.py` | 合格 | 実行後に`sys.modules`へダミーが残らないことも確認 |
| 本リポジトリ `test/vissim_launch_params_test.py` | 合格 | |

さらに、新しいテストが実際に不具合を検出できるかを、実装をわざと壊した状態で確認した(いずれも検出):
`numRuns`を1にしない、接続済みで値が違う`connect`を受け入れる、`SensorLoop`が停止判定を呼ばない
(期間経過・連続失敗の両方)、起動時検証をしない。

**Linux機で実行が必要なもの**(ZeroMQの実際の通信・`msgpack`・`carla`が必要なため、このPCでは未実行):

- CARLA: `util/rpc_protocol_test.py`、`util/vissim_adapter_stub_test.py`、
  `util/run_synchronization_loop_test.py`(本物の`carla`で)、既存の`util/pedestrian_sync_stub_test.py`・
  `util/signal_sync_stub_test.py`(回帰確認)
- 本リポジトリ: `test/vissim_rpc_protocol_test.py`、`test/vissim_adapter_stub_test.py`、既存の
  `test/vissim_pedestrian_sync_stub_test.py`(回帰確認)

**Linux機での実行結果(2026-09-28、ユーザーが`DIVP-WS03`で実施、`carla310_env`環境)**: 全11本合格。

| リポジトリ | テスト | 結果 |
|---|---|---|
| CARLA | `util/rpc_protocol_test.py` | 合格 |
| CARLA | `util/vissim_adapter_stub_test.py` | 合格(2回実行し2回とも合格。タイムアウトのテストもタイミングに左右されず安定) |
| CARLA | `util/run_synchronization_loop_test.py` | 合格(本物の`carla`で実行) |
| CARLA | `util/vissim_kernel_session_test.py` | 合格 |
| CARLA | `util/pedestrian_sync_stub_test.py`(回帰確認) | 合格 |
| CARLA | `util/signal_sync_stub_test.py`(回帰確認) | 合格 |
| 本リポジトリ | `test/vissim_rpc_protocol_test.py` | 合格 |
| 本リポジトリ | `test/vissim_adapter_stub_test.py` | 合格 |
| 本リポジトリ | `test/vissim_sim_period_test.py` | 合格 |
| 本リポジトリ | `test/vissim_launch_params_test.py` | 合格 |
| 本リポジトリ | `test/vissim_pedestrian_sync_stub_test.py`(回帰確認) | 合格 |

出力に含まれる`ERROR`/`WARNING`ログは、いずれもテストが意図的に起こしている状況(タイムアウト、連続失敗、
上限数、type 300の歩行者、スタブの信号機が一部しか無いこと)によるもので、想定どおり。

**launch引数の表示確認(2026-09-28、Linux機`DIVP-WS03`でビルド後に実施)**:
`ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml --show-args`と
`ros2 launch autoware_launch e2e_simulator.launch.xml --show-args`の両方で、`vissim_sim_period`
(既定`'600'`)・`vissim_max_consecutive_failures`(既定`'3'`)が説明文つきで表示されることを確認した。
ただし`--show-args`はinclude先のファイルの引数もまとめて表示するので、`e2e_simulator.launch.xml`側の
表示だけでは、同ファイル自体に追加されたかは判断できない。そこで、Linux機の
`~/autoware.1.9.0/src/launcher/autoware_launch/autoware_launch/launch/e2e_simulator.launch.xml`を
`grep`し、起動手順書2.6.1の4行(宣言2行: 40・41行目、include先への受け渡し2行: 101・102行目)が
入っていることを確認した(変更はこのPCで行い、ユーザーがLinux機に反映)。

### Step V8: ドキュメント更新

- [x] 本リポジトリ`docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md`: 新パラメータ、
      `.cosim.inpx`の生成、期間経過・連続失敗で全ノードが止まること、アダプタが固まった場合の
      再起動手順、`e2e_simulator.launch.xml`の変更内容。
- [x] 本リポジトリ`docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`: 0.3節2.が本計画で解消された
      ことを追記。
- [x] CARLAリポジトリ`docs/WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md`/
      `Vissim(win)-CARLA_co-sim_起動手順.md`: 同様に追記。

### Step V8実施内容(2026-09-28)

**本リポジトリ`docs/Vissim(win)-CARLA-Autoware_co-sim_起動手順.md`**

- 「0. 前提条件」: `.inpx`の`simPeriod`/`simRes`/`numRuns`は設定不要になったこと、CARLAリポジトリと
  本リポジトリは`PROTO_VERSION`が一致する組み合わせで使うこと、を追加。
- 「2.2 アダプタ起動」: 「シミュレーション期間・分解能について」を追加(`.cosim.inpx`の生成と書き換わる
  3つの値、`.inpx`のフォルダに書き込み権限が必要なこと、`.cosim.inpx`を直接指定しないこと、期間・
  ステップ時間を変えたらアダプタとVissimも起動し直すこと)。
- 「2.6 Vissim/CARLA/Autoware起動」: コマンド例に`vissim_sim_period:=600`を追加。オプション表に
  `vissim_sim_period`・`vissim_max_consecutive_failures`を追加。「注意」の「`.inpx`側の`simRes`を
  `fixed_delta_seconds`に必ず合わせること」を、「自動で書き込まれるので手で合わせる必要はない、ただし
  `fixed_delta_seconds`は1/N秒(N=1〜20)にすること」に書き換えた。
- 「2.6.1 `autoware_launch`側の対応」(新設): `e2e_simulator.launch.xml`に追加する4行(Step V6に記載した
  もの)。
- 「2.6.2 終了時の動作」(新設): 期間経過・連続失敗・Ctrl+Cそれぞれのログと、Vissimがどうなるか、
  期間の数え方(tickの成功回数)、`use_vissim:=false`では全体停止しないこと、連続失敗で止まったときの
  アダプタの再起動手順。

**本リポジトリ`docs/Vissim_CARLA_Autoware_統合_実装計画_v1.0.md`**

- 0.3節2.(run境界跨ぎで車両が消える、【未解決】)に「【解消・2026-09-28】」の注記を追加。Vissimが
  run境界に達する前にco-simが終了する構成になったので発生しなくなったこと、ただし現象そのものの原因は
  未調査のままであることを明記した。
- 5章「優先度A」の同じ項目、7章「実装時に確認が必要な事項」の期間の項目とステップ時間の一致の項目に、
  解決済みの旨を追記。

**CARLAリポジトリ`docs/WINDOWS_VISSIM_REMOTE_IMPLEMENTATION_PLAN.md`**

- 3.3節(`connect`の仕様): payloadの例を`PROTO_VERSION` 2の形にし、`sim_period`/`sim_res`の説明を追加。
- 4.8節(フェーズ8)の「run境界跨ぎの挙動を確認する」項目に、4.9節により確認不要になった旨を追記。
- 4.9節「フェーズ9: シミュレーション期間管理(`PROTO_VERSION` 2)」を新設(背景、方式、変更したファイルと
  コミット、既存不具合の修正、テスト結果、残りの実機検証)。詳細は本計画書を参照する形にした。

**CARLAリポジトリ`docs/Vissim(win)-CARLA_co-sim_起動手順.md`**

- 「0.」: `.inpx`の3つの値は設定不要になったことを追加。
- 「2. アダプタ起動」: 本リポジトリの起動手順書と同じ「シミュレーション期間・分解能について」を追加
  (引数名は`run_synchronization.py`のもの)。
- 「3.4 run_synchronization.py」: コマンド例に`--sim-period 600`を追加。オプション表に`--sim-period`・
  `--max-consecutive-failures`を追加し、`--step-length`に1/N秒の制約を追記。「終了時の動作」を追加。

**変更しなかったもの**: 本リポジトリの`README.md`(Vissim関連のパラメータがそもそも記載されていないため)、
CARLAリポジトリの`PTV-Vissim_windows/README.md`(Step V2で更新済み)。

### Step V9: 実機検証

- [ ] `vissim_sim_period`を短め(例: 60秒)にして起動し、次を確認する。
  - [ ] Windows側に`.cosim.inpx`が生成され、`simPeriod=70`/`simRes=20`/`numRuns=1`になっている。
  - [ ] 60秒(1200tick)経過でco-simが終了し、Vissimが閉じ、`e2e_simulator.launch.xml`の全ノードが
        止まる。
  - [ ] CARLA上にVissim由来のアクターが残らない。
- [ ] 実行中にWindows側アダプタを強制終了し、安全策(連続失敗3回)で停止すること、全ノードが
      止まることを確認する。
- [ ] アダプタを再起動せずに`vissim_sim_period`を変えて再接続した場合、エラーになることを確認する。
- [ ] `use_vissim=False`(CARLA単体)で、従来通り動作し、ブリッジ終了で全体停止しないことを確認する。

---

## 5. 対象外

- COM APIによるVissim操作(§2案B。不採用)。
- シミュレーション期間の途中変更、一時停止・再開。
- `numRuns`>1の複数run連続実行。
- アダプタがDLL呼び出しの中で固まった場合の自動復旧(手動再起動とする、§2.5注)。

---

## 6. タスクチェックリスト(サマリ)

- [x] V0: 事前確認(`simRes`範囲、`on_exit`の置換可否、`.cosim.inpx`のGUI確認)
- [x] V1: プロトコル・定数の更新(CARLAリポジトリ)
- [x] V2: Windowsアダプタの変更(CARLAリポジトリ)
- [x] V3: CARLA公式側Linuxオーケストレータの変更(CARLAリポジトリ)
- [x] V4: 本リポジトリのvendorファイル更新
- [x] V5: 本リポジトリのパラメータ・メインループ変更
- [x] V6: launchファイルの変更
- [x] V7: テスト(このPCで実行できるものは合格。残りはLinux機での実行待ち)
- [x] V8: ドキュメント更新
- [ ] V9: 実機検証
