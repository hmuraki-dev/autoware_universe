# Vissim-CARLA-Autoware: CARLAブループリント割り当て(実メッシュ寸法に合わせる) 実装計画 v1.0

作成日: 2026-11-16

対象ブランチ:

| リポジトリ                                                          | ブランチ                        | 主に変えるもの |
| ------------------------------------------------------------------- | ------------------------------- | -------------- |
| autoware_universe(`~/autoware.1.9.0/src/universe/autoware_universe`) | `feature/vissim_windows_co-sim` | 共通部品 `carla_fleet.py`、`vissim_integration/` のブループリント選択と配置、計測・生成ツール、規則と生成物、テスト、起動手順書・README |
| CARLA(`~/CARLA`)                                                    | `feature/vissim_windows`        | 変えない。`Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx` を生成ツールの入力として読むだけ |

参考:

- 移植ガイド `ブループリント割り当て処理.md`(束 `blueprint_assignment_handoff_20261116.tar.gz`。移植元: KAIT50 の SBIR_sim、`feat/carla-substep-10ms` の `3ea1650`)
- 束の `files/docs/CARLA_blueprint候補拡張_設計案.md`(規則の根拠。本文の「設計案 §n」はこの文書の節)
- 束の `files/reference/bridge_excerpts.md`(移植元ブリッジの組み込み箇所)
- `docs/Vissim_車両寸法一覧.md`、`docs/CARLA_車両寸法一覧.md`(今の割り当てを決めたときの資料。CARLA側はバウンディングボックスの値)
- SUMO構成への展開は別の計画書(`SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md`。本計画のF1〜F3の結果を前提にする)

---

## 0. 目的・スコープ

Vissimの車両1台をCARLAに出すとき、**どのブループリントを使うかを、Vissimの2D/3Dモデルの寸法に最も近いものに決める**。
今は車種番号(100・210・…)ごとの候補から乱数で選んでいるため、Vissimでは全長3.56 mのFiat 500が、CARLAでは
4.9 mのlincoln.mkzになることがある。LiDARに映るのはCARLAの体なので、Autowareの知覚が見る車の大きさとVissimの車の大きさが合わない。

方針は移植元と同じにする。

- **走行中には選ばない。** 生成ツールで「モデル → ブループリントの並び」の表(`carla_variants_<網>.yaml`)を事前に作る。
  ブリッジは起動時にその表を検査し、走行中は引くだけ。乱数は使わない。
- **寸法の基準はCARLAの実メッシュ**(レイキャストで測る)。`actor.bounding_box` は使わない。
- **黙って別のブループリントに替えない。** 表が古い・足りないときは、理由(`code`)を付けて止める。

### 0.1 目標の構成

```
[CARLA 0.9.15] --measure_blueprint_mesh.py--> data/carla_fleet/mesh_measurements_carla0.9.15.json
                                                     │
[Town01.inpx] --(モデルのL・W・カテゴリ・割合)--> build_carla_variants.py <-- data/carla_fleet/carla_variant_rules.yaml
                                                     │
                              vissim_integration/data/carla_variants_town01.yaml(生成物。コミットする)
                                                     │ 起動時に検査
            ブリッジ: (VehicleType, ModelFileName) -> モデル番号 -> actorの並び -> 実メッシュ中心で配置
```

### 0.2 スコープ外

- **作り置き(プール)**。移植元は、CARLA 0.10でspawn直後に車が落ちて見える問題があったためプールにした。
  こちらは今のspawn/destroyのままにする。0.9.15で同じ問題が出たら、別の計画で扱う(6章)。
- **連結車の折れ曲がり**(`follows_path`、`articulated.py`)。今のinpxで出現するモデルは、どれも1セグメント(1章の4)。
  生成物に `follows_path` が出たら、ブリッジは起動時に止める(2.5)。
- 歩行者(`ptypes.json`)。今のままにする。
- CARLA→Vissimの向き(自車)。自車のブループリントは `vehicle_type` パラメータのまま。
- Windows側アダプタ(`PTV-Vissim_windows/`)の変更。`model_filename` はすでに送られている(1章の5)。
- 知覚GT(Vissimの寸法の箱)の配信。今の構成にはないので扱わない。dL・dWはログに出す(2.4)。

---

## 1. 事前調査で分かったこと(2026-11-16)

| #   | 項目 | 内容 |
| --- | ---- | ---- |
| 1   | 今の割り当て | `BridgeHelper.get_carla_blueprint()`(`bridge_helper.py:184`)。`data/vtypes.json` の「車種番号 → 候補リスト」から、Vissimの車両IDをシードにした乱数で選ぶ。キーにない車種、空のリスト、CARLAにないブループリントは、エラーログを出してspawnしない。 |
| 2   | 1台あたりのactor数 | 1台 = 1 actor。`vissim2carla_ids` は `{vissim_id: carla_id}`。使っている所は `simulation_synchronization.py`(spawn・destroy・更新・ウォームアップ後の一括spawn・CARLA→Vissimの除外・close)、`ego_spawn_gate.py:313`(隙間の検査)、`carla_autoware.py:761`(終了時の削除)、テスト `vissim_ego_spawn_gate_test.py`。 |
| 3   | 位置の基準 | `get_carla_transform(transform, extent)` は、Vissimの位置を**前端中央**として、CARLAの箱の `extent.x` だけ後ろへずらして中心にしている(上流CARLAのコードのコメント)。移植元の抜粋には「Vissimの (x,y) は既に車体中心」とあり、食い違う。F0で確かめる。箱の `location.x`(原点からのずれ)は今は見ていない。 |
| 4   | 出現するモデル | CARLAリポジトリ `feature/vissim_windows` の `Town01.inpx`(sha256 `2265ff82…`)を移植元の読み込み処理(`net.py` のモデル部分)で読んだ結果。車両構成は1つ、自車の車種(`drivSimVehType`)は101で除外。割合 > 0 のモデルは14個で、**すべて1セグメント**。下の表。 |
| 5   | DSIで分かること | アダプタの応答の車両の行は `id`・`type`・`model_filename`・`color`・位置・向き・速度。**寸法はない**(歩行者の行にはある)。モデル番号は (`type`, `model_filename`) からinpxの表で引く必要がある(移植元と同じ)。`model_filename` の実際の文字列(パスや拡張子が付くか)は未確認。 |
| 6   | inpxの版の食い違い | `docs/Vissim_車両寸法一覧.md` は、Golfを「Golf VIII (2022)、4.32 m」、バイクをモデル302としている。リポジトリのinpxでは「Golf (2007)、4.211 m」、バイクはモデル70。**Windows PCで読まれるinpxがどちらかを、F0で確かめる**(信号設置のB0と同じ手順)。 |
| 7   | CARLAの版 | Linux側の `carla` は0.9.15。救急車は `vehicle.ford.ambulance`(移植元の `vehicle.ambulance.ford` と語順が違う)。パトカーは `vehicle.dodge.charger_police`・`_police_2020`、消防車は `vehicle.carlamotors.firetruck`、タクシー塗装は `vehicle.ford.crown`。 |
| 8   | 自車 | launchの `vehicle_type` の既定値は `vehicle.toyota.prius`。今は `vtypes.json` の100の候補にもpriusが入っている。 |
| 9   | 試算(箱の寸法で代用) | `CARLA_車両寸法一覧.md` の箱の寸法を実メッシュの代わりに使い、移植元の規則で試算した結果(下の表)。乗用車10モデル・トラック・バイクは1体で割り当たる。**トラクター(22)とバス(31)は割り当たらない**。実メッシュで値は変わるが、大型車の扱いはF3で決める必要がある。 |
| 10  | パッケージ | PyYAMLへの依存がない(`package.xml`)。データファイルは `vissim_integration/data/` に置けば、`vtypes.json` と同じく読める。テストは `test/` でpytest(`conftest.py` がROSなしでも読めるようにしている)。 |

1章の4の表(割合は網全体での値):

| モデル | 名前 | カテゴリ(車種) | 割合 | L [m] | W [m] |
| -----: | ---- | -------------- | ---: | ----: | ----: |
| 1  | Car - Fiat 500                 | CAR(100)  | 0.036 | 3.561  | 1.883 |
| 2  | Car - Tesla Model Y            | CAR(100)  | 0.030 | 4.201  | 1.910 |
| 3  | Car - Volkswagen Golf (2007)   | CAR(100)  | 0.108 | 4.211  | 2.004 |
| 4  | Car - Mercedes B Class         | CAR(100)  | 0.066 | 4.414  | 2.047 |
| 5  | Car - Mercedes C Class         | CAR(100)  | 0.084 | 4.751  | 2.032 |
| 6  | Car - Skoda Superb Combi       | CAR(100)  | 0.030 | 4.781  | 2.022 |
| 7  | Car - Audi A6 Avant            | CAR(100)  | 0.066 | 4.991  | 2.118 |
| 8  | SUV - Nissan Qashqai           | CAR(100)  | 0.042 | 4.328  | 1.992 |
| 9  | SUV - Volkswagen Tiguan        | CAR(100)  | 0.108 | 4.500  | 2.107 |
| 10 | SUV - Volkswagen ID 4          | CAR(100)  | 0.030 | 4.600  | 2.037 |
| 22 | HGV - Semi-Tractor Volvo E     | HGV(220)  | 0.100 | 5.960  | 2.550 |
| 23 | HGV - Delivery DAF LF          | HGV(210)  | 0.100 | 7.930  | 2.550 |
| 31 | Bus - C2 Standard              | BUS(300)  | 0.100 | 12.135 | 2.550 |
| 70 | Bike - Motorbike Yamaha MT 07  | BIKE(700) | 0.100 | 2.098  | 0.907 |

1章の9の試算(箱の寸法。除外はパトカー・タクシー・消防車・prius、fallbackは `vehicle.ford.ambulance`):

| モデル | 試算の割り当て | dL | dW | 割り当たらない理由 |
| -----: | -------------- | -: | -: | ------------------ |
| 1  | nissan.micra | +0.07 | −0.03 | |
| 2・3 | seat.leon | −0.01・−0.02 | −0.09・−0.18 | |
| 4・8 | volkswagen.t2_2021 | +0.03・+0.11 | −0.28・−0.22 | |
| 5  | ford.mustang | −0.03 | −0.14 | |
| 6  | tesla.model3 | +0.01 | +0.14 | |
| 7  | dodge.charger_2020 | +0.02 | −0.24 | |
| 9  | volkswagen.t2 | −0.02 | −0.04 | |
| 10 | nissan.patrol | 0.00 | −0.11 | |
| 23 | carlamotors.european_hgv | +0.01 | +0.34 | |
| 70 | kawasaki.ninja | −0.06 | −0.11 | |
| 22 | なし | | | carlacola は dL −0.76(許容0.75の外)、cybertruck・ambulance は細い(HGVの細い側の許容0.10の外)。救急車より短いので重ね置きもできない |
| 31 | なし | | | fusorosa は dL −1.87。救急車の重ね置きは幅が −0.20 で、BUSの許容0.10の外 |

---

## 2. 方針

### 2.1 ファイルの置き場所

ブランチが2つ(Vissim・SUMO)あり、どちらもまだmainに入っていない。共通部品は**両方のブランチで同じパスに置き、中身も同じにする**。
あとでマージしたときに、食い違いや衝突が起きないようにするため。

| ファイル | 置き場所(パッケージ `simulator/autoware_carla_interface/` から) | 由来 |
| -------- | ---------------------------------------------------------------- | ---- |
| 共通部品 | `src/autoware_carla_interface/carla_fleet.py` | 束の `runtime/bridge/carla_fleet.py`(パスの扱いだけ変える。2.2) |
| 規則 | `src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml` | 束の `config/carla_variant_rules.yaml`(F3で0.9.15向けに直す) |
| 実メッシュ | `src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json` | F2で測る |
| inpxのモデル読み込み | `src/autoware_carla_interface/vissim_integration/inpx_models.py` | 束の `reference/vissim/net.py` のモデル部分だけ(`model_key`・`model_body`・`_load_vehicle_models`・`model_shares`) |
| 生成物 | `src/autoware_carla_interface/vissim_integration/data/carla_variants_town01.yaml` | F4で作る |
| 計測ツール | `tools/carla_fleet/measure_blueprint_mesh.py` | 束(自車の除外を引数にする。F2で計測の基準をLiDARに変えた) |
| 生成ツール | `tools/carla_fleet/build_carla_variants.py` | 束(`--inpx` でファイルを直接受ける。登録簿を外す) |
| テスト | `test/test_carla_fleet.py`、`test/test_build_carla_variants.py`、`test/vissim_carla_variants_test.py` | 束の `tests/core/test_carla_variants.py` の規則の部分 + 新規 |
| テスト用の実メッシュ | `test/fixtures/carla_fleet/mesh_measurements_carla0.10_20260921.json` | 束(0.10の値。規則の回帰テスト用) |

### 2.2 パスの扱い(移植元から変えるところ)

移植元はリポジトリ直下(`REPO`)からの相対パスで `sources.*.path` を書き、起動時にsha256を照合する。
こちらはパッケージをインストールして動かすので、`REPO` が実行時にない。そこで次のようにする。

- `sources.geometry`・`sources.rules` のパスは、**生成物(carla_variants)のあるディレクトリからの相対パス**で書く。
  インストール先でも、`src/` でも同じに解決できる。
- `sources.inpx` には、生成に使ったinpxの**記録だけ**を残す(パス・sha256)。ブリッジは照合しない(inpxはWindows側にある。2.3)。

### 2.3 モデル番号の引き方(ブリッジは実行時にinpxを読まない)

移植元のブリッジは、起動時にinpxを読んでモデルの表を作る。こちらのinpxはWindows PCにあり、Linux側で同じファイルを読める保証がない。
そこで、**生成ツールがモデルの表を生成物に書き込み、ブリッジはそれを使う**。

- 生成物の `models` に、モデルごとの `file`(`model_key()` で拡張子とパスを除いた先頭セグメントのファイル名)、
  `types`(そのモデルを使う車種番号)、`L`・`W`・`segments`・`share` を書く(移植元の生成物にもある)。
- ブリッジは `(str(type), model_key(model_filename))` → モデル番号の表を、起動時に `models` から作る。
- 走行中に表にない組が来たら、`npc_model_unknown` で止める(移植元と同じ)。
  inpxを書き換えたのに生成物を作り直していない場合は、ここで止まる。
- 検査3(モデルのL・Wの照合)は、inpxを読まないので行わない。代わりに、F0とF7で「Windowsで読まれるinpx = 生成に使ったinpx」をsha256で確かめる手順を、起動手順書に書く。

### 2.4 配置(実メッシュの中心で合わせる)

体ごとの置き方は、移植元の `bodies_for` / `placements` と同じ考え方にする。

1. 前端の位置: Vissimの位置(前端中央かどうかはF0で確かめる)。CARLA座標への変換(y反転)は今の `get_carla_transform` と同じ。
2. 体の中心: 前端から `front_offset_m + mesh_length/2` だけ後ろ。
3. 原点の位置: 体の中心から、`(mesh_center_x, mesh_center_y)` を向きで回して引く。
4. 高さ: 今のまま(Vissimのz。spawnのときだけ `CARLA_SPAWN_OFFSET_Z` を足す)。

今の `get_carla_transform(transform, carla_actor.bounding_box.extent)` は使わなくなる。
dL・dW(CARLAの外形 − Vissimのモデル)は、spawnのときにdebugログで出す。

### 2.5 起動時に止める検査

`carla_fleet.VariantError` を、ブリッジの例外(既存の止め方に合わせる。F5で決める)に詰め替え、`code` と根拠をログに出して止める。

| code | 条件 | 移植元との違い |
| ---- | ---- | -------------- |
| `variant_config_missing` | 生成物がない | 同じ |
| `variant_blueprint_missing` | 割り当てたブループリントがCARLAにない | 同じ |
| `variant_source_changed` | 実メッシュ・規則のsha256が生成時と違う | inpxのL・Wは照合しない(2.3) |
| `variant_model_empty` | 割合 > 0 のモデルに割り当てがない(生成時)。走行中に割り当てのないモデルが来た | 同じ |
| `variant_mesh_carla_version_changed` | 実メッシュを測ったCARLAの版と、接続先の版が違う | 同じ |
| `variant_articulated_unsupported` | 生成物に `follows_path` がある | 新規(0.2) |
| `npc_model_unknown` | 走行中に、表にない (`type`, `model_filename`) が来た | 同じ |

### 2.6 重ね置き(1台に複数のactor)

F3の結果で、重ね置きを使うモデル(1章の9ではバスとトラクターが候補)が残るときだけ、F6で入れる。
`vissim2carla_ids` は `{vissim_id: carla_id}` のまま残し、2体目以降を `vissim2carla_extra_ids = {vissim_id: [carla_id, ...]}` に分けて持つ。
1章の2の使っている所(CARLA→Vissimの除外、close、終了時の削除、`ego_spawn_gate` の隙間の検査)は、1体目だけを見ても今の動作が変わらないようにし、
2体目以降は spawn・destroy・更新・除外・削除の5か所だけで扱う。隙間の検査は、1体目の箱で見ている今の動作のままにする(6章)。

### 2.7 今の `vtypes.json` の扱い

launchに `vissim_carla_variants`(生成物のパス)を足す。既定は `vissim_integration/data/carla_variants_town01.yaml`。
**`legacy_vtypes` を明示したときだけ**、今の `vtypes.json` と乱数の選び方に戻る(比べるためと、戻すため)。
最初は空文字にする案だったが、F5でキーワードに変えた(ROS 2のlaunchでは空文字のパラメータの扱いが不安定なため)。
既定のファイルが見つからないときは、`vtypes.json` に戻さず、`variant_config_missing` で止める。

---

## 3. タスク一覧

| #   | タスク | 場所 | 状態 |
| --- | ------ | ---- | ---- |
| F0  | 事前確認(使うinpx、`model_filename` の文字列、位置の基準、0.9.15での計測) | Windows・Linux・CARLA | 完了(2026-11-16) |
| F1  | 共通部品の取り込み(`carla_fleet.py`、規則の雛形、回帰テスト、PyYAMLの依存) | autoware_universe | 完了(2026-11-16) |
| F2  | 計測ツールの移植と、全ブループリントの実メッシュ計測 | autoware_universe・CARLA | 完了(2026-11-16。計測の基準をLiDARに変えた) |
| F3  | 規則の確定(除外、fallback、大型車、二輪) | autoware_universe | 完了(2026-11-16) |
| F4  | inpxの読み込み・生成ツールと、Town01の生成物 | autoware_universe | 完了(2026-11-16) |
| F5  | ブリッジへの組み込み(起動時の検査、モデルの引き、実メッシュ中心での配置) | autoware_universe | 完了(2026-11-16) |
| F6  | 重ね置き(F3で必要になったときだけ) | autoware_universe | 完了(2026-11-16) |
| F7  | ブリッジ単体での実機確認(Autowareなし) | Linux + Windows | 完了(2026-11-16) |
| F8  | Autoware全体での実機確認 | Linux + Windows | 未着手 |
| F9  | ドキュメントとコミット、SUMO計画への引き継ぎ | autoware_universe | 未着手 |

依存関係: F0 → F1 → F2 → F3 → F4 → F5 →(F6)→ F7 → F8 → F9。
F1はF0と並行してよい。F3で規則を変えたら、F4の生成物を作り直す(起動時の検査がsha256の違いで止めるので、作り直し忘れは分かる)。

---

## 4. 各タスクの内容

### F0: 事前確認

1. **使うinpxを決める**。Windows PCの `server.py` が読むinpx(`--vissim-network`)と、CARLAリポジトリ `feature/vissim_windows` の
   `examples/Town01/Town01.inpx` のsha256を比べる(WindowsはPowerShellの `Get-FileHash`、Linuxは `sha256sum`)。
   - 違うときは、モデル(`model2D3D`)・2D/3Dモデル分布・車種の差を見て、正にする方を決める。正にした方をCARLAリポジトリに入れる。
   - 1章の6の食い違い(Golf・バイク)も、ここで解消する。`Vissim_車両寸法一覧.md` の直しはF9。
2. **`model_filename` の文字列**。ブリッジ単体を短く動かし、`vissim_simulation.py` で受け取った車両の `type` と `model_filename` を
   車種ごとに1つずつログに出す(一時的なdebugログ。コミットしない)。
   - inpxの `file3D` に対して `model_key()` で一致するか(パス・拡張子・大文字小文字)を確かめる。
3. **位置の基準**。同じ実行で、直線を走るVissimの車1台について、Vissimの位置とCARLAの箱の前端・中心を比べる。
   - 確かめる方法: CARLAのスペクテーターで真上から見る。または、Vissim GUIの車両リスト(前端の座標)と、ログの位置を比べる。
   - 前端中央なら、2.4のとおり。中心なら、2.4の1を「中心 + L/2」に変える。
4. **0.9.15での計測**。束の `measure_blueprint_mesh.py` を、そのまま1体だけ動かす(`--only vehicle.lincoln.mkz`)。
   - `world.cast_ray` があるか、`str(hit.label)` が `Car` などの名前を返すか、地下(x=−4300、z=−60)が空いているかを見る。
   - `world.wait_for_tick()` は非同期モードでしか返らない。CARLAを非同期モードで起動するか、ツールで1回tickするかを決める。

完了条件: 使うinpxがCARLAリポジトリに入っている。`model_filename` と `model_key()` の対応が分かっている。位置の基準が分かっている。1体の計測値が出ている。

#### 結果(F0、2026-11-16)

| #   | 項目 | 結果 |
| --- | ---- | ---- |
| 1   | 使うinpx | **Windows PCの版とリポジトリの版は同じ**。Windows PC(`C:\Users\divp\SBIR\Vissim\work_muraki\CARLA\Town01.inpx`)のsha256は `2265FF82…B50F14` で、CARLAリポジトリ `feature/vissim_windows` の `4213552`(信号設置B4)の版と一致した。Golfは `Car - Volkswagen Golf (2007).fbx`(4.211 m)、バイクはモデル70。`c5df1a6` より前の版にも「Golf VIII (2022)」「モデル302」はない。`Vissim_車両寸法一覧.md` は別のinpxから作られたと考えられるので、F9でこの版の値に直す。1章の4の表は、この版で読んだもの。 |
| 2   | `model_filename` の文字列 | アダプタにつないで空のtickを300秒分送った(scratchpadのプローブ。`vissim_warmup_probe.py` の `AdapterClient` を使用。コミットしない)。**Linux側には `bytes` で届き**、中身は**パスと拡張子のないファイル名**(例: `b'Car - Volkswagen Golf (2007)'`、`b'Bus - C2 Standard 2-doors'`)。inpxの `file3D` に `model_key()` をかけた値と、出てきた13組すべてが一致した。出なかったのはモデル2(Tesla Model Y、割合0.030)だけ。F5では、UTF-8でデコードしてから `model_key()` をかける(`.fbx`・`.u3dm` が付いても同じ結果になるように)。 |
| 3   | 位置の基準 | **前端中央**。アダプタの `VISSIM_Veh_Data`(`DrivingSimulatorProxy.h` と同じ配置)のコメントが `Position_X  # front center of the vehicle in m`。今のブリッジの扱い(`get_carla_transform` で前端から `extent.x` だけ戻す)と同じなので、2.4のとおりに進める。見た目での確認はF7。 |
| 4   | 0.9.15での計測 | **測れた**。束の `measure_blueprint_mesh.py` で `vehicle.lincoln.mkz_2017` を1体だけ測った(0.9.15に `vehicle.lincoln.mkz` はない)。`world.cast_ray` があり、`str(hit.label)` は `Car` を返した。地下(x=−4300、z=−60)は空いていた。結果はL 4.863 m、W 2.101 m、高さ1.465 m、メッシュ中心のずれ0.000 m(箱はL 4.902 m、W 2.128 m)。ただし、**起動中のCARLAは同期モード(dt 0.05)で、tickするクライアントがいないと `world.wait_for_tick()` が返らない**。試しでは、同期モードなら `world.tick()` を呼ぶように差し替えて動かした。F2の移植でこの処理を入れる。計測したactorは消え、ワールドの設定は変わっていない。 |

出てきた (車種, `model_filename`) と台数(300秒):

| 車種 | `model_filename` | 台数 | モデル |
| ---: | ---------------- | ---: | -----: |
| 100 | `Car - Volkswagen Golf (2007)` | 8 | 3 |
| 100 | `SUV - Volkswagen Tiguan eHybrid (2022)` | 6 | 9 |
| 100 | `Car - Audi A6 Avant (2021)` | 6 | 7 |
| 100 | `Car - Mercedes B Class (2021)` | 4 | 4 |
| 100 | `Car - Mercedes C Class (2021)` | 4 | 5 |
| 100 | `SUV - Volkswagen ID 4 (2022)` | 3 | 10 |
| 100 | `Car - Skoda Superb Combi (2022)` | 3 | 6 |
| 100 | `SUV - Nissan Qashqai (2022)` | 2 | 8 |
| 100 | `Car - Fiat 500 (2018)` | 1 | 1 |
| 210 | `HGV - Delivery DAF LF (2014)` | 6 | 23 |
| 220 | `HGV - Semi-Tractor Volvo E (2019)` | 5 | 22 |
| 300 | `Bus - C2 Standard 2-doors` | 3 | 31 |
| 700 | `Bike - Motorbike Yamaha MT 07 (2022)` | 7 | 70 |

### F1: 共通部品の取り込み

- `carla_fleet.py` を2.1の場所に入れる。変えるのは2.2のパスの扱いだけ(`repo_path`・`display_path` を、生成物のディレクトリ基準に置き換える)。
  あわせて `check_sources()` を、渡したディレクトリ基準で解決するようにする。
- `variant_articulated_unsupported` の検査(2.5)を `check_no_follows_path()` として足す。
- 規則の雛形を2.1の場所に入れる(値はF3で確定する)。
- `package.xml` に `<exec_depend>python3-yaml</exec_depend>` を足す。
- 回帰テスト `test/test_carla_fleet.py`: 束の `test_carla_variants.py` から、規則・作り置き数以外の起動時検査のテストを持ってくる。
  入力は0.10の実メッシュ(`test/fixtures/...0.10...json`)のまま。「移植元と同じに動くか」を確かめるためのもの。
  移植元固有のテスト(出荷済みの生成物との一致、`/data/vissim/...`、`received_networks`、`measured_fleet_*.yaml`)は持ってこない。
- インストール後にyamlが読めることを、`colcon build` のあとで確かめる(`vtypes.json` と同じ扱いになるか)。

完了条件: `python3 -m pytest test/test_carla_fleet.py` が通る。`colcon build --packages-select autoware_carla_interface` が通る。

#### 結果(F1、2026-11-16)

- 足したファイル:
  - `src/autoware_carla_interface/carla_fleet.py`: 移植元からの変更は次の2つだけ。
    - パスの扱い: `REPO`・`repo_path`・`display_path` を、`resolve_source(path, base_dir)`・`relative_source(path, base_dir)`・`variants_dir(variants)` に置き換えた。`check_sources()`・`mesh_for()` は、carla_variants のあるディレクトリ(symlink は辿る)を基準にする。
    - `check_no_follows_path()`(`variant_articulated_unsupported`)を足した。
  - `src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml`: **暫定の雛形**。値は移植元のまま、ブループリント名だけを0.9.15に置き換えた(fallback `vehicle.ford.ambulance`、除外は消防車・パトカー2種・`vehicle.ford.crown`・`vehicle.toyota.prius`)。F3で確定する。
  - `test/test_carla_fleet.py`(45件): 移植元の規則・起動時検査の回帰テストと、新しく足した部分のテスト。
    - 入力は、移植元の0.10の実メッシュと規則を凍結した写し(`test/fixtures/carla_fleet/mesh_measurements_carla0.10_20260921.json`・`carla_variant_rules_carla0.10.yaml`)。本番の規則をF3で直しても、回帰テストは変わらない。
    - 大型車のテストは、移植元の `measured_fleet_odaiba_v016.yaml` との一致の代わりに、重ね置きの形(体の数、等間隔の前端、dL = 0、`follows_path`)を確かめる形にした。
    - 生成ツールに関わるテスト(`build()`・`expected_bodies()`)はF4へ回した。`pool_sizes()` の試算のテストは、割り当てを直接作る形にして残した。
    - 新しく足したテスト: 相対パスを carla_variants のディレクトリから解決すること(インストール先へ写しても読めること)、生成元のファイルがないとき、`check_no_follows_path()`、CARLAの版の照合、パッケージの規則が読めること、など。
  - `package.xml`: `<exec_depend>python3-yaml</exec_depend>` を足した。
- 確かめたこと:
  - `python3 -m pytest test/test_carla_fleet.py`: 45件成功。`python3 -m pytest test`(パッケージの全テスト): 136件成功、1件スキップ(前からスキップのもの)。
  - `colcon build --packages-select autoware_carla_interface --symlink-install` が通った。インストール先に `carla_fleet.py` と `data/carla_fleet/carla_variant_rules.yaml` がリンクとして入り、インストールしたパッケージから規則を読めた。
- 書式: リポジトリの pre-commit は black・isort(100桁)だが、この環境には入っておらず、既存のファイルも black の形ではない。既存のファイルの書き方に合わせた。

### F2: 計測ツールと実メッシュ計測

- 束の `measure_blueprint_mesh.py` を `tools/carla_fleet/` に入れる。変えるところ:
  - `'j6' in blueprint.id` を、`--exclude`(fnmatch、複数可)にする。既定は空。自車を外すかは規則で決める(F3)。
  - F0の4で決めた、tickの扱い。
  - 出力ファイル名を `--out <ディレクトリ>/mesh_measurements.json` から、`--out <ファイル>` に変える(置き場所を2.1にするため)。
- 全41ブループリントを測る。CARLA(Town01)を起動し、Autowareとブリッジは起動しない。
- 結果を `data/carla_fleet/mesh_measurements_carla0.9.15.json` に置く。測れなかったブループリントは、理由をこの計画書の結果に書く。
- `CARLA_車両寸法一覧.md` の箱の寸法と並べ、差の大きいもの(移植元では消防車・fusoで1 m前後)を結果に書く。

完了条件: 実メッシュのファイルに `carla_server_version` が記録されている。二輪を含む全ブループリントの値、または測れない理由がある。

#### 結果(F2、2026-11-16)

**計測の基準を、`cast_ray` からセマンティックLiDARに変えた**(移植元からの変更)。

- 移植元の方法(`world.cast_ray`)のまま測ると、次のことが分かった。
  - 0.9.15の `cast_ray` は、1本のレイが通った面をすべて返す。移植元のツールは「最初の面だけを返す」(0.10の動作)前提で書かれている。
  - 二輪は、どのレイも最初にラベル `NONE` の面(LiDARには映らない当たり判定の殻)に当たる。
- そこで、車の周りと上にセマンティックLiDAR(`sensor.lidar.ray_cast_semantic`)を置き、その車の点だけから寸法を出して比べた。Autowareが使うLiDARと同じレイの仕組み。
  - 乗用車の多くは、`cast_ray` とLiDARの差が数cm以内だった。
  - 次のものは、LiDARを密にしても食い違いが残った。向きもばらばら。

    | ブループリント | LiDAR | cast_ray | 箱 |
    | -------------- | ----: | -------: | -: |
    | `mitsubishi.fusorosa` の幅 | 3.176 | 3.893 | 3.944 |
    | `volkswagen.t2` の幅 | 2.058 | 2.204 | 2.069 |
    | `mercedes.coupe_2020` の幅 | 1.999 | 1.797 | 1.812 |
    | `vespa.zx125` の長さ | 1.810 | 2.064 | 1.817 |
    | 二輪4種(`diamondback`・`gazelle`・`kawasaki`・`harley`)の幅 | 0.69〜0.87 | 0.49〜0.76 | 0.58〜0.80 |

    fusorosaのLiDARの幅3.176 mは、移植元が0.10で測ったメッシュ(3.181 m)と合う。`cast_ray` は当たり判定の箱を拾っていると考えられる。二輪はライダーの腕などをLiDARだけが拾っている。
- 割り当ての目的は「LiDARに映る大きさ」に合わせることなので、`mesh_*_m` をLiDARの値にした。`cast_ray` の値は `castray_*_m` として比べるために残した(`cast_ray` は、各レイで `NONE` を飛ばした最も近い車両ラベルの面だけを使う形に直した)。
- LiDARの配置: 車の上14 mに1台、半径8 mの円周に16方位 × 高さ4段(0.5・1.2・2.0・3.0 m)。128 ch、毎秒400万点、1回転以上回す。8方位 × 2段から密にしても、乗用車の寸法の変化は数cm以内だった。

**ツールと計測結果**

- `tools/carla_fleet/measure_blueprint_mesh.py`: 上の変更のほか、`--exclude`(fnmatch)、同期モードでのtick、`--out` をファイルにした。
- `src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json`: 41ブループリントすべて(除外なし)を、Town01・CARLA 0.9.15で測った。かかった時間は73秒。計測後、ワールドに車両・センサーは残っていない。
- 気づいたこと:
  - `carlamotors.european_hgv` と `carlamotors.firetruck` は、LiDARでも点の数(1,787,820)まで同じ形に見える。0.9.15では同じ形を使っていると考えられる。箱の寸法は違う(7.936 m と 8.468 m)。
  - 箱との差が大きいもの: `european_hgv`(L +0.49、W +0.45)、`ford.ambulance`(W +0.27、中心が後ろへ0.32 m)、`mercedes.sprinter`(W +0.34)、`nissan.patrol_2021`(W +0.27)、`fusorosa`(L −0.49、W −0.77、中心が後ろへ0.51 m)、`jeep.wrangler_rubicon`(L −0.28)。
  - 中心のずれ(`mesh_center_x_m`)が0.1 mを超えるもの: `fusorosa` −0.514、`ford.ambulance` −0.322、`european_hgv`・`firetruck` −0.268、`ford.crown` +0.152、`jeep.wrangler_rubicon` +0.117。

**試算(暫定の規則のまま)**

暫定の規則(F1の雛形)と、この実メッシュで、1章の4の14モデルを割り当てた。**14モデルすべてに割り当てが作れた**。

| モデル | 割り当て | dL | dW |
| -----: | -------- | -: | -: |
| 1 Fiat 500 | `nissan.micra` | +0.01 | −0.07 |
| 2 Tesla Model Y | `audi.tt` | −0.05 | +0.06 |
| 3 Golf (2007) | `audi.tt` | −0.06 | −0.03 |
| 4 Mercedes B Class | `volkswagen.t2_2021` | −0.05 | −0.31 |
| 5 Mercedes C Class | `tesla.model3` | −0.00 | +0.06 |
| 6 Skoda Superb Combi | `tesla.model3` | −0.03 | +0.07 |
| 7 Audi A6 Avant | `dodge.charger_2020` | +0.01 | −0.22 |
| 8 Nissan Qashqai | `volkswagen.t2_2021` | +0.04 | −0.25 |
| 9 VW Tiguan | `mini.cooper_s_2021` | +0.00 | +0.05 |
| 10 VW ID 4 | `nissan.patrol` | −0.04 | −0.12 |
| 22 Semi-Tractor Volvo E | `ford.ambulance` | +0.35 | +0.07 |
| 23 Delivery DAF LF | `ford.ambulance` × 2(重ね置き) | 0.00 | +0.07 |
| 31 Bus C2 Standard | `ford.ambulance` × 2(重ね置き) | 0.00 | +0.07 |
| 70 Yamaha MT 07 | `kawasaki.ninja` | −0.08 | −0.03 |

- 箱の寸法での試算(1章の9)と違い、トラクターとバスにも割り当てが作れた。救急車の実際の幅(2.623 m)が箱(2.351 m)より広く、HGV・BUSの細い側の許容に入ったため。
- トラックとバスは救急車2体の重ね置きになる。**F6(重ね置き)が必要**。
- `european_hgv` は幅が +0.79 m で、太い側の許容(0.35 m)の外。
- 見た目の問題(トラック・バスが救急車になる、乗用車にバンの `t2_2021` が入る)は、F3で決める。

### F3: 規則の確定

F2の値で1章の9の試算をやり直し、`carla_variant_rules.yaml` を次のように決める。

| 項目 | 移植元 | こちらの案 |
| ---- | ------ | ---------- |
| `excluded` | 消防車、パトカー、タクシー2種、`*j6*` | `vehicle.carlamotors.firetruck`、`vehicle.dodge.charger_police*`、`vehicle.ford.crown`、自車(`vehicle.toyota.prius`)。塗装が特殊なものは、F2のときに撮影して決める |
| `fallback` | `vehicle.ambulance.ford` | `vehicle.ford.ambulance` か `vehicle.carlamotors.european_hgv`(5章の2) |
| 大型車(22・31) | 救急車の重ね置き | 5章の2で決める |
| 二輪 | 0.10にないので割合0 | 0.9.15にはある。BIKEの許容(細い側0.35 m)のまま、ライダーを含めた寸法で割り当てる(5章の3) |
| 許容 | L ±0.75、W +0.35 / −0.35(CAR・BIKE)・−0.10(HGV・BUS・TRAM) | 同じで始める。変えるときは理由を規則のコメントに書く |

- 乗用車に、バン(`volkswagen.t2`・`t2_2021`)やSUVが割り当たる。規則は分類で絞らないので、見た目が気になるときは `excluded` で外す(5章の4)。
- 決めた理由は、規則のファイルのコメントとこの計画書の結果に書く。

完了条件: 1章の4の14モデルすべてに割り当てがある。または、割り当てないモデルの扱い(inpxで割合を0にするなど)が決まっている。

#### 結果(F3、2026-11-16)

**撮影**: 候補のブループリントをTown01の直線道路(spawn point 7)に1台ずつ置き、CARLAのカメラで撮った(scratchpadのスクリプト。コミットしない)。重ね置きの見え方も撮った。撮影のあと、ワールドに車両・センサーは残っていない。

- 乗用車の候補: `docs/img/carla_blueprint_assignment/cars.webp`
- 大型車・二輪の単体: `heavy.webp`
- 重ね置き: `stacks.webp`
- 除外の候補(パトカー2種、タクシー、消防車、自車): `excluded.webp`

**利用者と決めたこと**

| 項目 | 決定 | 理由 |
| ---- | ---- | ---- |
| 大型車の体 | **`carlacola` を fallback にし、救急車は除外する** | 救急車はパトカー・タクシーと同じく特殊な塗装。`carlacola` は見た目がトラックで、幅2.560 mが大型車(2.55 m)に合う(dW +0.01) |
| 乗用車のバン | **`volkswagen.t2`・`t2_2021` を除外する** | 高さが約2.0 mで、乗用車のモデル(1.4〜1.7 m)と合わない。規則は高さを見ないので、除外で避ける |

ほかに、`carlamotors.european_hgv` を除外に加えた。見た目はトラクターだが、LiDARには消防車と同じ形(8.43 × 3.34 m)で映る。今の割り当てには出てこないが、カメラとLiDARで形が違うブループリントは使わない。

**確定した規則**(`data/carla_fleet/carla_variant_rules.yaml`、`name: carla_variant_rules_carla0.9.15_v1`)

- 許容は移植元と同じ: L ±0.75 m、W +0.35 m / −0.35 m(CAR・BIKE)・−0.10 m(HGV・BUS・TRAM)。
- fallback: `vehicle.carlamotors.carlacola`(重ね置き)。
- excluded: `carlamotors.firetruck`、`carlamotors.european_hgv`、`ford.ambulance`、`dodge.charger_police*`、`ford.crown`、`volkswagen.t2*`、`toyota.prius`(自車)。
- 二輪は出す。BIKEの許容はそのまま。LiDARで測った寸法はライダーを含む。

**Town01での割り当て**

| モデル | 割り当て | 前端からの位置 [m] | dL | dW |
| -----: | -------- | ------------------ | -: | -: |
| 1 Fiat 500 | `nissan.micra` | 0 | +0.013 | −0.073 |
| 2 Tesla Model Y | `audi.tt` | 0 | −0.049 | +0.059 |
| 3 Golf (2007) | `audi.tt` | 0 | −0.059 | −0.035 |
| 4 Mercedes B Class | `mini.cooper_s_2021` | 0 | +0.089 | +0.115 |
| 5 Mercedes C Class | `tesla.model3` | 0 | −0.002 | +0.065 |
| 6 Skoda Superb Combi | `tesla.model3` | 0 | −0.032 | +0.075 |
| 7 Audi A6 Avant | `dodge.charger_2020` | 0 | +0.012 | −0.217 |
| 8 Nissan Qashqai | `mini.cooper_s_2021` | 0 | +0.175 | +0.170 |
| 9 VW Tiguan | `mini.cooper_s_2021` | 0 | +0.003 | +0.055 |
| 10 VW ID 4 | `nissan.patrol` | 0 | −0.038 | −0.123 |
| 22 Semi-Tractor Volvo E | `carlacola` × 2 | 0、0.851 | 0.000 | +0.010 |
| 23 Delivery DAF LF | `carlacola` × 2 | 0、2.821 | 0.000 | +0.010 |
| 31 Bus C2 Standard | `carlacola` × 3 | 0、3.513、7.026 | 0.000 | +0.010 |
| 70 Yamaha MT 07 | `kawasaki.ninja` | 0 | −0.082 | −0.033 |

- 100台あたりのブループリントの体数(割合 × 体数): `carlacola` 70.0、`mini.cooper_s_2021` 21.6、`audi.tt` 13.8、`tesla.model3` 11.4、`kawasaki.ninja` 10.0、`dodge.charger_2020` 6.6、`nissan.micra` 3.6、`nissan.patrol` 3.0。
- トラクター・トラック・バスは重ね置きになる。**F6は必要**。
- テスト: `test/test_carla_fleet.py` に、この表の割り当てを確かめるテスト(14件)を足した。規則か実メッシュを変えて割り当てが変わったら、このテストが落ちる。全体で150件成功、1件スキップ。

**F6に向けて分かったこと**

- 重ねる体どうしは重なっているので、その場所に `spawn_actor` すると「Spawn failed because of collision」で失敗する。撮影では、地下(z = −60)の別々の場所にspawnし、物理を切ってから `set_transform` で並べた。ブリッジの重ね置きでも同じ手順が要る。
- 撮影の途中で、spawnに失敗した回の救急車が1台、道路に残っていた。カメラとLiDARの両方に映っていた。原因は、後片付けの確かめ方の誤り。**同期モードで新しく接続したクライアントは、tickを1回受け取るまで `get_actors()` が空を返す**ので、「消えた」と誤って判断していた。`get_actor(id)` で見つけて消し、tickのあとで一覧が空であることを確かめた。地下で行ったF2の計測には影響しない(計測はactorのIDで点を選んでいる)。

### F4: inpxの読み込み・生成ツールと生成物

- `vissim_integration/inpx_models.py`: 束の `net.py` から、モデルの部分だけを抜き出す(2.1)。車線・リンクなどの処理は持ってこない。
  CARLAに依存しない純粋なモジュールにする(生成ツールとテストから使う)。
- `tools/carla_fleet/build_carla_variants.py`: 束から持ってきて、次を変える。
  - 入力は `--inpx <ファイル>` だけ(SUMOの `--rou` はSUMO計画で足す)。`received_networks`・`check_out_name`・`sumo_source` は外す。
  - 生成物に、2.3の `models`(`file`・`types` を含む)を必ず書く。
  - `sources` のパスは、2.2のとおり生成物のディレクトリからの相対パス。
- `test/test_build_carla_variants.py`: 小さい合成inpx(モデル3つ・車種2つ・割合0のモデル1つ)で、モデルの読み込み、割合、skipped、
  生成物の中身(並び・`models`・`sources`)を確かめる。束の規則テストのうち、生成に関わるものもここに持ってくる。
- Town01の生成物 `vissim_integration/data/carla_variants_town01.yaml` を作る。`expected_bodies_per_100_vehicles` とdL・dWを、この計画書の結果に表で書く。

完了条件: テストが通る。生成物ができ、14モデル(またはF3で決めたモデル)に割り当てがある。

#### 結果(F4、2026-11-16)

- `vissim_integration/inpx_models.py`: 移植元 `net.py` の `model_key`・`model_body`・`_load_vehicle_models`・`model_shares` だけを、`InpxModels` クラスとして抜き出した。
  - `model_key()` は `bytes` も受ける(DSIの `model_filename` はbytesで届く。F0の2)。
  - 自車の車種(`drivSimVehType`)と歩行者の車種を外す判定を、`is_traffic_vehicle_type()` にまとめた。
- `tools/carla_fleet/build_carla_variants.py`: 移植元から次を変えた。
  - 入力は `--inpx` のファイルだけにした。SUMOの入力と、網の登録簿(`received_networks`・`check_out_name`)への依存は外した。
  - `--inpx-ref` を足した。渡すと、inpxのパスの代わりに出どころ(リポジトリ・コミット・パス)を `sources.inpx.ref` に記録する。gitから一時ファイルに取り出したinpxを使うときのため。
  - 割り当て(`assignment`)と割り当てなし(`skipped`)の各モデルに、`file`(DSIの `ModelFileName` と同じ鍵)・`types`(車種と車種内の割合)・`segments` を書く。ブリッジはこれでモデルを引く(2.3)。`skipped` にも書くのは、走行中に割合0のモデルが来たとき、「知らないモデル」(`npc_model_unknown`)ではなく「割り当てのないモデル」(`variant_model_empty`)として止めるため。
  - `sources.geometry`・`sources.rules` のパスは、生成物のディレクトリからの相対パス(2.2)。
- `test/test_build_carla_variants.py`(19件): 合成の小さなinpxで、次を確かめる。
  - 読み込み: `model_key` の形、連結車の全長、割合、自車・歩行者の除外、需要が使わない車両構成の除外。
  - 生成: 割り当て、`skipped`、止まる条件、体数。
  - 書き出しと起動時検査: 書き出した生成物が起動時の検査を通ること、連結車で `check_no_follows_path()` が止まること。
  - コミットしたTown01の生成物の検査。
  - inpxから作り直して一致するかのテストは、環境変数 `CARLA_TOWN01_INPX` があるときだけ動く。
- **Town01の生成物** `vissim_integration/data/carla_variants_town01.yaml`:
  - 入力: CARLAリポジトリ `4213552` の `Town01.inpx`(`git show` で取り出した。sha256 `2265ff82…` で、Windowsで読まれるものと同じ)。
  - 割り当ては14モデルで、F3の表と同じ。dLの小数3桁目の違い(例: モデル4はF3で+0.089、生成物で+0.088)は、F3ではinpxの寸法を丸めた値で試算していたため。
  - `skipped` は2モデル: 61 Bike - Cycle Man(車種610)、62 Bike - Cycle Woman(車種620)。どちらも割合0。
  - 100台あたりの体数: `carlacola` 70.0、`mini.cooper_s_2021` 21.6、`audi.tt` 13.8、`tesla.model3` 11.4、`kawasaki.ninja` 10.0、`dodge.charger_2020` 6.6、`nissan.micra` 3.6、`nissan.patrol` 3.0。
- 確かめたこと:
  - `python3 -m pytest test`: 168件成功、2件スキップ(前からの1件と、上の作り直しのテスト)。
  - `CARLA_TOWN01_INPX=<取り出したinpx>` を付けると、作り直しのテストも成功する。
  - `colcon build --packages-select autoware_carla_interface --symlink-install` が通った。インストール先の生成物を読み、`check_sources()` を通った。

作り直すときの手順:

```bash
git -C ~/CARLA show <コミット>:Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx > /tmp/Town01.inpx
python3 tools/carla_fleet/build_carla_variants.py \
    --inpx /tmp/Town01.inpx \
    --inpx-ref "~/CARLA feature/vissim_windows <コミット> Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx" \
    --geometry src/autoware_carla_interface/data/carla_fleet/mesh_measurements_carla0.9.15.json \
    --rules src/autoware_carla_interface/data/carla_fleet/carla_variant_rules.yaml \
    --out src/autoware_carla_interface/vissim_integration/data/carla_variants_town01.yaml
```

### F5: ブリッジへの組み込み

- `vissim_integration/fleet_plan.py`(新規)に `FleetPlan` を置く。移植元の `FleetPlan` から、プールの部分を外したもの。
  - `__init__(path, blueprint_library, server_version)`: 生成物を読み、2.5の検査をする。`(type, file)` → モデル番号の表を作る(2.3)。
  - `actors_for(type_id, model_filename)`: actorの並び。表にない、または割り当てがなければ止める。
  - `body(blueprint_id)`: 実メッシュのL・W・中心。
- `simulation_synchronization.py`:
  - 起動時: `vissim_carla_variants` が空でなければ `FleetPlan` を作る。空なら今の `vtypes.json`(2.7)。
  - spawn(通常・ウォームアップ後の一括): `BridgeHelper.get_carla_blueprint()` の代わりに `FleetPlan` の1体目を使う。
    色と `driver_id` は、今と同じく車両IDをシードにした乱数で選ぶ(形は決まり、色だけ変わる)。
  - 更新: 2.4の配置。`bounding_box.extent` は使わない。
- `bridge_helper.py`: 2.4の計算を、CARLAに依存しない関数(`body_transforms(front, yaw, actors, mesh)`)として足す。テストしやすくするため。
- launch・パラメータ: `vissim_carla_variants` を足す(`autoware_carla_interface.launch.xml` → `carla_autoware.py`)。`vissim_launch_params_test.py` に足す。
- 終了時: ブループリントごとのspawn回数を、info ログに1行で出す(移植元の `fleet_summary` の簡略版)。
- テスト `test/vissim_carla_variants_test.py`: 起動時の検査(code ごと)、モデルの引き、`npc_model_unknown`、配置の計算(前端・中心・回転)、
  `vissim_carla_variants:=""` で今の動作になること。既存の `vissim_ego_spawn_gate_test.py` などが通ること。

完了条件: 全テストが通る。`colcon build` が通る。

#### 結果(F5、2026-11-16)

**計画から変えたこと**

- 今の `vtypes.json` に戻す指定を、空文字から **`vissim_carla_variants:=legacy_vtypes`** に変えた(2.7)。ROS 2のlaunchでは、空文字のパラメータの扱いが不安定なため。
- 止め方(5章の7): 今のブリッジは、初期化中の例外を `main()` の外へそのまま投げる(トレースバックが出る)。そこで、`EgoSpawnGateError` と同じ形にそろえた。
  - `main()` で `carla_fleet.VariantError` を受け、`Error: CARLA blueprint assignment: <code>: <根拠>` の1行を出して、終了コード1で終わる。新しい例外クラスは作っていない。
  - 起動時の検査は、**Vissimアダプタに接続する前**(`_init_vissim_integration()` の最初)に行う。そのため、生成物が古いときは、Windows側でVissimが起動しない。
  - 走行中の `npc_model_unknown`・`variant_model_empty` も同じ1行で止まる。後片付け(`_cleanup()`)は通る。

**変えたファイル**

- `vissim_integration/fleet_plan.py`(新規): `FleetPlan`。
  - 起動時の検査(2.5の表のすべて)に、次の3つを足した。
    - `variant_blueprint_unmeasured`: 割り当てたブループリントに実メッシュがない。
    - `variant_model_ambiguous`: 同じ (車種, ファイル名) が2つのモデルにある。移植元は先のものを使っていたが、こちらは止める。
    - `variant_model_identity_missing`: 生成物に `file`・`types` がない。
  - `model_for()`・`actors_for()`・`spec_for()`・`body()`、spawn回数の記録と要約。
  - `resolve_variants_path()`: 相対パスを `vissim_integration/data/` から解決する。
- `bridge_helper.py`:
  - `body_origins()`(純粋関数): 2.4の配置。前端から `front_offset_m + L/2 + center_x` 後ろへ、横は `center_y` を戻す。高さは今の `get_carla_transform()` と同じく、ピッチで変える。
  - `BridgeHelper.get_fleet_blueprints()`: 割り当てたブループリントを `blueprint_library.find()` で取る。色とドライバーは、今と同じく車両IDをシードにした乱数で選ぶ。重ね置きの体は同じ色にする。
  - `BridgeHelper.get_fleet_transforms()`。
- `simulation_synchronization.py`:
  - spawn(通常・ウォームアップ後の一括)と更新を、`_vehicle_spawn()`・`_vehicle_transform()` にまとめた。
  - `fleet`(クラス属性の既定は `None`)がなければ、今までの経路(`get_carla_blueprint()` と箱の `extent`)をそのまま通る。
  - `fleet` があるときは、spawnの時点で最終の位置に置き、更新でも箱の寸法を使わない(`carla.get_actor()` を呼ばない)。
  - **F5では、体の並びの1体目だけを使う**(トラック・トラクターは `carlacola` 1体、バスも1体)。重ね置きはF6。
- `carla_autoware.py`: パラメータ `vissim_carla_variants`、`_load_vissim_fleet_plan()`、`main()` での `VariantError` の処理、終了時の要約。
  - 要約は `CARLA bodies spawned for Vissim vehicles: <ブループリント> <回数> (<比率>, expected <生成物の比率>), ...` の1行。
- `carla_ros.py`・launch: `vissim_carla_variants`(既定 `carla_variants_town01.yaml`)を、両方のnode宣言に足した。
- 既存のスクリプト形式のテスト3本(`vissim_sim_period_test.py`・`vissim_warmup_ego_spawn_test.py`・`vissim_warmup_params_test.py`)のパラメータ表に、`vissim_carla_variants: legacy_vtypes` を足した。これらのテストが確かめている今までの動作は変えない。

**確かめたこと**

- `test/test_vissim_fleet_plan.py`(30件、新規):
  - `FleetPlan`: モデルの引き方(bytes・パスや拡張子付き)、重ね置きの並び、止まる条件(未知、割合0、各検査、重複、`follows_path`)。
  - 配置の計算: 向き0度・90度・180度、メッシュ中心のずれ、重ね置きの位置、坂。
  - `get_carla_transform()` との前端の向きの一致。
  - 同期処理: 割り当てたブループリントと位置でspawnし、同じtickで更新すること。色は車両IDだけで決まること。未知のモデルで止まること。ウォームアップ後の一括spawn。`fleet` がないときの今の動作。
- `python3 -m pytest test`: 198件成功、2件スキップ。スクリプト形式の `test/vissim_*_test.py` 8本もすべて成功。
- 実際のCARLA(0.9.15、Town01)でのスモークテスト(scratchpadのスクリプト。コミットしない)を行った。
  - 本物のブループリントライブラリと版で `FleetPlan` を作り、検査を通った。
  - Fiat 500・Yamaha・バスの1体目を、`get_fleet_blueprints()`・`get_fleet_transforms()` でspawnした。`micra`・`kawasaki.ninja`(ドライバーあり)・`carlacola` になり、`role_name` は `vissim_driver`。
  - 消したあと、車両は残っていない。
- `colcon build --packages-select autoware_carla_interface --symlink-install` が通った。
- Vissimとつないだ実機の確認は、F6のあとのF7で行う。

### F6: 重ね置き(F3で必要になったときだけ)

- 2.6のとおり、2体目以降を `vissim2carla_extra_ids` で持つ。spawn・destroy・更新・CARLA→Vissimの除外・close・終了時の削除(`carla_autoware.py`)を直す。
- 1体目のspawnが成功して、2体目以降が失敗したら、その車の全actorを消し、エラーログを出す(今の「spawnに失敗したら出さない」と同じ扱い)。
- 重ねる体どうしは重なっているので、その位置に直接spawnすると衝突で失敗する(F3の結果)。2体目以降は、重ならない場所(地下など)にspawnして物理を切ってから、`set_transform` で並べる。
- テスト: 2体の車のspawn・更新・destroy、CARLA→Vissimの除外に2体目が入らないこと、途中で失敗したときの後始末。

完了条件: テストが通る。

#### 結果(F6、2026-11-16)

**変えたファイル**

- `simulation_synchronization.py`:
  - 2体目以降を `vissim2carla_extra_ids`(`{vissim_id: [carla_id, ...]}`)で持つ。`__init__` を通さずに作るテスト用のオブジェクトでも使えるように、最初に使ったときに作るプロパティにした。1体目は今までどおり `vissim2carla_ids` にある(2.6)。
  - `mirrored_carla_ids()`: すべての体のIDを返す。
  - spawn(通常・ウォームアップ後の一括): 1体目のspawnが成功したら、`_spawn_extra_bodies()` で2体目以降をspawnする。
    - 2体目以降は1体目と重なるので、**車の前端の真下の地下**にspawnする。z = −200 m − 10 m × (体の番号 + 再試行の回数 × 体の数) に `CARLA_SPAWN_OFFSET_Z` が足される。F3で分かったとおり、その位置に直接spawnすると衝突で失敗するため。
    - 同じtickのうちに、`_synchronize_extra_bodies()` で本来の位置へ動かす。
    - 1体でもspawnできなければ(地下で1回 + 再試行3回)、その車の体をすべて消してエラーログを出し、その車はCARLAに出さない(今の「spawnに失敗した車は出さない」と同じ扱い)。
  - 更新: 2体目以降も、毎tick `get_fleet_transforms()` の位置と、1体目と同じ速度で動かす。
  - 削除: Vissimで消えた車は、すべての体を消す。
  - **CARLA→Vissimの自動取り込みから、すべての体を除外する**(`sync_carla_to_vissim()` の `spawned_actors - mirrored_carla_ids()`)。除外しないと、2体目以降がEGOと同じくVissimへ送られてしまう。
  - `close()`: すべての体を消す。
- `carla_autoware.py` の `_cleanup_vissim()`: すべての体を消す。
- `ego_spawn_gate.py` の `collect_gap_vehicles()`: **重ね置きの車は全体の外形で見る**。
  - 1体目の箱だけでは、EGOの前にいるバスの後ろの体を見落とし、隙間を最大で約7 m多く見積もる(2.6・6章のリスク)。
  - Vissimの前端から最後の体の後端までの長さと、体の最大の幅(実メッシュ)で見る。
  - 1体の車は今までどおりCARLAの箱で見る。
- テスト:
  - `test_vissim_fleet_plan.py` に5件足した: 2体目以降が失敗したときの後始末、削除、CARLA→Vissimの除外、`close()`、隙間の検査の外形。F5のテスト2件は、重ね置きの動作に合わせて直した(バスは3体、地下でのspawn、体ごとの位置と速度、同じ色)。
  - `vissim_ego_spawn_gate_test.py` のモックの同期オブジェクトに `vissim2carla_extra_ids = {}` を足した(実物と同じ形にするため)。

**確かめたこと**

- `python3 -m pytest test`: 203件成功、2件スキップ。スクリプト形式の `test/vissim_*_test.py` 8本もすべて成功。
- 実際のCARLA(0.9.15、Town01)で、実際の `SimulationSynchronization`・`CarlaSimulation` に偽のVissimをつないで動かした(scratchpadのスクリプト。コミットしない)。
  - バス・トラック・乗用車を直線道路で5 m/sで走らせ、69 tickで消した。
  - 体の原点は、前端から次の位置(計算どおり。`front_offset_m` + L/2 + メッシュ中心のずれ)。
    - バス(`carlacola` 3体): 2.59・6.10・9.61 m 後ろ
    - トラック(2体): 2.59・5.41 m 後ろ
    - 乗用車(`micra`): 1.79 m 後ろ
  - 高さは3体とも同じ。
  - Vissimへ取り込まれた体は0。消したあと、`vissim2carla_ids`・`vissim2carla_extra_ids` は空になり、ワールドに車両・センサーは残っていない。
  - 走っている様子: `docs/img/carla_blueprint_assignment/f6_sync_stacks.webp`(左がバス、右がトラック)。
- Vissimとつないだ実機の確認はF7。

### F7: ブリッジ単体での実機確認(Autowareなし)

信号設置のB6と同じ構成で動かす。

1. 起動時: 検査を通り、生成物の名前とモデル数がログに出る。わざと規則を1文字変えて、`variant_source_changed` で止まることも確かめる(確かめたら戻す)。
2. 走行中: 14モデルがすべて出ること(`npc_model_unknown` が出ないこと)、ブループリントごとのspawn回数が `expected_bodies_per_100_vehicles` と大きく違わないこと。
3. 見た目: 乗用車・トラック・トラクター・バス・バイクを1台ずつ、真上と横からスクリーンショットで撮る。Vissim GUIの同じ車と、長さ・位置(前端)を比べる。
4. ウォームアップ(`vissim_warmup_time`)ありで、一括spawnが動くこと。
5. 自車(CARLA→Vissim)の動きが今と変わらないこと。

完了条件: 1〜5の結果を、この計画書に書いてある。

#### 結果(F7、2026-11-16、CARLA 0.9.15、Town01、Vissim(Windows)、ブリッジ単体)

起動は信号設置のB6と同じ引数(`use_vissim:=true`・`sync_traffic_lights:=true`・`spawn_point:=334.74,263.00,0.30,0.0,0.0,90.0`)。`vissim_carla_variants` は既定(`carla_variants_town01.yaml`)。CARLAの様子は、tickしない監視スクリプトで20秒ごとに見た(scratchpad。コミットしない)。

1. **起動時の検査**
   - 生成物と生成元の写しをscratchpadに作り、規則に1行足して、`vissim_carla_variants` に絶対パスで渡した。リポジトリのファイルは変えていない。
   - 結果: `Error: CARLA blueprint assignment: variant_source_changed: {'source': 'rules', ..., 'recorded_sha256': '08ba25bd…', 'current_sha256': '17776f50…'}` の1行を出し、終了コード1で止まった。launch全体も止まった(`on_exit="shutdown"`)。
   - ログに接続の行はなく、**Vissimアダプタには接続していない**。プロセスも残っていない。
   - 止まるときに出る `republish` の `exit code -11` は、11/12の起動ログにも出ている。今回の変更とは関係しない。
   - 通常の起動では、`CARLA blueprints of Vissim vehicles: .../carla_variants_town01.yaml (14 models)` が出た。
2. **走行中**(`vissim_sim_period:=240`。シミュレーション時間240秒、実時間で約9分)
   - `Vissim co-simulation period elapsed (4800 ticks), stopping.` のあと、後片付けまで通った。launchの終了コードは0。
   - **エラーは0件**: `npc_model_unknown`・`variant_model_empty`、spawnの失敗、2体目以降の失敗はなかった。出てきたVissimの車は、すべて生成物の表で引けた。
   - 終了時の要約(体の数。比率の見込みは `expected_bodies_per_100_vehicles` から):

     | ブループリント | 体の数 | 比率 | 見込み |
     | -------------- | -----: | ---: | -----: |
     | `carlamotors.carlacola` | 27 | 44% | 50% |
     | `mini.cooper_s_2021` | 10 | 16% | 15% |
     | `audi.tt` | 6 | 10% | 10% |
     | `dodge.charger_2020` | 6 | 10% | 5% |
     | `kawasaki.ninja` | 5 | 8% | 7% |
     | `tesla.model3` | 5 | 8% | 8% |
     | `nissan.patrol` | 2 | 3% | 2% |
     | `nissan.micra` | 1 | 2% | 3% |

     台数が少ないので多少のずれはあるが、見込みとおおむね合う。割り当て先8種がすべて出た。各車がどのモデルだったかはdebugログにしか出ないので、モデル単位では数えていない。
   - 監視スクリプト(約180秒分): 同時に出ていた車は最大49台。新しく現れたactorの `role_name` は、`vissim_driver` 48と、`ego_vehicle` 1(`toyota.prius`)だけ。
   - **地下に取り残された体は一度もなかった**(z < −50 mのactorが0)。
   - **重なった `carlacola` の組228組は、すべて横ずれ0.05 m未満、向きの差0.5度未満で並んでいた**。監視スクリプトが「並びが崩れている」とした3組は、横に4.0 m離れ、向きが180度違う。反対車線ですれ違ったトラックどうしを拾っただけ。
   - 終了後、CARLAに車両・センサーは残っていない(1回tickして確かめた)。
3. **見た目**: 走っているバスを4枚撮影した(`docs/img/carla_blueprint_assignment/f7_bus_with_vissim.webp`)。どれも、Vissimの道路に沿って `carlacola` 3体が隙間なく並んで走っている。Vissim GUIとの並べた比較はしていない(前端の位置はF0の3とF6で確かめた)。
4. **ウォームアップあり**(`vissim_warmup_time:=100`、`vissim_sim_period:=120`。アダプタは起動し直した)
   - `[VISSIM WARMUP] completed: t=100.0 s vehicles=14 pedestrians=2 (wall 33.4 s)`
   - `[VISSIM WARMUP] caught up: carla_spawned=14 vissim_only=0 pedestrians=2 pedestrians_vissim_only=0 spawn_retries=0`。重ね置きの車も、一括spawnで全部出た。
   - `[EGO SPAWN CHECK] t=100.00 s front=none rear=none overlap=none result=SAFE`
   - 一括spawnの直後から90秒見た。重なった `carlacola` の組44組はすべて並びが合い、地下に残った体はなかった。`role_name` は `vissim_driver` 33と `ego_vehicle` 1だけ。
   - 終了: `period elapsed (4400 ticks)`、要約(`carlacola` 27(48%)など)、`Cleanup complete.`、launchの終了コード0、CARLAに車両・センサーなし。
   - EGOの前後に車がいなかったので、重ね置きの車を全体の外形で見る隙間の検査(F6)は、実機では働いていない(テストでは確かめた)。
5. **自車(CARLA→Vissim)**
   - EGO(`prius`、`ego_vehicle`)は、2回とも出て走行が終わった。
   - 2体目以降がCARLA由来の車として誤ってVissimへ送られると、自車の枠(`vissim_simulator_vehicles=1`)が足りなくなるはず。しかし、ログに警告もエラーもなかった。
   - Vissim側で自車が正しく扱われたかを直接見るログはない。Autowareで走らせるF8で、EGOとVissimの車の動きを見る。

### F8: Autoware全体での実機確認

- 起動手順書の通常の手順で、Autowareまで動かす。
- 知覚が出す物体の大きさを、今(`vtypes.json`)と比べる。`/perception/object_recognition/objects` の寸法を、同じ地点・同じくらいの時刻で見る。
  大型車(トラック・バス)で差が出るかを中心に見る。
- EGOのゴールまでの走行に、悪い影響がないこと。

完了条件: 比べた結果と、気づいた問題を、この計画書に書いてある。

### F9: ドキュメントとコミット

- 起動手順書: 生成物の作り直しの手順(inpx・規則・CARLAの版を変えたとき)、`vissim_carla_variants` の意味、止まったときの `code` の見方。
- README: 「Vissim co-simulation」の節に、ブループリント割り当ての節を足す。Known Issuesを直す。
- `Vissim_車両寸法一覧.md`: F0で正にしたinpxの値に直し、割り当て先の列を生成物の値にする。`CARLA_車両寸法一覧.md` に実メッシュの列を足す。
- `vtypes.json`: 2.7の戻し用に残す。READMEに、既定では使わないことを書く。
- 計画書(本書)に、各タスクの結果を書く。
- SUMO計画書に渡すこと: 共通部品・規則・実メッシュのパスとコミット、SUMOで足すもの(`--rou`、`sumo_fleet.py`、ブリッジの組み込み)。
- コミット: タスクの区切りで、`feat(autoware_carla_interface): ...` / `docs(vissim-carla-autoware co-sim): ...`、日本語。pushはしない。

---

## 5. 決めること(作業の前か途中)

| #   | 内容 | 推奨 | 決める時期 |
| --- | ---- | ---- | ---------- |
| 1   | 使うinpx | 決定(F0): Windows PCとCARLAリポジトリの版が同じ(sha256 `2265ff82…`)なので、それを使う | F0 |
| 2   | 大型車(トラクター22・トラック23・バス31) | 決定(F3): fallbackを `carlacola` にし、救急車を除外する。トラクターとトラックは2体、バスは3体の重ね置き | F3 |
| 3   | 二輪 | 決定(F3): 出す。BIKEの許容はそのまま。0.9.15の二輪はライダーが乗った形で、LiDARで測った寸法もライダーを含む | F3 |
| 4   | 乗用車にバンが割り当たる | 決定(F3): `volkswagen.t2*` を除外する(高さが合わない) | F3(F7で見直す) |
| 5   | 塗装が特殊なもの | 決定(F3): パトカー2種・タクシー・消防車・救急車を外す。撮影して確かめた | F3 |
| 6   | 自車を候補から外すか | 決定(F3): 外す(`vehicle.toyota.prius`)。自車と同じ形のNPCが近くにいると、カメラ・LiDARで見分けにくい | F3 |
| 7   | 止めるときの例外 | 決定(F5): `EgoSpawnGateError` と同じく、`main()` で `carla_fleet.VariantError` を受けて1行のエラーを出し、終了コード1で終わる。新しい例外クラスは作らない。起動時の検査はVissimアダプタへの接続より前に行う | F5 |
| 8   | プール | 入れない(0.2)。spawn直後の落下などが出たら、別の計画で扱う | F7 |

---

## 6. リスクと対応

| リスク | 対応 |
| ------ | ---- |
| Windowsで読まれるinpxと、生成に使ったinpxが食い違う | F0で正を決める。起動手順書に、sha256で比べる手順を書く。モデルが変わっていれば、走行中に `npc_model_unknown` で止まる |
| 0.9.15で `cast_ray` やラベルが使えず、実メッシュを測れない | F0の4で先に1体だけ試す。だめなら、箱の寸法(`CARLA_車両寸法一覧.md`)を代わりに使い、生成物の `geometry` にそのことを書く(版の照合だけは働く) |
| 大型車がどの規則でも割り当たらない | 5章の2の案から選ぶ。決まるまでは、そのモデルだけ割合0のinpxで確かめを進める |
| 重ね置きで、隙間の検査(`ego_spawn_gate`)が1体目しか見ない | F6の最初に、`ego_spawn_gate.py` の隙間の計算が前後どちらの車を見ているかを調べる。後ろの車も見るなら、2体目以降を含めた外形で見るように直す。F7で、バスの近くにspawnする場合を見る |
| 位置の基準(前端か中心か)を取り違え、車が前後にずれる | F0の3で確かめる。F5のテストで、前端・中心・メッシュ中心のずれを数値で確かめる |
| `vtypes.json` と生成物の2つの経路が残り、手入れが漏れる | 生成物を既定にし、`vtypes.json` は空文字を明示したときだけにする。SUMO計画を終えた時点で、消すかを決める |
| SUMOブランチとの共通部品の食い違い | 共通部品は本計画で仕上げてから、同じパス・同じ中身でSUMOブランチへ写す(SUMO計画の最初のタスク)。写したコミットのハッシュをSUMO計画書に書く |
