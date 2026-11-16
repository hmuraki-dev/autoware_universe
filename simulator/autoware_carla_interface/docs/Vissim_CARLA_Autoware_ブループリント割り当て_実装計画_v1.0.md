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
| 計測ツール | `tools/carla_fleet/measure_blueprint_mesh.py` | 束(自車の除外を引数にする) |
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
**空文字を明示したときだけ**、今の `vtypes.json` と乱数の選び方に戻る(比べるためと、戻すため)。
規定のファイルが見つからないときは、`vtypes.json` に戻さず、`variant_config_missing` で止める。

---

## 3. タスク一覧

| #   | タスク | 場所 | 状態 |
| --- | ------ | ---- | ---- |
| F0  | 事前確認(使うinpx、`model_filename` の文字列、位置の基準、0.9.15での計測) | Windows・Linux・CARLA | 完了(2026-11-16) |
| F1  | 共通部品の取り込み(`carla_fleet.py`、規則の雛形、回帰テスト、PyYAMLの依存) | autoware_universe | 未着手 |
| F2  | 計測ツールの移植と、全ブループリントの実メッシュ計測 | autoware_universe・CARLA | 未着手 |
| F3  | 規則の確定(除外、fallback、大型車、二輪) | autoware_universe | 未着手 |
| F4  | inpxの読み込み・生成ツールと、Town01の生成物 | autoware_universe | 未着手 |
| F5  | ブリッジへの組み込み(起動時の検査、モデルの引き、実メッシュ中心での配置) | autoware_universe | 未着手 |
| F6  | 重ね置き(F3で必要になったときだけ) | autoware_universe | 未着手 |
| F7  | ブリッジ単体での実機確認(Autowareなし) | Linux + Windows | 未着手 |
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

### F2: 計測ツールと実メッシュ計測

- 束の `measure_blueprint_mesh.py` を `tools/carla_fleet/` に入れる。変えるところ:
  - `'j6' in blueprint.id` を、`--exclude`(fnmatch、複数可)にする。既定は空。自車を外すかは規則で決める(F3)。
  - F0の4で決めた、tickの扱い。
  - 出力ファイル名を `--out <ディレクトリ>/mesh_measurements.json` から、`--out <ファイル>` に変える(置き場所を2.1にするため)。
- 全41ブループリントを測る。CARLA(Town01)を起動し、Autowareとブリッジは起動しない。
- 結果を `data/carla_fleet/mesh_measurements_carla0.9.15.json` に置く。測れなかったブループリントは、理由をこの計画書の結果に書く。
- `CARLA_車両寸法一覧.md` の箱の寸法と並べ、差の大きいもの(移植元では消防車・fusoで1 m前後)を結果に書く。

完了条件: 実メッシュのファイルに `carla_server_version` が記録されている。二輪を含む全ブループリントの値、または測れない理由がある。

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

### F6: 重ね置き(F3で必要になったときだけ)

- 2.6のとおり、2体目以降を `vissim2carla_extra_ids` で持つ。spawn・destroy・更新・CARLA→Vissimの除外・close・終了時の削除(`carla_autoware.py`)を直す。
- 1体目のspawnが成功して、2体目以降が失敗したら、その車の全actorを消し、エラーログを出す(今の「spawnに失敗したら出さない」と同じ扱い)。
- テスト: 2体の車のspawn・更新・destroy、CARLA→Vissimの除外に2体目が入らないこと、途中で失敗したときの後始末。

完了条件: テストが通る。

### F7: ブリッジ単体での実機確認(Autowareなし)

信号設置のB6と同じ構成で動かす。

1. 起動時: 検査を通り、生成物の名前とモデル数がログに出る。わざと規則を1文字変えて、`variant_source_changed` で止まることも確かめる(確かめたら戻す)。
2. 走行中: 14モデルがすべて出ること(`npc_model_unknown` が出ないこと)、ブループリントごとのspawn回数が `expected_bodies_per_100_vehicles` と大きく違わないこと。
3. 見た目: 乗用車・トラック・トラクター・バス・バイクを1台ずつ、真上と横からスクリーンショットで撮る。Vissim GUIの同じ車と、長さ・位置(前端)を比べる。
4. ウォームアップ(`vissim_warmup_time`)ありで、一括spawnが動くこと。
5. 自車(CARLA→Vissim)の動きが今と変わらないこと。

完了条件: 1〜5の結果を、この計画書に書いてある。

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
| 2   | 大型車(トラクター22・バス31) | F2の実メッシュで試算し直してから決める。案は次の3つ。(a) fallbackを `european_hgv` にする(幅が太い側の許容内で、バスは2体の重ね置き。トラクターは救急車より短いので別に扱う)。(b) HGV・BUSの細い側の許容を広げる。(c) inpxでそのモデルの割合を0にする。推奨は(a)で、トラクターは長さの許容をそのモデルだけ広げる案と比べる | F3 |
| 3   | 二輪 | 出す。ライダーは乗せない(Vissimの寸法はバイクだけ)。BIKEの許容はそのまま | F3 |
| 4   | 乗用車にバンが割り当たる | 規則どおりにし、見た目で困るものだけ `excluded` で外す | F3(F7で見直す) |
| 5   | 塗装が特殊なもの | パトカー・タクシー・消防車を外す(移植元と同じ考え方)。撮影して決めた結果を規則のコメントに書く | F3 |
| 6   | 自車を候補から外すか | 外す(`vehicle.toyota.prius`)。自車と同じ形のNPCが近くにいると、カメラ・LiDARで見分けにくい | F3 |
| 7   | 止めるときの例外 | 今のブリッジが異常で止まるときの経路(`vissim_max_consecutive_failures` など)をF5の最初に調べ、それにそろえる。できれば新しい例外クラスは作らない | F5 |
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
