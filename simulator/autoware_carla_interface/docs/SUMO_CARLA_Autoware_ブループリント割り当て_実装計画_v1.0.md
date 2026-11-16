# SUMO-CARLA-Autoware: CARLAブループリント割り当て(Vissimと同じ車両構成) 実装計画 v1.0

作成日: 2026-11-16

対象ブランチ:

| リポジトリ | ブランチ | 主に変えるもの |
| ---------- | -------- | -------------- |
| autoware_universe(`~/autoware.1.9.0/src/universe/autoware_universe`) | `feature/sumo_co-sim` | Vissim構成から写す共通部品、SUMOの読み込み、`sumo_integration/` のブループリント選択と配置、生成物、テスト、起動手順書・README |
| autoware_universe | `feature/vissim_windows_co-sim` | 共通部品の小さな整理だけ(S1の1。配置の計算を共通部品へ移す) |
| CARLA(`~/CARLA`) | `main` | `Co-Simulation/Sumo/examples/` のTown01のvType(Vissimの14モデル)と、`Town01.rou.xml` の車両グループの中身 |

参考:

- Vissim構成の計画書: `git show feature/vissim_windows_co-sim:simulator/autoware_carla_interface/docs/Vissim_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md`。
  とくに7章「SUMO計画への引き継ぎ」と、F0〜F8の結果(本書では「Vissim計画のF2」のように書く)。
- 移植元の束 `blueprint_assignment_handoff_20261116.tar.gz`: `files/runtime/bridge/sumo_fleet.py`、`files/tests/core/test_sumo_fleet.py`、`files/reference/bridge_excerpts.md` のSUMOの節、`files/docs/CARLA_blueprint候補拡張_設計案.md` の§8。

---

## 0. 目的・スコープ

SUMO構成のTown01で、**車両の構成(寸法と割合)をVissim構成と同じにし、CARLAの体もVissim構成と同じ割り当てにする**。
これで、Vissim構成とSUMO構成を、同じ車両の構成で比べられるようになる。

今のSUMOのシナリオは、vTypeのIDがCARLAのブループリント名で、寸法もCARLAのバウンディングボックスの値(`carlavtypes.rou.xml`、41種)。
そのため、Vissim構成の仕組みをそのまま入れても、「各vTypeに自分と同じブループリントが割り当たる」だけで効果が小さい(Vissim計画の7章)。
そこで、利用者と次のように決めた(2026-11-16)。

- **Town01のSUMOのvTypeを、Vissimの14モデル(寸法・車種内の割合)にする。**
- **対象はTown01だけ。** Town05・Town10HDは、今の選び方(vType = ブループリント)のまま動くようにする。

### 0.1 目標の構成

```
[CARLAリポジトリ] Co-Simulation/Sumo/examples/
    vissim_fleet_vtypes.rou.xml(新規。Vissimの14モデルの寸法のvType)
    rou/Town01.rou.xml(車両グループ A〜E の中身を、上のvTypeに差し替える。flowはそのまま)
    Town01.sumocfg(route-files に上のファイルを足す)
                 │
[autoware_universe] build_carla_variants.py --sumocfg Town01.sumocfg(vTypeを寸法でまとめてモデルにする)
                 │   規則・実メッシュはVissim構成と同じファイル
                 ▼
    sumo_integration/data/carla_variants_sumo_town01.yaml(14モデル、割り当てはVissimと同じ)
                 │ 起動時に検査(.rou.xmlのsha256も照合)
    SUMOブリッジ: SUMOのvType ID -> モデル -> actorの並び -> 実メッシュ中心で配置(重ね置きを含む)
```

車両グループとVissimのモデルの対応:

| SUMOの車両グループ(今の中身) | 今の比率(flowの台数) | 差し替える中身(Vissimのモデル、グループ内の割合はVissimの車種内の割合) | CARLAの体(Vissim構成と同じ) |
| --------------------------- | -------------------: | ------------------------------------------------------------------ | ---------------------------- |
| A_car(乗用車16種) | 70% | 1 Fiat 500 0.06、2 Tesla Model Y 0.05、3 Golf 0.18、4 B Class 0.11、5 C Class 0.14、6 Superb 0.05、7 A6 0.11、8 Qashqai 0.07、9 Tiguan 0.18、10 ID 4 0.05 | `micra`・`audi.tt`・`mini.cooper_s_2021`・`tesla.model3`・`dodge.charger_2020`・`nissan.patrol` |
| B_truck(`carlacola`) | 10% | 23 Delivery DAF LF(7.93 × 2.55 m) | `carlacola` × 2 |
| C_trailer(`european_hgv`) | 5% | 22 Semi-Tractor Volvo E(5.96 × 2.55 m) | `carlacola` × 2 |
| D_bus(`fusorosa`) | 5% | 31 Bus C2 Standard(12.135 × 2.55 m) | `carlacola` × 3 |
| E_bike(バイク3種) | 10% | 70 Yamaha MT 07(2.098 × 0.907 m) | `kawasaki.ninja` |

グループどうしの比率(70・10・5・5・10%)と、flow・経路・信号は変えない(5章の1)。

### 0.2 スコープ外

- Town05・Town10HDのvType(今のまま。生成物を作らない。`legacy_vtypes` で動く)。
- SUMOの車両の挙動パラメータの調整(加速度・最高速度など。vClassの既定値のまま)。
- CARLA→SUMOの向き(自車)。自車のvTypeは今までどおり、`carlavtypes.rou.xml` の自車のブループリント名のもの。
- 作り置き(プール)、連結車。Vissim構成と同じ。
- Vissim構成の車種の比率(乗用車60%など)に合わせること(5章の1)。

---

## 1. 事前調査で分かったこと(2026-11-16)

| #   | 項目 | 内容 |
| --- | ---- | ---- |
| 1   | 今の割り当て | `sumo_integration/bridge_helper.py` の `get_carla_blueprint()`。SUMOのvType IDがCARLAのブループリント名と一致すれば、それを使う。一致しなければ、`data/vtypes.json` から同じvClassのものを `random.choice` で選ぶ(シードなし)。 |
| 2   | 今のシナリオ | `~/CARLA`(`main`)の `Co-Simulation/Sumo/examples/`。Town01・Town05・Town10HDとも、`route-files` は `carlavtypes.rou.xml,rou/<Town>.rou.xml`。`carlavtypes.rou.xml` の41のvTypeは、IDがブループリント名、寸法がCARLAの箱の値(小数15桁)、`vClass` と `guiShape` だけを持つ。 |
| 3   | Town01の需要 | `rou/Town01.rou.xml`。5つの車両グループ(vTypeDistribution `A_car`・`B_truck`・`C_trailer`・`D_bus`・`E_bike`)と、50のflow(`period="exp(...)"`)。グループの比率は70・10・5・5・10%(入力1・2は各300台/h、入力3〜10は各40台/h)。Town05・Town10HDは、1台ずつvTypeを指定している。 |
| 4   | 1台あたりのactor数 | 1台 = 1 actor。`sumo2carla_ids` を使っているのは、`simulation_synchronization.py`(spawn・destroy・更新・ウォームアップ後の一括spawn・CARLA→SUMOの除外・close)、`ego_spawn_gate.py:269`(隙間の検査)、`carla_autoware.py:673`(終了時の削除)。Vissim構成とほぼ同じ形。 |
| 5   | 位置の基準 | SUMOの位置も**前端中央**。`get_carla_transform(transform, extent)` が、SUMOの車の寸法(`extent`、TraCIの `VAR_LENGTH`・`VAR_WIDTH` から作る)で中心へ戻している。CARLAの箱は使っていない。 |
| 6   | モデルの手がかり | `SumoActor` は `type_id`(vTypeのID。vTypeDistributionの車は、選ばれたメンバーのID)、`vclass`、`extent` を持つ。DSIの `ModelFileName` に当たるものはないが、vTypeのIDで引ける。 |
| 7   | SUMOの版 | Linuxの `sumo` は1.20.0。SUMOはLinuxで動くので、ブリッジは実行時に `.sumocfg` と `.rou.xml` を読める(Vissim構成と違う点)。 |
| 8   | 移植元のSUMO部品 | `sumo_fleet.py`: `.rou.xml` のvTypeを寸法(L・W)でまとめてモデルにする(`L4.781_W2.022` の形)。割合は、flowの台数 × vTypeDistributionの確率。走行中は車の寸法でモデルを引く。`.rou.xml` は1つしか読まない(こちらは `carlavtypes.rou.xml` と `rou/Town01.rou.xml` の2つに分かれている)。 |

---

## 2. 方針

### 2.1 共通部品はVissim構成から写す(手で書き直さない)

Vissim計画の7章の表のファイルを、`git checkout feature/vissim_windows_co-sim -- <パス>` で同じパスに写す。
あとで2つのブランチをマージしたときに、食い違いや衝突が起きないようにするため。

ただし、配置の計算 `body_origins()` は、今は `vissim_integration/bridge_helper.py`(Vissim構成のvendorしたファイル)にある。
SUMOでも同じ計算を使うので、**先にVissim構成のブランチで `carla_fleet.py` に移し**、それから写す(S1の1)。

`build_carla_variants.py` は、`vissim_integration/inpx_models.py` を読み込む。そのため、`vissim_integration/inpx_models.py` と `vissim_integration/__init__.py`(空)も写す。
SUMOブランチに、Vissimのモジュールが1つだけ入ることになる。ただし、中身はVissim構成と同じなので、マージでは衝突しない。

### 2.2 SUMOのvType(CARLAリポジトリ)

- `Co-Simulation/Sumo/examples/vissim_fleet_vtypes.rou.xml` を新しく作る。Vissimの14モデルを、それぞれ1つのvTypeにする。
  - ID: `vissim.<モデル番号>.<短い名前>`(例: `vissim.1.fiat_500`)。ブループリント名とは重ならない。
  - `length`・`width`・`height`: Vissimのモデルの寸法(inpxの値、小数3桁)。
  - `vClass`: 乗用車 `passenger`、23 `truck`、22 `truck`、31 `bus`、70 `motorcycle`。`guiShape` もそれに合わせる。
- `rou/Town01.rou.xml` の5つのvTypeDistributionの `vTypes` と `probabilities` を、0.1の表のとおりに差し替える。flowは変えない。
- `Town01.sumocfg` の `route-files` を `carlavtypes.rou.xml,vissim_fleet_vtypes.rou.xml,rou/Town01.rou.xml` にする。
  - `carlavtypes.rou.xml` は残す。自車(CARLA→SUMO)のvTypeが、ここのブループリント名のものだから。
- 生成するのは、Windowsにある `.inpx` からではなく、Vissim構成の生成物(`carla_variants_town01.yaml` の `L`・`W`)と、`Vissim_車両寸法一覧.md` の高さから。手で書かず、小さなスクリプトで作る(S2)。

### 2.3 モデルの読み方と引き方

- 生成ツールに `--sumocfg` を足す。`route-files` の全ファイルを読み、移植元の `read_models` と同じ規則で、vTypeを寸法でまとめてモデルにする。
  割合は、flowの台数 × vTypeDistributionの確率。
  - 移植元の `read_models` は、1つのファイルの中だけでvTypeとflowを探す。こちらは、複数のファイルにまたがって読めるようにする。
  - Town01では、flowが使うのは差し替えた14のvTypeだけになる。`carlavtypes.rou.xml` の41のvTypeは、割合0で `skipped` に入る。
- モデルの名前は、移植元と同じ `L<長さ>_W<幅>`。生成物の各モデルに、まとめたvTypeのID(`vtypes`)を書く。
- **走行中は、vTypeのIDでモデルを引く**(移植元は寸法で引く)。SUMOがvTypeのIDを直接返すので、確実に引ける。
  あわせて、その車の寸法(`extent`)がモデルのL・Wと合うことも確かめる(合わなければ止める)。
- 起動時に `route-files` の各ファイルのsha256を照合する(移植元と同じく、`.rou.xml` が変わったら止める)。

### 2.4 ブリッジ

Vissim構成のF5・F6と同じ作りにする。

- `sumo_integration/fleet_plan.py`(新規): Vissim構成の `FleetPlan` と同じ検査をし、vTypeのIDで引く。
- spawn・更新: 前端(SUMOの位置)から、`carla_fleet.body_origins()` で実メッシュの中心に置く。SUMOの `extent` は使わない。
- 重ね置き: 2体目以降を `sumo2carla_extra_ids` で持ち、車の真下の地下にspawnしてから並べる。spawn・更新・削除・close・後片付けで扱い、CARLA→SUMOの自動取り込み(`sync_carla_to_sumo`)からは除外する。
- EGOの安全スポーンの隙間の検査(`collect_gap_actors`)は、重ね置きの車を全体の外形で見る。
- 起動パラメータ `sumo_carla_variants`(5章の3)。
- 止まり方: Vissim構成と同じく、`main()` で `VariantError` を受けて1行出し、終了コード1で終わる。起動時の検査は、SUMOを起動する前に行う。

---

## 3. タスク一覧

| #   | タスク | 場所 | 状態 |
| --- | ------ | ---- | ---- |
| S0  | 事前確認(TraCIが返すvType IDと寸法、SUMO単体での今の交通) | Linux | 完了(2026-11-16) |
| S1  | 共通部品の整理(Vissimブランチ)と取り込み(SUMOブランチ) | autoware_universe(両ブランチ) | 完了(2026-11-16) |
| S2  | SUMOのvTypeを作り、Town01の需要を差し替える | CARLA | 完了(2026-11-16、CARLA `main` の `6a1c918`) |
| S3  | SUMOの読み込み(`sumo_fleet.py`)、生成ツールの `--sumocfg`、Town01の生成物 | autoware_universe | 完了(2026-11-16) |
| S4  | ブリッジへの組み込み(検査、vType IDでの引き、配置、重ね置き、隙間の検査) | autoware_universe | 完了(2026-11-16) |
| S5  | ブリッジ単体での実機確認(Autowareなし) | Linux | 未着手 |
| S6  | Autoware全体での実機確認(Vissim構成との比較を含む) | Linux | 未着手 |
| S7  | ドキュメントとコミット | autoware_universe・CARLA | 未着手 |

依存関係: S0 → S1 → S2 → S3 → S4 → S5 → S6 → S7。S2はS1と並行してよい。
S5〜S6はSUMOがLinuxで動くので、Windowsの準備は要らない。

---

## 4. 各タスクの内容

### S0: 事前確認

1. **TraCIが返すvType IDと寸法**: 今のTown01を、SUMOだけでTraCIを通して動かす。vTypeDistributionの車について、`vehicle.getTypeID()` がメンバーのID(例: `vehicle.audi.tt`)を返すことと、`VAR_LENGTH`・`VAR_WIDTH` がそのvTypeの値と一致することを確かめる。
2. **今の交通の記録**: 今の `Town01.sumocfg` で、SUMOだけを1時間分動かす。入った台数、出た台数、teleport(詰まりで消えた車)の数、待ち(入れなかった車)を記録する。S2で差し替えたあとと比べるため。
3. **既存の文書が使っているTown01の値**: 起動手順書の確認済みのスポーン地点、ウォームアップの時間、信号の確認結果のうち、車両の長さに関わるものを洗い出す。大型車が長くなるので(バス10.27 → 12.135 m、トラック5.20 → 7.93 m)、S5で確かめ直す。

完了条件: 1の結果、2の表、3の一覧が、この計画書にある。

#### 結果(S0、2026-11-16)

**大事な前提: SUMOのシナリオはCARLAリポジトリの `main` にある。**
作業の途中で、`~/CARLA` の作業ツリーが `feature/vissim_windows` になっていた(reflogに、`main` との間を切り替えた跡がある)。
`feature/vissim_windows` の `Co-Simulation/Sumo/examples/` は古い版で、中身が違う。

- `carlavtypes.rou.xml` が27種。
- `rou/Town01.rou.xml` は1台ずつ `<vehicle type="...">` を書く形で、車両グループがない。

1章の2・3は `main` の版(41種、5つの車両グループ)。最初に `feature/vissim_windows` の版で1を動かしてしまったので、`main` の版で測り直した。
`main` の版は、作業ツリーを切り替えずに、`git -C ~/CARLA archive main Co-Simulation/Sumo/examples` でscratchpadに取り出して使った。
S2(CARLAリポジトリの変更)とS5・S6(実機)では `main` が要るので、5章の7で扱いを決める。

| #   | 項目 | 結果(`main` の版、SUMO 1.20.0、TraCI、0.05秒刻み) |
| --- | ---- | -------------------------------------------- |
| 1   | TraCIが返すvType IDと寸法 | **選ばれたメンバーのID**が返る(例: `A_car` の車は `vehicle.nissan.patrol` など)。車両グループのID(`A_car` など)は返らない。ただし `vehicletype.getIDList()` には車両グループのIDも入っている。300秒分で出発した61台すべてで、`vehicle.getLength()`・`getWidth()` がそのvTypeの値と完全に一致した。そのため、**vTypeのIDで引き、寸法で照合する方式(2.3)で進められる**。今のシナリオでも、自車と同じ `toyota.prius` がNPCとして出ている。 |
| 2   | 今の交通(SUMOだけ、1時間) | 下の表。teleportと衝突は0。 |
| 3   | 長さに関わる、起動手順書の値 | 下の一覧。S5で確かめ直す。 |

2の結果(`sumo -c Town01.sumocfg --end 3600 --step-length 0.05`、`--statistic-output`・`--tripinfo-output`。実時間6秒):

| 項目 | 値 |
| ---- | -: |
| 読み込んだ車 / 投入した車 / 投入を待っている車(終了時) | 924 / 919 / 5 |
| 到着した車 | 841(乗用車572、トラック96、バイク87、トレーラー53、バス33) |
| 終了時に走っている車 | 78 |
| teleport(詰まりなどで消えた車) | 0 |
| 衝突・緊急停止・急ブレーキ | 0・0・0 |
| 平均の所要時間 / 停止時間 / 時間損失 / 出発の遅れ [秒] | 180.6 / 84.3 / 118.4 / 1.8 |
| 平均の速度 [m/s] / 走行距離 [m] | 5.64 / 835.1 |

3の一覧(`docs/SUMO-CARLA-Autoware_co-sim_起動手順.md`):

- 2.4.1の起動コマンド(Town01): `sumo_warmup_time:=600`、`spawn_point:="267.65,133.49,0.30,0.0,0.0,0.0"`。ウォームアップ後の空き待ちで、すぐにスポーンできるか。
- 2.4.3の確認済みの地点の表: #37・#36・#144・#249・信号372の手前。交通量と、#37の空き待ち13.75秒の実績。
  - これは全交差点に信号を置く前の値で、手順書にも「値が変わる」と書いてある。
  - 大型車が長くなると、空き待ちの時間がさらに変わりうる。
- 2.4.2: `ego_spawn_front_margin`・`ego_spawn_rear_margin` の既定値20 m。隙間はバンパー間の距離で見るので、大型車が長くなると空きにくくなる。
- 2.7(信号状態の通知): 信号372の手前(`334.74,263.00,...`)での停止・発進の確認。前の車が大型車のときの停止位置。

### S1: 共通部品の整理と取り込み

1. **Vissimブランチで**: `vissim_integration/bridge_helper.py` の `body_origins()` を、`carla_fleet.py` に移す。`bridge_helper.py` は、それを呼ぶだけにする。テスト(`test_vissim_fleet_plan.py` の配置の計算)が通ることを確かめて、コミットする。
2. **SUMOブランチで**: Vissim計画の7章の表のファイルと、`tools/carla_fleet/build_carla_variants.py`、`vissim_integration/inpx_models.py`・`vissim_integration/__init__.py`、`test/test_build_carla_variants.py` を、`git checkout feature/vissim_windows_co-sim -- <パス>` で写す。
   - `package.xml` は、`python3-yaml` の行だけを手で足す(ほかの差分は持ち込まない)。
   - 写したテストが通ることを確かめる。`test_build_carla_variants.py` のうち、Vissimの生成物(`vissim_integration/data/carla_variants_town01.yaml`)を見るテストは、SUMOブランチにはファイルがないのでスキップさせる(テストを直すのはVissimブランチで行い、写し直す)。

完了条件: SUMOブランチで、写したテストが通る。写したファイルが、Vissimブランチと同じ中身(`git diff feature/vissim_windows_co-sim -- <パス>` が空)。

#### 結果(S1、2026-11-16)

1. **Vissimブランチ**(`2abb67729`、`refactor(autoware_carla_interface): ...`)
   - `body_origins()` を `vissim_integration/bridge_helper.py` から `carla_fleet.py` へ移した。`bridge_helper.py` は `from ..carla_fleet import body_origins` で読み込むだけにしたので、動作は変わらない。
   - 純粋な関数のテスト4件を `test_vissim_fleet_plan.py` から `test_carla_fleet.py` へ移した。
   - `test_build_carla_variants.py` で、Vissimの生成物を見る2件を、生成物がないブランチではスキップするようにした。
   - Vissimブランチのテスト: 203件成功・2件スキップで、移す前と同じ。スクリプト形式の `vissim_*_test.py` 8本もすべて成功。
2. **SUMOブランチ**(`307e60e1d`、`feat(autoware_carla_interface): ...(SUMO S1)`)
   - `git checkout feature/vissim_windows_co-sim -- <パス>` で、次の11ファイルを写した。
     - `carla_fleet.py`
     - `data/carla_fleet/`(規則・外形)
     - `tools/carla_fleet/`(計測・生成)
     - `vissim_integration/__init__.py`・`inpx_models.py`
     - `test/test_carla_fleet.py`・`test_build_carla_variants.py`・`test/fixtures/carla_fleet/`
   - `package.xml` に `python3-yaml` の1行を足した。Vissimブランチと同じ中身になった。
   - 確かめたこと:
     - 写したファイルと `package.xml` について、`git diff feature/vissim_windows_co-sim -- <パス>` が空。
     - 写したテストは80件成功、2件スキップ(Vissimの生成物を見るもの)。
     - `colcon build` が通った。
3. **以前からあったテストの失敗**(`5a5ea7f48`、`test(autoware_carla_interface): ...`)
   - SUMOブランチの `sumo_warmup_catchup_stub_test.py` の `test_collect_gap_actors` が、S1の変更の前から失敗していた。変更を退避した状態でも同じように失敗することを確かめた。
   - 原因は、`carla.Vector3D` がfloat32で値を持つため、幅2.2が `2.200000047…` になり、`==` での比較が合わないこと。コードの誤りではない。
   - このテストは、S4で手を入れる関数(`collect_gap_actors`)のテストなので、`math.isclose` で比べるように直した。
   - 直したあとのSUMOブランチのテスト: 190件成功・3件スキップ。スクリプト形式の `sumo_*_test.py` 4本もすべて成功。

### S2: SUMOのvTypeとTown01の需要(CARLAリポジトリ)

- 2.2のとおり、`vissim_fleet_vtypes.rou.xml` を作り、`rou/Town01.rou.xml` の分布と `Town01.sumocfg` を直す。
  - 作るスクリプトを `Co-Simulation/Sumo/util/make_vissim_fleet_vtypes.py` に置く。入力は、Vissim構成の生成物(`git show` で取り出す)と、高さの表。
- SUMOだけで1時間分動かし、S0の2と比べる。
  - 大型車が長くなるので、交差点の詰まりやteleportが増えないかを見る。増えたら、5章の4で決める。
- `sumo-gui` で、乗用車・トラック・バスの長さを目で見る(任意)。

完了条件: 差し替えた需要で、SUMOだけで1時間動く。S0の2との比較が、この計画書にある。

#### 結果(S2、2026-11-16、CARLAリポジトリ `main` の `6a1c918`)

利用者が `~/CARLA` を `main` に切り替えた(5章の7)。

**作ったもの・変えたもの**(`Co-Simulation/Sumo/`)

- `util/make_vissim_fleet_vtypes.py`(新規):
  - `.inpx` から、Vissimのモデルの寸法(全セグメントを繋いだ車体全体。丸めない)と、車両タイプ内の割合を読む。
  - vTypeのファイルを作り、`--rou` の車両グループ(`vTypeDistribution`)の `vTypes`・`probabilities` だけを置き換える。
  - 改行コードを変えず、何度実行しても同じ結果になる(2回目の実行で `Town01.rou.xml` が変わらないことを確かめた)。
  - 入力は、`git show feature/vissim_windows:Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx` で取り出したもの(sha256 `2265ff82…`。Windows PCのものと同じ)。
- `examples/vissim_fleet_vtypes.rou.xml`(新規、生成物): 14のvType。
  - ID: `vissim.<モデル番号>.<短い名前>`(例: `vissim.1.fiat_500`、`vissim.31.c2_standard`)。
  - vClass: 乗用車 `passenger`、23・22 `truck`、31 `bus`、70 `motorcycle`。
- `examples/rou/Town01.rou.xml`: 5つの車両グループの中身を、0.1の表のとおりに置き換えた。
  - A_carは、10モデルをVissimの割合(0.06・0.05・0.18・0.11・0.14・0.05・0.11・0.07・0.18・0.05)にした。
  - 冒頭とグループの上のコメントを、今の中身に合わせて直した。flow・経路は変えていない。
- `examples/Town01.sumocfg`: `route-files` を `carlavtypes.rou.xml,vissim_fleet_vtypes.rou.xml,rou/Town01.rou.xml` にした。
  - このファイルは改行がCRLFなので、それを変えずに1行だけ直した。最初の書き換えでLFに変わってしまい、やり直した。

**SUMOだけで1時間**(S0の2と同じ条件)

| 項目 | S0(今まで) | S2(Vissimの車両構成) |
| ---- | ---------: | --------------------: |
| 読み込んだ車 / 投入した車 / 投入を待っている車 | 924 / 919 / 5 | 924 / 919 / 5 |
| 到着した車 | 841 | 826 |
| 終了時に走っている車 | 78 | 93 |
| teleport | 0 | 1(詰まり。3560秒、`in7_A.19`、車線 `12.0.00_2`) |
| 急ブレーキ | 0 | 1(1713秒、`in6_A.13`、交差点の中 `:77_1_0`) |
| 衝突 | 0 | 0 |
| 平均の所要時間 / 停止時間 / 時間損失 / 出発の遅れ [秒] | 180.6 / 84.3 / 118.4 / 1.8 | 190.6 / 93.3 / 128.4 / 2.3 |
| 平均の所要時間(グループごと)[秒] | 乗用車180.1、トラック175.8、トレーラー205.3、バス192.7、バイク169.6 | 乗用車190.8、トラック185.3、トレーラー224.1、バス191.1、バイク174.1 |

**乱数のシードを変えて比べた**(`--seed`。co-simが使うのは既定のシード)

| シード | 今まで: 平均の所要時間 / teleport / 急ブレーキ | Vissimの車両構成: 平均の所要時間 / teleport / 急ブレーキ |
| -----: | ---------------------------------------------- | -------------------------------------------------------- |
| 1 | 169.9秒 / 0 / 1 | 177.4秒 / 1 / 1 |
| 2 | 174.4秒 / 0 / 1 | 190.1秒 / 0 / 1 |
| 3 | 284.8秒 / 1 / 2 | 347.6秒 / 6 / 1 |

- 大型車が長くなった(トラック5.20 → 7.93 m、トレーラー7.94 → 5.96 m、バス10.27 → 12.135 m)ため、交差点が詰まりやすくなり、所要時間がどのシードでも4〜22%延びた。
- シード3は、今までの需要でも詰まりが起きる。新しい需要では、それが強く出る。
- 急ブレーキは、今までの需要でも出ていた。
- 既定のシードでは、teleportは3560秒の1件だけ。co-simの使い方(ウォームアップ600秒+走行600秒)の範囲では起きていない。
- **5章の4は、「(b) そのまま受け入れ、起動手順書に書く」に決まった**(利用者)。

**Town05・Town10HDへの影響**: なし。両方の `.sumocfg` は `vissim_fleet_vtypes.rou.xml` を読み込まず、`rou/Town05.rou.xml`・`rou/Town10HD.rou.xml` も変えていない。

### S3: SUMOの読み込みと生成物

- `sumo_integration/sumo_fleet.py`(新規): 移植元の `read_models` を元にする。変えるところは次の2つ。
  - 複数のroute ファイル(`.sumocfg` の `route-files`)を読む。
  - 生成物に書くため、モデルごとにvTypeのID(`vtypes`)と、vTypeが入っている車両グループを返す。
- 生成ツール `build_carla_variants.py` に `--sumocfg` を足す(`--inpx` と、どちらか1つ)。
  - `sources` に、`route-files` の各ファイル(パスとsha256。SUMOのファイルはLinuxにあるので、照合に使う)と、`--sumocfg-ref`(出どころの記録)を書く。
  - 書き出し(`render()`)は、Vissimと共通の形にする。モデルの行には、`file`・`types` の代わりに `vtypes` を書く。
- テスト: 合成した小さな `.sumocfg`・`.rou.xml` で、読み込み(複数ファイル、分布、flowの台数、寸法のまとめ)、生成、書き出しを確かめる。移植元の `test_sumo_fleet.py` の、登録簿に関わらない部分も持ってくる。
- Town01の生成物 `sumo_integration/data/carla_variants_sumo_town01.yaml` を作る。
  - 14モデルの割り当てが、Vissim構成の `carla_variants_town01.yaml` と同じ(ブループリント・`front_offset_m`・dL・dW)になることを、テストで確かめる。
  - `carlavtypes.rou.xml` のvType(割合0)は `skipped` に入る。

完了条件: テストが通る。生成物の割り当てが、Vissim構成と一致する。

#### 結果(S3、2026-11-16)

- **`sumo_integration/sumo_fleet.py`**(新規、`4c1d5f6ea`): 移植元の `read_models` を元にした。traci・CARLAは使わない。
  - `route_file_names(sumocfg)`・`route_files(sumocfg)`: `.sumocfg` の `route-files` を、`.sumocfg` のあるディレクトリから解決する。
  - `read_models(files)`: 複数のrouteファイルにまたがって読む。vTypeは寸法(鍵は小数3桁の `L<長さ>_W<幅>`)でまとめる。
    - L・Wは丸めない(割り当てのdL・dWを、Vissimのモデルと同じにするため)。
    - 割合は、flow(`number`・`vehsPerHour`・`perHour`・`probability`・`period`(`exp(...)` を含む))・vehicle・tripの台数に、vTypeDistributionの確率を掛けたもの。分布の中に書いたvTypeも読む。
    - モデルごとに、まとめたvTypeのID(`vtypes`)と、入っている車両グループ(`distributions`)を返す。
    - 歩行者のvTypeと、車両グループそのもののIDは、モデルにしない。
- **生成ツール `build_carla_variants.py`**(SUMOブランチ `4c1d5f6ea`、Vissimブランチ `37ce81b48`、両方で同じ中身):
  - `--sumocfg`・`--sumocfg-ref` を足した(`--inpx` とどちらか1つ)。`sumo_fleet` は、`--sumocfg` のときだけ読み込む(Vissimブランチにはないため)。
  - SUMOの生成物には、各モデルに `vtypes` を書く。`sources` には `sumocfg`(出どころとsha256)と、`route_files`(`.sumocfg` に書かれた名前ごとのsha256)を書く。
  - 生成物の中は、車両グループごとにまとめる。
  - **Vissimの出力は変わらない**: Vissimブランチの `carla_variants_town01.yaml` を直したツールで作り直すと、作り方のコメントを除いて同じだった。Vissimブランチの作り直しのテスト(`CARLA_TOWN01_INPX` を付けたもの)も通った。
- **Town01の生成物** `sumo_integration/data/carla_variants_sumo_town01.yaml`:
  - 入力: `~/CARLA/Co-Simulation/Sumo/examples/Town01.sumocfg`(CARLAリポジトリ `main` の `6a1c918`)。
  - 14モデルに割り当て、`skipped` は42モデル(`carlavtypes.rou.xml` の41種(自車の `prius` を含む)と、SUMOの既定のvType)。
  - **14モデルすべての割り当て(ブループリント・`front_offset_m`・dL・dW・method)が、Vissimの `carla_variants_town01.yaml` と完全に一致した。**
  - 100台あたりの体数はVissimと違う(`carlacola` 45.0、`mini.cooper_s_2021` 25.2、`audi.tt` 16.1、`tesla.model3` 13.3、`kawasaki.ninja` 10.0、`dodge.charger_2020` 7.7、`nissan.micra` 4.2、`nissan.patrol` 3.5)。車両グループどうしの比率がSUMO(乗用車70%)とVissim(60%)で違うためで、想定どおり(5章の1)。
- **テスト `test/test_sumo_fleet.py`**(10件、SUMOブランチだけ): 合成の `.sumocfg` と2つのrouteファイルで、次を確かめる。
  - 読み込み: 解決、寸法でのまとめ、割合、カテゴリ、分布、歩行者の除外、止まる条件。
  - 生成: `sources`・`vtypes`、重ね置き、入力の指定。
  - コミットしたTown01の生成物の割り当てが、Vissimと同じであること。
  - `~/CARLA` から作り直して一致すること。これは、`~/CARLA` のrouteファイルが生成物と同じsha256のときだけ動く。
- テスト: SUMOブランチは200件成功・3件スキップ。Vissimブランチは203件成功・2件スキップ。
- 自分の誤り(記録のため): テストの合成データで、最初は `long_van` を7.0 × 2.2 mにしていた。これは `carlacola` の幅が太い側の許容を0.01 m超えて割り当てられず、生成ツールが正しく止まった。合計の台数も67台と誤って書いていた(正しくは57台)。どちらもテストの側を直した。

### S4: ブリッジへの組み込み

- `sumo_integration/fleet_plan.py`(新規): 2.3・2.4のとおり。
  - 検査: Vissim構成の検査に、`route-files` のsha256と、走行中の寸法の照合を足す。
  - 引き方: vTypeのID → モデル。表にないIDなら `npc_model_unknown`、`skipped` のIDなら `variant_model_empty`(走行中)。
- `sumo_integration/bridge_helper.py`: `get_fleet_blueprints()`・`get_fleet_transforms()` をVissim構成と同じ形で足す(配置の計算は `carla_fleet.body_origins()`)。
  - SUMOの座標(`net offset` を引いて、yを反転)から前端を出すところは、今の `get_carla_transform()` と同じ式を、`extent` を0にして使う。
- `sumo_integration/simulation_synchronization.py`: Vissim構成のF5・F6と同じ形。`fleet` がなければ今の動作(vType = ブループリント)のまま。
- `ego_spawn_gate.py`: 重ね置きの車を、全体の外形で見る。
- `carla_autoware.py`・`carla_ros.py`・launch: `sumo_carla_variants` と、`main()` での `VariantError` の処理。起動時の検査は、SUMOを起動する前(`_init_sumo_integration()` の最初)に行う。
- テスト: Vissim構成の `test_vissim_fleet_plan.py` と同じ項目を、SUMOの形で書く(`test_sumo_fleet_plan.py`)。今の `sumo_*_test.py` が通ること。

完了条件: 全テストが通る。`colcon build` が通る。

#### 結果(S4、2026-11-16)

- **`sumo_integration/fleet_plan.py`**(新規): Vissim構成の `FleetPlan` と同じ作りで、引き方だけをvTypeのIDにした。
  - 起動時の検査は、Vissim構成と同じもの(生成物がない、ブループリントがない、実メッシュ・規則の変更、連結車、CARLAの版、同じvTypeが2つのモデルにある、`vtypes` がない)に、次の2つを足した。
    - 渡された `.sumocfg` の `route-files` が、生成時と同じ並び・同じsha256か(`variant_source_changed`、`source: route_files`)。
    - `route-files` を `sumo_fleet.read_models()` で読み直し、`carla_fleet.check_models()` でモデルの寸法と、割合 > 0 のモデルに割り当てがあるかを見る。
  - 走行中: vType IDからモデルを引く。表にないIDは `npc_model_unknown`、`skipped` のIDは `variant_model_empty`。spawnのときは、その車の寸法(`extent` × 2)がモデルのL・Wと合うかも見て、合わなければ `variant_source_changed`(`source: vehicle_dimensions`)で止める。
  - `resolve_variants_path(value, sumo_cfg_file)`: `auto`(既定)は `sumo_integration/data/carla_variants_sumo_<.sumocfgの名前を小文字>.yaml` があれば使い、なければ今の選び方。`legacy_vtypes` は今の選び方。それ以外はファイルのパス(相対パスは `sumo_integration/data/` から)。
- **`bridge_helper.py`**: `get_fleet_blueprints(fleet_plan, sumo_id, sumo_actor, sync_color)` と `get_fleet_transforms(fleet_plan, sumo_transform, actors)` を足した。
  - 色と運転手は、SUMOのIDを種にした乱数で選ぶ(同じ車の体は同じ色)。`sync_vehicle_color` のときはSUMOの色。`role_name` は今までどおり `sumo_driver`。
  - 配置は、`get_carla_transform(transform, extent=0)` で前端を出し、`carla_fleet.body_origins()` で実メッシュの中心に置く。SUMOの長さ(`extent`)は使わない。
- **`simulation_synchronization.py`**: Vissim構成のF5・F6と同じ形。
  - 引数 `fleet_plan`(なければ今の動作)。spawn(`sync_sumo_to_carla`、ウォームアップ後の `spawn_all_sumo_actors_in_carla`)・更新・削除・`close` で、2体目以降(`sumo2carla_extra_ids`)も扱う。
  - 2体目以降は、車の真下の地下(−200 m から10 mずつ下)にspawnしてから、同じフレームで並べる。spawnできなければ、その車の体をすべて消して写さない。
  - `sync_carla_to_sumo` は `mirrored_carla_ids()`(全部の体)を除いてCARLA由来の車を探す。2体目以降がSUMOに取り込まれることはない。
- **`ego_spawn_gate.py`** `collect_gap_actors`: 重ね置きの車は、SUMOの前端から最後の体の後端までの長さ、いちばん広い体の幅で見る(バスは12.135 m)。
- **`carla_autoware.py`・`carla_ros.py`・launch**: パラメータ `sumo_carla_variants`(既定 `auto`)。
  - `_init_sumo_integration()` の最初、SUMOを起動する前に `FleetPlan` を作る。どの生成物を使うか(または今の選び方か、その理由)を1行出す。
  - `main()`: 起動時・走行中の `VariantError` を受け、`Error: CARLA blueprint assignment: <コード>: <根拠>` を1行出して、終了コード1で終わる(Vissim構成と同じ)。
  - 後片付け: 体の数の要約(`CARLA bodies spawned for SUMO vehicles: ...`)を出し、全部の体を消す。
- **テスト `test/test_sumo_fleet_plan.py`**(32件): Vissim構成の `test_vissim_fleet_plan.py` と同じ項目を、SUMOの形で書いた。SUMO特有のものとして、`route-files` の照合(中身の変更・並びの変更、`~/CARLA` のTown01があるときだけ)、車の寸法の照合、`auto` の解決、`sync_vehicle_color` の色、Vissimと同じ割り当てを足した。
- テスト: SUMOブランチは232件成功・3件スキップ。スクリプト形式の `sumo_*_test.py`・`pedestrian_sync_stub_test.py` もすべて通った。`colcon build` も通り、生成物がインストール先に入ることを確かめた。
- 実際の Town01(`~/CARLA` `main` の `6a1c918`)の `.sumocfg` を渡して、`FleetPlan` の起動時の検査が通ることを確かめた(CARLAなし、ブループリントの一覧は生成物から)。

### S5: ブリッジ単体での実機確認(Autowareなし)

Vissim計画のF7と同じ項目。SUMOはLinuxで動くので、すべて1台で行える。

1. 起動時の検査: 規則の写しを1文字変えて、`variant_source_changed` で止まる(SUMOを起動しない)。`.rou.xml` の写しを変えたときも止まる。
2. 走行中: エラー0、終了時の要約の比率が見込みどおり、`role_name` が `sumo_driver` とEGOだけ、地下に残った体がない、重ね置きの体が並ぶ。
3. 見た目: バス・トラック・トラクターの撮影。
4. ウォームアップあり(`sumo_warmup_time`): 一括spawnで重ね置きも出る。
5. Town05で、`sumo_carla_variants` が今の選び方で動く(5章の3)。
6. S0の3で洗い出した値(スポーン地点、ウォームアップ、信号の確認)のうち、長さが変わって影響が出そうなものを確かめ直す。

完了条件: 1〜6の結果が、この計画書にある。

### S6: Autoware全体での実機確認

- Vissim計画のF8と同じ記録・集計スクリプト(scratchpad)で、知覚した物体の大きさを記録する。
- 比べるもの: (a) SUMOの新しい構成、(b) SUMOの今の構成(`legacy_vtypes`)、(c) Vissim計画のF8の結果。
  (a)の大型車・乗用車の知覚した長さが、(c)と同じ程度にVissimのモデルに近いことを確かめる。
- EGOが目的地まで走ること、信号で止まること(今の起動手順書の確認済みの地点)。
- 止め方は、ブリッジだけを先にSIGINTで止める(Vissim計画のF8)。

完了条件: 比べた結果が、この計画書にある。

### S7: ドキュメントとコミット

- SUMOの起動手順書: `sumo_carla_variants`、生成物の作り直し、止まったときの `code`、Ctrl+Cの止め方。Town01の車両構成が変わったこと。
- README: 「SUMO Vehicle Blueprints」の節(または、Vissimの節と1つにまとめる。マージの都合で、Vissimブランチの文と同じにする)。
- CARLAリポジトリ: SUMOの起動手順書(`Co-Simulation/Sumo/docs/`)に、Town01の車両構成を書く。
- 計画書(本書)に、各タスクの結果を書く。
- コミット: autoware_universeは `feat(autoware_carla_interface): ...` / `docs(sumo-carla-autoware co-sim): ...`、CARLAは今のブランチの書き方。日本語。pushはしない。

---

## 5. 決めること(作業の前か途中)

| #   | 内容 | 推奨 | 決める時期 |
| --- | ---- | ---- | ---------- |
| 1   | 車両グループどうしの比率 | 今のSUMOの比率(乗用車70%・トラック10%・トレーラー5%・バス5%・バイク10%)を保つ。グループの中身だけをVissimのモデルにする。Vissimの比率(60・10・10・10・10%)に合わせると、交通量と信号の確認をやり直すことになる | S2の前 |
| 2   | vTypeの追加の仕方 | 新しいファイル `vissim_fleet_vtypes.rou.xml` を足し、`Town01.rou.xml` の分布の中身を差し替える。`carlavtypes.rou.xml` は残す(自車とTown05・Town10HDで使う) | S2 |
| 3   | `sumo_carla_variants` の既定値 | `auto`: `.sumocfg` の名前から `sumo_integration/data/carla_variants_sumo_<名前を小文字>.yaml` を探し、あればそれを使う。なければ今の選び方を使い、そのことをログに1行出す。Town01だけ生成物を作るので、Town05・Town10HDは今までどおり動く。`legacy_vtypes` の明示と、ファイルのパスの直接指定もできる | S4 |
| 4   | 大型車が長くなって詰まりが増えたとき | 決定(S2、2026-11-16、利用者): (b) そのまま受け入れ、起動手順書に書く。S2の比較では、所要時間が4〜22%延び、既定のシードではteleportが3560秒に1件だけだった(co-simの範囲では起きない)。(a) バス・トレーラーのflowを減らす、はあとからでもできる | S2 |
| 5   | vTypeの挙動パラメータ | vClassの既定値のまま(今の `carlavtypes.rou.xml` も既定値) | S2 |
| 6   | 共通部品の置き場所 | 配置の計算を `carla_fleet.py` に移す(S1の1)。`FleetPlan` は交通シムごとに分ける(引き方が違うため) | S1 |
| 7   | CARLAリポジトリのブランチ(S0で判明) | SUMOのシナリオは `main` にあり、今の作業ツリーは `feature/vissim_windows`。SUMOの作業(S2の変更、S5・S6の実機)をするときは、利用者が `~/CARLA` を `main` に切り替える。SUMOの起動手順書の `sumo_cfg_file` は `~/CARLA/Co-Simulation/Sumo/examples/` を指すので、実機のときも `main` である必要がある。2つのブランチを行き来するのが手間なら、`main` を別の場所に `git worktree add` する案もある(その場合、起動手順書のパスも変わる) | S2の前 |

---

## 6. リスクと対応

| リスク | 対応 |
| ------ | ---- |
| 大型車が長くなり、Town01の交差点で詰まる。今の起動手順書の確認済みの値(スポーン地点、ウォームアップ)が合わなくなる | S0の2・3で今の値を記録し、S2・S5で比べる。合わなければ5章の4で決める |
| 2つのブランチの共通部品が食い違う | S1で `git checkout` で写し、`git diff` で同じ中身であることを確かめる。共通部品を直すときは、先にVissimブランチで直して写す |
| SUMOブランチにVissimのモジュール(`inpx_models.py`)が入る | 中身はVissimブランチと同じなので、マージで衝突しない。生成ツールの `--inpx` のためだけに使う |
| TraCIが返すvType IDが、分布のIDになる(メンバーのIDでない) | S0の1で確かめる。分布のIDが返るなら、寸法で引く(移植元と同じ)に切り替える |
| Town05・Town10HDで、新しい仕組みが誤って働く | `auto` は生成物があるときだけ使う。S5の5で、Town05が今の選び方で動くことを確かめる |
| 自車のvType(`carlavtypes.rou.xml` の `vehicle.toyota.prius`)が、生成物に割り当てのないモデルとして引かれる | 自車はCARLA→SUMOの向きで、`sumo2carla` の引きには来ない。`skipped` に入るだけ。S5で、自車が今までどおり動くことを確かめる |
