# SUMO-CARLA-Autoware: Vissimの.inpxからSUMOの信号・需要・寸法を作る(Town10HD) 実装計画 v1.0

作成日: 2026-11-24

対象ブランチ:

| リポジトリ | ブランチ | 主に変えるもの |
| ---------- | -------- | -------------- |
| autoware_universe(`~/autoware.1.9.0/src/universe/autoware_universe`) | `feature/sumo_co-sim` | `tools/sumo_inpx/`(新規)、`sumo/`の生成物と`.sumocfg`、ブループリント表、テスト、`sumo/README.md`、起動手順書 |

作業は worktree `~/worktrees/autoware_universe_sumo`(`feature/sumo_co-sim`)で行う。
`~/autoware.1.9.0`の中に置くと、colconが同じ名前のパッケージを2つ見つけてビルドが止まるため、ワークスペースの外に置いた。

参考:

- 移植元の束 `sumo_inpx_port_20261124.tar.gz`(SBIR_sim `feat/carla-fleet-class-width` の `dedb96c`)。
  とくに `docs/SUMO構成_inpx変換_CARLA0.9環境への移植ガイド.md`(本書では「ガイド§5」のように書く)と、
  `sumo/vissim_to_rou.py`・`sumo/vissim_to_phases.py`・`sumo/gen_tll.py`。
- `vissim/README.md`(Town10HD.inpxの作り方。座標、信号灯の置き方、Vissimで直した車両設定)。
- `docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md`(SUMOの.sumocfgからブループリント表を作る仕組み)。

---

## 0. 目的・スコープ

SUMO構成のTown10HDで、**Vissim構成のTown10HD(`vissim/Town10HD.inpx`)と同じ交通条件でSUMOを走らせる**。
inpxから次の3つを作る。

| inpxから取るもの | 入る先 |
| ---------------- | ------ |
| 信号制御(固定時間の信号プログラム、5つ) | 追加ファイル `.tll.xml`(新しい信号プログラムとして読ませる) |
| 交通需要(流入・車種構成・希望速度・追従モデル・加減速) | `.rou.xml` |
| 車両寸法(inpxの3Dモデルの長さ・幅。Vissimで日本の車に直したもの) | `.rou.xml`のvType |

**道路網(`sumo/net/Town10HD.net.xml`)は作り直さない。** CARLAのOpenDRIVEからnetconvertで作った今の網をそのまま使う。

### 0.1 目標の構成

```
vissim/Town10HD.inpx ─────────────┬──────────────────────────────┐
                                  │ tools/sumo_inpx/vissim_to_tll.py │ tools/sumo_inpx/vissim_to_rou.py
sumo/net/Town10HD.net.xml ────────┤ (信号灯の名前 CARLA <id>          │ (流入リンク -> edge、分岐ごとに等分の
                                  │  -> linkSignalID -> linkIndex)    │  経路、vType・vTypeDistribution)
                                  ▼                                   ▼
               sumo/tll/Town10HD_vissim.tll.xml        sumo/rou/Town10HD_vissim.rou.xml
                                  └──────────────┬────────────────────┘
                                  sumo/Town10HD_vissim.sumocfg
                                                 │ build_carla_variants.py --sumocfg(0.9.16_jpの規則)
                                                 ▼
            sumo_integration/data/carla_variants_sumo_town10hd_vissim.yaml
                                                 │
            co-sim: sumo_cfg_file:=Town10HD_vissim.sumocfg(ブループリント表は auto で選ばれる)
```

### 0.2 スコープ外

- 道路網の作り直し(移植元の`gen_net.py`・`tl_extract.py`・`fold_junction.py`。lanelet2から網を作る手順)。
- 横断歩道・歩行者(`add_crosswalk.py`、`personFlow`)。Town10HD.inpxには歩行者の流入が無い(1章)。
- お台場の網(CARLA 0.9にお台場の地図が無い。ガイド§5の6)。
- Town01・Town05、今の`Town10HD.sumocfg`。今のまま動くようにする(新しい`.sumocfg`を足すだけ)。
- ブリッジ(`sumo_integration/`)の変更。今の仕組み(信号の同期、ブループリント割り当て)で動く見込み(2.4)。

---

## 1. 事前調査で分かったこと(2026-11-24)

ガイド§5の8項目:

| # | 項目 | 結果 |
| - | ---- | ---- |
| 1 | 車をCARLAに出す仕組み | CARLA標準のco-simを取り込んで手を加えたブリッジ(`sumo_integration/`)。寸法から割り当てるブループリント表(`carla_variants`)が入っている |
| 2 | 歩行者 | `personFlow`はSUMOからCARLAのwalkerへ片方向で同期している |
| 3 | 信号 | `tls_manager=sumo`でCARLAの信号機を凍結し、SUMOの色を設定する(`linkSignalID`でCARLAのlandmarkと対応)。AutowareへはCARLAの信号機の色を配信する(`traffic_light.publish`) |
| 4 | SUMOの網 | CARLAのxodrから作った`sumo/net/Town{01,05,10HD}.net.xml` |
| 5 | 座標 | SUMO = OpenDRIVE + netOffset(Town10HDは(109.34, 135.96))。Vissim = OpenDRIVE(`vissim/README.md`) |
| 6 | CARLAのお台場の地図 | 無い → Town10HDで作ることにした(利用者、2026-11-24) |
| 7 | SUMOの版・ステップ | 手元は1.20.0(移植元は1.27.1+EIDMの修正)。ステップ0.1 s。Townの網に`--lefthand`は無い(右側通行の地図) |
| 8 | 自車のブループリント | `vehicle.toyota.prius`(role名`ego_vehicle`) |

`vissim/Town10HD.inpx`の中身(移植元のスクリプトとの違い):

| 項目 | Town10HD.inpx | 移植元のスクリプト | 対応 |
| ---- | ------------- | ------------------ | ---- |
| 信号制御 | 5つ(SUMOの信号1つにつき1つ)。**ステージ方式**(`stageProg`)。周期94 s(青42・黄3・全赤2 ×2) | 1つ、信号グループ方式(`<prog>`)だけ | ステージを信号グループごとの色の時刻に展開する |
| 信号灯 | 30個。名前が`CARLA <信号ID>`、信号グループは`sg="<制御> <グループ>"` | 地図の信号のgroupと`signal_groups.json` | 信号灯の名前 → CARLAの信号ID → SUMOの`linkSignalID` → linkIndex で対応が決まる。地図は使わない |
| 経路 | `vehicleRouteStatic`が**無い**(車はコネクタを選びながら走る) | 静的経路をなぞる | 分岐ごとに等分(2.3、利用者が決めた) |
| 歩行者 | 流入が無い(`area` 74個は取り込みで付いたもの) | 待ち場所 → 横断歩道 | 持ってこない |
| 流入 | 19本、合計600台/h、時間帯1つ、STOCHASTIC | 同じ | 同じ規則 |
| 車両構成 | 1つ: 乗用車0.5・トラック0.1・トレーラー0.1・バス0.2・バイク0.1、希望速度は分布30(30〜35 km/h) | 同じ | 同じ規則。**バイクは割合0にしない**(2.2) |
| 3Dモデル | アルファード4.95×1.85、ノア4.71×1.736、JPNタクシー4.4×1.696、GIGA 11.81×2.49、Volvo E 5.96×2.55、エルガ10.43×2.484、ガーラ11.99×2.49、MT-07 2.098×0.907 m | 同じ | 同じ規則 |
| 運転挙動 | 全リンクが1「Urban (motorized)」、Wiedemann 74、ax 2 | 同じ | 同じ規則 |

SUMOの網(`sumo/net/Town10HD.net.xml`):

- 車の通る edge は28本。**行き止まりが無い**(どのedgeにも次のedgeがある)。
  分岐ごとに等分した経路を数え上げると、長さ400 mまでで1692本、600 mまでで2万本、800 mまでで23万本になり、
  経路の割合(routeDistribution)として書き出せない(2.3)。
- 信号5つ(189・23・468・532・719)。全linkIndexに`linkSignalID`があり、CARLAの信号15個に対応する。
  今の信号プログラムは周期90 s(青42・黄3 ×2、全赤なし)。
- Vissimのリンク名(`Road 10-0-Right`)とSUMOのedge ID(`-3`)は一致しない(netconvertが道路をつないでいる)。
  対応は幾何で取る(2.1)。

---

## 2. 方針

### 2.1 inpxの読み取りと、網との対応(S1)

- 移植元の`vissim_to_rou.py`の`Vissim`クラスと`vissim/net.py`の読み取りを、`tools/sumo_inpx/inpx_reader.py`にまとめる。
  `sbir_env`・`received_networks`・`transform.json`は使わない(座標はnetOffsetだけで合う)。
- Vissimのリンク → SUMOのedge: SUMOのedgeの、車が走れる車線の中ほどの点が、リンクの幅(+0.5 m)の中にあり、
  向きが45°以内のedgeを選ぶ(`tools/vissim/build_vissim_town.py`の`sumo_edge_to_link`の逆。流入リンクはその対応で置いた)。
  1つに決まらないときは止める。
- 信号プログラムは、信号グループ方式とステージ方式の両方を読む。
  黄は「赤の命令の時刻で終わる」、赤黄は「青の命令の時刻で終わる」(移植元と同じ。Vissimの.lsaで確かめた規則)。

### 2.2 需要と寸法(S3)

移植元の`vissim_to_rou.py`の規則をそのまま使う。

| Vissim(inpx) | `.rou.xml` |
| ------------ | ---------- |
| 流入 vehicleInput | 流入リンクに対応するedgeから出す。到着はポアソン(指数分布の間隔) |
| 車種構成 | `vTypeDistribution`の確率(車種 × 3Dモデルの割合 × 希望速度の区間) |
| 車種 | vClass: passenger / truck / trailer / bus / motorcycle |
| 寸法 | inpxの3Dモデルの長さ・幅・高さ |
| 希望速度(区分線形の分布) | 区間ごとのvType。`speedFactor="normc(...)"` × `desiredMaxSpeed="1"`(地図の制限速度に縛られない) |
| 運転挙動(Wiedemann 74) | `carFollowModel="Wiedemann"`、`minGap = w74ax` |
| 加減速 | accel = 希望加速度の0〜60 km/hの平均、decel = 希望減速度(50 km/h)、emergencyDecel = 最大減速度(50 km/h) |

- **バイク(MT-07)は割合0.1のまま残す。** 移植元はCARLA 0.10に二輪の体が無いので0にしていたが、CARLA 0.9には二輪の
  ブループリントがあり、Vissim構成のTown10HDも0.1のため(ガイド§7)。ブループリント表で体が決まらないときはS4で見直す。
- トレーラーは移植元と同じく`guiShape="truck"`(連結の形にしない)。
- 出力の先頭に、inpx・網・ツールのsha256を書く(日時は書かない。同じ入力から同じバイト列)。

### 2.3 経路: 分岐ごとに等分(S3)

Vissimの車は経路を持たず、分岐でコネクタを選びながら走る。SUMOでは**車1台ごとに、分岐ごとに次のedgeを等しい確率で選んだ経路**を
持たせる(利用者が決めた、2026-11-24)。

- 網に行き止まりが無いため、経路の数え上げ(routeDistribution)はできない(1章)。そこで、車1台ずつ`<vehicle>`と経路を書く。
  - 出発時刻: 流入ごとに、流量のポアソン到着(指数分布の間隔)で`--horizon`秒まで。乱数は`--seed`で固定する。
  - 経路: 流入のedgeから、次のedge(車が通れるconnectionのあるedge)を等しい確率で選び、長さが`--route-length`に
    達したedgeの終わりで消える。
- Vissimの車は網から出ない(閉じた網を回り続ける)が、SUMOでは経路の終わりで消える。この違いは`.rou.xml`の先頭に書く。
- 出発車線は`departLane="random"`(Vissimの流入は車線を選ばない)、出発速度は`departSpeed="max"`(移植元と同じ)。
- 既定値: `--horizon 3600`、`--route-length 2000`、`--seed 1`(5章の5、S3の結果)。

### 2.4 信号(S2)

- 信号制御ごとに、信号灯の名前から CARLAの信号ID → (信号制御, 信号グループ) を取り、SUMOの`linkSignalID`で
  CARLAの信号ID → (tlLogic, linkIndex) を取る。1つの信号制御のCARLAの信号が、別々のtlLogicにまたがるときは止める。
- 全信号グループの色が変わる時刻で区切り、区間ごとに1つの`<phase>`を作る。色: 青 `G`/`g`、黄 `y`、赤 `r`、赤黄 `u`。
- 許容(`g`)と優先(`G`)は、移植元の`gen_tll.py`と同じく手で決めない。**今の網の信号プログラムで、同じlinkIndexが青のときの
  文字を写す**(今の網はnetconvertが同じ青の組で`G`/`g`を決めたもの)。青の組が今のプログラムと違うときは止める。
- 書き出すのは追加ファイルの`<tlLogic programID="vissim">`。**今のプログラムの`linkSignalID`の`<param>`も写す**
  (ブリッジはプログラムごとの`param`からCARLAの信号機を引く。`sumo_simulation.py`の`SumoTLManager`)。
  SUMOは最後に読んだプログラムを使うので、`.sumocfg`の`additional-files`に入れれば切り替わる。netconvertでの焼き込みはしない。
- offsetは0(Vissimの`stageProg`のoffsetが0)。

### 2.5 起動の組とブループリント表(S4)

- `sumo/Town10HD_vissim.sumocfg`: net-file `net/Town10HD.net.xml`、route-files `rou/Town10HD_vissim.rou.xml`だけ
  (`carlavtypes.rou.xml`は入れない。ブループリント表のモデルに余計なvTypeが入らないように)、additional-files `tll/Town10HD_vissim.tll.xml`。
- ブループリント表: `build_carla_variants.py --sumocfg sumo/Town10HD_vissim.sumocfg`を、0.9.16_jp用の規則・実メッシュで作る
  (`docs/SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md`の手順)。出力は`carla_variants_sumo_town10hd_vissim.yaml`
  (`sumo_carla_variants:=auto`で`.sumocfg`の名前から選ばれる)。
- 網・ルートを作り直すと、今ある`variant_source_changed`の検査で起動時に止まる。ガイド§6.5のsha256照合の代わりにする。

---

## 3. タスク一覧

| ステップ | 内容 | コミット |
| -------- | ---- | -------- |
| S1 | inpxの読み取りと網との対応(`tools/sumo_inpx/inpx_reader.py`)、テスト | 1 |
| S2 | 信号プログラム → `sumo/tll/Town10HD_vissim.tll.xml`(`vissim_to_tll.py`)、テスト | 1 |
| S3 | 需要と寸法 → `sumo/rou/Town10HD_vissim.rou.xml`(`vissim_to_rou.py`)、テスト | 1 |
| S4 | `Town10HD_vissim.sumocfg`、ブループリント表、`sumo/README.md` | 1 |
| S5 | SUMO単体での確認(TraCI)、起動手順書への追記 | 1 |
| S6 | CARLA 0.9.16_jp・Autowareをつないだ確認(利用者) | 結果を本書に書く |

ステップごとにコミットする(pushしない)。

---

## 4. 各タスクの内容

### S1: inpxの読み取りと網との対応

- `tools/sumo_inpx/inpx_reader.py`: `Inpx`(リンク・3Dモデル・車種・車両構成・速度分布・加減速の関数・運転挙動・流入・信号制御・信号灯)、
  `signal_timeline`(信号プログラムの展開)、`carla_signal_groups`(信号灯の名前 → 信号グループ)、`link_to_edge`(リンク → edge)。
- `test/test_sumo_inpx.py`: 小さな入力での展開と読み取り、Town10HDでの流入19本の対応と信号5つの時刻。

#### 結果(S1、2026-11-24)

- 流入19本が、それぞれ別のSUMOのedge1本に対応した(例: `Road 10-0-Right` → `-3`、1車線の`Road 11-0-Right` → `-11`)。
- 5つの信号制御とも、周期94 s、信号グループ1は青0–42・黄42–45・赤45–94、グループ2は赤0–47・青47–89・黄89–92・赤92–94。
  `vissim/README.md`の2.2(青42・黄3・全赤2 ×2)と一致する。
- CARLAの信号15個が、それぞれ1つの(信号制御, 信号グループ)に対応した。
- `python3 -m pytest test/test_sumo_inpx.py`: 10件が通った。

### S2: 信号プログラム

- `tools/sumo_inpx/vissim_to_tll.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml --out sumo/tll/Town10HD_vissim.tll.xml`
- 期待値: tlLogic 5つ、各6フェーズ(42・3・2・42・3・2 s、計94 s)、`linkSignalID`は今の網と同じ。
- テスト: 作り直した結果がコミットしたファイルと同じ、フェーズの時刻、`G`/`g`が今の網と同じ。

#### 結果(S2、2026-11-24)

- `sumo/tll/Town10HD_vissim.tll.xml`: tlLogic 5つ(189・23・468・532・719 ← 信号制御1〜5)、各6フェーズ(42・3・2・42・3・2 s)。
  青と黄のフェーズは今の網のプログラム(青42・黄3 ×2)と同じ文字列で、全赤2 sを2回足した形になった。`linkSignalID`の`param`も今のプログラムと同じ。
- SUMO 1.20.0で網と一緒に読ませる(`sumo -n net/Town10HD.net.xml -a tll/Town10HD_vissim.tll.xml`)と、5つとも`vissim`のプログラムが使われ、
  TraCIで読んだプログラムごとの`param`(ブリッジが読むもの)も18・10・6・11・11個あった。
- tlLogic 189の色は、0 s 青 → 42.1 s 黄 → 45.1 s 全赤 → 47.1 s 横の青 → 89.1 s 黄 → 92.1 s 全赤 → 94.1 s 青、で切り替わった。
  0.1 s(1ステップ)遅れて見えるのはSUMOの動きで、今のプログラムでも同じ(42.1 sに黄)。
- `python3 -m pytest test/test_sumo_inpx.py`: 17件が通った(S2の分は7件)。

### S3: 需要と寸法

- `tools/sumo_inpx/vissim_to_rou.py --inpx vissim/Town10HD.inpx --net sumo/net/Town10HD.net.xml --out sumo/rou/Town10HD_vissim.rou.xml`
  (`--report`で書き出さずに流入と割合を表示)
- 期待値: 流入19本、合計600台/h(1時間で約600台)。vTypeの寸法がinpxの3Dモデルと同じ。車種の割合が0.5・0.1・0.1・0.2・0.1。
- テスト: 同じ`--seed`で同じバイト列、経路がつながっている(各edgeの次が網のconnectionにある)、割合。

#### 結果(S3、2026-11-24)

- `sumo/rou/Town10HD_vissim.rou.xml`: 流入19本(合計600台/h)から、0〜3600 sに582台(`--seed 1`)。
  流入リンク → edgeの対応はS1のとおり。流入リンクの始点がedgeの途中にあるもの(例: リンク3 → edge `4`の47.89 m)は、そこから出す。
- vTypeは8つ。乗用車(アルファード4.95×1.85・ノア4.71×1.736・JPNタクシー4.4×1.696、各1/6)、トラック(GIGA 11.81×2.49)0.1、
  トレーラー(Volvo E 5.96×2.55、`guiShape="truck"`)0.1、バス(エルガ10.43×2.484・ガーラ11.99×2.49)0.2、バイク(MT-07 2.098×0.907)0.1。
  希望速度は30〜35 km/h(分布30)、`minGap` 2 m、accel 2.062(乗用車・バイク)・1.823(トラック・トレーラー)・1.137 m/s²(バス)。
- 経路は1台あたり20〜47本のedge(中央32本)。Town10HDのedgeは短い(10〜50 mが多い)ので、2000 mで交差点を約30回通る。
- **SUMO単体で流すと、時間がたつと詰まる**(S2の信号と一緒に、SUMO 1.20.0、ステップ0.1 s):

  | 時刻 [s] | 50 | 70 | 100 | 300 | 600 | 700 | 860〜 |
  | -------- | -: | -: | --: | --: | --: | --: | ----- |
  | 網の中の台数 | 9 | 11 | 14 | 51 | 105 | 129 | テレポートが出始める |
  | 平均速度 [m/s] | 6.1 | 3.8 | 6.6 | 5.3 | 1.3 | 0.8 | 0.6以下 |

  - 最初の100 sの台数は、Vissimのウォームアップのログ(50 sで10台、70 sで11台、90 sで15台。2026-11-24のco-sim)と合う。
  - Vissimの網もSUMOの網も閉じている(どのリンクの先にもコネクタがある)ので、600台/hを流し続けると台数が増え続ける。
    Vissimの車は網から出ないので、Vissimでも同じように溜まるはず(Vissimには優先の決まっていないconflictAreaが86個あり、そこでは譲り合わずに通るので、SUMOより流れが良いかもしれない。未確認)。
  - 1時間流すと、582台のうち入れたのは367台、着いたのは2台、テレポート509回(Jam 263・Yield 206・Wrong Lane 40)。
  - 次のどれを変えても詰まりは解けなかった(40分流して比べた): 経路の長さ(1000 mで960 s、500 mで1655 sから詰まる)、
    Krauss、今の網の信号(全赤なし)、`impatience="1"`、`--ignore-junction-blocker 5`、`departLane="best"`。
  - 今のco-simの1回(ウォームアップ100 s + 600 s)の中では、テレポートは出ない。ただし後半はかなり遅くなる。
  - **経路の長さは2000 mのままにする**(利用者、2026-11-24)。網の中の台数をVissimに近いまま保ち、後半に混むのはVissimと同じ条件の結果として受け入れる。
    経路の長さは`--route-length`で変えられるので、Vissimの様子に合わせて作り直せる。
- 今の`Town10HD.sumocfg`(0〜199 sに200台)は、1500 sまで流しても詰まらずに全車が着いた(比べるために流した)。
- `python3 -m pytest test/test_sumo_inpx.py`: 22件が通った(S3の分は5件)。

### S4: 起動の組とブループリント表

- `sumo/Town10HD_vissim.sumocfg`、`sumo/README.md`の表に追記。
- `build_carla_variants.py --sumocfg`で`carla_variants_sumo_town10hd_vissim.yaml`を作り、モデルごとの体を確かめる
  (とくにバイク・トレーラー・バス)。

#### 結果(S4、2026-11-24)

- `sumo/Town10HD_vissim.sumocfg`: net `net/Town10HD.net.xml`、route-files `rou/Town10HD_vissim.rou.xml`だけ、additional-files `tll/Town10HD_vissim.tll.xml`。
  EGOのvTypeは、`carlavtypes.rou.xml`が無くても、ブリッジがCARLAの寸法から作る(`bridge_helper.py`の`_create_sumo_vtype`)。
- `carla_variants_sumo_town10hd_vissim.yaml`(0.9.16_jpの計測結果と規則): 8モデルすべてに割り当てがあり、**Vissim構成の`carla_variants_town10hd.yaml`と同じ**
  (ブループリント・dL・dW・割合まで同じ)。

  | vType | CARLAの体 | dL [m] | dW [m] | 割合 |
  | ----- | --------- | -----: | -----: | ---: |
  | 乗用車 アルファード(1001) | `vehicle.jp.alphard` | +0.021 | +0.310 | 0.1667 |
  | 乗用車 ノア(1002) | `vehicle.jp.minivan` | +0.015 | +0.344 | 0.1667 |
  | 乗用車 JPNタクシー(1003) | `vehicle.jp.taxi` | −0.001 | +0.176 | 0.1667 |
  | トラック GIGA(1021) | `vehicle.jp.large_truck` | +0.361 | +0.374 | 0.1 |
  | トレーラー Volvo E(22) | `carlacola` × 2 | 0.000 | +0.009 | 0.1 |
  | バス エルガ(1031) | `vehicle.jp.route_bus` | +0.269 | +0.396 | 0.1 |
  | バス ガーラ(1032) | `vehicle.jp.tour_bus` | +0.119 | +0.350 | 0.1 |
  | バイク MT-07(70) | `vehicle.kawasaki.ninja` | −0.081 | −0.032 | 0.1 |

- `sumo_carla_variants:=auto`で、`Town10HD_vissim.sumocfg`からこの表が選ばれ、起動時の検査(ブループリントの有無・計測結果と規則の照合・
  route-filesのsha256・CARLAの版`294096eb1-dirty`)を通ることを、`test/test_sumo_fleet_plan.py`に足したテストで確かめた。
- `sumo/README.md`に、ファイルの表と作り直しの手順を書いた。
- `python3 -m pytest test/`: 293件が通った(3件skip)。

### S5: SUMO単体での確認と手順書

- `sumo -c sumo/Town10HD_vissim.sumocfg`をTraCIで動かし、各信号の`getRedYellowGreenState`が0・42・45・47・89・92 sで切り替わること、
  1時間の出発台数(約600台)、テレポート・衝突の数を見る。
- `docs/SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md`に、生成の手順と`sumo_cfg_file:=Town10HD_vissim.sumocfg`での起動を書く。

#### 結果(S5、2026-11-24)

- `sumo -c sumo/Town10HD_vissim.sumocfg`(SUMO 1.20.0、ステップ0.1 s)をTraCIで700 s動かした。
  - 信号5つとも`vissim`のプログラムで、42.1・45.1・47.1・89.1・92.1・94.1 s(以降94 sごと)に切り替わった(0.1 s遅れはSUMOの動き。S2)。
  - 網の中の台数と平均速度: 100 sで14台・6.6 m/s、300 sで51台・5.3 m/s、500 sで86台・1.8 m/s、700 sで129台・0.8 m/s。テレポート・衝突なし。
- `colcon build --packages-select autoware_carla_interface --symlink-install`のあと、インストール先の`share/autoware_carla_interface/sumo/`に
  `Town10HD_vissim.sumocfg`・`tll/`・`rou/`のファイルが入り、`sumo_cfg_file:=Town10HD_vissim.sumocfg`がそこへ解決され、
  `sumo_carla_variants:=auto`で`carla_variants_sumo_town10hd_vissim.yaml`が選ばれることを確かめた。
- `docs/SUMO-CARLA(0.9.16)-Autoware_co-sim_起動手順.md`: 2.2.3の表に`Town10HD_vissim`の行、2.4.1に起動コマンド(`sumo_warmup_time:=100`)、
  3.8にシナリオの説明を足した。

### S6: CARLA・Autowareでの確認(利用者)

- CARLA 0.9.16_jpのTown10HD_Optで起動し、CARLAの信号機とAutowareが受け取る色が、SUMOの信号と同じ時刻で切り替わることを見る。
- 車の体(ブループリント)と寸法が合っていること、二輪が出ることを見る。

---

## 5. 決めること(作業の前か途中)

| # | 決めること | 案 | 決めた結果 |
| - | ---------- | -- | ---------- |
| 1 | 移植の対象 | お台場 / Town10HD | **Town10HD**(利用者、2026-11-24) |
| 2 | 経路 | 分岐ごとに等分 / 今の`Town10HD.rou.xml`の経路 | **分岐ごとに等分**(利用者、2026-11-24) |
| 3 | 信号の周期 | Vissimの94 s(全赤あり)/ 今のSUMOの90 s | **94 s**(Vissimと同じ条件にするのが目的のため) |
| 4 | バイクの割合 | 0.1のまま / 0(移植元と同じ) | **0.1のまま**(Vissim構成のTown10HDと同じ) |
| 5 | 経路の長さと書き出す時間 | `--route-length 2000`、`--horizon 3600` | **2000 m・3600 sのまま**(利用者、2026-11-24。S3の結果。co-simの後半に混むのはVissimと同じ条件の結果として受け入れる) |

---

## 6. リスクと対応

| リスク | 対応 |
| ------ | ---- |
| SUMOの版の違い(手元1.20.0、移植元1.27.1)で、Wiedemannの挙動が移植元と違う | 移植元の生成物とは比べない(網も違う)。S5で詰まり・テレポートを見る |
| 閉じた網で、Vissimの車は回り続けるが、SUMOの車は経路の終わりで消える | `.rou.xml`の先頭に書いた。どちらも台数が増え続け、SUMOでは700 sで平均0.8 m/sまで落ちる(S3)。Vissimの流れ方はまだ見ていない |
| 等分の経路が、SUMOで車線変更できずに詰まる(分岐の手前の車線が足りない) | S5でテレポートの数を見る。多いときは経路の選び方を車線のconnectionで重みづける案を出す |
| 追加ファイルの信号プログラムに`linkSignalID`が無いと、CARLAの信号機が動かない | 今のプログラムの`param`を写す。S2のテストで確かめる |
| Town10HD.inpxをVissimで直したとき(車両設定)、生成物が古いまま | 生成物の先頭のinpxのsha256と、ブループリント表の`variant_source_changed`で気づける。作り直しの手順を`sumo/README.md`に書く |
