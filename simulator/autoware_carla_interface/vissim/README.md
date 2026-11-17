# Vissimのネットワーク(Vissim-CARLA-Autoware co-sim)

Vissim(Windows)で読み込む`.inpx`を置く。CARLAの版(0.9.15・0.9.16)によらず、ここのものを使う。

| ファイル | 内容 | 作り方 |
| --- | --- | --- |
| `Town10HD_road.inpx` | Town10HDの道路網だけ(リンク・コネクタ)。信号・交通需要・車両設定はない | 1章(Windows PCのVissimでOpenDRIVEを取り込む) |
| `Town10HD.inpx` | co-simで使うTown10HDのネットワーク。道路網に、信号・交通需要・車両設定を足したもの | 2章(ツールで作る) |
| `signal_mapping_town10hd.json` | Vissimの(信号制御, 信号グループ) → CARLAの信号ID。Vissimブリッジの`data/signal_mapping.json`と同じ形 | 2章(同じツールで作る) |

座標は、Town01.inpxと同じく、OpenDRIVEの座標そのもの(Vissimの(x, y) = CARLAの(x, −y))。

# 1. 道路網の取り込み(Windows PC)

Town01.inpxは、VissimでOpenDRIVE(`Town01.xodr`)を取り込んで作られている(リンク名が`Road 0 - 0 - Right`の形で、座標がOpenDRIVEと一致する)。
Town10HDも同じ方法で道路網を作る。

## 1.1 OpenDRIVEをWindows PCへ写す

```bash
ls -l ~/CARLA_0.9.16_jp/LinuxNoEditor/CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr
```

- このファイルをWindows PCへ写す。
- 0.9.15(`~/CARLA/CarlaUE4/Content/Carla/Maps/OpenDrive/Town10HD.xodr`)と同じファイル(md5 `570c77d196b918813e95750fcb980214`)。
  `Town10HD_Opt.xodr`も同じ中身なので、どちらを使ってもよい。

## 1.2 Vissimで取り込んで保存する

1. Vissim 2026を起動し、新しいネットワークにする。
2. OpenDRIVEの取り込み(英語版のメニューは File > Import > OpenDRIVE。版によって名前が違うことがある)で、`Town10HD.xodr`を選ぶ。
3. 取り込んだあと、**何も足さない・動かさない**。
   - ネットワーク全体を移動・回転・拡大しない(座標がOpenDRIVEとずれると、CARLAと位置が合わなくなる)。
   - 信号・車両入力・経路・車両タイプは、2章のツールで足す。
4. `Town10HD_road.inpx`という名前で保存する。
5. 保存したファイルをLinuxへ写し、このフォルダ(`simulator/autoware_carla_interface/vissim/`)に置く。

確かめること:
- リンク名が`<番号>: Road <番号>-<番号>-Right`/`Left`の形になっていること(Vissim 2026で取り込んだTown10HDの形。Town01.inpxの`Road 0 - 0 - Right`とは空白の入り方が違う)。
  2026-11-17に取り込んだものは、リンク111本(1車線54・2車線57)、コネクタ148本、conflictArea 228個。
- Town10HDのOpenDRIVEには、交差点でない道路が23本(どれもレーンセクションは1つ)、交差点が9つある。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
grep -c "<link " vissim/Town10HD_road.inpx
grep -oE 'name="[0-9]+: Road [0-9]+-[0-9]+-(Right|Left)"' vissim/Town10HD_road.inpx | head
```

# 2. 信号・交通需要・車両設定を足す

`Town10HD_road.inpx`(1章)を変えたとき、またはSUMOの信号・CARLAの信号を変えたときだけ実行する。

**この章のツール(`tools/vissim/build_vissim_town.py`)とテストは、`feature/sumo_co-sim`ブランチにある**(入力にSUMOのnet・ルート(`sumo/`)を使うため)。
Vissim構成のブランチ(`feature/vissim_windows_co-sim`)の`vissim/`は、`feature/sumo_co-sim`で作ったものの写し。作り直したら、次のコマンドでこちらにも写す。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe   # feature/vissim_windows_co-sim を開いた状態で
git checkout feature/sumo_co-sim -- simulator/autoware_carla_interface/vissim/Town10HD.inpx \
    simulator/autoware_carla_interface/vissim/Town10HD_road.inpx \
    simulator/autoware_carla_interface/vissim/signal_mapping_town10hd.json
```

以下のコマンドは、`feature/sumo_co-sim`で実行する。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/vissim/build_vissim_town.py \
    --road vissim/Town10HD_road.inpx \
    --template ~/CARLA/Co-Simulation/PTV-Vissim/examples/Town01/Town01.inpx \
    --sumo-net sumo/net/Town10HD.net.xml --sumo-rou sumo/rou/Town10HD.rou.xml \
    --carla-json ~/autoware_map/Town10HD/carla_tl.json \
    --out vissim/Town10HD.inpx --signal-mapping-out vissim/signal_mapping_town10hd.json   # ← 生成物
python3 -m pytest test/test_build_vissim_town.py
```

> 📄 **生成物:** co-sim用のTown10HDのネットワークと、信号の対応表\
> `vissim/Town10HD.inpx`、`vissim/signal_mapping_town10hd.json`

- 実行すると、信号灯ごとの置き場所(CARLAの信号・信号制御・信号グループ・リンク・車線・位置)と、車両入力の一覧が出る。
- テストは、作り直した結果がコミットしたファイルと同じか、信号の対応がSUMOとCARLAに合っているかを確かめる。

| オプション | 説明 | 既定値 |
| --- | --- | --- |
| `--road` | 1章の道路網 | なし(省略不可) |
| `--template` | 車両設定と信号制御を写す`.inpx`(CARLAリポジトリ`main`のTown01.inpx) | なし(省略不可) |
| `--sumo-net` / `--sumo-rou` | 信号(`tlLogic`・`linkSignalID`)/ 車両の出発地点の割合 | なし(省略不可) |
| `--carla-json` | CARLAの信号と停止位置(`tools/carla_tl_to_lanelet2.py --dump-json`の出力) | なし(省略不可) |
| `--total-volume` | 車両入力の合計 [台/h] | `600` |

## 2.1 足したもの

| 足したもの | 中身 |
| --- | --- |
| 車両設定 | Town01.inpxの車両タイプ・2D/3Dモデル・車両構成・歩行者タイプ・走行挙動など。自車(EGO、車両タイプ101)と、ドライブシミュレータの設定(`drivSimActive="true"`など)、カーブの速度制限(無効)、駐車場・エレベータの既定値も同じ |
| 信号制御 | SUMOの信号1つにつき1つ(5つ)。中身はTown01.inpxの信号制御1と同じ固定時間の制御(信号グループ1・2が交互に青、周期94秒) |
| 信号灯 | CARLAの15信号の停止位置(信号ごとに2車線)に1つずつ、計30。SUMOの最初のフェーズで青の信号は信号グループ1、赤の信号は2 |
| 車両入力 | SUMOのルートの出発地点に当たる19リンク。合計600台/hを、SUMOの出発台数の割合で分ける |

取り込みで作られた信号制御(8つ)と信号灯(2つ。CARLAにない信号955・956のもの)は消した。

## 2.1.1 ツールのあとにVissimで直したもの(2026-11-17、利用者)

コミットしている`Town10HD.inpx`は、ツールで作ったあとに、VissimでCARLA 0.9.16_jpの日本の車に合わせて車両の設定を直したもの。
**ツールで作り直すと、この変更は消える**(作り直したら、Vissimで同じように直す)。

| 2D/3Dモデル分布 | ツールで作ったとき(Town01と同じ) | 直したあと |
| --- | --- | --- |
| 10 乗用車 | Vissimの乗用車10モデル | `[CUSTOM]Car - Toyota Alphard`・`Toyota NOAH`・`Toyota JPN Taxi`(各1:1:1。モデルファイル`vehicle.jp.alphard`・`vehicle.jp.minivan`・`vehicle.jp.taxi`) |
| 21 トラック | 23 DAF LF | `[CUSTOM]HGV - ISUZU GIGA`(`vehicle.jp.large_truck`) |
| 22 トレーラー | 22 Volvo E(トラクター、5.96 m) | 変えない(一度セミトレーラーのセグメントを足して連結車にしたが、外してトラクターだけに戻した。下の注意) |
| 30 バス | 31 C2 Standard | `[CUSTOM]Bus - ISUZU Erga`・`ISUZU GALA`(`vehicle.jp.route_bus`・`vehicle.jp.tour_bus`) |

- 2D/3Dモデルを6つ(1001〜1003・1021・1031・1032)と、色の分布`1006 grey`を足し、車両タイプ100・210・220・300の色の分布を変えた。
- 車両クラス10(乗用車)から、自車の車両タイプ101(EGO)を外した(乗用車クラスは車両タイプ100だけ)。
- 車両構成1(Default)の相対交通量: 乗用車 0.6 → 0.5、バス 0.1 → 0.2(トラック・トレーラー・バイクは0.1のまま)。
- 信号・車両入力・ドライブシミュレータの設定(`drivSimActive="true"`)は、ツールで作ったときのまま。
- Vissimで保存したので、書式(`/>`の前の空白、数値の桁)がツールの出力と違う。
- **トレーラーを連結車にしない**: モデル22にセミトレーラーのセグメントを足すと、車両のブループリント表に連結車(2体目からが軌跡に沿う体)が入る。
  Vissim構成のブリッジは連結車を扱わず、起動時の検査(`carla_fleet.py`の`check_no_follows_path`)で`Error: CARLA blueprint assignment: variant_articulated_unsupported`として止める。
  そのため、セミトレーラーのセグメントは外し、トラクターだけにした(2026-11-17)。外したあと、0.9.15の外形で作った表は8モデル、トレーラーは`carlacola` × 2(連結なし)で、この検査を通る。
  トレーラーの割合を0にするか、セミトレーラーのセグメントを外すか、`vissim_carla_variants:=legacy_vtypes`で起動する。

## 2.2 SUMO版との違い・注意

- **信号の周期**: Vissimは94秒(青42秒・黄3秒・全赤2秒 ×2)、SUMOは90秒(青42秒・黄3秒 ×2)。
  信号制御の中身はTown01.inpxからそのまま写している(書き換えると、Vissimが確かめる中身の値と合わなくなるおそれがあるため)。
- **経路**: 経路の指定はない(Town01.inpxと同じ)。車はコネクタを選びながら走る。SUMOのように1台ごとに決まった経路は走らない。
- **交通量**: SUMOは0〜199秒に計200台、Vissimは一定の流量(既定600台/h)。
- **交差点の優先**: 取り込みで作られたconflictArea 228個のうち86個は優先が決まっていない(`UNDETERMINED`)。信号のない交差点で車が譲り合わないときは、Vissimで設定する。
- **信号の対応表**: `~/CARLA`の`util/generate_signal_mapping.py`(信号灯にいちばん近いCARLAの信号の柱を選ぶ)は、Town10HDでは使えない。
  Town10HDの信号の柱は交差点の向こう側(停止線から30〜40 m先)にあり、いちばん近い柱が別の進入路の信号になるため。
  このツールは、CARLAが信号ごとに返す停止位置と、SUMOの`linkSignalID`で対応を決める。

## 2.3 co-simで使うとき(Vissim構成のブランチ)

手順は、`feature/vissim_windows_co-sim`ブランチの`docs/Vissim(win)-CARLA(0.9.16)-Autoware_co-sim_起動手順.md`(CARLA 0.9.16でTown10HDを使う手順)。

- `Town10HD.inpx`をWindows PCへ写し、`server.py --vissim-network`に渡す。
- `signal_mapping_town10hd.json`を、Vissimブリッジの`vissim_integration/data/signal_mapping.json`へ写す。今のブリッジは対応表を1つしか置けないので、Town01の対応表を上書きすることになる。
- Autowareと使うときは、車両のブループリント表を`build_carla_variants.py --inpx vissim/Town10HD.inpx`で作る(2.1.1)。

## 2.4 Vissimでの確認

- 2026-11-17に、Windows PCのVissim 2026で`Town10HD.inpx`を開き、エラーなく読み込めること、RUNで信号が切り替わることを確かめた(利用者が確認)。
  そのとき出た警告2件(駐車場の既定値の参照、カーブの速度制限)は、ツールを直して出ないようにした。
  直した版(コミット`f82a6b31b`)も同じ日に確かめた: 開いたとき・RUNを止めたときとも、エラー・警告なし(利用者が確認)。
- co-sim(Vissim-CARLA、Vissim-CARLA-Autoware)で動かす確認はまだ。
- Vissimで実行したあとに保存すると、`drivSimActive`が`false`に戻ることがある。co-simで使う前に確かめる。
