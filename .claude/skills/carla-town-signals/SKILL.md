---
name: carla-town-signals
description: SUMO-CARLA-Autoware co-simで、CARLAの信号に対応する信号をSUMOのnet(tlLogic+linkSignalID)とAutowareのLanelet2地図(traffic_light regulatory element・停止線)の両方に設置し、IDの対応・停止位置・交通量・手順書まで揃える手順。Town01で確立した方法を別のTown(Town04・Town05・Town10HDなど)へ展開するとき、SUMOやLanelet2の信号を作り直す・追加する・停止線の位置を直すとき、「信号が対応付かない」「EGOが信号の手前で止まりすぎる」「信号待ちで渋滞する」「CARLAの信号が赤に置き換えられる」を調べるときに使う。add_carla_tls.py、carla_tl_to_lanelet2.py、lanelet2_map_with_tl.osm、linkSignalID、traffic_light.publish が出てきたらこのスキルを使う。
---

# CARLAの信号をSUMOとLanelet2に設置する

CARLAの信号1つ = OpenDRIVEのlandmark ID 1つ。この**同じID**を次の2か所に入れることで、3つのシミュレータの信号がつながる。

- **SUMO**: `tlLogic`の`<param key="linkSignalID:<linkIndex>" value="<landmark>"/>`。ブリッジは`tls_manager:=sumo`のとき、SUMOの状態をCARLAの信号へ書き込む。
- **Lanelet2**: regulatory element(`subtype=traffic_light`)のID(= landmark + `--group-id-offset`)。`autoware_carla_interface`は、CARLAの状態をそのIDで`/perception/traffic_light_recognition/traffic_signals`へpublishする(`traffic_light.publish:=true`)。

## 場所

| もの                 | パス                                                                                                                                            |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| SUMOの信号ツール     | `~/CARLA/Co-Simulation/Sumo/util/add_carla_tls.py`                                                                                              |
| SUMOのnet / ルート(作業場所) | `~/CARLA/Co-Simulation/Sumo/examples/net/<Town>.net.xml` / `examples/rou/<Town>.rou.xml`(ここで作り、下の正本へ写す)                  |
| SUMOのシナリオ(co-simで使う正本) | `<autoware_universe>/simulator/autoware_carla_interface/sumo/examples/`(`.sumocfg`・`net/`・`rou/`・`carlavtypes.rou.xml`など。2026-11-17に`~/CARLA`から移した) |
| SUMO側の手順書       | `~/CARLA/Co-Simulation/Sumo/docs/SUMO-CARLA_co-sim_起動手順.md`「SUMOの信号(Town01)」                                                           |
| SUMO→CARLAの状態変換 | `~/CARLA/Co-Simulation/Sumo/sumo_integration/bridge_helper.py` `get_carla_traffic_light_state`                                                  |
| Lanelet2の信号ツール | `<autoware_universe>/simulator/autoware_carla_interface/tools/carla_tl_to_lanelet2.py`(テスト: `test/test_carla_tl_to_lanelet2.py`)             |
| Lanelet2地図         | `~/autoware_map/<Town>/lanelet2_map.osm` → `lanelet2_map_with_tl.osm`                                                                           |
| 地図の重複点の修正   | `<autoware_universe>/simulator/autoware_carla_interface/tools/fix_lanelet2_duplicate_points.py`(テスト: `test/test_fix_lanelet2_duplicate_points.py`) |
| Autoware側の手順書   | `<autoware_universe>/simulator/autoware_carla_interface/docs/SUMO-CARLA-Autoware_co-sim_起動手順.md` 2.2.2・3.1.2・3.4                               |
| 設計と確認結果       | 同`docs/CARLA_Autoware_信号状態publish_実装計画_v1.0.md`                                                                                        |
| IDの照合             | `<autoware_universe>/simulator/autoware_carla_interface/tools/check_sumo_lanelet2_tl_ids.py`(テスト: `test/test_check_sumo_lanelet2_tl_ids.py`) |

`<autoware_universe>` = このスキルがあるリポジトリ(例: `~/autoware.1.9.0/src/universe/autoware_universe`)。SUMO側は`~/CARLA`の別のgitリポジトリなので、コミットは別々にする。

## 手順

作業を始める前に、上の2つの手順書の該当する節を読む。ツールの引数や確認済みの値が変わっていることがある。

### 1. CARLAの信号を読み取って保存する

CARLAサーバーを起動して対象のTownを読み込み(Autoware側の手順書2.1・2.2)、Lanelet2のツールで信号を読み取る。`--dump-json`で保存しておくと、2回目からはCARLAなしで(`--from-json`)作り直せる。手順4でも使う。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/<Town>/lanelet2_map.osm \
    --dump-json ~/autoware_map/<Town>/carla_tl.json --stop-line-forward-offset 0
```

信号の数を確認する(Town01は36個、landmark 360〜395)。

地図がまだないTownは、Town01と同じ配布元(CARLA autoware-contents)の`vector_maps/lanelet2/<Town>.osm`と`point_cloud_maps/<Town>.pcd`(y軸反転版)を使う。
`~/CARLA/HDMaps/<Town>.pcd`はCARLAの座標のままなので使わない(NDTの自己位置がずれ、RVizでEGOが車線をまたぐ)。

配布元の地図は、境界線に同じ位置の点が続くことがある(Town05は48本、Town10HDは10本のlanelet、Town01はなし)。
そのままだと中心線の先頭がNaNになり、2D Goal Poseを置くとmission_plannerが落ちて経路ができない。
信号を置く前に`tools/fix_lanelet2_duplicate_points.py`で直す(`--check`で数だけ見られる)。

```bash
cp ~/autoware_map/<Town>/lanelet2_map.osm ~/autoware_map/<Town>/lanelet2_map.osm.orig
python3 tools/fix_lanelet2_duplicate_points.py ~/autoware_map/<Town>/lanelet2_map.osm.orig \
    ~/autoware_map/<Town>/lanelet2_map.osm
```

### 2. SUMOのnetに信号を置く

```bash
cd ~/CARLA/Co-Simulation/Sumo
cp examples/net/<Town>.net.xml examples/net/<Town>.net.xml.bak
SUMO_HOME=$(python3 -c 'import sumo; print(sumo.SUMO_HOME)') python3 util/add_carla_tls.py \
    ~/CARLA/CarlaUE4/Content/Carla/Maps/OpenDrive/<Town>.xodr examples/net/<Town>.net.xml -o examples/net/<Town>.net.xml
```

`add_carla_tls.py`の動き(作業の前に知っておくこと):

- `netconvert_carla.py`でxodrから参照netを作り、「どのSUMOリンクがどのlandmarkか」を取り出す。それを編集済みのnet(netedit で加えた横断歩道など)に適用する。
- 対象は、信号のない`priority`の交差点だけ。既存の信号は変えない。作り直したいときは、その交差点の信号を先に消す。
- 信号パターン: 進入路を、向かい合う2本(進む向きの差が135度より大きい)の組と、それ以外の1本の組に分け、組ごとに青42秒→黄3秒を順に回す(直進のある組が先)。T字路は主道路→従道路、十字路は一方の道路→交差する道路になる。2本の組の左折・Uターンは`g`、1本の組は全部`G`。1つのlandmarkのリンクは常に同じ色(`G`/`g`は同じ青)。
- T字路・十字路のどちらも扱える(2026-10-19に`build_programs`を拡張。Town01の出力は拡張の前と同じ)。Town04(12交差点・43信号)とTown05(15交差点・54信号)で最後まで通り、`--report-mixed`で`INFO`が出ないことを確かめた。`examples/net`のTown05には2026-10-19に置いた(Town04はまだ)。止まるのは、1つのlandmarkが複数の進入路にまたがるとき(`controls several approaches`)と、1本の進入路に向かい合う進入路が2本以上あるとき(`several opposite approaches`)。そのときは`approach_groups`を見直す。ツールを変えたら、同じ条件(1つのlandmarkのリンクは常に同じ色)を守り、SUMO側の手順書を更新する。
- `SUMO_HOME`が空だと、`netconvert_carla.py`が終了コード1で失敗する(pipのeclipse-sumoなので、上のように`sumo.SUMO_HOME`を渡す)。
- edgeのIDの付け方が違うと対応付けが0件になる(`--edge-suffix`、既定`.0.00`)。`No junctions to signalize.`が出たら、まずこれを疑う。

出力の`junction <id>: landmarks [...]`の行で、どの交差点にどの信号が付いたかを確認する。

**netがまだないTown**(CARLA同梱の例はTown01・04・05だけ。Town10HDは2026-10-21に作った):
`netconvert_carla.py`でxodrからnetを作り、信号を外して全交差点を`priority`にしてから`add_carla_tls.py`をかける
(`netconvert --tls.unset <ids>`だけでは`type="traffic_light"`が残るので、`sed`で`priority`に書き換えて`netconvert -s`で作り直す)。
このnetのedgeのIDには接尾辞がないので`--edge-suffix ''`を付ける。コマンドはSUMO側の手順書「SUMOの信号(Town10HD)」。
ルートは`randomTrips.py`で、既存のTownと同じく0〜199秒に計200台を作り、車種は既存のルートと同じ種類から選ぶ。`examples/<Town>.sumocfg`も作る。
xodrには、名前がなくCARLAにアクタのない信号がある(Town10HDの955・956)。CARLAの信号の数はCARLAから読んだ数(手順1)を正とする。

**co-simで使う正本へ写す**: co-simの`sumo_cfg_file`は`<autoware_universe>/simulator/autoware_carla_interface/sumo/examples/`を指す。
`~/CARLA`で作った・変えたnet・ルート・`.sumocfg`(と、それが読む`carlavtypes.rou.xml`など)を、同じ相対位置に写す。

```bash
cp ~/CARLA/Co-Simulation/Sumo/examples/net/<Town>.net.xml <autoware_universe>/simulator/autoware_carla_interface/sumo/examples/net/
cp ~/CARLA/Co-Simulation/Sumo/examples/rou/<Town>.rou.xml <autoware_universe>/simulator/autoware_carla_interface/sumo/examples/rou/
cp ~/CARLA/Co-Simulation/Sumo/examples/<Town>.sumocfg <autoware_universe>/simulator/autoware_carla_interface/sumo/examples/
```

Town01のルートを変えたときは、ブループリント表(`carla_variants_sumo_town01.yaml`)も作り直す(Autoware側の手順書2.2.3)。

### 3. Lanelet2地図に信号と停止線を置く

```bash
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/<Town>/lanelet2_map.osm \
    --from-json ~/autoware_map/<Town>/carla_tl.json --stop-line-forward-offset 6 --force
```

- **停止線を前にずらす量**: CARLAの停止位置は、信号灯の約7 m手前にある(Town01では6.7〜8.9 m)。そのままだと、EGOが交差点のかなり手前で止まる。Town01では6 mにした(停止線から交差点の入口まで0.96〜7.08 m)。Townごとに次の手順で決める。
  1. まず`0`で作り、`WARNING`がないことを確かめる。
  2. 値を上げていき、`WARNING: ... not added`(停止線がlaneletの外に出る)が出ない最大値より少し小さい値を選ぶ。Town01の上限は約6.9 m。
  3. 選んだ値と、交差点の入口までの距離の範囲を、手順書3.4.1に書く。
  - Townによっては0が正解になる。Town05は、CARLAの停止位置がもともと交差点の入口の1.3〜8.1 m手前で、1 mでも`WARNING`が出たので0にした。Town10HDも0.9〜6.9 m手前で、同じく0。
    距離はCARLAの道路で測れる(`get_stop_waypoints()`の各waypointから`next(0.1)`を`is_junction`まで進める)。
- **多車線の信号**: 1つの信号に停止位置が複数あると、停止位置ごとにregulatory elementができる(Town05は54信号で111個、Town10HDは15信号で30個)。
  OpenDRIVEのIDを持つのは最初の1つだけで、ほかは新しいID。信号灯(`refers`)は共有するので、照合ツールはそれで1信号にまとめる。
- **IDの衝突**: 既存の地図のIDとlandmark IDがぶつかると止まる。`--group-id-offset 100000`などを付ける。このとき、ブリッジはIDではなく位置で対応付ける(`traffic_light.map_path`、既定で有効)。
- 終了コード2は、追加されなかった信号があったという意味。理由を1つずつ確かめる。
- 出力は、元の地図と同じフォルダに置く(CARLAの地図名はフォルダ名から決まる)。

### 4. IDを照合する

```bash
cd <autoware_universe>/simulator/autoware_carla_interface
python3 tools/check_sumo_lanelet2_tl_ids.py \
    sumo/examples/net/<Town>.net.xml ~/autoware_map/<Town>/lanelet2_map_with_tl.osm \
    --carla-json ~/autoware_map/<Town>/carla_tl.json --report-mixed [--group-id-offset N]
```

- `OK`(終了コード0)なら、CARLAの全信号がSUMOとLanelet2の両方にあり、1つのlandmarkが複数のtlLogicに分かれていない。
- `INFO ... -> CARLA shows red`は、1つのlandmarkのリンクが`G`/`g`以外の違う状態に分かれる信号。そのフェーズの間、CARLAとAutowareには赤として届く(`bridge_helper.py`の仕様)。Town01では、手で設定した360と374だけが該当し、手順書3.4.4に書いてある。`add_carla_tls.py`で作った信号が出てきたら、ツールの不具合として直す。

### 5. 実機で確認する

Autoware側の手順書2.4.1の起動コマンドで、`lanelet2_map_file:=lanelet2_map_with_tl.osm`と`traffic_light.publish:=true`を付けて起動する。

```bash
grep -nE "Traffic-light matching|Traffic lights|traffic-light|Different SUMO" /tmp/autoware_carla.log
```

- `Traffic-light matching: N matched, 0 ambiguous, 0 too far`のNが信号の数と同じであること。
- `Different SUMO traffic-light states`が、手順4で`INFO`の出た信号でだけ出ていること。
- 信号の手前に向かうスポーン地点(手順書3.1.2の表の「信号372の手前」と同じ選び方)から走らせて、次を確かめる。
  スポーン地点は、信号の停止位置まで(`next`)だけでなく、手前の交差点から(`previous`)も直線で離れていることを確かめる。
  交差点を出た直後の地点は避ける(Town05で一度選んでしまった)。
  - 赤で停止線に止まるか(止まった位置と、交差点の入口までの距離)。
  - 青で発進するか。
  - 必要なら、SUMO側の手順書「信号機定点カメラ」(`dbg_mon_tls.py --id <landmark>`)で、CARLAの信号の色も見る。

### 6. 交通量を見直す

全交差点に信号を置くと、信号待ちの渋滞が起きる。Town01では、入力2か所×600台/hで、1800秒後に591台中131台が投入待ちになった。SUMO単体で回して、投入待ちとテレポートを確かめる。多すぎたら`rou/<Town>.rou.xml`の交通量を下げる(Town01では、入力1・2を各300台/h、多様な経路の入力3〜10を各40台/hにした)。

### 7. 記録とコミット

- **SUMO側の手順書**: 「SUMOの信号(<Town>)」の節(junctionとlandmarkの対応表、信号パターン、再生成のコマンド、交通量の注意)。
- **Autoware側の手順書**:
  - 2.2.2: 地図を作るコマンド例
  - 3.4.1: 停止線の値、信号の数
  - 3.4.4: 赤に置き換えられる信号
  - 3.1.2: 確認済みのスポーン地点。交通量と空き待ちの実績は、信号の設置前後で変わるので、いつの値かを書く。
  - 2.4.1: コマンド例を変えた場合
- **README**: Lanelet2側の引数や既定値を変えた場合。
- **ツールを変えたとき**: `test/test_carla_tl_to_lanelet2.py`にテストを足して実行する(`python3 -m pytest test/test_carla_tl_to_lanelet2.py`)。CARLAもROSもいらない。
- **コミット**: リポジトリごとに分ける(`~/CARLA`の作業場所と、`sumo/examples`の正本の両方)。Autoware側は`feat(autoware_carla_interface): ...`/`docs(sumo-carla-autoware co-sim): ...`の形で、日本語で書く。実機で確かめた結果は、利用者が確認した内容として、`docs(...)`のコミットに残す。
