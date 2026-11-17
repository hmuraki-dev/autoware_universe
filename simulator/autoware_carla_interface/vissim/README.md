# Vissimのネットワーク(Vissim-CARLA-Autoware co-sim)

Vissim(Windows)で読み込む`.inpx`を置く。CARLAの版(0.9.15・0.9.16)によらず、ここのものを使う。

| ファイル | 内容 | 作り方 |
| --- | --- | --- |
| `Town10HD_road.inpx` | Town10HDの道路網だけ(リンク・コネクタ)。信号・交通需要・車両設定はない | 1章(Windows PCのVissimでOpenDRIVEを取り込む) |
| `Town10HD.inpx` | co-simで使うTown10HDのネットワーク。道路網に、信号・交通需要・車両設定を足したもの | 2章(作成中) |

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
- リンク名が`Road <番号> - <番号> - Right`/`Left`の形になっていること。
- Town10HDのOpenDRIVEには、交差点でない道路が23本(どれもレーンセクションは1つ)、交差点が9つある。

```bash
cd ~/autoware.1.9.0/src/universe/autoware_universe/simulator/autoware_carla_interface
grep -c "<link " vissim/Town10HD_road.inpx
grep -o 'name="Road [0-9]* - [0-9]* - [A-Za-z]*"' vissim/Town10HD_road.inpx | head
```

# 2. 信号・交通需要・車両設定を足す(作成中)

`Town10HD_road.inpx`に、次を足して`Town10HD.inpx`を作るツールを作成中。

| 足すもの | 中身 | 元にするもの |
| --- | --- | --- |
| 信号制御 | 交差点ごとに1つの信号制御(5つ)、信号グループは2つ(交互に青) | SUMOの`sumo/net/Town10HD.net.xml`の信号(5交差点、42秒青・3秒黄の2現示)。制御の中身はTown01.inpxと同じ形 |
| 信号灯 | CARLAの15信号の停止位置の車線に置く | `~/autoware_map/Town10HD/carla_tl.json`(CARLAから読んだ信号と停止位置) |
| 信号の対応表 | Vissimの(信号制御, 信号グループ)→CARLAの信号ID | 上の2つから決まる(`signal_mapping.json`) |
| 交通需要 | 車両入力 | SUMOの`sumo/rou/Town10HD.rou.xml` |
| 車両設定 | 車両タイプ・2D/3Dモデル・車両構成、自車(EGO)のタイプ、ドライブシミュレータの設定 | Town01.inpx |
