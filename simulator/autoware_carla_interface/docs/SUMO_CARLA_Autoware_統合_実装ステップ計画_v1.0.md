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

## Step 1: CARLA接続の一元化(v0.5 2.5 / 2.6 / 3.2-1)

- `CarlaSimulation.__init__` を改修し、外部から `client`/`world` を注入できるようにする
- `SimulationSynchronization.__init__` 内の `world.apply_settings()`/`traffic_manager.set_synchronous_mode()` の重複呼び出しを削除
- この時点ではSUMOはまだ繋がない。既存の `autoware_carla_interface`(CARLA単体)の動作に影響がないことだけを確認する

**確認方法**: 既存のCARLA単体シミュレーションが従来通り動くことを回帰確認

---

## Step 2: パラメータ追加・ステップ時間統一(v0.5 2.1 / 2.2)

- launchファイルにSUMO関連パラメータ(`.sumocfg`パス、`--sumo-gui`、`tls-manager`等)を追加(まだ未使用でOK)
- `fixed_delta_seconds` のデフォルトを0.05→0.1秒に変更(決定済み事項)

**確認方法**: CARLA単体で0.1秒ステップでも従来通り動作することを確認

---

## Step 3: SUMO起動・TraCI接続 + 同期エンジン生成(v0.5 2.3 / 2.4 / 2.7 / 2.8)

- `SumoSimulation` のインポート・起動・TraCI接続を `InitializeInterface` に組み込む
- `SimulationSynchronization`(ID対応表・座標変換込み)を生成する
- `tick()` はまだ呼ばない。接続確認のみ

**確認方法**: CARLA・SUMOの両方が起動し、正常に接続・切断(2.13相当のクリーンアップ)できることを確認

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

| Step | 内容 | 対応するSUMO Tick/CARLA Tickの実動作 |
|---|---|---|
| 0 | 設計決定の確定 | なし |
| 1 | CARLA接続の一元化 | なし(CARLA単体のまま) |
| 2 | パラメータ追加・ステップ時間統一 | なし(CARLA単体のまま) |
| 3 | SUMO起動・TraCI接続 + 同期エンジン生成 | なし(接続確認のみ) |
| 4 | メインループへの同期処理組み込み | **あり(最重要ステップ)** |
| 5 | NPC排他制御 | あり |
| 6 | EGO登録・双方向同期の検証 | あり |
| 7 | 終了処理の統合 | あり(終了時) |
| 8 | 動作安定化 | あり |
| 9〜 | 機能拡張 | あり |
