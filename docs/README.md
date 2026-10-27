# 設計資料

最初の環境構築・チーム共有手順は [Foxy Dockerの使い方](docker_foxy.md) を参照してください。

- [robot_parameters.md](robot_parameters.md): 機体寸法・質量・センサ配置・関節制限の設定。
- [simulation.md](simulation.md): sim起動・手動操作・トピック・制約。
- [original_readme.md](original_readme.md): 友人のリポジトリの元のREADME。
- `specification.md`（今後追加）: 台車寸法、センサ仕様、座標系、速度上限、通信仕様。
- `wiring.md`（今後追加）: 電源、モータ、エンコーダ、カメラ、LiDARの配線とピン番号。
- `roles.md`（今後追加）: 担当者、担当パッケージ、ノード間インターフェース。

実装コードは `../packages/` に置き、ここにはチームで共有する設計情報を記述します。
現在のsimと実機用ノードは、`/cmd_vel`、`/odom`、`/camera/image_raw`、`/scan` などの
ROSトピックで接続・置換できます。自動追従デモは正解座標を使うため、認識ノードの
開発時は `demo:=false` で起動してください。
