# Robominton2026

2026年度 ロボット設計製作特論（グループG班）のバドミントンロボット開発プロジェクトです。
ROS 2を用いて、移動台車・センサ・ラケットの制御とシャトルの追跡・打撃動作を検証します。

現在は、4輪独立操舵・独立駆動台車の簡易シミュレーションと、
Ubuntu 20.04 / ROS 2 FoxyによるDocker開発環境を提供しています。

## 実装内容

- 4輪独立操舵台車、ラケット、カメラ、ピッチ可動2D LiDARのURDFモデル
- 台車の逆運動学、速度指令による移動、目標位置への追従
- 仮想カメラ画像と2D LiDARスキャンの生成
- ラケットのピッチスイングとシャトルの衝突反射
- 重力・空気抵抗を用いたシャトルの飛行計算、打撃後の軌道・着地点予測
- RVizによるモデル、センサ情報、実軌道・予測軌道の表示

シミュレーションのモデルと制約は [シミュレーション仕様](docs/simulation.md) を参照してください。

## 必要環境

| 項目 | 要件 |
|---|---|
| ホスト環境 | Linux。検証環境はUbuntu 22.04 / amd64 |
| コンテナ実行環境 | Docker Engine、Docker Composeプラグイン（`docker compose`） |
| コンテナ内環境 | Ubuntu 20.04、ROS 2 Foxy、Python 3.8、C++17 |
| GUI表示 | X11またはXWayland、ホスト側の `xauth` |

ホスト側へのROSインストールは不要です。ホストのROS 2 HumbleとFoxyのコンテナを併用できます。
Foxyはサポート終了済みであり、Jetson向けソフトウェアとの互換性確認を目的に使用しています。
Jetson実機の対応状況は「検証範囲と制約」を参照してください。

## セットアップ

以下のコマンドは、ホスト側の端末で実行します。

### リポジトリの取得

```bash
mkdir -p ~/robominton_2026_ws/src
cd ~/robominton_2026_ws/src
git clone --branch sim/robom1-integration --single-branch https://github.com/open-rdc/Robominton2026.git
cd Robominton2026
```

### 開発環境の構築

```bash
bash scripts/foxy.sh image
bash scripts/foxy.sh build
```

`image` はDockerイメージの構築、`build` はコンテナ内でのROSパッケージのビルドを実行します。
イメージの初回構築にはネットワーク接続が必要です。

## シミュレーションの実行

以降のコマンドは、リポジトリ直下で実行します。

### ラケット衝突・軌道予測

```bash
bash scripts/foxy.sh gui impact_demo:=true
```

台車を静止させ、ラケットへのシャトル供給とスイングを繰り返します。
接触時のラケット速度と反発係数から打撃後の速度を計算し、軌道と着地点を表示します。

| 表示 | 内容 |
|---|---|
| 青線 | シャトルの実軌道 |
| 緑線 | 打撃後の予測軌道 |
| 黄色 | 予測着地点 |
| 赤点 | ラケットとの接触位置 |
| 水色の円弧 | ラケットヘッド中心のスイング軌道 |

### 台車追従

```bash
bash scripts/foxy.sh gui
```

シャトルの正解座標を用いて台車を移動させるデモです。画像認識による追従ではありません。

### 画面表示を使用しない実行

```bash
bash scripts/foxy.sh sim impact_demo:=true
```

シミュレーションは、起動した端末で `Ctrl+C` を入力すると停止します。

## 開発とテスト

ホスト側で編集したソースは、コンテナ内の `/ws/src/Robominton2026` に反映されます。
変更後は必要に応じてビルドし、対象ノードを再起動してください。
Dockerfileやコンテナ内の依存パッケージを変更した場合は、イメージも再構築します。

| コマンド | 用途 |
|---|---|
| `bash scripts/foxy.sh image` | Dockerイメージの構築・更新 |
| `bash scripts/foxy.sh build` | ROSパッケージのビルド |
| `bash scripts/foxy.sh test` | 単体テストと結果の確認 |
| `bash scripts/foxy.sh smoke` | ROSノード間の通信・動作・終了処理の検証 |
| `bash scripts/foxy.sh shell` | Foxy環境の対話シェル起動。終了は `exit` |

スモークテストは、自動追従・手動操作・目標位置への追従・ラケット衝突の4モードを検証します。

Foxyのビルド成果物は、`/ws/build`・`/ws/install`・`/ws/log` に対応するDocker専用ボリュームに保存します。
ホスト側のHumble用成果物とは分離しています。Gitではソースと環境構築設定を管理します。

追加端末からの操作、更新手順、保存先、トラブル対処は
[Docker開発環境](docs/docker_foxy.md) を参照してください。

## リポジトリ構成

```text
Robominton2026/
├── README.md
├── compose.yaml                  # Foxy開発環境
├── compose.gui.yaml              # GUI表示用の追加設定
├── docker/foxy/                  # Dockerfile、entrypoint
├── scripts/                      # 環境操作、動作検証
├── docs/                         # 操作・設計資料
└── packages/
    ├── robominton_app/
    ├── robom1_description/
    ├── robom1_control/
    ├── robom1_sim/
    └── robom1_bringup/
```

| パッケージ | 役割 | 主な記述言語 |
|---|---|---|
| `robominton_app` | 認識・戦略処理の実装用パッケージ。現在は雛形 | Python |
| `robom1_description` | URDFモデル、センサ配置、RViz設定 | XML / Xacro |
| `robom1_control` | 台車の逆運動学、速度処理、目標追従 | C++ |
| `robom1_sim` | シャトル、仮想センサ、ラケット衝突、飛行予測 | Python |
| `robom1_bringup` | 各ノードの起動とパラメータ設定 | Python / YAML |

colconは `packages/` 以下の5パッケージを検出します。

## ホストのROS 2 Humbleによる実行

Ubuntu 22.04 / ROS 2 Humbleがインストールされた環境では、Dockerを使用せずに実行できます。
ビルドはワークスペース直下で実施します。

```bash
cd ~/robominton_2026_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch robom1_bringup sim.launch.py impact_demo:=true
```

## 検証範囲と制約

開発PC（amd64）のFoxy Docker環境で、5パッケージのビルド、単体テスト、
4モードのスモークテストを確認しています。GUIはウィンドウの起動を確認済みです。
既存パッケージの著作権検査1件はスキップされています。

- 台車は理想運動学モデルであり、タイヤの滑りやモータ負荷は扱いません。
- 台車追従とLiDAR照準にはシミュレーション上の正解座標を使用します。実画像認識・迎撃計画は未実装です。
- ラケットとシャトルの衝突は、有限円柱と球による簡易モデルです。反発係数・空力係数は実測同定していません。
- ネット・壁・台車本体との衝突は扱いません。
- Jetson実機は未検証です。GPU・カメラ・各種I/Oの利用には、機種とJetPackに応じた設定が必要です。

## 関連ドキュメント

- [Docker開発環境](docs/docker_foxy.md)：環境構築、操作、更新、トラブル対処
- [シミュレーション仕様](docs/simulation.md)：モデル、トピック、制御指令、衝突・軌道予測
- [設計資料](docs/README.md)：機体仕様、配線、担当分担の資料構成
