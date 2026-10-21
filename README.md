# Robominton2026

2026年度 ロボット設計製作特論（グループG班）のバドミントンロボット開発リポジトリです。
この `sim/robom1-integration` ブランチには、4輪独立操舵台車の簡易simと、
Ubuntu 20.04 / ROS 2 FoxyのDocker開発環境をまとめています。

## まず動かす（別のPCで初めて使う場合）

Linux PCに **Docker EngineとDocker Composeプラグイン（`docker compose`）** が必要です。
PC側にROSをインストールする必要はありません。Humbleを使っているPCでも実行できます。
RViz表示にはX11またはXWaylandのデスクトップと `xauth` が必要です。

以下はすべて **PC側の端末** で実行します。

```bash
# mainではなく、simとDockerを含むブランチを取得
mkdir -p ~/robominton_2026_ws/src
cd ~/robominton_2026_ws/src
git clone --branch sim/robom1-integration --single-branch https://github.com/open-rdc/Robominton2026.git
cd Robominton2026

# 初回の環境作成 → ROSパッケージのビルド → 動作確認
bash scripts/foxy.sh image
bash scripts/foxy.sh build
bash scripts/foxy.sh smoke

# ラケット衝突・打撃後の軌道予測をRVizで表示
bash scripts/foxy.sh gui impact_demo:=true
```

すでにこのブランチを取得済みなら `cd ~/robominton_2026_ws/src/Robominton2026` から始めてください。
初回は依存パッケージのダウンロードが必要です。終了は起動した端末で **Ctrl+C**。

衝突デモでは台車を止めてシャトル供給とピッチスイングを繰り返します。
RVizの **青線は実軌道、緑線は打撃後の予測軌道、黄色は予測着地点、赤点は接触位置** です。
引数を省略して `bash scripts/foxy.sh gui` とすると、台車の自動追従デモになります。

## 普段の操作

すべてリポジトリ直下のPC端末から実行できます。

| コマンド | 内容 |
|---|---|
| `bash scripts/foxy.sh image` | Dockerイメージ作成・設定変更後の再作成 |
| `bash scripts/foxy.sh build` | ROSパッケージのビルド |
| `bash scripts/foxy.sh test` | 単体テストと結果確認 |
| `bash scripts/foxy.sh smoke` | 自動追従・手動・目標追従・衝突の4モードを確認 |
| `bash scripts/foxy.sh gui impact_demo:=true` | 衝突と軌道予測をRVizで表示 |
| `bash scripts/foxy.sh gui` | 台車追従をRVizで表示 |
| `bash scripts/foxy.sh sim impact_demo:=true` | 衝突デモを画面なしで起動 |
| `bash scripts/foxy.sh shell` | Foxyの操作用シェルへ入る。出るときは `exit` |

ソースはホストとコンテナで共有されます。VS Codeで編集し、`build` の後にsimを再起動します。
PythonやURDFも、初回ビルド後は変更を共有できますが、起動中のノードへは自動反映しません。
詳しい操作、同じコンテナでの追加端末、更新・保存先・トラブル対処は
[Docker環境の使い方](docs/docker_foxy.md) を参照してください。

## Gitで共有するもの

このブランチで **ソース・Dockerfile・Compose設定・操作スクリプト・説明書** を共有します。
各PCで `image` と `build` を実行して環境を構築します。
Dockerイメージ本体、コンテナ、ビルド済み実行ファイル、ログはGitには含めません。

PC側のHumble用成果物と、Docker内のFoxy用成果物は分離しています。
Foxy内の `/ws/build`・`/ws/install`・`/ws/log` は専用のDockerボリュームです。
複数人がそれぞれのPCで同じ手順を使えます。JetsonはCPU・JetPack・機器の確認が別途必要です。

## 配置と担当

```text
robominton_2026_ws/
├── build/                       # ホストでビルドした場合の成果物
├── install/
├── log/
└── src/
    └── Robominton2026/          # Gitリポジトリ
        ├── README.md
        ├── compose.yaml
        ├── compose.gui.yaml
        ├── docker/foxy/         # Dockerfile、entrypoint
        ├── scripts/            # Docker操作、スモークテスト
        ├── docs/               # 操作・設計資料
        └── packages/
            ├── robominton_app/
            ├── robom1_description/
            ├── robom1_control/
            ├── robom1_sim/
            └── robom1_bringup/
```

| パッケージ | 主な役割 | 言語 |
|---|---|---|
| `robominton_app` | チームの認識・戦略などの実装場所。現在は雛形 | Python |
| `robom1_description` | URDF/Xacro、センサ配置、RViz設定 | XML |
| `robom1_control` | 台車の逆運動学、速度処理、目標追従 | C++ |
| `robom1_sim` | シャトル、仮想センサ、ラケット衝突と飛行予測 | Python |
| `robom1_bringup` | 全体のlaunch・設定値 | Python/YAML |

元の単一Pythonパッケージは `packages/robominton_app` へ移し、ROSの命名規則に合わせて
小文字名へ改名しています。元のREADMEは [original_readme.md](docs/original_readme.md) に保存しています。
リポジトリ直下には `package.xml` を置かず、colconが子の5パッケージを検出します。

## シミュレーションの範囲

台車は理想運動学、シャトルは重力・二乗速度抵抗、ラケットは有限の円柱と球の衝突反射です。
台車追従とLiDAR照準には正解座標を使っています。実画像認識・迎撃計画は未実装です。
反発係数・空力係数は仮値で、ネット・壁・台車本体との衝突や実機精度は再現しません。
Foxyはサポート終了済みで、既存Jetsonへの互換性検証を目的に使っています。

詳細: [sim操作・設計](docs/simulation.md) / [設計資料の案内](docs/README.md)。

## ホストのHumbleで使う場合

Ubuntu 22.04 / ROS 2 Humbleがインストール済みのPCでは、Dockerを使わず起動することもできます。
ビルドはワークスペース直下で行い、FoxyのDockerボリュームとは共有しません。

```bash
cd ~/robominton_2026_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch robom1_bringup sim.launch.py impact_demo:=true
```

## 検証

開発PC（amd64）のFoxy Dockerで5パッケージのビルド、単体テスト、
4モードのROSスモークテストを確認しています。元の雛形の著作権検査1件はskipされています。
GUIはウィンドウ起動を確認済みで、Jetson実機は未検証です。
