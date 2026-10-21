# Foxy Docker開発環境の使い方

このブランチで共有するのは環境を構築する設定とソースです。
各PCで同じDockerイメージをビルドし、コンテナ内でROSパッケージをビルドして使います。
ホスト側のROSやPythonの設定はコンテナへ引き継がれません。

## 事前確認

Linux PCの通常ユーザーで次が成功することを確認してください。

```bash
docker --version
docker compose version
docker info
```

Docker EngineとComposeプラグインのインストールは別途必要です。
`permission denied ... docker.sock` はホスト側のDocker利用権限の問題です。
Dockerの利用権限を整え、通常ユーザーでスクリプトを実行してください。
`sudo bash scripts/foxy.sh image` はUID/GIDをroot用に変えるため使わないでください。
GUIにはX11またはXWayland、`DISPLAY`、ホストの `xauth` が必要です。
Ubuntuでxauthだけ不足する場合は `sudo apt install xauth` で導入できます。

## 初めて取得する人

**PC側の端末** で実行します。

```bash
mkdir -p ~/robominton_2026_ws/src
cd ~/robominton_2026_ws/src
git clone --branch sim/robom1-integration --single-branch https://github.com/open-rdc/Robominton2026.git
cd Robominton2026
git branch --show-current
```

`sim/robom1-integration` と表示されることを確認します。
すでにリポジトリがある場合は重ねてcloneせず、変更を確認してからこのブランチへ切り替えてください。

```bash
cd ~/robominton_2026_ws/src/Robominton2026
bash scripts/foxy.sh image
bash scripts/foxy.sh build
bash scripts/foxy.sh test
bash scripts/foxy.sh smoke
```

`image` はUbuntu 20.04 / Foxyと依存ツールを構築します。初回はネット接続が必要です。
`build` はソースをコンパイルします。ホストにROSがなくても構いません。
スモークテストでは、デモ・手動・目標追従・衝突の4件の `PASS` が出れば成功です。

## RVizで確認する

PCのデスクトップ端末で次を実行します。Docker内のシェルからは実行しません。

```bash
cd ~/robominton_2026_ws/src/Robominton2026
bash scripts/foxy.sh gui impact_demo:=true
```

台車を止め、シャトルをラケットへ供給して打ち返すデモです。
青線=実軌道、緑線=予測軌道、黄色=予測着地点、赤点=接触位置、水色=ラケットの軌道です。
同じ端末でCtrl+Cを押すと停止します。先に起動していたsimは停止してください。

通常の台車追従デモは `bash scripts/foxy.sh gui`。
画面なしは `bash scripts/foxy.sh sim impact_demo:=true` です。
`sim` と `gui` の末尾にはlaunch引数を渡せます。

GUI設定は `compose.gui.yaml` で追加します。X11ソケットと、一時的な認証ファイルだけを共有します。
一時ファイルはスクリプトの終了時に削除します。`xhost +` は必要ありません。
描画はソフトウェア方式を使い、PCのNVIDIA GPUの割り当ては不要です。
Windows/macOSのDocker DesktopでのGUI表示は、この手順の検証対象外です。

## Dockerに入って操作する

PC側で次を実行します。

```bash
bash scripts/foxy.sh shell
```

`developer@...:/ws$` が **コンテナ内** の端末です。

```bash
echo $ROS_DISTRO               # foxy
pwd                            # /ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robom1_bringup sim.launch.py rviz:=false impact_demo:=true
```

sim停止はCtrl+C。コンテナから出るには `exit`。GUI表示はPC側の `gui` コマンドを使います。
初回ビルド前に入ったシェルでは、ビルド後に `source install/setup.bash` が必要です。

起動中のsimと同じROS環境でトピック確認やサービス呼び出しを行うには、
**別のPC側端末** で起動中のコンテナ名を調べ、その名前を指定します。

```bash
cd ~/robominton_2026_ws/src/Robominton2026
docker ps --filter label=com.docker.compose.project=robominton2026-foxy --format 'table {{.Names}}\t{{.Status}}'
# 表示されたNAMEを指定。下の CONTAINER_NAME は置き換えてください。
docker exec -it CONTAINER_NAME /usr/local/bin/robominton-entrypoint bash --rcfile /usr/local/bin/robominton-entrypoint
```

その追加端末内では以下を実行できます。

```bash
ros2 topic list
ros2 topic echo /shuttle/hit_count
# 上のechoをCtrl+Cで停止してから、必要に応じて実行
ros2 service call /shuttle/feed_to_racket std_srvs/srv/Trigger "{}"
```

別途 `shell` コマンドで新しいコンテナを作ると、既定のローカル限定通信では元のsimと通信しません。
追加操作には `docker exec` で同じコンテナへ入ってください。
手動供給サービスは台車停止時に使います。[トピック・サービスの詳細](simulation.md)。

## 編集・更新する

ソースはホストのVS Codeなどで編集できます。
リポジトリ全体を `/ws/src/Robominton2026` へマウントするため、変更はすぐ共有されます。

```bash
# PC側。変更したコードをビルドし、simを再起動
bash scripts/foxy.sh build
bash scripts/foxy.sh gui impact_demo:=true
```

Dockerfileやイメージ内の依存パッケージを変更した場合は、先に `image` を再実行します。
ユーザーが違うPCでも、そのユーザーのUID/GIDで `image` を実行してください。
ビルド済みバイナリや既存のDockerボリュームを、別PCへコピーする必要はありません。

友人が更新を取得する場合は、作業中のsimを停止し、未コミット変更を確認してから:

```bash
git status
git pull --ff-only origin sim/robom1-integration
bash scripts/foxy.sh image
bash scripts/foxy.sh build
bash scripts/foxy.sh smoke
```

## Gitに含まれるもの・保存先

| 対象 | 保存先・共有方法 |
|---|---|
| ROSソース | `packages/`。Gitで共有 |
| Docker構築設定 | `docker/foxy/Dockerfile`、entrypoint、Compose。Gitで共有 |
| 操作・検証 | `scripts/`、README、`docs/`。Gitで共有 |
| Foxyのビルド成果物 | `/ws/build`・`/ws/install`・`/ws/log`。専用Dockerボリューム |
| ホストのHumble成果物 | `~/robominton_2026_ws/build`・`install`・`log`。Git対象外 |
| Dockerイメージ・コンテナ | 各PCのDockerストレージ。Git対象外 |

Foxy用ボリューム名は `robominton2026-foxy_foxy_build`、`..._foxy_install`、`..._foxy_log` です。
`run --rm` でコンテナを終了してもソースとボリュームは残ります。
`docker compose down` もボリュームを保持します。
**`docker compose down --volumes` はFoxyのビルド成果物を削除します。** 必要なときだけ使ってください。

## トラブル対処

- `scripts/foxy.sh: No such file`: `Robominton2026` 直下で実行し、`scripts` の末尾のsも確認。
- パッケージが見つからない: `build` を実行。すでに開いているコンテナでは `source /ws/install/setup.bash`。
- `DISPLAY is not set`: コンテナ内や画面なしSSH端末を出て、PCのデスクトップ端末で `gui` を実行。
- `No X11 authentication cookie`: デスクトップの `DISPLAY` と `XAUTHORITY` を確認。xauthが使えるログインセッションで起動。
- 変更が反映されない: `build` 後にsimとRVizを再起動。Dockerfileの変更なら先に `image`。
- 他のコンテナからトピックが見えない: ローカル限定通信が既定。同じコンテナへ `docker exec` で入る。

## 実機と検証範囲

Foxyはサポート終了済みで、既存Jetsonへの互換性確認に使います。
開発PC（amd64）でビルド・単体テスト・4モードのスモークテストとGUIのウィンドウ起動を確認済みです。
Jetson（arm64）のGPU・CSIカメラ・USB・CANなどは機種とJetPackに合わせた追加設定が必要です。
この開発用設定では実機デバイスやホストネットワークを割り当てていません。
ROS_DOMAIN_IDの初期値は87、通信はコンテナ内のローカルに限定します。
