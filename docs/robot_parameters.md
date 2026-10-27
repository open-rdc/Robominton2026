# 機体モデルの設定

この開発ブランチ `dev/robot-description-parameters` は、共有済みの
`sim/robom1-integration`（`a7f9351`）を起点としています。
Jetsonで確認できた従来のブランチを維持し、機体仕様の具体化をこちらで進めます。

## 設定ファイルと適用先

| ファイル | 内容 |
|---|---|
| `packages/robom1_description/config/robot.yaml` | 機体寸法、質量、取り付け姿勢、関節制限、センサ仕様、スイング条件 |
| `packages/robom1_description/config/robot_large.example.yaml` | 寸法・センサ姿勢・速度制限を変えた設定例。実機の設計値ではありません |
| `packages/robom1_bringup/config/sim.yaml` | 初期台車位置、台車の速度・加速度、投射周期、空気抵抗、反発係数などの実験条件 |

launchは機体YAMLを検証し、同じファイルからURDFと各ノードのパラメータを作成します。
形状を変更するためにURDFとPythonをそれぞれ編集する必要はありません。
機体寸法・センサ設定は `sim.yaml` へ追加せず `robot.yaml` で管理してください。
起動時に読み込む設定であり、起動中の動的な変更には対応していません。

## 座標系と単位

- 長さm、質量kg、角度rad、角速度rad/s、努力上限N·m、時間sを使用します。
- `base_footprint` は床面の台車基準。x前・y左・z上です。
- `chassis.center_z` は `base_link` の床からの高さです。
- `camera.xyz`、`lidar.xyz`、`mast.xyz`、`racket.pivot` はすべて `base_footprint` 基準です。
  `base_link` の高さを変えても、取り付け高さを二重に加算しません。
- `rpy` は固定軸のroll/pitch/yawで、回転行列は `Rz(yaw) Ry(pitch) Rx(roll)` です。
- カメラ本体フレームは+x前向き、光学フレームは+z前・+x右・+y下です。
- LiDARは固定の取り付け姿勢を適用した後、そのローカル-y軸まわりにピッチ回転します。
  正のピッチ指令で走査面が上を向きます。
- ラケットは固定の取り付け姿勢の後、ローカル+y軸まわりに振ります。
  ヘッドは支点からローカル+z方向の `reach` の位置にあります。

## 設定項目

### 台車 `chassis`

| キー | 内容 |
|---|---|
| `size` | 車体の長さ・幅・厚さ `[x, y, z]` |
| `center_z` | 車体中心の床からの高さ |
| `mass` | 車体リンクの質量 |

車体底面が床より下になる設定は拒否します。タイヤとの干渉や床下クリアランスの妥当性は
別途確認してください。台車全体の質量ではなく、各リンクにそれぞれの質量を割り当てます。

### 車輪と操舵 `wheels`

| キー | 内容 |
|---|---|
| `wheelbase`, `track` | 前後・左右の操舵軸間距離 |
| `radius`, `width`, `mass` | 車輪の半径・幅・1輪の質量 |
| `drive_velocity`, `drive_effort` | 車輪軸の速度・努力上限 |
| `steer_size`, `steer_mass` | 操舵部の箱形状と1個の質量 |
| `steer_velocity`, `steer_effort` | 操舵軸の速度・努力上限 |
| `damping`, `friction` | URDFの関節dynamics属性 |

4輪独立操舵・独立駆動、操舵軸と車輪中心のオフセット0、無制限回転の操舵軸を仮定しています。
車輪中心の高さは半径から計算し、タイヤ底面を床へ合わせます。
制限付き操舵や車輪ごとに違う形状は、機構確定後に運動学と合わせて拡張してください。
`drive_velocity` は車輪軸のrad/sです。モータ軸のRPMの場合は減速比と単位変換が必要です。

### 支柱 `mast`

`size` は箱の3辺、`xyz` は中心位置、`mass` は質量です。
カメラとLiDARの取り付け位置は独立して設定するため、支柱を伸ばしてもセンサ位置は自動では変わりません。

### カメラ `camera`

| キー | 内容 |
|---|---|
| `size`, `mass` | 本体形状と質量 |
| `xyz`, `rpy` | 本体の取り付け位置と姿勢 |
| `width`, `height` | 画像の幅・高さpx |
| `fx`, `fy`, `cx`, `cy` | ピンホールカメラの内部パラメータpx |
| `rate` | 仮想画像の配信周波数Hz（最大50） |

内部パラメータはCameraInfoと合成画像の両方へ適用します。解像度だけ変えても
焦点距離や主点は自動スケーリングしません。実カメラでは校正結果へ置き換えてください。
歪み、露光、ローリングシャッター、検出遅延は現在の合成画像には含みません。

### LiDAR `lidar`

| キー | 内容 |
|---|---|
| `size`, `mass`, `xyz`, `rpy` | 本体形状・質量・取り付け姿勢 |
| `lower`, `upper`, `initial` | ピッチ関節の可動範囲と初期角 |
| `effort`, `velocity` | ピッチ軸の努力・角速度上限 |
| `damping`, `friction` | URDFの関節dynamics属性 |
| `angle_min`, `angle_max`, `samples` | 走査角範囲とサンプル数。両端を含む |
| `range_min`, `range_max` | 有効距離範囲m |
| `rate` | 仮想スキャン配信周波数Hz（最大50） |

URDFの取り付け姿勢と同じ回転を仮想レーザーにも適用します。
関節角は指定範囲へ制限し、`velocity` に従って変化します。
スキャンは瞬時取得であり、1点ごとの測距時間やピッチのバックラッシュはモデル化していません。

### ラケット `racket`

| キー | 内容 |
|---|---|
| `pivot`, `rpy` | 柄の根元の支点位置と固定取り付け姿勢 |
| `handle_length`, `handle_width`, `handle_mass` | 柄の形状と質量 |
| `grip_length`, `grip_width` | グリップの表示寸法 |
| `reach`, `radius`, `thickness`, `head_mass` | ヘッド中心までの距離・ヘッド半径・厚さ・質量 |
| `lower`, `upper`, `effort`, `velocity` | スイング関節の制限 |
| `damping`, `friction` | URDFの関節dynamics属性 |
| `ready`, `backswing`, `follow_through` | 構え・テイクバック・振り抜きの角度 |
| `manual_velocity` | 角度指定による手動移動の速度上限 |
| `preparation_time`, `stroke_time`, `hold_time`, `return_time` | 準備・振り抜き・保持・復帰の要求時間 |

`lower <= backswing < ready < follow_through <= upper` を満たす必要があります。
5次補間の最大角速度を計算し、要求時間が短すぎる場合は区間を延ばして `velocity` を守ります。
そのため速度上限の変更は打撃時刻にも影響します。衝突デモの供給時刻はこの所要時間から計算しますが、
大幅に遅いスイングや異なる姿勢では供給条件が成立しない場合があります。

柄の質量はグリップを含むリンク全体の値として入力します。グリップの太い部分は表示のみで、
柄の慣性・collisionは一様な細い箱で近似します。ヘッドは一様な円柱で、衝突形状と寸法を共有します。

## 慣性・制限の反映範囲

| 項目 | 今回の簡易simでの扱い |
|---|---|
| 寸法・位置・姿勢 | URDF、TF、逆運動学、仮想センサ、ラケット衝突へ反映 |
| リンク質量・慣性 | URDFへ反映。箱と円柱の重心まわりの慣性を計算 |
| 車輪の回転速度上限 | 台車の速度ベクトル全体を縮小して各輪の上限を守る |
| LiDAR・ラケットの関節角・速度 | 指令と動作へ反映 |
| 操舵速度上限 | URDFの仕様値。現在の台車simは瞬時操舵で、追従遅れは未実装 |
| 関節努力・摩擦・減衰 | URDFへ記述。トルクや摩擦を解く物理エンジンでは未使用 |

質量を増やしただけで台車の加速度やラケットのスイングが遅くなることはありません。
加速度・時間・速度制限は別途設定します。
CADから得た非一様な慣性テンソル、重心の偏り、減速機・駆動系の反作用は今後の拡張項目です。
URDFのcollisionがあっても、今のsimが計算する接触はシャトルとラケットヘッドのみです。

## 設定の変更と切り替え

標準設定は `packages/robom1_description/config/robot.yaml` を編集し、simを再起動します。
このブランチへ切り替えた初回はconfigのインストールが必要なのでビルドしてください。

```bash
cd ~/robominton_2026_ws/src/Robominton2026
bash scripts/foxy.sh build
bash scripts/foxy.sh gui impact_demo:=true
```

別の設定例を使う場合は、コンテナから見える絶対パスを指定します。

```bash
bash scripts/foxy.sh gui demo:=false robot_config:=/ws/src/Robominton2026/packages/robom1_description/config/robot_large.example.yaml
```

カスタム設定は標準YAMLをコピーし、全項目を保った上で編集してください。
差分だけを記述したYAMLのマージには対応していません。欠落・未知のキーはエラーにします。
Docker内で読むファイルはリポジトリ配下へ配置するなど、マウント範囲内に置いてください。

ホストのHumbleの場合はホスト側のパスを指定します。

```bash
source /opt/ros/humble/setup.bash
source ~/robominton_2026_ws/install/setup.bash
ros2 launch robom1_bringup sim.launch.py robot_config:=/absolute/path/to/robot.yaml
```

単独でURDFを出力する場合も同じYAMLを使えます。

```bash
# Foxyコンテナ内
xacro /ws/install/robom1_description/share/robom1_description/urdf/robom1.urdf.xacro \
  robot_config:=/ws/src/Robominton2026/packages/robom1_description/config/robot.yaml > /tmp/robom1.urdf
```

Foxy互換のため、Xacro内では旧版でも利用可能な `load_yaml()` を使っています。
launch経由では事前に設定値を検証します。単独のxacroコマンドはlaunchの検証処理を通りません。

## 実機仕様が決まった際の確認

機体寸法・取り付け位置は実測値またはCAD値へ更新し、回転方向とゼロ角をTFで確認してください。
現在の関節軸は車輪+y、操舵+z、LiDAR-y、ラケット+yに固定しています。
軸方向が異なる機構の場合は、URDFと運動学を同時に変更する必要があります。
モータID、通信方式、エンコーダ換算、原点復帰、非常停止はこの機体形状YAMLに含めていません。
これらは今後の実機ドライバと通信・制御仕様で定義します。

## 検証

Foxy Dockerで標準設定の4モードに加え、別設定の起動・画像仕様・LiDAR仕様・
取り付け姿勢TF・ピッチ角制限を検証します。数値テストでは慣性の正値性、URDF接続、
設定値の拒否、取り付け回転後の衝突、低速スイング、車輪速度上限を確認します。
