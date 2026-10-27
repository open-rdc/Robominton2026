# ROBOM1 — ROS 2 Humble バドミントン簡易シミュレーション

4輪独立操舵・独立駆動（オムステ／swerve）の台車、ラケット、固定Webカメラ、
ピッチ可動2D LiDARをURDF/Xacroで構成した、最小の開発用環境です。
Ubuntu 22.04 / ROS 2 Humbleを想定しています。

**RViz表示の簡易シミュレーションです。台車は運動学、シャトルは重力・空気抵抗とラケット面の衝突反射を計算します。Gazeboなどの汎用剛体物理エンジンは使っていません。**
ビルド後にlaunchすると、コート上のシャトルが約4秒ごとに投射され、台車が追従します。
デモの台車追従とLiDAR照準には正解座標を使用します。認識による自律動作ではありません。

## パッケージ構成

ワークスペースの `src/Robominton2026/packages/` 以下に次を置いています。ここに親の `package.xml` は置きません。
colconが子の4パッケージを再帰的に検出します。

```text
robominton_2026_ws/src/Robominton2026/packages/
├── robom1_description/  # URDF/Xacro、RViz設定
├── robom1_control/      # C++: オムステ逆運動学・台車sim・目標点追従
├── robom1_sim/          # Python: シャトル、仮想カメラ、2D LiDAR、コート
└── robom1_bringup/      # Python launch、共通パラメータ
```

## ビルドと起動

ワークスペース直下でビルドします。`build/`、`install/`、`log/` は
`~/robominton_2026_ws/` に生成され、`src/` 以下にはソースだけを置きます。

```bash
cd ~/robominton_2026_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robom1_bringup sim.launch.py
```

`colcon` の出力先は標準では実行したディレクトリが基準です。
`src/Robominton2026/packages/` ではなく、常に `~/robominton_2026_ws` でビルドしてください。

別ターミナルでも同じ `setup.bash` を読み込みます。終了はlaunchした端末で `Ctrl+C`。
画面なしで動かす場合:

```bash
ros2 launch robom1_bringup sim.launch.py rviz:=false
```

RVizでは台車・コート・シャトル・LiDAR点群・Synthetic Camera画像が表示されます。
シャトルは小さく、固定カメラの画角から外れると画像に写りません。
この簡易シミュレーションは壁時計で動作するため、全ノードを `use_sim_time=false`
としており `/clock` は配信しません。

別のPCで依存が不足する場合は、Humbleをインストールした上で:

```bash
cd ~/robominton_2026_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 手動操作

デモを終了し、速度指令を手動で送るモードで起動します。
`demo:=false` では自動追従ノードが起動しないので、指令が競合しません。

```bash
ros2 launch robom1_bringup sim.launch.py demo:=false
```

別ターミナルから、左へ0.4 m/sで横移動:

```bash
source /opt/ros/humble/setup.bash
source ~/robominton_2026_ws/install/setup.bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.4}, angular: {z: 0.0}}"
```

`linear.x`: 前進、`linear.y`: 左移動、`angular.z`: 左旋回。台車座標系です。
送信を `Ctrl+C` で止めると、0.3秒の指令タイムアウト後に設定した加速度で減速停止します。
通信停止直後の瞬時停止ではありません。

LiDARを上向き約23度にする（角度はrad）:

```bash
ros2 topic pub --once /lidar_pitch/command std_msgs/msg/Float64 "{data: 0.4}"
```

ピッチ範囲は `-0.6～1.2 rad`、正方向が上向き。速度上限は1.5 rad/s。
デモ中に手動ピッチ指令を受けた場合、その起動中は自動照準を解除します。

ラケットの角度変更とシャトルの投げ直し:

```bash
ros2 topic pub --once /racket_swing/command std_msgs/msg/Float64 "{data: 0.6}"
ros2 service call /shuttle/reset std_srvs/srv/Trigger "{}"
```

ラケット範囲は `-1.8～2.0 rad`、スイング速度上限は8 rad/sです。
手動角度指定の移動速度は4 rad/sです。
ラケットは1自由度のモデルで、シャトル球とヘッド面の接触・反射を計算します。

## 目標点への移動

```bash
ros2 launch robom1_bringup sim.launch.py demo:=false follow_goal:=true
```

RVizの「2D Goal Pose」、または次の指令で指定した位置へ移動します。
目標は `odom` 座標系で、向きは無視します。

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 1.0}, orientation: {w: 1.0}}}"
```

これは位置誤差に比例した速度を出す簡単な追従器です。
障害物回避、経路探索、到達可能性判定は行いません。手動速度操作と同時に使用しないでください。
デモの目標点だけは自陣内に制限しています。台車自体に壁との衝突・コート境界制限はありません。

## センサと座標系

| トピック | 型 | 内容 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 台車の速度入力 |
| `/odom` | `nav_msgs/Odometry` | 理想運動から計算した位置・速度、100 Hz |
| `/joint_states` | `sensor_msgs/JointState` | 台車8軸とセンサ・ラケット2軸 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | odom→base_footprint→各リンク |
| `/camera/image_raw` | `sensor_msgs/Image` | 320×240 BGR、10 Hz、合成シャトル画像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 焦点距離220 px、歪みなし |
| `/scan` | `sensor_msgs/LaserScan` | 360度、0.25度刻み、0.05～20 m、10 Hz |
| `/shuttle/ground_truth` | `geometry_msgs/PointStamped` | 検証・デモ用の正解座標、50 Hz |
| `/sim/markers` | `visualization_msgs/MarkerArray` | コート・ネット・シャトル |
| `/lidar_pitch/command` | `std_msgs/Float64` | LiDARピッチ角入力 |
| `/racket_swing/command` | `std_msgs/Float64` | ラケット角入力 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 目標点追従モードの入力 |

センサ配信はSensorData QoS（Best Effort）。購読側もBest Effortに合わせてください。
`odom` の原点は自陣側ベースライン中央、xは相手コート方向、yは左方向、zは上。
ネットはx=6.7 m、初期台車位置は(1, 0) mです。
カメラ光学座標はz前・x右・y下で、URDFに光学フレームを定義しています。

LiDARは `lidar_link` のxy面を走査します。ピッチはURDFの負のy軸まわりに回転させます。
床・外周壁・半径3.5 cmのシャトル球との幾何交差の最短距離を返します。
走査面がシャトルに当たらなければ、壁・床の距離または `inf` になります。
ピッチを変えても、高速かつ小さな対象を常に測距できるわけではありません。
走査は瞬時取得で、実機の走査時間、反射率、ノイズ、遅延はモデル化していません。
ネット・台車・ラケットとのLiDAR交差および自己遮蔽も未実装です。

カメラは単純なピンホール投影でシャトルだけを描画します。背景の3Dレンダリング、
ラケットの画像、遮蔽、画像認識はありません。
シャトル運動は重力と調整用の二乗速度抵抗で近似しています。
係数は実測同定しておらず、羽根の姿勢や実際の空力を再現するものではありません。

## 寸法・制御の変更

- `robom1_description/config/robot.yaml`: 機体寸法、センサ仕様、取り付け位置・姿勢、関節制限。
- `robom1_bringup/config/sim.yaml`: 台車の速度・加速度、初期位置、投射周期などの実験条件。
- `robom1_description/urdf/robom1.urdf.xacro`: 共通YAMLからリンク・関節・慣性を生成するモデル定義。
- `robom1_sim/robom1_sim/scene.py`: 仮想センサ・シャトル・衝突の更新処理。

車輪の3寸法はlaunchでURDFとC++へ同じ値を渡しています。
**センサ位置・関節制限は `robot.yaml` の変更をURDFとsimへ共通適用します。** 詳細は [機体モデルの設定](robot_parameters.md)。
センサの初期取り付け位置はbase_footprint基準で、カメラ(0.12, 0, 1.0)、LiDAR(0.18, 0, 0.9) m。
台車の操舵は瞬時に理想角へ追従します。実際の操舵遅れ、タイヤ摩擦、モータトルクは未実装です。
URDFの慣性・collision形状は将来の物理sim用の仮値で、今の台車simには作用しません。

## C++ / Python の使い分けと次に足すもの

台車制御、逆運動学、高周期の軌道追従、実機ハードウェア接続はC++が扱いやすい構成です。
今回も台車の100 Hzループと目標追従をC++にしています。
ただし言語をC++にしただけで厳密なリアルタイム性が保証されるわけではありません。

カメラ認識の試作、学習モデルの呼び出し、戦略、実験、launchはPythonが便利です。
経路生成は最初からすべてC++にする必要はありません。試作はPythonでも行えますが、
高頻度の再計画やNav2のプラグインにする段階ではC++が適しています。

今後は、以下を独立パッケージとして `src/Robominton2026/packages/` 以下に追加する構成を推奨します。

| 追加候補 | 主な役割 | 言語の目安 |
|---|---|---|
| `robom1_perception` | カメラでシャトル・ラケット検出、画像時刻付き方位推定 | Pythonから開始 |
| `robom1_tracking` | 画像・LiDAR・TF・時刻の対応付け、3D位置/速度推定、落下点予測 | Python試作→必要部分C++ |
| `robom1_planning` | 到達可能な打点と台車・ラケット軌道の生成 | C++中心 |
| `robom1_hardware` | 駆動・操舵・ピッチ用モータ接続、エンコーダ、ros2_control連携 | C++中心 |
| `robom1_gazebo` | 接触物理、カメラ描画、打撃・シャトル空力のモデル | SDF/URDF + C++ |

次の実装順は、カメラの検出方位→LiDARピッチ制御→距離との対応付け→位置・速度推定→
打点予測→台車移動→ラケット動作、が追いやすいです。
実センサでシャトルを捕捉できるかを確かめ、正解座標のデモ入力を推定値に置き換えます。

## 検証

```bash
cd ~/robominton_2026_ws
source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
python3 src/Robominton2026/scripts/smoke_test.py
```

単体テストは車輪速度の再構成・操舵の反転最適化・台車運動積分・カメラ投影・
LiDAR走査面とシャトルの交差・遮蔽を確認します。
スモークテストはGUIなしでデモ／手動／目標追従／衝突／別機体設定の5モードを起動し、TF、センサ配信、
横移動、旋回、指令タイムアウト停止、関節指令、シャトル再投射を検証します。
テスト専用のROS_DOMAIN_ID=87とROS_LOCALHOST_ONLY=1を使い、終了時に起動したノードを停止します。

移植元では2026-10-13にHumbleで4パッケージのビルド、単体テスト7件、3モードの
スモークテスト成功を確認しました。RVizのGUI表示は目視未確認です。

設計の参考: [ROS 2 Humble公式ドキュメントのURDFチュートリアル](https://github.com/ros2/ros2_documentation/tree/humble/source/Tutorials/Intermediate/URDF)。

## ピッチ方向のラケットスイング

支点は柄の根元、回転軸は台車の+y軸です。ヘッド中心まで0.62 mの腕として、
x-z平面内を円弧状に振ります。柄とヘッドが一緒に回転し、
テイクバック約-100度→フォロースルー約109度→構え約-20度と動きます。
5次補間により各区間の始点・終点で速度と加速度が0になります。
RVizの水色の線はヘッド中心の理想軌道です。

`demo:=true` では投射ごとにスイングを繰り返します。これは動作確認用のアニメーションで、
通常デモの打撃タイミングは固定です。接触すれば反射しますが、移動しながら必ず打つ迎撃計画は未実装です。
`demo:=false` では自動スイングを停止します。手動で1回振るには同じROS環境内から:

```bash
ros2 service call /racket_swing/trigger std_srvs/srv/Trigger "{}"
```

動作中の再トリガーは受け付けません。従来の `/racket_swing/command` で角度を送ると
動作中のスイングを中断し、その起動中は自動スイングを解除します。
モデル変更を反映するにはsimとRVizを再起動してください。

## ラケット衝突と打撃後の軌道予測

PC側の端末から、衝突を確認する専用デモを起動できます。

```bash
cd ~/robominton_2026_ws/src/Robominton2026
bash scripts/foxy.sh gui impact_demo:=true
```

`impact_demo:=true` は台車の自動追従を停止し、静止台車のラケットへ向かう
シャトル供給とスイングを約4秒ごとに繰り返します。供給条件を逆積分して接触しやすく
していますが、反射や衝突通知を強制的に発生させる処理はありません。毎回の幾何判定で
接触した場合にだけ速度を更新します。通常デモではこの引数を省略してください。

| RViz表示 | 意味 |
|---|---|
| 青線 | シャトルの実際の計算軌道 |
| 緑線 | 打撃直後の状態から計算した自由飛行の予測軌道 |
| 赤い点 | ラケット上の接触位置 |
| 黄色い円 | 予測着地点 |
| 水色の円弧 | ラケットヘッド中心のスイング軌道 |

ラケットの当たり判定はURDFと同じ半径・厚さを持つ有限円柱、
シャトルは半径 `shuttle_radius` の球です。表面・裏面・縁との接触を判定します。
重なりを解消し、近づいている相対法線速度だけを反射します。
ラケット表面速度 `u` には台車の並進・旋回とピッチスイングの速度を含めます。

```text
v_out = v_in - (1 + e) * dot(v_in - u, n) * n
```

`n` は接触面の外向き単位法線、`e` は反発係数です。接線方向は摩擦なしとして保持します。
ラケットは指令軌道どおりに動く無限有効質量の物体と仮定し、打撃の反作用で減速しません。
最大2 msかつ相対移動が球半径の0.4倍以下になるよう時間刻みを細分化して、すり抜けを抑えます。

打撃後は `dv/dt = (0,0,-9.81) - shuttle_drag * |v| * v` を中点法で積分します。
予測も同じ式を使い、球中心が床から `shuttle_radius` の高さに達した点を着地点とします。
予測時間の上限は8秒で、上限内に着地しなければ着地点を出しません。
予測は打撃直後の自由飛行で、将来の再接触は予測に含みません。実際に再接触したら更新します。
床接触後のシャトルは静止させます。

| トピック | 型・内容 |
|---|---|
| `/shuttle/velocity` | `geometry_msgs/Vector3Stamped`: 現在の速度、odom座標、m/s |
| `/shuttle/hit_count` | `std_msgs/Int32`: 今回の投射からの衝突回数 |
| `/shuttle/impact_point` | `geometry_msgs/PointStamped`: 接触位置 |
| `/shuttle/impact_velocity` | `geometry_msgs/Vector3Stamped`: 打撃直後の速度、m/s |
| `/shuttle/post_hit_path` | `nav_msgs/Path`: 予測軌道、各poseの時刻は未来の予測時刻 |
| `/shuttle/predicted_landing` | `geometry_msgs/PointStamped`: 球中心の予測着地位置 |
| `/shuttle/predicted_flight_time` | `std_msgs/Float64`: 打撃から着地までの予測時間、秒 |
| `/shuttle/actual_landing` | `geometry_msgs/PointStamped`: 実際に床へ達した球中心位置 |

接触・予測トピックは接触時に配信します。再投射時は回数と予測Pathをクリアします。
前回のPointStampedを保持する購読側は、時刻とhit_countも確認してください。

同じコンテナのROS環境内で台車を止め、手動で1回供給するには:

```bash
ros2 service call /shuttle/feed_to_racket std_srvs/srv/Trigger "{}"
```

反発係数・空気抵抗の調整箇所は `robom1_bringup/config/sim.yaml` の `scene_sim.ros__parameters` です。
`racket_restitution` は0～1（初期値0.8）、`shuttle_drag` は初期値0.12です。
これらは実測同定していない仮値で、現実の飛行や打撃の精度を保証しません。
ラケット寸法・支点・固定取り付け姿勢は `robom1_description/config/robot.yaml` の
`racket` グループからURDFと衝突計算へ同じ値を渡します。支点高さはbase_footprint基準です。

羽根の姿勢、シャフト・ガットの変形、ネット・壁・台車との衝突、モータ負荷は未実装です。
特にネットは表示のみなので、予測がネットを越えるかどうかの競技判定には使えません。

数値テストは表裏の反射、空振り・縁、移動面の速度、解析的な放物運動、
高速球のすり抜け抑制、実飛行と予測の一致を確認します。
ROSの衝突テストでは接触回数・打撃後速度・予測Path・実着地を確認し、
このモデル内で予測と実着地の差が3 cm以内になることを検証します。
