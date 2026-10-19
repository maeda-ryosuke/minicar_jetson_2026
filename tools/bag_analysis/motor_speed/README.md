# motor_speed: 速度 PI の実走解析

motor_driver の速度 PI（`speed_control: pi`）を実走 bag で評価するためのスクリプト。
ROS は不要で、PC の Python（numpy）だけで動く。

## 手順

1. **Jetson ホスト**：小さいトピックだけを抜き出して圧縮する

   ```bash
   cd ~/workspace/workspace_2026/minicar_jetson_2026
   python3 tools/bag_analysis/motor_speed/extract_small.py      # 一番新しい bag
   gzip -f ~/bag_small/*.db3
   ```

2. **PC**：コピーして展開する

   ```powershell
   scp jetson@172.20.10.9:~/bag_small/*.gz .\bags\small\
   ```

3. **PC**：デコードして解析する（作業ディレクトリに中間ファイルができる）

   ```bash
   python -X utf8 -I recover.py bags/small/<name>_0.db3 r_<name>.pkl   # SQLite のページを直接読む（途中で切れた bag も読める）
   python -X utf8 -I decode.py r_<name>.pkl d_<name>.npz                # CDR をデコードして npz に保存
   python -X utf8 -I dips.py <name>                                     # 止まりかけた場面を、地図上の位置つきで一覧にする
   python -X utf8 -I map_fit.py <name> 24 107 128 185                   # 通常走行区間で実際に必要だった出力（マップを引き直す材料）
   ```

## /motor/speed_ctrl の列

`[v_cmd, v_meas, u_ff, u_p, u_i, u_out, state]`

state は次のいずれか。

| 値 | 意味 |
|---|---|
| 1 | PI 動作中 |
| 0 | 不使用（off、停止中、後退中） |
| -1 | /odom 途絶のため、マップの値だけで出力している |

## これまでの結果（2026-10-19）

| bag | 設定 | 実速/指令 | 実速が 0.35 m/s 未満の時間 |
|---|---|---|---|
| 202610162011_last ほか | オープンループ | 0.34〜0.38 | 24〜32% |
| 202610191124_speedpi | PI、ki 0.6、下限なし | 0.93（〜100 s） | 11% |
| 202610191200_speedpi2 | PI、ki 0.4、下限 0.25 | 1.00（通常区間） | 約 1% |

- 長い停止は、地図上の (-3.0, 3.4) 付近のコーナーでの壁への接触だった。舵いっぱいで 3 s 以上旋回した後、0.25 s で停止している。
- 必要な出力は 0.45〜0.85 m/s でほぼ一定（約 362 カウント）だった。マップは傾きが急すぎる。
