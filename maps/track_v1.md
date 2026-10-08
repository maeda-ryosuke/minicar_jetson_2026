# track_v1

slam_toolbox localization用の地図(2026-10-05走行から作成)。

- 元データ: bag `202610052106`(1周、0.2〜0.3 m/s)
- 作成条件(第2弾検証の H4): scan_filter 前方240deg・2 m、ループ閉じ込みなし、
  distance/angle_variance_penalty 0.03/0.03、minimum_distance/angle_penalty 0.3/0.5、
  minimum_travel_distance 0.3
- 再生成: `tools/slam_sweep/run_sweep.py --bag /bags/bags/202610052106 --extra "name=MAP_H4,..."`
  (条件は `replay_params/sweep/MAP_H4/slam.yaml`)
- 地図原点 = 10/05走行の開始位置・向き。10/06の走行も同じ位置から開始しており、
  開始姿勢はおおよそ (x, y, yaw) = (-0.09, 0.02, -0.7deg)。
- `track_v1.posegraph` と `track_v1.data` は必ず対で使う(拡張子なしの basename を指定)。

localization の評価(2026-10-07): 現行の `slam_toolbox_localization.yaml` ではこの地図でも
自己位置を見失う。評価で最良だった設定(L31)は次の通り。
distance/angle_variance_penalty 0.03/0.03、minimum_distance/angle_penalty 0.3/0.5、
minimum_travel_distance 0.3、do_loop_closing true、loop_match_minimum_chain_size 25、
loop_match_minimum_response_coarse/fine 0.5/0.6、loop_search_space_dimension 1.0、
scan_filter 前方240deg・2 m。この設定でも一致率は低速62%・高速49%で、外周の広い区間で
ずれる。
