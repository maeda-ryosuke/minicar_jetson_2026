"""実機の raceline_manager + motor_driver を起動する（実機）。

    raceline.csv -> [raceline_manager] --FollowPath--> controller_server(MPPI)
                                                             | /cmd_vel_raw
                                                             v
                                       [safety_node] --/cmd_vel--> [motor_driver] --PWM--> 車両

**この launch は nav2 の controller_server と safety_node を起動しない。**
前者は minicar_jetson_2026 の Dockerfile にまだ入っていない（jetson_src/README.md
の「事前に潰すこと」を参照）。後者は minicar_safety が持っている。
起動する側を 1 箇所にしておかないと、同じノードが二重に上がって
/cmd_vel の publisher が複数になる。

    ros2 launch minicar_raceline raceline.launch.py \\
      raceline_file:=/maps/raceline.csv

    # モータドライバを実際に駆動する（車輪を浮かせてから）
    ros2 launch minicar_raceline raceline.launch.py \\
      raceline_file:=/maps/raceline.csv backend:=fabo_pca9685

use_sim_time の既定は false（実機なので実時間）。minicar_jetson_2026 の
mppi.launch.py と同じ作法で、文字列の launch 引数を ParameterValue で
bool に変換して渡す。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    raceline_share = Path(get_package_share_directory("minicar_raceline"))
    motor_share = Path(get_package_share_directory("minicar_motor"))
    # 車両諸元は minicar_bringup が唯一の出所（実機リポジトリの作法）。
    # ここへ値を写さない。
    vehicle = str(Path(get_package_share_directory("minicar_bringup"))
                  / "config/vehicle_params.yaml")

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("vehicle_params_file", default_value=vehicle),
        # 参照経路。既定は compose がバインドする /maps。
        DeclareLaunchArgument("raceline_file", default_value="/maps/raceline.csv"),
        # 既定は dryrun。実機で PWM を出すときだけ fabo_pca9685 にする。
        # 綴りを間違えると起動時に落ちる（黙って dryrun へ落ちない）。
        DeclareLaunchArgument("backend", default_value="dryrun"),

        Node(
            package="minicar_raceline",
            executable="raceline_manager_node",
            name="raceline_manager",
            output="screen",
            parameters=[
                str(raceline_share / "config" / "raceline_params.yaml"),
                {"raceline_file": LaunchConfiguration("raceline_file"),
                 "use_sim_time": sim_time},
            ],
        ),
        Node(
            package="minicar_motor",
            executable="motor_driver_node",
            name="motor_driver",
            output="screen",
            parameters=[
                str(motor_share / "config" / "motor_driver_params.yaml"),
                {"vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                 # 校正値はパッケージ同梱のものを既定にする。実測したら
                 # このファイルを差し替えるか、pwm_params_file で上書きする。
                 "pwm_params_file": str(motor_share / "config" / "pwm_params.json"),
                 "backend": LaunchConfiguration("backend"),
                 "use_sim_time": sim_time},
            ],
        ),
    ])
