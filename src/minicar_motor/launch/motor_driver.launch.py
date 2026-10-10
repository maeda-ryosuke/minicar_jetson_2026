"""motor_driver を単体で起動する（実機）。MOTOR_BENCH.md 1-2 の ros2 run と同じ引数。

    [safety_node / motor_bench] --/cmd_vel--> [motor_driver] --PWM--> 車両

    ros2 launch minicar_motor motor_driver.launch.py

    # PWM を出さずに変換だけ確かめる
    ros2 launch minicar_motor motor_driver.launch.py backend:=dryrun

raceline.launch.py と違い、backend の既定は fabo_pca9685（実機で PWM を出す）。
単体で上げる用途はベンチと実走だけなので。**車輪を浮かせてから起動する。**

raceline.launch.py も motor_driver を起動するので、同時に動かさないこと
（/cmd_vel の購読と PCA9685 への書き込みが二重になる）。

マップなど配列パラメータの一時的な上書き (throttle_map_u など) は launch 引数に
していない。必要なときは MOTOR_BENCH.md 1-2 の ros2 run に -p を足して起動する。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    motor_share = Path(get_package_share_directory("minicar_motor"))
    # 車両諸元は minicar_bringup が唯一の出所。ここへ値を写さない。
    vehicle = str(Path(get_package_share_directory("minicar_bringup"))
                  / "config/vehicle_params.yaml")

    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("vehicle_params_file", default_value=vehicle),
        DeclareLaunchArgument(
            "pwm_params_file",
            default_value=str(motor_share / "config" / "pwm_params.json")),
        # 綴りを間違えると起動時に落ちる（黙って dryrun へ落ちない）。
        DeclareLaunchArgument("backend", default_value="fabo_pca9685"),

        Node(
            package="minicar_motor",
            executable="motor_driver_node",
            name="motor_driver",
            output="screen",
            parameters=[
                str(motor_share / "config" / "motor_driver_params.yaml"),
                {"vehicle_params_file": LaunchConfiguration("vehicle_params_file"),
                 "pwm_params_file": LaunchConfiguration("pwm_params_file"),
                 "backend": LaunchConfiguration("backend"),
                 "use_sim_time": sim_time},
            ],
        ),
    ])
