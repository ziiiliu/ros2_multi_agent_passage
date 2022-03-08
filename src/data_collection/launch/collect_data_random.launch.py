from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    uuids = [
        "robomaster_0",
        # "robomaster_1",
        # "robomaster_2",
        # "robomaster_3",
        # "robomaster_4",
    ]

    sim_params = {
        "uuids": uuids,
        "robomaster_0_initial_position": [-2.0, 2.0, 0.0],
        # "robomaster_1_initial_position": [-2.0, -2.0, 0.0],
        # "robomaster_2_initial_position": [-3.0, -1.0, 0.0],
        # "robomaster_3_initial_position": [-1.0, 0.0, 0.0],
        # "robomaster_4_initial_position": [-3.0, 1.0, 0.0],
    }

    ld = [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                "launch/rvo_robomaster.launch.yaml"
            )
        ),
            Node(
                package="simple_simulator",
                namespace="/sim",
                executable="robomaster",
                name="simulation",
                parameters=[sim_params],
            ),

            Node(
                package="data_collection",
                namespace="collecter",
                executable="state_listener",
                name="listner",
                # parameters=[sim_params],
            ),
    ]

    for uuid in uuids:
        ld.append(
            Node(
                package="data_collection",
                executable="pub_cmd_vel",
                namespace=f"publisher_{uuid}",
                # parameters=
                output="screen"

            )
        )
    return LaunchDescription(ld)
