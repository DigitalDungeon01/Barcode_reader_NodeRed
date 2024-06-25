from launch import  LaunchDescription
from launch_ros.actions  import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        Node(

            package="barcodefyp_nodered", #name of the package
            executable="barcode_reader.py",  #name of the files
            name="barcode_reader_node"  #name of the node
        ),

        Node(

            package="barcodefyp_nodered", #name of the package
            executable="node_red.py",  #name of the files
            name="node_redpub_node"  #name of the node
        )

    ])# contains list of things that we want to configure or run in the launch file


