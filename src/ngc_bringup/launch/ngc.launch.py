import os
import launch
from launch import LaunchDescription
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessStart
import yaml

#: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/ 
def generate_launch_description():
    
     # Path to the simulator_config.yaml file
    simulator_config_file = os.path.join(
        get_package_share_directory('ngc_bringup'),
        'config',
        'simulator_config.yaml'
    )
    
    # Load the YAML file
    with open(simulator_config_file, 'r') as file:
        simulator_config = yaml.safe_load(file)
    
    # Check if the 'simulator_in_the_loop' flag is True
    simulator_in_the_loop = simulator_config.get('simulator_in_the_loop', False)

    plotjuggler_config = os.path.join(
        get_package_share_directory('ngc_bringup'),
        'config',
        'PlotJuggler_layout.xml'
    )
    
    sim_node = Node(
        package    = "ngc_hull_sim", 
        executable = "simulate",
        name       = 'ngc_hull_sim',
        #output    = 'screen'
    )

    propulsion_node = Node(
        package    = "ngc_propulsion_sim", 
        executable = "ngc_propulsion_sim",
        name       = 'ngc_propulsion_sim',
        output     = 'screen'
    )
    
    gnss_node = Node(
        package    = "ngc_sensor_sims", 
        executable = "gnss",
        name       = 'gnss'
    )
    
    compass_node = Node(
        package    = "ngc_sensor_sims", 
        executable = "compass",
        name       = 'compass'
    )

    hmi_node_autopilot = Node(
        package     = "ngc_hmi", 
        executable  = "ngc_hmi_autopilot",
        name        = 'hmi_ap',
        output     = 'screen'
    )

    plotjuggler_node = Node(
        package    = "plotjuggler", 
        executable = "plotjuggler",
        arguments  = ["--layout",  plotjuggler_config] 
    )

    hmi_node_yaml_editor = Node(
        package     = "ngc_hmi", 
        executable  = "ngc_hmi_yaml_editor",
        name        = 'hmi_editor',
        output     = 'screen'
    )

    kontroller_node = Node(
        package     = "ControlSystem", 
        executable  = "kontroller",
        name        = 'kontroller',
        output     = 'screen'
    )
    
    estimator_node = Node(
        package     = "ControlSystem", 
        executable  = "estimator",
        name        = 'estimator',
        output     = 'screen'
    )

    waypoint_node = Node(
        package     = "ControlSystem", 
        executable  = "waypoint_mottaker",
        name        = 'waypoint_mottaker',
        output     = 'screen'
    )

    allokator_node = Node(
        package     = "ControlSystem", 
        executable  = "allokator",
        name        = 'allokator',
        output     = 'screen'
    )

    otter_interface = Node(
        package     = "ngc_otter_interface", 
        executable  = "otter_interface",    
        name        = 'otter_interface',
        output     = 'screen'
    )

    delayed_plotjuggler= TimerAction(period= 3.0, actions=[plotjuggler_node])
    delayed_autopilot_hmi = TimerAction(period= 1.0, actions=[hmi_node_autopilot])

    ld = LaunchDescription() 
    
    ld.add_action(otter_interface)
    #ld.add_action(estimator_node)
    #ld.add_action(kontroller_node)
    #ld.add_action(allokator_node)
    #ld.add_action(hmi_node_yaml_editor)
    ld.add_action(delayed_autopilot_hmi)
    ld.add_action(delayed_plotjuggler)
    #ld.add_action(waypoint_node)

    if simulator_in_the_loop:
        ld.add_action(sim_node)
        ld.add_action(gnss_node)
        ld.add_action(compass_node)
        ld.add_action(propulsion_node)
    

    return ld
