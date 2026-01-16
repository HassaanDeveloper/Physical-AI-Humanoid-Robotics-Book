# Isaac Sim Synthetic Data Generation Launch File
# Launch file for educational synthetic data generation examples

import sys
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Generate launch description for synthetic data generation."""

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('isaac_tutorials'),
            'config',
            'synthetic_data_gen.yaml'
        ),
        description='Path to synthetic data generation configuration file'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='./synthetic_data_output',
        description='Directory to save generated synthetic data'
    )

    scene_file_arg = DeclareLaunchArgument(
        'scene_file',
        default_value=os.path.join(
            get_package_share_directory('isaac_tutorials'),
            'environments',
            'basic_humanoid_env.usd'
        ),
        description='USD file containing the simulation scene'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    output_directory = LaunchConfiguration('output_directory')
    scene_file = LaunchConfiguration('scene_file')

    # Create synthetic data generation node
    synthetic_data_node = Node(
        package='omni.isaac.synthetic_utils',
        executable='synthetic_data_generator',
        name='synthetic_data_generator',
        parameters=[
            {'config_file': config_file},
            {'output_directory': output_directory},
            {'scene_file': scene_file},
            {'enable_domain_randomization': True},
            {'batch_size': 4},
            {'max_frames': 100}
        ],
        output='screen',
        on_exit=Shutdown()
    )

    # Camera calibration node
    camera_calibrator = Node(
        package='isaac_ros_calibration',
        executable='camera_calibrator',
        name='camera_calibrator',
        parameters=[
            {'camera_model': 'pinhole'},
            {'width': 640},
            {'height': 480},
            {'fx': 320.0},
            {'fy': 320.0},
            {'cx': 320.0},
            {'cy': 240.0}
        ],
        output='screen'
    )

    # Annotation processor node
    annotation_processor = Node(
        package='isaac_ros_synthetic_data',
        executable='annotation_processor',
        name='annotation_processor',
        parameters=[
            {'input_format': 'coco'},
            {'output_format': 'kitti'},
            {'class_mapping': {'humanoid': 1, 'cube': 2, 'sphere': 3, 'cylinder': 4}}
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        output_dir_arg,
        scene_file_arg,
        synthetic_data_node,
        camera_calibrator,
        annotation_processor
    ])


# Educational example function for students to understand the process
def run_synthetic_data_generation_example():
    """
    Example function demonstrating synthetic data generation workflow.
    This is for educational purposes to show students how the process works.
    """
    print("=== Isaac Sim Synthetic Data Generation Example ===")
    print("This example demonstrates the key steps in synthetic data generation:")

    # Step 1: Load configuration
    config_path = "./config/synthetic_data_gen.yaml"
    print(f"1. Loading configuration from: {config_path}")

    # Step 2: Load scene
    scene_path = "./environments/basic_humanoid_env.usd"
    print(f"2. Loading scene from: {scene_path}")

    # Step 3: Configure data types
    data_types = ["RGB", "Depth", "Semantic Segmentation", "Bounding Boxes"]
    print(f"3. Configuring data types: {', '.join(data_types)}")

    # Step 4: Apply domain randomization
    print("4. Applying domain randomization techniques")

    # Step 5: Generate data
    print("5. Generating synthetic data frames...")
    print("   - Frame 1/100: Basic scene")
    print("   - Frame 2/100: Adjusted lighting")
    print("   - Frame 3/100: Modified object positions")
    print("   - ...")

    # Step 6: Save output
    output_dir = "./synthetic_data_output"
    print(f"6. Saving output to: {output_dir}")

    print("\nSynthetic data generation complete!")
    print("Generated data includes:")
    print("- RGB images for computer vision")
    print("- Depth maps for 3D reconstruction")
    print("- Semantic segmentation masks")
    print("- Bounding box annotations")
    print("- Instance segmentation masks")


if __name__ == '__main__':
    # For educational purposes, we can run the example function
    # when this file is executed directly
    if len(sys.argv) > 1 and sys.argv[1] == '--example':
        run_synthetic_data_generation_example()
    else:
        # Otherwise, generate the launch description for ROS
        pass