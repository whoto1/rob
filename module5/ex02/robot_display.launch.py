import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


# создаём и возвращаем описание запуска
def generate_launch_description():
    # ищем директорию пакета
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot').find('robot')
    # формируем путь к URDF-файлу робота путем объединения пути к директории пакета и пути к URDF-файлу
    default_model_path = os.path.join(pkg_share, 'src/description/robot.urdf.xacro')
    # формируем путь к файлу конфигурации RViz путем объединения пути к директории пакета и пути к файлу конфигурации
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # создаём узел, который отвечает за публикацию состояния робота
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    # создаём узел, который используется для управления состоянием суставов робота
    # узел не будет запущен, если значение аргумента gui установлено в True.
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # создаём узел, который предоставляет графический интерфейс для управления состоянием суставов робота.
    # узел будет запущен только в том случае, если значение аргумента gui установлено в True
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    # для визуализации робота в RViz
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    # возвращаем описание запуска, включая объявление аргументов,
    # созданные узлы и параметры, связанные с ним
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
