"""Это код является фрагментом файла запуска для ROS 2. 
Используется для запуска узлов и других компонентов.

Мы запускаем несколько узлов, включая демонстрационные узлы из файла
turtle_tf2_demo.launch.py и узел "dynamic" с заданными параметрами.
Это используется для демонстрации и управления черепашкой"""


# Для получения пути к файлам и директориям
import os

# Для получения пути к пакетам ros2
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

# Для указания источника запуска, который необходимо включить
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# Создаём и возвращаем описание запуска ros2
def generate_launch_description():

    # Создаём экземпляр IncludeLaunchDescription,
    # который используется для включения другого файла запуска
    demo_nodes = IncludeLaunchDescription(
        # Ожидаем список путей к файлам запуска, которые нужно включить
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('carrot'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
       launch_arguments={'target_frame': 'carrot1'}.items(),
       )

    # Создание экземпляра узла ROS 2 с именем "dynamic" из пакета my_carrot
    # Узел выполняет исполняемый файл с именем "dynamic" с указанными аргументами.
    return LaunchDescription([
        demo_nodes,
        Node(
            package='carrot',
            executable='dynamic',
            name='dynamic',
            arguments=['--radius', '2', '--direction_of_rotation', '1']
        ),
    ])