""" Модернизированный пример из туториала в гугл-классе.

Этот код представляет собой узел для отправки динамических
трансформаций между фреймами turtle1 и carrot1"""

import math
import sys

from geometry_msgs.msg import TransformStamped

# Создание узлов их их управления
import rclpy
from rclpy.node import Node

# Для отправки трансформаций
from tf2_ros import TransformBroadcaster


# DynamicFrameBroadcaster представляет собой узел, отвечающий за бродкастинг динамических трансформаций
class DynamicFrameBroadcaster(Node):

    def __init__(self, args):
        super().__init__('dynamic_frame_tf2_broadcaster')
        
        # Создаём объект TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Устанавливаем таймер для периодической отправки трансформаций
        self.timer = self.create_timer(0.025, lambda: self.broadcast_timer_callback(args))

    # Вызываем при срабатывании таймера
    def broadcast_timer_callback(self, args):
        # Получили текущее время
        seconds, _ = self.get_clock().now().seconds_nanoseconds()

	# Создали объект TransformStamped, представляющий трансформацию от фрейма turtle1 к carrot1
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        x = seconds
        
        # Вычисляем новые значения x и y
        t.transform.translation.x = math.cos(x*float(args[4])) * float(args[2]) 
        t.transform.translation.y = math.sin(x*float(args[4])) * float(args[2]) 
        
        # Установили значения для трансформации
        
        # Трансформация представляет двухмерное положение в плоскости
        t.transform.translation.z = 0.0
        
        # xyz - ось поворота
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        # Угол вращения
        t.transform.rotation.w = 1.0

	# Отправили
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    
    node = DynamicFrameBroadcaster(sys.argv)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()