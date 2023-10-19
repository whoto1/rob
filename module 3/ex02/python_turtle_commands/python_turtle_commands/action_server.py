import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_tutorials_interfaces.action import MessageTurtleCommands

COMMAND_FORWARD = 'forward'
COMMAND_LEFT = 'turn_left'
COMMAND_RIGHT = 'turn_right'

class CommandActionServer(Node):

    def __init__(self):
        super().__init__('action_server')
        # Создаём переменные класса для хранения:
        # скорости черепахи
        self.twist = Twist()
        
        # флага
        # flag == 0: узел ждёт обновления позиции черепахи и сохраняет предыдущую позицию
        # flag == 1: узел начинает выполнение цели и отслеживает обратную связь о перемещении черепахи
        self.flag = 0
        
        # текущей позиции
        self.after_pose = Pose()
        # предыдущей позиции 
        self.before_pose = Pose()
        
        # Создаём публикатора для сообщений типа Twist с именем топика "/turtle1/cmd_vel"
        # 10 - размер очереди сообщений
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Создаём action сервер с указанными параметрами: текущий узел (self), тип цели (MessageTurtleCommands), и именем сервера "move_turtle". Также указываем метод self.execute_callback для выполнения цели
        self._action_server = ActionServer(self, MessageTurtleCommands, 'move_turtle', self.execute_callback)
        
        # Создаём подписчика для сообщений типа Pose с именем топика "/turtle1/pose"
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)

    # Метод для обработки обновлений позиции черепахи
    def callback(self, msg):
        # Получили текущую похицию
        self.after_pose = msg
        if self.flag == 1:
            self.before_pose = msg
            self.flag = 0

    # Метод для выполнения цели, отправленной клиентом
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # Установили 1, чтобы начать отслеживать обратную связь о перемещении черепахи
        self.flag = 1
        
        # Команда вперёд
        if goal_handle.request.command == COMMAND_FORWARD:
            self.twist.linear.x = float(goal_handle.request.s)
            self.twist.angular.z = 0.0
            
        # Команда поворот налево    
        elif goal_handle.request.command == COMMAND_LEFT:
            self.twist.linear.x = 0.0
            self.twist.angular.z = float(goal_handle.request.angle) * math.pi /180
        
        # Команда поворот направо    
        elif goal_handle.request.command == COMMAND_RIGHT:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -float(goal_handle.request.angle) * math.pi /180
            
        # Публикуем сообщение о скорости движения черепахи
        self.publisher_.publish(self.twist)
        
        feedback_msg = MessageTurtleCommands.Feedback()
        feedback_msg.odom = 0
        
        # Этот цикл используется для ожидания начала движения черепахи после того, как была отправлена action цель
        # Необходим для того, чтобы убедиться, что черепаха начала двигаться перед тем, как начать измерять расстояние и предоставлять feedback клиенту
        while self.flag == 1 and (self.after_pose.linear_velocity == 0 or self.after_pose.angular_velocity == 0):
            pass
        
        # Отслеживаем движения черепахи и обновления feedback до тех пор, пока она движется
        while self.after_pose.linear_velocity != 0 or self.after_pose.angular_velocity != 0:
        
        # Обновляется feedback_msg.odom на основе расстояния между текущей и предыдущей позициями
            feedback_msg.odom = int(math.sqrt((self.after_pose.x - self.before_pose.x)**2+(self.after_pose.y - self.before_pose.y)**2))
            
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.odom))
            
            # Публикуем feedback для клиента
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()
        result = MessageTurtleCommands.Result()
        result.result = True
        
        return result


def main(args=None):
    rclpy.init(args=args)
    
    action_turtle_server = CommandActionServer()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_turtle_server)
    
    # Запустили выполнение узлов
    executor.spin()
    executor.shutdown()
    
    action_turtle_server.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()