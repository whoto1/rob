import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import MessageTurtleCommands

goals_for_turtle = []
COMMAND_FORWARD = 'forward'
COMMAND_LEFT = 'turn_left'
COMMAND_RIGHT = 'turn_right'

class CommandsActionClient(Node):

    # Метод инициализации класса
    def __init__(self):
        super().__init__('action_client')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'move_turtle')

    # Метод для отправки goal черепахе.
    def send_goal(self, command, s, angle):
        # Создаём goal_msg с параметрами command, s (путь)
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = s
        goal_msg.angle = angle

	# Ожидаем, когда сервер action станет доступным
        self._action_client.wait_for_server()
        
        # Отправляем асинхронную цель на сервер и устанавливаем feedback_callback для обработки обратной связи
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
	
	# Добавляем обратный вызов для обработки ответа от сервера на goal
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Метод для обработки ответа от сервера на goal
    def goal_response_callback(self, future):
    
        # Получаем результат обработки goal
        goal_handle = future.result()
        
        # Проверяем: принята или нет
        if not goal_handle.accepted:
            self.get_logger().info('Goal is rejected')
            return

	# Дошли до этого момента - задача была принята, выводим соответ. сообщение и начинаем ждать результат
        self.get_logger().info('Goal is accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Метод для обработки получения результата от сервера
    def get_result_callback(self, future):
        # Получаем результат и выводим соответ. сообщение
        result = future.result().result
        self.get_logger().info('The path is traveled: {0}'.format(result.result))
        
        global goals_for_turtle
        # Удаляем первую задачу, считая, что она была выполнена
        goals_for_turtle.pop(0)
        
        # Если в списке задач остались задачи, отправляем следующую черепахе, иначе завершает работу ROS
        if goals_for_turtle: self.send_goal(*goals_for_turtle[0])
        else: rclpy.shutdown()
        
    # Метод для обработки feedback от сервера
    def feedback_callback(self, feedback_msg):
        # Извлекаем feedback из сообщения
        feedback = feedback_msg.feedback


def main(args=None):
    # Инициализируем ROS 2 с переданными аргументами
    rclpy.init(args = args)

    action_client = CommandsActionClient()
    
    global goals_for_turtle
    
    # Добавляем задачи
    goals_for_turtle.append([COMMAND_FORWARD, 2, 0])
    goals_for_turtle.append([COMMAND_RIGHT, 0, 90])
    goals_for_turtle.append([COMMAND_FORWARD, 1, 0])
    
    # Отправили первую задачу
    action_client.send_goal(*goals_for_turtle[0])

    # Запустили бесконечный цикл обработки событий ROS, ожидая завершения action клиентом
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()