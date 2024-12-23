import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from interfaces_package.msg import RobotControl as RosRobotControl
from interfaces_package.srv import SetCommunicationMode, SetTeamColor
from protobuf_package import MoveGlobalVelocity as RosMoveGlobalVelocity
from protobuf_package import RobotCommand, RobotControl

from .communications.commands.SSLCommand import MoveGlobalVelocity
from .communications.UDPCommunication import UDPCommunication


class SimCommunicationNode(Node):
    def __init__(self):
        super().__init__("sim_communication_node")

        # Parâmetros configuráveis
        self.declare_parameter("number_of_robots", 3)
        self.declare_parameter("frequency", 100.0)
        self.declare_parameter("ip", "127.0.0.1")
        num_robots = self.get_parameter("number_of_robots").value
        self.frequency = self.get_parameter("frequency").value
        self.ip = self.get_parameter("ip").value

        self.robot_ids = list(range(num_robots))
        self.received_data = {}
        self.communication_mode = None
        self.team_color = None
        self.port = None

        self.udp_communication = UDPCommunication(self.ip, None)
        self.udp_command = MoveGlobalVelocity()

        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        qos_profile = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            RosRobotControl,
            "communication/robots_control",
            self.receive_robot_control,
            qos_profile,
        )

        self.communication_service = self.create_service(
            SetCommunicationMode,
            "communication/mode",
            self.set_communication_mode,
            callback_group=self.service_callback_group,
        )

        self.team_color_client = self.create_client(
            SetTeamColor, "communication/set_team_color"
        )

        self.timer = self.create_timer(
            1.0 / self.frequency, self.publish_aggregate_data
        )

        self.get_initial_team_color()
        self.check_color_periodically()

    def get_initial_team_color(self):
        self.team_color_client.wait_for_service()
        self._request_team_color()

    def check_color_periodically(self):
        self.create_timer(1 / 120, self._request_team_color)

    def _request_team_color(self):
        request = SetTeamColor.Request()
        request.team_color = ""
        future = self.team_color_client.call_async(request)
        future.add_done_callback(self._handle_team_color_response)

    def _handle_team_color_response(self, future):
        response = future.result()
        if response.success:
            normalized_color = response.current_color.lower()
            if self.team_color != normalized_color:
                self.team_color = normalized_color
                self._update_port_based_on_color(normalized_color)

    def _update_port_based_on_color(self, color):
        new_port = 10301 if color == "blue" else 10302 if color == "yellow" else None
        if new_port and self.port != new_port:
            self.port = new_port
            self.udp_communication.update_port(new_port)
            self.get_logger().info(
                f"Port updated to {self.port} based on team color {color}"
            )

    def receive_robot_control(self, msg):
        self.received_data[msg.robot_id] = msg

    def publish_aggregate_data(self):
        if self.communication_mode is None:
            return

        aggregate_msg = RobotControl()

        for robot_id in self.robot_ids:
            if robot_id in self.received_data:
                control_msg = self.received_data[robot_id]
            else:
                control_msg = RosRobotControl()
                control_msg.robot_id = robot_id
                control_msg.kick_speed = 0.0
                control_msg.vx = 0.0
                control_msg.vy = 0.0
                control_msg.vtheta = 0.0
                robot_command = RobotCommand()
                robot_command.id = control_msg.robot_id
                robot_command.kick_speed = control_msg.kick_speed

                move_global_velocity = RosMoveGlobalVelocity(
                    x=control_msg.vx, y=control_msg.vy, angular=control_msg.vtheta
                )
                robot_command.move_command.global_velocity.CopyFrom(
                    move_global_velocity
                )
                aggregate_msg.robot_commands.append(robot_command)
            if self.communication_mode == "udp":
                final_command = self.udp_command.generate_command(
                    robot_id,
                    control_msg.vx,
                    control_msg.vy,
                    control_msg.vtheta,
                    control_msg.kick_speed,
                )
                if self.port:
                    protobuf_data = final_command.SerializeToString()
                    self.udp_communication.send_command(protobuf_data)
                else:
                    self.get_logger().error("Port not set. Unable to send UDP command.")

    def set_communication_mode(self, request, response):
        if request.mode in ["udp", "radio"]:
            self.communication_mode = request.mode
            self.get_logger().info(
                f"Setting communication mode to {self.communication_mode}"
            )

            response.success = True
            response.message = f"Communication mode set to {self.communication_mode}"
        else:
            response.success = False
            response.message = "Invalid communication mode. Use 'udp' or 'radio'."
        return response


def main(args=None):
    rclpy.init(args=args)
    sim_transmitter_node = SimCommunicationNode()

    executor = MultiThreadedExecutor()
    executor.add_node(sim_transmitter_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        sim_transmitter_node.destroy_node()
        rclpy.shutdown()
