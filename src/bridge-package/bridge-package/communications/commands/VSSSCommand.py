from protobuf_package import RobotControl

from .command import command


class MoveWheelVelocity(command):
    def generate_command(self, fr, br, bl, fl):
        robot_control = RobotControl()
        new_command = robot_control.robot_commands.add()
        new_command.id = self.robot_id
        wheel_velocity = new_command.move_command.wheel_velocity
        wheel_velocity.front_right = fr
        wheel_velocity.back_right = br
        wheel_velocity.back_left = bl
        wheel_velocity.front_left = fl
        return robot_control


class MoveLocalVelocity(command):
    def generate_command(self, robot_id, forward, left, angular):
        robot_control = RobotControl()
        new_command = robot_control.robot_commands.add()
        new_command.id = robot_id
        new_command.kick_speed = 3.0

        local_velocity = new_command.move_command.local_velocity
        local_velocity.forward = forward
        local_velocity.left = left
        local_velocity.angular = angular
        return robot_control


class MoveGlobalVelocity(command):
    def generate_command(self, robot_id, vx, vy, vtheta, kick_speed=None):
        robot_control = RobotControl()

        new_command = robot_control.robot_commands.add()
        if kick_speed is not None:
            new_command.kick_speed = kick_speed
        new_command.id = robot_id

        # print(new_command)

        global_velocity = new_command.move_command.global_velocity
        global_velocity.x = vx
        global_velocity.y = vy
        global_velocity.angular = vtheta
        # print(robot_control)

        return robot_control