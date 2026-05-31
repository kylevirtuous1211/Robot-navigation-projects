class ModeManager:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager

    def update_mode(self, pressed_key_info):
        title = pressed_key_info.split(":")[-2].replace(" ", "")
        subtitle = pressed_key_info.split(":")[-1]
        if subtitle == "down" or subtitle == "upwn":
            return
        if "Control Vehicle" in pressed_key_info:
            car_control_signal = f"{title}:{subtitle}"
            if title == "Manual_Nav" or title == "Customize_Nav":
                if subtitle == "q":
                    self.ros_manager.car_action_client.cancel_navigation_goal()
                else:
                    # 發送導航目標
                    self.ros_manager.car_action_client.send_navigation_goal(mode=title)
                    self.ros_manager.arm_action_client.send_arm_mode(mode="catch")


                    # 設置後續動作，當導航完成時會調用這個 lambda
                    # self.ros_manager.car_action_client.start_next_action(next_action)

                    print("end")
            else:
                self.ros_manager.publish_car_signal(car_control_signal)

        elif "Manual Arm Control" in pressed_key_info:
            arm_control_signal = f"{title}:{subtitle}"
            self.ros_manager.publish_arm_signal(arm_control_signal)
        elif "Automatic Arm Mode" in pressed_key_info:
            arm_control_signal = f"{title}:{subtitle}"
            if (
                title == "catch"
                or title == "catch2"
                or title == "wave"
                or title == "object_follow"
                or title == "init_pose"
                or title == "up"
                or title == "down"
                or title == "left"
                or title == "right"
                or title == "backward"
                or title == "forward"
                or title == "test"
            ):
                if subtitle == "q":
                    self.ros_manager.arm_action_client.cancel_arm()
                else:
                    self.ros_manager.arm_action_client.send_arm_mode(mode=title)

        elif "Manual Crane Control" in pressed_key_info:
            pass
        # else:
        #     print("Invalid key pressed")
