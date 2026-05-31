from car_control_pkg.action_config import ACTION_MAPPINGS


def get_action_mapping(action_name: str):
    """
    根據 action_name 回傳對應的 action mapping。
    """
    velocities = ACTION_MAPPINGS[action_name]
    return velocities


def parse_control_signal(signal_str: str):
    """
    解析從 topic 收到的控制字串，格式預期為 "mode:keyboard_command"
    回傳mode, keyboard_command。
    """
    parts = [s.strip() for s in signal_str.split(":")]
    if len(parts) >= 2:
        return parts[0], parts[1].lower()
    else:
        return None, None
