import urwid
from pros_car_py.base_mode import BaseMode
import threading
import time


class VehicleMode(BaseMode):
    # Movement keys are "hold to drive". A terminal has no key-release event, so
    # we lean on the keyboard auto-repeat that arrives while a key is held and a
    # watchdog timer that fires once the repeats stop (i.e. you let go).
    MOVE_KEYS = {"w", "a", "s", "d", "e", "r"}
    # Seconds with no key repeat before we auto-stop. Must be a bit longer than
    # the terminal's auto-repeat interval so holding a key doesn't stutter.
    STOP_TIMEOUT = 0.3

    def enter(self):
        text = urwid.Text(
            "Vehicle Mode\n"
            "Hold a key to drive; release to stop.\n"
            "Press 'q' to return to main menu."
        )
        filler = urwid.Filler(text, valign="top")

        self._stop_alarm = None
        self.app.loop.widget = filler
        self.app.loop.unhandled_input = self.handle_input

    def _cancel_stop_alarm(self):
        if getattr(self, "_stop_alarm", None) is not None:
            self.app.loop.remove_alarm(self._stop_alarm)
            self._stop_alarm = None

    def _auto_stop(self, _loop, _user_data):
        # No key repeat arrived within STOP_TIMEOUT -> the key was released.
        self._stop_alarm = None
        self.app.car_controller.manual_control("z")  # STOP -> publishes zeros

    def handle_input(self, key):
        if key == "q":
            self._cancel_stop_alarm()
            self.app.car_controller.manual_control("z")  # ensure wheels stop
            self.app.main_menu()
            return

        self.app.car_controller.manual_control(key)

        if key in self.MOVE_KEYS:
            # (Re)arm the watchdog. Each auto-repeat keypress reschedules it;
            # once you release the key the repeats stop and STOP gets published.
            self._cancel_stop_alarm()
            self._stop_alarm = self.app.loop.set_alarm_in(
                self.STOP_TIMEOUT, self._auto_stop
            )
        else:
            # Any explicit non-movement key (e.g. 'z' stop) clears the watchdog.
            self._cancel_stop_alarm()

    def exit(self):
        self._cancel_stop_alarm()


class ArmMode(BaseMode):
    submodes = ["0", "1", "2", "3", "4"]

    def enter(self):
        self.app.horizontal_select(self.submodes, self.handle_submode_select)

    def handle_submode_select(self, submode):
        def on_key(key):
            self.app.arm_controller.manual_control(int(submode), key)

        self.show_submode_screen(
            message=f"Arm Mode: Submode {submode}\nPress 'q' to go back.", on_key=on_key
        )


class CraneMode(BaseMode):
    submodes = ["0", "1", "2", "3", "4", "5", "6", "99"]

    def enter(self):
        self.app.horizontal_select(self.submodes, self.handle_submode_select)

    def handle_submode_select(self, submode):
        def on_key(key):
            self.app.crane_controller.manual_control(int(submode), key)

        self.show_submode_screen(
            message=f"Crane Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key,
        )


class AutoNavMode(BaseMode):
    submodes = ["manual_auto_nav", "target_auto_nav", "custom_nav"]

    def enter(self):
        self.app.horizontal_select(self.submodes, self.handle_submode_select)

    def handle_submode_select(self, submode):
        def on_key(key):
            self.app.car_controller.auto_control(submode, key)
            if key == "q":
                self.app.car_controller.auto_control(submode, key=key)

        self.show_submode_screen(
            message=f"AutoNav Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key,
        )


class AutoArmMode(BaseMode):
    submodes = ["auto_arm_human"]

    def enter(self):
        self.app.horizontal_select(self.submodes, self.handle_submode_select)

    def handle_submode_select(self, submode):
        def on_key(key):
            self.app.arm_controller.auto_control(mode=submode, key=key)
            if key == "q":
                self.app.arm_controller.auto_control(mode=submode, key=key)

        self.show_submode_screen(
            message=f"AutoArm Mode: Submode {submode}\nPress 'q' to go back.",
            on_key=on_key,
        )


class Task1Mode(BaseMode):
    """Final Project Task 1 全自動任務：搜尋→靠近→觀察→夾取→Nav2 返航。"""

    def enter(self):
        self.app.task1_mission.start()
        self.show_submode_screen(
            message=(
                "Task 1 Mission running...\n"
                "(SEARCH -> APPROACH -> OBSERVE -> GRIP -> RETURN)\n"
                "See the terminal log for live state.\n\n"
                "Press 'q' to stop the mission and return to the main menu."
            ),
            on_quit=self.on_quit,
        )

    def on_quit(self):
        self.app.task1_mission.stop()
        self.app.main_menu()

    def exit(self):
        # 切換到其他模式時也確保任務停止
        self.app.task1_mission.stop()


class Task2Mode(BaseMode):
    """Final Project Task 2 全自動任務：過橋 → 取熊 → 位姿式返航。"""

    def enter(self):
        self.app.task2_mission.start()
        self.show_submode_screen(
            message=(
                "Task 2 Mission running...\n"
                "(BRIDGE_APPROACH -> SNAP_90 -> VISUAL_CLIMB (climb on the\n"
                " bridge mask) -> GRIP -> DESCEND (down the far side) -> RETURN)\n"
                "See the terminal log for live state.\n\n"
                "Press 'q' to stop the mission and return to the main menu."
            ),
            on_quit=self.on_quit,
        )

    def on_quit(self):
        self.app.task2_mission.stop()
        self.app.main_menu()

    def exit(self):
        self.app.task2_mission.stop()


class Task3Mode(BaseMode):
    """Final Project Task 3 全自動任務：門把定位觀察 → 解鎖 → 推開門。"""

    def enter(self):
        self.app.task3_mission.start()
        self.show_submode_screen(
            message=(
                "Task 3 Mission running...\n"
                "(SEARCH -> APPROACH -> OBSERVE -> UNLOCK -> CLEAR)\n"
                "See the terminal log for live state.\n\n"
                "Press 'q' to stop the mission and return to the main menu."
            ),
            on_quit=self.on_quit,
        )

    def on_quit(self):
        self.app.task3_mission.stop()
        self.app.main_menu()

    def exit(self):
        self.app.task3_mission.stop()
