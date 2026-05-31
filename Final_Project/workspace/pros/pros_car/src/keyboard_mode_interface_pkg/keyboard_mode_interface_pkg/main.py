import urwid
import rclpy
from keyboard_mode_interface_pkg.ros_pub_sub import ROS2Manager
from keyboard_mode_interface_pkg.mode_manager import ModeManager
import os
import threading


class MenuApp:
    def __init__(self, ros_manager, mode_manager):
        self.ros_manager = ros_manager
        self.mode_manager = mode_manager

        # 這個 Text 用來顯示動態的「Pressed key: ...」訊息
        self.pressed_key_text = urwid.Text("", align="center")

        # === 定義選單結構 ===
        self.menu_items = {
            "Control Vehicle": {
                "Manual_Control": None,
                "Manual_Nav": None,
                "Auto_Nav": None,
                "Customize_Nav": None,
            },
            "Manual Arm Control": {
                "0": None,
                "1": None,
                "2": None,
                "3": None,
                "4": None,
            },
            "Automatic Arm Mode": {
                "catch": None,
                "catch2": None,
                "wave": None,
                "init_pose": None,
                "object_follow": None,
                "test": None,
                "up": None,
                "down": None,
                "left": None,
                "right": None,
                "forward": None,
                "backward": None,
            },
            "Manual Crane Control": {
                "Lift": None,
                "Lower": None,
                "Rotate Left": None,
                "Rotate Right": None,
            },
            "Exit": None,
        }

        # === 初始化主選單 ===
        self.menu_stack = []  # 用於儲存 (menu dict, menu title)
        self.current_menu = self.menu_items
        self.current_title = "Main Menu"

        # 顯示標題和底部訊息的元件
        self.header_text = urwid.Text(self.current_title, align="center")
        self.footer_text = urwid.Text("", align="center")

        # 建立主 ListBox
        self.menu = urwid.ListBox(urwid.SimpleFocusListWalker(self.create_menu()))

        # 組合成 Frame
        self.main_frame = urwid.Frame(
            self.menu, header=self.header_text, footer=self.footer_text
        )

    def create_menu(self):
        """根據 self.current_menu 建立動態選單按鈕"""
        menu_widgets = []
        for item in self.current_menu:
            button = urwid.Button(f"{item}")
            urwid.connect_signal(button, "click", self.menu_selected, item)
            menu_widgets.append(urwid.AttrMap(button, None, focus_map="reversed"))
        return menu_widgets

    def menu_selected(self, button, choice):
        """
        點擊選單選項的回呼：
        - 若對應值是 dict，就進入子選單
        - 若對應值是 None(或其他非 dict)，表示執行指令 (或沒有子選單)
        """
        if isinstance(self.current_menu[choice], dict):
            # --- 進入子選單 ---
            self.menu_stack.append((self.current_menu, self.current_title))
            self.current_menu = self.current_menu[choice]
            self.current_title = choice
            self.header_text.set_text(self.current_title)
            self.menu.body = urwid.SimpleFocusListWalker(self.create_menu())

        else:
            # --- 執行指令 / 或者該大標題本身就沒有子選單 ---
            if choice == "Exit":
                raise urwid.ExitMainLoop()  # 直接結束
            else:
                # 先發送 ROS 指令
                # 將「目前的狀態」壓到 stack，表示我們要「進入」一個結果畫面
                self.menu_stack.append((self.current_menu, self.current_title))

                # 重新定義一個「結果畫面」(視為新的子選單)
                self.current_menu = {}
                self.current_title = f"Command Result: {choice}"
                self.header_text.set_text(self.current_title)

                # 直接在畫面上顯示「指令已執行」
                self.menu.body = urwid.SimpleFocusListWalker(
                    [
                        urwid.Text(f"已執行指令: {choice}", align="center"),
                        urwid.Divider(),
                        self.pressed_key_text,
                        urwid.Divider(),
                        urwid.Text("按 'q' 回到上一頁", align="center"),
                    ]
                )

    def handle_unhandled_input(self, key):
        """處理未攔截的按鍵，特別是 'q' 用於返回上一層"""
        big_heading = self.get_big_heading()
        sub_heading = self.get_sub_heading(big_heading)
        pressed_key_info = f"{big_heading}:{sub_heading}:{key}"
        self.mode_manager.update_mode(pressed_key_info)

        if key == "q":
            os.system("clear")
            if self.menu_stack:
                # 從堆疊彈回上一層

                prev_menu, prev_title = self.menu_stack.pop()
                self.current_menu = prev_menu
                self.current_title = prev_title
                self.header_text.set_text(self.current_title)
                new_listbox = urwid.ListBox(
                    urwid.SimpleFocusListWalker(self.create_menu())
                )
                self.menu = new_listbox
                self.main_frame.body = new_listbox

            else:
                # 如果沒有上一層可回，就結束程式
                raise urwid.ExitMainLoop()

    def get_big_heading(self):
        """
        取得目前所在的「大標題」:
          - 如果 self.current_title 是主選單裡的 key (如 "Manual Arm Control")，那就是大標題
          - 否則從 stack 逆向找，找到第一個在主選單的 key，視為大標題
          - 若都沒找到，就回傳 'Main Menu'
        """
        if self.current_title in self.menu_items:
            return self.current_title

        for menu_dict, title in reversed(self.menu_stack):
            # 如果 title 剛好是主選單的 key
            if title in self.menu_items:
                return title

        return "Main Menu"

    def get_sub_heading(self, big_heading):
        """
        取得「子標題」:
          - 如果當前的標題 == big_heading，代表尚未進入子標題 (或該大標題無子選單)，則顯示 'None'
          - 否則，代表已進入某個子選項，就顯示 self.current_title
        """
        if self.current_title == big_heading:
            return "None"
        else:
            return self.current_title

    def run(self):
        """啟動 UI 迴圈"""
        palette = [
            ("reversed", "standout", ""),
            ("header", "white,bold", "dark blue"),
        ]
        loop = urwid.MainLoop(
            widget=self.main_frame,
            palette=palette,
            unhandled_input=self.handle_unhandled_input,
            handle_mouse=False,  # Disable mouse input
        )
        loop.screen.set_terminal_properties(colors=256)
        loop.run()


def main():
    rclpy.init()
    ros_manager = ROS2Manager()
    mode_manager = ModeManager(ros_manager)

    # Create and start ROS spin thread
    ros_spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_manager))
    ros_spin_thread.daemon = True
    ros_spin_thread.start()

    # Run UI in main thread
    app = MenuApp(ros_manager, mode_manager)
    try:
        app.run()
    finally:
        rclpy.shutdown()
        ros_manager.destroy_node()


if __name__ == "__main__":
    main()
