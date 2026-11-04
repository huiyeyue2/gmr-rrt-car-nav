#!/usr/bin/env python3
"""自然语言指令图形界面 —— 把输入框里的文字发到 /language/command 话题。

启动:
  ros2 run car_nav language_gui

前置条件:
  另开终端跑 demo4（或 ros2 run car_nav language_mission），本窗口才有人收听。

说明:
  本界面只是 /language/command 的发布者，任务解析与执行全在 language_mission
  节点内完成。窗口下方日志订阅 /rosout 并只显示 language_mission 的输出。
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

QUICK = [
    '去货架A看看有没有包裹',
    '去货架B检查地上有没有箱子',
    '去打包区看看有没有人',
    '巡逻一遍所有地点',
    '返回充电区',
]

BG = '#1e1f22'
PANEL = '#2b2d31'
FG = '#e6e6e6'
MUTED = '#9aa0a6'
ACCENT = '#2f7d32'
OK_GREEN = '#7fd68a'
WARN = '#e0a030'

CJK = 'Noto Sans CJK SC'


class CommandGui(Node):
    """一个节点：发布指令 + 订阅 rosout 日志。"""

    def __init__(self, log_queue):
        super().__init__('language_gui')
        self.pub = self.create_publisher(String, '/language/command', 10)
        self.sub_count = 0
        self.log_queue = log_queue
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Log, '/rosout', self.on_log, qos)
        self.create_timer(0.5, self.refresh_count)

    def refresh_count(self):
        self.sub_count = self.count_subscribers('/language/command')

    def on_log(self, msg):
        if 'language_mission' in msg.name:
            self.log_queue.put(msg.msg)

    def send(self, text):
        self.pub.publish(String(data=text))


def main():
    rclpy.init()
    log_queue = queue.Queue()
    node = CommandGui(log_queue)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('仓库机器人 · 自然语言指令台')
    root.geometry('780x560')
    root.configure(bg=BG)

    tk.Label(root, text='仓库机器人 · 自然语言指令台', bg=BG, fg=FG,
             font=(CJK, 14, 'bold')).pack(pady=(14, 2))
    tk.Label(root, text='输入中文指令，按回车或点发送；任务由 language_mission 节点执行',
             bg=BG, fg=MUTED, font=(CJK, 9)).pack()

    # ---- 输入行
    row = tk.Frame(root, bg=BG)
    row.pack(fill='x', padx=16, pady=12)

    entry = tk.Entry(row, bg=PANEL, fg=FG, insertbackground=FG,
                     relief='flat', font=(CJK, 12))
    entry.pack(side='left', fill='x', expand=True, ipady=7)
    entry.focus_set()

    def do_send(_event=None):
        text = entry.get().strip()
        if not text:
            return
        node.send(text)
        append_log('[你] ' + text, 'me')
        entry.delete(0, 'end')

    entry.bind('<Return>', do_send)

    tk.Button(row, text='发送', command=do_send, bg=ACCENT, fg='white',
              activebackground='#3c8f41', activeforeground='white',
              relief='flat', padx=20, cursor='hand2',
              font=(CJK, 11, 'bold')).pack(side='left', padx=(10, 0))

    # ---- 快捷指令
    qwrap = tk.Frame(root, bg=BG)
    qwrap.pack(fill='x', padx=16)

    for i, phrase in enumerate(QUICK):
        def make(phrase=phrase):
            node.send(phrase)
            append_log('[你] ' + phrase, 'me')

        tk.Button(qwrap, text=phrase, command=make, bg=PANEL, fg=FG,
                  activebackground='#3a3d42', activeforeground=FG,
                  relief='flat', padx=10, pady=5, cursor='hand2',
                  font=(CJK, 9)).grid(row=i // 3, column=i % 3,
                                     sticky='ew', padx=3, pady=3)

    for c in range(3):
        qwrap.columnconfigure(c, weight=1)

    # ---- 日志区
    box = tk.Frame(root, bg=BG)
    box.pack(fill='both', expand=True, padx=16, pady=(12, 6))

    text = tk.Text(box, bg='#161b1f', fg='#cfe8d2', relief='flat',
                   wrap='word', font=(CJK, 10), height=12,
                   insertbackground=FG)
    scroll = ttk.Scrollbar(box, command=text.yview)
    text.configure(yscrollcommand=scroll.set)
    scroll.pack(side='right', fill='y')
    text.pack(side='left', fill='both', expand=True)

    text.tag_config('me', foreground='#7cc4ff')
    text.tag_config('task', foreground='#cfe8d2')
    text.tag_config('err', foreground='#ff8a80')
    text.configure(state='disabled')

    def append_log(line, tag='task'):
        text.configure(state='normal')
        text.insert('end', line + '\n', tag)
        text.see('end')
        text.configure(state='disabled')

    # ---- 状态栏
    status = tk.Label(root, text='正在连接...', bg=BG, fg=MUTED, anchor='w',
                      font=(CJK, 9))
    status.pack(fill='x', padx=16, pady=(0, 10))

    def pump():
        try:
            while True:
                msg = log_queue.get_nowait()
                append_log('[任务] ' + msg, 'task')
        except queue.Empty:
            pass
        if node.sub_count > 0:
            status.configure(
                text='已连接 language_mission（订阅者 %d）' % node.sub_count,
                fg=OK_GREEN)
        else:
            status.configure(
                text='未检测到 language_mission —— 请先运行 demo4 或 '
                     'ros2 run car_nav language_mission',
                fg=WARN)
        root.after(200, pump)

    def on_close():
        try:
            node.destroy_node()
        except Exception:                                      # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.after(200, pump)
    root.mainloop()


if __name__ == '__main__':
    main()
