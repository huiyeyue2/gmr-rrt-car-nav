#!/usr/bin/env python3
"""自然语言任务节点 —— 对标"打字下指令, 机器人去看并回答"演示。

交互:
  终端交互模式: ros2 run car_nav language_mission
      任务接口已就绪, 请输入地点和观察问题 (/help 查看, /quit 退出)
      > 去货架A看看有没有包裹
  话题模式(脚本/远程触发):
      ros2 topic pub --once /language/command std_msgs/msg/String \
        "{data: '去货架A看看有没有包裹'}"

流程: 指令 -> LLM解析(未配API则规则解析) -> 语义地图查坐标
      -> NavigateToPose 导航 -> 到达后相机抓拍 -> VLM 看图回答(未配API则存图)
      -> 任务日志写入 ~/mission_logs/*.json

架构说明: 全部 rclpy 事件处理在主线程 spin_once 循环中完成(输入线程只入队),
         不使用 BasicNavigator(其内部全局 spin 与订阅回调冲突)。
"""

import json
import math
import os
import queue
import sys
import threading
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import yaml

from car_nav import ai_backend

HELP = ('/help 查看帮助, /quit 退出\n'
        '指令示例:\n'
        '  去货架A看看有没有包裹\n'
        '  去打包区检查地上有没有箱子\n'
        '  巡逻一遍所有地点\n'
        '观察回答依赖视觉API: 在 ~/.ai_backend.json 配置 api_key 后自动启用,\n'
        '未配置时仅抓拍保存图像(离线模式)。')


class LanguageMission(Node):

    def __init__(self):
        super().__init__('language_mission')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('save_dir',
                               os.path.expanduser('~/mission_logs'))

        # 语义地图
        import ament_index_python
        share = ament_index_python.packages.get_package_share_directory(
            'car_nav')
        map_path = os.path.join(share, 'config', 'semantic_map.yaml')
        with open(map_path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self.place_pose = {p['name']: (p['x'], p['y'], p.get('yaw', 0.0))
                           for p in data['places']}
        self.places = [(p['name'], p.get('aliases', []),
                        p.get('description', '')) for p in data['places']]
        self.get_logger().info(
            '语义地图已加载: ' + ', '.join(self.place_pose))

        # AI 后端
        self.cfg = ai_backend.load_config()
        if self.cfg:
            self.get_logger().info(
                f"AI 后端: {self.cfg['base_url']} "
                f"(LLM={self.cfg['llm_model']}, VLM={self.cfg['vlm_model']})")
        else:
            self.get_logger().warn(
                '未配置 AI 后端(~/.ai_backend.json), 使用离线规则解析 + 存图模式')

        # 相机帧缓存
        self.bridge = CvBridge()
        self.last_image = None          # (recv_time, jpeg_bytes)
        cam_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, self.get_parameter('camera_topic').value,
            self.on_image, cam_qos)
        self.create_subscription(String, '/language/command', self.on_cmd, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        os.makedirs(self.get_parameter('save_dir').value, exist_ok=True)
        self.cmd_queue = queue.Queue()

    # ------------------------------------------------------------------ 回调
    def on_image(self, msg):
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            import cv2
            ok, buf = cv2.imencode('.jpg', cv)
            if ok:
                self.last_image = (time.time(), buf.tobytes())
        except Exception:                                   # noqa: BLE001
            pass

    def on_cmd(self, msg):
        self.cmd_queue.put(msg.data)

    # ------------------------------------------------------------------ 事件循环
    def run(self):
        print('\n=== 机器任务接口已就绪, 请输入地点和观察问题 ===')
        print(HELP + '\n')
        if sys.stdin.isatty():
            threading.Thread(target=self._stdin_loop, daemon=True).start()
        else:
            print('非交互模式, 通过 /language/command 话题触发\n')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                text = self.cmd_queue.get_nowait()
            except queue.Empty:
                continue
            if text:
                try:
                    self._handle(text.strip())
                except Exception as e:                      # noqa: BLE001
                    self.get_logger().error(f'任务执行异常: {e}')

    def _stdin_loop(self):
        while True:
            try:
                line = input('> ').strip()
            except EOFError:
                break
            if line:
                self.cmd_queue.put(line)

    # ------------------------------------------------------------------ 阻塞辅助(内部 spin_once)
    def _spin_wait(self, cond, timeout=30.0):
        t0 = time.time()
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                return False
        return rclpy.ok()

    # ------------------------------------------------------------------ 主流程
    def _handle(self, text):
        log = {'time': time.strftime('%Y-%m-%d %H:%M:%S'),
               'command': text}
        self.get_logger().info(f'【goal】: {text}')
        if text in ('/help', 'help', '帮助'):
            self.get_logger().info(HELP)
            return
        if text in ('/quit', 'exit', 'q'):
            self.get_logger().info('bye')
            raise KeyboardInterrupt

        # 1. 解析: LLM 优先, 规则兜底
        parsed = None
        llm_used = False
        if self.cfg:
            parsed = ai_backend.llm_parse_command(text, self.places, self.cfg)
            llm_used = parsed is not None
            if llm_used:
                self.get_logger().info(f'[LLM 解析] {parsed["reply"]}')
            else:
                self.get_logger().warn('LLM 解析失败, 回退规则解析')
        if parsed is None:
            parsed = ai_backend.rule_parse(text, self.places)
            self.get_logger().info(parsed['reply'])
        log['parsed'] = {k: parsed.get(k) for k in
                         ('place', 'question', 'patrol')}
        log['llm_used'] = llm_used

        # 2. 执行
        if parsed.get('patrol'):
            results = {}
            for name in self.place_pose:
                ok = self._goto(name, log)
                if not ok and rclpy.ok():
                    self.get_logger().warn(f'[{name}] 重试一次')
                    ok = self._goto(name, log)
                results[name] = ok
                if ok:
                    self._observe(name, '画面中有什么?', log)
                if not rclpy.ok():
                    return
            log['patrol_results'] = results
            self._save_log(log)
            return

        if not parsed.get('place') or parsed['place'] not in self.place_pose:
            self.get_logger().error(
                '未识别地点。可用: ' + ', '.join(self.place_pose))
            self._save_log(log)
            return
        place = parsed['place']
        ok = self._goto(place, log)
        if not ok and rclpy.ok():
            self.get_logger().warn(f'[{place}] 重试一次')
            ok = self._goto(place, log)
        log['nav_result'] = 'SUCCEEDED' if ok else 'FAILED'

        # 3. 到达后观察
        if ok and parsed.get('question'):
            self._observe(place, parsed['question'], log)
        self._save_log(log)

    # ------------------------------------------------------------------ 导航
    def _goto(self, place, log):
        x, y, yaw = self.place_pose[place]
        self.get_logger().info(f'目标位姿: (x: {x:.2f}, y: {y:.2f}, '
                               f'yaw: {yaw:.2f}) —— 已准备导航至[{place}]')

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action 不可用')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.feedback_text = None
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        if not self._spin_wait(send_future.done, timeout=10.0):
            self.get_logger().error('发送导航目标超时')
            return False
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error(f'[{place}] 导航目标被拒绝')
            return False
        result_future = handle.get_result_async()
        t0 = time.time()
        last_log = 0.0
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.feedback_text and time.time() - last_log >= 5.0:
                self.get_logger().info(f'    导航中[{place}], {self.feedback_text}')
                last_log = time.time()
        if not rclpy.ok():
            return False
        status = result_future.result().status
        dur = time.time() - t0
        ok = (status == 4)                   # SUCCEEDED
        mark = '✓' if ok else '✗'
        verdict = '完成' if ok else f'失败 (status={status})'
        self.get_logger().info(
            f'{mark} [{place}] 导航{verdict}, 耗时 {dur:.0f} 秒')
        log.setdefault('nav', []).append(
            {'place': place, 'pose': [x, y, yaw], 'ok': ok,
             'duration_s': round(dur, 1)})
        return ok

    def _on_feedback(self, fb_msg):
        self.feedback_text = f'剩余 {fb_msg.feedback.distance_remaining:.2f} m'

    # ------------------------------------------------------------------ 观察
    def _observe(self, place, question, log):
        self.get_logger().info(f'【观察】: {question}')
        self.last_image = None
        got = self._spin_wait(
            lambda: self.last_image is not None, timeout=5.0)
        save_dir = self.get_parameter('save_dir').value
        desc = dict((p[0], p[2]) for p in self.places)[place]
        stamp = time.strftime('%Y%m%d-%H%M%S')
        img_path = os.path.join(save_dir, f'observe_{place}_{stamp}.jpg')
        obs = log.setdefault('observations', {})
        if not got:
            self.get_logger().warn('没有收到相机图像, 无法观察')
            obs[place] = {'question': question, 'answer': None,
                          'error': 'no_image'}
            log['observation'] = obs[place]      # 兼容: 单地点任务读 observation
            return
        img = self.last_image[1]
        with open(img_path, 'wb') as f:
            f.write(img)
        if self.cfg:
            ok, answer = ai_backend.vlm_answer(img, question, desc, self.cfg)
            vlm_used = ok
        else:
            answer = (f'(离线模式) 已保存观察图像: {img_path}; '
                      '配置 ~/.ai_backend.json 的 api_key 后可自动看图回答')
            vlm_used = False
        self.get_logger().info(f'【观察回答】: {answer}')
        # 巡逻任务会对多个地点各观察一次, 必须按地点分别存, 否则只剩最后一个
        obs[place] = {'question': question, 'answer': answer,
                      'image': img_path, 'vlm_used': vlm_used}
        log['observation'] = obs[place]

    # ------------------------------------------------------------------ 日志
    def _save_log(self, log):
        save_dir = self.get_parameter('save_dir').value
        base = os.path.join(save_dir, time.strftime('%Y%m%d-%H%M%S'))
        path = base + '.json'
        n = 1
        while os.path.exists(path):          # 同一秒内多次保存不覆盖
            path = f'{base}-{n}.json'
            n += 1
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(log, f, ensure_ascii=False, indent=2)
        self.get_logger().info(f'[INFO] 任务日志: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = LanguageMission()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
