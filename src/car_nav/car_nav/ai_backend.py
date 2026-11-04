#!/usr/bin/env python3
"""AI 后端: GLM API(LLM 指令解析 + VLM 图像问答) 与 离线规则兜底。

配置文件查找顺序:
  1. 环境变量 AI_BACKEND_CONFIG 指向的 json
  2. ~/.ai_backend.json
  3. (无) —— 使用离线规则模式

json 格式:
  {
    "api_key": "xxxxxxxx",
    "base_url": "https://open.bigmodel.cn/api/paas/v4",   # 或 https://api.z.ai/api/paas/v4
    "llm_model": "glm-4-flash",
    "vlm_model": "glm-4v-flash"
  }

未配置 api_key 时所有函数安全降级: llm_parse_command 返回 None(调用方用规则解析),
vlm_answer 返回 (False, 提示文本)。
"""

import base64
import json
import os
import re
import urllib.request

DEFAULT_BASE_URL = 'https://open.bigmodel.cn/api/paas/v4'
DEFAULT_LLM = 'glm-4-flash'
DEFAULT_VLM = 'glm-4v-flash'
TIMEOUT = 30


# ------------------------------------------------------------------ 配置

def load_config():
    for path in (os.environ.get('AI_BACKEND_CONFIG'),
                 os.path.expanduser('~/.ai_backend.json')):
        if path and os.path.exists(path):
            try:
                with open(path, encoding='utf-8') as f:
                    cfg = json.load(f)
                if cfg.get('api_key'):
                    cfg.setdefault('base_url', DEFAULT_BASE_URL)
                    cfg.setdefault('llm_model', DEFAULT_LLM)
                    cfg.setdefault('vlm_model', DEFAULT_VLM)
                    return cfg
            except Exception:
                pass
    return None


def _chat(cfg, messages, model):
    body = json.dumps({
        'model': model,
        'messages': messages,
        'temperature': 0.1,
    }).encode('utf-8')
    req = urllib.request.Request(
        cfg['base_url'].rstrip('/') + '/chat/completions',
        data=body,
        headers={'Content-Type': 'application/json',
                 'Authorization': 'Bearer ' + cfg['api_key']})
    with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
        data = json.loads(resp.read().decode('utf-8'))
    return data['choices'][0]['message']['content']


# ------------------------------------------------------------------ LLM 解析

def llm_parse_command(text, places, cfg):
    """用 LLM 把自然语言指令解析为 {place, question, reply}。

    places: [(name, [alias...], description), ...]
    失败(网络/格式)返回 None, 由调用方回退规则解析。
    """
    place_table = [
        {'name': name, 'aliases': aliases, 'desc': desc}
        for name, aliases, desc in places
    ]
    system = (
        '你是仓库机器人任务解析器。可用地点: ' + json.dumps(
            place_table, ensure_ascii=False) + '\n'
        '把用户指令解析为 JSON, 只输出 JSON, 不要输出其他文字。字段:\n'
        '{"place": "地点名(必须是上面某个 name)", '
        '"question": "到达后用相机观察的问题, 没有则为 null", '
        '"patrol": 是否要求巡遍所有地点(bool)}')
    try:
        content = _chat(cfg, [
            {'role': 'system', 'content': system},
            {'role': 'user', 'content': text},
        ], cfg['llm_model'])
        m = re.search(r'\{.*\}', content, re.S)
        if m is None:
            return None
        out = json.loads(m.group(0))
        if out.get('place') and out['place'] not in \
                [p[0] for p in places]:
            return None
        return {'place': out.get('place'),
                'question': out.get('question') or None,
                'patrol': bool(out.get('patrol')),
                'reply': content.strip()}
    except Exception:
        return None


# ------------------------------------------------------------------ VLM 观察

def vlm_answer(jpeg_bytes, question, place_desc, cfg):
    """把相机截图发给 VLM 回答观察问题。返回 (ok, answer)。"""
    b64 = base64.b64encode(jpeg_bytes).decode('ascii')
    prompt = (
        f'这是仓库机器人在[{place_desc}]用前视相机拍到的画面。'
        f'请根据画面简要回答: {question} (50字以内, 先给结论)。')
    try:
        content = _chat(cfg, [{
            'role': 'user',
            'content': [
                {'type': 'image_url',
                 'image_url': {'url': 'data:image/jpeg;base64,' + b64}},
                {'type': 'text', 'text': prompt},
            ],
        }], cfg['vlm_model'])
        return True, content.strip()
    except Exception as e:                                  # noqa: BLE001
        return False, f'视觉接口调用失败: {e}'


# ------------------------------------------------------------------ 规则兜底

_PATROL_PAT = re.compile(r'巡逻|全部|每个地方|所有地方|挨个')
_Q_PAT = re.compile(r'(?:看看|检查|观察|瞧瞧|看下|看一下)([^,。；;]*)')


def rule_parse(text, places):
    """离线规则解析: 地名匹配 + 观察问题提取。始终可用。"""
    out = {'place': None, 'question': None, 'patrol': False,
           'reply': '', 'rule': True}
    if _PATROL_PAT.search(text):
        out['patrol'] = True
        out['reply'] = '解析为巡逻任务: 依次走遍全部地点'
        return out
    low = text.lower()
    for name, aliases, _desc in places:
        cands = [name.lower()] + [a.lower() for a in aliases]
        if any(c and c in low for c in cands):
            out['place'] = name
            break
    m = _Q_PAT.search(text)
    if m:
        frag = m.group(1).replace('有没有', '').replace('是否', '') \
            .strip(' ,，。')
        out['question'] = f'画面中能看到{frag}吗?' if frag else '画面中有什么?'
    elif re.search(r'有没有|是否|什么', text):
        out['question'] = text.strip() + ' (请根据画面回答)'
    out['reply'] = (f"[规则解析] 地点={out['place'] or '未识别'}, "
                    f"观察问题={out['question']}")
    return out
