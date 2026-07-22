import re
from transformers import AutoModel, AutoTokenizer
import torch
import rospy
from std_msgs.msg import String
import json
import time
import threading
import sys
import base64
import io
from PIL import Image

# Terminal colors (optional, works in most terminals)
BOLD = '\033[1m'
DIM = '\033[2m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
CYAN = '\033[36m'
RESET = '\033[0m'

class Multi_RflySim:
    def __init__(self, prompt="./prompts/mission_knowledge.txt"):
        knowledge_prompt = open(prompt, "r", encoding="utf-8").read()
        self.model_file = './FM9G4B-V'
        self.model = AutoModel.from_pretrained(self.model_file, trust_remote_code=True,
                                               attn_implementation='sdpa', torch_dtype=torch.bfloat16)
        self.model = self.model.eval().cuda()
        self.tokenizer = AutoTokenizer.from_pretrained(self.model_file, trust_remote_code=True)
        self.msgs = []
        self.knowledge_prompt = knowledge_prompt
        self.feedback_sub = rospy.Subscriber('/mission_feedback', String, self.feedback_callback)
        self.vision_sub = rospy.Subscriber('/vision_query', String, self.vision_callback)
        self.vision_result_pub = rospy.Publisher('/vision_result', String, queue_size=5)
        self.last_feedback = None
        self.feedback_history = []

    def feedback_callback(self, msg):
        try:
            data = json.loads(msg.data)
            step = data.get('step', 'unknown')
            content = data.get('content', '')
            self.last_feedback = (step, content)
            self.feedback_history.append((step, content))
            # 即时输出：先换行避免干扰当前行，打印后恢复提示符视觉
            if len(content) > 120:
                content = content[:117] + "..."
            sys.stdout.write(f"\n{DIM}┌─ Feedback ───────────────────────────────────{RESET}\n")
            sys.stdout.write(f"{DIM}│{RESET} {CYAN}[{step}]{RESET} {content}\n")
            sys.stdout.write(f"{DIM}└──────────────────────────────────────────────{RESET}\n")
            sys.stdout.write(f"{BOLD}>>{RESET} ")
            sys.stdout.flush()
        except Exception:
            pass

    def vision_callback(self, msg):
        """接收视觉查询：base64图片解码后用多模态大模型分析"""
        try:
            data = json.loads(msg.data)
            query = data.get('query', '图片中出现了什么？')
            img_b64 = data.get('image_b64', '')
            rospy.loginfo(f"Vision query received: query='{query[:60]}...', b64_len={len(img_b64)}")
            if not img_b64:
                self.vision_result_pub.publish(json.dumps({"result": "no image data"}))
                return
            img_bytes = base64.b64decode(img_b64)
            image = Image.open(io.BytesIO(img_bytes)).convert('RGB')
            rospy.loginfo(f"Image decoded: {image.size}, mode={image.mode}")
            # 保存一份方便人工核对
            image.save("/tmp/mission_vision_debug_received.jpg")
        except Exception as e:
            rospy.logerr(f"Vision decode error: {e}")
            self.vision_result_pub.publish(json.dumps({"result": f"decode error: {str(e)}"}))
            return

        try:
            prompt = f"{query}\n请用简短的中文回答，不超过50字。"
            msgs = [{'role': 'user', 'content': [image, prompt]}]
            rospy.loginfo(f"Calling multimodal LLM...")
            t0 = time.time()
            res = self.model.chat(image=None, msgs=msgs, tokenizer=self.tokenizer)
            rospy.loginfo(f"Vision result ({time.time()-t0:.1f}s): {res[:200]}")
            self.vision_result_pub.publish(json.dumps({"result": res}, ensure_ascii=False))
        except Exception as e:
            rospy.logerr(f"Vision analysis error: {e}")
            self.vision_result_pub.publish(json.dumps({"result": f"error: {str(e)}"}))

    def ask(self, user_prompt):
        full_prompt = self.knowledge_prompt + user_prompt
        msgs = [{"role": "user", "content": [full_prompt]}]
        result = self.model.chat(
            image=None, msgs=msgs, tokenizer=self.tokenizer,
            sampling=True, stream=True,
        )
        # 整洁的输出格式
        print(f"\n{GREEN}┌─ LLM ────────────────────────────────────────{RESET}")
        content = ""
        for new_text in result:
            print(new_text, flush=True, end='')
            content += new_text
        print(f"\n{GREEN}└──────────────────────────────────────────────{RESET}")
        self.msgs.append({"role": "assistant", "content": content})
        return content

    def extract_code(self, text):
        bold_regex = re.compile(r"\*\*(.*?)\*\*", re.DOTALL)
        bold_blocks = bold_regex.findall(text)
        if bold_blocks:
            for block in bold_blocks:
                block = block.strip()
                if block.startswith("command"):
                    block = block[7:].strip()
                if block in ["uav_reconnaissance_mission", "result_sync",
                             "car_navigation", "thing_detect", "back_home",
                             "sequence_all"]:
                    return block
            return bold_blocks[0].strip()
        cmd_patterns = [
            "uav_reconnaissance_mission", "result_sync",
            "car_navigation", "thing_detect", "back_home",
        ]
        for cmd in cmd_patterns:
            if cmd in text:
                return cmd
        return None

    def process(self, user_input):
        if any(w in user_input for w in ["退出", "exit", "quit"]):
            return None
        if any(w in user_input for w in ["全部", "所有", "完整", "全流程", "依次执行", "顺序执行"]):
            return "sequence_all"

        response = self.ask(user_input)
        code = self.extract_code(response)
        if code:
            print(f"  {YELLOW}>>> {BOLD}{code}{RESET}")
            return code
        return response


if __name__ == "__main__":
    rospy.init_node('mission_commander')
    pub = rospy.Publisher('/mission_command', String, queue_size=10)
    multi_rflysim = Multi_RflySim()

    print(f"\n{BOLD}{'='*56}{RESET}")
    print(f"  {BOLD}Embodied Intelligence — Air-Ground Collaborative Rescue{RESET}")
    print(f"  {DIM}Commands: recon | sync | transport | verify | return{RESET}")
    print(f"  {DIM}Type full mission to execute all | 'quit' to exit{RESET}")
    print(f"{BOLD}{'='*56}{RESET}")

    while not rospy.is_shutdown():
        try:
            command = input(f"\n{BOLD}>>{RESET} ")
        except (EOFError, KeyboardInterrupt):
            break
        if not command.strip():
            continue
        if command in ["退出", "exit", "quit"]:
            break

        mission = multi_rflysim.process(command)
        if mission is None:
            continue

        if mission == "sequence_all":
            all_commands = [
                "uav_reconnaissance_mission",
                "result_sync",
                "car_navigation",
                "thing_detect",
                "back_home",
            ]
            print(f"  {YELLOW}>>> Full sequence: 5 tasks{RESET}")
            for cmd in all_commands:
                pub.publish(cmd)
                time.sleep(8.0)
        else:
            pub.publish(mission)

    print(f"\n{DIM}Shutting down...{RESET}")
