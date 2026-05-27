import os
from typing import Dict, List, Tuple


EXIT_WORDS = {"q", "quit", "exit", "back"}


def is_exit_message(message: str) -> bool:
    return message.strip().lower() in EXIT_WORDS


def build_teto_reply(message: str, history: List[Tuple[str, str]] | None = None) -> str:
    text = message.strip()
    lower = text.lower()
    history = history or []

    if not text:
        return "I'm here. Type anything you want to think through, or `back` to return."

    if lower in {"help", "?", "menu"}:
        return (
            "You can chat here without choosing an image. "
            "Use `back`, `q`, `quit`, or `exit` when you want to return to the main menu."
        )

    if any(word in lower for word in ["hello", "hi", "hey", "你好", "嗨"]):
        return "Hi, I'm TETO. No image this time, just us and the terminal."

    if any(word in lower for word in ["who are you", "what are you", "你是谁"]):
        return "I'm TETO, your local VLM launcher companion. I can keep you company between conversion and recognition runs."

    if any(word in lower for word in ["output", "outputs", "result", "结果", "目录"]):
        return (
            "Current project outputs are split into image conversion results and recognition results. "
            "Conversion goes under `outputs/image_conversion/`; recognition goes under `outputs/results/`."
        )

    if any(word in lower for word in ["qwen", "backend", "model", "模型"]):
        return (
            "For model work, use `test`. This chat mode is intentionally lightweight and does not touch Qwen."
        )

    if any(word in lower for word in ["thanks", "thank you", "谢谢", "thx"]):
        return "Anytime. Tiny terminal rituals count too."

    if len(history) >= 3:
        return f"I hear you. The thread so far feels like: {text}"

    return f"I hear you: {text}"


class TETOChatSession:
    def __init__(self, backend="mock", model_path=None):
        self.backend = (backend or "mock").strip().lower()
        if self.backend == "local":
            self.backend = "qwen"
        if self.backend not in {"mock", "qwen"}:
            raise ValueError("Unknown chat backend. Available backends: mock, qwen")

        self.model_path = model_path or os.environ.get("TETO_QWEN_MODEL")
        self.model_status = "not_loaded"
        self._chat = None
        self.history: List[Dict[str, str]] = []
        self.mock_history: List[Tuple[str, str]] = []

    def load_model(self) -> Dict[str, str]:
        if self.backend == "mock":
            self.model_status = "mock"
            return {"backend": self.backend, "status": self.model_status, "model_path": self.model_path or ""}

        try:
            from ollama import chat
        except ImportError as exc:
            raise RuntimeError(
                "Qwen chat backend failed while importing ollama. "
                "No dependency or model was installed by TETO. Original error: "
                f"{exc}"
            ) from exc

        self._chat = chat
        self.model_path = self.model_path or "qwen2.5vl:3b"
        self.model_status = "qwen_loaded"
        return {"backend": self.backend, "status": self.model_status, "model_path": self.model_path}

    def reply(self, message: str) -> str:
        if self.model_status == "not_loaded":
            self.load_model()

        if self.backend == "mock":
            reply = build_teto_reply(message, self.mock_history)
            self.mock_history.append((message, reply))
            return reply

        messages = self._messages_for_qwen(message)
        try:
            response = self._chat(model=self.model_path, messages=messages)
        except Exception as exc:
            raise RuntimeError(
                "Qwen chat backend failed during ollama.chat. "
                f"Model: {self.model_path}. Original error: {exc}"
            ) from exc

        reply = self._extract_ollama_content(response)
        self.history.append({"role": "user", "content": message})
        self.history.append({"role": "assistant", "content": reply})
        return reply

    def _messages_for_qwen(self, message: str) -> List[Dict[str, str]]:
        system_message = {
            "role": "system",
            "content": (
                "You are TETO, a warm and practical assistant inside a local VLM launcher. "
                "Answer conversationally and help with the TETO project when asked."
            ),
        }
        recent_history = self.history[-12:]
        return [system_message, *recent_history, {"role": "user", "content": message}]

    @staticmethod
    def _extract_ollama_content(response) -> str:
        message = getattr(response, "message", None)
        content = getattr(message, "content", None)
        if content is not None:
            return content

        if isinstance(response, dict):
            message = response.get("message", {})
            if isinstance(message, dict) and "content" in message:
                return message["content"]

        raise RuntimeError("Qwen chat backend returned an Ollama response without message.content.")
