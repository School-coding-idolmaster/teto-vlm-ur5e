import os
from pathlib import Path

from src.image_utils import prepare_image_for_vlm


class VLMInferencer:
    """Small VLM inference wrapper with mock and local Qwen/Ollama backends."""

    def __init__(self, model_path=None, device="auto", backend="mock", image_max_size=None, image_quality=90):
        self.backend = (backend or "mock").strip().lower()
        if self.backend == "local":
            self.backend = "qwen"
        if self.backend not in {"mock", "qwen"}:
            raise ValueError("Unknown backend. Available backends: mock, qwen")

        self.model_path = model_path or os.environ.get("TETO_QWEN_MODEL")
        self.device = device
        self.image_max_size = self._resolve_image_max_size(image_max_size)
        self.image_quality = int(image_quality)
        self.model_status = "not_loaded"
        self._chat = None

    def _resolve_image_max_size(self, image_max_size):
        if image_max_size is not None:
            return int(image_max_size)
        env_value = os.environ.get("TETO_VLM_MAX_SIZE")
        if env_value:
            return int(env_value)
        if self.backend == "qwen":
            return 768
        return 1024

    def load_model(self):
        if self.backend == "mock":
            self.model_status = "mock"
            return {
                "status": self.model_status,
                "backend": self.backend,
                "model_path": self.model_path,
                "device": self.device,
            }

        try:
            from ollama import chat
        except ImportError as exc:
            raise RuntimeError(
                "Qwen backend failed while importing ollama in src/vlm_infer.py. "
                "The existing demo uses `from ollama import chat`; no model was "
                "downloaded or installed by this project. Original error: "
                f"{exc}"
            ) from exc

        self._chat = chat
        self.model_path = self.model_path or "qwen2.5vl:3b"
        self.model_status = "qwen_loaded"
        return {
            "status": self.model_status,
            "backend": self.backend,
            "model_path": self.model_path,
            "device": self.device,
        }

    def infer(self, image_path, prompt):
        path = Path(image_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Image file not found: {path}")
        if not path.is_file():
            raise ValueError(f"Image path is not a file: {path}")
        if self.model_status == "not_loaded":
            self.load_model()

        if self.backend == "qwen":
            last_error = None
            for max_size in self._qwen_image_size_candidates():
                prepared_path = prepare_image_for_vlm(
                    path,
                    max_size=max_size,
                    quality=self.image_quality,
                )
                try:
                    result = self._infer_qwen(path, prepared_path, prompt)
                    result["image_max_size"] = max_size
                    return result
                except RuntimeError as exc:
                    last_error = exc
                    if not self._is_retryable_runner_error(exc):
                        raise

            raise RuntimeError(
                f"{last_error} Retried with smaller image sizes and Ollama still stopped. "
                "Try setting TETO_VLM_MAX_SIZE=384, closing other GPU workloads, "
                "or using a smaller/quantized Ollama vision model."
            ) from last_error

        prepared_path = prepare_image_for_vlm(
            path,
            max_size=self.image_max_size,
            quality=self.image_quality,
        )

        return {
            "image_path": str(prepared_path),
            "source_image_path": str(path),
            "image_max_size": self.image_max_size,
            "prompt": prompt,
            "response": "This is a placeholder VLM response. Real model inference will be added later.",
            "backend": self.backend,
            "model_status": self.model_status,
        }

    def _qwen_image_size_candidates(self):
        candidates = [self.image_max_size]
        candidates.extend(size for size in (768, 512, 384) if size < self.image_max_size)
        unique_candidates = []
        for size in candidates:
            size = int(size)
            if size > 0 and size not in unique_candidates:
                unique_candidates.append(size)
        return unique_candidates

    @staticmethod
    def _is_retryable_runner_error(exc):
        message = str(exc).lower()
        retryable_phrases = [
            "runner has unexpectedly stopped",
            "resource limitations",
            "status code: 500",
            "out of memory",
            "oom",
        ]
        return any(phrase in message for phrase in retryable_phrases)

    def _infer_qwen(self, source_path, prepared_path, prompt):
        if self._chat is None:
            self.load_model()

        try:
            response = self._chat(
                model=self.model_path,
                messages=[
                    {
                        "role": "user",
                        "content": prompt,
                        "images": [str(prepared_path)],
                    }
                ],
            )
        except Exception as exc:
            raise RuntimeError(
                "Qwen backend failed during ollama.chat in src/vlm_infer.py. "
                f"Model: {self.model_path}. Image: {prepared_path}. Original error: {exc}"
            ) from exc

        content = self._extract_ollama_content(response)
        return {
            "image_path": str(prepared_path),
            "source_image_path": str(source_path),
            "prompt": prompt,
            "response": content,
            "backend": self.backend,
            "model_path": self.model_path,
            "model_status": self.model_status,
        }

    @staticmethod
    def _extract_ollama_content(response):
        message = getattr(response, "message", None)
        content = getattr(message, "content", None)
        if content is not None:
            return content

        if isinstance(response, dict):
            message = response.get("message", {})
            if isinstance(message, dict) and "content" in message:
                return message["content"]

        raise RuntimeError("Qwen backend returned an Ollama response without message.content.")
