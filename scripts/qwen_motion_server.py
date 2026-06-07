#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.qwen_motion_parser import build_qwen_motion_prompt  # noqa: E402


DEFAULT_MODEL = "Qwen/Qwen2.5-VL-3B-Instruct"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 18080
DEFAULT_MAX_NEW_TOKENS = 128
DEFAULT_TEMPERATURE = 0.0
DEFAULT_TIMEOUT_S = 60.0


class QwenMotionEngine:
    def __init__(
        self,
        *,
        model_name: str,
        max_new_tokens: int,
        temperature: float,
        mock_response: str | None,
    ) -> None:
        self.model_name = model_name
        self.max_new_tokens = int(max_new_tokens)
        self.temperature = float(temperature)
        self.mock_response = mock_response
        self.mock = mock_response is not None
        self.model = None
        self.processor = None
        self.torch = None
        self.device = "mock" if self.mock else "not_loaded"
        self.cuda_available = False
        self._lock = threading.Lock()

    def load(self) -> None:
        if self.mock:
            self.device = "mock"
            return
        import torch
        from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration

        self.torch = torch
        self.cuda_available = bool(torch.cuda.is_available())
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            self.model_name,
            torch_dtype="auto",
            device_map="auto",
            local_files_only=False,
        )
        self.processor = AutoProcessor.from_pretrained(self.model_name)
        self.device = str(getattr(self.model, "device", "auto"))

    @property
    def model_loaded(self) -> bool:
        return self.mock or (self.model is not None and self.processor is not None)

    def generate(self, prompt: str, *, max_new_tokens: int | None = None, temperature: float | None = None) -> str:
        if self.mock:
            return self.mock_response or ""
        if self.model is None or self.processor is None or self.torch is None:
            raise RuntimeError("Qwen model is not loaded")
        max_new_tokens = int(max_new_tokens or self.max_new_tokens)
        temperature = self.temperature if temperature is None else float(temperature)
        messages = [{"role": "user", "content": [{"type": "text", "text": prompt}]}]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        with self._lock:
            inputs = self.processor(text=[text], return_tensors="pt").to(self.model.device)
            kwargs: dict[str, Any] = {"max_new_tokens": max_new_tokens}
            if temperature > 0.0:
                kwargs["temperature"] = temperature
                kwargs["do_sample"] = True
            else:
                kwargs["do_sample"] = False
            with self.torch.no_grad():
                generated = self.model.generate(**inputs, **kwargs)
            generated = generated[:, inputs.input_ids.shape[-1] :]
            decoded = self.processor.batch_decode(
                generated,
                skip_special_tokens=True,
                clean_up_tokenization_spaces=False,
            )
        return decoded[0].strip() if decoded else ""

    def health(self) -> dict[str, Any]:
        return {
            "status": "ok",
            "model": self.model_name,
            "model_loaded": self.model_loaded,
            "cuda_available": self.cuda_available,
            "device": self.device,
            "mock": self.mock,
        }


def make_handler(engine: QwenMotionEngine):
    class QwenMotionHandler(BaseHTTPRequestHandler):
        server_version = "TETOQwenMotionHTTP/1.0"

        def do_GET(self) -> None:
            if self.path == "/health":
                self._write_json(200, engine.health())
                return
            self._write_json(404, {"error": "not found"})

        def do_POST(self) -> None:
            if self.path == "/api/generate":
                self._handle_generate()
                return
            if self.path == "/parse_motion":
                self._handle_parse_motion()
                return
            self._write_json(404, {"error": "not found"})

        def log_message(self, format: str, *args: Any) -> None:
            sys.stderr.write("[qwen_motion_server] " + format % args + "\n")

        def _handle_generate(self) -> None:
            payload = self._read_json()
            prompt = _string(payload.get("prompt"))
            if not prompt:
                self._write_json(400, {"error": "prompt is required", "done": True})
                return
            options = payload.get("options") if isinstance(payload.get("options"), dict) else {}
            try:
                response = engine.generate(
                    prompt,
                    max_new_tokens=_optional_int(options.get("num_predict")) or _optional_int(options.get("max_new_tokens")),
                    temperature=_optional_float(options.get("temperature")),
                )
            except Exception as exc:
                self._write_json(500, {"error": str(exc), "done": True})
                return
            self._write_json(
                200,
                {
                    "model": _string(payload.get("model")) or engine.model_name,
                    "response": response,
                    "done": True,
                },
            )

        def _handle_parse_motion(self) -> None:
            payload = self._read_json()
            text = _string(payload.get("text"))
            if not text:
                self._write_json(400, {"error": "text is required", "done": True})
                return
            prompt = build_qwen_motion_prompt(text)
            try:
                response = engine.generate(prompt)
            except Exception as exc:
                self._write_json(500, {"error": str(exc), "done": True})
                return
            self._write_json(200, {"response": response, "done": True})

        def _read_json(self) -> dict[str, Any]:
            length = _optional_int(self.headers.get("Content-Length")) or 0
            if length <= 0:
                return {}
            raw = self.rfile.read(length).decode("utf-8")
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                return {}
            return payload if isinstance(payload, dict) else {}

        def _write_json(self, status: int, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, sort_keys=True).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return QwenMotionHandler


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Local HuggingFace Qwen2.5-VL text-only motion parser server.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--max-new-tokens", type=int, default=DEFAULT_MAX_NEW_TOKENS)
    parser.add_argument("--temperature", type=float, default=DEFAULT_TEMPERATURE)
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--mock-response", help="Return this response without loading a model.")
    args = parser.parse_args(argv)

    engine = QwenMotionEngine(
        model_name=args.model,
        max_new_tokens=args.max_new_tokens,
        temperature=args.temperature,
        mock_response=args.mock_response,
    )
    engine.load()
    server = ThreadingHTTPServer((args.host, int(args.port)), make_handler(engine))
    server.timeout = float(args.timeout_s)
    print(
        f"TETO Qwen motion server listening on http://{args.host}:{args.port} "
        f"model={args.model} mock={engine.mock}",
        flush=True,
    )
    print("Health: curl -s http://127.0.0.1:18080/health", flush=True)
    print("Dry-run: bash scripts/run_text_to_ur5e_dry_run.sh", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def _string(value: Any) -> str | None:
    return value.strip() if isinstance(value, str) and value.strip() else None


def _optional_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _optional_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


if __name__ == "__main__":
    raise SystemExit(main())
