import importlib.util
import platform
import shutil
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _is_installed(module_name: str) -> bool:
    return importlib.util.find_spec(module_name) is not None


def _run_command(command: list[str], timeout: int = 5) -> tuple[bool, str]:
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)

    output = (completed.stdout or completed.stderr or "").strip()
    return completed.returncode == 0, output


def collect_env_info() -> dict:
    nvidia_smi_path = shutil.which("nvidia-smi")
    ollama_path = shutil.which("ollama")
    nvidia_smi_ok = False
    nvidia_smi_output = None
    ollama_server_reachable = False
    ollama_status = None

    if nvidia_smi_path:
        nvidia_smi_ok, nvidia_smi_output = _run_command([nvidia_smi_path, "-L"])

    if ollama_path:
        ollama_server_reachable, ollama_status = _run_command([ollama_path, "list"])

    info = {
        "python_version": sys.version.replace("\n", " "),
        "platform": platform.platform(),
        "current_working_directory": str(Path.cwd()),
        "project_root": str(PROJECT_ROOT),
        "pillow_installed": _is_installed("PIL"),
        "pyyaml_installed": _is_installed("yaml"),
        "ollama_python_installed": _is_installed("ollama"),
        "ollama_command": ollama_path,
        "ollama_server_reachable": ollama_server_reachable,
        "ollama_status": ollama_status,
        "nvidia_smi_command": nvidia_smi_path,
        "nvidia_smi_ok": nvidia_smi_ok,
        "nvidia_smi_output": nvidia_smi_output,
        "torch_installed": _is_installed("torch"),
        "torch_version": None,
        "cuda_available": False,
        "gpu_name": None,
        "runtime_notes": [],
    }

    if info["torch_installed"]:
        import torch

        info["torch_version"] = torch.__version__
        info["cuda_available"] = torch.cuda.is_available()
        if info["cuda_available"]:
            info["gpu_name"] = torch.cuda.get_device_name(0)

    if info["cuda_available"]:
        info["runtime_notes"].append("PyTorch CUDA is available.")
    elif info["torch_installed"]:
        info["runtime_notes"].append(
            "PyTorch is installed, but PyTorch CUDA is not available. This only affects PyTorch-based inference."
        )
    else:
        info["runtime_notes"].append(
            "PyTorch is not installed. This is OK for mock mode and the Ollama/Qwen backend."
        )

    if info["nvidia_smi_ok"]:
        info["runtime_notes"].append("NVIDIA driver check passed via nvidia-smi.")
    elif info["nvidia_smi_command"]:
        info["runtime_notes"].append(
            "nvidia-smi is installed but cannot talk to the NVIDIA driver. CUDA may not be usable until the driver/device setup is fixed."
        )
    else:
        info["runtime_notes"].append("nvidia-smi was not found, so the NVIDIA driver status was not verified.")

    if info["ollama_server_reachable"]:
        info["runtime_notes"].append("Ollama is reachable. The qwen backend can use the local Ollama server.")
    elif info["ollama_command"]:
        info["runtime_notes"].append(
            "Ollama is installed, but the local Ollama server was not reachable during this check."
        )
    else:
        info["runtime_notes"].append("Ollama command was not found. The qwen backend needs Ollama installed and running.")

    return info


def print_env_info() -> None:
    info = collect_env_info()
    print("TETO V1 Environment Check")
    print("=" * 32)
    print(f"Python version: {info['python_version']}")
    print(f"Platform: {info['platform']}")
    print(f"Current working directory: {info['current_working_directory']}")
    print(f"Project root: {info['project_root']}")
    print(f"Pillow installed: {info['pillow_installed']}")
    print(f"PyYAML installed: {info['pyyaml_installed']}")
    print(f"Ollama Python package installed: {info['ollama_python_installed']}")
    print(f"Ollama command: {info['ollama_command'] or 'not found'}")
    print(f"Ollama server reachable: {info['ollama_server_reachable']}")
    print(f"nvidia-smi command: {info['nvidia_smi_command'] or 'not found'}")
    print(f"nvidia-smi check passed: {info['nvidia_smi_ok']}")
    print(f"Torch installed: {info['torch_installed']}")
    if info["torch_installed"]:
        print(f"Torch version: {info['torch_version']}")
        print(f"CUDA available: {info['cuda_available']}")
        if info["cuda_available"]:
            print(f"GPU name: {info['gpu_name']}")
    print("Runtime notes:")
    for note in info["runtime_notes"]:
        print(f"- {note}")


def main() -> int:
    print_env_info()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
