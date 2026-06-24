import os
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def _run(args, env):
    merged_env = os.environ.copy()
    merged_env.update(env)
    return subprocess.run(
        args,
        cwd=REPO_ROOT,
        env=merged_env,
        text=True,
        capture_output=True,
        timeout=20,
    )


def test_qwen_bootstrap_health_already_ok_does_not_start_server(tmp_path):
    start_marker = tmp_path / "started"

    completed = _run(
        ["bash", "scripts/ensure_qwen_motion_server.sh"],
        {
            "TETO_QWEN_HEALTH_CMD": "exit 0",
            "TETO_QWEN_START_CMD": f"touch {start_marker}",
            "TETO_QWEN_BOOTSTRAP_LOG": str(tmp_path / "qwen.log"),
            "TETO_QWEN_BOOTSTRAP_PID": str(tmp_path / "qwen.pid"),
            "TETO_QWEN_BOOTSTRAP_TIMEOUT_S": "1",
            "TETO_QWEN_BOOTSTRAP_POLL_S": "1",
        },
    )

    assert completed.returncode == 0, completed.stderr
    assert "already healthy" in completed.stdout
    assert not start_marker.exists()


def test_qwen_bootstrap_starts_server_and_waits_until_healthy(tmp_path):
    count_file = tmp_path / "health_count"
    start_marker = tmp_path / "started"
    health_cmd = (
        f"count=$(cat {count_file} 2>/dev/null || echo 0); "
        "count=$((count + 1)); "
        f"echo $count > {count_file}; "
        '[ "$count" -ge 2 ]'
    )

    completed = _run(
        ["bash", "scripts/ensure_qwen_motion_server.sh"],
        {
            "TETO_QWEN_HEALTH_CMD": health_cmd,
            "TETO_QWEN_START_CMD": f"touch {start_marker}; sleep 5",
            "TETO_QWEN_BOOTSTRAP_LOG": str(tmp_path / "qwen.log"),
            "TETO_QWEN_BOOTSTRAP_PID": str(tmp_path / "qwen.pid"),
            "TETO_QWEN_BOOTSTRAP_TIMEOUT_S": "3",
            "TETO_QWEN_BOOTSTRAP_POLL_S": "1",
        },
    )

    assert completed.returncode == 0, completed.stderr
    assert start_marker.exists()
    assert "healthy" in completed.stdout


def test_qwen_bootstrap_fails_when_health_never_becomes_ok(tmp_path):
    completed = _run(
        ["bash", "scripts/ensure_qwen_motion_server.sh"],
        {
            "TETO_QWEN_HEALTH_CMD": "exit 1",
            "TETO_QWEN_START_CMD": "echo start attempted; sleep 5",
            "TETO_QWEN_BOOTSTRAP_LOG": str(tmp_path / "qwen.log"),
            "TETO_QWEN_BOOTSTRAP_PID": str(tmp_path / "qwen.pid"),
            "TETO_QWEN_BOOTSTRAP_TIMEOUT_S": "1",
            "TETO_QWEN_BOOTSTRAP_POLL_S": "1",
        },
    )

    assert completed.returncode != 0
    assert "did not become healthy" in completed.stderr
    assert "start attempted" in completed.stderr


def test_manual_acceptance_auto_start_calls_bootstrap_before_dry_run(tmp_path):
    order_path = tmp_path / "order.txt"
    args_path = tmp_path / "args.txt"
    bootstrap = tmp_path / "bootstrap.sh"
    fake_python = tmp_path / "python"

    bootstrap.write_text(f"#!/usr/bin/env bash\necho bootstrap >> {order_path}\n", encoding="utf-8")
    fake_python.write_text(
        "#!/usr/bin/env bash\n"
        f"echo python >> {order_path}\n"
        f"printf '%s\\n' \"$@\" > {args_path}\n",
        encoding="utf-8",
    )
    bootstrap.chmod(0o755)
    fake_python.chmod(0o755)

    completed = _run(
        [
            "bash",
            "scripts/legacy/run_qwen_manual_acceptance.sh",
            "--cmd",
            "raise the tcp by 2 millimeters",
            "--dry-run",
            "--auto-start-qwen",
        ],
        {
            "TETO_QWEN_BOOTSTRAP_SCRIPT": str(bootstrap),
            "TETO_QWEN_ACCEPTANCE_PYTHON": str(fake_python),
        },
    )

    assert completed.returncode == 0, completed.stderr
    assert order_path.read_text(encoding="utf-8").splitlines() == ["bootstrap", "python"]
    args = args_path.read_text(encoding="utf-8").splitlines()
    assert args[:3] == ["scripts/legacy/text_to_ur5e_real_motion.py", "--acceptance", "--parser"]
    assert "qwen" in args
    assert "--dry-run" in args
    assert "--auto-start-qwen" not in args
    assert "--real-small-motion" not in args
    assert "--real" not in args


def test_manual_acceptance_no_auto_start_preserves_existing_behavior(tmp_path):
    args_path = tmp_path / "args.txt"
    bootstrap = tmp_path / "bootstrap.sh"
    fake_python = tmp_path / "python"

    bootstrap.write_text("#!/usr/bin/env bash\nexit 9\n", encoding="utf-8")
    fake_python.write_text(
        "#!/usr/bin/env bash\n"
        f"printf '%s\\n' \"$@\" > {args_path}\n",
        encoding="utf-8",
    )
    bootstrap.chmod(0o755)
    fake_python.chmod(0o755)

    completed = _run(
        [
            "bash",
            "scripts/legacy/run_qwen_manual_acceptance.sh",
            "--cmd",
            "raise the tcp by 2 millimeters",
            "--dry-run",
            "--no-auto-start-qwen",
        ],
        {
            "TETO_QWEN_BOOTSTRAP_SCRIPT": str(bootstrap),
            "TETO_QWEN_ACCEPTANCE_PYTHON": str(fake_python),
        },
    )

    assert completed.returncode == 0, completed.stderr
    args = args_path.read_text(encoding="utf-8").splitlines()
    assert "--dry-run" in args
    assert "--no-auto-start-qwen" not in args
    assert "--real-small-motion" not in args
