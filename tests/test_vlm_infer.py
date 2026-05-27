from PIL import Image

from src import image_utils
import src.vlm_infer as vlm_infer
from src.vlm_infer import VLMInferencer


def test_infer_prepares_image_before_mock_result(tmp_path, monkeypatch):
    source = tmp_path / "input.png"
    Image.new("RGBA", (16, 16), (0, 255, 0, 100)).save(source)

    def prepare_in_tmp(path, max_size=1024, quality=90):
        return image_utils.prepare_image_for_vlm(path, output_dir=tmp_path / "auto", max_size=max_size, quality=quality)

    monkeypatch.setattr(vlm_infer, "prepare_image_for_vlm", prepare_in_tmp)
    result = VLMInferencer(backend="mock").infer(source, "describe")

    assert result["source_image_path"] == str(source)
    assert result["image_path"].endswith(".jpg")


def test_qwen_retries_with_smaller_image_after_runner_error(tmp_path, monkeypatch):
    source = tmp_path / "input.png"
    Image.new("RGB", (16, 16), (0, 255, 0)).save(source)
    prepared_sizes = []
    chat_calls = []

    def prepare_in_tmp(path, max_size=1024, quality=90):
        prepared_sizes.append(max_size)
        return image_utils.prepare_image_for_vlm(path, output_dir=tmp_path / "auto", max_size=max_size, quality=quality)

    def fake_chat(model, messages):
        chat_calls.append(messages[0]["images"][0])
        if len(chat_calls) == 1:
            raise RuntimeError("model runner has unexpectedly stopped due to resource limitations")
        return {"message": {"content": "ok"}}

    monkeypatch.setattr(vlm_infer, "prepare_image_for_vlm", prepare_in_tmp)
    inferencer = VLMInferencer(backend="qwen", image_max_size=768)
    inferencer._chat = fake_chat
    inferencer.model_status = "qwen_loaded"

    result = inferencer.infer(source, "describe")

    assert prepared_sizes == [768, 512]
    assert result["response"] == "ok"
    assert result["image_max_size"] == 512
