from src import image_utils
import json


def test_image_utils_imports():
    assert image_utils is not None


def test_supported_extensions_contains_expected_formats():
    assert ".jpg" in image_utils.SUPPORTED_EXTENSIONS
    assert ".jpeg" in image_utils.SUPPORTED_EXTENSIONS
    assert ".png" in image_utils.SUPPORTED_EXTENSIONS
    assert ".webp" in image_utils.SUPPORTED_EXTENSIONS
    assert ".heic" in image_utils.SUPPORTED_EXTENSIONS
    assert ".heif" in image_utils.SUPPORTED_EXTENSIONS


def test_prepare_image_for_vlm_creates_rgb_jpeg(tmp_path):
    source = tmp_path / "transparent.png"
    output_dir = tmp_path / "auto"

    image = image_utils.Image.new("RGBA", (1200, 800), (255, 0, 0, 128))
    image.save(source)

    prepared = image_utils.prepare_image_for_vlm(source, output_dir=output_dir, max_size=512)

    assert prepared.suffix == ".jpg"
    assert prepared.exists()
    with image_utils.Image.open(prepared) as result:
        assert result.format == "JPEG"
        assert result.mode == "RGB"
        assert max(result.size) <= 512


def test_prepare_image_dataset_writes_manifest(tmp_path):
    input_dir = tmp_path / "raw"
    nested_dir = input_dir / "nested"
    output_dir = tmp_path / "prepared"
    manifest = tmp_path / "manifest.jsonl"
    nested_dir.mkdir(parents=True)

    image_utils.Image.new("RGB", (32, 32), (255, 0, 0)).save(input_dir / "a.jpg")
    image_utils.Image.new("RGBA", (32, 32), (0, 255, 0, 128)).save(nested_dir / "b.png")
    (input_dir / "notes.txt").write_text("not an image", encoding="utf-8")

    stats = image_utils.prepare_image_dataset(input_dir, output_dir=output_dir, manifest_path=manifest)

    assert stats["success"] == 2
    assert stats["failed"] == 0
    assert len(stats["items"]) == 2
    lines = manifest.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    item = json.loads(lines[0])
    assert set(item) == {"source_image_path", "image_path"}
    assert image_utils.Path(item["image_path"]).exists()


def test_batch_convert_images_writes_batch_artifacts(tmp_path):
    input_dir = tmp_path / "raw"
    output_root = tmp_path / "batches"
    input_dir.mkdir()
    image_utils.Image.new("RGBA", (1200, 800), (0, 255, 0, 128)).save(input_dir / "a.png")
    (input_dir / "notes.txt").write_text("not an image", encoding="utf-8")

    result = image_utils.batch_convert_images(input_dir, output_root=output_root, max_size=256, quality=80)

    assert result["ok"] is True
    assert result["total"] == 1
    assert result["success"] == 1
    assert result["failed"] == 0
    assert image_utils.Path(result["processed_dir"]).exists()
    assert image_utils.Path(result["manifest_path"]).exists()
    assert image_utils.Path(result["summary_path"]).exists()
    assert image_utils.Path(result["errors_path"]).exists()

    manifest_item = json.loads(image_utils.Path(result["manifest_path"]).read_text(encoding="utf-8").splitlines()[0])
    assert set(manifest_item) == {"source_path", "output_path", "status", "error"}
    assert manifest_item["status"] == "success"
    assert image_utils.Path(manifest_item["output_path"]).exists()

    summary = json.loads(image_utils.Path(result["summary_path"]).read_text(encoding="utf-8"))
    assert summary["max_size"] == 256
    assert summary["quality"] == 80
