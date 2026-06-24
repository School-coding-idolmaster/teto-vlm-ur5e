import hashlib
import json
import re
from pathlib import Path
from typing import Dict, Iterable, Optional

from PIL import Image, ImageOps

from src.output_paths import IMAGE_CONVERSION_BATCH_ROOT, create_image_conversion_batch_dir


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VLM_IMAGE_DIR = PROJECT_ROOT / "data" / "processed" / "auto"
DEFAULT_BATCH_OUTPUT_ROOT = IMAGE_CONVERSION_BATCH_ROOT
HEIF_EXTENSIONS = {".heic", ".heif"}
SUPPORTED_EXTENSIONS = {
    ".jpg",
    ".jpeg",
    ".png",
    ".webp",
    ".bmp",
    ".tif",
    ".tiff",
    ".gif",
    *HEIF_EXTENSIONS,
}
BATCH_SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp"}
SUPPORTED_OUTPUT_FORMATS = {"JPEG", "PNG", "WEBP"}
_HEIF_REGISTERED = False


def _normalize_path(path) -> Path:
    return Path(path).expanduser()


def _validate_supported_image(path: Path) -> None:
    if path.suffix.lower() not in SUPPORTED_EXTENSIONS:
        supported = ", ".join(sorted(SUPPORTED_EXTENSIONS))
        raise ValueError(f"Unsupported image format '{path.suffix}'. Supported formats: {supported}")


def _ensure_heif_support(path: Path) -> None:
    global _HEIF_REGISTERED
    if path.suffix.lower() not in HEIF_EXTENSIONS or _HEIF_REGISTERED:
        return

    try:
        from pillow_heif import register_heif_opener
    except ImportError as exc:
        raise ValueError(
            "Legacy/debug HEIC/HEIF input requires the optional dependency `pillow-heif`. "
            "Install it with `python3 -m pip install pillow-heif`, then try again."
        ) from exc

    register_heif_opener()
    _HEIF_REGISTERED = True


def load_image(path):
    image_path = _normalize_path(path)
    if not image_path.exists():
        raise FileNotFoundError(f"Image file not found: {image_path}")
    if not image_path.is_file():
        raise ValueError(f"Image path is not a file: {image_path}")
    _validate_supported_image(image_path)
    _ensure_heif_support(image_path)

    try:
        image = Image.open(image_path)
        image.load()
        return image
    except Exception as exc:
        raise ValueError(f"Failed to load image '{image_path}': {exc}") from exc


def _resize_pil_image(image: Image.Image, max_size: int) -> Image.Image:
    if max_size <= 0:
        raise ValueError("max_size must be greater than 0")

    width, height = image.size
    longest_edge = max(width, height)
    if longest_edge <= max_size:
        return image.copy()

    scale = max_size / longest_edge
    new_size = (max(1, int(width * scale)), max(1, int(height * scale)))
    resized = image.copy()
    resized.thumbnail(new_size, Image.Resampling.LANCZOS)
    return resized


def _prepare_for_format(image: Image.Image, output_format: str) -> Image.Image:
    output_format = output_format.upper()
    if output_format == "JPEG" and image.mode in {"RGBA", "LA", "P"}:
        background = Image.new("RGB", image.size, (255, 255, 255))
        if image.mode == "P":
            image = image.convert("RGBA")
        alpha = image.getchannel("A") if "A" in image.getbands() else None
        background.paste(image.convert("RGBA"), mask=alpha)
        return background
    if output_format == "JPEG" and image.mode != "RGB":
        return image.convert("RGB")
    return image


def _safe_output_stem(path: Path) -> str:
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", path.stem).strip("._")
    return stem or "image"


def _cache_key(path: Path, max_size: int, quality: int) -> str:
    stat = path.stat()
    source = f"{path.resolve()}|{stat.st_size}|{stat.st_mtime_ns}|{max_size}|{quality}"
    return hashlib.sha256(source.encode("utf-8")).hexdigest()[:12]


def prepare_image_for_vlm(input_path, output_dir=None, max_size=1024, quality=90) -> Path:
    """Create a cached JPEG/RGB image that is safe to pass to VLM backends."""
    source = _normalize_path(input_path)
    image = load_image(source)
    destination_dir = _normalize_path(output_dir) if output_dir else DEFAULT_VLM_IMAGE_DIR
    destination_dir.mkdir(parents=True, exist_ok=True)

    cache_key = _cache_key(source, max_size, quality)
    destination = destination_dir / f"{_safe_output_stem(source)}-{cache_key}.jpg"
    if destination.exists():
        return destination

    try:
        if getattr(image, "is_animated", False):
            image.seek(0)
            image.load()

        image = ImageOps.exif_transpose(image)
        resized = _resize_pil_image(image, max_size)
        prepared = _prepare_for_format(resized, "JPEG")
        if prepared.mode != "RGB":
            prepared = prepared.convert("RGB")
        prepared.save(destination, format="JPEG", quality=int(quality), optimize=True)
        return destination
    except Exception as exc:
        raise RuntimeError(f"Failed to prepare image for VLM '{input_path}': {exc}") from exc


def iter_supported_image_paths(input_dir, recursive=True) -> Iterable[Path]:
    source_dir = _normalize_path(input_dir)
    if not source_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {source_dir}")
    if not source_dir.is_dir():
        raise ValueError(f"Input path is not a directory: {source_dir}")

    paths = source_dir.rglob("*") if recursive else source_dir.iterdir()
    for path in sorted(paths):
        if path.is_file() and path.suffix.lower() in SUPPORTED_EXTENSIONS:
            yield path


def prepare_image_dataset(
    input_dir,
    output_dir=None,
    manifest_path: Optional[Path] = None,
    max_size=1024,
    quality=90,
    recursive=True,
) -> Dict[str, object]:
    """Legacy/debug RGB-folder preparation; not a RealSense snapshot dataset builder."""
    destination_dir = _normalize_path(output_dir) if output_dir else DEFAULT_VLM_IMAGE_DIR
    manifest = _normalize_path(manifest_path) if manifest_path else None
    if manifest:
        manifest.parent.mkdir(parents=True, exist_ok=True)

    stats = {
        "input_dir": str(_normalize_path(input_dir)),
        "output_dir": str(destination_dir),
        "manifest_path": str(manifest) if manifest else None,
        "success": 0,
        "failed": 0,
        "items": [],
        "errors": [],
    }

    manifest_file = manifest.open("w", encoding="utf-8") if manifest else None
    try:
        for image_path in iter_supported_image_paths(input_dir, recursive=recursive):
            try:
                prepared_path = prepare_image_for_vlm(
                    image_path,
                    output_dir=destination_dir,
                    max_size=max_size,
                    quality=quality,
                )
            except Exception as exc:
                stats["failed"] += 1
                stats["errors"].append({"source_image_path": str(image_path), "error": str(exc)})
                continue

            item = {
                "source_image_path": str(image_path),
                "image_path": str(prepared_path),
            }
            stats["success"] += 1
            stats["items"].append(item)
            if manifest_file:
                manifest_file.write(json.dumps(item, ensure_ascii=False) + "\n")
    finally:
        if manifest_file:
            manifest_file.close()

    return stats


def _format_from_output_path(output_path: Path, default: str = "JPEG") -> str:
    suffix_map = {
        ".jpg": "JPEG",
        ".jpeg": "JPEG",
        ".png": "PNG",
        ".webp": "WEBP",
    }
    return suffix_map.get(output_path.suffix.lower(), default).upper()


def resize_image(input_path, output_path, max_size=1024):
    image = load_image(input_path)
    destination = _normalize_path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)

    resized = _resize_pil_image(image, max_size)
    output_format = _format_from_output_path(destination)
    resized = _prepare_for_format(resized, output_format)
    resized.save(destination, format=output_format)
    return destination


def convert_image(input_path, output_path, format="JPEG"):
    output_format = format.upper()
    if output_format == "JPG":
        output_format = "JPEG"
    if output_format not in SUPPORTED_OUTPUT_FORMATS:
        supported = ", ".join(sorted(SUPPORTED_OUTPUT_FORMATS))
        raise ValueError(f"Unsupported output format '{format}'. Supported formats: {supported}")

    image = load_image(input_path)
    destination = _normalize_path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)

    converted = _prepare_for_format(image, output_format)
    converted.save(destination, format=output_format)
    return destination


def compress_image(input_path, output_path, max_size=1024, quality=85):
    try:
        image = load_image(input_path)
        destination = _normalize_path(output_path)
        destination.parent.mkdir(parents=True, exist_ok=True)

        output_format = _format_from_output_path(destination)
        resized = _resize_pil_image(image, max_size)
        prepared = _prepare_for_format(resized, output_format)

        save_kwargs = {}
        if output_format in {"JPEG", "WEBP"}:
            save_kwargs["quality"] = int(quality)
            save_kwargs["optimize"] = True
        if output_format == "PNG":
            save_kwargs["optimize"] = True

        prepared.save(destination, format=output_format, **save_kwargs)
        return destination
    except Exception as exc:
        raise RuntimeError(f"Failed to compress image '{input_path}' -> '{output_path}': {exc}") from exc


def batch_process_images(input_dir, output_dir, max_size=1024, quality=85) -> Dict[str, object]:
    source_dir = _normalize_path(input_dir)
    target_dir = _normalize_path(output_dir)
    if not source_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {source_dir}")
    if not source_dir.is_dir():
        raise ValueError(f"Input path is not a directory: {source_dir}")

    target_dir.mkdir(parents=True, exist_ok=True)
    stats = {"success": 0, "failed": 0, "errors": []}

    for image_path in sorted(source_dir.iterdir()):
        if not image_path.is_file() or image_path.suffix.lower() not in SUPPORTED_EXTENSIONS:
            continue
        output_path = target_dir / image_path.with_suffix(".jpg").name
        try:
            compress_image(image_path, output_path, max_size=max_size, quality=quality)
            stats["success"] += 1
        except Exception as exc:
            stats["failed"] += 1
            stats["errors"].append({"image": str(image_path), "error": str(exc)})

    return stats


def _create_batch_dir(output_root=None) -> Path:
    return create_image_conversion_batch_dir(output_root or DEFAULT_BATCH_OUTPUT_ROOT)


def _batch_output_path(source: Path, source_dir: Path, processed_dir: Path) -> Path:
    relative = source.relative_to(source_dir)
    relative_stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(relative.with_suffix(""))).strip("._")
    relative_stem = relative_stem or _safe_output_stem(source)
    digest = hashlib.sha256(str(relative).encode("utf-8")).hexdigest()[:8]
    return processed_dir / f"{relative_stem}-{digest}.jpg"


def batch_convert_images(input_dir, output_root=None, max_size=1024, quality=85) -> Dict[str, object]:
    source_dir = _normalize_path(input_dir)
    if not source_dir.exists():
        return {
            "ok": False,
            "message": f"Input directory not found: {source_dir}",
            "input_dir": str(source_dir),
        }
    if not source_dir.is_dir():
        return {
            "ok": False,
            "message": f"Input path is not a directory: {source_dir}",
            "input_dir": str(source_dir),
        }

    image_paths = [
        path
        for path in sorted(source_dir.rglob("*"))
        if path.is_file() and path.suffix.lower() in BATCH_SUPPORTED_EXTENSIONS
    ]
    if not image_paths:
        supported = ", ".join(sorted(BATCH_SUPPORTED_EXTENSIONS))
        return {
            "ok": False,
            "message": f"No supported images found in {source_dir}. Supported formats: {supported}",
            "input_dir": str(source_dir),
        }

    batch_dir = _create_batch_dir(output_root)
    processed_dir = batch_dir / "processed"
    processed_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = batch_dir / "manifest.jsonl"
    summary_path = batch_dir / "summary.json"
    errors_path = batch_dir / "errors.log"

    success = 0
    failed = 0
    manifest_items = []
    error_lines = []

    for image_path in image_paths:
        output_path = _batch_output_path(image_path, source_dir, processed_dir)
        item = {
            "source_path": str(image_path),
            "output_path": str(output_path),
            "status": "success",
            "error": "",
        }
        try:
            compress_image(image_path, output_path, max_size=max_size, quality=quality)
            success += 1
        except Exception as exc:
            failed += 1
            item["status"] = "failed"
            item["error"] = str(exc)
            error_lines.append(f"{image_path}\t{exc}")
        manifest_items.append(item)

    with manifest_path.open("w", encoding="utf-8") as manifest_file:
        for item in manifest_items:
            manifest_file.write(json.dumps(item, ensure_ascii=False) + "\n")

    summary = {
        "input_dir": str(source_dir),
        "output_dir": str(processed_dir),
        "total": len(image_paths),
        "success": success,
        "failed": failed,
        "max_size": int(max_size),
        "quality": int(quality),
    }
    with summary_path.open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, ensure_ascii=False, indent=2)
        summary_file.write("\n")

    errors_path.write_text("\n".join(error_lines) + ("\n" if error_lines else ""), encoding="utf-8")

    return {
        "ok": True,
        "batch_dir": str(batch_dir),
        "processed_dir": str(processed_dir),
        "manifest_path": str(manifest_path),
        "summary_path": str(summary_path),
        "errors_path": str(errors_path),
        **summary,
    }
