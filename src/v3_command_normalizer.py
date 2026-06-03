from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, Dict


CURRENT_V3_COMMAND_NORMALIZER_VERSION = "TETO V3.0.0"

INTENT_HOVER_TO_OBJECT = "hover_to_object"
INTENT_RETURN_HOME = "return_home"
INTENT_STOP = "stop"

E_UNSUPPORTED_CONTACT_TASK = "E_UNSUPPORTED_CONTACT_TASK"
E_TARGET_DESCRIPTION_MISSING = "E_TARGET_DESCRIPTION_MISSING"
E_AMBIGUOUS_TARGET = "E_AMBIGUOUS_TARGET"
E_UNSUPPORTED_COMMAND = "E_UNSUPPORTED_COMMAND"

SUPPORTED_INTENTS = (INTENT_HOVER_TO_OBJECT, INTENT_RETURN_HOME, INTENT_STOP)

CONTACT_PATTERNS = (
    "grasp",
    "take",
    "pick up",
    "pickup",
    "grab",
    "push",
    "press",
    "抓",
    "拿",
    "推",
    "按",
    "つかむ",
    "掴む",
)

HOVER_PATTERNS = (
    "hover over",
    "move above",
    "go above",
    "move the arm above",
    "move arm above",
    "above",
    "over",
    "悬停",
    "上方",
    "上で",
    "上に",
)

RETURN_HOME_PATTERNS = (
    "return home",
    "go home",
    "回到home",
    "回到 home",
    "ホームに戻る",
)

STOP_PATTERNS = (
    "stop",
    "halt",
    "停止",
    "止まれ",
)


@dataclass(frozen=True)
class V3NormalizedCommand:
    intent_name: str | None
    target_query: str | None
    target_label_hint: str | None
    language: str
    accepted: bool
    rejected: bool
    error_code: str | None
    normalized_command: str
    user_command: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "teto_version": CURRENT_V3_COMMAND_NORMALIZER_VERSION,
            "intent_name": self.intent_name,
            "target_query": self.target_query,
            "target_label_hint": self.target_label_hint,
            "language": self.language,
            "accepted": self.accepted,
            "rejected": self.rejected,
            "error_code": self.error_code,
            "normalized_command": self.normalized_command,
            "user_command": self.user_command,
        }


def normalize_v3_command(user_command: str | None) -> Dict[str, Any]:
    raw = user_command if isinstance(user_command, str) else ""
    normalized = _normalize_text(raw)
    language = _detect_language(raw)

    if not normalized:
        return _rejected(raw, normalized, language, E_UNSUPPORTED_COMMAND)
    if _contains_any(normalized, CONTACT_PATTERNS):
        return _rejected(raw, normalized, language, E_UNSUPPORTED_CONTACT_TASK)

    if _is_hover_command(normalized):
        target = _extract_hover_target(normalized, language)
        if not target:
            return _rejected(raw, normalized, language, E_TARGET_DESCRIPTION_MISSING, INTENT_HOVER_TO_OBJECT)
        if _has_multiple_targets(target, language):
            return _rejected(raw, normalized, language, E_AMBIGUOUS_TARGET, INTENT_HOVER_TO_OBJECT)
        return _accepted(raw, normalized, language, INTENT_HOVER_TO_OBJECT, target)

    if _contains_any(normalized, RETURN_HOME_PATTERNS):
        return _accepted(raw, normalized, language, INTENT_RETURN_HOME, None)
    if _contains_any(normalized, STOP_PATTERNS):
        return _accepted(raw, normalized, language, INTENT_STOP, None)
    return _rejected(raw, normalized, language, E_UNSUPPORTED_COMMAND)


def normalize_command(user_command: str | None) -> Dict[str, Any]:
    return normalize_v3_command(user_command)


def _accepted(
    raw: str,
    normalized: str,
    language: str,
    intent_name: str,
    target: str | None,
) -> Dict[str, Any]:
    target_hint = _target_label_hint(target)
    return V3NormalizedCommand(
        intent_name=intent_name,
        target_query=target,
        target_label_hint=target_hint,
        language=language,
        accepted=True,
        rejected=False,
        error_code=None,
        normalized_command=normalized,
        user_command=raw,
    ).to_dict()


def _rejected(
    raw: str,
    normalized: str,
    language: str,
    error_code: str,
    intent_name: str | None = None,
) -> Dict[str, Any]:
    return V3NormalizedCommand(
        intent_name=intent_name,
        target_query=None,
        target_label_hint=None,
        language=language,
        accepted=False,
        rejected=True,
        error_code=error_code,
        normalized_command=normalized,
        user_command=raw,
    ).to_dict()


def _normalize_text(value: str) -> str:
    collapsed = re.sub(r"\s+", " ", value.strip())
    return collapsed.casefold()


def _detect_language(value: str) -> str:
    if re.search(r"[\u3040-\u30ff]", value):
        return "ja"
    if re.search(r"[\u4e00-\u9fff]", value):
        return "zh"
    return "en"


def _contains_any(text: str, patterns: tuple[str, ...]) -> bool:
    return any(pattern.casefold() in text for pattern in patterns)


def _is_hover_command(text: str) -> bool:
    return _contains_any(text, HOVER_PATTERNS)


def _extract_hover_target(text: str, language: str) -> str | None:
    if language == "zh":
        target = text
        target = re.sub(r".*?(?:移动到|移動到)", "", target)
        target = re.sub(r"^(?:把)?(?:机械臂|機械臂)?", "", target)
        target = re.sub(r"(?:上方.*|悬停.*|懸停.*)$", "", target)
        return _clean_target(target, language)
    if language == "ja":
        target = re.split(r"(?:の上|上で|上に|上方)", text, maxsplit=1)[0]
        target = re.sub(r"(?:移動|動かして|止まって|戻る|ホーム)", "", target)
        return _clean_target(target, language)

    target = text
    target = re.sub(r"\band\s+stop\b.*$", "", target)
    target = re.sub(r"\b(?:please\s+)?(?:hover\s+over|move\s+the\s+arm\s+above|move\s+arm\s+above|move\s+above|go\s+above)\b", "", target)
    target = re.sub(r"^.*?\b(?:above|over)\b", "", target)
    target = re.sub(r"\b(?:the|a|an|arm|robot|ur5|and|stop|please)\b", " ", target)
    return _clean_target(target, language)


def _clean_target(value: str, language: str) -> str | None:
    target = re.sub(r"[，,.;。!！?？]", " ", value).strip()
    target = re.sub(r"\s+", " ", target).strip()
    if language in {"zh", "ja"}:
        target = target.replace(" ", "")
    return target or None


def _has_multiple_targets(target: str, language: str) -> bool:
    if language in {"zh", "ja"}:
        return bool(re.search(r"(?:和|及|与|と|や)", target))
    return bool(re.search(r"\b(?:and|or)\b|/", target))


def _target_label_hint(target: str | None) -> str | None:
    if not target:
        return None
    hint = re.sub(r"\s+", "_", target.strip())
    hint = re.sub(r"[^0-9A-Za-z_\-\u3040-\u30ff\u4e00-\u9fff]", "", hint)
    return hint or None
