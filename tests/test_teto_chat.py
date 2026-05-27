from src.teto_chat import TETOChatSession, build_teto_reply, is_exit_message


def test_teto_chat_exit_words():
    assert is_exit_message("q")
    assert is_exit_message(" Back ")
    assert not is_exit_message("hello")


def test_teto_chat_mentions_output_structure():
    reply = build_teto_reply("where are outputs saved?")

    assert "outputs/image_conversion/" in reply
    assert "outputs/results/" in reply


def test_qwen_chat_session_uses_ollama_chat():
    calls = []

    def fake_chat(model, messages):
        calls.append({"model": model, "messages": messages})
        return {"message": {"content": "real-ish qwen reply"}}

    session = TETOChatSession(backend="qwen", model_path="qwen2.5vl:3b")
    session._chat = fake_chat
    session.model_status = "qwen_loaded"

    reply = session.reply("hello teto")

    assert reply == "real-ish qwen reply"
    assert calls[0]["model"] == "qwen2.5vl:3b"
    assert calls[0]["messages"][-1] == {"role": "user", "content": "hello teto"}
    assert session.history[-2:] == [
        {"role": "user", "content": "hello teto"},
        {"role": "assistant", "content": "real-ish qwen reply"},
    ]
