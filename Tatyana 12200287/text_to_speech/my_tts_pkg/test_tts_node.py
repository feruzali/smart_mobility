import pytest
from tts_node import TTSNode

@pytest.fixture
def tts_node():
    return TTSNode()

def test_text_to_speech(tts_node):
    text = "Testing text-to-speech functionality."
    tts_node.text_to_speech(text)

   

