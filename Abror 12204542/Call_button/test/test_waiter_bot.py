# test_waiter_bot.py

import pytest
from std_msgs.msg import String
from waiter_bot import WaiterBot

@pytest.fixture
def mock_waiter_bot():
    # Fixture to create a mock WaiterBot with test topics
    return WaiterBot('test_speech_topic', 'test_button_topic')

def test_speech_to_text_callback(capfd, mock_waiter_bot):
    # Test the speech_to_text_callback method
    mock_waiter_bot.speech_to_text_callback(String(data='Test Order'))
    captured = capfd.readouterr()
    assert 'Customer order: Test Order' in captured.out

def test_call_waiter_callback(capfd, mock_waiter_bot):
    # Test the call_waiter_callback method
    mock_waiter_bot.call_waiter_callback(String(data='pressed'))
    captured = capfd.readouterr()
    assert 'move_to_customer_location' in captured.out


