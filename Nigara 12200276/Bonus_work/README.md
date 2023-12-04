# Voice-Activated Menu Order System

This is a bonus task that combines three functions: Speech-To-Text, Text-To-Speech, and Order_Confirmation. It was implemented for a better understanding of how our user_interaction functions work together.

This Python script implements a voice-activated menu order system. Users can place food orders by speaking into a microphone, and the system processes the spoken words to generate a list of ordered items. The script utilizes various libraries for speech recognition, text-to-speech conversion, and fuzzy string matching. 


## How it Works

1. **Speech Input**: The script uses the `SpeechRecognition` library to capture and convert spoken words from the user through the microphone.

2. **Menu Comparison**: The recognized items are then compared to a predefined menu using the `fuzzywuzzy` library, allowing for flexibility in recognizing similar-sounding words.

3. **User Feedback**: The system provides feedback to the user, listing the recognized items on the menu and identifying any items that were not found.

4. **Text-to-Speech Output**: The feedback is converted into speech using the `gTTS` (Google Text-to-Speech) library, providing an audible confirmation to the user.

5. **Order Confirmation**: The user is prompted to confirm their order or add more items. The script handles additional items or finalizes the order based on user input.

## Requirements

- Python 3.x
- Install required packages by running: `pip install gtts pydub fuzzywuzzy SpeechRecognition`

## Usage

1. Run the script: `python voice_order_system.py`
2. Speak your order into the microphone.
3. The system provides feedback on recognized and unrecognized items.
4. Confirm the order or add more items as prompted.

## Configuration

- Customize the menu items in the `menu` list in the `main` function.
- Adjust the similarity threshold in the `compare_to_menu` function based on your needs.

## Dependencies

- [gTTS (Google Text-to-Speech)](https://pypi.org/project/gTTS/): Converts text to speech.
- [pydub](https://pypi.org/project/pydub/): Manipulates audio files.
- [fuzzywuzzy](https://pypi.org/project/fuzzywuzzy/): Provides fuzzy string matching.
- [SpeechRecognition](https://pypi.org/project/SpeechRecognition/): Recognizes speech using various APIs.

## Notes

- This script uses Google Speech Recognition for speech-to-text, and an internet connection is required for it to work.

Feel free to modify and adapt the script to suit your specific use case.
