from gtts import gTTS
import os
import platform

def text_to_speech(text, language='en', save_to_file='output.mp3'):
    # Create a gTTS object
    tts = gTTS(text=text, lang=language, slow=False)

    # Save the speech to a file
    tts.save(save_to_file)

    # Play the saved speech using the default audio player
    if platform.system() == 'Darwin':  # Check if the platform is macOS
        os.system("open " + save_to_file)
    else:
        print("Unsupported platform for playing audio.")

if __name__ == "__main__":
    # Example usage
    text = "Hello, your order is ready. Enjoy your meal! Please let me know if you need anything else"
    text_to_speech(text)
