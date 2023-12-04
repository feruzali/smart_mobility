from gtts import gTTS
import os
import speech_recognition as sr
from fuzzywuzzy import fuzz
from pydub import AudioSegment
from pydub.playback import play
import sys

def speech_to_text():
    recognizer = sr.Recognizer()
    orders = []
    confirmed = False

    while not confirmed:
        with sr.Microphone() as source:
            print("I'm ready to take your order:")
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)

        try:
            text = recognizer.recognize_google(audio).lower()

            if "done" in text or "finish" in text or "complete" in text:
                return orders
            elif "confirm" in text:
                confirmed = True
            else:
                orders.append(text)
                print(f"{text} ")
        except sr.UnknownValueError:
            print("Sorry, could you repeat?")
        except sr.RequestError as e:
            print(f"No results from Google Speech Recognition service; {e}")

    return orders

def compare_to_menu(order_items, menu):
    found_items = []
    not_found_items = []

    for order in order_items:
        found = False
        for item in menu:
            similarity = fuzz.ratio(order, item.lower())
            if similarity >= 80:  # You can adjust the threshold based on your needs
                found_items.append(item)
                found = True
                break

        if not found:
            not_found_items.append(order)

    return found_items, not_found_items

def text_to_speech(feedback_list):
    for text in feedback_list:
        tts = gTTS(text=text, lang='en')
        tts.save("output.mp3")
        sound = AudioSegment.from_mp3("output.mp3")
        play(sound)

def main():
    menu = ["coke", "juice", "pizza", "cheeseburger", "noodles", "chicken"]  # Your menu items

    order_items = speech_to_text()

    found_items, not_found_items = compare_to_menu(order_items, menu)

    if found_items:
        feedback_text = f"We have {', '.join(found_items)} on the menu"
        print(feedback_text)
        text_to_speech([feedback_text])

        if not_found_items:
            not_found_text = f"But we don't have {', '.join(map(str.lower, not_found_items))}"
            print(not_found_text)
            text_to_speech([not_found_text])

        confirmation_text = "Do you want to proceed with your order? Say 'confirm' to confirm or 'add' to add more items."
        print(confirmation_text)
        text_to_speech([confirmation_text])

        # Listen for user confirmation or additional order
        additional_order = speech_to_text()

        # if "confirm" in additional_order:
        #     print("Wait for your order, please.")
        #     text_to_speech(["Wait for your order, please."])
        #     sys.exit()  # Finish the execution after confirming the order
        if "add" in additional_order:
            print("Okay, what else would you like to order?")
            order_items.extend(speech_to_text())
            found_items, not_found_items = compare_to_menu(order_items, menu)
            # Continue to check the new items
        else:
            print("Wait for your order, please")
            text_to_speech(["Wait for your order, please"])
            sys.exit()

    else:
        print("No dishes from your order are on the menu.")
        text_to_speech(["Sorry, we don't have none of these on the menu."])

if __name__ == "__main__":
    main()
