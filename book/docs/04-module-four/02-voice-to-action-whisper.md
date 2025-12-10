---
id: 02-voice-to-action-whisper
title: "Voice-to-Action: Using OpenAI Whisper for Voice Commands Generation"
sidebar_label: "Chapter 2: Voice-to-Action"
sidebar_position: 2
---

# Voice-to-Action: Using OpenAI Whisper for Voice Commands Generation

## Introduction

Enabling robots to understand and act upon spoken commands is a significant step towards more intuitive and natural human-robot interaction (HRI). The "Voice-to-Action" pipeline involves two primary stages: **Speech-to-Text (STT)** to transcribe spoken words into text, and **Natural Language Understanding (NLU)** to extract actionable intent from that text. OpenAI Whisper is a powerful general-purpose speech recognition model that excels at the STT component, laying the foundation for converting human voice commands into robot actions.

## 2.1. Speech-to-Text Overview

Speech-to-Text (STT) technology converts audio input (spoken language) into written text. This is a crucial first step for any voice-controlled system. The accuracy and robustness of the STT model directly impact the robot's ability to correctly interpret commands. Factors like background noise, accents, and multiple speakers can all affect STT performance.

## 2.2. OpenAI Whisper API Integration

**OpenAI Whisper** is a neural network that has been trained on a large dataset of diverse audio and text, making it highly robust to various languages, accents, and background noise. It can perform not only speech recognition but also language identification and speech translation.

### Key Features:

-   **High Accuracy**: Generally provides excellent transcription quality across a wide range of audio inputs.
-   **Multilingual**: Supports transcription in many languages and can translate them into English.
-   **Robustness**: Handles background noise and different speaking styles well.

### Integration Steps (High-Level)

1.  **Obtain API Key**: Access the OpenAI API by obtaining an API key.
2.  **Install Client Library**: Use the official OpenAI Python client library (`pip install openai`).
3.  **Audio Input**: Capture audio from a microphone or load an audio file (e.g., WAV, MP3).
4.  **API Call**: Send the audio data to the Whisper API endpoint.
5.  **Receive Transcription**: Process the API response to get the transcribed text.

### Example Python Script for Whisper API Call

```python
import os
import openai
from pydub import AudioSegment # pip install pydub
from pydub.playback import play # pip install simpleaudio (for playback)

# Ensure your OpenAI API key is set as an environment variable
# os.environ["OPENAI_API_KEY"] = "YOUR_OPENAI_API_KEY"
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio(audio_file_path):
    """
    Transcribes an audio file using OpenAI Whisper API.
    """
    if not os.path.exists(audio_file_path):
        print(f"Error: Audio file not found at {audio_file_path}")
        return None

    try:
        with open(audio_file_path, "rb") as audio_file:
            transcript = openai.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file,
                response_format="text" # or "json" for more details
            )
        return transcript
    except openai.APIError as e:
        print(f"OpenAI API Error: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

def main():
    # --- Example 1: Transcribe a local audio file ---
    # Create a dummy audio file for demonstration
    # In a real scenario, this would be actual recorded audio.
    # For now, we'll create a silent WAV for demonstration purposes
    dummy_audio_file = "dummy_command.wav"
    AudioSegment.silent(duration=2000).export(dummy_audio_file, format="wav")
    print(f"Created a dummy audio file: {dummy_audio_file}")

    # Note: For actual testing, you would replace dummy_audio_file with a file
    # containing spoken commands, e.g., "robot, go forward and pick up the blue ball".
    
    # Transcription
    # In a real application, you'd feed actual spoken audio here
    # For this example, we'll simulate a transcription for a known command.
    simulated_transcription = "Robot, go to the kitchen and fetch the coffee mug."
    print(f"\nSimulated Whisper Transcription: '{simulated_transcription}'")

    # --- Example 2: Actual API call (requires valid audio file and API key) ---
    # if openai.api_key:
    #     print("\nAttempting actual Whisper API call (requires valid audio and API key)...")
    #     # Replace with your actual audio file containing speech
    #     # real_audio_file = "my_voice_command.mp3"
    #     # transcription_from_api = transcribe_audio(real_audio_file)
    #     # if transcription_from_api:
    #     #     print(f"Actual Whisper Transcription: '{transcription_from_api}'")
    # else:
    #     print("\nSkipping actual Whisper API call: OPENAI_API_KEY not set.")


if __name__ == '__main__':
    main()
```
**Note**: To run this example, you need an OpenAI API key and potentially `pydub` and `simpleaudio` for handling audio files. For actual use, you would record audio from a microphone and feed it to the `transcribe_audio` function.

## 2.3. Processing Voice Commands: Intent Recognition and Basic Parsing

Once you have the transcribed text, the next step is to understand the user's **intent** and extract relevant information (entities, parameters) to form an executable robot command. This is the NLU part of the pipeline.

### Basic Command Parsing

For simple commands, rule-based parsing or regular expressions can be effective. For more complex and flexible commands, Natural Language Processing (NLP) techniques, often powered by smaller LLMs or fine-tuned models, are used.

```python
# Pseudo-code for basic command parsing
def parse_robot_command(text_command: str) -> dict:
    """
    Parses a transcribed text command into a structured robot action.
    """
    command_lower = text_command.lower()
    action = {"action": "unknown"}

    if "go to" in command_lower:
        action["action"] = "navigate"
        if "kitchen" in command_lower:
            action["target"] = "kitchen"
        elif "living room" in command_lower:
            action["target"] = "living_room"
        else:
            action["target"] = "unspecified_location"
    elif "pick up" in command_lower:
        action["action"] = "manipulate"
        if "coffee mug" in command_lower:
            action["object"] = "coffee_mug"
        elif "blue ball" in command_lower:
            action["object"] = "blue_ball"
        else:
            action["object"] = "unspecified_object"
    elif "stop" in command_lower or "halt" in command_lower:
        action["action"] = "stop"
    
    return action

def main():
    simulated_transcription = "Robot, please go to the living room."
    parsed_command = parse_robot_command(simulated_transcription)
    print(f"Transcribed: '{simulated_transcription}'")
    print(f"Parsed Command: {parsed_command}")

    simulated_transcription_2 = "Hey robot, pick up the blue ball now."
    parsed_command_2 = parse_robot_command(simulated_transcription_2)
    print(f"\nTranscribed: '{simulated_transcription_2}'")
    print(f"Parsed Command: {parsed_command_2}")

if __name__ == '__main__':
    main()
```

This parsed dictionary can then be used to trigger specific ROS 2 actions or service calls.

## Summary

OpenAI Whisper provides a highly accurate and robust Speech-to-Text solution, forming the initial critical step in a voice-to-action pipeline for robots. By transcribing spoken commands into text, and then applying basic parsing or more advanced NLU techniques, robots can begin to understand and respond to human verbal instructions, paving the way for more natural and intuitive human-robot interaction. The next step is to translate these understood natural language commands into actionable robot plans, which we will explore in [Chapter 3: Cognitive Planning: Using LLMs to Translate Natural Language into ROS 2 Actions](03-cognitive-planning-llms-ros2.md).

## Further Reading

-   [OpenAI Whisper API Documentation](https://platform.openai.com/docs/guides/speech-to-text)
-   [OpenAI Python Library](https://github.com/openai/openai-python)
-   [Speech Recognition with Python](https://realpython.com/python-speech-recognition/)
-   [Natural Language Processing (NLP) Basics](https://www.ibm.com/cloud/learn/natural-language-processing)
