# JunoNav

Role
Juno acts as an interactive campus assistant for new students, visitors, or anyone not yet familiar with the campus map and bus 🚍 schedule.

Main Tasks
In this project, we will implement the following main features:
Voice-Activated Q&A interaction—Users can ask questions via voice, and the system will recognize their speech and respond with a text answer. For example, it can provide the opening hours of common campus locations by matching keywords.


Campus shuttle options — The system will offer travel suggestions for campus shuttle routes, which will be replied to in TTS (text-to-speech) in response to user queries.


Language Mode Switching (for International Students)—To support international students during orientation, Juno can switch its STT/TTS pipeline between English and Malay upon a voice request (e.g., “Speak Malay?”), delivering all subsequent interactions in the selected language.

Subtasks
Voice-based Q&A interaction
Wake-Word Detection
Use a ROS wrapper around Porcupine or Snowboy (e.g., porcupine_ros) to publish a /wake_word Bool message.


Speech-to-Text (STT)
Leverage a cloud STT node (e.g., google_speech_ros) that subscribes to /audio_raw and publishes /stt/text.


Intent & Keyword Extraction
Run a small NLU node (could even be a Python ROS node using Rasa NLU or simple regex/rule-based logic) that listens on /stt/text and outputs an /intent topic.




Knowledge-Base Lookup
Implement a ROS service (/campus_info_service) that your NLU node calls, backed by a JSON/SQLite database.


Response Formatting & TTS
Have a dialogue_manager node format the reply and call the standard sound_play package or a cloud TTS node (publishing on /tts/audio), which then plays back on the speaker.

Campus Shuttle Options Display
Shuttle Schedule Database
Store the routes, stops, and departure times in a JSON or CSV file under config/.


Shuttle Intent Detection
Extend the existing NLU node to recognize “shuttle” queries (“When’s the next shuttle to North Campus?”).


Route Selection Logic
In the /shuttle_query service callback, filter the schedule by origin/destination and current time, then pick the next departure.


TTS (Text-to-Speech) Response
Publish a single std_msgs/String message on /tts/text containing, for example:
The next shuttle to KK12 is at 14:00. 
It will stop at UM Central, KK10, APIUM, KK11, and KK12.

Fallback
If a destination is not recognized, publish a std_msgs/String message on /tts/text though TTS prompting another destination.

Language Mode Switching
Language-Switch Intent Detection
Extend nlu_processor_node to recognize a new language_switch intent and extract the target language entity (e.g. “English” or “Bahasa Melayu”) from /stt/text.


Language Manager
Create a language_manager_node that subscribes to /intent (or a dedicated /language_switch topic), validates the requested language against supported options, and sets a ROS parameter or publishes the current mode on /language_mode.


Dynamic STT Configuration
Modify stt_client_node to read the /language_mode parameter before each transcription request and pass the appropriate language code (en-US for English, ms-MY for Malay) to the STT API.





Dynamic TTS Configuration
Update tts_client_node so it likewise reads /language_mode and selects the matching voice/language code when publishing text to /tts/text.


Translation Service (for Dynamic Replies)
(Optional) Implement a /translate_text ROS service that takes an English string and target language, calls a translation API (e.g., Google Translate), and returns the localized string for TTS or display.


Switch Confirmation
After the mode is changed, have language_manager_node publish a confirmation message on /tts/text in the newly selected language, e.g.:


English mode: “Okay, I will continue in English.”


Malay mode: “Baiklah, saya akan bercakap dalam Bahasa Melayu.”


Unsupported Language Fallback
If a user requests a language outside the supported set, language_manager_node should publish on /tts/text:


 “Sorry, I currently support only English and Malay.”



Then remain in the previous language mode.





Storyboard
Scene 1: Personalised Greeting & Library Hours Query
Environment: Juno on the library front desk

User Action: A student pauses in front of the camera.

System Response:
The Juno camera detects the student’s face and matches it to the user database.
Speaker plays: “Hello, Eric(student name)! How can I help you today?”

User Action: Eric asks: “Hey Juno, when’s the library open?”

System Response:
Wake-word detection: The /wake_word topic triggers STT.
STT/NLU: Converts speech to text: “When’s the library open?” → Extracts intent opening_hours and keyword library.
Knowledge lookup: Calls /campus_info_service to fetch library hours from the database.
TTS: Speaker responds: “The main library is open from 8 AM to 10 PM today.”


Scene 2: Campus Shuttle Inquiry 

User Action: Student asks: “Hey Juno, when’s the next shuttle bus to KK12?”

System Response:
Intent detection: NLU identifies shuttle_schedule intent and destination “KK12”.
Bus service: Queries the bus database for the next departure.
TTS: *“The next shuttle bus departs at 3:00 PM from UM Central.”*

Backend Process:
shuttle_query service filters the schedule CSV and returns the closest match.

Scene 3: Language Switch Request
User Action
Eric: “Hey Juno, do you speak Malay?”


System Process
/wake_word triggers /stt/request


STT Client Node:


Uses current language (en-US) to transcribe


NLU Processor Node:


Detects intent: language_switch


Entity extracted: "Bahasa Melayu"


Language Manager Node
Subscribed to /intent or /language_switch


Validates request: "Bahasa Melayu" ∈ [English, Bahasa Melayu]


Updates:


ROS Param /language_mode: ms-MY


Publishes on /language_mode: ms-MY


Publishes to /tts/text (in Malay):
 Output: “Baiklah, saya akan bercakap dalam Bahasa Melayu.”
