# audio.py
import speech_recognition as sr
import threading
import time
import requests
import queue

class VoiceProcessor:
    def __init__(self):
        self.commands = {
            "избушка реакция": self.reaction,
            "избушка крути": self.start_casino,
            "избушка рычаг": self.pull_lever,
            "избушка отмена": self.exit_casino
        }
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.command_queue = queue.Queue()
        self.setup_microphone()
        
    def setup_microphone(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            
    def reaction(self):
        from reaction import get_random_reaction
        reaction_image = get_random_reaction()
        requests.post('http://127.0.0.1:8000/change-image', json={"image": reaction_image})
        
    def start_casino(self):
        import webbrowser
        webbrowser.open('http://127.0.0.1:8000/casino')
        
    def pull_lever(self):
        requests.post('http://127.0.0.1:8000/casino/spin')
        
    def exit_casino(self):
        requests.post('http://127.0.0.1:8000/change-image', json={"image": "/static/default.jpg"})
        
    def process_commands(self):
        while True:
            command_func = self.command_queue.get()
            if command_func is None:
                break
            command_func()
            time.sleep(2)
            
    def listen(self):
        def callback(recognizer, audio):
            try:
                text = recognizer.recognize_google(audio, language="ru-RU").lower()
                print(f"🎤 Распознано: {text}")
                for command, func in self.commands.items():
                    if command in text:
                        print(f"✅ Выполняю команду: {command}")
                        self.command_queue.put(func)
            except sr.UnknownValueError:
                pass
            except sr.RequestError as e:
                print(f"❌ Ошибка распознавания: {e}")
                
        self.recognizer.listen_in_background(self.microphone, callback)
        processor_thread = threading.Thread(target=self.process_commands)
        processor_thread.daemon = True
        processor_thread.start()