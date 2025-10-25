import speech_recognition as sr
import threading
import time
import requests
import queue

class VoiceProcessor:
    def __init__(self):
        self.commands = {
            "избушка реакция": self.random_expression,
            "избушка радость": lambda: self.set_expression("happy"),
            "избушка злость": lambda: self.set_expression("angry"),
            "избушка удивление": lambda: self.set_expression("surprised"),
            "избушка грусть": lambda: self.set_expression("sad"),
            "избушка смех": lambda: self.set_expression("laughing"),
            "избушка подмигни": lambda: self.set_expression("winking"),
            "избушка сон": lambda: self.set_expression("sleepy"),
            "избушка нейтрально": lambda: self.set_expression("neutral"),
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
            
    def random_expression(self):
        from reaction import get_random_expression
        expression = get_random_expression()
        try:
            response = requests.post('http://127.0.0.1:8000/change-expression', 
                                   json={"expression": expression})
            print(f"✅ Выражение изменено: {expression}")
        except Exception as e:
            print(f"❌ Ошибка: {e}")
        
    def set_expression(self, expression_name):
        try:
            response = requests.post('http://127.0.0.1:8000/change-expression', 
                                   json={"expression": expression_name})
            print(f"✅ Выражение изменено: {expression_name}")
        except Exception as e:
            print(f"❌ Ошибка: {e}")
        
    def start_casino(self):
        import webbrowser
        webbrowser.open('http://127.0.0.1:8000/casino')
        
    def pull_lever(self):
        try:
            response = requests.post('http://127.0.0.1:8000/casino/spin')
            print("✅ Рычаг дернут")
        except Exception as e:
            print(f"❌ Ошибка: {e}")
        
    def exit_casino(self):
        self.set_expression("neutral")
        
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