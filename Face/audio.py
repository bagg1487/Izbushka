import threading
import time
import queue
import requests
import pyttsx3
import speech_recognition as sr
import wikipedia
from openai import OpenAI

client = OpenAI(api_key="")
wikipedia.set_lang("ru")

tts = pyttsx3.init()
tts.setProperty("rate", 200)
tts_lock = threading.Lock()


class VoiceProcessor:

    DOM_WORDS = {
        "дом",
        "дам",
        "том",
        "дон",
        "домой",
        "дома",
        "домик",
        "дном",
        "док",
        "домъ"
    }

    ASSISTANT_WORDS = {
        "ассистент",
        "асистент",
        "ассистэнт",
        "асистэнт",
        "ассист",
        "ассик",
        "ассис",
        "ассет",
        "ассистен",
        "асист"
    }

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
            "избушка отмена": self.exit_casino,
        }


        self.system_prompt = {
            "role": "system",
            "content": (
                "Ты голосовой ассистент «Избушка». "
                "Отвечай кратко, по-русски, без лишних объяснений."
            )
        }

        self.dialog_history=[self.system_prompt]

        self.recognizer = sr.Recognizer()

        self.dialog_active = False
        self.dialog_until = 0.0

        print("Список доступных микрофонов:")
        print(sr.Microphone.list_microphone_names())

        self.microphone = sr.Microphone(
            device_index=1,
            sample_rate=48000,
            chunk_size=2048,
        )

        self.command_queue: "queue.Queue[callable]" = queue.Queue()
        self.setup_microphone()

    def speak(self, text: str):
        print("ассистент говорит:", text)
        try:
            with tts_lock:
                tts.say(text)
                tts.runAndWait()
        except Exception as e:
            print("TTS error:", e)

    def trim_dialog_history(self, max_mess=5):
        self.dialog_history = ([self.system_prompt] + self.dialog_history[1:][-max_mess:])

    def ask_gpt(self, prompt: str) -> str:
        try:
            print("[GPT] отправляю запрос:", prompt)
            self.dialog_history.append({"role": "user", "content": prompt})
            response = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=self.dialog_history
            )
            answer = response.choices[0].message.content

            self.dialog_history.append({"role": "assistant", "content": answer})
            self.trim_dialog_history()

            print("[GPT] ответ получен")
            return answer
        except Exception as e:
            print("GPT ERROR:", e)
            return "Мне не удалось получить ответ от модели."

    def search_wiki(self, query: str) -> str | None:
        try:
            return wikipedia.summary(query, sentences=2)
        except Exception as e:
            print("[WIKI error]:", e)
            return None

    def handle_gpt_wake(self, text: str):
        text = text.replace("дом", "").strip()

        if text == "":
            self.speak("Готов ответить. Скажи, что тебя интересует.")
            return

        print("[ASSISTANT] режим GPT по слову 'дом'")
        answer = self.ask_gpt(text)
        self.speak(answer)

    def handle_assistant_request(self, text: str):
        text = text.replace("ассистент", "").strip()

        if text == "":
            self.speak("Да, я слушаю.")
            return

        wiki = self.search_wiki(text)
        if wiki:
            print("[ASSISTANT] ответ из Wikipedia")
            self.speak(wiki)
            return

        print("[ASSISTANT] использую GPT")
        answer = self.ask_gpt(text)
        self.speak(answer)

    def setup_microphone(self):
        with self.microphone as source:
            print("Калибрую микрофон...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            print("Готов к распознаванию")

    def random_expression(self):
        from reaction import get_random_expression

        expression = get_random_expression()
        try:
            requests.post(
                "http://127.0.0.1:8000/change-expression",
                json={"expression": expression},
            )
            print(f"Выражение изменено: {expression}")
        except Exception as e:
            print(f"Ошибка при смене выражения: {e}")

    def set_expression(self, expression_name: str):
        try:
            requests.post(
                "http://127.0.0.1:8000/change-expression",
                json={"expression": expression_name},
            )
            print(f"Выражение изменено: {expression_name}")
        except Exception as e:
            print(f"Ошибка при смене выражения: {e}")

    def start_casino(self):
        import webbrowser
        webbrowser.open("http://127.0.0.1:8000/casino")

    def pull_lever(self):
        try:
            requests.post("http://127.0.0.1:8000/casino/spin")
            print("Рычаг дернут")
        except Exception as e:
            print(f"Ошибка при запуске спина: {e}")

    def exit_casino(self):
        self.set_expression("neutral")

    def process_commands(self):
        while True:
            task = self.command_queue.get()
            if task is None:
                break
            try:
                task()
            except Exception as e:
                print("Ошибка при выполнении задачи:", e)
            time.sleep(0.3)

    def activate_dialog(self, seconds=10):
        self.dialog_active = True
        self.dialog_until = time.time() + seconds

    def is_dialog_active(self):
        if not self.dialog_active:
            return False

        if time.time() > self.dialog_until:
            self.dialog_active = False
            return False

        return True

    def listen(self):
        def callback(recognizer: sr.Recognizer, audio: sr.AudioData):
            try:
                text = recognizer.recognize_google(audio, language="ru-RU").lower()
                print("Распознано:", text)

                words = set(text.split())

                if self.is_dialog_active():
                    self.activate_dialog(10)
                    threading.Thread(
                        target=self.handle_gpt_wake,
                        args=(text,),
                        daemon=True
                    ).start()
                    return

                if self.DOM_WORDS & words:
                    self.activate_dialog(10)
                    clean_text = self.remove_wake_words(text)
                    if clean_text:
                        threading.Thread(
                            target=self.handle_gpt_wake,
                            args=(clean_text,),
                            daemon=True
                        ).start()
                    else:
                        self.speak("Я слушаю")
                    return

                # if self.DOM_WORDS & words:
                #     threading.Thread(
                #         target=self.handle_gpt_wake,
                #         args=(text,),
                #         daemon=True,
                #     ).start()
                #     return

                if self.ASSISTANT_WORDS & words:
                    threading.Thread(
                        target=self.handle_assistant_request,
                        args=(text,),
                        daemon=True,
                    ).start()
                    return

                for cmd, func in self.commands.items():
                    if cmd in text:
                        print("Выполняю команду:", cmd)
                        self.command_queue.put(func)
                        return

            except sr.UnknownValueError:
                return
            except sr.RequestError as e:
                print("[Speech API error]:", e)
                return
            except Exception as e:
                print("[Unexpected speech error]:", e)
                return

        print("Запускаю фоновое прослушивание")
        self.recognizer.listen_in_background(
            self.microphone,
            callback,
            phrase_time_limit=5,
        )

        worker = threading.Thread(target=self.process_commands)
        worker.daemon = True
        worker.start()
