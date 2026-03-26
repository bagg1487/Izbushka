import os
import time

import pyttsx3
import datetime

import sounddevice
from pyttsx3 import voice
import platform

from pydub.playback import play

from transliterate import translit
from speechkit import model_repository, configure_credentials, creds

#import torch

os_name = platform.system()
if os_name == "Windows":
    print("Вы используете Windows.")
    engine = pyttsx3.init('sapi5')

elif os_name == "Linux":
    print("Вы используете Linux.")
    engine = pyttsx3.init()

voices = engine.getProperty('voices')  # даёт подробности о текущем установленном голосе
engine.setProperty('voice', voice)  # 0-мужской , 1-женский
model = model_repository.synthesis_model()

# Задайте настройки синтеза.
model.voice = 'jane'
model.role = 'good'

language = 'ru'
speaker = 'xenia'
#device = torch.device('cpu')
#torch.set_num_threads(4)
local_file = 'v4_ru.pt'
#model_silero, example_text = torch.hub.load(repo_or_dir='snakers4/silero-models',
#                                    model='silero_tts',
#                                    language='ru',
#                                    speaker='ru_v3')
sample_rate = 48000
#model_silero.to(device)  # gpu or cpu


class Speaker:
    def __init__(self):
        self.language = None
        self.model_id = None
        self.device = None
        self.model = None
        self.audio = None
        self.speech_trig = False
        self.rec_res = None
        configure_credentials(
            yandex_credentials=creds.YandexCredentials(
                api_key=''
            )
        )

    def speakV2(self, audio):
        """Преобразование текста в речь с помощью offline системного синтезатора"""
        if not self.speech_trig:
            self.speech_trig = True
            engine.say(audio)
            engine.runAndWait()  # Без этой команды мы не услышим речь
            self.speech_trig = False

    def speakV3(self, audio):
        """Преобразование текста в речь с помощью offline системного синтезатора"""
        if not self.speech_trig:
            # result = model_silero.apply_tts(text=audio,
            #                                 speaker=speaker,
            #                   sample_rate=sample_rate)
            #result = model_silero.apply_tts(text=audio,
            #            speaker=speaker,
            #            sample_rate=sample_rate)
            self.speech_trig = True
            #sounddevice.play(result, sample_rate)
            #time.sleep(len(result) / sample_rate + 0.1)
            sounddevice.stop()
            self.speech_trig = False

    def speak(self, audio):
        """Преобразование текста в речь с помощью Yandex speechkit"""
        # Синтез речи и создание аудио с результатом.
        result = model.synthesize(audio, raw_format=False)
        # eng_name = translit(audio[0:21], language_code='ru',
        #                     reversed=True)
        # eng_name=eng_name.replace(' ', '_').replace('?','')
        # result.export('voice/'+eng_name+'.wav', format='wav')
        self.speech_trig = True
        play(result)
        self.speech_trig = False

    def wishMe(self):
        """Приветственная фраза"""
        hour = int(datetime.datetime.now().hour)
        if hour >= 0 and hour < 12:
            self.speak("Доброе утро!")

        elif hour >= 12 and hour < 18:
            self.speak('Добрый день!')

        else:
            self.speak("Добрый вечер!")

        self.speak(
            'Чем я могу вам помочь?')

    if __name__ == '__main__':
        pass
