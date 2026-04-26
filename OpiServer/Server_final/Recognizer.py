import sys
import sounddevice as sd
import vosk
import queue
import json


class Recognizer:
    
    def __init__(self):
        self.full_phrase = ""
        self.partial_phrase = ""
        self._listening = False
        self._use_microphone = False
        self.q = queue.Queue()
        self.rec = None
        self.stream = None
        
        try:
            devices = sd.query_devices()
            has_input = any(d['max_input_channels'] > 0 for d in devices)
            
            if has_input:
                self._init_real_recognizer()
                self._use_microphone = True
            else:
                raise Exception("Нет устройств ввода")
                
        except Exception as e:
            print(f"Распознавание недоступно: {e}")
            self._use_microphone = False

    def _init_real_recognizer(self):
        try:
            model = vosk.Model(lang="ru")
            self.rec = vosk.KaldiRecognizer(model, 16000)
            
            self.stream = sd.RawInputStream(
                samplerate=16000,
                blocksize=8000,
                dtype="int16",
                channels=1,
                callback=self.callback
            )
            self.stream.start()
            print("Реальный распознаватель инициализирован")
        except Exception as e:
            print(f"Ошибка инициализации: {e}")
            self._use_microphone = False

    def callback(self, indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        if self.q.qsize() < 10:
            self.q.put(bytes(indata))

    def listening(self):
        if not self._use_microphone:
            return
        
        if self.q.not_empty:
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                phrase = json.loads(self.rec.Result())
                self.full_phrase = phrase['text']
            else:
                self.full_phrase = ''
                phrase = json.loads(self.rec.PartialResult())
                self.partial_phrase = phrase.get('partial', '')

    def get_partial_phrase(self):
        return self.partial_phrase

    def get_full_phrase(self):
        return self.full_phrase

    def stop(self):
        if self.stream:
            self.stream.stop()
        self._listening = False

    def start(self):
        if self.stream:
            self.stream.start()
        self._listening = True

    def close(self):
        if self.stream:
            self.stream.close()