import argparse
import json
import queue
import sys
import sounddevice as sd

from vosk import Model, KaldiRecognizer


class Recognizer:
    """Класс распознавания голоса"""
    def __init__(self):
        self.q = queue.Queue()
        self.full_phrase = ""
        self.partial_phrase = ""
        self.rec = None
        self.stream = None
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument(
            "-l", "--list-devices", action="store_true",
            help="show list of audio devices and exit")
        args, remaining = parser.parse_known_args()
        if args.list_devices:
            print(sd.query_devices())
            parser.exit(0)
        parser = argparse.ArgumentParser(
            description=__doc__,
            formatter_class=argparse.RawDescriptionHelpFormatter,
            parents=[parser])
        parser.add_argument(
            "-f", "--filename", type=str, metavar="FILENAME",
            help="audio file to store recording to")
        parser.add_argument(
            "-d", "--device", type=self.int_or_str,
            help="input device (numeric ID or substring)")
        parser.add_argument(
            "-r", "--samplerate", type=int, help="sampling rate")
        parser.add_argument(
            "-m", "--model", type=str, help="language model; e.g. en-us, fr, nl; default is en-us")
        args = parser.parse_args(remaining)

        try:
            if args.samplerate is None:
                device_info = sd.query_devices(args.device, "input")
                # soundfile expects an int, sounddevice provides a float:
                args.samplerate = int(device_info["default_samplerate"])

            if args.model is None:
                model = Model(lang="ru")
            else:
                model = Model(lang=args.model)

            if args.filename:
                dump_fn = open(args.filename, "wb")
            else:
                dump_fn = None

            self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000, device='hw:3,0',
                                            dtype="int16", channels=1, callback=self.callback)
            self.stream.start()
            print("#" * 80)
            print("Press Ctrl+C to stop the recording")
            print("#" * 80)
            self.rec = KaldiRecognizer(model, args.samplerate)
        except KeyboardInterrupt:
            print("\nDone")
            parser.exit(0)

    def int_or_str(self, text):
        """Helper function for argument parsing."""
        try:
            return int(text)
        except ValueError:
            return text

    def callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        try:
            if status:
                print(status, file=sys.stderr)
            if self.q.qsize() < 10:
                self.q.put(bytes(indata))
        except Exception as e:
            print(e)

    def listening(self):
        """Слушает микрофон. Записывает частичную фразу и по окончанию прослушивания полную"""
        # Если очередь не пуста
        if self.q.not_empty:
            # Получаем данные из очереди
            data = self.q.get()
            # Передаем данные в распознаватель речи
            if self.rec.AcceptWaveform(data):
                # Получаем результат полного распознавания и сохраняем фразу
                phrase = json.loads(self.rec.Result())
                self.full_phrase = phrase['text']
                print(phrase)
            else:
                # Если результат не окончательный, сохраняем частичный результат
                self.full_phrase = ''
                phrase = json.loads(self.rec.PartialResult())
                self.partial_phrase = phrase['partial']
    def get_partial_phrase(self):
        """Возвращает частичную фразу"""
        return self.partial_phrase

    def get_full_phrase(self):
        """Возвращает полную фразу"""
        return self.full_phrase

    def stop(self):
        """Останавливает прослушивание"""
        self.stream.stop()

    def start(self):
        """Запускает прослушивание"""
        self.stream.start()

    def close(self):
        self.stream.close()
