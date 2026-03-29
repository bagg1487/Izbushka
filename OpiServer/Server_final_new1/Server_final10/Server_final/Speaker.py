import os
from gtts import gTTS
import pygame
import time

# ЖЕСТКАЯ ПРИВЯЗКА К ALSA И КАРТЕ 3
os.environ["SDL_AUDIODRIVER"] = "alsa"
os.environ["AUDIODEV"] = "hw:3,0"

class Speaker:
    # очищение привязки к карте (решение конфликта (Radio/GPT))
    def sync_clean(self):
        try:
            if pygame.mixer.get_init():
                pygame.mixer.music.stop()
                pygame.mixer.quit()
        except Exception as e:
            print(f"Ошибка {e}")
    def init(self):
        # Пробуем инициализировать микшер с разными параметрами
        try:
            # Сначала пробуем стандартные настройки, которые любят Orange Pi
            pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=2048)
            pygame.mixer.init()
            print("🔊 Аудио система инициализирована успешно (hw:3,0)")
        except Exception as e:
            print(f"⚠️ ОШИБКА ИНИЦИАЛИЗАЦИИ ЗВУКА: {e}")
            # Попытка №2 с другими параметрами (иногда помогает)
            try:
                pygame.mixer.quit()
                pygame.mixer.init(frequency=48000, size=-16, channels=1, buffer=4096)
                print("🔊 Аудио система инициализирована (попытка 2)")
            except Exception as e2:
                print(f"⚠️ ФАТАЛЬНАЯ ОШИБКА ЗВУКА: {e2}")

    def speak(self, text):
        """Озвучка текста через Google TTS"""
        if not text:
            return
            
        print(f"Избушка говорит: {text}")
        
        # Проверка, жив ли микшер
        if not pygame.mixer.get_init():
            print("Ошибка: Микшер не инициализирован. Пытаюсь перезапустить...")
            try:
                pygame.mixer.init()
            except:
                print("Не вышло. Звука не будет.")
                return

        try:
            filename = f"response_{int(time.time())}.mp3"
            tts = gTTS(text=text, lang='ru')
            tts.save(filename)

            # Проигрываем
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()

            # Ждем окончания
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)
                
            # Удаляем файл
            pygame.mixer.music.unload()
            os.remove(filename)
            
        except Exception as e:
            print(f"❌ Ошибка воспроизведения: {e}")

    def wishMe(self):
        self.speak("Системы загружены. Я готова к работе.")

if __name__ == "__main__":
    s = Speaker()
    s.speak("Проверка звуковой системы.")