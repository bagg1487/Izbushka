import vlc
import time

print("Проверка VLC на Orange Pi...")

instance = vlc.Instance("--aout=alsa", "--alsa-audio-device=plughw:3,0")

player = instance.media_player_new()

media = instance.media_new("http://stream.nonstopplay.co.uk/nsp-128k-mp3")
player.set_media(media)

print("Запуск воспроизведения на карте 3 (rockchipes8388)...")
player.play()

print("Радио должно играть 10 секунд...")
time.sleep(10)

player.stop()
print("Тест завершен!")
