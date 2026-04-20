import webbrowser

from KeyPhraseAnalizer import *
class PlayVideoViaBrowser:
    def __init__(self):
        self.key_phrase_analizer = KeyPhraseAnalizer()

    def search_youtube(self, query):
        """Открытие браузера с запросом на youtube"""
        url = f'https://www.youtube.com/results?search_query={query}'
        print(url)
        webbrowser.open(url)
    def search_vk(self, query):
        """Открытие браузера с запросом на vk video"""
        url = f'https://vk.com/video?q={query}'
        print(url)
        webbrowser.open(url)
    def search_rutube(self, query):
        """Открытие браузера с запросом на rutube"""
        url = f'https://rutube.ru/search/?query={query}'
        webbrowser.open(url)
    def find_video_browser(self, query,key_phrase):
        """Открытие видео в браузере"""
        # Вывод запроса
        print(query)
        # Удаление стоп-слов из запроса
        query = self.key_phrase_analizer.cut_stop_word(query)
        # Поиск на YouTube, если в ключевой фразе есть 'youtube'
        if 'youtube' in key_phrase:
            self.search_youtube(query)
        # Поиск на VK, если в ключевой фразе есть 'vk'
        elif 'vk' in key_phrase:
            self.search_vk(query)
        # Поиск на Rutube, если в ключевой фразе есть 'rutube'
        elif 'rutube' in key_phrase:
            self.search_rutube(query)
        # В противном случае выполняется поиск на YouTube
        else:
            # Преобразование списка запроса в предложение
            sentence = " ".join(map(str, query))
            # Поиск на YouTube
            self.search_youtube(sentence)


if __name__ == '__main__':
    pass
