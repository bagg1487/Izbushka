import google.generativeai as genai
import requests

class GeminiAssistant:
    def init(self):
        # Твой ключ
        self.api_key = "AIzaSyACRWS7gvVYIbf2u8StBvVNHMuA-0_cjrs"
        genai.configure(api_key=self.api_key)

        try:
            # Пытаемся подключить модель 2.5
            self.model = genai.GenerativeModel('models/gemini-2.5-flash-lite')
        except:
            # Если нет, откатываемся на 1.5
            self.model = genai.GenerativeModel('gemini-2.5-flash')

    def ask(self, prompt):
        """Запрос к нейросети"""
        system_instruction = (
            "Ты — голосовой помощник робота Избушка. Отвечай строго кратко (до 2 предложений), "
            "с юмором, емко и по делу. Не используй списки и спецсимволы markdown. "
            "Вопрос пользователя: "
        )
        try:
            response = self.model.generate_content(system_instruction + prompt)
            return response.text
        except Exception as e:
            return f"У меня проблемы со связью с космосом: {e}"

    def get_weather(self, user_query):
        """Получение погоды через Open-Meteo"""
        try:
            day_index = 0
            day_name = "Сегодня"
            if "завтра" in user_query:
                day_index = 1
                day_name = "Завтра"
            
            # Координаты Новосибирска (можешь поменять)
            url = "https://api.open-meteo.com/v1/forecast"
            params = {
                "latitude": 55.0084,
                "longitude": 82.9357,
                "daily": "temperature_2m_max,temperature_2m_min,weathercode",
                "timezone": "auto"
            }
            r = requests.get(url, params=params, timeout=5)
            data = r.json()
            
            temp_max = round(data["daily"]["temperature_2m_max"][day_index])
            temp_min = round(data["daily"]["temperature_2m_min"][day_index])
            code = data["daily"]["weathercode"][day_index]

            # Простая расшифровка кодов WMO
            weather_desc = "ясно" if code < 3 else "облачно"
            if code > 45: weather_desc = "туманно"
            if code > 50: weather_desc = "дождливо"
            if code > 70: weather_desc = "снежно"
            
            return f"{day_name} температура от {temp_min} до {temp_max} градусов, {weather_desc}."
        except:
            return "Не удалось узнать погоду, сервер метеорологов молчит."