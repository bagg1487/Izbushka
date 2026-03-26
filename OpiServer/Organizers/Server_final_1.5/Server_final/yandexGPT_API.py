import requests
import json

class YandexAPI:

    def __init__(self):
        super().__init__()

        self.error_message = "An answer to a given topic cannot be generated" # такое текстовое сообщение присылает yandexGPT API если генерация ответа не удалась
        self.error_answer = "Я не хочу отвечать на это"                       # сообщение, которое мы озвучиваем, если несколько генераций не удалось
        self.system_instructions = "Представь, что ты говоришь от лица робота избушки на курьих ножках. Отвечай кратко, будто разговариваешь" # задаём "характер" избушки. Базовые настройки
        self.messages = [                                                     # тут будем хранить историю текущего диалога
            {
                "role": "system",
                "text": self.system_instructions
            }
        ]

        self.url = "https://llm.api.cloud.yandex.net/foundationModels/v1/completion"
        self.headers = {
            "Content-Type": "application/json",
            "Authorization": "Api-Key -----------------"
        }
        self.prompt = {
            "modelUri": "gpt://b1gvt0v34vs1p55m3h57/yandexgpt",
            "completionOptions": {
                "stream": False,
                "temperature": 0.5,                                        # "индивидуальность" генерации нейросети. 0 - сухой текст, 1 - полёт фантазии
                "maxTokens": "200"                                         # максимум токенов на ответ (маловажная переменная т.к. в system_instructions написано "отвечай кратко"
            },
            "messages": self.messages
        }

    def request(self, data):
        """Функция принимает запрос data, обрабатывает его и отдает ответ answer_text"""
        self.messages.append({
            "role": "user",
            "text": data
        })
        self.counter = 0
        answer_text = self.one_more_try()                                                       # пытаемся получить ответ от нейросети
        if type(answer_text) != str:
            answer_text = self.error_answer
        answer_text.replace("\n", "")                                               # вырезаем из строки ответа символы переноса

        # print(answer_text)
        return answer_text

    def one_more_try(self):
        """Функция запроса в yandexGPT, отдает ответ answer_text"""
        full_answer = requests.post(self.url, headers=self.headers, json=self.prompt)
        if "error" in full_answer.text:
            if self.error_message in full_answer.text and self.counter < 3:  # если генерация неудачна, то пробуем ещё раз
                self.counter += 1
                answer_text = self.one_more_try()
            elif self.error_message in full_answer.text:
                answer_text = full_answer  # после 3х попыток мы соглашаемся, что генерация не получится
                self.counter = 0
            else:
                answer_text = full_answer # при неизвестной ошибке мы говорим об этом
                print("Неизвестная ошибка ответа yandexGNT_API")
                self.counter = 0
        else:
            try:
                answer_text = (json.loads(full_answer.text)                             # вытаскиваем нужную часть словаря ответа
                               .get("result", self.error_answer)
                               .get("alternatives", self.error_answer)[0]
                               .get("message", self.error_answer)
                               .get("text", self.error_answer))                         # выбираем сам текст ответа
                self.messages.append({                                                  # добавляем в историю диалога ответ нейросети
                    "role": "assistant",
                    "text": answer_text
                })

            except:
                print("yandexGNT_API_error")
                answer_text = self.error_answer
            self.counter = 0

        return answer_text

#if __name__ == '__main__':
#    YandexAPI().request("какие у тебя планы на выходные?")
