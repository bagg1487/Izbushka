import google.generativeai as genai

# Ваш ключ
genai.configure(api_key="AIzaSyCBKkHNOcqtW-rRwFjwhM5rau7FSmExub4")

print("Пробую соединиться с Google через Франкфурт...")

try:
    # Получаем список всех доступных моделей
    models = genai.list_models()
    found = False
    for m in models:
        if 'generateContent' in m.supported_generation_methods:
            print(f"✅ НАЙДЕНА МОДЕЛЬ: {m.name}")
            found = True

    if not found:
        print("❌ Список моделей пуст. Возможно, API не включен в консоли Google Cloud.")

except Exception as e:
    print(f"❌ ОШИБКА ПОДКЛЮЧЕНИЯ: {e}")