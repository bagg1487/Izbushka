# reaction.py
import os
import random

def get_random_reaction():
    # Получаем все файлы из папки static кроме default.png
    static_files = [f for f in os.listdir('static') 
                   if os.path.isfile(os.path.join('static', f)) 
                   and f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif'))
                   and f != 'default.png']
    
    if static_files:
        return f"/static/{random.choice(static_files)}"
    return "/static/default.png"