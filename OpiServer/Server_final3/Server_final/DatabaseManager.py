import sqlite3
import pickle

class DatabaseManager:
    """Класс управления БД, используется для сохранения аудио-заметок"""
    def __init__(self, db_name):
        """Функция инициализации"""
        self.db_name = db_name
        self.conn = sqlite3.connect(db_name, check_same_thread=False)
        self.cursor = self.conn.cursor()
    def GetBlob(self,file):
        """Функция получения 'cлепка лица'"""
        with open(file, 'rb') as file:
            blob_data = file.read()
        return blob_data

    def create_user_table(self):
        """Функция создания таблицы, если её нет"""
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS users_main (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                username TEXT NOT NULL,
                face_data BLOB
            )
        ''')
        self.conn.commit()

    def create_audio_table(self):
        """Функция создания таблицы, если её нет"""
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS audio_notes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                user_id INTEGER,
                status INTEGER,
                audio_data BLOB,
                datetime TEXT,
                FOREIGN KEY (user_id) REFERENCES users_main(id)
            )
        ''')
        self.conn.commit()

    def insert_user(self, username):
        """Добавление записи пользователся в таблицу"""
        name = list(username.keys())[0]
        data = username[name]
        self.cursor.execute('INSERT INTO users_main (username, face_data) VALUES (?, ?)', (name, pickle.dumps(data)))
        self.conn.commit()

    def insert_audio_file(self, user_id, datetime, audio_data):
        """Добавление аудиозаписи в таблицу"""
        self.cursor.execute('INSERT INTO audio_notes (user_id, status, audio_data, datetime) VALUES (?, ?, ?, ?)',
                            (user_id, 0, audio_data, datetime))
        self.conn.commit()

    def delete_user(self, username):
        """Удаление пользователся из таблицы"""
        self.cursor.execute('DELETE FROM users_main WHERE username = ?', (username,))
        self.conn.commit()

    def delete_audio_file(self, audio_id):
        """Добавление аудиозаписи из таблицы"""
        self.cursor.execute('DELETE FROM audio_notes WHERE id = ?', (audio_id,))
        self.conn.commit()

    def get_users(self):
        """Возвращает список пользователей"""
        self.cursor.execute('SELECT * FROM users_main')
        return self.cursor.fetchall()

    def get_audio_files(self, user_id):
        """Возвращает список аудиозаписей"""
        self.cursor.execute('SELECT audio_data FROM audio_notes WHERE id = ?', (user_id))
        return self.cursor.fetchall()

    def close_connection(self):
        self.conn.close()


'''# Пример использования класса
db_manager = DatabaseManager()

# Создание таблицы пользователей
db_manager.create_user_table()

# Создание таблицы аудиофайлов
db_manager.create_audio_table()'''


