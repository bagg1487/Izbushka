sudo apt update

sudo apt install python3 python3-pip python3-venv -y

python3 -m venv izba-env

source izba-env/bin/activate

pip install fastapi uvicorn speechrecognition requests

sudo apt install portaudio19-dev python3-pyaudio -y

pip install pyaudio

pip install sounddevice // Или если PyAudio не ставится, альтернатива:

sudo apt install libasound2-dev libportaudio2 libportaudiocpp0 -y
