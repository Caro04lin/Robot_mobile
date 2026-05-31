""" Commande STOP urgence (arrêt du robot) et reprise des mouvements par l'IMU
"""


from vosk import Model, KaldiRecognizer
import sounddevice as sd
import numpy as np
from scipy.signal import resample_poly
import queue, json, socket
from math import gcd

# -------- CONFIG --------
MODEL_PATH = "vosk-model-small-en-us-0.15"
DEVICE_MIC = 1
ROBOT_IP = "192.168.4.1"
ROBOT_PORT = 5000
GRAMMAR = '["robot forward", "robot stop", "stop", "quit", "[unk]"]'
# -----------------------------------------------------------------------------

info = sd.query_devices(DEVICE_MIC, 'input')
SAMPLE_RATE_MIC = int(info['default_samplerate'])
SAMPLE_RATE_VOSK = 16000
g = gcd(SAMPLE_RATE_MIC, SAMPLE_RATE_VOSK)
UP, DOWN = SAMPLE_RATE_VOSK // g, SAMPLE_RATE_MIC // g

print(f"Micro samplerate detecte: {SAMPLE_RATE_MIC} Hz")

model = Model(MODEL_PATH)
rec = KaldiRecognizer(model, SAMPLE_RATE_VOSK, GRAMMAR)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ROBOT_IP, ROBOT_PORT))
print(" Connected to robot")

q = queue.Queue()

def callback(indata, frames, time, status):
  audio = np.frombuffer(bytes(indata), dtype=np.int16)
  if SAMPLE_RATE_MIC == SAMPLE_RATE_VOSK:
    q.put(audio.tobytes())
  else:
    q.put(resample_poly(audio, UP, DOWN).astype(np.int16).tobytes())

with sd.RawInputStream(
  samplerate=SAMPLE_RATE_MIC,
  device=DEVICE_MIC,
  dtype='int16',
  channels=1,
  callback=callback
):
  print(" Speak now")
  while True:
    data = q.get()
    if rec.AcceptWaveform(data):
      text = json.loads(rec.Result()).get("text", "")
      if not text:
        continue
      print("Recognized:", text)

      if "robot forward" in text:
        sock.sendall(b"UP\n")
      elif "robot stop" in text or text == "stop":
        sock.sendall(b"STOP\n")
      elif "quit" in text:
        break