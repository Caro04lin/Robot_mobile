"""
Programme principal du Raspberry Pi cote utilisateur.

Il lit les IMUs, applique la calibration, detecte les gestes, gere les commandes
vocales optionnelles et envoie la commande finale au robot par TCP.
"""

import argparse
import logging
import sys
import time
import os
import tty
import termios
import select
import threading
import queue
import json
from math import gcd
from pathlib import Path


LOOP_RATE_HZ  = 100
LOOP_INTERVAL_S = 1.0 / LOOP_RATE_HZ

DISPLAY_RATE_HZ = 2
DISPLAY_INTERVAL = 1.0 / DISPLAY_RATE_HZ

VOICE_MODEL_PATH = "vosk-model-small-en-us-0.15"
VOICE_DEVICE_MIC = 1
VOICE_SAMPLE_RATE = 16000
VOICE_GRAMMAR = '["robot forward", "robot stop", "stop", "quit", "[unk]"]'
VOICE_STOP_HOLD_S = 5.0


# -----------------------------------------------------------------------------
# Commande vocale
# -----------------------------------------------------------------------------
class VoiceCommandListener:
  """
  Ecoute Vosk en arriere-plan et expose la derniere commande vocale reconnue.
  "robot forward" -> AVANCER (equivalent UP cote robot)
  "robot stop"/"stop" -> REPOS (equivalent STOP cote robot)
  """

  def __init__(
    self,
    model_path: str = VOICE_MODEL_PATH,
    device_mic: int = VOICE_DEVICE_MIC,
    sample_rate_vosk: int = VOICE_SAMPLE_RATE,
    grammar: str = VOICE_GRAMMAR,
  ):
    self.model_path = model_path
    self.device_mic = device_mic
    self.sample_rate_vosk = sample_rate_vosk
    self.grammar = grammar

    self._audio_queue = queue.Queue()
    self._lock = threading.Lock()
    self._stop_event = threading.Event()
    self._thread = None
    self._latest_command = None
    self._latest_text = ""
    self._error = None

  def start(self) -> None:
    self._stop_event.clear()
    self._thread = threading.Thread(
      target=self._run,
      daemon=True,
      name="VoiceCommandThread",
    )
    self._thread.start()

  def stop(self) -> None:
    self._stop_event.set()
    if self._thread:
      self._thread.join(timeout=2.0)

  def consume_command(self):
    with self._lock:
      command = self._latest_command
      self._latest_command = None
    return command

  @property
  def latest_text(self) -> str:
    with self._lock:
      return self._latest_text

  @property
  def error(self):
    with self._lock:
      return self._error

  def _set_command(self, command: str, text: str) -> None:
    with self._lock:
      self._latest_command = command
      self._latest_text = text

  def _set_error(self, error: Exception) -> None:
    with self._lock:
      self._error = error

  def _run(self) -> None:
    try:
      from vosk import Model, KaldiRecognizer
      import sounddevice as sd
      import numpy as np
      from scipy.signal import resample_poly

      info = sd.query_devices(self.device_mic, "input")
      sample_rate_mic = int(info["default_samplerate"])
      ratio_gcd = gcd(sample_rate_mic, self.sample_rate_vosk)
      up = self.sample_rate_vosk // ratio_gcd
      down = sample_rate_mic // ratio_gcd

      model = Model(self.model_path)
      recognizer = KaldiRecognizer(
        model,
        self.sample_rate_vosk,
        self.grammar,
      )

      def callback(indata, frames, callback_time, status):
        audio = np.frombuffer(bytes(indata), dtype=np.int16)
        if sample_rate_mic == self.sample_rate_vosk:
          self._audio_queue.put(audio.tobytes())
        else:
          resampled = resample_poly(audio, up, down)
          self._audio_queue.put(resampled.astype(np.int16).tobytes())

      with sd.RawInputStream(
        samplerate=sample_rate_mic,
        device=self.device_mic,
        dtype="int16",
        channels=1,
        callback=callback,
      ):
        while not self._stop_event.is_set():
          try:
            data = self._audio_queue.get(timeout=0.2)
          except queue.Empty:
            continue

          if recognizer.AcceptWaveform(data):
            text = json.loads(recognizer.Result()).get("text", "")
            if not text:
              continue

            if "robot forward" in text:
              self._set_command("AVANCER", text)
            elif "robot stop" in text or text == "stop":
              self._set_command("REPOS", text)
            elif "quit" in text:
              self._set_command("REPOS", text)
              self._stop_event.set()

    except Exception as error:
      self._set_error(error)

# Utilitaires terminal
class TerminalDisplay:
  """
  Affichage terminal compatible SSH.
  Affiche les angles relatifs a la posture calibree.
  """

  CMD_COLORS = {
    "AVANCER":    "\033[92m",
    "RECULER":    "\033[93m",
    "TOURNER_DROITE": "\033[96m",
    "TOURNER_GAUCHE": "\033[96m",
    "REPOS":     "\033[90m",
    "ARRET_URGENCE": "\033[91m",
  }

  RESET = "\033[0m"
  BOLD = "\033[1m"
  GRAY = "\033[90m"

  def draw(
    self,
    rel_angles: dict,
    command: str,
    vote_score: str,
    loop_hz: float,
    imu_command: str = "REPOS",
    voice_command: str = "AUCUNE",
    voice_status: str = "",
    rotation_shoulders: float = 0.0,
    back_roll: float = 0.0,
  ) -> None:
    """
    Affiche l'etat courant du systeme.
    """

    os.system("clear")

    lg = rel_angles.get("left_shoulder", {"pitch": 0.0, "roll": 0.0, "yaw": 0.0})
    rg = rel_angles.get("right_shoulder", {"pitch": 0.0, "roll": 0.0, "yaw": 0.0})
    bk = rel_angles.get("back",      {"pitch": 0.0, "roll": 0.0, "yaw": 0.0})

    col = self.CMD_COLORS.get(command, "\033[97m")
    imu_col = self.CMD_COLORS.get(imu_command, "\033[97m")
    voice_col = self.CMD_COLORS.get(voice_command, "\033[97m")
    SEP = "-" * 58

    print(f" {self.GRAY}ICAM Pi4 {loop_hz:.0f} Hz Vote: {vote_score}{self.RESET}")
    print(f" {SEP}")

    print(
      f" Epaule gauche : "
      f"pitch {lg['pitch']:+6.1f}d  "
      f"roll {lg['roll']:+6.1f}d"
    )

    print(
      f" Epaule droite : "
      f"pitch {rg['pitch']:+6.1f}d  "
      f"roll {rg['roll']:+6.1f}d"
    )

    print(
      f" Dos      : "
      f"pitch {bk['pitch']:+6.1f}d  "
      f"roll {bk['roll']:+6.1f}d  "
      f"yaw {bk.get('yaw', 0.0):+6.1f}d"
    )

    print(f" {SEP}")

    print(f" Rotation epaules : {rotation_shoulders:+6.1f}d")
    print(f" Rotation dos roll : {back_roll:+6.1f}d")

    print(f" {SEP}")

    voice_suffix = f" {self.GRAY}({voice_status}){self.RESET}" if voice_status else ""

    print(f" Commande IMU  : {imu_col}{imu_command}{self.RESET}")
    print(f" Commande vocale : {voice_col}{voice_command}{self.RESET}{voice_suffix}")
    print(
      f" {self.BOLD}Envoyee robot  : "
      f"{col}{command}{self.RESET}"
    )

    print(f" {SEP}")
    print(f" {self.GRAY}[q] Quitter [r] Recalibrer{self.RESET}")

  def clear_and_reset(self) -> None:
    os.system("clear")


class RawKeyReader:

  def __init__(self):
    self._fd = sys.stdin.fileno()
    self._old_settings = None

  def enable(self) -> None:
    try:
      self._old_settings = termios.tcgetattr(self._fd)
      tty.setraw(self._fd)
    except termios.error:
      self._old_settings = None

  def disable(self) -> None:
    if self._old_settings:
      try:
        termios.tcsetattr(
          self._fd,
          termios.TCSADRAIN,
          self._old_settings
        )
      except termios.error:
        pass

  def read_key_nonblocking(self) -> str:
    try:
      if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    except Exception:
      pass
    return ""


# CLI
def parse_args() -> argparse.Namespace:

  parser = argparse.ArgumentParser(
    description="ICAM - Controle IMU temps reel | Raspberry Pi 4",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
  )

  parser.add_argument(
    "--robot-ip",
    default="192.168.1.101",
    metavar="IP",
    help="Adresse IP du Raspberry Pi robot"
  )

  parser.add_argument(
    "--no-log",
    action="store_true",
    help="Desactive l'enregistrement CSV"
  )

  parser.add_argument(
    "--session",
    default="session_pi4",
    metavar="NOM",
    help="Nom de la session CSV"
  )

  parser.add_argument(
    "--log-level",
    default="INFO",
    choices=["DEBUG", "INFO", "WARNING"],
    help="Niveau de verbosite"
  )

  parser.add_argument(
    "--no-voice",
    action="store_true",
    help="Desactive les commandes vocales"
  )

  parser.add_argument(
    "--voice-model",
    default=VOICE_MODEL_PATH,
    metavar="DOSSIER",
    help="Chemin vers le modele Vosk"
  )

  parser.add_argument(
    "--voice-device",
    type=int,
    default=VOICE_DEVICE_MIC,
    metavar="ID",
    help="Identifiant du micro pour sounddevice"
  )

  return parser.parse_args()


# Logging
def setup_logging(level: str) -> None:

  logging.basicConfig(
    level=getattr(logging, level),
    format="%(asctime)s [%(levelname)s] %(name)s - %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
      logging.FileHandler("icam_robot.log", encoding="utf-8"),
      logging.StreamHandler(sys.stderr),
    ]
  )

  logging.getLogger().handlers[1].setLevel(logging.WARNING)


# Labels rapides
QUICK_LABELS = {
  "1": "avancer",
  "2": "reculer",
  "3": "tourner_droite",
  "4": "tourner_gauche",
  "5": "repos",
}


# MAIN
def main() -> None:

  args = parse_args()
  setup_logging(args.log_level)

  logger = logging.getLogger("Main")

  logger.info("=== Demarrage ICAM - Pi4 humain ===")

  
  # Imports projet
  
  try:
    from imu_reader import IMUManager
    from gesture_detector import GestureDetector, RobotCommand
    from calibration import CalibrationManager
    from data_logger import DataLogger
    from tcp_sender import TCPCommandSender

  except ImportError as e:
    print(f"\nERREUR - Module manquant : {e}", file=sys.stderr)
    print("Tous les fichiers .py doivent etre dans le meme dossier.")
    sys.exit(1)

  
  # Initialisation objets
  
  imu_manager = IMUManager(i2c_bus_number=1)

  detector = GestureDetector()

  calibration = CalibrationManager(imu_manager)

  sender = TCPCommandSender(robot_ip=args.robot_ip)

  data_logger = DataLogger() if not args.no_log else None

  display = TerminalDisplay()

  key_reader = RawKeyReader()

  voice_listener = None

  if not args.no_voice:
    voice_listener = VoiceCommandListener(
      model_path=args.voice_model,
      device_mic=args.voice_device,
    )

  os.system("clear")

  print("\n Initialisation du systeme ICAM - Raspberry Pi 4...\n")

  current_label = ""

  try:

    
    # 1. Initialisation IMUs
    
    print(" [1/3] Initialisation des capteurs IMU...", end="", flush=True)

    imu_manager.initialize()

    print(" OK")

    
    # 2. Calibration
    
    print(" [2/3] Calibration...", end="", flush=True)

    if not calibration.load():

      print()

      calibration.calibrate(detector, verbose=True)

    else:

      print(" OK (chargee depuis fichier)")

      detector.reset_filters()

      calibration.restore_references(detector)

      print(
        "\n Recalibrer maintenant a"
        "Appuyez sur 'o' + ENTREE, ou ENTREE pour continuer : ",
        end="",
        flush=True
      )

      answer = input().strip().lower()

      if answer == "o":
        calibration.calibrate(detector, verbose=True)

    detector.reset_filters()

    calibration.restore_references(detector)

    
    # 3. Services
    
    print(" [3/3] Demarrage TCP et CSV...", end="", flush=True)

    sender.start()

    if voice_listener:
      voice_listener.start()
      logger.info("Commande vocale demarree.")

    if data_logger:
      csv_path = data_logger.start_session(args.session)
      logger.info(f"CSV ouvert : {csv_path}")

    print(" OK")

    print(f"\n Connexion vers robot : {args.robot_ip}:5005")

    if data_logger:
      print(f" Enregistrement CSV : {csv_path}")

    time.sleep(0.5)

    
    # RAW terminal
    
    os.system("clear")

    key_reader.enable()

    
    # BOUCLE PRINCIPALE
    
    last_display = time.monotonic()
    last_hz_check = time.monotonic()

    hz_counter = 0
    measured_hz = 100.0

    running = True
    voice_error_logged = False
    voice_active_command = None
    voice_display_command = "AUCUNE"
    voice_status = ""
    wait_imu_repos_after_voice_stop = False
    voice_stop_hold_until = 0.0

    while running:

      t_start = time.monotonic()

      
      # 1. Lecture IMUs
      
      raw_data = imu_manager.read_all()

      clean_data = calibration.apply(raw_data)

      
      # 2. Detection geste
      
      imu_command = detector.update(clean_data)
      command = imu_command

      if voice_listener:
        voice_error = voice_listener.error
        if voice_error and not voice_error_logged:
          logger.warning(f"Commande vocale indisponible : {voice_error}")
          voice_error_logged = True

        voice_command = voice_listener.consume_command()
        if voice_command:
          recognized_command = RobotCommand(voice_command)

          if recognized_command == RobotCommand.AVANCER:
            voice_active_command = RobotCommand.AVANCER
            voice_display_command = recognized_command.value
            voice_status = "active"
            wait_imu_repos_after_voice_stop = False
            voice_stop_hold_until = 0.0
          elif recognized_command == RobotCommand.REPOS:
            voice_active_command = None
            voice_display_command = recognized_command.value
            voice_status = f"pause {VOICE_STOP_HOLD_S:.0f}s"
            wait_imu_repos_after_voice_stop = False
            voice_stop_hold_until = time.monotonic() + VOICE_STOP_HOLD_S
            command = RobotCommand.REPOS

          logger.info(
            f"Commande vocale : {voice_listener.latest_text} -> "
            f"{recognized_command.value}"
          )

      
      # 3. Envoi robot
      
      if voice_stop_hold_until > 0.0:
        remaining_voice_stop = voice_stop_hold_until - time.monotonic()
        if remaining_voice_stop > 0.0:
          command = RobotCommand.REPOS
          voice_status = f"pause {remaining_voice_stop:.1f}s"
        else:
          voice_stop_hold_until = 0.0
          voice_display_command = "AUCUNE"
          voice_status = ""

      elif voice_active_command:
        command = voice_active_command

      sender.set_command(command.value)

      
      # 4. CSV
      
      debug = detector.get_debug_info()

      if data_logger and data_logger.is_recording:

        data_logger.log(
          imu_data = clean_data,
          angles  = detector.last_angles,
          command  = command.value,
          fsm_state = debug.get("vote_score", ""),
          label   = current_label,
        )

      
      # 5. Clavier
      
      key = key_reader.read_key_nonblocking()

      if key:

        if key == "q" or key == "\x03":
          running = False

        elif key == "r":

          key_reader.disable()

          display.clear_and_reset()

          calibration.calibrate(detector, verbose=True)

          detector.reset_filters()

          calibration.restore_references(detector)

          key_reader.enable()

        elif key in QUICK_LABELS:

          current_label = QUICK_LABELS[key]

          if data_logger:
            data_logger.set_label(current_label)

        elif key == " ":

          current_label = ""

          if data_logger:
            data_logger.set_label("")

      
      # 6. Mesure frequence reelle
      
      hz_counter += 1

      now = time.monotonic()

      if now - last_hz_check >= 1.0:

        measured_hz = hz_counter / (now - last_hz_check)

        hz_counter = 0

        last_hz_check = now

      
      # 7. Affichage terminal
      
      if now - last_display >= DISPLAY_INTERVAL:

        last_display = now

        display.draw(
          rel_angles     = debug["angles_rel"],
          command       = command.value,
          vote_score     = debug["vote_score"],
          loop_hz       = measured_hz,
          imu_command     = imu_command.value,
          voice_command    = voice_display_command,
          voice_status    = voice_status,
          rotation_shoulders = debug.get("rotation_shoulders", 0.0),
          back_roll     = debug.get("back_roll", 0.0),
        )

      
      # 8. Respect timing 100 Hz
      
      elapsed = time.monotonic() - t_start

      sleep_t = LOOP_INTERVAL_S - elapsed

      if sleep_t > 0:

        time.sleep(sleep_t)

      else:

        logger.debug(
          f"Boucle lente : {elapsed*1000:.1f} ms "
          f"(budget = {LOOP_INTERVAL_S*1000:.0f} ms)"
        )

  except Exception as e:

    logger.critical(
      f"Erreur fatale dans la boucle principale : {e}",
      exc_info=True
    )

    raise

  finally:

    key_reader.disable()

    display.clear_and_reset()

    print("\n Arret du systeme ICAM...\n")

    if voice_listener:
      voice_listener.stop()

    sender.stop()

    logger.info(f"TCP stats : {sender.stats}")

    if data_logger and data_logger.is_recording:

      summary = data_logger.stop_session()

      print(
        f" Session CSV : "
        f"{summary['rows']} lignes "
        f"en {summary['duration_s']}s"
      )

      logger.info(f"Session CSV terminee : {summary}")

    imu_manager.close()

    print(" OK Systeme arrete proprement.\n")

    logger.info("=== Systeme arrete ===")


if __name__ == "__main__":
  main()
