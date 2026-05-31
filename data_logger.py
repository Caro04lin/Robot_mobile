"""
Enregistrement CSV des essais IMU.

Chaque session cree un fichier dans logs/. Les lignes contiennent les donnees
capteurs, les angles filtres, la commande robot et un label manuel optionnel.
Ces fichiers servent a analyser les gestes et a ajuster les seuils.
"""

import csv
import os
import time
import logging
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger("DataLogger")

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
LOG_DIRECTORY = Path("./logs")  # Repertoire de sortie des fichiers CSV

# En-tetes du fichier CSV (une ligne par cycle de lecture, pour chaque IMU)
CSV_HEADERS = [
  "timestamp_s",   # Timestamp depuis le demarrage de la session (secondes)
  "datetime",     # Date/heure lisible (YYYY-MM-DD HH:MM:SS.mmm)
  "imu_name",     # Identifiant de l'IMU (left_shoulder, right_shoulder, back)
  "ax_g",       # Acceleration X en g
  "ay_g",       # Acceleration Y en g
  "az_g",       # Acceleration Z en g
  "gx_dps",      # Vitesse angulaire X en deg/s
  "gy_dps",      # Vitesse angulaire Y en deg/s
  "gz_dps",      # Vitesse angulaire Z en deg/s
  "pitch_deg",    # Angle pitch filtre en degres
  "roll_deg",     # Angle roll filtre en degres
  "shoulder_diff_deg",# Difference de roll epaules (droite - gauche), colonne synthetique
  "command",     # Commande robot active a cet instant
  "fsm_state",    # Etat de la FSM (IDLE / DETECTING / CONFIRMED)
  "label",      # Label manuel optionnel (pour annotation)
]


class DataLogger:
  """
  Enregistreur de donnees IMU vers CSV, avec support de sessions multiples.

  Chaque session cree un fichier CSV horodate dans LOG_DIRECTORY/.
  Les donnees sont ecrites en temps reel (flush apres chaque ligne).

  Utilisation :
    logger = DataLogger()
    logger.start_session("calibration_avant")
    logger.log(imu_data, angles, command, fsm_state)
    logger.stop_session()
  """

  def __init__(self, log_dir: Path = LOG_DIRECTORY):
    self.log_dir     = log_dir
    self._csv_file    = None
    self._writer     = None
    self._session_name  = None
    self._session_start = None
    self._row_count   = 0
    self._current_label = ""    # Label courant, modifiable en cours de session

    # Creation du repertoire de logs si inexistant
    self.log_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"DataLogger initialise. Repertoire : {self.log_dir.resolve()}")

  def start_session(self, session_name: str = "session") -> Path:
    """
    Demarre une nouvelle session d'enregistrement.
    Cree un fichier CSV nomme avec le nom de session et le timestamp.

    Args:
      session_name : nom descriptif de la session (ex: "calibration_avant")

    Returns:
      Chemin complet du fichier CSV cree
    """
    if self._csv_file is not None:
      logger.warning("Session deja en cours. Arret de l'ancienne session...")
      self.stop_session()

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = self.log_dir / f"{session_name}_{timestamp_str}.csv"

    self._csv_file   = open(filename, "w", newline="", encoding="utf-8")
    self._writer    = csv.DictWriter(self._csv_file, fieldnames=CSV_HEADERS)
    self._writer.writeheader()
    self._csv_file.flush()

    self._session_name = session_name
    self._session_start = time.monotonic()
    self._row_count   = 0

    logger.info(f"Session demarree : {filename}")
    return filename

  def set_label(self, label: str) -> None:
    """
    Change le label courant pour annoter les prochaines lignes enregistrees.
    Utile pour marquer les sections (ex: "avancer", "repos", "tourner_droite")

    Args:
      label : chaine de caracteres decrivant le mouvement en cours
    """
    self._current_label = label
    logger.debug(f"Label de session change : '{label}'")

  def log(
    self,
    imu_data:  dict,
    angles:   dict,
    command:   str,
    fsm_state:  str,
    label:    Optional[str] = None
  ) -> None:
    """
    Enregistre une trame de donnees IMU dans le CSV.

    Ecrit 3 lignes par appel (une par IMU) avec toutes les colonnes remplies.

    Args:
      imu_data : donnees brutes des 3 IMUs (sortie IMUManager.read_all())
      angles  : angles filtres des 3 IMUs (sortie GestureDetector.last_angles)
      command  : commande robot active (string, ex: "AVANCER")
      fsm_state : etat FSM courant (string, ex: "CONFIRMED")
      label   : label optionnel pour cette ligne (ecrase self._current_label)
    """
    if self._writer is None:
      return  # Silencieux si pas de session active

    now_monotonic = time.monotonic() - self._session_start
    now_datetime = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    active_label = label if label is not None else self._current_label

    # Calcul de la difference de roll epaules (feature cle pour rotation)
    shoulder_diff = 0.0
    if "right_shoulder" in angles and "left_shoulder" in angles:
      shoulder_diff = (angles["right_shoulder"].roll -
               angles["left_shoulder"].roll)

    # Ecriture de 3 lignes (une par IMU)
    for imu_name in ("left_shoulder", "right_shoulder", "back"):
      if imu_name not in imu_data:
        continue

      d = imu_data[imu_name]
      a = angles.get(imu_name)

      row = {
        "timestamp_s":    round(now_monotonic, 4),
        "datetime":     now_datetime,
        "imu_name":     imu_name,
        "ax_g":       round(d["ax"], 5),
        "ay_g":       round(d["ay"], 5),
        "az_g":       round(d["az"], 5),
        "gx_dps":      round(d["gx"], 3),
        "gy_dps":      round(d["gy"], 3),
        "gz_dps":      round(d["gz"], 3),
        "pitch_deg":     round(a.pitch, 3) if a else 0.0,
        "roll_deg":     round(a.roll, 3) if a else 0.0,
        "shoulder_diff_deg": round(shoulder_diff, 3),
        "command":      command,
        "fsm_state":     fsm_state,
        "label":       active_label,
      }
      self._writer.writerow(row)

    # Flush immediat donnees visibles meme si le script est interrompu
    self._csv_file.flush()
    self._row_count += 3

  def stop_session(self) -> dict:
    """
    Termine la session d'enregistrement et ferme le fichier CSV.

    Returns:
      Resume de la session (nom, duree, nombre de lignes)
    """
    if self._csv_file is None:
      logger.warning("Aucune session active.")
      return {}

    duration = time.monotonic() - self._session_start

    self._csv_file.close()
    self._csv_file = None
    self._writer  = None

    summary = {
      "session":  self._session_name,
      "rows":    self._row_count,
      "duration_s": round(duration, 2),
    }
    logger.info(
      f"Session '{self._session_name}' terminee. "
      f"{self._row_count} lignes enregistrees en {duration:.1f}s."
    )

    self._session_name = None
    self._session_start = None
    self._row_count   = 0

    return summary

  @property
  def is_recording(self) -> bool:
    """True si une session est en cours."""
    return self._csv_file is not None

  def list_sessions(self) -> list:
    """
    Liste tous les fichiers CSV dans le repertoire de logs.

    Returns:
      Liste de chemins vers les fichiers CSV, tries par date (plus recent en premier)
    """
    csvs = sorted(self.log_dir.glob("*.csv"), reverse=True)
    return csvs


# -----------------------------------------------------------------------------
# Classe utilitaire pour analyse rapide des seuils (console)
# -----------------------------------------------------------------------------
class ThresholdAnalyzer:
  """
  Analyse un fichier CSV pour suggerer des seuils de detection optimaux.

  Lit le CSV produit par DataLogger et, pour chaque label de mouvement,
  calcule les statistiques (min, max, moyenne, std) des angles pitch et roll.

  Usage :
    analyzer = ThresholdAnalyzer("logs/calibration_avant_20241201_120000.csv")
    analyzer.analyze()
    analyzer.print_report()
  """

  def __init__(self, csv_path: str):
    self.csv_path = Path(csv_path)
    self._data  = []

  def load(self) -> None:
    """Charge les donnees CSV en memoire."""
    with open(self.csv_path, "r", encoding="utf-8") as f:
      reader = csv.DictReader(f)
      self._data = list(reader)
    logger.info(f"Fichier charge : {len(self._data)} lignes.")

  def analyze(self) -> dict:
    """
    Analyse les angles par label et par IMU.

    Returns:
      dict[label][imu_name] stats (min, max, mean, std, count)
    """
    from collections import defaultdict
    import statistics

    # Regroupement des valeurs par (label, imu_name)
    groups = defaultdict(lambda: defaultdict(list))
    for row in self._data:
      label  = row.get("label", "")
      imu_name = row.get("imu_name", "")
      if not label or not imu_name:
        continue
      try:
        groups[label][imu_name].append({
          "pitch": float(row["pitch_deg"]),
          "roll": float(row["roll_deg"]),
          "shoulder_diff": float(row["shoulder_diff_deg"]),
        })
      except (ValueError, KeyError):
        pass

    # Calcul des statistiques
    results = {}
    for label, imus in groups.items():
      results[label] = {}
      for imu_name, samples in imus.items():
        pitches = [s["pitch"] for s in samples]
        rolls  = [s["roll"] for s in samples]
        diffs  = [s["shoulder_diff"] for s in samples]
        results[label][imu_name] = {
          "count":   len(samples),
          "pitch_min": round(min(pitches), 2),
          "pitch_max": round(max(pitches), 2),
          "pitch_mean": round(statistics.mean(pitches), 2),
          "pitch_std": round(statistics.stdev(pitches) if len(pitches) > 1 else 0, 2),
          "roll_min":  round(min(rolls), 2),
          "roll_max":  round(max(rolls), 2),
          "roll_mean": round(statistics.mean(rolls), 2),
          "shoulder_diff_mean": round(statistics.mean(diffs), 2),
        }
    return results

  def print_report(self) -> None:
    """Affiche un rapport textuel des statistiques dans la console."""
    results = self.analyze()
    print("\n" + "="*70)
    print(f"RAPPORT D'ANALYSE : {self.csv_path.name}")
    print("="*70)
    for label, imus in results.items():
      print(f"\n Label : '{label}'")
      for imu_name, stats in imus.items():
        if imu_name == "back":
          print(
            f"  [{imu_name}] "
            f"Pitch : [{stats['pitch_min']:+.1f}, {stats['pitch_max']:+.1f}] "
            f"(moy={stats['pitch_mean']:+.1f}, std={stats['pitch_std']:.1f})"
          )
        elif imu_name in ("left_shoulder", "right_shoulder"):
          print(
            f"  [{imu_name}] "
            f"Roll diff epaules : {stats['shoulder_diff_mean']:+.1f}"
          )
    print("\n" + "="*70)
    print("SEUILS SUGGERES (a titre indicatif) :")
    # Affichage des recommandations si les labels correspondent aux gestes
    forward_stats = results.get("avancer",    {}).get("back", {})
    backward_stats = results.get("reculer",    {}).get("back", {})
    rotate_r_stats = results.get("tourner_droite", {}).get("right_shoulder", {})
    if forward_stats:
      print(f" THRESHOLD_FORWARD_DEG = {forward_stats.get('pitch_mean', '<-'):.1f} "
         f"(pitch moyen lors de 'avancer')")
    if backward_stats:
      print(f" THRESHOLD_BACKWARD_DEG = {backward_stats.get('pitch_mean', '<-'):.1f} "
         f"(pitch moyen lors de 'reculer')")
    if rotate_r_stats:
      print(f" THRESHOLD_ROTATE_DEG  = {abs(rotate_r_stats.get('shoulder_diff_mean', 0)):.1f} "
         f"(diff roll epaules lors de 'tourner_droite')")
    print("="*70 + "\n")


# -----------------------------------------------------------------------------
# Test standalone : python data_logger.py
# -----------------------------------------------------------------------------
if __name__ == "__main__":
  # Exemple d'utilisation avec des donnees fictives
  dl = DataLogger()
  filepath = dl.start_session("test_fictif")
  print(f"Enregistrement dans : {filepath}")

  # Simulation de quelques trames
  from gesture_detector_claude import FilteredAngles

  fake_imu_data = {
    "left_shoulder": {"ax": 0.01, "ay": 0.0, "az": 1.0, "gx": 0.1, "gy": 0.2, "gz": 0.0, "temp": 25.0},
    "right_shoulder": {"ax": 0.01, "ay": 0.0, "az": 1.0, "gx": 0.1, "gy": 0.2, "gz": 0.0, "temp": 25.0},
    "back":      {"ax": 0.3, "ay": 0.0, "az": 0.95,"gx": 0.1, "gy": 1.5, "gz": 0.0, "temp": 25.0},
  }
  fake_angles = {
    "left_shoulder": FilteredAngles(pitch=2.0, roll=-1.5),
    "right_shoulder": FilteredAngles(pitch=2.0, roll= 1.5),
    "back":      FilteredAngles(pitch=22.0, roll= 0.5),
  }

  dl.set_label("avancer")
  for _ in range(5):
    dl.log(fake_imu_data, fake_angles, "AVANCER", "CONFIRMED")
    time.sleep(0.1)

  summary = dl.stop_session()
  print(f"Session terminee : {summary}")
