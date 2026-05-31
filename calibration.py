"""
Calibration des IMUs.

Le module calcule les offsets des capteurs, stabilise le filtre, puis enregistre
la posture neutre de l'utilisateur. Les resultats sont sauvegardes dans
calibration_offsets.json et reutilises au demarrage suivant.
"""

import json
import time
import logging
from pathlib import Path

logger = logging.getLogger("Calibration")

CALIBRATION_FILE = Path("./calibration_offsets.json")

SAMPLES_RAW_OFFSET = 100
SAMPLES_WARMUP   = 150
SAMPLES_ANGLE_REF = 100
LOOP_INTERVAL   = 1.0 / 100  # 100 Hz


class CalibrationManager:
  """
  Calibration en 3 phases pour 1 a 3 capteurs IMU.

  Interface publique :
    calibrate(detector)    : procedure complete interactive
    apply(raw_data)      : soustrait les offsets bruts (phase 1)
    load() / save()      : persistance JSON
    restore_references(det)  : restaure les angles de reference dans le detecteur
    is_calibrated       : True si pret a l'emploi
  """

  def __init__(self, imu_manager):
    """
    Args:
      imu_manager : instance IMUManager (read_all() retourne un dict
             {nom_capteur: {ax, ay, az, gx, gy, gz}}).
             Seuls les capteurs presents dans ce dict sont traites.
    """
    self.imu_manager = imu_manager
    self.raw_offsets: dict = {}  # offsets bruts par capteur present
    self.angle_refs: dict = {}  # angles de repos par capteur present
    self._calibrated = False

  # -----------------------------------------------------------------------------
  # Procedure complete
  # -----------------------------------------------------------------------------

  def calibrate(self, detector, verbose: bool = True) -> None:
    """
    Lance la procedure complete de calibration en 3 phases.

    S'adapte automatiquement aux capteurs presents dans imu_data :
    si seul le dos est branche, seul "back" est calibre.

    Args:
      detector : instance GestureDetector
      verbose : affiche les instructions et la progression
    """
    # Lire une fois pour savoir quels capteurs sont disponibles
    sample = self.imu_manager.read_all()
    active_sensors = list(sample.keys())
    has_shoulders = (
      "left_shoulder" in active_sensors and
      "right_shoulder" in active_sensors
    )

    if verbose:
      print("\n" + "=" * 58)
      print("     CALIBRATION SYSTEME ICAM")
      print("=" * 58)
      print(f"\nCapteurs detectes : {', '.join(active_sensors)}")
      if not has_shoulders:
        print("  Mode DOS SEUL (pas de capteurs d'epaules)")
      print("\nInstructions :")
      print(" - Asseyez-vous dans votre position de conduite habituelle")
      print(" - Dos aussi droit que possible, naturellement")
      print(" - Bras le long du corps, epaules detendues")
      print(" - Restez IMMOBILE pendant toute la calibration (~5 sec)")
      print("\nAppuyez sur ENTREE quand vous etes pret...")
      input()

    # Phase 1 : offsets bruts 
    if verbose:
      print("\n[1/3] Capture des offsets bruts (immobile)...")
    self._capture_raw_offsets(active_sensors, verbose)

    # Phase 2 : chauffe du filtre 
    if verbose:
      print("[2/3] Chauffe du filtre (ne pas bouger)...")
    detector.reset_filters()
    self._warmup_filter(detector, verbose)

    # Phase 3 : angles de reference 
    if verbose:
      print("[3/3] Capture des angles de reference (posture neutre)...")
    self._capture_angle_refs(detector, verbose)

    # Transmettre les angles de reference au detecteur
    from gesture_detector import FilteredAngles
    ref_dict = {
      name: FilteredAngles(pitch=v["pitch"], roll=v["roll"])
      for name, v in self.angle_refs.items()
    }
    detector.set_reference(ref_dict)

    self._calibrated = True
    self.save()

    if verbose:
      print("\n Calibration terminee !")
      print("\nAngles de repos captures :")
      for name, ref in self.angle_refs.items():
        print(
          f" {name:<16s} : "
          f"pitch={ref['pitch']:+6.2f}  roll={ref['roll']:+6.2f}"
        )
      print()

  # -----------------------------------------------------------------------------
  # Phase 1 : offsets bruts
  # -----------------------------------------------------------------------------

  def _capture_raw_offsets(self, active_sensors: list, verbose: bool) -> None:
    """
    Moyenne SAMPLES_RAW_OFFSET lectures brutes pour chaque capteur present.

    Correction de la gravite selon l'orientation physique du capteur :
      Epaules (z bas) : az -1g au repos
        offset_az = moyenne_az - (-1.0) = moyenne_az + 1.0
      Dos (y haut) : ay +1g au repos
        offset_ay = moyenne_ay - 1.0
    Tous les autres axes : la moyenne est l'offset (valeur attendue = 0).
    """
    sums = {
      name: {"ax": 0.0, "ay": 0.0, "az": 0.0,
          "gx": 0.0, "gy": 0.0, "gz": 0.0}
      for name in active_sensors
    }

    for i in range(SAMPLES_RAW_OFFSET):
      data = self.imu_manager.read_all()
      for name in active_sensors:
        if name not in data:
          continue
        for axis in ("ax", "ay", "az", "gx", "gy", "gz"):
          sums[name][axis] += data[name][axis]
      if verbose and (i + 1) % 25 == 0:
        pct = (i + 1) * 100 // SAMPLES_RAW_OFFSET
        print(f"  {pct:3d}% {'' * (pct // 5)}")
      time.sleep(LOOP_INTERVAL)

    self.raw_offsets = {}
    for name, s in sums.items():
      offsets = {
        axis: s[axis] / SAMPLES_RAW_OFFSET
        for axis in ("ax", "ay", "az", "gx", "gy", "gz")
      }
      # Correction de la composante gravitationnelle
      if name in ("left_shoulder", "right_shoulder"):
        # z bas : az -1g offset = moy_az - (-1g) = moy_az + 1
        offsets["az"] = s["az"] / SAMPLES_RAW_OFFSET + 1.0
      elif name == "back":
        # y haut : ay +1g offset = moy_ay - 1g
        offsets["ay"] = s["ay"] / SAMPLES_RAW_OFFSET - 1.0
        # az 0g (z arriere) : pas de correction speciale
      self.raw_offsets[name] = offsets

  # -----------------------------------------------------------------------------
  # Phase 2 : chauffe du filtre
  # -----------------------------------------------------------------------------

  def _warmup_filter(self, detector, verbose: bool) -> None:
    """
    Fait tourner le filtre complementaire SAMPLES_WARMUP cycles
    pour qu'il converge depuis 0 vers les vrais angles de la posture.
    Les valeurs ne sont pas memorisees.
    """
    for i in range(SAMPLES_WARMUP):
      raw  = self.imu_manager.read_all()
      clean = self.apply(raw)
      detector.update(clean)
      if verbose and (i + 1) % 50 == 0:
        pct = (i + 1) * 100 // SAMPLES_WARMUP
        print(f"  {pct:3d}% {'' * (pct // 5)}")
      time.sleep(LOOP_INTERVAL)

  # -----------------------------------------------------------------------------
  # Phase 3 : angles de reference
  # -----------------------------------------------------------------------------

  def _capture_angle_refs(self, detector, verbose: bool) -> None:
    """
    Moyenne les angles FILTRES sur SAMPLES_ANGLE_REF cycles.

    Ces angles representent la posture neutre reelle (dos courbe,
    capteurs legerement de travers, port asymetrique...).
    Seuls les capteurs presents dans last_angles sont memorises.
    """
    # Initialiser les accumulateurs pour les capteurs actifs seulement
    sums: dict = {}

    for i in range(SAMPLES_ANGLE_REF):
      raw  = self.imu_manager.read_all()
      clean = self.apply(raw)
      detector.update(clean)

      # detector.last_angles ne contient que les capteurs presents
      for name, a in detector.last_angles.items():
        if name not in sums:
          sums[name] = {"pitch": 0.0, "roll": 0.0, "count": 0}
        sums[name]["pitch"] += a.pitch
        sums[name]["roll"] += a.roll
        sums[name]["count"] += 1

      if verbose and (i + 1) % 25 == 0:
        pct = (i + 1) * 100 // SAMPLES_ANGLE_REF
        print(f"  {pct:3d}% {'' * (pct // 5)}")
      time.sleep(LOOP_INTERVAL)

    self.angle_refs = {
      name: {
        "pitch": s["pitch"] / s["count"],
        "roll": s["roll"] / s["count"],
      }
      for name, s in sums.items()
      if s["count"] > 0
    }

  # -----------------------------------------------------------------------------
  # Application des offsets bruts
  # -----------------------------------------------------------------------------

  def apply(self, raw_data: dict) -> dict:
    """
    Soustrait les offsets bruts (phase 1) des donnees IMU brutes.

    Seuls les capteurs presents dans raw_data ET dans raw_offsets
    sont corriges. Les autres passent tels quels (pas de plantage).

    Args:
      raw_data : {nom_imu: {ax, ay, az, gx, gy, gz, temp}}

    Returns:
      dict de meme structure avec les valeurs corrigees
    """
    if not self.raw_offsets:
      return raw_data  # pas encore calibre donnees brutes

    corrected = {}
    for name, vals in raw_data.items():
      if name not in self.raw_offsets:
        # Capteur present dans imu_data mais pas calibre (ex: ajoute apres)
        # on le passe tel quel plutot que de planter
        corrected[name] = vals
        continue
      off = self.raw_offsets[name]
      corrected[name] = {
        "ax":  vals["ax"] - off["ax"],
        "ay":  vals["ay"] - off["ay"],
        "az":  vals["az"] - off["az"],
        "gx":  vals["gx"] - off["gx"],
        "gy":  vals["gy"] - off["gy"],
        "gz":  vals["gz"] - off["gz"],
        "temp": vals.get("temp", 0.0),
      }
    return corrected

  # -----------------------------------------------------------------------------
  # Persistance JSON
  # -----------------------------------------------------------------------------

  def save(self, path: Path = CALIBRATION_FILE) -> None:
    """Sauvegarde offsets et angles de reference dans un fichier JSON."""
    data = {
      "raw_offsets": self.raw_offsets,
      "angle_refs": self.angle_refs,
    }
    with open(path, "w", encoding="utf-8") as f:
      json.dump(data, f, indent=2)
    logger.info(f"Calibration sauvegardee : {path}")

  def load(self, path: Path = CALIBRATION_FILE) -> bool:
    """
    Charge la calibration depuis un fichier JSON.

    Returns:
      True si le fichier existe et contient des donnees valides.
    """
    if not path.exists():
      logger.warning(f"Pas de fichier de calibration : {path}")
      return False
    with open(path, "r", encoding="utf-8") as f:
      data = json.load(f)
    self.raw_offsets = data.get("raw_offsets", {})
    self.angle_refs = data.get("angle_refs", {})
    self._calibrated = bool(self.raw_offsets and self.angle_refs)
    if self._calibrated:
      logger.info(f"Calibration chargee depuis {path}")
    return self._calibrated

  def restore_references(self, detector) -> None:
    """
    Restaure les angles de reference dans le detecteur apres un load().

    A appeler apres load() + detector.reset_filters().
    Seuls les capteurs presents dans angle_refs sont restaures ;
    les capteurs absents du fichier gardent 0 comme reference.
    """
    if not self.angle_refs:
      logger.warning("restore_references : aucun angle de reference disponible.")
      return
    from gesture_detector import FilteredAngles
    ref_dict = {
      name: FilteredAngles(pitch=v["pitch"], roll=v["roll"])
      for name, v in self.angle_refs.items()
    }
    detector.set_reference(ref_dict)
    logger.info(
      f"References restaurees pour : {', '.join(self.angle_refs.keys())}"
    )

  # -----------------------------------------------------------------------------

  @property
  def is_calibrated(self) -> bool:
    """True si la calibration est complete et utilisable."""
    return self._calibrated
