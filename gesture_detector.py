"""
Detection des gestes a partir des angles IMU.

Le module filtre les mesures, compare les angles a la posture neutre, puis
valide les commandes avec un vote temporel pour eviter les declenchements
parasites.
"""

import math
import time
import logging

from collections import deque
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Deque

logger = logging.getLogger("GestureDetector")

# -----------------------------------------------------------------------------
# Filtre complementaire
# -----------------------------------------------------------------------------
ALPHA = 0.95

# -----------------------------------------------------------------------------
# Seuils
# -----------------------------------------------------------------------------
THRESH_BACK_PITCH_FWD  = 12.0
THRESH_BACK_PITCH_BWD  = -12.0

THRESH_SHOULDER_FWD   =  8.0
THRESH_SHOULDER_BWD   = -8.0

THRESH_ROTATION     = 15.0

# Inclinaison laterale du dos pour tourner
THRESH_BACK_ROLL    = 12.0

THRESH_NEUTRAL_BACK   =  7.0
THRESH_NEUTRAL_SHOULDER =  5.0

# Anti-derive yaw
YAW_DECAY        = 0.992
YAW_GYRO_DEADZONE    = 1.0

# -----------------------------------------------------------------------------
# Vote robuste
# -----------------------------------------------------------------------------
VOTE_WINDOW  = 10
VOTE_THRESHOLD = 7

HOLD_TIMEOUT_S = 0.30
CONFIRM_TIME_S = 0.15


# -----------------------------------------------------------------------------
# Commandes
# -----------------------------------------------------------------------------
class RobotCommand(Enum):

  REPOS      = "REPOS"
  AVANCER     = "AVANCER"
  RECULER     = "RECULER"
  TOURNER_DROITE = "TOURNER_DROITE"
  TOURNER_GAUCHE = "TOURNER_GAUCHE"
  ARRET_URGENCE  = "ARRET_URGENCE"


@dataclass
class FilteredAngles:

  pitch: float = 0.0
  roll: float = 0.0


# -----------------------------------------------------------------------------
# Filtre complementaire
# -----------------------------------------------------------------------------
class ComplementaryFilter:

  def __init__(self,
         alpha: float = ALPHA,
         sensor_type: str = "shoulder"):

    self.alpha    = alpha
    self.sensor_type = sensor_type

    self.pitch = 0.0
    self.roll = 0.0

    # Rotation buste integree
    self.yaw  = 0.0

    self._last_time = None

  # -----------------------------------------------------------------------------

  def reset(self) -> None:

    self.pitch = 0.0
    self.roll = 0.0
    self.yaw  = 0.0

    self._last_time = None

  # -----------------------------------------------------------------------------

  def update(self,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float) -> FilteredAngles:

    now = time.monotonic()

    # -----------------------------------------------------------------------------
    # Premiere mesure
    # -----------------------------------------------------------------------------
    if self._last_time is None:

      self.pitch = self._accel_pitch(ax, ay, az)
      self.roll = self._accel_roll(ax, ay, az)

      self._last_time = now

      return FilteredAngles(
        pitch=self.pitch,
        roll=self.roll
      )

    # -----------------------------------------------------------------------------
    # dt
    # -----------------------------------------------------------------------------
    dt = now - self._last_time
    self._last_time = now

    if dt <= 0 or dt > 0.5:
      dt = 0.01

    # -----------------------------------------------------------------------------
    # Accelerometre
    # -----------------------------------------------------------------------------
    pitch_accel = self._accel_pitch(ax, ay, az)
    roll_accel = self._accel_roll(ax, ay, az)

    # -----------------------------------------------------------------------------
    # Gyroscope
    # -----------------------------------------------------------------------------
    gyro_pitch, gyro_roll = self._gyro_axes(gx, gy, gz)

    # -----------------------------------------------------------------------------
    # Filtre complementaire
    # -----------------------------------------------------------------------------
    self.pitch = (
      self.alpha * (self.pitch + gyro_pitch * dt)
      + (1.0 - self.alpha) * pitch_accel
    )

    self.roll = (
      self.alpha * (self.roll + gyro_roll * dt)
      + (1.0 - self.alpha) * roll_accel
    )

    # -----------------------------------------------------------------------------
    # Rotation buste yaw
    # -----------------------------------------------------------------------------
    if self.sensor_type == "back":

      # gz = rotation gauche/droite du torse.
      # On integre seulement si la vitesse depasse la zone morte gyro.
      if abs(gz) > YAW_GYRO_DEADZONE:
        self.yaw += gz * dt

      # decroissance douce
      self.yaw *= YAW_DECAY

    return FilteredAngles(
      pitch=self.pitch,
      roll=self.roll
    )

  # -----------------------------------------------------------------------------

  def _accel_pitch(self, ax, ay, az) -> float:

    if self.sensor_type == "shoulder":

      return math.degrees(
        math.atan2(
          -ax,
          math.sqrt(ay**2 + az**2)
        )
      )

    else:

      return math.degrees(
        math.atan2(
          -az,
          math.sqrt(ax**2 + ay**2)
        )
      )

  # -----------------------------------------------------------------------------

  def _accel_roll(self, ax, ay, az) -> float:

    if self.sensor_type == "shoulder":

      return math.degrees(
        math.atan2(ay, -az)
      )

    else:

      return math.degrees(
        math.atan2(ax, ay)
      )

  # -----------------------------------------------------------------------------

  def _gyro_axes(self, gx, gy, gz):

    if self.sensor_type == "shoulder":

      return gy, gx

    else:

      return gz, gx


# -----------------------------------------------------------------------------
# Detecteur principal
# -----------------------------------------------------------------------------
class GestureDetector:

  _OPPOSITES = {
    RobotCommand.AVANCER:    RobotCommand.RECULER,
    RobotCommand.RECULER:    RobotCommand.AVANCER,
    RobotCommand.TOURNER_DROITE: RobotCommand.TOURNER_GAUCHE,
    RobotCommand.TOURNER_GAUCHE: RobotCommand.TOURNER_DROITE,
  }

  # -----------------------------------------------------------------------------

  def __init__(self):

    self.filters = {

      "left_shoulder":
        ComplementaryFilter(sensor_type="shoulder"),

      "right_shoulder":
        ComplementaryFilter(sensor_type="shoulder"),

      "back":
        ComplementaryFilter(sensor_type="back"),
    }

    # Angles calibration
    self.ref_angles = {

      name: FilteredAngles(0.0, 0.0)

      for name in (
        "left_shoulder",
        "right_shoulder",
        "back"
      )
    }

    # Vote robuste
    self._vote_window: Deque = deque(
      maxlen=VOTE_WINDOW
    )

    self._confirmed_gesture: Optional[
      RobotCommand
    ] = None

    self._last_confirm_time = 0.0

    self._candidate_start = 0.0

    self._candidate_gesture: Optional[
      RobotCommand
    ] = None

    self._cancelled_by_opposite: Optional[
      RobotCommand
    ] = None

    self.current_command = RobotCommand.REPOS

    # Debug
    self.last_angles  = {}
    self.last_relative = {}

    logger.info("GestureDetector initialise.")

  # -----------------------------------------------------------------------------
  # Calibration
  # -----------------------------------------------------------------------------

  def set_reference(self,
           angles_at_rest: dict) -> None:

    for name, a in angles_at_rest.items():

      self.ref_angles[name] = FilteredAngles(
        pitch=a.pitch,
        roll=a.roll
      )

      logger.info(
        f"Ref calibration '{name}' : "
        f"pitch={a.pitch:+.2f} "
        f"roll={a.roll:+.2f}"
      )

  # -----------------------------------------------------------------------------

  def reset_filters(self) -> None:

    for f in self.filters.values():
      f.reset()

    self._vote_window.clear()

    self._confirmed_gesture = None
    self._last_confirm_time = 0.0

    self._candidate_start  = 0.0
    self._candidate_gesture = None

    self._cancelled_by_opposite = None

    self.current_command = RobotCommand.REPOS

    logger.info(
      "Filtres et etat reinitialises."
    )

  # -----------------------------------------------------------------------------
  # Update principal
  # -----------------------------------------------------------------------------

  def update(self,
        imu_data: dict) -> RobotCommand:

    # -----------------------------------------------------------------------------
    # 1. Filtrage
    # -----------------------------------------------------------------------------
    abs_angles = {}

    for name, filt in self.filters.items():

      d = imu_data[name]

      abs_angles[name] = filt.update(
        d["ax"],
        d["ay"],
        d["az"],
        d["gx"],
        d["gy"],
        d["gz"]
      )

    self.last_angles = abs_angles

    # -----------------------------------------------------------------------------
    # 2. Angles relatifs calibration
    # -----------------------------------------------------------------------------
    rel = {}

    for name, a in abs_angles.items():

      ref = self.ref_angles[name]

      rel[name] = FilteredAngles(
        pitch=a.pitch - ref.pitch,
        roll=a.roll - ref.roll
      )

    self.last_relative = rel

    # -----------------------------------------------------------------------------
    # 3. Classification
    # -----------------------------------------------------------------------------
    raw = self._classify(rel)

    # -----------------------------------------------------------------------------
    # 4. Decision robuste
    # -----------------------------------------------------------------------------
    self.current_command = self._decide(raw)

    return self.current_command

  # -----------------------------------------------------------------------------
  # Classification
  # -----------------------------------------------------------------------------

  def _classify(self,
         rel: dict) -> Optional[RobotCommand]:

    back_pitch = rel["back"].pitch

    left_pitch = rel["left_shoulder"].pitch

    right_pitch = rel["right_shoulder"].pitch

    # Difference epaules
    rotation_diff = left_pitch - right_pitch

    # Inclinaison laterale du dos
    back_roll = rel["back"].roll

    # -----------------------------------------------------------------------------
    # Rotation droite
    # -----------------------------------------------------------------------------
    if (
      rotation_diff > THRESH_ROTATION
      or back_roll > THRESH_BACK_ROLL
    ):
      return RobotCommand.TOURNER_DROITE

    # -----------------------------------------------------------------------------
    # Rotation gauche
    # -----------------------------------------------------------------------------
    if (
      rotation_diff < -THRESH_ROTATION
      or back_roll < -THRESH_BACK_ROLL
    ):
      return RobotCommand.TOURNER_GAUCHE

    # -----------------------------------------------------------------------------
    # Avancer
    # -----------------------------------------------------------------------------
    if (
      back_pitch > THRESH_BACK_PITCH_FWD
      and left_pitch > THRESH_SHOULDER_FWD
      and right_pitch > THRESH_SHOULDER_FWD
    ):
      return RobotCommand.AVANCER

    # Dos seul
    if back_pitch > THRESH_BACK_PITCH_FWD * 1.4:
      return RobotCommand.AVANCER

    # -----------------------------------------------------------------------------
    # Reculer
    # -----------------------------------------------------------------------------
    if (
      back_pitch < THRESH_BACK_PITCH_BWD
      and left_pitch < THRESH_SHOULDER_BWD
      and right_pitch < THRESH_SHOULDER_BWD
    ):
      return RobotCommand.RECULER

    # Dos seul
    if back_pitch < THRESH_BACK_PITCH_BWD * 1.4:
      return RobotCommand.RECULER

    # -----------------------------------------------------------------------------
    # Zone neutre
    # -----------------------------------------------------------------------------
    if (
      abs(back_pitch) < THRESH_NEUTRAL_BACK
      and abs(rotation_diff)
        < THRESH_NEUTRAL_SHOULDER * 2
      and abs(back_roll)
        < THRESH_BACK_ROLL * 0.5
    ):
      return None

    return None

  # -----------------------------------------------------------------------------
  # Vote robuste
  # -----------------------------------------------------------------------------

  def _decide(self,
        raw: Optional[RobotCommand]) -> RobotCommand:

    now = time.monotonic()

    self._vote_window.append(raw)

    if raw is None and self._cancelled_by_opposite is not None:
      self._cancelled_by_opposite = None
      self._vote_window.clear()

    # -----------------------------------------------------------------------------
    # Comptage votes
    # -----------------------------------------------------------------------------
    votes = {}

    for v in self._vote_window:

      if v is not None:
        votes[v] = votes.get(v, 0) + 1

    # -----------------------------------------------------------------------------
    # Gagnant
    # -----------------------------------------------------------------------------
    best_gesture = None
    best_count  = 0

    for gesture, count in votes.items():

      if count > best_count:

        best_count  = count
        best_gesture = gesture

    # -----------------------------------------------------------------------------
    # Validation
    # -----------------------------------------------------------------------------
    if (
      best_gesture is not None
      and best_count >= VOTE_THRESHOLD
    ):

      if best_gesture != self._candidate_gesture:

        self._candidate_gesture = best_gesture
        self._candidate_start  = now

      elapsed = now - self._candidate_start

      if elapsed >= CONFIRM_TIME_S:

        opposite = self._OPPOSITES.get(
          self.current_command
        )

        if (
          self.current_command == RobotCommand.REPOS
          and self._confirmed_gesture is not None
        ):
          command_to_cancel = self._confirmed_gesture
          opposite = self._OPPOSITES.get(
            command_to_cancel
          )
        else:
          command_to_cancel = self.current_command

        if best_gesture == self._cancelled_by_opposite:
          return RobotCommand.REPOS

        # Annulation par le mouvement inverse
        if best_gesture == opposite:

          logger.info(
            f"Annulation : "
            f"{command_to_cancel.value} REPOS"
          )

          self._confirmed_gesture = None
          self._last_confirm_time = now
          self._candidate_gesture = None
          self._cancelled_by_opposite = best_gesture

          return RobotCommand.REPOS

        self._confirmed_gesture = best_gesture

        self._last_confirm_time = now

        if best_gesture != self.current_command:

          logger.info(
            f"Commande : "
            f"{best_gesture.value}"
          )

        return best_gesture

    # -----------------------------------------------------------------------------
    # Maintien commande
    # -----------------------------------------------------------------------------
    else:

      self._candidate_gesture = None

      if (
        self._confirmed_gesture is not None
        and now - self._last_confirm_time
          < HOLD_TIMEOUT_S
      ):
        return self.current_command

    return RobotCommand.REPOS

  # -----------------------------------------------------------------------------
  # Debug
  # -----------------------------------------------------------------------------

  def get_debug_info(self) -> dict:

    rel = self.last_relative

    votes = {}

    for v in self._vote_window:

      if v is not None:
        votes[v] = votes.get(v, 0) + 1

    best_count = max(votes.values()) if votes else 0

    rotation_shoulders = (
      rel["left_shoulder"].pitch
      - rel["right_shoulder"].pitch
    )

    return {

      "current_command":
        self.current_command.value,

      "vote_score":
        f"{best_count}/{VOTE_WINDOW}",

      "angles_rel": {

        name: {

          "pitch": round(a.pitch, 1),

          "roll": round(a.roll, 1),

        }

        for name, a in rel.items()
      },

      # Debug rotation epaules
      "rotation_shoulders":
        round(rotation_shoulders, 1),

      # Debug inclinaison laterale dos
      "back_roll":
        round(
          rel["back"].roll,
          1
        ),
    }
