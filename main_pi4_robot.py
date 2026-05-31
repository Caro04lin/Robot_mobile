"""
Programme principal du Raspberry Pi cote robot.

Il recoit les commandes TCP du Pi cote humain, pilote les moteurs et le servo,
surveille l'ultrason et arrete le robot si le watchdog expire.
"""

import socket
import time
import logging
import sys

from motion import motor_controller
from motion import servo_controller
from sensors import ultrason_sensor

HOST        = '0.0.0.0'
PORT        = 5000
OBSTACLE_DIST_CM  = 30
CHECK_OBSTACLE   = True
WATCHDOG_TIMEOUT_S = 600.0

MOTOR_STOP     = 0
MOTOR_FORWARD_SLOW = 1
MOTOR_FORWARD_FAST = 2
MOTOR_BACKWARD   = -1

logging.basicConfig(
  level=logging.INFO,
  format="%(asctime)s [%(levelname)s] %(message)s",
  datefmt="%H:%M:%S",
  stream=sys.stdout,
)
logger = logging.getLogger("Robot")

# Toutes les commandes acceptees (IMU + clavier pour compatibilite)
COMMAND_MAP = {
  "AVANCER":    {"motor": 1, "servo": "center"},
  "RECULER":    {"motor": -1, "servo": "center"},
  "TOURNER_DROITE": {"motor": 1, "servo": "right"},
  "TOURNER_GAUCHE": {"motor": 1, "servo": "left"},
  "REPOS":     {"motor": 0, "servo": "center"},
  "ARRET_URGENCE": {"motor": 0, "servo": "center"},
  "UP":       {"motor": 1, "servo": "center"},
  "DOWN":      {"motor": -1, "servo": "center"},
  "RIGHT":     {"motor": 1, "servo": "right"},
  "LEFT":      {"motor": 1, "servo": "left"},
  "STOP":      {"motor": 0, "servo": "center"},
}

def apply_command(cmd, current_motor, current_servo, last_drive_command):
  mapping = COMMAND_MAP.get(cmd)
  if mapping is None:
    logger.warning(f"Commande inconnue : {repr(cmd)}")
    return current_motor, current_servo, last_drive_command

  if cmd == "AVANCER":
    new_motor = (
      MOTOR_FORWARD_FAST
      if (
        current_motor == MOTOR_STOP
        and last_drive_command == "AVANCER"
      )
      else MOTOR_FORWARD_SLOW
    )
    new_servo = "center"
    new_last_drive_command = "AVANCER"

  elif cmd == "UP":
    # Compatibilite clavier historique : UP incremente jusqu'a la vitesse 2.
    new_motor = min(current_motor + 1, MOTOR_FORWARD_FAST)
    new_servo = current_servo
    new_last_drive_command = "UP"

  elif cmd == "DOWN":
    # Compatibilite clavier historique : DOWN decremente jusqu'a la marche arriere.
    new_motor = max(current_motor - 1, MOTOR_BACKWARD)
    new_servo = current_servo
    new_last_drive_command = "DOWN"

  else:
    new_motor = mapping["motor"]
    new_servo = mapping["servo"]

    if cmd in ("REPOS", "STOP", "ARRET_URGENCE"):
      new_last_drive_command = last_drive_command
    else:
      new_last_drive_command = cmd

  if new_servo == "right":
    servo_controller.right()
  elif new_servo == "left":
    servo_controller.left()
  else:
    servo_controller.center()

  if new_motor != current_motor or new_servo != current_servo:
    motor_controller.set_state(new_motor)
    logger.info(f"{cmd} -> motor={new_motor} servo={new_servo}")

  return new_motor, new_servo, new_last_drive_command


def main():
  motor_controller.setup()
  servo_controller.setup(channel=0)
  ultrason_sensor.setup()
  logger.info("Materiel initialise.")

  server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  server.bind((HOST, PORT))
  server.listen(1)
  logger.info(f"En attente de connexion sur port {PORT}...")

  while True:
    try:
      conn, addr = server.accept()
      conn.setblocking(False)
      logger.info(f"Connecte : {addr}")
    except Exception as e:
      logger.error(f"Erreur accept : {e}")
      time.sleep(1)
      continue

    buffer    = ""
    motor_state  = 0
    servo_state  = "center"
    last_drive_command = None
    last_cmd_time = time.monotonic()
    emergency   = False

    motor_controller.set_state(0)
    servo_controller.center()

    try:
      while True:
        try:
          raw = conn.recv(1024)
          if not raw:
            logger.info("Client deconnecte.")
            break

          buffer += raw.decode('utf-8', errors='replace')
          lines  = buffer.split('\n')
          buffer = lines[-1]
          commands = [l.strip() for l in lines[:-1] if l.strip()]

          for cmd in commands:
            last_cmd_time = time.monotonic()
            emergency   = False

            if cmd == "ARRET_URGENCE":
              motor_controller.set_state(0)
              servo_controller.center()
              motor_state = 0
              servo_state = "center"
              last_drive_command = None
              emergency  = True
              logger.warning("ARRET URGENCE recu")
            else:
              motor_state, servo_state, last_drive_command = apply_command(
                cmd,
                motor_state,
                servo_state,
                last_drive_command,
              )

        except BlockingIOError:
          pass

        # Watchdog
        if time.monotonic() - last_cmd_time > WATCHDOG_TIMEOUT_S:
          if motor_state != 0:
            logger.warning("Watchdog -> arret")
            motor_state = 0
            servo_state = "center"
            last_drive_command = None
            motor_controller.set_state(0)
            servo_controller.center()

        # Ultrason local
        if CHECK_OBSTACLE and motor_state > 0 and not emergency:
          if ultrason_sensor.is_obstacle_detected(OBSTACLE_DIST_CM):
            logger.warning(f"Obstacle detecte -> arret")
            motor_state = 0
            servo_state = "center"
            last_drive_command = None
            motor_controller.set_state(0)
            emergency  = True

        time.sleep(0.05)  # 20 Hz

    except Exception as e:
      logger.error(f"Erreur : {e}")
    finally:
      conn.close()
      motor_controller.set_state(0)
      servo_controller.center()
      logger.info("En attente d'une nouvelle connexion...")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    motor_controller.set_state(0)
    servo_controller.center()
    logger.info("Arret.")
