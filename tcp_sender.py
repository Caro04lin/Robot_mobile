"""
Envoi TCP des commandes vers le robot.

Le sender tourne dans un thread separe, tente de se reconnecter si necessaire
et n'envoie une commande que lorsqu'elle change.
"""

import socket
import time
import logging
import threading
from typing import Optional

logger = logging.getLogger("TCPSender")

ROBOT_IP  = "192.168.4.1"  # Valeur par defaut surchargee par --robot-ip
ROBOT_PORT = 5000
SEND_RATE_HZ = 20


class TCPCommandSender:
  """
  Envoie les commandes IMU au robot via TCP.
  Meme protocole que keyboardInputPCLinux_humain.py.
  Tourne dans un thread daemon, reconnexion automatique.
  """

  def __init__(self, robot_ip: str = ROBOT_IP, robot_port: int = ROBOT_PORT):
    self.robot_ip  = robot_ip
    self.robot_port = robot_port
    self._interval = 1.0 / SEND_RATE_HZ

    self._current_cmd = "REPOS"
    self._last_sent  = None   # Envoi uniquement si changement
    self._stop_event  = threading.Event()
    self._thread: Optional[threading.Thread] = None
    self._sock: Optional[socket.socket] = None
    self._lock = threading.Lock()

    self._packets_sent = 0
    self._reconnects  = 0

    logger.info(f"TCPCommandSender -> {robot_ip}:{robot_port}")

  def set_command(self, command: str) -> None:
    """Met a jour la commande (thread-safe)."""
    with self._lock:
      self._current_cmd = command

  def start(self) -> None:
    self._stop_event.clear()
    self._thread = threading.Thread(
      target=self._loop, daemon=True, name="TCPSenderThread"
    )
    self._thread.start()
    logger.info("TCPSender demarre.")

  def stop(self) -> None:
    # Envoyer REPOS une derniere fois
    try:
      if self._sock:
        self._sock.sendall(b"REPOS\n")
    except Exception:
      pass
    self._stop_event.set()
    if self._thread:
      self._thread.join(timeout=2.0)
    self._close_socket()
    logger.info(f"TCPSender arrete. Paquets: {self._packets_sent}, reconnexions: {self._reconnects}")

  def _close_socket(self):
    if self._sock:
      try:
        self._sock.close()
      except Exception:
        pass
      self._sock = None

  def _connect(self) -> bool:
    """Tente une connexion TCP. Retourne True si reussi."""
    self._close_socket()
    try:
      s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      s.settimeout(3.0)
      s.connect((self.robot_ip, self.robot_port))
      s.settimeout(None)
      self._sock = s
      self._reconnects += 1
      logger.info(f"Connecte au robot {self.robot_ip}:{self.robot_port}")
      return True
    except Exception as e:
      logger.warning(f"Connexion echouee : {e} nouvel essai dans 2s")
      return False

  def _loop(self):
    """Thread : connexion + envoi en boucle."""
    while not self._stop_event.is_set():
      # Connexion
      if self._sock is None:
        if not self._connect():
          time.sleep(2.0)
          continue
        self._last_sent = None  # Forcer renvoi apres reconnexion

      # Envoi si changement de commande
      with self._lock:
        cmd = self._current_cmd

      if cmd != self._last_sent:
        try:
          self._sock.sendall((cmd + "\n").encode("utf-8"))
          self._last_sent = cmd
          self._packets_sent += 1
        except Exception as e:
          logger.warning(f"Erreur envoi : {e} reconnexion")
          self._close_socket()

      time.sleep(self._interval)

  @property
  def stats(self) -> dict:
    return {
      "packets_sent": self._packets_sent,
      "reconnects":  self._reconnects,
      "current_cmd": self._current_cmd,
    }
