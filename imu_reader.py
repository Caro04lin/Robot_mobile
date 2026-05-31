"""
Lecture des IMUs MPU-6050 via le multiplexeur TCA9548A.

Le multiplexeur permet d'utiliser plusieurs MPU-6050 ayant la meme adresse I2C.
Le gestionnaire initialise les capteurs disponibles, lit leurs donnees et fournit
des valeurs neutres lorsqu'un capteur optionnel est absent.
"""

import smbus2
import time
import logging

# -----------------------------------------------------------------------------
# Configuration du logger
# -----------------------------------------------------------------------------
logger = logging.getLogger("IMUReader")

# -----------------------------------------------------------------------------
# Adresses I2C
# -----------------------------------------------------------------------------
TCA9548A_ADDRESS = 0x70  # Adresse par defaut du multiplexeur (AD0=AD1=AD2=GND)
MPU6050_ADDRESS = 0x68  # Adresse par defaut du MPU-6050

# -----------------------------------------------------------------------------
# Registres du MPU-6050 (cf. datasheet RM-MPU-6000A Rev 4.2)
# -----------------------------------------------------------------------------
MPU6050_REG_PWR_MGMT_1  = 0x6B # Gestion alimentation (reveil du capteur)
MPU6050_REG_SMPLRT_DIV  = 0x19 # Diviseur de frequence d'echantillonnage
MPU6050_REG_CONFIG    = 0x1A # Configuration filtre passe-bas numerique (DLPF)
MPU6050_REG_GYRO_CONFIG = 0x1B # Plage gyroscope (250 / 500 / 1000 / 2000 deg/s)
MPU6050_REG_ACCEL_CONFIG = 0x1C # Plage accelerometre (2 / 4 / 8 / 16 g)
MPU6050_REG_ACCEL_XOUT_H = 0x3B # Premier registre de donnees (14 octets total)
MPU6050_REG_WHO_AM_I   = 0x75 # Identifiant materiel (doit retourner 0x68)

# -----------------------------------------------------------------------------
# Facteurs de conversion (selon plage configuree dans initialize())
# -----------------------------------------------------------------------------
# Plage accelerometre 2g  sensibilite = 16384 LSB/g (2g couvre bien les gestes humains)
# Plage gyroscope 250 deg/s : sensibilite = 131.0 LSB/(deg/s)
ACCEL_SCALE_FACTOR = 16384.0  # LSB/g  convertit valeur brute g
GYRO_SCALE_FACTOR = 131.0   # LSB/(deg/s), conversion en deg/s

# -----------------------------------------------------------------------------
# Canaux TCA9548A position physique de chaque IMU
# -----------------------------------------------------------------------------
IMU_CHANNELS = {
  "left_shoulder": 0,  # Epaule gauche
  "right_shoulder": 1,  # Epaule droite
  "back":      2,  # Dos (omoplate droite position recommandee)
}


# -----------------------------------------------------------------------------
# Pilote du multiplexeur TCA9548A
# -----------------------------------------------------------------------------
class TCA9548A:
  """
  Pilote du multiplexeur I2C TCA9548A (8 canaux).

  Role :
    Permet de selectionner quel canal I2C est actif, afin de communiquer
    avec l'un des 3 MPU-6050 qui partagent tous la meme adresse (0x68).
    Sans ce multiplexeur, les 3 capteurs rentreraient en conflit sur le bus.

  Protocole :
    Ecriture d'un seul octet sur l'adresse 0x70.
    Le bit N de cet octet active le canal N.
    Exemple : 0b00000100 = canal 2 actif.
  """

  def __init__(self, bus: smbus2.SMBus, address: int = TCA9548A_ADDRESS):
    self.bus   = bus
    self.address = address

  def select_channel(self, channel: int) -> None:
    """
    Active un et un seul canal du multiplexeur.

    Args:
      channel : numero de canal a activer (0 a 7)

    Raises:
      ValueError : si le numero de canal est hors plage
    """
    if not 0 <= channel <= 7:
      raise ValueError(f"Canal TCA9548A invalide : {channel}. Doit etre entre 0 et 7.")
    # 1 << channel : decalage binaire active uniquement le bit correspondant au canal
    # Exemple : channel=2 1<<2 = 0b00000100
    self.bus.write_byte(self.address, 1 << channel)

  def disable_all_channels(self) -> None:
    """
    Desactive tous les canaux en ecrivant 0x00.

    Bonne pratique : desactiver tous les canaux entre deux sequences
    de lecture pour eviter tout conflit accidentel sur le bus I2C.
    """
    self.bus.write_byte(self.address, 0x00)


# -----------------------------------------------------------------------------
# Pilote du capteur IMU MPU-6050
# -----------------------------------------------------------------------------
class MPU6050:
  """
  Pilote complet du capteur inertiel MPU-6050.

  Fournit (apres conversion) :
    - Acceleration lineaire (ax, ay, az) en g
    - Vitesse angulaire  (gx, gy, gz) en /s
    - Temperature interne        en C (informatif)

  Lecture en burst (14 octets en une seule transaction I2C) :
    Plus efficace que 6 lectures separees car le bus n'est verrouille
    qu'une seule fois, et toutes les valeurs sont lues au meme instant.

  Axes du MPU-6050 (orientation montage standard) :
    X gauche/droite (roll)
    Y avant/arriere (pitch)
    Z haut/bas   (yaw / acceleration gravitationnelle)
  """

  def __init__(self, bus: smbus2.SMBus, address: int = MPU6050_ADDRESS):
    self.bus   = bus
    self.address = address

  def initialize(self) -> None:
    """
    Initialise et configure le MPU-6050.

    Sequence :
      1. Verification WHO_AM_I (detection du capteur)
      2. Sortie du mode sleep (le capteur demarre en veille par defaut)
      3. Filtre passe-bas DLPF niveau 3 (44 Hz) elimine les vibrations
      4. Frequence d'echantillonnage 100 Hz (via SMPLRT_DIV)
      5. Plage gyroscope 250/s (resolution maximale)
      6. Plage accelerometre 2g (resolution maximale)

    Raises:
      RuntimeError : si le capteur n'est pas detecte sur le bus I2C
    """
    # Verification d'identite 
    # Le registre WHO_AM_I retourne 0x68 sur un MPU-6050 fonctionnel.
    # Si la valeur est differente cablage incorrect ou capteur defectueux.
    who_am_i = self.bus.read_byte_data(self.address, MPU6050_REG_WHO_AM_I)
    if who_am_i != 0x68:
      raise RuntimeError(
        f"MPU-6050 non detecte a l'adresse 0x{self.address:02X}. "
        f"WHO_AM_I retourne : {hex(who_am_i)} (attendu : 0x68). "
        f"Verifiez le cablage SDA/SCL et le canal TCA9548A."
      )

    # Reveil du capteur 
    # Par defaut au power-on, le bit SLEEP (bit 6 de PWR_MGMT_1) est a 1.
    # Ecrire 0x00 efface ce bit et demarre le capteur.
    # Le bit CLKSEL=0 selectionne l'oscillateur interne 8 MHz (suffisant a 100 Hz).
    self.bus.write_byte_data(self.address, MPU6050_REG_PWR_MGMT_1, 0x00)
    time.sleep(0.1) # Attendre la stabilisation de l'alimentation interne

    # Filtre passe-bas numerique (DLPF) 
    # DLPF_CFG = 3 bande passante accelerometre 44 Hz / gyroscope 42 Hz
    # Cela elimine les vibrations mecaniques (>44 Hz) sans degrader les gestes
    # humains (typiquement <5 Hz). Le delai de groupe introduit est ~4.9 ms.
    self.bus.write_byte_data(self.address, MPU6050_REG_CONFIG, 0x03)

    # Frequence d'echantillonnage 
    # Fs = Fgyro / (1 + SMPLRT_DIV) = 1000 Hz / (1 + 9) = 100 Hz
    # Avec DLPF actif, Fgyro = 1000 Hz (cf. datasheet Table 9).
    self.bus.write_byte_data(self.address, MPU6050_REG_SMPLRT_DIV, 0x09)

    # Plage gyroscope 250/s 
    # FS_SEL = 0b00 (bits [4:3] = 00) 250/s, sensibilite = 131 LSB//s
    # Suffisant pour les mouvements du tronc (rarement > 200/s).
    self.bus.write_byte_data(self.address, MPU6050_REG_GYRO_CONFIG, 0x00)

    # Plage accelerometre 2g 
    # AFS_SEL = 0b00 (bits [4:3] = 00) 2g, sensibilite = 16384 LSB/g
    # 2g couvre largement les inclinaisons du corps humain (1g statique).
    self.bus.write_byte_data(self.address, MPU6050_REG_ACCEL_CONFIG, 0x00)

  def _read_raw_word_signed(self, high_byte: int, low_byte: int) -> int:
    """
    Reconstruit un entier signe 16 bits depuis deux octets (notation complement a 2).

    Le MPU-6050 stocke chaque mesure sur 2 octets (MSB en premier) :
      octet fort [15:8] suivi de l'octet faible [7:0]

    Pour les valeurs negatives (complement a 2) :
      Si la valeur reconstruite 0x8000 (32768) soustraire 0x10000 (65536)
      Cela donne la vraie valeur signee [-32768, 32767].

    Args:
      high_byte : octet de poids fort (MSB)
      low_byte : octet de poids faible (LSB)

    Returns:
      Entier signe 16 bits dans l'intervalle [-32768, 32767]
    """
    value = (high_byte << 8) | low_byte # Recomposition 16 bits non signe
    if value >= 0x8000:          # Bit de signe active negatif
      value -= 0x10000
    return value

  def read_data(self) -> dict:
    """
    Lit en une seule transaction I2C les 14 octets de mesures brutes.

    Structure de la plage memoire depuis 0x3B :
      Octets 0-1 : ACCEL_XOUT (acceleration X)
      Octets 2-3 : ACCEL_YOUT (acceleration Y)
      Octets 4-5 : ACCEL_ZOUT (acceleration Z)
      Octets 6-7 : TEMP_OUT  (temperature non utilisee)
      Octets 8-9 : GYRO_XOUT  (vitesse angulaire X)
      Octets 10-11 : GYRO_YOUT  (vitesse angulaire Y)
      Octets 12-13 : GYRO_ZOUT  (vitesse angulaire Z)

    Returns:
      Dictionnaire avec les cles :
        ax, ay, az  accelerations en g
        gx, gy, gz  vitesses angulaires en /s
        temp     temperature en C
    """
    # Lecture burst : 14 octets consecutifs depuis le registre 0x3B
    raw = self.bus.read_i2c_block_data(
      self.address, MPU6050_REG_ACCEL_XOUT_H, 14
    )

    # Reconstruction des valeurs brutes signees 
    ax_raw  = self._read_raw_word_signed(raw[0], raw[1])
    ay_raw  = self._read_raw_word_signed(raw[2], raw[3])
    az_raw  = self._read_raw_word_signed(raw[4], raw[5])
    temp_raw = self._read_raw_word_signed(raw[6], raw[7])
    gx_raw  = self._read_raw_word_signed(raw[8], raw[9])
    gy_raw  = self._read_raw_word_signed(raw[10], raw[11])
    gz_raw  = self._read_raw_word_signed(raw[12], raw[13])

    return {
      # Division par le facteur d'echelle conversion en unites physiques
      "ax":  ax_raw  / ACCEL_SCALE_FACTOR,  # g
      "ay":  ay_raw  / ACCEL_SCALE_FACTOR,  # g
      "az":  az_raw  / ACCEL_SCALE_FACTOR,  # g
      "gx":  gx_raw  / GYRO_SCALE_FACTOR,  # deg/s
      "gy":  gy_raw  / GYRO_SCALE_FACTOR,  # deg/s
      "gz":  gz_raw  / GYRO_SCALE_FACTOR,  # deg/s
      "temp": temp_raw / 340.0 + 36.53     # C (formule datasheet 4.18)
    }


# -----------------------------------------------------------------------------
# Gestionnaire principal des 3 IMUs
# -----------------------------------------------------------------------------
class IMUManager:
  """
  Gestionnaire des 3 capteurs MPU-6050 via le TCA9548A.

  Responsabilites :
    1. Ouvrir le bus I2C (/dev/i2c-1 sur Raspberry Pi 4)
    2. Initialiser sequentiellement les 3 MPU-6050
    3. Lire les 3 capteurs a chaque appel de read_all()
    4. Fermer le bus proprement a la fin

  Usage typique dans la boucle principale :
    manager = IMUManager()
    manager.initialize()
    while True:
      data = manager.read_all()
      # data["left_shoulder"]["ax"] acceleration X epaule gauche
      # data["back"]["gy"]      vitesse angulaire Y du dos
  """

  def __init__(self, i2c_bus_number: int = 1):
    """
    Args:
      i2c_bus_number : numero du bus I2C Linux.
               Sur Raspberry Pi 4 : 1 (correspond a /dev/i2c-1).
               Connecteurs GPIO2 (SDA) et GPIO3 (SCL).
    """
    self.bus     = smbus2.SMBus(i2c_bus_number)
    self.tca     = TCA9548A(self.bus)
    self.mpu     = MPU6050(self.bus)
    self._initialized = False
    logger.info(f"IMUManager cree bus I2C-{i2c_bus_number} (/dev/i2c-{i2c_bus_number})")

  def initialize(self) -> None:
    """
    Initialise les 3 capteurs MPU-6050 via le multiplexeur.

    Pour chaque IMU, tente la connexion individuellement et affiche :
      "Connected to IMU gauche/droit/dos"   si detecte
      "Not connected to IMU gauche/droit/dos" si absent ou erreur

    Le programme continue meme si un IMU est manquant les lectures
    pour cet IMU retourneront des zeros (gere dans read_all).

    Raises:
      RuntimeError : si AUCUN IMU n'est detecte (impossible de continuer)
    """
    # Labels lisibles pour les messages utilisateur
    imu_labels = {
      "left_shoulder": "IMU gauche",
      "right_shoulder": "IMU droit",
      "back":      "IMU dos",
    }

    self._available = {}  # {imu_name: bool} IMUs effectivement connectes

    connected_count = 0
    for name, channel in IMU_CHANNELS.items():
      label = imu_labels[name]
      try:
        self.tca.select_channel(channel)
        time.sleep(0.01)
        self.mpu.initialize()
        self._available[name] = True
        connected_count += 1
        print(f"Connected to {label}")
        logger.info(f"Connected to {label} (canal {channel})")
      except Exception as e:
        self._available[name] = False
        print(f"Not connected to {label}")
        logger.warning(f"Not connected to {label} (canal {channel}) : {e}")

    self.tca.disable_all_channels()

    if connected_count == 0:
      raise RuntimeError(
        "Aucun IMU detecte. Verifiez le cablage I2C et l'alimentation."
      )

    self._initialized = True

  def read_all(self) -> dict:
    """
    Lit les donnees des IMUs disponibles.
    Pour un IMU non connecte, retourne des zeros (az = -1g pour les epaules,
    ay = +1g pour le dos valeurs de repos neutres).

    Returns:
      dict {imu_name: {ax, ay, az, gx, gy, gz, temp}}
    """
    if not self._initialized:
      raise RuntimeError(
        "IMUManager non initialise. Appelez initialize() avant read_all()."
      )

    # Valeurs de repos par defaut selon l'orientation de chaque capteur
    _defaults = {
      "left_shoulder": {"ax": 0.0, "ay": 0.0, "az": -1.0, "gx": 0.0, "gy": 0.0, "gz": 0.0, "temp": 0.0},
      "right_shoulder": {"ax": 0.0, "ay": 0.0, "az": -1.0, "gx": 0.0, "gy": 0.0, "gz": 0.0, "temp": 0.0},
      "back":      {"ax": 0.0, "ay": 1.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0, "temp": 0.0},
    }

    readings = {}
    for name, channel in IMU_CHANNELS.items():
      if not self._available.get(name, False):
        readings[name] = _defaults[name]
        continue
      try:
        self.tca.select_channel(channel)
        readings[name] = self.mpu.read_data()
      except Exception as e:
        logger.warning(f"Erreur lecture {name} : {e} valeurs par defaut")
        readings[name] = _defaults[name]

    self.tca.disable_all_channels()
    return readings

  def close(self) -> None:
    """
    Libere proprement le bus I2C.
    A appeler dans le bloc finally du script principal.
    """
    try:
      self.tca.disable_all_channels()
      self.bus.close()
      logger.info("Bus I2C ferme proprement.")
    except Exception as e:
      logger.warning(f"Erreur lors de la fermeture du bus I2C : {e}")


# -----------------------------------------------------------------------------
# Test standalone Raspberry Pi 4 : python imu_reader.py
# -----------------------------------------------------------------------------
if __name__ == "__main__":
  import sys
  logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s %(message)s",
    datefmt="%H:%M:%S"
  )

  print("=== Test de lecture IMU Raspberry Pi 4 ===")
  print("Ctrl+C pour arreter\n")

  manager = IMUManager(i2c_bus_number=1)
  try:
    manager.initialize()
    print(f"{'IMU':<20s} {'ax':>8s} {'ay':>8s} {'az':>8s} {'gx':>8s} {'gy':>8s} {'gz':>8s}")
    print("-" * 75)
    while True:
      data = manager.read_all()
      # Effacer les 3 lignes precedentes pour affichage en place
      for imu_name, v in data.items():
        print(
          f"{imu_name:<20s} "
          f"{v['ax']:+7.3f}g {v['ay']:+7.3f}g {v['az']:+7.3f}g "
          f"{v['gx']:+7.1f} {v['gy']:+7.1f} {v['gz']:+7.1f}/s"
        )
      print(f"\033[{len(IMU_CHANNELS)}A", end="") # Remonter le curseur (affichage live)
      time.sleep(0.05) # 20 Hz pour le test standalone

  except KeyboardInterrupt:
    print(f"\n{'':75s}")  # Effacer la derniere ligne partiellement ecrasee
    print("Arret.")
  finally:
    manager.close()
