import os
import cv2
import time
import queue
import torch
import threading
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
from datetime import datetime, timedelta

"""
image_processing.py
"""
def compute_depth_map_advanced(img_left, img_right):
    """
    Calcola una mappa di profondità avanzata dalle immagini stereo.
    - img_left: immagine della telecamera sinistra.
    - img_right: immagine della telecamera destra.
    Returns: mappa di profondità normalizzata.
    """
    # Converti le immagini in scala di grigi
    gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
    # Equalizzazione dell'istogramma per migliorare il contrasto
    gray_left = cv2.equalizeHist(gray_left)
    gray_right = cv2.equalizeHist(gray_right)
    # Configura il calcolo della disparità con StereoBM
    stereo = cv2.StereoBM_create(numDisparities=16*10, blockSize=21)
    stereo.setMinDisparity(0)
    stereo.setUniquenessRatio(10)
    stereo.setSpeckleWindowSize(100)
    stereo.setSpeckleRange(32)
    stereo.setDisp12MaxDiff(1)
    # Calcola la mappa di disparità
    disparity = stereo.compute(gray_left, gray_right)
    # Normalizza la mappa di disparità
    disparity = cv2.medianBlur(disparity, 5)
    disparity = cv2.normalize(disparity, None, alpha=0, beta=255,norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    return disparity

def enhanced_retinex(image, depth, sigma_list=[15, 80, 250]):
    """
    Applica il Retinex migliorato con depth-awareness.
    - image: immagine originale.
    - depth: mappa di profondità.
    - sigma_list: lista di valori sigma per il Retinex multi-scala.
    Returns: immagine migliorata.
    """
    image = image.astype(np.float32) + 1.0
    retinex = np.zeros_like(image)
    weights = [1/len(sigma_list)] * len(sigma_list)
    # Applica il Retinex multi-scala
    for sigma in sigma_list:
        blur = cv2.GaussianBlur(image, (0, 0), sigma)
        retinex += weights[sigma_list.index(sigma)] * (np.log10(image) - np.log10(blur))
    # Normalizza la mappa di profondità
    depth_norm = cv2.normalize(depth, None, 0, 1, cv2.NORM_MINMAX)
    # Combina Retinex e informazione di profondità
    retinex = retinex * (1 + depth_norm[:, :, np.newaxis])
    retinex = (retinex - np.min(retinex)) / (np.max(retinex) - np.min(retinex)) * 255
    retinex = np.clip(retinex, 0, 255).astype(np.uint8)
    return retinex

"""
inference.py
"""
def compute_frame_similarity(frame1, frame2):
    """
    Calcola la similarità tra due frame usando MSE (Mean Squared Error).
    Returns: True se i frame sono simili, False altrimenti
    """
    if frame1 is None or frame2 is None:
        return False
    # Ridimensiona i frame per velocizzare il confronto
    size = (160, 120)  # Risoluzione ridotta per il confronto
    f1 = cv2.resize(frame1, size)
    f2 = cv2.resize(frame2, size)
    # Calcola MSE
    mse = np.mean((f1 - f2) ** 2)
    # Soglia di similarità (da regolare in base alle tue necessità)
    return mse < 1000  # Soglia da calibrare

def check_cameras():
    """
    Verifica la disponibilità delle telecamere.
    Returns: (bool, str) - (disponibilità, messaggio)
    """
    try:
        cap1 = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)
        if not cap1.isOpened() and not cap2.isOpened():
            return False, "Nessuna telecamera trovata"
        elif not cap1.isOpened():
            return False, "Telecamera sinistra non trovata"
        elif not cap2.isOpened():
            return False, "Telecamera destra non trovata"
        cap1.release()
        cap2.release()
        return True, "Telecamere OK"
    except Exception as e:
        return False, f"Errore nell'accesso alle telecamere: {str(e)}"

def check_model(model_path, device):
    """
    Verifica l'esistenza e la validità del modello.
    Returns: (bool, str) - (disponibilità, messaggio)
    """
    if not os.path.exists(model_path):
        return False, f"Modello non trovato in: {model_path}"
    try:
        # Prova a caricare il modello
        torch.load(model_path, map_location=device)
        return True, "Modello OK"
    except Exception as e:
        return False, f"Errore nel caricamento del modello: {str(e)}"

"""
models.py
"""
class ConvNeXtBlock(nn.Module):
    """
    Blocco base dell'architettura ConvNeXt modificato per immagini grandi.
    """
    def __init__(self, dim, drop_path=0.):
        super().__init__()
        self.dwconv = nn.Conv2d(dim, dim, kernel_size=7, padding=3, groups=dim)
        self.norm = nn.BatchNorm2d(dim)
        self.pwconv1 = nn.Conv2d(dim, 4 * dim, 1)
        self.act = nn.GELU()
        self.pwconv2 = nn.Conv2d(4 * dim, dim, 1)
        self.drop_path = DropPath(drop_path) if drop_path > 0. else nn.Identity()

    def forward(self, x):
        input = x
        x = self.dwconv(x)
        x = self.norm(x)
        x = self.pwconv1(x)
        x = self.act(x)
        x = self.pwconv2(x)
        x = input + self.drop_path(x)
        return x

class EnhancedClassifier(nn.Module):
    """
    Classificatore ottimizzato per immagini 1600x1200.
    """
    def __init__(self, num_classes=2, input_channels=3):
        super().__init__()
        # Feature extractor
        self.features = nn.Sequential(
            # Stem - riduce la dimensione dell'immagine
            nn.Conv2d(input_channels, 64, kernel_size=7, stride=4, padding=3),
            nn.BatchNorm2d(64),
            nn.GELU(),
            nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
            # Stage 1
            self._make_stage(64, 128, 2),
            # Stage 2
            self._make_stage(128, 256, 2),
            # Stage 3
            self._make_stage(256, 512, 2),
            # Global pooling
            nn.AdaptiveAvgPool2d(1)
        )
        # Classificatore
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(512, 256),
            nn.BatchNorm1d(256),
            nn.GELU(),
            nn.Dropout(0.3),
            nn.Linear(256, num_classes)
        )

    def _make_stage(self, in_channels, out_channels, num_blocks):
        layers = [
            nn.Conv2d(in_channels, out_channels, kernel_size=2, stride=2),
            nn.BatchNorm2d(out_channels),
            nn.GELU()
        ]
        for _ in range(num_blocks):
            layers.append(ConvNeXtBlock(out_channels))
        return nn.Sequential(*layers)

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

class DropPath(nn.Module):
    """
    Drop Path per regolarizzazione.
    """
    def __init__(self, drop_prob=None):
        super(DropPath, self).__init__()
        self.drop_prob = drop_prob

    def forward(self, x):
        if self.drop_prob == 0. or not self.training:
            return x
        keep_prob = 1 - self.drop_prob
        shape = (x.shape[0],) + (1,) * (x.ndim - 1)
        random_tensor = keep_prob + torch.rand(shape, dtype=x.dtype, device=x.device)
        random_tensor.floor_()
        output = x.div(keep_prob) * random_tensor
        return output

"""
stereo_capture.py
"""
class StereoVideoProcessor:
    """
    Classe per la gestione dell'acquisizione video stereo in real-time.
    Permette di acquisire frame sincronizzati da due telecamere o video.
    """
    def __init__(self, left_source=0, right_source=1, buffer_size=10):
        """
        Inizializza il processore video stereo.
        - left_source, right_source: indici delle telecamere o percorsi video.
        - buffer_size: dimensione del buffer per i frame.
        """
        self.left_cap = cv2.VideoCapture(left_source)
        self.right_cap = cv2.VideoCapture(right_source)
        # Verifica apertura degli stream video
        if not self.left_cap.isOpened() or not self.right_cap.isOpened():
            raise ValueError("Errore nell'apertura degli stream video")
        # Imposta le stesse proprietà per entrambe le camere
        for cap in [self.left_cap, self.right_cap]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Larghezza del frame
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Altezza del frame
            cap.set(cv2.CAP_PROP_FPS, 30)  # Frame per secondo
        # Code thread-safe per i frame
        self.left_queue = queue.Queue(maxsize=buffer_size)
        self.right_queue = queue.Queue(maxsize=buffer_size)
        # Flag per il controllo dei thread
        self.is_running = False
        self.save_frames = False
        # Cartelle per il salvataggio dei frame
        self.save_dir = 'captured_frames'
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.save_dir, 'left'), exist_ok=True)
        os.makedirs(os.path.join(self.save_dir, 'right'), exist_ok=True)

    def start_capture(self):
        """
        Avvia i thread di cattura per entrambe le telecamere.
        """
        self.is_running = True
        self.left_thread = threading.Thread(target=self._capture_frames,args=(self.left_cap, self.left_queue, 'left'))
        self.right_thread = threading.Thread(target=self._capture_frames,args=(self.right_cap, self.right_queue, 'right'))
        self.left_thread.daemon = True
        self.right_thread.daemon = True
        self.left_thread.start()
        self.right_thread.start()

    def _capture_frames(self, cap, frame_queue, side):
        """
        Thread worker per la cattura dei frame.
        - cap: oggetto VideoCapture.
        - frame_queue: coda per i frame catturati.
        - side: 'left' o 'right' per identificare la telecamera.
        """
        while self.is_running:
            ret, frame = cap.read()
            if not ret:
                print("Errore nella lettura del frame dalla camera {side}")
                continue
            # Se la coda è piena, rimuovi il frame più vecchio
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            try:
                frame_queue.put(frame, timeout=0.1)
                # Salva i frame se richiesto
                if self.save_frames:
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    filename = f"{side}_{timestamp}.jpg"
                    save_path = os.path.join(self.save_dir, side, filename)
                    cv2.imwrite(save_path, frame)
            except queue.Full:
                continue

    def get_stereo_frames(self, timeout=1.0):
        """
        Ottiene una coppia di frame sincronizzati dalle telecamere.
        Returns: (left_frame, right_frame) o (None, None) se timeout.
        """
        try:
            left_frame = self.left_queue.get(timeout=timeout)
            right_frame = self.right_queue.get(timeout=timeout)
            return left_frame, right_frame
        except queue.Empty:
            return None, None

    def toggle_save_frames(self):
        """
        Attiva/disattiva il salvataggio dei frame.
        """
        self.save_frames = not self.save_frames
        print(f"Salvataggio frame {'attivato' if self.save_frames else 'disattivato'}")

    def stop(self):
        """
        Ferma l'acquisizione e rilascia le risorse.
        """
        self.is_running = False
        if hasattr(self, 'left_thread'):
            self.left_thread.join()
        if hasattr(self, 'right_thread'):
            self.right_thread.join()
        self.left_cap.release()
        self.right_cap.release()
