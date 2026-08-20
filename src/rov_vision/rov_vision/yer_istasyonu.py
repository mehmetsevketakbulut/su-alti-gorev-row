#!/usr/bin/env python3
"""
=============================================================================
ALTAY ROV - YER ISTASYONU  (Windows/Linux PC)
=============================================================================
Bu dosya PC'de calisir. Jetson'a UDP 5005'ten komut yollar, 5006'dan
telemetri dinler. Videoyu http://JETSON_IP:5000/video adresinden ceker.

KURULUM:  pip install pygame opencv-python
CALISTIR: python yer_istasyonu.py

KUMANDA
-------
  Sol analog          ileri/geri + yengec (strafe)
  Sag analog          dalis/cikis + donus (yaw)
  START               ARM / DISARM  (motor kilidi)
  BACK                Acil DISARM
  LB                  Ana ROV manuel  (otonom sirasinda ANINDA devralma)
  RB                  Otonom cizgi takip
  Y                   Mini ROV'u birak + Mini'ye gec
                        Ilk basista miknatis atesler, sonrakilerde sadece
                        moda gecer. Boylece tekrar Mini'ye donerken
                        miknatis ikinci kez tetiklenmez.
  X                   Kamera rolesi degistir (Ana <-> Mini)
  A                   Fotograf
  B                   Video kayit baslat/durdur
  D-Pad yukari/asagi  Kamera tilt
  D-Pad sag/sol       Aydinlatma
  ESC                 Cikis (cikarken motorlari kilitler)
=============================================================================
"""

import ctypes
import json
import platform
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import pygame
import serial

# ------------------------------------------------------------------ AYARLAR
TOP_CAN_PORT = "COM15" # Bilgisayara bagli olan Deneyap CAN kartinin portu. Aygit Yoneticisinden bakin.

if len(sys.argv) > 1:
    JETSON_IP = sys.argv[1]
else:
    JETSON_IP = input("Jetson'in IP Adresini Girin (ornegin 192.168.1.5): ").strip()
    if not JETSON_IP:
        JETSON_IP = "192.168.1.10"

CMD_PORT = 5005
TELEM_PORT = 5006
print(f"Baglanilacak Jetson IP: {JETSON_IP}")

STREAM_URL = f"http://{JETSON_IP}:5000/video"
GONDERIM_HZ = 50

try:
    ctypes.windll.user32.SetProcessDPIAware()
except Exception:
    pass

# Eksen haritasi platforma gore DEGISIR. SDL2/Linux'ta 2 ve 5 tetiklerdir;
# Windows/XInput'ta 3 ve 4 sag analogdur. Yanlis harita = ters/kacik hareket.
if platform.system() == "Windows":
    AX_LX, AX_LY, AX_RX, AX_RY = 0, 1, 3, 4
else:
    AX_LX, AX_LY, AX_RX, AX_RY = 0, 1, 2, 3

BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB, BTN_BACK, BTN_START = 4, 5, 6, 7


class Kamera:
    def __init__(self, url):
        self.url, self.cap, self.frame = url, None, None
        self.lock = threading.Lock()
        self.running = True
        self.donuk = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def _ac(self):
        self.cap = cv2.VideoCapture(self.url)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

    def _loop(self):
        self._ac()
        onceki = None
        while self.running:
            if not self.cap or not self.cap.isOpened():
                time.sleep(1)
                self._ac()
                continue
            try:
                ok, f = self.cap.read()
                if not ok:
                    self.donuk += 1
                    time.sleep(0.1)
                    continue
            except Exception:
                self.donuk += 1
                time.sleep(0.1)
                continue
            
            if self.donuk > 60:
                self.donuk = 0
                self._ac()
            
            # Role Ana<->Mini gecisi yapinca yakalama karti sinyal senkronunu
            # kaybeder ve HATA DONDURMEZ - son kareyi tekrar tekrar verir.
            # Pilot ekranda eski goruntuyu canli sanmasin diye tespit ediyoruz.
            imza = int(f[::40, ::40].sum())
            self.donuk = self.donuk + 1 if imza == onceki else 0
            onceki = imza
            with self.lock:
                self.frame = f

    def read(self):
        with self.lock:
            if self.frame is None:
                return None, False
            return self.frame.copy(), True

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()


class Telemetri:
    def __init__(self, port):
        self.running = True
        self.lock = threading.Lock()
        self.son_addr = "Bilinmiyor"
        self.d = {"motorlar": [1500] * 6, "servo": 45, "failsafe": 1,
                  "magnet": 0, "can_err": 0, "uart_ok": 0,
                  "derinlik_cm": 0.0, "mesafe_cm": 0.0,
                  "roll": 0.0, "pitch": 0.0}
        self.son = 0.0
        self.log = deque(["Jetson bekleniyor..."], maxlen=5)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.3)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                self.son_addr = addr[0]
                t = json.loads(data.decode())
                with self.lock:
                    self.d.update(t)
                    self.son = time.time()
                    if t.get("failsafe"):
                        self.log.append("ERR: ANA ROV FAILSAFE")
                    elif t.get("can_err", 0) > 0:
                        self.log.append(f"UYARI: CAN hata={t['can_err']}")
                    else:
                        self.log.append("OK: link saglam")
            except socket.timeout:
                continue
            except Exception:
                continue

    def snap(self):
        with self.lock:
            return dict(self.d), list(self.log), self.son

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    pygame.init()
    pygame.joystick.init()
    joy = None
    if pygame.joystick.get_count() > 0:
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"[JOYSTICK] {joy.get_name()}")
    else:
        print("[JOYSTICK] Kumanda bulunamadi!")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    info = pygame.display.Info()
    W, H = info.current_w, info.current_h
    screen = pygame.display.set_mode((W, H), pygame.FULLSCREEN)
    pygame.display.set_caption("ALTAY ROV Kontrol Istasyonu")

    f_ttl = pygame.font.SysFont("Consolas", 30, bold=True)
    f_hd = pygame.font.SysFont("Consolas", 21, bold=True)
    f_md = pygame.font.SysFont("Consolas", 18)
    f_sm = pygame.font.SysFont("Consolas", 14)

    C_BG, C_PANEL = (10, 15, 25), (20, 25, 35)
    C_CYAN, C_GREEN, C_RED = (0, 255, 255), (57, 255, 20), (255, 50, 50)
    C_TEXT, C_YELLOW = (220, 220, 220), (255, 215, 0)

    cam = Kamera(STREAM_URL)
    tel = Telemetri(TELEM_PORT)
    clock = pygame.time.Clock()

    mod = 0
    kamera_role = 0
    tilt = 45
    magnet_evt = 0
    kill_switch = 0
    mini_serbest = False
    armed = False
    isik_ana = isik_mini = 0
    seq = 0
    kayit, out = False, None
    running = True

    try:
        top_can = serial.Serial(TOP_CAN_PORT, 115200, timeout=0)
        print(f"[CAN] {TOP_CAN_PORT} baglandi.")
    except Exception as e:
        top_can = None
        print(f"[CAN HATA] {TOP_CAN_PORT} acilamadi: {e}")

    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and
                                          ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.JOYDEVICEREMOVED:
                # PS4 controller causes phantom disconnects, do NOT disarm automatically
                joy = None
                print("[JOYSTICK] Cikarildi (Phantom event ignored)")
            elif ev.type == pygame.JOYDEVICEADDED:
                if joy is None and pygame.joystick.get_count() > 0:
                    joy = pygame.joystick.Joystick(0)
                    joy.init()
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE:
                    armed = not armed
                    print("[KLAVYE] SPACE basildi -> ARMED:", armed)
                elif ev.key == pygame.K_y:
                    magnet_evt = 1 if magnet_evt == 0 else 0
                    print(f"[KLAVYE] Y basildi -> MIKNATIS RÖLESİ: {magnet_evt}")
                elif ev.key == pygame.K_k:
                    kill_switch = 1 if kill_switch == 0 else 0
                    print(f"[KLAVYE] K basildi -> GÜCÜ KES (KILL SWITCH): {kill_switch}")
                elif ev.key == pygame.K_m:
                    # Mini ROV Modu + Kamera
                    mod, kamera_role = 2, 1
                    print("[KLAVYE] M basildi -> MINI ROV MODUNA GECILDI")
                elif ev.key == pygame.K_1:
                    mod, kamera_role = 0, 0
                elif ev.key == pygame.K_2:
                    mod = 1
                elif ev.key == pygame.K_l:
                    # Mini ROV Işığını aç/kapat (CAN Bus Testi için)
                    isik_mini = 100 if isik_mini == 0 else 0
                    print("[KLAVYE] L basildi -> MINI ISIK:", isik_mini)

            elif ev.type == pygame.JOYBUTTONDOWN:
                b = ev.button
                if b == BTN_START:
                    armed = not armed
                elif b == BTN_BACK:
                    armed = False
                elif b == BTN_LB:
                    mod, kamera_role = 0, 0
                elif b == BTN_RB:
                    mod = 1
                elif b == BTN_Y:
                    magnet_evt = 1 if magnet_evt == 0 else 0
                    mod, kamera_role = 2, 1
                elif b == BTN_X:
                    kill_switch = 1 if kill_switch == 0 else 0
                    print(f"[JOYSTICK] X (Kare) basildi -> KILL SWITCH: {kill_switch}")
                elif b == BTN_A:
                    f, ok = cam.read()
                    if ok:
                        cv2.imwrite(datetime.now().strftime("FOTO_%H%M%S.jpg"), f)
                elif b == BTN_B:
                    if not kayit:
                        f, ok = cam.read()
                        if ok:
                            hh, ww = f.shape[:2]      # gercek cozunurluk
                            out = cv2.VideoWriter(
                                datetime.now().strftime("VID_%H%M%S.avi"),
                                cv2.VideoWriter_fourcc(*"XVID"), 20.0, (ww, hh))
                            kayit = True
                    else:
                        kayit = False
                        if out:
                            out.release()
                            out = None

        fwd = strafe = dive = yaw = 0
        if joy is not None:
            try:
                raw_lx = joy.get_axis(0)
                raw_ly = joy.get_axis(1)
                raw_rx = joy.get_axis(3)
                raw_ry = joy.get_axis(4)
                
                fwd = int(-raw_ly * 75)
                strafe = int(raw_lx * 75)
                dive = int(-raw_ry * 75)
                yaw = int(raw_rx * 75)
                
                # Ekrana basarak PS4 trigger (L2/R2) sorunu var mi gormek icin
                if seq % 20 == 0:
                    print(f"JOY_DEBUG -> L_Y(Fwd):{fwd}  R_Y(Dive):{dive}  L_X(Strafe):{strafe}  R_X(Yaw):{yaw}")

                if abs(fwd) < 15: fwd = 0
                if abs(strafe) < 15: strafe = 0
                if abs(dive) < 15: dive = 0
                if abs(yaw) < 15: yaw = 0
                try:
                    if joy.get_numhats() > 0:
                        hat = joy.get_hat(0)
                        if hat[1] == 1: tilt = max(0, tilt - 1)
                        elif hat[1] == -1: tilt = min(90, tilt + 1)
                        if hat[0] == 1: isik_ana = min(100, isik_ana + 2)
                        elif hat[0] == -1: isik_ana = max(0, isik_ana - 2)
                        isik_mini = isik_ana
                except pygame.error:
                    pass
            except pygame.error:
                pass  # Sadece sessizce atla, terminali bogmasin
                
        # --- KLAVYE SURUS DESTEGI ---
        keys = pygame.key.get_pressed()
        # Brownout (kapanma) olmamasi icin test asamasinda gucu %20'ye cektik.
        if keys[pygame.K_w]: fwd = 20
        elif keys[pygame.K_s]: fwd = -20
        if keys[pygame.K_a]: strafe = -20
        elif keys[pygame.K_d]: strafe = 20
        if keys[pygame.K_LSHIFT]: dive = 20
        elif keys[pygame.K_LCTRL]: dive = -20
        if keys[pygame.K_q]: yaw = -20
        elif keys[pygame.K_e]: yaw = 20

        if not armed:
            fwd = strafe = dive = yaw = 0
            
        # YENI MIMARI: Mini ROV Yonlendirmesi (Topside CAN)
        fwd_jetson, strafe_jetson, dive_jetson, yaw_jetson = fwd, strafe, dive, yaw
        fwd_mini, strafe_mini, dive_mini, yaw_mini = 0, 0, 0, 0
        fs_mini = 0 if armed else 1
        
        if mod == 2:
            # Mini ROV Modu: İtkiler Jetson'a 0 gider, PC'deki CAN modülüne gider
            fwd_jetson = strafe_jetson = dive_jetson = yaw_jetson = 0
            fwd_mini, strafe_mini, dive_mini, yaw_mini = fwd, strafe, dive, yaw
            
        if 'top_can' in globals() and top_can is not None:
            try:
                # Format: M,fwd,strafe,dive,yaw,fs_mini
                msg = f"M,{fwd_mini},{strafe_mini},{dive_mini},{yaw_mini},{fs_mini}\n"
                top_can.write(msg.encode('utf-8'))
                
                # Topside CAN'den gelen debug mesajlarini oku ve ekrana bas
                while top_can.in_waiting:
                    can_line = top_can.readline().decode('utf-8', 'ignore').strip()
                    if can_line:
                        print(can_line)
            except Exception as e:
                print(f"[CAN HATA] Islem basarisiz: {e}")

        seq += 1
        paket = {"seq": seq, "t": time.time(), "mod": mod,
                 "armed": 1 if armed else 0,
                 "fwd": fwd_jetson, "strafe": strafe_jetson, "dive": dive_jetson, "yaw": yaw_jetson,
                 "kamera_role": kamera_role, "kamera_tilt": tilt,
                 "magnet_evt": magnet_evt,       # HER PAKETTE gonderilir
                 "kill_switch": kill_switch,
                 "isik_ana": isik_ana, "isik_mini": isik_mini}
        try:
            sock.sendto(json.dumps(paket).encode(), (JETSON_IP, CMD_PORT))
        except Exception:
            pass

        # ----------------------------------------------------------- CIZIM
        t, loglar, son = tel.snap()
        link = (time.time() - son) < 0.6

        screen.fill(C_BG)
        vw, vh = int(W * 0.66), int(H * 0.62)
        frame, ok = cam.read()
        if ok:
            if kayit and out:
                out.write(frame)
            fh, fw = frame.shape[:2]
            cv2.line(frame, (fw//2-30, fh//2), (fw//2+30, fh//2), (0, 255, 0), 2)
            cv2.line(frame, (fw//2, fh//2-30), (fw//2, fh//2+30), (0, 255, 0), 2)
            rz = cv2.resize(frame, (vw, vh))
            rgb = cv2.cvtColor(rz, cv2.COLOR_BGR2RGB)
            surf = pygame.image.frombuffer(rgb.tobytes(), rgb.shape[1::-1], "RGB")
            screen.blit(surf, (40, 40))
            pygame.draw.rect(screen, C_CYAN, (40, 40, vw, vh), 3, border_radius=5)
            if cam.donuk > 15:
                screen.blit(f_hd.render("GORUNTU DONMUS / KAMERA DEGISIYOR",
                                        True, C_RED), (60, 60))
        else:
            pygame.draw.rect(screen, (25, 30, 40), (40, 40, vw, vh), border_radius=5)
            screen.blit(f_hd.render("KAMERA YAYINI BEKLENIYOR...", True, C_RED),
                        (60, vh // 2))

        arm_r = pygame.Rect(40, 40 + vh + 12, vw, 50)
        pygame.draw.rect(screen, C_GREEN if armed else C_RED, arm_r, border_radius=8)
        screen.blit(f_hd.render(
            "MOTORLAR SERBEST (ARMED) - START ile kilitle" if armed
            else "MOTORLAR KILITLI - START ile ac", True, (0, 0, 0)),
            (arm_r.x + 20, arm_r.y + 14))

        my = arm_r.bottom + 12
        pygame.draw.rect(screen, C_PANEL, (40, my, vw, H - my - 40), border_radius=12)
        pygame.draw.rect(screen, C_CYAN, (40, my, vw, H - my - 40), 2, border_radius=12)
        if mod == 2:
            screen.blit(f_hd.render("MINI ROV HESAPLANAN CIKISLAR (PC -> Topside CAN)",
                                    True, C_YELLOW), (65, my + 12))
            
            y1_cikis = -dive_mini * 5
            x2_donme = yaw_mini * 5
            y2_ileri = fwd_mini * 5
            
            x2_donme = (x2_donme / 500.0)**3 * 500.0 * 0.5
            y2_ileri = y2_ileri * 0.7
            
            destek_ileri = y1_cikis * 0.25
            destek_batma = y2_ileri * 0.15
            
            onsol = int(1500 + y1_cikis + destek_batma)
            onsag = int(1500 - y1_cikis - destek_batma)
            arsol = int(1500 + y2_ileri + destek_ileri + x2_donme)
            arsag = int(1500 + y2_ileri + destek_ileri - x2_donme)
            
            adlar = ["M1 On Sol", "M2 On Sag", "M3 Ark Sol", "M4 Ark Sag"]
            pwms = [onsol, onsag, arsol, arsag]
            
            for i, ad in enumerate(adlar):
                pwm = max(1236, min(1764, pwms[i]))
                yz = my + 48 + (i % 2) * 45
                xz = 65 + (i // 2) * (vw // 2)
                yzd = int((pwm - 1500) / 400 * 100)
                renk = C_GREEN if abs(yzd) < 70 else C_YELLOW
                screen.blit(f_md.render(f"{ad}: {pwm} us ({yzd:+4d}%)", True, renk), (xz, yz))
        else:
            screen.blit(f_hd.render("ANA ROV GERCEK ESC CIKISLARI (firmware telemetrisi)",
                                    True, C_YELLOW), (65, my + 12))
            adlar = ["M1 On Sag", "M2 On Sol", "M3 Ark Sag",
                     "M4 Ark Sol", "M5 Dik Sol", "M6 Dik Sag"]
            for i, ad in enumerate(adlar):
                pwm = int(t["motorlar"][i]) if i < len(t["motorlar"]) else 1500
                yz = my + 48 + (i % 3) * 30
                xz = 65 + (i // 3) * (vw // 2)
                yzd = int((pwm - 1500) / 400 * 100)
                renk = C_GREEN if abs(yzd) < 70 else C_YELLOW
                screen.blit(f_md.render(f"{ad}: {pwm} us ({yzd:+4d}%)", True, renk), (xz, yz))

        px, py = 40 + vw + 20, 40
        pw, ph = W - px - 40, H - 80
        pygame.draw.rect(screen, C_PANEL, (px, py, pw, ph), border_radius=12)
        pygame.draw.rect(screen, C_CYAN, (px, py, pw, ph), 2, border_radius=12)
        ttl = f_ttl.render("ALTAY ISTASYONU", True, C_CYAN)
        screen.blit(ttl, (px + (pw - ttl.get_width()) // 2, py + 18))
        
        # DEBUG SATIRI (gecici)
        dbg = f"DBG->fwd={fwd_jetson} mod={mod} arm={armed} rawW={keys[pygame.K_w]}"
        screen.blit(f_md.render(dbg, True, C_RED), (px + 10, py + 48))
        
        # LOG YAZMA (Teshis icin)
        if seq % 20 == 0:
            try:
                with open("rov_debug_log.txt", "a") as f:
                    f.write(f"fwd_jetson: {fwd_jetson}, Jetson: {link}, Failsafe: {t.get('failsafe', -1)}, Motorlar: {t.get('motorlar', [])}\n")
            except Exception as e:
                print(f"Log hatasi: {e}")

        y = py + 68
        screen.blit(f_hd.render("SISTEM MODU", True, C_TEXT), (px + 22, y))
        screen.blit(f_hd.render(["ANA ROV MANUEL", "OTONOM CIZGI TAKIP",
                                 "MINI ROV MANUEL"][mod], True, C_YELLOW),
                    (px + 22, y + 26))
        y += 70

        screen.blit(f_hd.render("BAGLANTI", True, C_TEXT), (px + 22, y))
        screen.blit(f_md.render(f"Jetson    : {'OK (' + tel.son_addr + ')' if link else 'KOPUK'}", True,
                                C_GREEN if link else C_RED), (px + 22, y + 26))
        screen.blit(f_md.render(f"Failsafe  : {'AKTIF' if t['failsafe'] else 'kapali'}",
                                True, C_RED if t['failsafe'] else C_GREEN),
                    (px + 22, y + 48))
        screen.blit(f_md.render(f"CAN hata  : {t['can_err']}", True,
                                C_RED if t['can_err'] else C_TEXT), (px + 22, y + 70))
        y += 105
        
        if kill_switch == 1:
            pygame.draw.rect(screen, C_RED, (px + 20, y - 5, pw - 40, 30), border_radius=5)
            screen.blit(f_hd.render("!!! KILL SWITCH AKTIF (K) !!!", True, C_TEXT), (px + 30, y))
            y += 40
        else:
            screen.blit(f_md.render("Kill Switch: KAPALI (K tusu ile ac)", True, C_TEXT), (px + 22, y))
            y += 40

        screen.blit(f_hd.render("SENSORLER", True, C_TEXT), (px + 22, y))
        screen.blit(f_md.render(f"Derinlik : {t.get('derinlik_cm', 0):.1f} cm", True, C_TEXT),
                    (px + 22, y + 26))
        screen.blit(f_md.render(f"Basinc   : {t.get('basinc_mbar', 0):.2f} mBar", True, C_TEXT),
                    (px + 22, y + 48))
        screen.blit(f_md.render(f"Sicaklik : {t.get('sicaklik_c', 0):.1f} °C", True, C_TEXT),
                    (px + 22, y + 70))
        screen.blit(f_md.render(f"Mesafe   : {t.get('mesafe_cm', 0):.1f} cm", True, C_TEXT),
                    (px + 22, y + 92))
        screen.blit(f_md.render(f"Roll     : {t.get('roll', 0):.1f} d", True, C_TEXT),
                    (px + 22, y + 114))
        screen.blit(f_md.render(f"Pitch    : {t.get('pitch', 0):.1f} d", True, C_TEXT),
                    (px + 22, y + 136))
        y += 160

        screen.blit(f_hd.render("GORUNTU / ISIK", True, C_TEXT), (px + 22, y))
        screen.blit(f_md.render(f"Kamera : {'MINI ROV' if kamera_role else 'ANA ROV'}",
                                True, C_GREEN), (px + 22, y + 26))
        screen.blit(f_md.render(f"Tilt   : {tilt} d  (gercek {int(t.get('servo', 0))} d)",
                                True, C_YELLOW), (px + 22, y + 48))
        screen.blit(f_md.render(f"Isik   : %{isik_ana}", True, C_TEXT), (px + 22, y + 70))
        if kayit:
            pygame.draw.circle(screen, C_RED, (px + pw - 35, y + 30), 9)
            screen.blit(f_md.render("KAYIT", True, C_RED), (px + pw - 110, y + 22))
        y += 105

        screen.blit(f_hd.render("AYRILMA", True, C_YELLOW), (px + 22, y))
        screen.blit(f_md.render(
            f"{'MINI SERBEST' if mini_serbest else 'MINI KILITLI'} (evt #{magnet_evt})",
            True, C_GREEN if mini_serbest else C_TEXT), (px + 22, y + 26))
        if t["magnet"]:
            screen.blit(f_md.render("MIKNATIS DARBESI SURUYOR", True, C_RED),
                        (px + 22, y + 48))
        y += 82

        screen.blit(f_hd.render("LOG", True, C_YELLOW), (px + 22, y))
        pygame.draw.rect(screen, (15, 20, 28), (px + 18, y + 26, pw - 36, 110),
                         border_radius=5)
        for i, m in enumerate(loglar):
            renk = C_GREEN if m.startswith("OK") else (C_RED if m.startswith("ERR") else C_TEXT)
            screen.blit(f_sm.render(m[:44], True, renk), (px + 26, y + 34 + i * 21))

        pygame.display.flip()
        clock.tick(GONDERIM_HZ)

    # Cikarken motorlari kesin kilitle
    for _ in range(10):
        seq += 1
        try:
            sock.sendto(json.dumps({"seq": seq, "mod": 0, "armed": 0, "fwd": 0,
                                    "strafe": 0, "dive": 0, "yaw": 0,
                                    "magnet_evt": magnet_evt}).encode(),
                        (JETSON_IP, CMD_PORT))
        except Exception:
            pass
        time.sleep(0.02)

    sock.close()
    tel.stop()
    cam.stop()
    if out:
        out.release()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
