import pygame
import serial
import cv2
import socket
import time
from datetime import datetime
import threading 

# --- AYARLAR ---
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
POWER_LIMIT = 75

class CameraThread:
    def __init__(self):
        self.cap = None
        self.src_id = -1
        
        # 0'dan 3'e kadar olan kameraları dene (BlueOS sistemlerinde video1 veya video2 olabilir)
        for i in range(4):
            temp_cap = cv2.VideoCapture(i)
            if temp_cap.isOpened():
                ret, _ = temp_cap.read()
                if ret:
                    self.cap = temp_cap
                    self.src_id = i
                    print(f"✅ [KAMERA] Başarıyla açıldı: /dev/video{i}")
                    break
            temp_cap.release()
            
        if self.cap is None:
            print("❌ [HATA] Hiçbir USB kamera bulunamadı veya açılamadı!")
            self.cap = cv2.VideoCapture(-1) # Hata vermemesi için boş obje

        # Çözünürlük dayatması bazı kameralarda hataya yol açar, OpenCV bunu destekliyorsa yapar
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.ret = False
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.update)
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                if ret: self.frame = frame.copy()

    def read(self):
        with self.lock:
            if self.frame is not None: return self.ret, self.frame.copy()
            return self.ret, None

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()

# --- SENSÖR THREAD (Arka Planda Sürekli Veri Çeker) ---
class SensorThread:
    def __init__(self):
        self.running = True
        self.basinc = 1013.0
        self.mesafe = 0.0
        self.roll = 0.0
        self.thread = threading.Thread(target=self.update)
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while self.running:
            time.sleep(0.05) 

    def stop(self):
        self.running = False
        self.thread.join()

def calculate_thruster_mix(fwd_rev, strain_lr, dive_ud, yaw_lr):
    m_fr = fwd_rev - strain_lr - yaw_lr
    m_fl = fwd_rev + strain_lr + yaw_lr
    m_rr = fwd_rev - strain_lr + yaw_lr
    m_rl = fwd_rev + strain_lr - yaw_lr
    m_vf = dive_ud
    m_vr = dive_ud
    return [int(max(-1.0, min(1.0, val)) * POWER_LIMIT) for val in [m_fr, m_fl, m_rr, m_rl, m_vf, m_vr]]

def calculate_minirov_mix(y1, x1, x2, y2, komp_ileri=0.25, komp_batma=0.15):
    y1_cikis = 1500 - y1
    x2_donme = x2 - 1500
    y2_ileri = 1500 - y2

    x2_donme = (x2_donme / 500.0) ** 3 * 500.0 * 0.5
    y2_ileri = y2_ileri * 0.7

    destek_ileri = y1_cikis * komp_ileri
    destek_batma = y2_ileri * komp_batma

    onsol = 1500 + y1_cikis + destek_batma
    onsag = 1500 - y1_cikis - destek_batma
    arsol = 1500 + y2_ileri + destek_ileri + x2_donme
    arsag = 1500 + y2_ileri + destek_ileri - x2_donme
    
    def constr(val): return max(1236, min(1764, int(val)))
    return [constr(onsag), constr(onsol), constr(arsag), constr(arsol)]

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    except:
        ser = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)

    print("✅ [SİSTEM] PyGame başlatılıyor...")
    pygame.init()
    try:
        # FULLSCREEN bazen Linux'ta sessizce çökmeye neden olur, bu yüzden normal pencere açıyoruz
        screen = pygame.display.set_mode((1280, 720))
        print("✅ [SİSTEM] PyGame ekranı başarıyla oluşturuldu.")
    except Exception as e:
        print(f"❌ [HATA] Ekran açılamadı: {e}")
        return
    
    font_title = pygame.font.SysFont("Consolas", 36, bold=True)
    font_header = pygame.font.SysFont("Consolas", 26, bold=True)
    font_mid = pygame.font.SysFont("Consolas", 22)
    font_small = pygame.font.SysFont("Consolas", 18)
    
    C_BG, C_PANEL = (10, 15, 25), (20, 25, 35)
    C_CYAN, C_GREEN, C_RED = (0, 255, 255), (57, 255, 20), (255, 50, 50)
    C_TEXT, C_GRAY, C_YELLOW = (220, 220, 220), (150, 150, 150), (255, 215, 0)
    
    cap = CameraThread() 
    sensorler = SensorThread()
    seri_log = ["Sistem Baslatiliyor..."] * 5 
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    clock = pygame.time.Clock()
    running = True

    y1, x1, x2, y2 = 1500, 1500, 1500, 1500
    btn_kapat_ana, btn_kapat_mini, btn_miknatis = 0, 0, 0
    aydinlatma_ana, aydinlatma_mini = 0, 0
    aktif_arac = 0
    role_durumu = 0
    onceki_kamera_btn, onceki_foto_btn, onceki_video_btn = 0, 0, 0
    is_recording, out = False, None

    while running:
        sensor_basinc = sensorler.basinc
        sensor_mesafe = sensorler.mesafe
        sensor_roll = sensorler.roll
        
        if ser:
            try:
                while ser.in_waiting > 0:
                    gelen = ser.readline().decode('utf-8', errors='ignore').strip()
                    if gelen:
                        seri_log.append(gelen)
                        if len(seri_log) > 5:
                            seri_log.pop(0) 
            except Exception:
                pass
                
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        son_veri = None
        try:
            while True:
                data, _ = sock.recvfrom(1024)
                son_veri = data
        except BlockingIOError:
            pass 

        if son_veri:
            veri = son_veri.decode().split(',')
            y1, x1, x2, y2 = int(veri[0]), int(veri[1]), int(veri[2]), int(veri[3])
            btn_kapat_ana, btn_kapat_mini = int(veri[4]), int(veri[5])
            btn_kamera, btn_miknatis = int(veri[6]), int(veri[7])
            aydinlatma_ana, aydinlatma_mini = int(veri[8]), int(veri[9])
            btn_foto, btn_video = int(veri[10]), int(veri[11])
            aktif_arac = int(veri[12])
            
            if btn_kamera == 1 and onceki_kamera_btn == 0:
                role_durumu = 1 if role_durumu == 0 else 0
            onceki_kamera_btn = btn_kamera
            
            if btn_foto == 1 and onceki_foto_btn == 0:
                ret, f = cap.read()
                if ret and f is not None: 
                    cv2.imwrite(datetime.now().strftime("FOTO_%H%M%S.jpg"), f)
            onceki_foto_btn = btn_foto
            
            if btn_video == 1 and onceki_video_btn == 0:
                if not is_recording:
                    ret, f = cap.read()
                    if ret and f is not None:
                        out = cv2.VideoWriter(datetime.now().strftime("VID_%H%M%S.avi"), fourcc, 20.0, (1280, 720))
                        is_recording = True
                else:
                    is_recording = False
                    if out: out.release()
            onceki_video_btn = btn_video
            
        ana_motorlar = [0, 0, 0, 0, 0, 0]
        mini_motorlar = [1500, 1500, 1500, 1500]
        mini_y1, mini_x1, mini_x2, mini_y2 = 1500, 1500, 1500, 1500
        
        if aktif_arac == 0: 
            ana_motorlar = calculate_thruster_mix((1500 - y2) / 500.0, (x1 - 1500) / 500.0, (1500 - y1) / 500.0, (x2 - 1500) / 500.0)
        else:               
            mini_y1, mini_x1, mini_x2, mini_y2 = y1, x1, x2, y2
            mini_motorlar = calculate_minirov_mix(y1, x1, x2, y2)

        if ser:
            paket_ana = f"A,{ana_motorlar[0]},{ana_motorlar[1]},{ana_motorlar[2]},{ana_motorlar[3]},{ana_motorlar[4]},{ana_motorlar[5]},{btn_kapat_ana},{role_durumu},{btn_miknatis},{aydinlatma_ana},{int(sensor_roll)}\n"
            ser.write(paket_ana.encode())
            paket_mini = f"M,{mini_y1},{mini_x1},{mini_x2},{mini_y2},{btn_kapat_mini},{aydinlatma_mini}\n"
            ser.write(paket_mini.encode())

        screen.fill(C_BG)
        ret, frame_raw = cap.read()
        if ret and frame_raw is not None:
            if is_recording and out: out.write(frame_raw)
            cv2.line(frame_raw, (frame_raw.shape[1]//2-40, frame_raw.shape[0]//2), (frame_raw.shape[1]//2+40, frame_raw.shape[0]//2), (0, 255, 0), 2)
            cv2.line(frame_raw, (frame_raw.shape[1]//2, frame_raw.shape[0]//2-40), (frame_raw.shape[1]//2, frame_raw.shape[0]//2+40), (0, 255, 0), 2)
            cv2.circle(frame_raw, (frame_raw.shape[1]//2, frame_raw.shape[0]//2), 15, (0, 255, 0), 1)
            frame_resized = cv2.resize(frame_raw, (1280, 720))
            f_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            f_surf = pygame.image.frombuffer(f_rgb.tobytes(), f_rgb.shape[1::-1], "RGB")
            screen.blit(f_surf, (50, 50))
            pygame.draw.rect(screen, C_CYAN, (50, 50, 1280, 720), 3, border_radius=5)
        else:
            pygame.draw.rect(screen, (30,30,30), (50, 50, 1280, 720), border_radius=5)
            screen.blit(font_header.render("KAMERA SINYALI YOK", True, C_RED), (550, 380))

        motor_y = 800
        pygame.draw.rect(screen, C_PANEL, (50, motor_y, 1280, 230), border_radius=15)
        pygame.draw.rect(screen, C_CYAN, (50, motor_y, 1280, 230), 2, border_radius=15)
        
        if aktif_arac == 0:
            screen.blit(font_header.render("ANAROV MOTOR ÇIKIŞLARI (%)", True, C_YELLOW), (80, motor_y + 20))
            screen.blit(font_mid.render(f"M1(Ön Sağ) : %{ana_motorlar[0]:3d}", True, C_CYAN), (80, motor_y + 70))
            screen.blit(font_mid.render(f"M2(Ön Sol) : %{ana_motorlar[1]:3d}", True, C_CYAN), (80, motor_y + 110))
            screen.blit(font_mid.render(f"M3(Ark Sağ): %{ana_motorlar[2]:3d}", True, C_CYAN), (450, motor_y + 70))
            screen.blit(font_mid.render(f"M4(Ark Sol): %{ana_motorlar[3]:3d}", True, C_CYAN), (450, motor_y + 110))
            screen.blit(font_mid.render(f"M5(Dik Sol): %{ana_motorlar[4]:3d}", True, C_CYAN), (820, motor_y + 70))
            screen.blit(font_mid.render(f"M6(Dik Sağ): %{ana_motorlar[5]:3d}", True, C_CYAN), (820, motor_y + 110))
        else:
            screen.blit(font_header.render("MİNİROV MOTOR ÇIKIŞLARI (PWM)", True, C_GREEN), (80, motor_y + 20))
            screen.blit(font_mid.render(f"Ön Sağ (M1)  : {mini_motorlar[0]}", True, C_GREEN), (80, motor_y + 70))
            screen.blit(font_mid.render(f"Ön Sol (M2)  : {mini_motorlar[1]}", True, C_GREEN), (80, motor_y + 110))
            screen.blit(font_mid.render(f"Arka Sağ (M3): {mini_motorlar[2]}", True, C_GREEN), (450, motor_y + 70))
            screen.blit(font_mid.render(f"Arka Sol (M4): {mini_motorlar[3]}", True, C_GREEN), (450, motor_y + 110))

        panel_x, panel_y, panel_w, panel_h = 1380, 50, 490, 980
        pygame.draw.rect(screen, C_PANEL, (panel_x, panel_y, panel_w, panel_h), border_radius=15)
        pygame.draw.rect(screen, C_CYAN, (panel_x, panel_y, panel_w, panel_h), 2, border_radius=15)
        
        title = font_title.render("ALTAY ISTASYONU", True, C_CYAN)
        screen.blit(title, (panel_x + (panel_w - title.get_width()) // 2, panel_y + 30))
        pygame.draw.line(screen, C_CYAN, (panel_x + 30, panel_y + 80), (panel_x + panel_w - 30, panel_y + 80), 2)

        y_off = panel_y + 110
        screen.blit(font_header.render("GÖRÜNTÜ SISTEMI", True, C_TEXT), (panel_x + 30, y_off))
        screen.blit(font_mid.render(f"Aktif Görüntü  : {'KAMERA 2' if role_durumu else 'KAMERA 1'}", True, C_GREEN), (panel_x + 30, y_off + 40))
        if is_recording:
            pulse = abs(time.time() % 1 - 0.5) * 2
            pygame.draw.circle(screen, (int(155 + (100 * pulse)), 0, 0), (panel_x + panel_w - 50, y_off + 100), 10)
            screen.blit(font_mid.render("KAYITTA", True, C_RED), (panel_x + panel_w - 150, y_off + 90))
        pygame.draw.line(screen, (50, 60, 80), (panel_x + 30, y_off + 140), (panel_x + panel_w - 30, y_off + 140), 1)

        y_off += 160
        screen.blit(font_header.render("ROS 2 SENSÖRLERI", True, C_TEXT), (panel_x + 30, y_off))
        screen.blit(font_mid.render(f"Basınc (MS5837)  : {sensor_basinc:.2f} mBar", True, C_TEXT), (panel_x + 30, y_off + 40))
        screen.blit(font_mid.render(f"Mesafe (DYP-L08) : {sensor_mesafe:.1f} cm", True, C_TEXT), (panel_x + 30, y_off + 80))
        screen.blit(font_mid.render(f"Roll (BNO085)    : {sensor_roll:.1f} °", True, C_TEXT), (panel_x + 30, y_off + 120))
        pygame.draw.line(screen, (50, 60, 80), (panel_x + 30, y_off + 170), (panel_x + panel_w - 30, y_off + 170), 1)

        y_off += 190
        screen.blit(font_header.render("SISTEM DURUMU", True, C_TEXT), (panel_x + 30, y_off))
        screen.blit(font_header.render(f"KONTROL: {'MINIROV' if aktif_arac else 'ANAROV'}", True, C_CYAN), (panel_x + 30, y_off + 40))
        
        screen.blit(font_mid.render(f"AnaROV Isik    : %{aydinlatma_ana}", True, C_YELLOW), (panel_x + 30, y_off + 90))
        screen.blit(font_mid.render(f"MiniROV Isik   : %{aydinlatma_mini}", True, C_YELLOW), (panel_x + 30, y_off + 130))
        
        pygame.draw.rect(screen, (40, 20, 20) if btn_miknatis else (20, 40, 20), (panel_x + 20, y_off + 170, panel_w - 40, 160), border_radius=10)
        
        miknatis_yazi = "MIKNATIS : SERBEST BIRAKILDI!" if btn_miknatis else "MIKNATIS : KILITLI"
        screen.blit(font_mid.render(miknatis_yazi, True, C_RED if btn_miknatis else C_GREEN), (panel_x + 30, y_off + 180))
        
        ana_yazi = "ANAROV (SHARE) : KILITLI" if btn_kapat_ana else "ANAROV (SHARE) : AKTIF"
        screen.blit(font_mid.render(ana_yazi, True, C_RED if btn_kapat_ana else C_GREEN), (panel_x + 30, y_off + 230))
        
        mini_yazi = "MINIROV (OPT)  : KILITLI" if btn_kapat_mini else "MINIROV (OPT)  : AKTIF"
        screen.blit(font_mid.render(mini_yazi, True, C_RED if btn_kapat_mini else C_GREEN), (panel_x + 30, y_off + 280))

        y_off += 330
        screen.blit(font_header.render("DENEYAP CAN BUS / SERI LOG", True, C_YELLOW), (panel_x + 30, y_off))
        pygame.draw.rect(screen, (30, 30, 30), (panel_x + 20, y_off + 40, panel_w - 40, 130), border_radius=5)
        
        for i, log_msg in enumerate(seri_log):
            renk = C_GREEN if "CAN_OK" in log_msg else (C_RED if "ERR" in log_msg else C_TEXT)
            screen.blit(font_small.render(log_msg, True, renk), (panel_x + 30, y_off + 50 + (i * 20)))

        pygame.display.flip()
        clock.tick(30)

    if ser: ser.close()
    if out: out.release()
    sensorler.stop()
    cap.stop()
    pygame.quit()

if __name__ == '__main__':
    main()
