import pygame
import serial
import cv2
import time
import sys
import ctypes
from datetime import datetime

# Windows'un yakınlaştırma yapmasını engellemek için sistem kilidi
try:
    ctypes.windll.user32.SetProcessDPIAware()
except AttributeError:
    pass

# --- AYARLAR ---
SERIAL_PORT = 'COM8' # Portunuzu kontrol edin
BAUD_RATE = 115200
POWER_LIMIT_PERCENT = 75 # Maksimum güç sınırı %75

# Başlangıç Değerleri
kp = 1.5
kd = 0.25
camera_index = 0 

def calculate_thruster_mix(fwd_rev, strain_lr, dive_ud, yaw_lr):
    # Yatay Motorlar (Vektörel)
    m_fr = fwd_rev - strain_lr - yaw_lr
    m_fl = fwd_rev + strain_lr + yaw_lr
    m_rr = fwd_rev + strain_lr - yaw_lr
    m_rl = fwd_rev - strain_lr + yaw_lr
    
    # Dikey Motorlar
    m_vf = dive_ud
    m_vr = dive_ud
    
    thrusters = [m_fr, m_fl, m_rr, m_rl, m_vf, m_vr]
    scaled_thrusters = []
    
    for val in thrusters:
        val = max(-1.0, min(1.0, val)) 
        scaled_val = val * POWER_LIMIT_PERCENT
        scaled_thrusters.append(int(scaled_val))
        
    return scaled_thrusters

def init_camera(index):
    # Windows için DirectShow (CAP_DSHOW) kullanımı kameranın hızlı açılmasını sağlar
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap

def main():
    global kp, kd, camera_index
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Bağlantı başarılı: {SERIAL_PORT}")
    except:
        ser = None
        print("UYARI: Seri port bağlantısı kurulamadı. Sadece arayüz çalışıyor.")

    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
    if joystick: joystick.init()

    # GERÇEK EKRAN ÇÖZÜNÜRLÜĞÜNÜ AL
    infoObject = pygame.display.Info()
    REAL_WIDTH, REAL_HEIGHT = infoObject.current_w, infoObject.current_h
    screen = pygame.display.set_mode((REAL_WIDTH, REAL_HEIGHT), pygame.FULLSCREEN)
    
    # SANAL TUVAL (1920x1080)
    VIRTUAL_W, VIRTUAL_H = 1920, 1080
    virtual_surface = pygame.Surface((VIRTUAL_W, VIRTUAL_H))
    
    # Yazı Tipleri
    font_title = pygame.font.SysFont("Consolas", 36, bold=True)
    font_header = pygame.font.SysFont("Consolas", 26, bold=True)
    font_text = pygame.font.SysFont("Consolas", 22)
    font_small = pygame.font.SysFont("Consolas", 18)
    
    # Renk Paleti
    C_BG = (10, 15, 25)
    C_PANEL = (20, 25, 35)
    C_CYAN = (0, 255, 255)
    C_GREEN = (57, 255, 20)
    C_RED = (255, 50, 50)
    C_TEXT = (220, 220, 220)
    C_YELLOW = (255, 215, 0)
    C_GRAY = (150, 150, 150)
    
    cap = init_camera(camera_index)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = None
    is_recording = False
    
    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
                
            if event.type == pygame.JOYBUTTONDOWN:
                # PID
                if event.button == 11: kp = round(kp + 0.1, 1) # Yön Yukarı
                elif event.button == 12: kp = round(max(0.0, kp - 0.1), 1) # Yön Aşağı
                elif event.button == 10: kd = round(kd + 0.1, 1) # R1
                elif event.button == 9:  kd = round(max(0.0, kd - 0.1), 1) # L1
                
                # KAMERA DEĞİŞTİRME
                elif event.button == 4:
                    cap.release()
                    time.sleep(0.5)
                    camera_index = (camera_index + 1) % 3
                    cap = init_camera(camera_index)
                    
                # KAYIT
                elif event.button == 1: # O (Yuvarlak)
                    ret, f = cap.read()
                    if ret: cv2.imwrite(datetime.now().strftime("FOTO_%H%M%S.jpg"), cv2.flip(f, 1))
                elif event.button == 0: # X (Çarpı)
                    if not is_recording:
                        ret, f = cap.read()
                        if ret:
                            out = cv2.VideoWriter(datetime.now().strftime("VID_%H%M%S.avi"), fourcc, 20.0, (f.shape[1], f.shape[0]))
                            is_recording = True
                    else:
                        is_recording = False
                        if out: out.release()

        # --- EKSEN (AXIS) OKUMALARI VE HESAPLAMALAR ---
        x1, y1, x2, y2, btn_kapat = 0.0, 0.0, 0.0, 0.0, 0
        motor_degerleri = [0, 0, 0, 0, 0, 0]
        
        if joystick:
            btn_kapat = joystick.get_button(6) # SHARE Tuşu (Acil Kapatma)
            
            # Sol Analog (X1, Y1) -> Yengeç ve Batma
            x1 = joystick.get_axis(0) 
            y1 = joystick.get_axis(1) * -1.0 # Yukarı iterken eksi verir, tersliyoruz
            
            # Sağ Analog (X2, Y2) -> Dönüş ve İleri
            x2 = joystick.get_axis(2) 
            y2 = joystick.get_axis(3) * -1.0 
            
            # Deadzone (Titremeyi önlemek için)
            deadzone = 0.05
            if abs(x1) < deadzone: x1 = 0.0
            if abs(y1) < deadzone: y1 = 0.0
            if abs(x2) < deadzone: x2 = 0.0
            if abs(y2) < deadzone: y2 = 0.0

            # Mikser Fonksiyonu: -100 ile +100 arası motor PWM karşılıklarını üretir
            motor_degerleri = calculate_thruster_mix(fwd_rev=y2, strain_lr=x1, dive_ud=y1, yaw_lr=x2)

        # --- SERİ İLETİŞİM ---
        # YENİ FORMAT: A,M1,M2,M3,M4,M5,M6,KapatmaTusu,Kp,Kd
        if ser:
            paket = f"A,{motor_degerleri[0]},{motor_degerleri[1]},{motor_degerleri[2]},{motor_degerleri[3]},{motor_degerleri[4]},{motor_degerleri[5]},{btn_kapat},{int(kp*100)},{int(kd*100)}\n"
            ser.write(paket.encode('utf-8'))

        # ==========================================================
        # --- SANAL TUVALE ÇİZİM ---
        # ==========================================================
        virtual_surface.fill(C_BG) 
        
        # 1. KAMERA ALANI
        ret, frame_raw = cap.read()
        if ret:
            frame = cv2.flip(frame_raw, 1)
            if is_recording and out: out.write(frame)
            
            # Dijital Nişangah
            cv2.line(frame, (frame.shape[1]//2-40, frame.shape[0]//2), (frame.shape[1]//2+40, frame.shape[0]//2), (0, 255, 0), 2)
            cv2.line(frame, (frame.shape[1]//2, frame.shape[0]//2-40), (frame.shape[1]//2, frame.shape[0]//2+40), (0, 255, 0), 2)
            cv2.circle(frame, (frame.shape[1]//2, frame.shape[0]//2), 15, (0, 255, 0), 1)
            
            frame = cv2.resize(frame, (1280, 720))
            f_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            f_surf = pygame.image.frombuffer(f_rgb.tobytes(), f_rgb.shape[1::-1], "RGB")
            virtual_surface.blit(f_surf, (50, 150))
            pygame.draw.rect(virtual_surface, C_CYAN, (50, 150, 1280, 720), 3, border_radius=5)
        else:
            pygame.draw.rect(virtual_surface, (30,30,30), (50, 150, 1280, 720), border_radius=5)
            virtual_surface.blit(font_header.render("KAMERA SINYALI YOK", True, C_RED), (550, 480))

        # 2. SAĞ KONTROL PANELİ ALANI
        panel_x = 1380
        panel_y = 50
        panel_w = 490
        panel_h = 980
        pygame.draw.rect(virtual_surface, C_PANEL, (panel_x, panel_y, panel_w, panel_h), border_radius=15)
        pygame.draw.rect(virtual_surface, C_CYAN, (panel_x, panel_y, panel_w, panel_h), 2, border_radius=15)
        
        # BAŞLIK
        title = font_title.render("AnaROV İSTASYONU", True, C_CYAN)
        virtual_surface.blit(title, (panel_x + (panel_w - title.get_width()) // 2, panel_y + 30))
        pygame.draw.line(virtual_surface, C_CYAN, (panel_x + 30, panel_y + 80), (panel_x + panel_w - 30, panel_y + 80), 2)

       # --- MODÜL 1: KAMERA VE KAYIT ---
        y_off = panel_y + 100
        virtual_surface.blit(font_header.render("GÖRÜNTÜ SİSTEMİ", True, C_TEXT), (panel_x + 30, y_off))
        
        virtual_surface.blit(font_text.render(f"Kamera İndeksi : {camera_index}", True, C_GREEN), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render("Değiştirmek için [BUTON 4] kullanın", True, C_GRAY), (panel_x + 30, y_off + 65))
        
        # Eklenen Kayıt ve Fotoğraf Butonu Yazıları
        virtual_surface.blit(font_text.render("Fotoğraf Çek   : [O Yuvarlak]", True, C_TEXT), (panel_x + 30, y_off + 110))
        virtual_surface.blit(font_text.render("Video Kayıt    : [X Çarpı]", True, C_TEXT), (panel_x + 30, y_off + 150))
        
        if is_recording:
            pulse = abs(time.time() % 1 - 0.5) * 2 
            pygame.draw.circle(virtual_surface, (int(155 + (100 * pulse)), 0, 0), (panel_x + panel_w - 50, y_off + 160), 10)
            virtual_surface.blit(font_text.render("KAYITTA", True, C_RED), (panel_x + panel_w - 150, y_off + 150))
            
        pygame.draw.line(virtual_surface, (50, 60, 80), (panel_x + 30, y_off + 200), (panel_x + panel_w - 30, y_off + 200), 1)

        # --- MODÜL 2: PID AYARLARI ---
        y_off += 230 # Modül 1 büyüdüğü için Modül 2'yi biraz daha aşağı kaydırdık
        virtual_surface.blit(font_header.render("YALPAMA (ROLL) PID", True, C_TEXT), (panel_x + 30, y_off))
        virtual_surface.blit(font_text.render(f"Kp Değeri : {kp:.1f}", True, C_GREEN), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render("+ [Yön Yukarı]  /  - [Yön Aşağı]", True, C_GRAY), (panel_x + 30, y_off + 65))
        virtual_surface.blit(font_text.render(f"Kd Değeri : {kd:.1f}", True, C_GREEN), (panel_x + 30, y_off + 100))
        virtual_surface.blit(font_small.render("+ [R1 Tuşu]     /  - [L1 Tuşu]", True, C_GRAY), (panel_x + 30, y_off + 125))
        pygame.draw.line(virtual_surface, (50, 60, 80), (panel_x + 30, y_off + 160), (panel_x + panel_w - 30, y_off + 160), 1)

        # --- MODÜL 3: ANALOG VE İTKİ DEĞERLERİ (YENİ EKLENEN KISIM) ---
        y_off += 180
        virtual_surface.blit(font_header.render("KONTROL VE İTKİ DEĞERLERİ", True, C_TEXT), (panel_x + 30, y_off))
        
        # Analog Ham Değerler
        virtual_surface.blit(font_small.render(f"Sol Analog (X1, Y1) : {x1:.2f} , {y1:.2f}", True, C_YELLOW), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render(f"Sağ Analog (X2, Y2) : {x2:.2f} , {y2:.2f}", True, C_YELLOW), (panel_x + 30, y_off + 70))
        
        # 6 Motorun Yüzdelik Çıkışları
        m_y_baslangic = y_off + 120
        virtual_surface.blit(font_text.render(f"M1(Ön Sağ) : %{motor_degerleri[0]:3d}", True, C_CYAN), (panel_x + 30, m_y_baslangic))
        virtual_surface.blit(font_text.render(f"M2(Ön Sol) : %{motor_degerleri[1]:3d}", True, C_CYAN), (panel_x + 250, m_y_baslangic))
        
        virtual_surface.blit(font_text.render(f"M3(Ark Sağ): %{motor_degerleri[2]:3d}", True, C_CYAN), (panel_x + 30, m_y_baslangic + 40))
        virtual_surface.blit(font_text.render(f"M4(Ark Sol): %{motor_degerleri[3]:3d}", True, C_CYAN), (panel_x + 250, m_y_baslangic + 40))
        
        virtual_surface.blit(font_text.render(f"M5(Dik Sol): %{motor_degerleri[4]:3d}", True, C_CYAN), (panel_x + 30, m_y_baslangic + 80))
        virtual_surface.blit(font_text.render(f"M6(Dik Sağ): %{motor_degerleri[5]:3d}", True, C_CYAN), (panel_x + 250, m_y_baslangic + 80))

        pygame.draw.line(virtual_surface, (50, 60, 80), (panel_x + 30, m_y_baslangic + 130), (panel_x + panel_w - 30, m_y_baslangic + 130), 1)

        # --- MODÜL 4: SİSTEM DURUMU ---
        y_off = m_y_baslangic + 150
        virtual_surface.blit(font_header.render("SİSTEM DURUMU", True, C_TEXT), (panel_x + 30, y_off))
        
        durum_renk = C_RED if btn_kapat else C_GREEN
        durum_metin = "KİLİTLİ (MOTORLAR DURDU)" if btn_kapat else "AKTİF (MOTORLAR HAZIR)"
        virtual_surface.blit(font_text.render(durum_metin, True, durum_renk), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render("Acil Kapatma için [SHARE] tuşuna basın", True, C_GRAY), (panel_x + 30, y_off + 70))
        
        virtual_surface.blit(font_small.render("Çıkmak İçin [ESC] Tuşuna Basın", True, (80, 80, 80)), (panel_x + 30, panel_y + panel_h - 40))

        # ==========================================================
        # EKRANA BAS
        # ==========================================================
        scaled_surface = pygame.transform.scale(virtual_surface, (REAL_WIDTH, REAL_HEIGHT))
        screen.blit(scaled_surface, (0, 0))
        
        pygame.display.flip()
        clock.tick(30)

    if ser: ser.close()
    if out: out.release()
    cap.release()
    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()