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
SERIAL_PORT = 'COM8' 
BAUD_RATE = 115200

# Başlangıç Değerleri
kp = 2.5
kd = 0.5
komp_ileri = 0.25 
komp_batma = 0.15 
camera_index = 1 

def map_value(val, in_min, in_max, out_min, out_max):
    normalized = (val - in_min) / (in_max - in_min)
    return int(out_min + (normalized * (out_max - out_min)))

def init_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap

def main():
    global kp, kd, komp_ileri, komp_batma, camera_index
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except:
        ser = None

    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
    if joystick: joystick.init()

    # GERÇEK EKRAN ÇÖZÜNÜRLÜĞÜNÜ AL
    infoObject = pygame.display.Info()
    REAL_WIDTH, REAL_HEIGHT = infoObject.current_w, infoObject.current_h
    screen = pygame.display.set_mode((REAL_WIDTH, REAL_HEIGHT), pygame.FULLSCREEN)
    
    # SANAL TUVAL (Her şey buraya 1920x1080 olarak çizilecek, sonra ekrana sığdırılacak)
    VIRTUAL_W, VIRTUAL_H = 1920, 1080
    virtual_surface = pygame.Surface((VIRTUAL_W, VIRTUAL_H))
    
    # Yazı Tipleri (Sanal tuvale göre sabit boyutlu)
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
    C_GRAY = (150, 150, 150)
    
    cap = init_camera(camera_index)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = None
    is_recording = False
    
    last_l2_time = 0
    last_r2_time = 0
    
    clock = pygame.time.Clock()
    running = True

    while running:
        current_time = time.time()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
                
            if event.type == pygame.JOYBUTTONDOWN:
                # PID
                if event.button == 11: kp = round(kp + 0.1, 1)
                elif event.button == 12: kp = round(max(0.0, kp - 0.1), 1)
                elif event.button == 10: kd = round(kd + 0.1, 1)
                elif event.button == 9:  kd = round(max(0.0, kd - 0.1), 1)
                
                # KOMPANZASYON (İleri)
                elif event.button == 3: komp_ileri = round(min(1.0, komp_ileri + 0.05), 2) 
                elif event.button == 2: komp_ileri = round(max(0.0, komp_ileri - 0.05), 2) 
                
                # KAMERA
                elif event.button == 4:
                    cap.release()
                    time.sleep(0.5)
                    camera_index = (camera_index + 1) % 3
                    cap = init_camera(camera_index)
                    
                # KAYIT
                elif event.button == 1:
                    ret, f = cap.read()
                    if ret: cv2.imwrite(datetime.now().strftime("FOTO_%H%M%S.jpg"), cv2.flip(f, 1))
                elif event.button == 0:
                    if not is_recording:
                        ret, f = cap.read()
                        if ret:
                            out = cv2.VideoWriter(datetime.now().strftime("VID_%H%M%S.avi"), fourcc, 20.0, (f.shape[1], f.shape[0]))
                            is_recording = True
                    else:
                        is_recording = False
                        if out: out.release()

        # --- EKSEN (AXIS) OKUMALARI ---
        x1, y1, x2, y2, btn_kapat = 1500, 1500, 1500, 1500, 0
        if joystick:
            btn_kapat = joystick.get_button(6)
            
            x1 = map_value(joystick.get_axis(0), -1.0, 1.0, 1060, 1940)
            y1 = map_value(joystick.get_axis(1), -1.0, 1.0, 1060, 1940)
            x2 = map_value(joystick.get_axis(2), -1.0, 1.0, 1060, 1940)
            y2 = map_value(joystick.get_axis(3), -1.0, 1.0, 1060, 1940)
            
            # KOMPANZASYON (Batma) L2 / R2 Tetikleri ile
            l2_val = joystick.get_axis(4)
            r2_val = joystick.get_axis(5)
            
            if r2_val > 0.5 and (current_time - last_r2_time > 0.2):
                komp_batma = round(min(1.0, komp_batma + 0.05), 2)
                last_r2_time = current_time
                
            if l2_val > 0.5 and (current_time - last_l2_time > 0.2):
                komp_batma = round(max(0.0, komp_batma - 0.05), 2)
                last_l2_time = current_time

        # --- SERİ İLETİŞİM ---
        if ser:
            paket = f"{y1},{x1},{x2},{y2},{btn_kapat},{int(kp*100)},{int(kd*100)},{int(komp_ileri*100)},{int(komp_batma*100)}\n"
            ser.write(paket.encode('utf-8'))


        # ==========================================================
        # --- SANAL TUVALE ÇİZİM (1920x1080) ---
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
            
            # Kamerayı 1280x720 boyutuna sabitleyip Sanal Tuvale Çiz
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
        title = font_title.render("YER ISTASYONU", True, C_CYAN)
        virtual_surface.blit(title, (panel_x + (panel_w - title.get_width()) // 2, panel_y + 30))
        pygame.draw.line(virtual_surface, C_CYAN, (panel_x + 30, panel_y + 80), (panel_x + panel_w - 30, panel_y + 80), 2)

        # --- MODÜL 1: KAMERA VE KAYIT ---
        y_off = panel_y + 110
        virtual_surface.blit(font_header.render("GÖRÜNTÜ SISTEMI", True, C_TEXT), (panel_x + 30, y_off))
        
        virtual_surface.blit(font_text.render(f"Kamera Indeksi : {camera_index}", True, C_GREEN), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render("Degistirmek icin [BUTON 4] kullanin", True, C_GRAY), (panel_x + 30, y_off + 65))
        
        virtual_surface.blit(font_text.render("Fotograf Çek   : [O Yuvarlak]", True, C_TEXT), (panel_x + 30, y_off + 110))
        virtual_surface.blit(font_text.render("Video Kayit    : [X Carpi]", True, C_TEXT), (panel_x + 30, y_off + 150))
        
        if is_recording:
            pulse = abs(time.time() % 1 - 0.5) * 2 
            pygame.draw.circle(virtual_surface, (int(155 + (100 * pulse)), 0, 0), (panel_x + panel_w - 50, y_off + 160), 10)
            virtual_surface.blit(font_text.render("KAYITTA", True, C_RED), (panel_x + panel_w - 150, y_off + 150))
            
        pygame.draw.line(virtual_surface, (50, 60, 80), (panel_x + 30, y_off + 200), (panel_x + panel_w - 30, y_off + 200), 1)

        # --- MODÜL 2: PID AYARLARI ---
        y_off += 230
        virtual_surface.blit(font_header.render("PID DENGELEME", True, C_TEXT), (panel_x + 30, y_off))
        
        virtual_surface.blit(font_text.render(f"Kp Değeri : {kp:.1f}", True, C_GREEN), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render("+ [Yön Yukarı]  /  - [Yön Aşağı]", True, C_GRAY), (panel_x + 30, y_off + 65))
        
        virtual_surface.blit(font_text.render(f"Kd Değeri : {kd:.1f}", True, C_GREEN), (panel_x + 30, y_off + 110))
        virtual_surface.blit(font_small.render("+ [R1 Tuşu]     /  - [L1 Tuşu]", True, C_GRAY), (panel_x + 30, y_off + 135))
        
        pygame.draw.line(virtual_surface, (50, 60, 80), (panel_x + 30, y_off + 180), (panel_x + panel_w - 30, y_off + 180), 1)

        # --- MODÜL 3: ÇAPRAZ KOMPANZASYON ---
        y_off += 210
        virtual_surface.blit(font_header.render("ÇAPRAZ KOMPANZASYON", True, C_TEXT), (panel_x + 30, y_off))
        
        virtual_surface.blit(font_text.render(f"İleri Komp : {komp_ileri:.2f}", True, C_CYAN), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render("+ [Üçgen Tuşu]  /  - [Kare Tuşu]", True, C_GRAY), (panel_x + 30, y_off + 65))
        
        virtual_surface.blit(font_text.render(f"Batma Komp : {komp_batma:.2f}", True, C_CYAN), (panel_x + 30, y_off + 110))
        virtual_surface.blit(font_small.render("+ [R2 Tetik]    /  - [L2 Tetik]", True, C_GRAY), (panel_x + 30, y_off + 135))
        
        pygame.draw.line(virtual_surface, (50, 60, 80), (panel_x + 30, y_off + 180), (panel_x + panel_w - 30, y_off + 180), 1)

        # --- MODÜL 4: SİSTEM DURUMU ---
        y_off += 210
        virtual_surface.blit(font_header.render("SISTEM DURUMU", True, C_TEXT), (panel_x + 30, y_off))
        
        durum_renk = C_RED if btn_kapat else C_GREEN
        durum_metin = "KILITLI (MOTORLAR DURDU)" if btn_kapat else "AKTIF (MOTORLAR HAZIR)"
        virtual_surface.blit(font_text.render(durum_metin, True, durum_renk), (panel_x + 30, y_off + 40))
        virtual_surface.blit(font_small.render("Acil Kapatma icin [SHARE] tusuna basin", True, C_GRAY), (panel_x + 30, y_off + 70))
        
        # Çıkış uyarısı
        virtual_surface.blit(font_small.render("Yazilimdan Cikmak Icin [ESC] Tusuna Basin", True, (80, 80, 80)), (panel_x + 30, panel_y + panel_h - 40))

        # ==========================================================
        # SANAL TUVALİ GERÇEK EKRANA SIĞDIR (MUCİZE BURADA)
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