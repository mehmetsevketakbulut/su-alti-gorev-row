#!/usr/bin/env python3
"""
Jetson ↔ Deneyap UART Haberleşme Test Scripti

Bu scripti Jetson Orin Nano üzerinde çalıştırın.
Seri port bağlantısını ve motor kontrolünü adım adım test eder.

Kullanım:
  python3 test_serial_jetson.py                     # Temel bağlantı testi
  python3 test_serial_jetson.py --motor-test        # Motor sıra testi
  python3 test_serial_jetson.py --port /dev/ttyTHS0 # Farklı port
  python3 test_serial_jetson.py --list-ports        # Mevcut portları listele
"""

import serial
import serial.tools.list_ports
import time
import sys
import argparse
import os


# ══════════════════════════════════════════════════════════════════
#  RENK KODLARI (Terminal çıktısı için)
# ══════════════════════════════════════════════════════════════════
class C:
    OK   = '\033[92m'  # Yeşil
    WARN = '\033[93m'  # Sarı
    FAIL = '\033[91m'  # Kırmızı
    INFO = '\033[94m'  # Mavi
    BOLD = '\033[1m'
    END  = '\033[0m'


def print_ok(msg):   print(f"  {C.OK}✅ {msg}{C.END}")
def print_fail(msg): print(f"  {C.FAIL}❌ {msg}{C.END}")
def print_warn(msg): print(f"  {C.WARN}⚠️  {msg}{C.END}")
def print_info(msg): print(f"  {C.INFO}ℹ️  {msg}{C.END}")


def print_header(title):
    print(f"\n{C.BOLD}{'═' * 60}")
    print(f"  {title}")
    print(f"{'═' * 60}{C.END}")


# ══════════════════════════════════════════════════════════════════
#  TEST 0: SİSTEM KONTROLÜ
# ══════════════════════════════════════════════════════════════════
def test_system_check():
    print_header("TEST 0: Sistem Kontrolü")
    
    # Kullanıcı grubu kontrolü
    try:
        groups = os.popen('groups').read().strip()
        if 'dialout' in groups:
            print_ok(f"Kullanıcı 'dialout' grubunda: {groups}")
        else:
            print_fail(f"Kullanıcı 'dialout' grubunda DEĞİL!")
            print_info("Düzeltme: sudo usermod -a -G dialout $USER && reboot")
    except Exception:
        print_warn("Grup kontrolü yapılamadı (Windows?)")

    # nvgetty kontrolü (Jetson'da seri konsol servisi)
    try:
        result = os.popen('systemctl is-active nvgetty.service 2>/dev/null').read().strip()
        if result == 'active':
            print_fail("nvgetty servisi AKTİF! Seri portu bloke ediyor olabilir.")
            print_info("Düzeltme: sudo systemctl stop nvgetty && sudo systemctl disable nvgetty")
        elif result == 'inactive':
            print_ok("nvgetty servisi devre dışı (iyi)")
        else:
            print_info(f"nvgetty durumu: {result}")
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════
#  TEST 1: PORT LİSTELEME
# ══════════════════════════════════════════════════════════════════
def test_list_ports():
    print_header("TEST 1: Mevcut Seri Portlar")
    
    ports = serial.tools.list_ports.comports()
    if not ports:
        print_fail("Hiç seri port bulunamadı!")
        
        # Jetson'a özgü port kontrolü
        jetson_ports = ['/dev/ttyTHS0', '/dev/ttyTHS1', '/dev/ttyTHS2',
                       '/dev/ttyAMA0', '/dev/ttyAMA1', '/dev/ttyS0']
        print_info("Jetson portları kontrol ediliyor...")
        for p in jetson_ports:
            if os.path.exists(p):
                print_ok(f"  {p} MEVCUT")
            else:
                print_warn(f"  {p} bulunamadı")
    else:
        for p in ports:
            print_ok(f"{p.device} — {p.description} [{p.hwid}]")
    
    return ports


# ══════════════════════════════════════════════════════════════════
#  TEST 2: PORT AÇMA
# ══════════════════════════════════════════════════════════════════
def test_open_port(port, baud=115200):
    print_header(f"TEST 2: Port Açma ({port} @ {baud})")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print_ok(f"Port açıldı: {ser.name}")
        print_info(f"  Baud rate : {ser.baudrate}")
        print_info(f"  Data bits : {ser.bytesize}")
        print_info(f"  Parity    : {ser.parity}")
        print_info(f"  Stop bits : {ser.stopbits}")
        return ser
    except serial.SerialException as e:
        print_fail(f"Port açılamadı: {e}")
        if 'Permission' in str(e):
            print_info("Düzeltme: sudo chmod 666 " + port)
            print_info("  veya: sudo usermod -a -G dialout $USER && reboot")
        elif 'FileNotFoundError' in str(type(e).__name__) or 'No such file' in str(e):
            print_info(f"Port '{port}' mevcut değil. --list-ports ile kontrol edin.")
        return None
    except Exception as e:
        print_fail(f"Beklenmeyen hata: {e}")
        return None


# ══════════════════════════════════════════════════════════════════
#  TEST 3: VERİ GÖNDERME
# ══════════════════════════════════════════════════════════════════
def test_send_data(ser):
    print_header("TEST 3: Veri Gönderme (Nötr Paket)")
    
    # Nötr motor değerleri (tüm motorlar durgun)
    paket = "A,0,0,0,0,0,0,0,150,25\n"
    
    print_info(f"Gönderilen paket: {repr(paket)}")
    print_info(f"Paket boyutu   : {len(paket)} byte")
    
    try:
        bytes_sent = ser.write(paket.encode('utf-8'))
        print_ok(f"{bytes_sent} byte gönderildi")
        
        # 5 paket daha gönder (stabilite testi)
        for i in range(5):
            ser.write(paket.encode('utf-8'))
            time.sleep(0.05)  # 20Hz
        print_ok("5 nötr paket daha gönderildi (20Hz)")
        
        return True
    except Exception as e:
        print_fail(f"Veri gönderilemedi: {e}")
        return False


# ══════════════════════════════════════════════════════════════════
#  TEST 4: SÜREKLİ GÖNDERİM (Failsafe test)
# ══════════════════════════════════════════════════════════════════
def test_continuous_send(ser, duration=5):
    print_header(f"TEST 4: Sürekli Gönderim ({duration}s, 20Hz)")
    
    paket = "A,0,0,0,0,0,0,0,150,25\n"
    count = 0
    start = time.time()
    
    print_info("Nötr paketler gönderiliyor (motorlar dönmemeli)...")
    print_info("Deneyap'ın failsafe'e düşmemesi için sürekli veri gerekli.")
    
    try:
        while time.time() - start < duration:
            ser.write(paket.encode('utf-8'))
            count += 1
            time.sleep(0.05)  # 20Hz
        
        hz = count / duration
        print_ok(f"{count} paket gönderildi ({hz:.1f} Hz)")
        
        # Deneyap'tan gelen debug yanıtını oku (varsa)
        time.sleep(0.1)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
            print_ok(f"Deneyap yanıtı: {response[:200]}")
        else:
            print_warn("Deneyap'tan yanıt gelmedi (debug kapalıysa normal)")
        
        return True
    except Exception as e:
        print_fail(f"Sürekli gönderim hatası: {e}")
        return False


# ══════════════════════════════════════════════════════════════════
#  TEST 5: MOTOR SIRA TESTİ
# ══════════════════════════════════════════════════════════════════
def test_motors(ser, power=20, duration_per_motor=2.0):
    print_header(f"TEST 5: Motor Sıra Testi (%{power} güç, {duration_per_motor}s)")
    
    motor_names = [
        "M1 (Ön Sağ - Yatay)",
        "M2 (Ön Sol - Yatay)",
        "M3 (Arka Sağ - Yatay)",
        "M4 (Arka Sol - Yatay)",
        "M5 (Dikey Sol)",
        "M6 (Dikey Sağ)"
    ]
    
    print_warn("⚡ DİKKAT: Motorlar dönecek! Pervane güvenliğini sağlayın!")
    print_info("Devam etmek için Enter'a basın, iptal için Ctrl+C...")
    
    try:
        input()
    except KeyboardInterrupt:
        print_warn("İptal edildi.")
        return
    
    nötr = "A,0,0,0,0,0,0,0,150,25\n"
    
    for i in range(6):
        motor_vals = [0, 0, 0, 0, 0, 0]
        motor_vals[i] = power
        
        paket = f"A,{motor_vals[0]},{motor_vals[1]},{motor_vals[2]},{motor_vals[3]},{motor_vals[4]},{motor_vals[5]},0,150,25\n"
        
        print(f"\n  🔄 {motor_names[i]} — %{power} güç ({duration_per_motor}s)")
        print(f"     Paket: {paket.strip()}")
        
        # Motor çalıştır
        start = time.time()
        while time.time() - start < duration_per_motor:
            ser.write(paket.encode('utf-8'))
            time.sleep(0.05)
        
        # Nötre al
        for _ in range(10):
            ser.write(nötr.encode('utf-8'))
            time.sleep(0.05)
        
        print(f"     ✅ {motor_names[i]} tamamlandı — nötre alındı")
        time.sleep(1.0)  # Motorlar arası bekleme
    
    # Final nötr
    for _ in range(20):
        ser.write(nötr.encode('utf-8'))
        time.sleep(0.05)
    
    print_ok("Tüm motor testleri tamamlandı!")


# ══════════════════════════════════════════════════════════════════
#  TEST 6: KILL SWITCH TESTİ
# ══════════════════════════════════════════════════════════════════
def test_kill_switch(ser):
    print_header("TEST 6: Kill Switch (btn_kapat) Testi")
    
    # Önce nötr paketler gönder
    nötr = "A,0,0,0,0,0,0,0,150,25\n"
    for _ in range(20):
        ser.write(nötr.encode('utf-8'))
        time.sleep(0.05)
    print_ok("Nötr paketler gönderildi (1s)")
    
    # Kill komutu gönder
    kill = "A,0,0,0,0,0,0,1,150,25\n"
    print_info(f"Kill paketi gönderiliyor: {kill.strip()}")
    for _ in range(10):
        ser.write(kill.encode('utf-8'))
        time.sleep(0.05)
    print_ok("Kill komutu gönderildi")
    
    # Tekrar nötr
    time.sleep(1)
    for _ in range(20):
        ser.write(nötr.encode('utf-8'))
        time.sleep(0.05)
    print_ok("Nötr pakete geri dönüldü")


# ══════════════════════════════════════════════════════════════════
#  ANA PROGRAM
# ══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description='Jetson ↔ Deneyap UART Haberleşme Test Scripti',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Örnekler:
  python3 test_serial_jetson.py                      # Temel test
  python3 test_serial_jetson.py --motor-test         # Motor testi
  python3 test_serial_jetson.py --port /dev/ttyTHS0  # Farklı port
  python3 test_serial_jetson.py --list-ports         # Port listele
  python3 test_serial_jetson.py --power 30           # %30 güçle motor testi
        """
    )
    parser.add_argument('--port', default='/dev/ttyTHS1',
                       help='Seri port (varsayılan: /dev/ttyTHS1)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate (varsayılan: 115200)')
    parser.add_argument('--list-ports', action='store_true',
                       help='Mevcut seri portları listele ve çık')
    parser.add_argument('--motor-test', action='store_true',
                       help='Motor sıra testi çalıştır')
    parser.add_argument('--kill-test', action='store_true',
                       help='Kill switch testi')
    parser.add_argument('--power', type=int, default=20,
                       help='Motor test gücü %% (varsayılan: 20)')
    parser.add_argument('--duration', type=float, default=2.0,
                       help='Her motor için test süresi (varsayılan: 2.0s)')
    
    args = parser.parse_args()
    
    print(f"{C.BOLD}")
    print("╔══════════════════════════════════════════════════════════╗")
    print("║     JETSON ↔ DENEYAP UART HABERLEŞME TEST ARACI       ║")
    print("║     AnaROV Protokolü: A,m1,m2,m3,m4,m5,m6,btn,kp,kd  ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(f"{C.END}")
    
    # Sistem kontrolü
    test_system_check()
    
    # Port listele
    test_list_ports()
    
    if args.list_ports:
        return
    
    # Port aç
    ser = test_open_port(args.port, args.baud)
    if not ser:
        print(f"\n{C.FAIL}Port açılamadı, testler durduruluyor.{C.END}")
        sys.exit(1)
    
    try:
        # Veri gönderme testi
        test_send_data(ser)
        
        # Sürekli gönderim testi
        test_continuous_send(ser, duration=3)
        
        # Motor testi (opsiyonel)
        if args.motor_test:
            test_motors(ser, power=args.power, duration_per_motor=args.duration)
        
        # Kill switch testi (opsiyonel)
        if args.kill_test:
            test_kill_switch(ser)
        
        print_header("TEST SONUÇLARI")
        print_ok("Tüm temel testler başarıyla tamamlandı!")
        if not args.motor_test:
            print_info("Motor testi için: python3 test_serial_jetson.py --motor-test")
        
    except KeyboardInterrupt:
        print(f"\n{C.WARN}Kullanıcı durdurdu.{C.END}")
        # Nötr paket gönder
        try:
            nötr = "A,0,0,0,0,0,0,0,150,25\n"
            for _ in range(10):
                ser.write(nötr.encode('utf-8'))
                time.sleep(0.05)
            print_ok("Motorlar nötre alındı.")
        except Exception:
            pass
    finally:
        ser.close()
        print_info("Seri port kapatıldı.")


if __name__ == '__main__':
    main()
