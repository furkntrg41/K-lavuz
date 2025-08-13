"""
===============================================================================
                    ALAN MERKEZLİ ROBOT KONTROL SİMÜLASYONU
===============================================================================

ÖZELLİKLER:
- Gerçek zamanlı robot simülasyonu
- Gerçek odometri denklemleri (Δd, Δθ)
- Dual mod: Alan merkezli + Orijinal kontrol
- Hassas açı kontrolü (0.1° hassasiyet)
- Çapraz yön sabitlenmesi
- Görsel pusula ve debug paneli
- Hareket izi takibi

KONTROLLER:
W/A/S/D: Hareket kontrolleri
M: Mod değiştir (Alan Merkezli ↔ Orijinal)
R: Rastgele yön ata
C: Sistemi sıfırla
ESC: Çıkış

KOORDINAT SİSTEMİ:
Kuzey=0°, Doğu=90°, Güney=180°, Batı=270°
===============================================================================
"""

import pygame
import numpy as np
import math
import time

pygame.init()
pygame.joystick.init()

# Ekran ayarları
GENISLIK = 1200
YUKSEKLIK = 800
ekran = pygame.display.set_mode((GENISLIK, YUKSEKLIK))
pygame.display.set_caption("Alan Merkezli Robot Kontrol Simülasyonu")

# Renkler
BEYAZ = (255, 255, 255)
SIYAH = (0, 0, 0)
KIRMIZI = (255, 0, 0)
YESIL = (0, 255, 0)
MAVI = (0, 0, 255)
TURUNCU = (255, 165, 0)
GRI = (128, 128, 128)
ACIK_MAVI = (173, 216, 230)
ACIK_GRI = (220, 220, 220)
PEMBE = (255, 192, 203)
KOYU_YESIL = (0, 150, 0)

# Font
font = pygame.font.Font(None, 24)
buyuk_font = pygame.font.Font(None, 36)
baslik_font = pygame.font.Font(None, 28)

class RobotSimulasyonu:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.robotAci = 0.0
        self.hedefAci = 0.0
        self.hareketAktif = False
        self.alanMerkezliMod = True
        self.simulasyonAci = 0.0
        
        # Sistem parametreleri (Arduino ile TAM AYNI)
        self.hassasiyet = 0.7
        self.maxPWM = 255
        self.MAX_DONUS_HIZI = 255
        self.DONUS_HASSASIYETI = 0.1
        self.JOYSTICK_HASSASIYETI = 0.3
        self.MAX_HAREKET_HIZI = 255  # Maksimum hızı 255'e çıkardık
        self.DEBUG_INTERVAL = 200
        
        # Odometri parametreleri (Gerçek robot fiziksel değerleri)
        self.b = 15.0  # Tekerlekler arası mesafe (aks genişliği) - cm cinsinden
        self.tekerlek_capi = 6.5  # Tekerlek çapı - cm cinsinden
        self.encoder_ticks_per_rev = 20  # Tekerlek başına enkoder tick sayısı
        self.encoder_skalasi = 1.0  # Enkoder verilerini gerçek mesafeye çeviren faktör
        self.delta_d_right = 0.0  # Sağ tekerlek yer değiştirmesi
        self.delta_d_left = 0.0   # Sol tekerlek yer değiştirmesi
        self.delta_d = 0.0        # Ortalama doğrusal yer değiştirme
        self.delta_theta = 0.0    # Açısal değişim
        
        # Görsel parametreler
        self.boyut = 30
        self.hiz_skalasi = 0.01
        self.donus_hiz_skalasi = 1.0
        self.gecmis_pozisyonlar = []
        self.max_gecmis = 100
        
        # Motor durumu
        self.sol_motor_pwm = 0
        self.sag_motor_pwm = 0
        self.sol_motor_yon = 0
        self.sag_motor_yon = 0
        
        # Debug
        self.debug_mesaj = "Sistem Hazır"
    
    def encoder_verilerini_hesapla(self, sol_pwm, sag_pwm, sol_yon, sag_yon):
        """Enkoder verilerini simüle eder ve tekerlek yer değiştirmelerini hesaplar"""
        # Gerçek robot fiziksel hesaplamaları
        tekerlek_cevresi = math.pi * self.tekerlek_capi  # cm
        
        # PWM'den RPM'e çevirme (yaklaşık)
        sol_rpm = (sol_pwm / 255.0) * 60.0 * sol_yon  # RPM
        sag_rpm = (sag_pwm / 255.0) * 60.0 * sag_yon  # RPM
        
        # RPM'den cm/s'ye çevirme
        sol_hiz_cms = (sol_rpm / 60.0) * tekerlek_cevresi  # cm/s
        sag_hiz_cms = (sag_rpm / 60.0) * tekerlek_cevresi  # cm/s
        
        # Bir zaman adımındaki yer değiştirmeleri (60 FPS = 1/60 s)
        zaman_adimi = 1.0 / 60.0  # saniye
        self.delta_d_left = sol_hiz_cms * zaman_adimi * self.encoder_skalasi
        self.delta_d_right = sag_hiz_cms * zaman_adimi * self.encoder_skalasi
        
        return self.delta_d_left, self.delta_d_right
    
    def odometri_hesapla(self, delta_d_right, delta_d_left):
        """Odometri denklemlerini kullanarak Δd ve Δθ hesaplar"""
        # Adım 1: Ortalama doğrusal yer değiştirme
        self.delta_d = (delta_d_right + delta_d_left) / 2.0
        
        # Adım 1: Açısal değişim
        self.delta_theta = (delta_d_right - delta_d_left) / self.b
        
        return self.delta_d, self.delta_theta
    
    def konum_guncelle(self, delta_d, delta_theta):
        """Odometri denklemlerini kullanarak aracın yeni konumunu günceller"""
        # Önceki durumları kaydet
        x_prev = self.x
        y_prev = self.y
        theta_prev = self.robotAci
        
        # Adım 2: Yeni konum ve yönelim hesaplama
        # Pygame koordinat sistemi: Y aşağı doğru pozitif
        # Bizim koordinat sistemimiz: Kuzey=0°, Doğu=90°, Güney=180°, Batı=270°
        
        # Yeni X konumu: x_t = x_{t-1} + Δd * sin(θ_{t-1} + Δθ/2)
        self.x = x_prev + delta_d * math.sin(math.radians(theta_prev + delta_theta/2))
        
        # Yeni Y konumu: y_t = y_{t-1} - Δd * cos(θ_{t-1} + Δθ/2) (Y ekseni ters)
        self.y = y_prev - delta_d * math.cos(math.radians(theta_prev + delta_theta/2))
        
        # Yeni yönelim: θ_t = θ_{t-1} + Δθ
        self.robotAci = theta_prev + math.degrees(delta_theta)
        self.simulasyonAci = self.robotAci
        
        # Açıyı 0-360 arasında tut
        if self.robotAci >= 360:
            self.robotAci -= 360
        if self.robotAci < 0:
            self.robotAci += 360
        
        # Sınırları kontrol et
        self.x = max(self.boyut, min(GENISLIK - 300, self.x))
        self.y = max(self.boyut, min(YUKSEKLIK - self.boyut, self.y))
        
        # Geçmiş pozisyonları güncelle
        self.gecmis_pozisyonlar.append((self.x, self.y))
        if len(self.gecmis_pozisyonlar) > self.max_gecmis:
            self.gecmis_pozisyonlar.pop(0)
        
    def hesaplaAciFarki(self, hedef, mevcut):
        fark = hedef - mevcut
        if fark > 180:
            fark -= 360
        elif fark < -180:
            fark += 360
        return fark
    
    def joystickOku(self, raw_x, raw_y):
        joy_x = raw_x
        joy_y = raw_y
        
        deadzone = 255 * self.JOYSTICK_HASSASIYETI
        if abs(joy_x) < deadzone:
            joy_x = 0
        if abs(joy_y) < deadzone:
            joy_y = 0
        
        return joy_x, joy_y
    
    def joystickdanPolarHesapla(self, joy_x, joy_y):
        """Joystick'ten polar koordinatları hesapla (yarıçap + açı)"""
        if joy_x == 0 and joy_y == 0:
            return 0.0, -1  # Yarıçap=0, Açı=-1 (geçersiz)
        
        # Yarıçap hesapla (0-255 arası)
        yaricap = math.sqrt(joy_x * joy_x + joy_y * joy_y)
        yaricap = min(255.0, yaricap)  # Maksimum 255 ile sınırla
        
        # Açı hesapla (Arduino ile TAM AYNI: atan2(joy.x, -joy.y) * 180.0 / PI)
        aci = math.degrees(math.atan2(joy_x, -joy_y))
        if aci < 0:
            aci += 360
        
        # ÇAPRAZ AÇILAR İÇİN HASSAS DÜZELTME
        aci = self.caprazAciDuzelt(aci, joy_x, joy_y)
        
        return yaricap, aci
    
    def joystickdanAciHesapla(self, joy_x, joy_y):
        """Eski fonksiyon - geriye uyumluluk için"""
        yaricap, aci = self.joystickdanPolarHesapla(joy_x, joy_y)
        return aci
    
    def caprazAciDuzelt(self, aci, joy_x, joy_y):
        """Çapraz açılar için hassas düzeltme fonksiyonu - Arduino ile TAM AYNI"""
        # Çapraz yönlerde tam hassasiyet için açıları sabitle
        absX = abs(joy_x)
        absY = abs(joy_y)
        
        # Eğer X ve Y değerleri birbirine çok yakınsa (çapraz hareket)
        if absX > 100 and absY > 100:
            oran = float(absX) / float(absY)
            
            # %10 toleransla çapraz hareket kontrolü (0.9 - 1.1 arası)
            if oran >= 0.9 and oran <= 1.1:
                # Tam çapraz hareket - açıyı sabitle
                if joy_x > 0 and joy_y < 0:
                    self.debug_mesaj = "KUZEYDOĞU SABİTLENDİ"
                    return 45.0   # Kuzeydoğu
                elif joy_x > 0 and joy_y > 0:
                    self.debug_mesaj = "GÜNEYDOĞU SABİTLENDİ"
                    return 135.0  # Güneydoğu
                elif joy_x < 0 and joy_y > 0:
                    self.debug_mesaj = "GÜNEYBATI SABİTLENDİ"
                    return 225.0  # Güneybatı
                elif joy_x < 0 and joy_y < 0:
                    self.debug_mesaj = "KUZEYBATI SABİTLENDİ"
                    return 315.0  # Kuzeybatı
        
        # Ana yönler için de hassas düzeltme
        if absX < 50 and absY > 200:
            # Dikey hareket
            if joy_y < 0:
                self.debug_mesaj = "KUZEY SABİTLENDİ"
                return 0.0    # Kuzey
            else:
                self.debug_mesaj = "GÜNEY SABİTLENDİ"
                return 180.0  # Güney
        elif absY < 50 and absX > 200:
            # Yatay hareket
            if joy_x > 0:
                self.debug_mesaj = "DOĞU SABİTLENDİ"
                return 90.0   # Doğu
            else:
                self.debug_mesaj = "BATI SABİTLENDİ"
                return 270.0  # Batı
        
        self.debug_mesaj = f"Serbest Açı: {aci:.1f}°"
        return aci  # Orijinal açıyı döndür
    
    def polar_differential_control(self, yaricap, hedef_aci):
        """Kutupsal koordinatlardan diferansiyel tekerlek hızlarını hesapla."""
        theta_rad = math.radians(hedef_aci)
        # İleri/geri bileşen (cosine) ve dönüş bileşeni (sine)
        ileri = (yaricap / self.MAX_HAREKET_HIZI) * math.cos(theta_rad)
        donus = (yaricap / self.MAX_HAREKET_HIZI) * math.sin(theta_rad)

        # Ham sol/sağ hız değerleri [-1, 1] aralığında
        sol_hiz = ileri + donus
        sag_hiz = ileri - donus

        # Normalizasyon
        max_deger = max(abs(sol_hiz), abs(sag_hiz), 1.0)
        sol_hiz /= max_deger
        sag_hiz /= max_deger

        # PWM değerlerine çevir
        sol_pwm = int(abs(sol_hiz) * self.MAX_HAREKET_HIZI)
        sag_pwm = int(abs(sag_hiz) * self.MAX_HAREKET_HIZI)

        self.sol_motor_yon = 1 if sol_hiz >= 0 else -1
        self.sag_motor_yon = 1 if sag_hiz >= 0 else -1
        self.sol_motor_pwm = sol_pwm
        self.sag_motor_pwm = sag_pwm

        if yaricap > 0:
            self.odometri_hareket_et()
        else:
            self.motorlariDurdur()
    
    def orijinalKontrol(self, joy_x, joy_y):
        """Orijinal kontrol - Arduino ile TAM AYNI"""
        # Arduino ile aynı: abs(xMapped) < (maxPWM * hassasiyet)
        hassasiyet_siniri = self.maxPWM * self.hassasiyet
        
        xMapped = joy_x
        yMapped = joy_y
        
        if abs(xMapped) < hassasiyet_siniri:
            xMapped = 0
        if abs(yMapped) < hassasiyet_siniri:
            yMapped = 0
        
        ileriGeriPWM = yMapped
        donusPWM = xMapped
        
        if ileriGeriPWM == 0 and donusPWM != 0:
            # Yerinde yön değiştirme
            if donusPWM > 0:
                self.sol_motor_yon = 1
                self.sag_motor_yon = -1
            else:
                self.sol_motor_yon = -1
                self.sag_motor_yon = 1
            
            # Arduino ile aynı: abs(donusPWM)/2
            pwm = abs(donusPWM) // 2
            self.sol_motor_pwm = pwm
            self.sag_motor_pwm = pwm
            
            # Dönüş için odometri hesaplaması
            self.odometri_hareket_et()
        else:
            # Arduino ile aynı: constrain(basePWM - donusFarki, 0, maxPWM)
            basePWM = abs(ileriGeriPWM)
            
            solPWM = max(0, min(self.maxPWM, basePWM - donusPWM))
            sagPWM = max(0, min(self.maxPWM, basePWM + donusPWM))
            
            self.sol_motor_pwm = solPWM
            self.sag_motor_pwm = sagPWM
            
            if ileriGeriPWM > 0:
                self.sol_motor_yon = 1
                self.sag_motor_yon = 1
            elif ileriGeriPWM < 0:
                self.sol_motor_yon = -1
                self.sag_motor_yon = -1
            else:
                self.sol_motor_yon = 0
                self.sag_motor_yon = 0
            
            if basePWM > 0:
                self.odometri_hareket_et()
    
    def odometri_hareket_et(self):
        """Odometri denklemlerini kullanarak aracın hareketini hesaplar"""
        # Enkoder verilerini hesapla
        delta_d_left, delta_d_right = self.encoder_verilerini_hesapla(
            self.sol_motor_pwm, self.sag_motor_pwm, 
            self.sol_motor_yon, self.sag_motor_yon
        )
        
        # Odometri parametrelerini hesapla
        delta_d, delta_theta = self.odometri_hesapla(delta_d_right, delta_d_left)
        
        # Konumu güncelle
        self.konum_guncelle(delta_d, delta_theta)
        
        # Debug mesajını güncelle (eğer zaten polar mesajı yoksa)
        if not self.debug_mesaj.startswith("Polar:"):
            self.debug_mesaj = f"Odometri: Δd={delta_d:.2f}, Δθ={math.degrees(delta_theta):.2f}°"
    
    def motorlariDurdur(self):
        self.sol_motor_yon = 0
        self.sag_motor_yon = 0
        self.sol_motor_pwm = 0
        self.sag_motor_pwm = 0
    
    def robotReset(self):
        self.motorlariDurdur()
        self.simulasyonAci = 0.0
        self.robotAci = 0.0
        self.hedefAci = 0.0
        self.hareketAktif = False
        self.gecmis_pozisyonlar.clear()
        self.debug_mesaj = "Sistem Sıfırlandı"
    
    def modDegistir(self):
        self.alanMerkezliMod = not self.alanMerkezliMod
        if not self.alanMerkezliMod:
            self.motorlariDurdur()
        return "ALAN MERKEZLİ" if self.alanMerkezliMod else "ORİJİNAL"
    
    def kontrol(self, raw_joy_x, raw_joy_y, hedef_aci=None):
        self.robotAci = self.simulasyonAci
        
        joy_x, joy_y = self.joystickOku(raw_joy_x, raw_joy_y)
        
        if joy_x != 0 or joy_y != 0:
            self.hareketAktif = True
            
            if self.alanMerkezliMod:
                # Polar koordinatları hesapla (yarıçap + açı)
                yaricap, aci = self.joystickdanPolarHesapla(joy_x, joy_y)
                
                # Eğer hedef açı verilmişse onu kullan, yoksa joystick'ten hesapla
                if hedef_aci is not None:
                    self.hedefAci = hedef_aci
                else:
                    self.hedefAci = aci
                
                if self.hedefAci >= 0:
                    # Yarıçap ve hedef açıyı kullanarak diferansiyel kontrol
                    self.polar_differential_control(yaricap, self.hedefAci)
                    
                    # Debug mesajını güncelle
                    self.debug_mesaj = f"Polar: R={yaricap:.0f}, θ={self.hedefAci:.1f}°"
            else:
                self.orijinalKontrol(joy_x, joy_y)
        else:
            self.hareketAktif = False
            self.motorlariDurdur()
            self.debug_mesaj = "Durgun"
    
    def ciz(self, ekran):
        # Hareket izi
        if len(self.gecmis_pozisyonlar) > 1:
            for i in range(1, len(self.gecmis_pozisyonlar)):
                alpha = i / len(self.gecmis_pozisyonlar)
                renk_yogunlugu = int(255 * alpha * 0.5)
                renk = (renk_yogunlugu, renk_yogunlugu, 255)
                pygame.draw.circle(ekran, renk, 
                                 (int(self.gecmis_pozisyonlar[i][0]), 
                                  int(self.gecmis_pozisyonlar[i][1])), 2)
        
        # Robot gövdesi - duruma göre renk
        if self.alanMerkezliMod:
            if self.hareketAktif:
                aci_farki = abs(self.hesaplaAciFarki(self.hedefAci, self.robotAci))
                if aci_farki > self.DONUS_HASSASIYETI:
                    robot_rengi = KIRMIZI
                else:
                    robot_rengi = YESIL
            else:
                robot_rengi = MAVI
        else:
            robot_rengi = TURUNCU
        
        pygame.draw.circle(ekran, robot_rengi, (int(self.x), int(self.y)), self.boyut)
        pygame.draw.circle(ekran, SIYAH, (int(self.x), int(self.y)), self.boyut, 3)
        
        # Robot yönü (mevcut)
        aci_radyan = math.radians(self.robotAci)
        son_x = self.x + math.sin(aci_radyan) * self.boyut
        son_y = self.y - math.cos(aci_radyan) * self.boyut
        pygame.draw.line(ekran, BEYAZ, (self.x, self.y), (son_x, son_y), 5)
        
        # Hedef yön (alan merkezli modda)
        if self.alanMerkezliMod and self.hareketAktif:
            hedef_radyan = math.radians(self.hedefAci)
            hedef_x = self.x + math.sin(hedef_radyan) * (self.boyut + 20)
            hedef_y = self.y - math.cos(hedef_radyan) * (self.boyut + 20)
            pygame.draw.line(ekran, KIRMIZI, (self.x, self.y), (hedef_x, hedef_y), 3)

class Joystick:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.aktif = False
        self.yon_adi = ""
        self.hedef_aci = 0.0  # Hedef açıyı sakla
    
    def guncelle(self, tuslar):
        self.x = 0
        self.y = 0
        self.hedef_aci = 0.0
        
        # Tek yön tuşları - doğrudan yön ataması
        if tuslar[pygame.K_w]:
            self.y = 255  # İleri hareket için pozitif
            self.hedef_aci = 0.0  # Kuzey
            self.yon_adi = "KUZEY"
        elif tuslar[pygame.K_s]:
            self.y = 255  # İleri hareket için pozitif
            self.hedef_aci = 180.0  # Güney
            self.yon_adi = "GÜNEY"
        elif tuslar[pygame.K_a]:
            self.y = 255  # İleri hareket için pozitif
            self.hedef_aci = 270.0  # Batı
            self.yon_adi = "BATI"
        elif tuslar[pygame.K_d]:
            self.y = 255  # İleri hareket için pozitif
            self.hedef_aci = 90.0  # Doğu
            self.yon_adi = "DOĞU"
        
        # Çapraz yönler için kombinasyonlar
        if tuslar[pygame.K_w] and tuslar[pygame.K_d]:
            self.y = 255
            self.hedef_aci = 45.0  # Kuzeydoğu
            self.yon_adi = "KUZEYDOĞU"
        elif tuslar[pygame.K_s] and tuslar[pygame.K_d]:
            self.y = 255
            self.hedef_aci = 135.0  # Güneydoğu
            self.yon_adi = "GÜNEYDOĞU"
        elif tuslar[pygame.K_s] and tuslar[pygame.K_a]:
            self.y = 255
            self.hedef_aci = 225.0  # Güneybatı
            self.yon_adi = "GÜNEYBATI"
        elif tuslar[pygame.K_w] and tuslar[pygame.K_a]:
            self.y = 255
            self.hedef_aci = 315.0  # Kuzeybatı
            self.yon_adi = "KUZEYBATI"
        
        if self.y == 0:
            self.yon_adi = "MERKEZ"
        
        self.aktif = (self.y != 0)

class SanalJoystick:
    def __init__(self, x, y, boyut=80):
        self.merkez_x = x
        self.merkez_y = y
        self.boyut = boyut
        self.x = 0
        self.y = 0
        self.aktif = False
        self.surukleniyor = False
        self.deadzone = 0.1
        
    def guncelle(self, mouse_pos, mouse_basili):
        if not mouse_basili:
            self.surukleniyor = False
            self.x = 0
            self.y = 0
            self.aktif = False
            return
            
        mouse_x, mouse_y = mouse_pos
        
        # Joystick alanına tıklanmış mı kontrol et
        mesafe = math.sqrt((mouse_x - self.merkez_x)**2 + (mouse_y - self.merkez_y)**2)
        
        if mesafe <= self.boyut:
            self.surukleniyor = True
        
        if self.surukleniyor:
            # Merkezden uzaklığı hesapla
            dx = mouse_x - self.merkez_x
            dy = mouse_y - self.merkez_y
            
            # Mesafeyi sınırla
            mesafe = math.sqrt(dx*dx + dy*dy)
            if mesafe > self.boyut:
                dx = (dx / mesafe) * self.boyut
                dy = (dy / mesafe) * self.boyut
            
            # Deadzone uygula
            if abs(dx) < self.boyut * self.deadzone:
                dx = 0
            if abs(dy) < self.boyut * self.deadzone:
                dy = 0
            
            # [-boyut, boyut] -> [-255, 255]
            self.x = int((dx / self.boyut) * 255)
            self.y = int((dy / self.boyut) * 255)
            
            self.aktif = (abs(self.x) > 10 or abs(self.y) > 10)
    
    def ciz(self, ekran):
        # Ana çember
        pygame.draw.circle(ekran, ACIK_GRI, (self.merkez_x, self.merkez_y), self.boyut)
        pygame.draw.circle(ekran, SIYAH, (self.merkez_x, self.merkez_y), self.boyut, 2)
        
        # Merkez nokta
        pygame.draw.circle(ekran, SIYAH, (self.merkez_x, self.merkez_y), 5)
        
        # Joystick topu
        if self.aktif:
            top_x = self.merkez_x + int((self.x / 255.0) * self.boyut * 0.8)
            top_y = self.merkez_y + int((self.y / 255.0) * self.boyut * 0.8)
            pygame.draw.circle(ekran, KIRMIZI, (top_x, top_y), 15)
            pygame.draw.circle(ekran, SIYAH, (top_x, top_y), 15, 2)
        else:
            pygame.draw.circle(ekran, GRI, (self.merkez_x, self.merkez_y), 15)
            pygame.draw.circle(ekran, SIYAH, (self.merkez_x, self.merkez_y), 15, 2)
        
        # Yön göstergeleri
        for aci in [0, 90, 180, 270]:
            radyan = math.radians(aci)
            x = self.merkez_x + math.sin(radyan) * (self.boyut + 10)
            y = self.merkez_y - math.cos(radyan) * (self.boyut + 10)
            pygame.draw.circle(ekran, KOYU_YESIL, (int(x), int(y)), 8)
        
        # Değer göstergesi
        if self.aktif:
            deger_text = font.render(f"X:{self.x:3d} Y:{self.y:3d}", True, SIYAH)
            ekran.blit(deger_text, (self.merkez_x - 40, self.merkez_y + self.boyut + 10))

class Gamepad:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.aktif = False
        self._dev = None
        self._deadzone = 0.15
        self._ensure_device()
    
    def _ensure_device(self):
        if self._dev is None and pygame.joystick.get_count() > 0:
            try:
                self._dev = pygame.joystick.Joystick(0)
                self._dev.init()
            except Exception:
                self._dev = None
    
    def guncelle(self):
        # Cihaz takıldıysa bağla
        self._ensure_device()
        self.x = 0
        self.y = 0
        self.aktif = False
        if self._dev is None:
            return
        try:
            axis_x = float(self._dev.get_axis(0))
            axis_y = float(self._dev.get_axis(1))
        except Exception:
            return
        # Deadzone uygula
        if abs(axis_x) < self._deadzone:
            axis_x = 0.0
        if abs(axis_y) < self._deadzone:
            axis_y = 0.0
        # [-1,1] -> [-255,255]
        self.x = int(max(-1.0, min(1.0, axis_x)) * 255)
        self.y = int(max(-1.0, min(1.0, axis_y)) * 255)
        # Çubuğa bir miktar hareket varsa aktif kabul et
        self.aktif = (abs(self.x) > 30 or abs(self.y) > 30)

def pusula_ciz(ekran, x, y, robot_aci, hedef_aci=None):
    boyut = 100
    
    pygame.draw.circle(ekran, ACIK_MAVI, (x, y), boyut, 0)
    pygame.draw.circle(ekran, SIYAH, (x, y), boyut, 3)
    
    # Ana yönler ve çapraz yönler
    yonler = [
        (0, "K", KIRMIZI), (45, "KD", PEMBE), (90, "D", KOYU_YESIL), (135, "GD", PEMBE),
        (180, "G", MAVI), (225, "GB", PEMBE), (270, "B", TURUNCU), (315, "KB", PEMBE)
    ]
    
    for aci, harf, renk in yonler:
        radyan = math.radians(aci)
        pos_x = x + math.sin(radyan) * (boyut - 15)
        pos_y = y - math.cos(radyan) * (boyut - 15)
        
        font_boyut = buyuk_font if len(harf) == 1 else font
        text = font_boyut.render(harf, True, renk)
        text_rect = text.get_rect(center=(pos_x, pos_y))
        ekran.blit(text, text_rect)
    
    # Robot yönü
    robot_radyan = math.radians(robot_aci)
    robot_x = x + math.sin(robot_radyan) * (boyut - 30)
    robot_y = y - math.cos(robot_radyan) * (boyut - 30)
    pygame.draw.line(ekran, MAVI, (x, y), (robot_x, robot_y), 6)
    
    # Hedef yön
    if hedef_aci is not None:
        hedef_radyan = math.radians(hedef_aci)
        hedef_x = x + math.sin(hedef_radyan) * (boyut - 35)
        hedef_y = y - math.cos(hedef_radyan) * (boyut - 35)
        pygame.draw.line(ekran, KIRMIZI, (x, y), (hedef_x, hedef_y), 4)

def bilgi_paneli_ciz(ekran, robot, joystick, gamepad=None, sanal_joystick=None):
    panel_x = GENISLIK - 300
    y_offset = 10
    
    pygame.draw.rect(ekran, ACIK_GRI, (panel_x - 10, 0, 310, YUKSEKLIK))
    pygame.draw.line(ekran, SIYAH, (panel_x - 10, 0), (panel_x - 10, YUKSEKLIK), 2)
    
    # Başlık
    baslik = baslik_font.render("SİMÜLASYON PANELİ", True, KOYU_YESIL)
    ekran.blit(baslik, (panel_x, y_offset))
    y_offset += 40
    
    # Mod göstergesi
    mod_text = buyuk_font.render(f"MOD: {robot.alanMerkezliMod and 'ALAN' or 'ORİJ'}", 
                                True, KIRMIZI if robot.alanMerkezliMod else TURUNCU)
    ekran.blit(mod_text, (panel_x, y_offset))
    y_offset += 40
    
    aci_farki = robot.hesaplaAciFarki(robot.hedefAci, robot.robotAci)
    
    # Giriş kaynağını belirle
    giris_kaynagi = "Klavye"
    if sanal_joystick and sanal_joystick.aktif:
        giris_kaynagi = "Sanal Joystick"
    elif gamepad and gamepad.aktif:
        giris_kaynagi = "Gamepad"
    
    bilgiler = [
        f"Pozisyon: ({robot.x:.0f}, {robot.y:.0f})",
        f"Robot Açısı: {robot.robotAci:.1f}°",
        f"Hedef Açısı: {robot.hedefAci:.1f}°",
        f"Açı Farkı: {aci_farki:.1f}°",
        f"Hassasiyet: ±{robot.DONUS_HASSASIYETI}°",
        "",
        "POLAR KONTROL:",
        f"Yarıçap: {math.sqrt(robot.sol_motor_pwm**2 + robot.sag_motor_pwm**2):.0f}/255",
        f"Hız: {max(robot.sol_motor_pwm, robot.sag_motor_pwm)}/255",
        "",
        "ODOMETRİ VERİLERİ:",
        f"Δd_right: {robot.delta_d_right:.2f} cm",
        f"Δd_left: {robot.delta_d_left:.2f} cm",
        f"Δd: {robot.delta_d:.2f} cm",
        f"Δθ: {math.degrees(robot.delta_theta):.2f}°",
        f"Aks Genişliği (b): {robot.b:.1f} cm",
        f"Tekerlek Çapı: {robot.tekerlek_capi:.1f} cm",
        "",
        f"Joystick: {joystick.yon_adi}",
        f"Giriş Kaynağı: {giris_kaynagi}",
        (f"Gamepad XY: ({gamepad.x}, {gamepad.y})" if gamepad and gamepad.aktif else ""),
        (f"Sanal XY: ({sanal_joystick.x}, {sanal_joystick.y})" if sanal_joystick and sanal_joystick.aktif else ""),
        f"Debug: {robot.debug_mesaj}",
        "",
        f"Sol Motor: {robot.sol_motor_yon} ({robot.sol_motor_pwm})",
        f"Sağ Motor: {robot.sag_motor_yon} ({robot.sag_motor_pwm})",
        "",
        f"Durum: {robot_durum_al(robot)}",
        "",
        "SİSTEM ÖZELLİKLERİ:",
        "✓ Gerçek odometri denklemleri",
        "✓ Hassas açı kontrolü (0.1°)",
        "✓ Çapraz yön sabitlenmesi",
        "✓ Dual kontrol modu",
        "✓ Polar koordinat sistemi",
        "✓ Yüksek hız kontrolü (255)",
        "✓ Gerçek zamanlı debug",
        "✓ Hareket izi takibi",
        "✓ Sanal joystick kontrolü",
        "",
        "KONTROLLER:",
        "W/A/S/D: Hareket (Klavye)",
        "Mouse: Sanal Joystick",
        "M: Mod değiştir",
        "R: Rastgele yön",
        "C: Reset",
        "ESC: Çıkış"
    ]
    
    for bilgi in bilgiler:
        if bilgi:
            renk = SIYAH
            if "Durum:" in bilgi:
                if "DÖNÜYOR" in bilgi:
                    renk = KIRMIZI
                elif "İLERİ" in bilgi:
                    renk = YESIL
            elif "✓" in bilgi:
                renk = KOYU_YESIL
            elif "SİSTEM" in bilgi:
                renk = MAVI
            
            text = font.render(bilgi, True, renk)
            ekran.blit(text, (panel_x, y_offset))
        y_offset += 20

def robot_durum_al(robot):
    if not robot.hareketAktif:
        return "DURGUN"
    
    if robot.alanMerkezliMod:
        aci_farki = robot.hesaplaAciFarki(robot.hedefAci, robot.robotAci)
        if abs(aci_farki) > robot.DONUS_HASSASIYETI:
            return f"DÖNÜYOR ({('SAĞ' if aci_farki > 0 else 'SOL')} {abs(aci_farki):.1f}°)"
        else:
            return "İLERİ (HASSAS!)"
    else:
        return "ORİJİNAL MOD"

def main():
    saat = pygame.time.Clock()
    calisir = True
    
    robot = RobotSimulasyonu(GENISLIK // 2 - 150, YUKSEKLIK // 2)
    joystick = Joystick()
    gamepad = Gamepad()
    sanal_joystick = SanalJoystick(150, YUKSEKLIK - 150)  # Sol alt köşe
    
    print("===============================================================================")
    print("                    ALAN MERKEZLİ ROBOT KONTROL SİMÜLASYONU")
    print("===============================================================================")
    print("Sistem Özellikleri:")
    print("✓ Gerçek odometri denklemleri")
    print("✓ Hassas açı kontrolü (0.1° hassasiyet)")
    print("✓ Çapraz yön sabitlenmesi")
    print("✓ Dual kontrol modu (Alan Merkezli + Orijinal)")
    print("✓ Gerçek zamanlı debug ve görselleştirme")
    print("✓ Hareket izi takibi")
    print("✓ Sanal joystick kontrolü")
    print("")
    print("Kontroller:")
    print("- W/A/S/D: Klavye hareketi")
    print("- Mouse: Sanal joystick (sol alt köşe)")
    print("- M: Mod değiştir, R: Rastgele yön, C: Reset, ESC: Çıkış")
    print("===============================================================================")
    
    while calisir:
        mouse_pos = pygame.mouse.get_pos()
        mouse_basili = pygame.mouse.get_pressed()[0]  # Sol tık
        
        for olay in pygame.event.get():
            if olay.type == pygame.QUIT:
                calisir = False
            elif olay.type == pygame.KEYDOWN:
                if olay.key == pygame.K_ESCAPE:
                    calisir = False
                elif olay.key == pygame.K_m:
                    mod = robot.modDegistir()
                    print(f"Mod değiştirildi: {mod}")
                elif olay.key == pygame.K_r:
                    robot.simulasyonAci = np.random.randint(0, 360)
                    print(f"Rastgele yön atandı: {robot.simulasyonAci}°")
                elif olay.key == pygame.K_c:
                    robot.robotReset()
                    print("Sistem sıfırlandı")
        
        tuslar = pygame.key.get_pressed()
        joystick.guncelle(tuslar)
        gamepad.guncelle()
        sanal_joystick.guncelle(mouse_pos, mouse_basili)
        
        # Giriş önceliği: Sanal Joystick > Gamepad > Klavye
        if sanal_joystick.aktif:
            robot.kontrol(sanal_joystick.x, sanal_joystick.y, None)
        elif gamepad.aktif:
            robot.kontrol(gamepad.x, gamepad.y, None)
        else:
            robot.kontrol(joystick.x, joystick.y, joystick.hedef_aci)
        
        ekran.fill(BEYAZ)
        
        robot.ciz(ekran)
        pusula_ciz(ekran, GENISLIK - 150, 150, robot.robotAci, 
                  robot.hedefAci if robot.alanMerkezliMod and robot.hareketAktif else None)
        sanal_joystick.ciz(ekran)  # Sanal joystick'i çiz
        bilgi_paneli_ciz(ekran, robot, joystick, gamepad, sanal_joystick)
        
        pygame.display.flip()
        saat.tick(60)
    
    pygame.quit()
    print("===============================================================================")
    print("Simülasyon sonlandırıldı. Teşekkürler!")
    print("===============================================================================")

if __name__ == "__main__":
    main()