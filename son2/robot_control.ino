#include <Wire.h>

// Pin tanımları
#define JOYX A0
#define JOYY A1
#define JOY_BUTTON 2

// Parametreler
#define MAX_DONUS_HIZI 255
#define DONUS_HASSASIYETI 0.1     // 0.1° HASSASİYET - ÇOK HASSAS AMA PRATİK!
#define JOYSTICK_HASSASIYETI 0.3
#define MAX_HAREKET_HIZI 200
#define DEBUG_INTERVAL 200

float hassasiyet = 0.7;
const int maxPWM = 255;

// Motor yapısı
struct MotorPins {
  int pwmSol = 5, pwmSag = 6;
  int solIleri = 7, solGeri = 8;
  int sagIleri = 9, sagGeri = 10;
};

MotorPins motor;

// Sistem değişkenleri
float robotAci = 0.0;
float hedefAci = 0.0;
bool hareketAktif = false;
bool sensorVarMi = false;
bool alanMerkezliMod = true;
unsigned long sonDebugZamani = 0;
float simulasyonAci = 0.0;

// Joystick yapısı
struct JoystickData {
  int x, y;
  float hiz;
  bool aktif;
};

void setup() {
  Serial.begin(9600);
  Serial.println("=== OPTİMİZE EDİLMİŞ ROBOT KONTROL ===");
  
  // Pin ayarları
  pinMode(motor.pwmSol, OUTPUT);
  pinMode(motor.pwmSag, OUTPUT);
  pinMode(motor.solIleri, OUTPUT);
  pinMode(motor.solGeri, OUTPUT);
  pinMode(motor.sagIleri, OUTPUT);
  pinMode(motor.sagGeri, OUTPUT);
  pinMode(JOY_BUTTON, INPUT_PULLUP);
  
  sensorKontrol();
  Serial.println("Sistem hazır! Komutlar: mod, reset, info");
}

void loop() {
  serialKomutIsle();
  
  if (alanMerkezliMod) {
    alanMerkezliKontrol();
  } else {
    orijinalKontrol();
  }
  
  delay(10);
}

// ===== ALAN MERKEZLİ KONTROL =====

void alanMerkezliKontrol() {
  JoystickData joy = joystickOku();
  robotAci = sensorVarMi ? gercekSensorOku() : simulasyonAci;
  
  if (digitalRead(JOY_BUTTON) == LOW) {
    robotReset();
    delay(300);
  }
  
  if (joy.aktif) {
    hedefAci = joystickdanAciHesapla(joy);
    hareketAktif = true;
    ultraHizliDonus(joy.hiz);
  } else {
    motorlariDurdur();
    hareketAktif = false;
  }
  
  if (millis() - sonDebugZamani > DEBUG_INTERVAL) {
    debugYazdir();
    sonDebugZamani = millis();
  }
}

JoystickData joystickOku() {
  JoystickData joy;
  
  int xRaw = analogRead(JOYX);
  int yRaw = analogRead(JOYY);
  
  joy.x = map(xRaw, 0, 1023, -255, 255);
  joy.y = map(yRaw, 0, 1023, 255, -255);
  
  int deadzone = 255 * JOYSTICK_HASSASIYETI;
  if (abs(joy.x) < deadzone) joy.x = 0;
  if (abs(joy.y) < deadzone) joy.y = 0;
  
  joy.hiz = sqrt(joy.x * joy.x + joy.y * joy.y);
  joy.aktif = (joy.x != 0 || joy.y != 0);
  
  return joy;
}

float joystickdanAciHesapla(JoystickData joy) {
  if (!joy.aktif) return -1;
  
  float aci = atan2(joy.x, -joy.y) * 180.0 / PI;
  if (aci < 0) aci += 360;
  
  // ÇAPRAZ AÇILAR İÇİN HASSAS DÜZELTME
  aci = caprazAciDuzelt(aci, joy.x, joy.y);
  
  return aci;
}

// Çapraz açılar için hassas düzeltme fonksiyonu
float caprazAciDuzelt(float aci, int joy_x, int joy_y) {
  // Çapraz yönlerde tam hassasiyet için açıları sabitle
  int absX = abs(joy_x);
  int absY = abs(joy_y);
  
  // Eğer X ve Y değerleri birbirine çok yakınsa (çapraz hareket)
  if (absX > 100 && absY > 100) {
    float oran = (float)absX / (float)absY;
    
    // %10 toleransla çapraz hareket kontrolü (0.9 - 1.1 arası)
    if (oran >= 0.9 && oran <= 1.1) {
      // Tam çapraz hareket - açıyı sabitle
      if (joy_x > 0 && joy_y < 0) {
        return 45.0;   // Kuzeydoğu
      } else if (joy_x > 0 && joy_y > 0) {
        return 135.0;  // Güneydoğu
      } else if (joy_x < 0 && joy_y > 0) {
        return 225.0;  // Güneybatı
      } else if (joy_x < 0 && joy_y < 0) {
        return 315.0;  // Kuzeybatı
      }
    }
  }
  
  // Ana yönler için de hassas düzeltme
  if (absX < 50 && absY > 200) {
    // Dikey hareket
    if (joy_y < 0) return 0.0;    // Kuzey
    else return 180.0;            // Güney
  } else if (absY < 50 && absX > 200) {
    // Yatay hareket
    if (joy_x > 0) return 90.0;   // Doğu
    else return 270.0;            // Batı
  }
  
  return aci; // Orijinal açıyı döndür
}

void ultraHizliDonus(float hareketHizi) {
  float aciFarki = hesaplaAciFarki(hedefAci, robotAci);
  float mutlakFark = abs(aciFarki);
  
  if (mutlakFark > DONUS_HASSASIYETI) {
    // Dönüş gerekli
    if (aciFarki > 0) {
      motorYonuAyarla(&motor, 1, -1);
    } else {
      motorYonuAyarla(&motor, -1, 1);
    }
    
    analogWrite(motor.pwmSol, MAX_DONUS_HIZI);
    analogWrite(motor.pwmSag, MAX_DONUS_HIZI);
    
    if (!sensorVarMi) {
      float donusHizi = (aciFarki > 0) ? 8.0 : -8.0;
      simulasyonAci += donusHizi;
      if (simulasyonAci >= 360) simulasyonAci -= 360;
      if (simulasyonAci < 0) simulasyonAci += 360;
    }
  } else {
    // Düz hareket
    if (hareketHizi > 0) {
      motorYonuAyarla(&motor, 1, 1);
      int pwm = constrain(hareketHizi * 0.8, 0, MAX_HAREKET_HIZI);
      analogWrite(motor.pwmSol, pwm);
      analogWrite(motor.pwmSag, pwm);
    } else {
      motorlariDurdur();
    }
  }
}

// ===== ORİJİNAL KONTROL =====

void orijinalKontrol() {
  int joyXraw = analogRead(JOYX);
  int joyYraw = analogRead(JOYY);

  int xMapped = map(joyXraw, 0, 1023, -maxPWM, maxPWM);
  int yMapped = map(joyYraw, 0, 1023, -maxPWM, maxPWM);

  if (abs(xMapped) < (maxPWM * hassasiyet)) xMapped = 0;
  if (abs(yMapped) < (maxPWM * hassasiyet)) yMapped = 0;

  joystickKontrol(yMapped, xMapped);
}

void joystickKontrol(int ileriGeriPWM, int donusPWM) {
  if (ileriGeriPWM == 0 && donusPWM != 0) {
    if (donusPWM > 0) {
      motorYonuAyarla(&motor, 1, -1);
    } else {
      motorYonuAyarla(&motor, -1, 1);
    }
    analogWrite(motor.pwmSol, abs(donusPWM)/2);
    analogWrite(motor.pwmSag, abs(donusPWM)/2);
  } else {
    int basePWM = ileriGeri(&motor, ileriGeriPWM);
    yonBelirle(&motor, basePWM, donusPWM);
  }
}

int ileriGeri(MotorPins* m, int hiz) {
  if (hiz > 0) {
    motorYonuAyarla(m, 1, 1);
  } else if (hiz < 0) {
    motorYonuAyarla(m, -1, -1);
  } else {
    motorYonuAyarla(m, 0, 0);
  }
  return abs(hiz);
}

void yonBelirle(MotorPins* m, int basePWM, int donusFarki) {
  int solPWM = constrain(basePWM - donusFarki, 0, maxPWM);
  int sagPWM = constrain(basePWM + donusFarki, 0, maxPWM);
  analogWrite(m->pwmSol, solPWM);
  analogWrite(m->pwmSag, sagPWM);
}

// ===== YARDIMCI FONKSİYONLAR =====

float hesaplaAciFarki(float hedef, float mevcut) {
  float fark = hedef - mevcut;
  if (fark > 180) fark -= 360;
  else if (fark < -180) fark += 360;
  return fark;
}

void motorYonuAyarla(MotorPins* m, int solYon, int sagYon) {
  digitalWrite(m->solIleri, solYon == 1);
  digitalWrite(m->solGeri,  solYon == -1);
  digitalWrite(m->sagIleri, sagYon == 1);
  digitalWrite(m->sagGeri,  sagYon == -1);
}

void motorlariDurdur() {
  motorYonuAyarla(&motor, 0, 0);
  analogWrite(motor.pwmSol, 0);
  analogWrite(motor.pwmSag, 0);
}

void robotReset() {
  Serial.println("RESET");
  motorlariDurdur();
  simulasyonAci = 0.0;
  robotAci = 0.0;
  hedefAci = 0.0;
  hareketAktif = false;
}

// ===== SENSÖR VE SİSTEM =====

void sensorKontrol() {
  Wire.begin();
  byte sensorAdresleri[] = {0x1E, 0x68, 0x28, 0x29, 0x0D};
  
  for (int i = 0; i < 5; i++) {
    Wire.beginTransmission(sensorAdresleri[i]);
    if (Wire.endTransmission() == 0) {
      Serial.print("Sensör: 0x");
      Serial.println(sensorAdresleri[i], HEX);
      sensorVarMi = true;
      break;
    }
  }
  
  if (!sensorVarMi) {
    Serial.println("Sensör yok - Simülasyon modu");
  }
}

float gercekSensorOku() {
  return simulasyonAci;
}

void debugYazdir() {
  float aciFarki = hesaplaAciFarki(hedefAci, robotAci);
  
  Serial.print(alanMerkezliMod ? "ALAN" : "ORİJ");
  Serial.print(" | R:");
  Serial.print(robotAci, 1);
  Serial.print("° H:");
  Serial.print(hedefAci, 1);
  Serial.print("° F:");
  Serial.print(aciFarki, 1);
  Serial.print("° | ");
  
  if (alanMerkezliMod) {
    if (abs(aciFarki) > DONUS_HASSASIYETI) {
      Serial.println(aciFarki > 0 ? "SAĞ" : "SOL");
    } else if (hareketAktif) {
      Serial.println("İLERİ");
    } else {
      Serial.println("DUR");
    }
  } else {
    Serial.println("ORİJ");
  }
}

void serialKomutIsle() {
  if (Serial.available()) {
    String komut = Serial.readString();
    komut.trim();
    komut.toLowerCase();
    
    if (komut == "mod") {
      alanMerkezliMod = !alanMerkezliMod;
      Serial.println(alanMerkezliMod ? "ALAN MERKEZLİ" : "ORİJİNAL");
      if (!alanMerkezliMod) motorlariDurdur();
    } else if (komut == "reset") {
      robotReset();
    } else if (komut == "info") {
      Serial.println("=== BİLGİ ===");
      Serial.println("Mod: " + String(alanMerkezliMod ? "ALAN" : "ORİJ"));
      Serial.println("Dönüş: " + String(MAX_DONUS_HIZI) + " PWM");
      Serial.println("Hassasiyet: ±" + String(DONUS_HASSASIYETI) + "°");
      Serial.println("Sensör: " + String(sensorVarMi ? "VAR" : "YOK"));
      Serial.println("Açı: " + String(robotAci) + "°");
    }
  }
}