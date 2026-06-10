<div align="center">

# Robot Kılavuz

**Arduino joystick kontrollü robot ve Python hareket simülasyonu**

[![Arduino](https://img.shields.io/badge/Arduino-C%2FC%2B%2B-00979D?style=flat-square&logo=arduino&logoColor=white)](https://arduino.cc)
[![Python](https://img.shields.io/badge/Python-simulation-3776AB?style=flat-square&logo=python&logoColor=white)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-22c55e?style=flat-square)](LICENSE)


</div>

---

## İçerik

| Dosya | Açıklama |
|-------|---------|
| `son2/robot_control.ino` | Arduino joystick kontrol kodu |
| `son2/simulation.py` | Python hareket simülasyonu |
| `son2/orjinal_kod.txt` | Referans notlar |

---

## Donanım

```
Arduino
  ├── A0 ← Joystick Vrx  (X ekseni: sağ/sol)
  ├── A1 ← Joystick Vry  (Y ekseni: ileri/geri)
  ├── D2 ← Joystick SW   (buton, opsiyonel)
  └── GND, 5V ← Joystick GND, VCC
```

**Gerekli parçalar:**
- Arduino Uno / Nano
- Analog joystick modülü
- Motor sürücü (L298N veya benzer)

---

## Kurulum

### Arduino

```
1. Arduino IDE'yi açın
2. son2/robot_control.ino dosyasını açın
3. Doğru port ve kart seçimini yapın
4. Upload butonuna tıklayın
```

### Python Simülasyon

```bash
cd son2
python simulation.py
```

---

## Lisans

[MIT](LICENSE) © 2026 furkntrg41
