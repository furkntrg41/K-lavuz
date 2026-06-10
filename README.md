# Robot Kılavuz

Arduino tabanlı joystick kontrollü robot projesi ve Python hareket simülasyonu.

## İçerik

| Dosya | Açıklama |
|-------|---------|
| `robot_control.ino` | Arduino joystick kontrol kodu |
| `simulation.py` | Python tabanlı hareket simülasyonu |
| `orjinal_kod.txt` | Referans ve notlar |

## Donanım

- Arduino
- Analog joystick (Vrx/Vry eksen + SW buton)
  - `Vrx` → X ekseni (sağ/sol)
  - `Vry` → Y ekseni (ileri/geri)
  - `SW` → D2 pini (opsiyonel buton)

## Kurulum

**Arduino:**
```
Arduino IDE → robot_control.ino → Yükle
```

**Simülasyon:**
```bash
python simulation.py
```
