# 🎶 Lo-Fi Sampler DSP

A compact DIY sampler / DSP with an **STM32 MCU** and **audio codec**, designed for guitar.  
This project was made to provide guitar players versatility in their effects and provide a 
way to play record and enjoy their music.


---

## 🖼️ Hardware Preview


![IMG_20250906_125656620 (1)](https://github.com/user-attachments/assets/e3231ba0-5bec-4933-8256-623def35e6a2)


---

## ✨ Features
- 🎛️ Real-time audio sampling and playback  
- 🎚️ Adjustable sample rate + bit depth for lo-fi character  
- 🎵 Onboard effects: bit-crushing, filtering, pitch shifting and DSP functions 
- 💾 SD card storage for sample loading and saving  


---

## 🔧 Hardware
- **MCU**: STM32F405RGT6  
- **Audio Codec**: TLV320AIC3104  
- **Op-amps**: TLV9162 for instrument impendace matching   
- **Display**: 128x64 OLED (SSD1306)  
- **Storage**: microSD card  (to do)
- **Power**: USB-C with multiple LDO regulators (3.3V analog/digital, 1.8V core)  
- **Class D amplifier**:Powered by a discrete supply to provide extra versatility

---

## 🧑‍💻 Software
- Written in **C (STM32 HAL)**  
- **SSD1306 driver** for OLED  
- **I2S DMA** for audio streaming  
- **UI**: Potentiometers + buttons with OLED feedback  

---

## 📂 Project Structure




---

## 🚀 Getting Started
1. Flash firmware with **STM32CubeProgrammer**  
2. Power via USB-C  
3. Control parameters with knobs + buttons  

---

## 🛠️ Roadmap / TODO

---

## 📜 License
MIT License – free to use, modify, and share.  

---



