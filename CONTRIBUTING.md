# Contributing to BackTrack

Thank you for your interest in improving this project!  
We welcome contributions of all kinds — bug fixes, documentation, firmware improvements,
ML model enhancements, and new features.

## 🧩 How to Contribute

1. **Fork this repository**
2. **Create a new branch**
   git checkout -b feature/my-feature
3. **Write clean, commented, modular code**
4. **Test your changes thoroughly**
5. **Submit a Pull Request (PR)** with:

- A clear description of your change
- Screenshots or logs (if applicable)
- Mention which issue(s) it solves

---

## 🛠 Code Guidelines

### Firmware (ESP32/Arduino)

- Follow the existing `.h/.cpp` modular structure
- Keep functions small and purposeful
- Add comments for calibration, thresholds, and sensor logic

### Python Tools

- Follow PEP8 style guide
- Use descriptive variable names
- Avoid hardcoding — use `.env` wherever possible

---

## 🧪 Testing

Before submitting a PR:

- Verify IMU readings are stable
- Test posture classification thresholds
- Ensure vibration feedback works reliably
- If Blynk is used, verify that cloud widgets update correctly

---

## 📦 ML Model (Edge Impulse)

If you are modifying the ML pipeline:

- Include the updated `.eim` file
- Document training parameters or changes
- Add sample dataset files if relevant

---

## 🤝 Community Values

We maintain a collaborative and respectful environment.  
Be kind, constructive, and open to discussion.

Thank you for helping make Postura+ better!
