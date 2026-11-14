# Dragon---Mini-Captsone-2

Dragon robot for MEC2105 Projet intégrateur 2 (Polytechnique Montréal).
One ESP32 DevKit drives 4 wheel motors, 2 wing servos, an egg servo and a bubble motor,
controlled by the course wireless remote (5 buttons) or a phone over Wi-Fi.

- `RP2_Dragon_Remote_PlatformIO/RP2_Dragon_Remote/` — main firmware (PlatformIO, board `esp32dev`)
- `RP2_Motor_code/` — earlier motor test sketch
- `CLAUDE.md` — wiring, pinout, behaviour and tuning notes

Build / upload: open `RP2_Dragon_Remote_PlatformIO/RP2_Dragon_Remote` in VS Code + PlatformIO, then `pio run -t upload`.
