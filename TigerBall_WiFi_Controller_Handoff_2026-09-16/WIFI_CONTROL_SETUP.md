# TigerBall Wi-Fi controller — friend handoff

Status: **WHEELS-RAISED BENCH TEST ONLY — NOT FOR ANIMAL USE**

## Connection

```text
laptop/phone browser -> Wi-Fi -> Raspberry Pi 3 -> Teensy USB
-> Teensy pins 20/21 -> Sabertooth S1/S2 -> two motors
```

Use the included `teensy_50cm_benchmark.ino`, not the one-motor firmware from the supplied IMU/UWB ZIP. That older sketch uses different pins and cannot drive this two-wheel GUI.

## On the Raspberry Pi

1. Copy this complete folder to the Pi.
2. Connect the Teensy to the Pi with a USB data cable.
3. Keep the motor battery disconnected during setup.
4. Install the serial dependency:

```text
sudo apt update
sudo apt install -y python3-serial
```

5. First prove the page in simulation, which cannot move motors:

```text
cd /path/to/TigerBall_WiFi_Controller
python3 pi_wifi_bridge.py --simulate
```

6. From the laptop, open the exact private URL printed by the Pi, similar to:

```text
http://tigerball-pi.local:8000/?key=GENERATED_KEY
```

7. Stop simulation with `Ctrl+C`. Start the real bridge:

```text
python3 pi_wifi_bridge.py
```

If automatic Teensy detection cannot identify it, list ports:

```text
python3 -m serial.tools.list_ports -v
```

Then start with the correct port, preferably its stable `/dev/serial/by-id/...` name:

```text
python3 pi_wifi_bridge.py --serial-port /dev/ttyACM0
```

## In the laptop GUI

The page should show:

- `Pi: online`
- `Teensy: /dev/...`
- `Control: available`

Click **Take control**. With both wheels raised and restrained, set **10%**, leave **Force full pulse** off, and only then Arm for the first brief direction check.

## Stop protection

- Releasing a direction sends Stop.
- The browser sends the Pi a heartbeat every 100 ms.
- Missing browser/Wi-Fi heartbeat for 350 ms makes the Pi request Stop and Disarm.
- Missing Pi-to-Teensy commands for 400 ms makes the Teensy neutral and disarm.
- The Sabertooth loss-of-signal setting must still be enabled and physically tested.
- The physical DC battery disconnect remains the final emergency isolation method.

Before increasing above 10%, test page closure, laptop Wi-Fi loss, Pi process exit, and Pi-to-Teensy USB disconnection separately with both wheels raised.

Do not port-forward TCP 8000 from the router. This is a local trusted-network bench tool, not the final animal-use controller.

## Package contents

- `controller_wifi.html` — laptop/phone GUI served by the Pi
- `pi_wifi_bridge.py` — Wi-Fi/HTTP-to-Teensy safety bridge
- `requirements.txt` — Python dependency record
- `teensy_50cm_benchmark/teensy_50cm_benchmark.ino` — required two-motor firmware
- `WIFI_CONTROL_SETUP.md` — this handoff
