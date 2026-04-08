# Telemetry Dashboard

Tkinter desktop dashboard for telemetry viewing and joystick control of the ESP32 ball controller.

## Setup

1. Install Python 3.10+.
2. Install dependencies:

```bash
pip install pygame
```

## Run

```bash
python telemetry_dashboard_gui.py
```

## Joystick Input (Thrustmaster T.16000M)

- The GUI now auto-detects USB joysticks through `pygame`.
- It prefers devices whose name contains `Thrustmaster` or `T.16000`.
- If the joystick is connected, it becomes the active input source automatically.
- If no joystick is found (or `pygame` is not installed), drag and WASD controls continue to work.
- A live status line under the joystick shows axis values and currently pressed button indices so you can verify hardware input.

You can see the active input source under the joystick widget in the GUI.

### Default Joystick Button Actions

Current default mapping in the GUI code:

- Button `0` -> Calibrate
- Button `1` -> Send Direction
- Button `2` -> Start Running
- Button `3` -> Emergency Stop

These actions trigger on press (rising edge), so holding a button does not repeatedly spam commands.

### VKB Gladiator NXT EVO Notes

- The GUI now auto-detects VKB devices (name match: `vkb`, `gladiator`, `nxt`, `evo`) and applies a VKB profile.
- VKB profile defaults to `axis_x=0`, `axis_y=1`, and inverted Y.
- The live hardware line shows the active profile and which buttons are currently pressed.

If your VKB button numbers differ from defaults, change `self.joystick_profiles` in `telemetry_dashboard_gui.py` under `vkb_gladiator_nxt_evo` -> `button_map`.
