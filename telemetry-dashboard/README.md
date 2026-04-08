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

- Button `0` -> Enter Calibration Mode (works from Standby, Calibration, or Running)
- Button `1` -> Send Calibration Direction and Auto-Start
- Button `2` -> Stop to Standby
- Button `3` -> Start Running

These actions trigger on press (rising edge), so holding a button does not repeatedly spam commands.

Direction safety behavior:

- Joystick movement alone does **not** send direction commands anymore.
- Direction is only sent when you explicitly trigger an action (`SEND DIRECTION` button or joystick button action).
- Calibration direction send requires a minimum stick deflection (`0.35` magnitude by default) to avoid accidental tiny commands.

Running mode behavior:

- While state is `Running`, the GUI continuously streams joystick direction at ~10 Hz.
- This applies to mouse drag, WASD, and USB joystick input.

## Mock Mode (No ESP)

- Use the `Mock mode (no ESP required)` toggle in the Status panel.
- In mock mode, the dashboard behaves as connected and simulates telemetry locally.
- You can verify joystick axes, button mappings, state transitions, and control workflow without ESP hardware.

### VKB Gladiator NXT EVO Notes

- The GUI now auto-detects VKB devices (name match: `vkb`, `gladiator`, `nxt`, `evo`) and applies a VKB profile.
- VKB profile defaults to `axis_x=0`, `axis_y=1`, and inverted Y.
- The live hardware line shows the active profile and which buttons are currently pressed.

If your VKB button numbers differ from defaults, change `self.joystick_profiles` in `telemetry_dashboard_gui.py` under `vkb_gladiator_nxt_evo` -> `button_map`.
