from inputs import get_gamepad
import requests
import time

ESP_IP = "http://192.168.1.100"  # CHANGE THIS

# -------- SETTINGS --------
DEADZONE = 0.15
SEND_THRESHOLD = 0.05
LOOP_DELAY = 0.02  # ~50 Hz

# -------- STATE --------
forward = 0
reverse = 0
turn = 0

prev_left = 0
prev_right = 0

# -------- HELPERS --------
def normalize_trigger(val):
    return val / 255

def normalize_stick(val):
    return val / 32768

def apply_deadzone(val):
    return val if abs(val) > DEADZONE else 0

def clamp(val):
    return max(-1, min(1, val))

# -------- MAIN --------
print("🎮 Controller → ESP system active\n")

while True:
    events = get_gamepad()

    for e in events:

        # RT → Forward
        if e.code == "ABS_RZ":
            forward = normalize_trigger(e.state)

        # LT → Reverse
        elif e.code == "ABS_Z":
            reverse = normalize_trigger(e.state)

        # Left stick → Steering
        elif e.code == "ABS_X":
            turn = apply_deadzone(normalize_stick(e.state))

        # Buttons → 90° turns
        elif e.ev_type == "Key" and e.state == 1:
            try:
                if e.code == "BTN_TR":
                    requests.get(f"{ESP_IP}/turn_right", timeout=0.05)
                    print("↪ TURN RIGHT 90°")

                elif e.code == "BTN_TL":
                    requests.get(f"{ESP_IP}/turn_left", timeout=0.05)
                    print("↩ TURN LEFT 90°")

            except requests.exceptions.RequestException:
                print("⚠ ESP not reachable (turn command)")

    # Combine speed
    speed = forward - reverse

    # Differential drive
    left = clamp(speed + turn)
    right = clamp(speed - turn)

    # Send only if change is significant
    if (abs(left - prev_left) > SEND_THRESHOLD or 
        abs(right - prev_right) > SEND_THRESHOLD):

        try:
            requests.get(
                f"{ESP_IP}/move?L={left:.2f}&R={right:.2f}",
                timeout=0.05
            )
            print(f"L:{left:.2f} | R:{right:.2f}")

            prev_left = left
            prev_right = right

        except requests.exceptions.RequestException:
            print("⚠ ESP not reachable (move command)")

    time.sleep(LOOP_DELAY)