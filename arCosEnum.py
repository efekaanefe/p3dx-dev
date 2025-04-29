# === ARCOS Binary Command Definitions for Pioneer 3-DX ===

# --- Motion Control ---
ENABLE = 0x83               # Enable motors
DISABLE = 0x84              # Disable motors
VEL = 0xB0                  # Set translational velocity (mm/s)
RVEL = 0xB1                 # Set rotational velocity (deg/s)
VEL2 = 0x89                 # Set individual left/right wheel velocities (mm/s)
MOVE_DISTANCE = 0x86        # Move a distance (mm)
ROTATE = 0x87               # Rotate a certain angle (degrees)
STOP = 0xE0                 # Emergency stop (also REBOOT)

# --- Velocity & Acceleration Configuration ---
SET_ACCEL = 0xB2            # Set translational acceleration (mm/s²)
SET_ROT_ACCEL = 0xB3        # Set rotational acceleration (deg/s²)
SET_MAX_TRANS_VEL = 0xB5    # Max translational velocity (mm/s)
SET_MAX_ROT_VEL = 0xB6      # Max rotational velocity (deg/s)
SET_MAX_TRANS_ACCEL = 0xB7  # Max translational acceleration
SET_MAX_ROT_ACCEL = 0xB8    # Max rotational acceleration

# --- Sensor Requests ---
GET_POSE = 0x94             # Request robot pose (X, Y, Theta)
GET_VEL = 0x95              # Request current velocity (translational, rotational)
GET_BATT_VOLT = 0x8B        # Get battery voltage
GET_BATT_CURRENT = 0x8C     # Get battery current

# --- System / Status Commands ---
GET_STATUS = 0x90           # Get system status flags
GET_ID = 0x91               # Get robot hardware ID
GET_VERSION = 0x92          # Get firmware version
GET_PID = 0xB9              # Get PID controller values

# --- Sonar Control ---
SET_SONAR_POWER = 0xA0      # Enable/disable sonar sensors

# --- Command Flow Control ---
SUSPEND = 0x9A              # Suspend command execution
RESUME = 0x9B               # Resume command execution

# --- Reset / Sync ---
REBOOT = 0xE0               # Reboot the robot (also acts as STOP)
SYNC0 = 0xFA                # Sync byte 0 (packet header)
SYNC1 = 0xFB                # Sync byte 1 (packet header)
