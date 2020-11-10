# -----------------------------------------------------------------------------
# Hardware config.

# Maximum velocity for each axis in millimeter per minute.
MAX_VELOCITY_MM_PER_MIN_X = 4001
MAX_VELOCITY_MM_PER_MIN_Y = 4001
MAX_VELOCITY_MM_PER_MIN_Z = 1
MAX_VELOCITY_MM_PER_MIN_E = 1
MIN_VELOCITY_MM_PER_MIN = 10
# Average velocity for endstop calibration procedure
CALIBRATION_VELOCITY_MM_PER_MIN = 1000

# Stepper motors steps per millimeter for each axis.
ENABLE_L293D = True
if ENABLE_L293D:
    ENABLE_FULL_STEP = False
    ENABLE_HALF_STEP = True
    if ENABLE_FULL_STEP:
        STEPPER_PULSES_PER_MM_X = 6.667
        STEPPER_PULSES_PER_MM_Y = 6.667
    elif ENABLE_HALF_STEP:
        STEPPER_PULSES_PER_MM_X = 16
        STEPPER_PULSES_PER_MM_Y = 14

STEPPER_PULSES_PER_MM_Z = 1
STEPPER_PULSES_PER_MM_E = 1

# Stepper motors steps per rotation for each axis.
STEPPER_STEPS_PER_REV_X = 20
STEPPER_STEPS_PER_REV_Y = 20

# Invert axises direction, by default(False) high level means increase of
# position. For inverted(True) axis, high level means decrease of position.
STEPPER_INVERTED_X = True 
STEPPER_INVERTED_Y = False
STEPPER_INVERTED_Z = False
STEPPER_INVERTED_E = True

# Invert zero end stops switches. By default(False) low level on input pin
# means that axis in zero position. For inverted(True) end stops, high level
# means zero position.
ENDSTOP_INVERTED_X = True
ENDSTOP_INVERTED_Y = True
ENDSTOP_INVERTED_Z = False  # Auto leveler

# Workplace physical size.
TABLE_SIZE_X_MM = 51
TABLE_SIZE_Y_MM = 41
TABLE_SIZE_Z_MM = 0

# Mixed settings.
STEPPER_PULSE_LENGTH_US = 2
STEPPER_MAX_ACCELERATION_MM_PER_S2 = 3000  # for all axis, mm per sec^2
SPINDLE_MAX_RPM = 10000
EXTRUDER_MAX_TEMPERATURE = 250
BED_MAX_TEMPERATURE = 100
MIN_TEMPERATURE = 40
EXTRUDER_PID = {"P": 0.059161177519,
                "I": 0.00206217171374,
                "D": 0.206217171374}
BED_PID = {"P": 0.226740848076,
           "I": 0.00323956215053,
           "D": 0.323956215053}

# -----------------------------------------------------------------------------
# Pins configuration.

# Enable pin for all steppers, low level is enabled.
ENABLE_STEPPER_ENABLE_PIN = True
STEPPERS_ENABLE_PIN = 0

STEPPER_STEP_PIN_X = 0
STEPPER_STEP_PIN_Y = 0
STEPPER_STEP_PIN_Z = 0
STEPPER_STEP_PIN_E = 0
if ENABLE_L293D:
    STEPPER_STEP_PINS_Y = [24,25,8,7]
    STEPPER_STEP_PINS_X = [12,16,20,21] 
    PEN_PIN = 18 

STEPPER_DIR_PIN_X = 0
STEPPER_DIR_PIN_Y = 0 
STEPPER_DIR_PIN_Z = 0
STEPPER_DIR_PIN_E = 0

ENABLE_SPINDLE = False
SPINDLE_PWM_PIN = 0
ENABLE_FAN = False
FAN_PIN = 0
ENABLE_EXTRUDER_HEATER = False
EXTRUDER_HEATER_PIN = 0
ENABLE_BED_HEATER = False
BED_HEATER_PIN = 0
if ENABLE_EXTRUDER_HEATER:
    EXTRUDER_TEMPERATURE_SENSOR_CHANNEL = 2
if ENABLE_BED_HEATER:
    BED_TEMPERATURE_SENSOR_CHANNEL = 1

ENDSTOP_PIN_X = 10
ENDSTOP_PIN_Y = 9
ENDSTOP_PIN_Z = 25

# -----------------------------------------------------------------------------
#  Behavior config

# Run command immediately after receiving and stream new pulses, otherwise
# buffer will be prepared firstly and then command will run.
# Before enabling this feature, please make sure that board performance is
# enough for streaming pulses(faster then real time).
INSTANT_RUN = True

# If this parameter is False, error will be raised on command with velocity
# more than maximum velocity specified here. If this parameter is True,
# velocity would be decreased(proportional for all axises) to fit the maximum
# velocity.
AUTO_VELOCITY_ADJUSTMENT = True

# Automatically turn on fan when extruder is heating, boolean value.
AUTO_FAN_ON = True
