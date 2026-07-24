#include <Arduino.h>
#include <math.h>

// ============================================================
// Hardware pins
// ============================================================

// Pololu encoder
constexpr uint8_t ENCODER_A_PIN = 32;
constexpr uint8_t ENCODER_B_PIN = 33;

// BD65496MUV motor driver
constexpr uint8_t MOTOR_PWM_PIN = 27;  // INA
constexpr uint8_t MOTOR_DIR_PIN = 26;  // INB

// Driver PWM/MODE pin remains connected directly to ESP32 3V3.

// ============================================================
// Encoder configuration
// ============================================================

// Pololu magnetic encoder: 20 counts per motor-shaft revolution
// Gearbox: exact 25:1
//
// 20 × 25 = 500 counts per gearbox output-shaft revolution
constexpr float COUNTS_PER_OUTPUT_REV = 500.0f;

// ============================================================
// PWM configuration
// ============================================================

constexpr uint8_t PWM_CHANNEL = 0;
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;

constexpr float PWM_MAX = 255.0f;
constexpr float PWM_MIN = -255.0f;

// ============================================================
// PI controller configuration
// ============================================================

// 100 Hz control loop
constexpr uint32_t CONTROL_PERIOD_US = 10000;
constexpr float CONTROL_PERIOD_S = 0.010f;

// Initial gains.
// These are starting values and need experimental tuning, specially on the ground with the wheel
float kp = 1.70f;
float ki = 3.5f;

// Filter coefficient:
// 1.0 = no filtering
// smaller = more filtering
constexpr float SPEED_FILTER_ALPHA = 0.25f;

// Integral anti-windup limits
constexpr float INTEGRAL_MIN = -255.0f;
constexpr float INTEGRAL_MAX = 255.0f;

// Target values below this are treated as zero.
constexpr float RPM_DEADBAND = 0.5f;

// ============================================================
// Encoder variables
// ============================================================

volatile int64_t encoderCount = 0;
volatile uint8_t previousEncoderState = 0;

// Full quadrature lookup table.
//
// Index:
// previous AB state in bits 3:2
// current  AB state in bits 1:0
//
// Valid transitions produce +1 or -1.
// Invalid/no-change transitions produce 0.
constexpr int8_t QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// ============================================================
// Controller state
// ============================================================

float targetRPM = 0.0f;
float measuredRPM = 0.0f;
float rawRPM = 0.0f;

float integralTerm = 0.0f;
float controllerOutput = 0.0f;
float speedErrorRPM = 0.0f;

int64_t previousControlEncoderCount = 0;
uint32_t previousControlTimeUs = 0;

// ============================================================
// Encoder interrupt
// ============================================================

void IRAM_ATTR updateEncoder()
{
    const uint8_t encoderA =
        static_cast<uint8_t>(digitalRead(ENCODER_A_PIN));

    const uint8_t encoderB =
        static_cast<uint8_t>(digitalRead(ENCODER_B_PIN));

    const uint8_t currentEncoderState =
        static_cast<uint8_t>((encoderA << 1) | encoderB);

    const uint8_t tableIndex =
        static_cast<uint8_t>(
            (previousEncoderState << 2) |
            currentEncoderState
        );

    encoderCount += QUADRATURE_TABLE[tableIndex];

    previousEncoderState = currentEncoderState;
}

// ============================================================
// Motor output
// ============================================================

void setMotorPWM(float pwmCommand)
{
    pwmCommand = constrain(
        pwmCommand,
        PWM_MIN,
        PWM_MAX
    );

    uint32_t pwmMagnitude =
        static_cast<uint32_t>(roundf(fabsf(pwmCommand)));

    pwmMagnitude = constrain(
        pwmMagnitude,
        static_cast<uint32_t>(0),
        static_cast<uint32_t>(255)
    );

    if (pwmCommand > 0.0f) {
        digitalWrite(MOTOR_DIR_PIN, HIGH);
        ledcWrite(PWM_CHANNEL, pwmMagnitude);
    }
    else if (pwmCommand < 0.0f) {
        digitalWrite(MOTOR_DIR_PIN, LOW);
        ledcWrite(PWM_CHANNEL, pwmMagnitude);
    }
    else {
        ledcWrite(PWM_CHANNEL, 0);
    }
}

// ============================================================
// Controller reset
// ============================================================

void resetController()
{
    integralTerm = 0.0f;
    controllerOutput = 0.0f;
    speedErrorRPM = 0.0f;

    setMotorPWM(0.0f);
}

void stopMotor()
{
    targetRPM = 0.0f;
    resetController();
}

// ============================================================
// Encoder count access
// ============================================================

int64_t readEncoderCount()
{
    noInterrupts();
    const int64_t count = encoderCount;
    interrupts();

    return count;
}

void resetEncoderCount()
{
    noInterrupts();
    encoderCount = 0;
    interrupts();

    previousControlEncoderCount = 0;
}

// ============================================================
// PI speed controller
// ============================================================

void updateSpeedController(float deltaTimeSeconds)
{
    const int64_t currentEncoderCount =
        readEncoderCount();

    const int64_t deltaCounts =
        -currentEncoderCount +
        previousControlEncoderCount;

    previousControlEncoderCount =
        currentEncoderCount;

    // Encoder counts per second
    const float countsPerSecond =
        static_cast<float>(deltaCounts) /
        deltaTimeSeconds;

    // Convert encoder rate to output shaft RPM
    rawRPM =
        countsPerSecond *
        60.0f /
        COUNTS_PER_OUTPUT_REV;

    // Low-pass-filter the velocity estimate
    measuredRPM =
        SPEED_FILTER_ALPHA * rawRPM +
        (1.0f - SPEED_FILTER_ALPHA) *
        measuredRPM;

    if (fabsf(targetRPM) < RPM_DEADBAND) {
        measuredRPM =
            fabsf(measuredRPM) < RPM_DEADBAND
                ? 0.0f
                : measuredRPM;

        resetController();
        return;
    }

    speedErrorRPM =
        targetRPM - measuredRPM;

    const float proportionalTerm =
        kp * speedErrorRPM;

    // Candidate integral update
    const float candidateIntegral =
        constrain(
            integralTerm +
                ki *
                speedErrorRPM *
                deltaTimeSeconds,
            INTEGRAL_MIN,
            INTEGRAL_MAX
        );

    const float candidateOutput =
        proportionalTerm +
        candidateIntegral;

    /*
     * Conditional anti-windup:
     *
     * Update the integral when:
     * 1. the proposed output is not saturated, or
     * 2. the error would drive the actuator away from saturation.
     */
    const bool outputNotSaturated =
        candidateOutput > PWM_MIN &&
        candidateOutput < PWM_MAX;

    const bool reducingPositiveSaturation =
        candidateOutput >= PWM_MAX &&
        speedErrorRPM < 0.0f;

    const bool reducingNegativeSaturation =
        candidateOutput <= PWM_MIN &&
        speedErrorRPM > 0.0f;

    if (
        outputNotSaturated ||
        reducingPositiveSaturation ||
        reducingNegativeSaturation
    ) {
        integralTerm = candidateIntegral;
    }

    controllerOutput =
        constrain(
            proportionalTerm + integralTerm,
            PWM_MIN,
            PWM_MAX
        );

    setMotorPWM(controllerOutput);
}

// ============================================================
// Serial commands
// ============================================================

void printHelp()
{
    Serial.println();
    Serial.println("ESP32 PI motor-speed controller");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  f  : set target to +100 RPM");
    Serial.println("  r  : set target to -100 RPM");
    Serial.println("  s  : stop");
    Serial.println("  +  : increase target by 25 RPM");
    Serial.println("  -  : decrease target by 25 RPM");
    Serial.println("  p  : increase Kp by 0.05");
    Serial.println("  P  : decrease Kp by 0.05");
    Serial.println("  i  : increase Ki by 0.10");
    Serial.println("  I  : decrease Ki by 0.10");
    Serial.println("  z  : reset encoder position count");
    Serial.println("  h  : print help");
    Serial.println();
}

void processSerialCommands()
{
    while (Serial.available() > 0) {
        const char command = Serial.read();

        switch (command) {
            case 'f':
                targetRPM = 100.0f;
                integralTerm = 0.0f;

                Serial.printf(
                    "Target speed: %.1f RPM\n",
                    targetRPM
                );
                break;

            case 'r':
                targetRPM = -100.0f;
                integralTerm = 0.0f;

                Serial.printf(
                    "Target speed: %.1f RPM\n",
                    targetRPM
                );
                break;

            case 's':
                stopMotor();
                Serial.println("Motor stopped.");
                break;

            case '+':
                targetRPM += 25.0f;

                Serial.printf(
                    "Target speed: %.1f RPM\n",
                    targetRPM
                );
                break;

            case '-':
                targetRPM -= 25.0f;

                Serial.printf(
                    "Target speed: %.1f RPM\n",
                    targetRPM
                );
                break;

            case 'p':
                kp += 0.05f;

                Serial.printf(
                    "Kp: %.3f\n",
                    kp
                );
                break;

            case 'P':
                kp = fmaxf(
                    0.0f,
                    kp - 0.05f
                );

                Serial.printf(
                    "Kp: %.3f\n",
                    kp
                );
                break;

            case 'i':
                ki += 0.10f;

                Serial.printf(
                    "Ki: %.3f\n",
                    ki
                );
                break;

            case 'I':
                ki = fmaxf(
                    0.0f,
                    ki - 0.10f
                );

                Serial.printf(
                    "Ki: %.3f\n",
                    ki
                );
                break;

            case 'z':
                resetEncoderCount();
                Serial.println(
                    "Encoder count reset."
                );
                break;

            case 'h':
            case 'H':
                printHelp();
                break;

            case '\r':
            case '\n':
                break;

            default:
                Serial.printf(
                    "Unknown command: %c\n",
                    command
                );
                break;
        }
    }
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(
        ENCODER_A_PIN,
        INPUT_PULLUP
    );

    pinMode(
        ENCODER_B_PIN,
        INPUT_PULLUP
    );

    pinMode(
        MOTOR_DIR_PIN,
        OUTPUT
    );

    digitalWrite(
        MOTOR_DIR_PIN,
        LOW
    );

    // Read initial encoder state before enabling interrupts.
    const uint8_t initialA =
        static_cast<uint8_t>(
            digitalRead(ENCODER_A_PIN)
        );

    const uint8_t initialB =
        static_cast<uint8_t>(
            digitalRead(ENCODER_B_PIN)
        );

    previousEncoderState =
        static_cast<uint8_t>(
            (initialA << 1) | initialB
        );

    // Configure ESP32 PWM.
    const double actualPWMFrequency =
        ledcSetup(
            PWM_CHANNEL,
            PWM_FREQUENCY_HZ,
            PWM_RESOLUTION_BITS
        );

    if (actualPWMFrequency == 0.0) {
        Serial.println(
            "ERROR: PWM setup failed."
        );

        while (true) {
            delay(1000);
        }
    }

    ledcAttachPin(
        MOTOR_PWM_PIN,
        PWM_CHANNEL
    );

    // Full quadrature decoding:
    // interrupt on every edge of both encoder channels.
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_PIN),
        updateEncoder,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_B_PIN),
        updateEncoder,
        CHANGE
    );

    stopMotor();

    previousControlEncoderCount =
        readEncoderCount();

    previousControlTimeUs =
        micros();

    Serial.println();
    Serial.println(
        "ESP32 PI speed controller initialized."
    );

    Serial.printf(
        "PWM frequency: %.0f Hz\n",
        actualPWMFrequency
    );

    Serial.printf(
        "Encoder resolution: %.1f counts/output rev\n",
        COUNTS_PER_OUTPUT_REV
    );

    Serial.printf(
        "Kp: %.3f\n",
        kp
    );

    Serial.printf(
        "Ki: %.3f\n",
        ki
    );

    printHelp();
}

// ============================================================
// Main loop
// ============================================================

void loop()
{
    processSerialCommands();

    const uint32_t currentTimeUs =
        micros();

    const uint32_t elapsedTimeUs =
        currentTimeUs -
        previousControlTimeUs;

    if (elapsedTimeUs >= CONTROL_PERIOD_US) {
        const float elapsedTimeSeconds =
            static_cast<float>(elapsedTimeUs) /
            1000000.0f;

        previousControlTimeUs =
            currentTimeUs;

        updateSpeedController(
            elapsedTimeSeconds
        );

        // Print at approximately 5 Hz.
        static uint8_t printDivider = 0;
        printDivider++;

        if (printDivider >= 20) {
            printDivider = 0;

            const int64_t positionCount =
                readEncoderCount();

            const float outputRevolutions =
                static_cast<float>(positionCount) /
                COUNTS_PER_OUTPUT_REV;

            Serial.printf(
                "target=%7.1f RPM, "
                "measured=%7.1f RPM, "
                "raw=%7.1f RPM, "
                "error=%7.1f RPM, "
                "PWM=%7.1f, "
                "integral=%7.1f, "
                "count=%lld, "
                "revs=%.3f\n",
                targetRPM,
                measuredRPM,
                rawRPM,
                speedErrorRPM,
                controllerOutput,
                integralTerm,
                positionCount,
                outputRevolutions
            );
        }
    }
}
