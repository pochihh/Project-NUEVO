/**
 * @file test_uart_tlv.ino
 * @brief Test sketch for UART TLV Communication (v2.0)
 *
 * Tests the TLV v2.0 protocol implementation by:
 * 1. Initializing MessageCenter (UART on Serial2)
 * 2. Calling processIncoming() / sendTelemetry() at 100 Hz
 * 3. Printing system state and liveness status at 1 Hz
 *
 * In IDLE state the firmware sends only SYS_STATUS (1 Hz).
 * After receiving SYS_CMD_START it enters RUNNING and emits full
 * telemetry on Serial2 at up to 100 Hz.
 *
 * How to Test:
 *   Debug output: USB Serial (Serial0) at DEBUG_BAUD_RATE (115200)
 *   Protocol I/O: Serial2 (pins 16 / 17, via level shifter)
 *
 *   Option A — loopback:
 *     Connect pin 16 (TX2) → pin 17 (RX2). The Arduino will receive
 *     its own outgoing packets, which resets the liveness timer.
 *
 *   Option B — PC / RPi:
 *     Send TLV-framed packets on Serial2 using the Python bridge or
 *     a terminal tool that can send raw bytes:
 *       SYS_HEARTBEAT (type=1) every 200 ms — keeps liveness alive
 *       SYS_CMD_START (type=3, cmd=1) — transitions to RUNNING
 *       SYS_CMD_STOP  (type=3, cmd=2) — returns to IDLE
 *       SYS_CMD_ESTOP (type=3, cmd=4) — emergency stop
 *       SYS_CMD_RESET (type=3, cmd=3) — reset from ESTOP/ERROR
 *
 * Verification:
 *   1. Compile success
 *   2. Debug output shows "MessageCenter initialized" and state changes
 *   3. Liveness remains "OK" when packets are flowing; "TIMEOUT" when quiet
 *   4. State transitions match commands sent
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/modules/MessageCenter.h"
#include "src/messages/TLV_Payloads.h"

// ============================================================================
// TASKS
// ============================================================================

/**
 * @brief UART communication task (100 Hz)
 *
 * processIncoming() drains the UART Rx FIFO, parses TLV frames, and routes
 * commands to the appropriate subsystem modules.
 *
 * sendTelemetry() manages its own rate table — calling it every 10 ms is
 * correct; it only transmits a given message type when its interval is due.
 */
void taskUART() {
    MessageCenter::processIncoming();
    MessageCenter::sendTelemetry();
}

/**
 * @brief Status print task (1 Hz)
 *
 * Prints current system state and liveness status to USB Serial.
 * This is debug output only; it does not affect protocol behaviour.
 */
void taskPrintStatus() {
    SystemState state = MessageCenter::getSystemState();

    DEBUG_SERIAL.print(F("[Status] State: "));
    switch (state) {
        case SYS_STATE_INIT:    DEBUG_SERIAL.print(F("INIT"));    break;
        case SYS_STATE_IDLE:    DEBUG_SERIAL.print(F("IDLE"));    break;
        case SYS_STATE_RUNNING: DEBUG_SERIAL.print(F("RUNNING")); break;
        case SYS_STATE_ERROR:   DEBUG_SERIAL.print(F("ERROR"));   break;
        case SYS_STATE_ESTOP:   DEBUG_SERIAL.print(F("ESTOP"));   break;
        default:                DEBUG_SERIAL.print(F("?"));       break;
    }

    if (MessageCenter::isHeartbeatValid()) {
        DEBUG_SERIAL.print(F("  Liveness: OK (last TLV "));
        DEBUG_SERIAL.print(MessageCenter::getTimeSinceHeartbeat());
        DEBUG_SERIAL.print(F(" ms ago)"));
    } else {
        DEBUG_SERIAL.print(F("  Liveness: TIMEOUT — actuators disabled"));
    }

    DEBUG_SERIAL.println();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000)
        ; // Wait for USB serial connection (up to 2 s)

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  UART TLV Protocol Test (v2.0)"));
    DEBUG_SERIAL.println(F("========================================"));

    Scheduler::init();
    DEBUG_SERIAL.println(F("[Setup] Scheduler initialized"));

    MessageCenter::init();
    DEBUG_SERIAL.println(F("[Setup] MessageCenter initialized"));
    DEBUG_SERIAL.print(F("  - Serial2 @ "));
    DEBUG_SERIAL.print(RPI_BAUD_RATE);
    DEBUG_SERIAL.println(F(" baud"));
    DEBUG_SERIAL.print(F("  - Device ID: 0x"));
    DEBUG_SERIAL.println(DEVICE_ID, HEX);
    DEBUG_SERIAL.println(F("  - Initial state: IDLE"));

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("--- Protocol state machine ---"));
    DEBUG_SERIAL.println(F("  IDLE    : SYS_STATUS every 1 s"));
    DEBUG_SERIAL.println(F("  RUNNING : full telemetry up to 100 Hz"));
    DEBUG_SERIAL.println(F("  Liveness timeout: 500 ms without any TLV"));
    DEBUG_SERIAL.println(F("  SYS_CMD_START (type=3, cmd=1) -> RUNNING"));
    DEBUG_SERIAL.println(F("  SYS_CMD_STOP  (type=3, cmd=2) -> IDLE"));
    DEBUG_SERIAL.println(F("  SYS_CMD_ESTOP (type=3, cmd=4) -> ESTOP"));
    DEBUG_SERIAL.println(F("  SYS_CMD_RESET (type=3, cmd=3) -> IDLE"));

    // Register tasks
    Scheduler::registerTask(taskUART,        10,   1);  // 10 ms period = 100 Hz
    Scheduler::registerTask(taskPrintStatus, 1000, 2);  // 1000 ms period = 1 Hz

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("[Setup] Tasks registered — starting..."));
    DEBUG_SERIAL.println(F("========================================"));
}

// ============================================================================
// LOOP
// ============================================================================

void loop() { Scheduler::tick(); }
