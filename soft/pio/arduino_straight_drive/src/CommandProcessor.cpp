// Command parsing separated from main control
#include "CommandProcessor.h"
#include "IMUOrientation.h"
#include "CarState.h"
#include "IMUOrientation.h"
#include "RadioModule.h"
#include <RadioProtocol.h>

// Mirror default macro definitions if not provided by build flags before inclusion
#ifndef MOVEMENT_COMMAND_DURATION_MS
#define MOVEMENT_COMMAND_DURATION_MS 3000UL
#endif
#ifndef MANUAL_COMMAND_DURATION_MS
#define MANUAL_COMMAND_DURATION_MS 2000UL
#endif
#ifndef MIN_EFFECTIVE_DUTY
#define MIN_EFFECTIVE_DUTY 0.18f
#endif

static inline float f_abs(float v){ return v<0?-v:v; }

void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  f/w  forward   (PID, fixed duration)"));
    Serial.println(F("  b/s  backward  (PID, fixed duration)"));
    Serial.println(F("  a    turn left 90"));
    Serial.println(F("  d    turn right 90"));
    Serial.println(F("  x    stop"));
    Serial.println(F("  z    zero yaw & target"));
    Serial.println(F("  yr   toggle always-reset vs adaptive yaw reset"));
    Serial.println(F("  zg   set targetYaw to current yaw (no zero)"));
    Serial.println(F("  p    print PID gains"));
    Serial.println(F("  P#/I#/D# set gain (e.g. P1.2)"));
    Serial.println(F("  dur  print configured durations"));
    Serial.println(F("  rem  remaining ms in active movement"));
    Serial.println(F("  stats radio counters"));
    Serial.println(F("  rdiag radio printDetails"));
    Serial.println(F("  rtry attempt radio reinit"));
    Serial.println(F("  rmin/rnorm minimal/normal radio mode"));
    Serial.println(F("  ping send ping command packet"));
    Serial.println(F("  lf/lr/ls rf/rr/rs manual single-wheel control (timeout)"));
    Serial.println(F("  t    motor test pattern"));
}

void commandSetLastCmd(const String &tok) {
    // Currently unused outside; placeholder for telemetry tagging expansion.
    (void)tok;
}

void commandProcessToken(const String &tok) {
    if (tok.equalsIgnoreCase("lf")) {
        if (autoResetYawOnMove) resetYawAll(F("lf"));
        manualOverride = true; mode = Mode::Idle; movementExpireMs = 0; manualExpireMs = millis()+MANUAL_COMMAND_DURATION_MS;
        drive(0.5f, lastCmdRight); lastCmdLeft = 0.5f; Serial.println(F("Manual left forward 0.5"));
    } else if (tok.equalsIgnoreCase("lr")) {
        if (autoResetYawOnMove) resetYawAll(F("lr"));
        manualOverride = true; mode = Mode::Idle; movementExpireMs = 0; manualExpireMs = millis()+MANUAL_COMMAND_DURATION_MS;
        drive(-0.5f, lastCmdRight); lastCmdLeft = -0.5f; Serial.println(F("Manual left reverse -0.5"));
    } else if (tok.equalsIgnoreCase("ls")) {
        if (autoResetYawOnMove) resetYawAll(F("ls"));
        manualOverride = true; mode = Mode::Idle; movementExpireMs = 0; manualExpireMs = millis()+MANUAL_COMMAND_DURATION_MS;
        drive(0.0f, lastCmdRight); lastCmdLeft = 0.0f; Serial.println(F("Manual left stop"));
    } else if (tok.equalsIgnoreCase("rf")) {
        if (autoResetYawOnMove) resetYawAll(F("rf"));
        manualOverride = true; mode = Mode::Idle; movementExpireMs = 0; manualExpireMs = millis()+MANUAL_COMMAND_DURATION_MS;
        drive(lastCmdLeft, 0.5f); lastCmdRight = 0.5f; Serial.println(F("Manual right forward 0.5"));
    } else if (tok.equalsIgnoreCase("rr")) {
        if (autoResetYawOnMove) resetYawAll(F("rr"));
        manualOverride = true; mode = Mode::Idle; movementExpireMs = 0; manualExpireMs = millis()+MANUAL_COMMAND_DURATION_MS;
        drive(lastCmdLeft, -0.5f); lastCmdRight = -0.5f; Serial.println(F("Manual right reverse -0.5"));
    } else if (tok.equalsIgnoreCase("rs")) {
        if (autoResetYawOnMove) resetYawAll(F("rs"));
        manualOverride = true; mode = Mode::Idle; movementExpireMs = 0; manualExpireMs = millis()+MANUAL_COMMAND_DURATION_MS;
        drive(lastCmdLeft, 0.0f); lastCmdRight = 0.0f; Serial.println(F("Manual right stop"));
    } else if (tok.equalsIgnoreCase("f") || tok.equalsIgnoreCase("w")) {
        if (mode == Mode::Forward && (millis() - lastMoveStartMs) < 400) { Serial.println(F("(Duplicate forward ignored)")); return; }
        if (autoResetYawOnMove) { resetYawAll(F("forward")); } else { float cur=imu.yaw(); if (f_abs(cur)>autoResetThresholdDeg) resetYawAll(F("forward(auto)")); else targetYaw=cur; }
        mode=Mode::Forward; turningToAngle=false; integral=0; prevErr=0; lastMoveStartMs=millis(); movementExpireMs=lastMoveStartMs+MOVEMENT_COMMAND_DURATION_MS; Serial.print(F("Forward (yaw reset, ")); Serial.print(MOVEMENT_COMMAND_DURATION_MS/1000); Serial.println(F("s)")); manualOverride=false;
    } else if (tok.equalsIgnoreCase("b") || tok.equalsIgnoreCase("s")) {
        if (mode == Mode::Backward && (millis() - lastMoveStartMs) < 400) { Serial.println(F("(Duplicate backward ignored)")); return; }
        if (autoResetYawOnMove) { resetYawAll(F("backward")); } else { float cur=imu.yaw(); if (f_abs(cur)>autoResetThresholdDeg) resetYawAll(F("backward(auto)")); else targetYaw=cur; }
        mode=Mode::Backward; turningToAngle=false; integral=0; prevErr=0; lastMoveStartMs=millis(); movementExpireMs=lastMoveStartMs+MOVEMENT_COMMAND_DURATION_MS; Serial.print(F("Backward (yaw reset, ")); Serial.print(MOVEMENT_COMMAND_DURATION_MS/1000); Serial.println(F("s)")); manualOverride=false;
    } else if (tok.equalsIgnoreCase("x")) {
        mode=Mode::Idle; drive(0,0); lastCmdLeft=0; lastCmdRight=0; movementExpireMs=0; Serial.println(F("Stop")); manualOverride=false;
    } else if (tok.equalsIgnoreCase("z")) {
        imu.zeroYaw(); targetYaw=0; Serial.println(F("Yaw zeroed"));
    } else if (tok.equalsIgnoreCase("a")) {
        if (mode == Mode::Turn && turnDir == -1 && (millis()-lastMoveStartMs)<400) { Serial.println(F("(Duplicate left turn ignored)")); return; }
        if (autoResetYawOnMove) resetYawAll(F("left turn")); else { float cur=imu.yaw(); if (f_abs(cur)>autoResetThresholdDeg) resetYawAll(F("left turn(auto)")); }
        mode=Mode::Turn; turningToAngle=true; targetYaw=-90.0f; turnDir=-1; lastMoveStartMs=millis(); movementExpireMs=lastMoveStartMs+MOVEMENT_COMMAND_DURATION_MS; Serial.print(F("Turn left to -90 (yaw reset, <=")); Serial.print(MOVEMENT_COMMAND_DURATION_MS/1000); Serial.println(F("s)")); manualOverride=false;
    } else if (tok.equalsIgnoreCase("d")) {
        if (mode == Mode::Turn && turnDir == +1 && (millis()-lastMoveStartMs)<400) { Serial.println(F("(Duplicate right turn ignored)")); return; }
        if (autoResetYawOnMove) resetYawAll(F("right turn")); else { float cur=imu.yaw(); if (f_abs(cur)>autoResetThresholdDeg) resetYawAll(F("right turn(auto)")); }
        mode=Mode::Turn; turningToAngle=true; targetYaw=90.0f; turnDir=+1; lastMoveStartMs=millis(); movementExpireMs=lastMoveStartMs+MOVEMENT_COMMAND_DURATION_MS; Serial.print(F("Turn right to +90 (yaw reset, <=")); Serial.print(MOVEMENT_COMMAND_DURATION_MS/1000); Serial.println(F("s)")); manualOverride=false;
    } else if (tok.equalsIgnoreCase("yr")) {
        autoResetYawOnMove = !autoResetYawOnMove; if (baselineSet) baselineYaw=0; Serial.print(F("Auto yaw reset on move: ")); Serial.println(autoResetYawOnMove?F("ON"):F("OFF (adaptive)"));
    } else if (tok.equalsIgnoreCase("zg")) {
        targetYaw = imu.yaw(); integral=0; prevErr=0; Serial.print(F("Target locked to current yaw: ")); Serial.println(targetYaw,1);
    } else if (tok.equalsIgnoreCase("p")) {
        Serial.print(F("Kp=")); Serial.print(Kp,3); Serial.print(F(" Ki=")); Serial.print(Ki,4); Serial.print(F(" Kd=")); Serial.println(Kd,3);
    } else if (tok.equalsIgnoreCase("dur")) {
        Serial.print(F("Durations movement=")); Serial.print(MOVEMENT_COMMAND_DURATION_MS); Serial.print(F("ms manual=")); Serial.print(MANUAL_COMMAND_DURATION_MS); Serial.print(F("ms baseSpeed=")); Serial.print(baseSpeed,2); Serial.print(F(" minDuty=")); Serial.print(MIN_EFFECTIVE_DUTY,2); Serial.println();
    } else if (tok.equalsIgnoreCase("rem")) {
        unsigned long now=millis(); if (movementExpireMs && now < movementExpireMs) { Serial.print(F("Remaining movement ms: ")); Serial.println(movementExpireMs-now); } else { Serial.println(F("No active movement")); }
    } else if (tok.equalsIgnoreCase("stats")) {
        Serial.print(F("RADIO stats cmdRx=")); Serial.print(radioGetCmdRx()); Serial.print(F(" tlmOk=")); Serial.print(radioGetTlmOk()); Serial.print(F(" tlmFail=")); Serial.println(radioGetTlmFail());
    } else if (tok.equalsIgnoreCase("rdiag")) {
        radioPrintDetails();
    } else if (tok.equalsIgnoreCase("rtry")) {
        radioTryReinit();
    } else if (tok.equalsIgnoreCase("rmin")) {
        radioSetMinimal(true); Serial.println(F("Minimal radio mode ON"));
    } else if (tok.equalsIgnoreCase("rnorm")) {
        radioSetMinimal(false); Serial.println(F("Normal radio mode ON"));
    } else if (tok.equalsIgnoreCase("ping")) {
        bool ok=radioSendPing(); Serial.print(F("PING send:")); Serial.println(ok?F("ok"):F("fail"));
    } else if (tok.equalsIgnoreCase("h")) {
        printHelp();
    } else if (tok.length()>1 && (tok.charAt(0)=='P'||tok.charAt(0)=='I'||tok.charAt(0)=='D')) {
        char c=tok.charAt(0); float v=tok.substring(1).toFloat(); if (c=='P') Kp=v; else if (c=='I') Ki=v; else if (c=='D') Kd=v; Serial.print(F("Set ")); Serial.print(c); Serial.print(F(" to ")); Serial.println(v,4);
    } else if (tok.equalsIgnoreCase("t")) {
        Serial.println(F("Motor test start")); drive(0.4f,0.4f); delay(800); drive(-0.4f,-0.4f); delay(800); drive(0.4f,-0.4f); delay(600); drive(-0.4f,0.4f); delay(600); drive(0,0); lastCmdLeft=0; lastCmdRight=0; mode=Mode::Idle; movementExpireMs=0; Serial.println(F("Motor test done"));
    } else {
        Serial.print(F("Unknown cmd: ")); Serial.println(tok);
    }
}

void commandProcessSerial() {
    while (Serial.available()) {
        String tok = Serial.readStringUntil('\n');
        tok.trim();
        if (tok.length()==0) continue;
        Serial.print(F("CMD> ")); Serial.println(tok);
        commandSetLastCmd(tok);
        commandProcessToken(tok);
    }
}
