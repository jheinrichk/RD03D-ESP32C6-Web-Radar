/*
  ESP32-C6 + Ai-Thinker RD-03D Filtered Live Web Radar

  Current version:
    - RD-03D only. RF transceiver code intentionally omitted.
    - 9 meter, 120 degree forward sector browser radar display.
    - Radar display shows filtered confirmed targets only.
    - Pedestrian, vehicle and unknown moving classification.
    - Detection history with saved long movement trails.
    - Full Clear Log behavior for all detection history stored in RAM.
    - Logging only records completed movement paths of at least 2 meters.
    - More aggressive filtering for foliage, breeze and micro movement.
    - Manual radar reset and automatic stall / frozen-content recovery.
    - Centered stable UI layout with reserved detection information space.

  Wiring:
    RD-03D VCC -> 5V
    RD-03D GND -> ESP32-C6 GND
    RD-03D TX  -> ESP32-C6 GPIO7
    RD-03D RX  -> ESP32-C6 GPIO6

  Serial Monitor:
    115200 baud

  RD-03D UART:
    UART1 @ 256000 8N1 on GPIO7 RX and GPIO6 TX

  WiFi Credentials:
    For local use, copy secrets.example.h to secrets.h and edit that file.
    secrets.h is intentionally ignored by git so real WiFi credentials are not published.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <math.h>

// ================== WIFI CREDENTIALS ==================
// For local use, copy secrets.example.h to secrets.h and edit that file.
// secrets.h is intentionally ignored by git so real WiFi credentials are not published.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  const char* wifi_ssid = "YOUR_WIFI_SSID";
  const char* wifi_password = "YOUR_WIFI_PASSWORD";
#endif

// ================== RD03D PINS ==================
static const int RD03D_RX_PIN = 7;       // ESP32 RX <- RD03D TX
static const int RD03D_TX_PIN = 6;       // ESP32 TX -> RD03D RX
static const uint32_t RD03D_BAUD = 256000;

HardwareSerial RadarSerial(1);

// ================== WEB SERVER ==================
AsyncWebServer server(80);

// ================== DETECTION TUNING ==================
static const uint16_t MAX_DETECTION_MM = 9000;
static const uint16_t MIN_DETECTION_MM = 300;

// Bush / wind / micro movement filtering
static const int16_t MICRO_SPEED_MAX_CMS = 12;
static const uint16_t MICRO_DIST_JITTER_MAX_MM = 420;
static const uint16_t MICRO_X_JITTER_MAX_MM = 520;
static const uint8_t MICRO_REPEAT_LIMIT = 3;

// Still target clearing
static const int16_t STILL_SPEED_CLEAR_CMS = 5;
static const uint8_t STATIONARY_CLEAR_THRESHOLD = 3;

// Live radar display qualification
// A target is not plotted until it has passed the same filtering logic used for classification.
// UNKNOWN is allowed only after several frames of measurable movement, not from raw single-frame noise.
static const int16_t UNKNOWN_MIN_SPEED_CMS = 12;
static const uint16_t UNKNOWN_MIN_DIST_JITTER_MM = 240;
static const uint16_t UNKNOWN_MIN_X_JITTER_MM = 260;
static const uint8_t UNKNOWN_HIT_THRESHOLD = 4;

// Pedestrian
static const uint16_t PED_MIN_MM = 300;
static const uint16_t PED_MAX_MM = 9000;
static const int16_t PED_MIN_SPEED_CMS = 4;
static const int16_t PED_MAX_SPEED_CMS = 220;
static const uint8_t PED_HIT_THRESHOLD = 3;

// Vehicle
static const uint16_t VEH_MIN_MM = 700;
static const uint16_t VEH_MAX_MM = 9000;
static const int16_t VEH_MIN_SPEED_CMS = 18;
static const int16_t VEH_FAST_SPEED_CMS = 50;
static const uint8_t VEH_HIT_THRESHOLD = 2;

// Lateral acceptance zones
static const uint16_t VEHICLE_LATERAL_ZONE_MM = 5000;
static const uint16_t PEDESTRIAN_LATERAL_ZONE_MM = 3200;

// Clear thresholds
static const uint8_t MISS_CLEAR_THRESHOLD = 8;

// ================== RADAR RECOVERY ==================
uint32_t lastRadarRecoveryMillis = 0;
uint32_t radarRecoveryCount = 0;
uint32_t radarFrozenRecoveryCount = 0;

static const uint32_t RADAR_FRAME_STALL_RESET_MS = 3500;
static const uint32_t RADAR_FRAME_RESET_COOLDOWN_MS = 6000;

static const uint32_t RADAR_CONTENT_STALL_RESET_MS = 12000;
static const uint32_t RADAR_CONTENT_RESET_COOLDOWN_MS = 15000;

// ================== STATS ==================
uint32_t totalUartBytes = 0;
uint32_t lastSecondUartBytes = 0;
uint32_t totalBadFrames = 0;
uint32_t lastFrameMillis = 0;
uint32_t lastStatusMillis = 0;

// ================== LOGGING ==================
bool loggingEnabled = true;
uint32_t nextLogId = 1;
static const uint8_t MAX_LOG_ITEMS = 48;
static const uint8_t MAX_TRAIL_POINTS = 80;
static const uint16_t MIN_TRAIL_SAMPLE_STEP_MM = 120;
static const uint16_t PIN_TRAIL_THRESHOLD_MM = 4000;
static const uint16_t MIN_LOG_PATH_MM = 2000;
static const uint32_t TRACK_END_TIMEOUT_MS = 1200;

// ================== CLASSIFICATION ENUM ==================
enum TargetClassification : uint8_t {
  CLASS_NONE = 0,
  CLASS_PEDESTRIAN = 1,
  CLASS_VEHICLE = 2,
  CLASS_UNKNOWN_MOVING = 3
};

// ================== TARGET DATA ==================
struct Target {
  bool active = false;
  bool pedestrianLikely = false;
  bool vehicleLikely = false;
  uint8_t classification = CLASS_NONE;
  float x = 0;
  float y = 0;
  float speed = 0;
  float dist = 0;
};

Target webTargets[3];

// ================== FILTER STATE ==================
uint8_t pedestrianHits[3] = {0, 0, 0};
uint8_t vehicleHits[3] = {0, 0, 0};
uint8_t unknownHits[3] = {0, 0, 0};
uint8_t misses[3] = {0, 0, 0};
uint8_t stationaryHits[3] = {0, 0, 0};

int16_t lastXmm[3] = {0, 0, 0};
uint16_t lastDistMm[3] = {0, 0, 0};
uint8_t microMovementHits[3] = {0, 0, 0};

// ================== RAW CONTENT SNAPSHOT ==================
struct RawSnapshot {
  bool valid = false;
  int16_t x_mm = 0;
  int16_t y_mm = 0;
  int16_t speed_cms = 0;
  uint16_t dist_mm = 0;
};

RawSnapshot lastRawSnapshot[3];
uint32_t lastRadarContentChangeMillis = 0;

// ================== TRAIL / LOG STRUCTURES ==================
struct TrailPoint {
  int16_t x_mm = 0;
  int16_t y_mm = 0;
};

struct EventTracker {
  bool active = false;
  uint8_t slotId = 0;
  uint8_t classification = CLASS_NONE;
  uint32_t startMs = 0;
  uint32_t lastSeenMs = 0;
  uint16_t minDistMm = 65535;
  int16_t peakSpeedCms = 0;
  uint32_t pathLenMm = 0;
  int16_t firstX = 0;
  int16_t firstY = 0;
  int16_t lastX = 0;
  int16_t lastY = 0;
  int16_t minX = 0;
  int16_t maxX = 0;
  int16_t minY = 0;
  int16_t maxY = 0;
  bool havePoint = false;
  uint8_t trailCount = 0;
  TrailPoint trail[MAX_TRAIL_POINTS];
};

struct LogItem {
  bool valid = false;
  uint32_t id = 0;
  uint32_t startMs = 0;
  uint32_t endMs = 0;
  uint8_t slotId = 0;
  uint8_t classification = CLASS_NONE;
  uint16_t minDistMm = 0;
  int16_t peakSpeedCms = 0;
  uint32_t pathLenMm = 0;
  uint16_t netDisplacementMm = 0;
  uint16_t maxAxisSpanMm = 0;
  bool pinnedTrail = false;
  uint8_t trailCount = 0;
  TrailPoint trail[MAX_TRAIL_POINTS];
};

EventTracker trackers[3];
LogItem logItems[MAX_LOG_ITEMS];

// ================== RD03D READER ==================
class RD03DReader {
public:
  struct RD03DTarget {
    bool valid = false;
    int16_t x_mm = 0;
    int16_t y_mm = 0;
    int16_t speed_cms = 0;
    uint16_t dist_mm = 0;
  };

  bool begin(HardwareSerial& s) {
    ser = &s;
    ser->begin(RD03D_BAUD, SERIAL_8N1, RD03D_RX_PIN, RD03D_TX_PIN);
    reset();
    return true;
  }

  void initMultiTarget() {
    if (!ser) return;
    while (ser->available()) ser->read();
    delay(100);
    ser->write(MULTI_TARGET_CMD, sizeof(MULTI_TARGET_CMD));
    ser->flush();
    Serial.println("RD03D multi target command sent");
    delay(300);
  }

  bool poll(uint32_t* bytesReadThisCall = nullptr) {
    if (!ser) return false;
    bool gotFrame = false;

    while (ser->available() > 0) {
      uint8_t b = (uint8_t)ser->read();
      if (bytesReadThisCall) (*bytesReadThisCall)++;
      if (consumeByte(b)) {
        decodeFrame();
        gotFrame = true;
      }
    }

    return gotFrame;
  }

  const RD03DTarget& target(uint8_t i) const { return tg[i]; }

private:
  HardwareSerial* ser = nullptr;
  static constexpr uint8_t FRAME_LEN = 30;
  uint8_t buf[FRAME_LEN];
  uint8_t idx = 0;
  RD03DTarget tg[3];

  const uint8_t MULTI_TARGET_CMD[12] = {
    0xFD, 0xFC, 0xFB, 0xFA,
    0x02, 0x00,
    0x90, 0x00,
    0x04, 0x03, 0x02, 0x01
  };

  void reset() {
    idx = 0;
    memset(buf, 0, sizeof(buf));
  }

  static int16_t decodeSigned15(uint16_t raw) {
    int16_t mag = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? mag : -mag;
  }

  bool consumeByte(uint8_t b) {
    switch (idx) {
      case 0:
        if (b == 0xAA) buf[idx++] = b;
        return false;

      case 1:
        if (b == 0xFF) {
          buf[idx++] = b;
        } else {
          idx = (b == 0xAA) ? 1 : 0;
          if (idx) buf[0] = 0xAA;
        }
        return false;

      case 2:
        if (b == 0x03) buf[idx++] = b;
        else idx = 0;
        return false;

      case 3:
        if (b == 0x00) buf[idx++] = b;
        else idx = 0;
        return false;

      default:
        buf[idx++] = b;
        if (idx >= FRAME_LEN) {
          bool ok = (buf[FRAME_LEN - 2] == 0x55) && (buf[FRAME_LEN - 1] == 0xCC);
          idx = 0;
          if (ok) {
            lastFrameMillis = millis();
            return true;
          } else {
            totalBadFrames++;
            return false;
          }
        }
        return false;
    }
  }

  void decodeFrame() {
    for (int i = 0; i < 3; i++) {
      int base = 4 + i * 8;
      uint16_t x_raw = (uint16_t)buf[base]   | ((uint16_t)buf[base + 1] << 8);
      uint16_t y_raw = (uint16_t)buf[base+2] | ((uint16_t)buf[base + 3] << 8);
      uint16_t s_raw = (uint16_t)buf[base+4] | ((uint16_t)buf[base + 5] << 8);
      uint16_t d_raw = (uint16_t)buf[base+6] | ((uint16_t)buf[base + 7] << 8);

      RD03DTarget& t = tg[i];
      t.x_mm = decodeSigned15(x_raw);
      t.y_mm = decodeSigned15(y_raw);
      t.speed_cms = decodeSigned15(s_raw);
      t.dist_mm = d_raw;
      t.valid = (d_raw != 0) || (x_raw != 0) || (y_raw != 0) || (s_raw != 0);
    }
  }
};

RD03DReader rd;

// ================== HELPERS ==================
uint16_t distanceMm(int32_t dx, int32_t dy) {
  float v = sqrtf((float)(dx * dx) + (float)(dy * dy));
  if (v < 0.0f) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)(v + 0.5f);
}

uint8_t classPriorityValue(uint8_t c) {
  switch (c) {
    case CLASS_VEHICLE: return 3;
    case CLASS_PEDESTRIAN: return 2;
    case CLASS_UNKNOWN_MOVING: return 1;
    default: return 0;
  }
}

String classToText(uint8_t c) {
  switch (c) {
    case CLASS_PEDESTRIAN: return "PEDESTRIAN";
    case CLASS_VEHICLE: return "VEHICLE";
    case CLASS_UNKNOWN_MOVING: return "UNKNOWN";
    default: return "NONE";
  }
}

void clearWebTarget(uint8_t i) {
  webTargets[i].active = false;
  webTargets[i].pedestrianLikely = false;
  webTargets[i].vehicleLikely = false;
  webTargets[i].classification = CLASS_NONE;
  webTargets[i].x = 0;
  webTargets[i].y = 0;
  webTargets[i].speed = 0;
  webTargets[i].dist = 0;
}

void clearAllTargetsAndState() {
  for (int i = 0; i < 3; i++) {
    clearWebTarget(i);
    pedestrianHits[i] = 0;
    vehicleHits[i] = 0;
    unknownHits[i] = 0;
    misses[i] = 0;
    stationaryHits[i] = 0;
    lastXmm[i] = 0;
    lastDistMm[i] = 0;
    microMovementHits[i] = 0;
  }
}

void resetTracker(EventTracker &tr) {
  tr = EventTracker();
}

bool radarContentChanged() {
  bool changed = false;

  for (int i = 0; i < 3; i++) {
    const auto& t = rd.target(i);

    if (t.valid != lastRawSnapshot[i].valid ||
        t.x_mm != lastRawSnapshot[i].x_mm ||
        t.y_mm != lastRawSnapshot[i].y_mm ||
        t.speed_cms != lastRawSnapshot[i].speed_cms ||
        t.dist_mm != lastRawSnapshot[i].dist_mm) {
      changed = true;
    }

    lastRawSnapshot[i].valid = t.valid;
    lastRawSnapshot[i].x_mm = t.x_mm;
    lastRawSnapshot[i].y_mm = t.y_mm;
    lastRawSnapshot[i].speed_cms = t.speed_cms;
    lastRawSnapshot[i].dist_mm = t.dist_mm;
  }

  return changed;
}

bool radarHasPotentialMotion() {
  for (int i = 0; i < 3; i++) {
    const auto& t = rd.target(i);
    if (t.valid &&
        t.dist_mm >= MIN_DETECTION_MM &&
        t.dist_mm <= MAX_DETECTION_MM &&
        abs(t.speed_cms) > STILL_SPEED_CLEAR_CMS) {
      return true;
    }
  }
  return false;
}

int findLogReplacementIndex(bool incomingPinned) {
  for (int i = 0; i < MAX_LOG_ITEMS; i++) {
    if (!logItems[i].valid) return i;
  }

  int oldestNonPinned = -1;
  uint32_t oldestId = 0xFFFFFFFF;

  for (int i = 0; i < MAX_LOG_ITEMS; i++) {
    if (logItems[i].valid && !logItems[i].pinnedTrail) {
      if (logItems[i].id < oldestId) {
        oldestId = logItems[i].id;
        oldestNonPinned = i;
      }
    }
  }

  if (oldestNonPinned >= 0) return oldestNonPinned;
  return -1;
}

void appendLogItemFromTracker(const EventTracker &tr) {
  if (!loggingEnabled) return;
  if (!tr.active) return;
  if (tr.classification == CLASS_NONE) return;
  if (tr.peakSpeedCms == 0 && tr.pathLenMm == 0) return;

  uint16_t netDisp = distanceMm((int32_t)tr.lastX - tr.firstX, (int32_t)tr.lastY - tr.firstY);
  uint16_t axisSpanX = (uint16_t)abs((int32_t)tr.maxX - tr.minX);
  uint16_t axisSpanY = (uint16_t)abs((int32_t)tr.maxY - tr.minY);
  uint16_t maxAxisSpan = max(axisSpanX, axisSpanY);

  if (tr.pathLenMm < MIN_LOG_PATH_MM) {
    return;
  }

  bool pinTrail =
    (tr.pathLenMm >= PIN_TRAIL_THRESHOLD_MM) ||
    (netDisp >= PIN_TRAIL_THRESHOLD_MM) ||
    (maxAxisSpan >= PIN_TRAIL_THRESHOLD_MM);

  int idx = findLogReplacementIndex(pinTrail);
  if (idx < 0) {
    Serial.println("Log storage full with pinned trail items only. New log dropped.");
    return;
  }

  LogItem &li = logItems[idx];
  li = LogItem();
  li.valid = true;
  li.id = nextLogId++;
  li.startMs = tr.startMs;
  li.endMs = tr.lastSeenMs;
  li.slotId = tr.slotId;
  li.classification = tr.classification;
  li.minDistMm = (tr.minDistMm == 65535) ? 0 : tr.minDistMm;
  li.peakSpeedCms = tr.peakSpeedCms;
  li.pathLenMm = tr.pathLenMm;
  li.netDisplacementMm = netDisp;
  li.maxAxisSpanMm = maxAxisSpan;
  li.pinnedTrail = pinTrail;

  if (pinTrail) {
    li.trailCount = tr.trailCount;
    for (int i = 0; i < tr.trailCount; i++) {
      li.trail[i] = tr.trail[i];
    }
  }
}

void updateTrackerPoint(EventTracker &tr, int16_t x_mm, int16_t y_mm) {
  if (!tr.havePoint) {
    tr.firstX = x_mm;
    tr.firstY = y_mm;
    tr.lastX = x_mm;
    tr.lastY = y_mm;
    tr.minX = x_mm;
    tr.maxX = x_mm;
    tr.minY = y_mm;
    tr.maxY = y_mm;
    tr.havePoint = true;

    tr.trail[0].x_mm = x_mm;
    tr.trail[0].y_mm = y_mm;
    tr.trailCount = 1;
    return;
  }

  uint16_t step = distanceMm((int32_t)x_mm - tr.lastX, (int32_t)y_mm - tr.lastY);
  tr.pathLenMm += step;
  tr.lastX = x_mm;
  tr.lastY = y_mm;

  if (x_mm < tr.minX) tr.minX = x_mm;
  if (x_mm > tr.maxX) tr.maxX = x_mm;
  if (y_mm < tr.minY) tr.minY = y_mm;
  if (y_mm > tr.maxY) tr.maxY = y_mm;

  if (tr.trailCount > 0 && tr.trailCount < MAX_TRAIL_POINTS) {
    TrailPoint &lastSaved = tr.trail[tr.trailCount - 1];
    uint16_t savedStep = distanceMm((int32_t)x_mm - lastSaved.x_mm, (int32_t)y_mm - lastSaved.y_mm);
    if (savedStep >= MIN_TRAIL_SAMPLE_STEP_MM) {
      tr.trail[tr.trailCount].x_mm = x_mm;
      tr.trail[tr.trailCount].y_mm = y_mm;
      tr.trailCount++;
    }
  }
}

void updateEventTrackers() {
  uint32_t now = millis();

  for (int i = 0; i < 3; i++) {
    const Target &t = webTargets[i];

    if (t.active) {
      EventTracker &tr = trackers[i];

      if (!tr.active) {
        tr.active = true;
        tr.slotId = (uint8_t)(i + 1);
        tr.classification = t.classification;
        tr.startMs = now;
        tr.lastSeenMs = now;
        tr.minDistMm = (uint16_t)t.dist;
        tr.peakSpeedCms = abs((int16_t)t.speed);
        updateTrackerPoint(tr, (int16_t)t.x, (int16_t)t.y);
      } else {
        tr.lastSeenMs = now;
        if ((uint16_t)t.dist < tr.minDistMm) tr.minDistMm = (uint16_t)t.dist;
        if (abs((int16_t)t.speed) > tr.peakSpeedCms) tr.peakSpeedCms = abs((int16_t)t.speed);

        if (classPriorityValue(t.classification) > classPriorityValue(tr.classification)) {
          tr.classification = t.classification;
        }

        updateTrackerPoint(tr, (int16_t)t.x, (int16_t)t.y);
      }
    } else {
      EventTracker &tr = trackers[i];
      if (tr.active && (now - tr.lastSeenMs) > TRACK_END_TIMEOUT_MS) {
        appendLogItemFromTracker(tr);
        resetTracker(tr);
      }
    }
  }
}

void finalizeExpiredTrackers() {
  uint32_t now = millis();

  for (int i = 0; i < 3; i++) {
    EventTracker &tr = trackers[i];
    if (tr.active && (now - tr.lastSeenMs) > TRACK_END_TIMEOUT_MS) {
      appendLogItemFromTracker(tr);
      resetTracker(tr);
    }
  }
}

void clearAllLogs() {
  for (int i = 0; i < MAX_LOG_ITEMS; i++) {
    logItems[i] = LogItem();
  }

  for (int i = 0; i < 3; i++) {
    resetTracker(trackers[i]);
  }

  nextLogId = 1;
}

void recoverRadarSensor() {
  Serial.println("Radar recovery reset starting");

  clearAllTargetsAndState();

  while (RadarSerial.available()) {
    RadarSerial.read();
  }

  RadarSerial.end();
  delay(200);

  rd.begin(RadarSerial);
  delay(150);
  rd.initMultiTarget();
  delay(250);
  rd.initMultiTarget();

  memset(lastRawSnapshot, 0, sizeof(lastRawSnapshot));
  lastRadarContentChangeMillis = millis();
  lastRadarRecoveryMillis = millis();
  radarRecoveryCount++;

  for (int i = 0; i < 3; i++) {
    resetTracker(trackers[i]);
  }

  Serial.print("Radar recovery reset complete. Count: ");
  Serial.println(radarRecoveryCount);
}

void updateTargetsFromRadar() {
  for (int i = 0; i < 3; i++) {
    const auto& t = rd.target(i);

    int16_t absSpeed = abs(t.speed_cms);
    int16_t absX = abs(t.x_mm);

    bool rawValid =
      t.valid &&
      t.dist_mm >= MIN_DETECTION_MM &&
      t.dist_mm <= MAX_DETECTION_MM;

    bool nearlyStill =
      t.valid &&
      absSpeed <= STILL_SPEED_CLEAR_CMS &&
      t.dist_mm >= MIN_DETECTION_MM &&
      t.dist_mm <= MAX_DETECTION_MM;

    uint16_t distJitter = 9999;
    uint16_t xJitter = 9999;

    if (lastDistMm[i] > 0) {
      distJitter = abs((int)t.dist_mm - (int)lastDistMm[i]);
      xJitter = abs((int)t.x_mm - (int)lastXmm[i]);
    }

    bool microMovementLikely =
      rawValid &&
      absSpeed <= MICRO_SPEED_MAX_CMS &&
      distJitter <= MICRO_DIST_JITTER_MAX_MM &&
      xJitter <= MICRO_X_JITTER_MAX_MM;

    if (microMovementLikely) {
      if (microMovementHits[i] < 255) microMovementHits[i]++;
    } else if (microMovementHits[i] > 0) {
      microMovementHits[i]--;
    }

    bool rejectAsWindOrMicroMovement =
      microMovementHits[i] >= MICRO_REPEAT_LIMIT;

    if (rawValid) {
      lastXmm[i] = t.x_mm;
      lastDistMm[i] = t.dist_mm;
    }

    if (nearlyStill) {
      if (stationaryHits[i] < 255) stationaryHits[i]++;
    } else if (stationaryHits[i] > 0) {
      stationaryHits[i]--;
    }

    if (stationaryHits[i] >= STATIONARY_CLEAR_THRESHOLD) {
      pedestrianHits[i] = 0;
      vehicleHits[i] = 0;
      unknownHits[i] = 0;
      stationaryHits[i] = 0;
      microMovementHits[i] = 0;
      clearWebTarget(i);
      continue;
    }

    bool inPedestrianZone =
      rawValid &&
      t.dist_mm >= PED_MIN_MM &&
      t.dist_mm <= PED_MAX_MM &&
      absX <= PEDESTRIAN_LATERAL_ZONE_MM;

    bool inVehicleZone =
      rawValid &&
      t.dist_mm >= VEH_MIN_MM &&
      t.dist_mm <= VEH_MAX_MM &&
      absX <= VEHICLE_LATERAL_ZONE_MM;

    bool pedestrianCandidate =
      !rejectAsWindOrMicroMovement &&
      inPedestrianZone &&
      absSpeed >= PED_MIN_SPEED_CMS &&
      absSpeed <= PED_MAX_SPEED_CMS &&
      !(t.dist_mm >= 2500 && absSpeed >= 10);

    bool vehicleCandidate =
      !rejectAsWindOrMicroMovement &&
      inVehicleZone &&
      (
        absSpeed >= VEH_MIN_SPEED_CMS ||
        (t.dist_mm >= 2500 && absSpeed >= 10) ||
        (t.dist_mm >= 4500)
      );

    if (!rejectAsWindOrMicroMovement &&
        rawValid &&
        t.dist_mm >= 5000 &&
        absSpeed >= VEH_FAST_SPEED_CMS) {
      vehicleCandidate = true;
      pedestrianCandidate = false;
    }

    if (!rejectAsWindOrMicroMovement &&
        rawValid &&
        t.dist_mm < 2200 &&
        absSpeed >= PED_MIN_SPEED_CMS &&
        absSpeed < VEH_MIN_SPEED_CMS) {
      pedestrianCandidate = true;
    }

    bool unknownCandidate =
      !rejectAsWindOrMicroMovement &&
      rawValid &&
      !pedestrianCandidate &&
      !vehicleCandidate &&
      absSpeed >= UNKNOWN_MIN_SPEED_CMS &&
      (
        distJitter >= UNKNOWN_MIN_DIST_JITTER_MM ||
        xJitter >= UNKNOWN_MIN_X_JITTER_MM
      );

    if (unknownCandidate) {
      if (unknownHits[i] < 255) unknownHits[i]++;
    } else if (unknownHits[i] > 0) {
      unknownHits[i]--;
    }

    if (pedestrianCandidate) {
      if (pedestrianHits[i] < 255) pedestrianHits[i]++;
    } else if (pedestrianHits[i] > 0) {
      pedestrianHits[i]--;
    }

    if (vehicleCandidate) {
      if (vehicleHits[i] < 255) vehicleHits[i]++;
    } else if (vehicleHits[i] > 0) {
      vehicleHits[i]--;
    }

    if (rawValid) misses[i] = 0;
    else if (misses[i] < 255) misses[i]++;

    bool pedestrianConfirmed = pedestrianHits[i] >= PED_HIT_THRESHOLD;
    bool vehicleConfirmed = vehicleHits[i] >= VEH_HIT_THRESHOLD;
    bool unknownConfirmed = unknownHits[i] >= UNKNOWN_HIT_THRESHOLD;

    uint8_t classification = CLASS_NONE;

    if (vehicleConfirmed && !pedestrianConfirmed) {
      classification = CLASS_VEHICLE;
    } else if (pedestrianConfirmed && !vehicleConfirmed) {
      classification = CLASS_PEDESTRIAN;
    } else if (vehicleConfirmed && pedestrianConfirmed) {
      classification = (absSpeed >= 12 || t.dist_mm >= 2000)
        ? CLASS_VEHICLE
        : CLASS_PEDESTRIAN;
    } else if (unknownConfirmed) {
      classification = CLASS_UNKNOWN_MOVING;
    }

    bool confirmed = rawValid && classification != CLASS_NONE;

    if (misses[i] >= MISS_CLEAR_THRESHOLD) {
      confirmed = false;
      pedestrianHits[i] = 0;
      vehicleHits[i] = 0;
      unknownHits[i] = 0;
      stationaryHits[i] = 0;
      microMovementHits[i] = 0;
      lastXmm[i] = 0;
      lastDistMm[i] = 0;
    }

    if (confirmed) {
      webTargets[i].active = true;
      webTargets[i].pedestrianLikely = classification == CLASS_PEDESTRIAN;
      webTargets[i].vehicleLikely = classification == CLASS_VEHICLE;
      webTargets[i].classification = classification;
      webTargets[i].x = t.x_mm;
      webTargets[i].y = t.y_mm;
      webTargets[i].speed = t.speed_cms;
      webTargets[i].dist = t.dist_mm;
    } else {
      clearWebTarget(i);
    }
  }

  if (lastFrameMillis > 0 && millis() - lastFrameMillis > 1500) {
    clearAllTargetsAndState();
  }
}

// ================== HTML DASHBOARD ==================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>RD03D Live Radar</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {background:#000;}
    body {
      font-family:Arial;
      text-align:center;
      background:#000;
      color:#0f0;
      margin:0;
      padding:14px;
      overflow-y:scroll;
    }
    .page {
      width:min(1120px, 100%);
      margin:0 auto;
      display:flex;
      flex-direction:column;
      align-items:center;
    }
    h1 {
      width:100%;
      margin:10px 0 14px 0;
      text-align:center;
    }
    #radarFrame {
      width:min(640px, calc(100vw - 36px));
      height:min(420px, calc((100vw - 36px) * 0.65625));
      min-height:300px;
      margin:0 auto;
      display:flex;
      align-items:center;
      justify-content:center;
      overflow:hidden;
      flex:0 0 auto;
    }
    canvas {
      display:block;
      width:100%;
      height:100%;
      border:4px solid #0f0;
      background:#111;
      box-sizing:border-box;
      flex:0 0 auto;
    }
    #alert {display:none; position:fixed; top:20px; left:50%; transform:translateX(-50%); background:#f00; color:#fff; padding:15px 30px; font-size:1.2em; border-radius:12px; box-shadow:0 0 20px #f00; z-index:10;}
    #info {
      width:min(1000px, calc(100vw - 36px));
      height:176px;
      margin:12px auto 10px auto;
      font-size:1.0em;
      background:#222;
      padding:12px;
      border-radius:8px;
      white-space:pre-wrap;
      text-align:left;
      overflow:auto;
      box-sizing:border-box;
      flex:0 0 auto;
    }
    #controls {
      width:100%;
      min-height:54px;
      margin:10px auto 6px auto;
      display:flex;
      align-items:center;
      justify-content:center;
      gap:10px;
      flex-wrap:wrap;
      flex:0 0 auto;
    }
    button {padding:10px 20px; font-size:1.0em;}
    table {width:100%; max-width:1000px; margin:16px auto 20px auto; border-collapse:collapse; background:#111;}
    th, td {border:1px solid #0a0; padding:8px; text-align:left; font-size:0.95em;}
    th {background:#0a0; color:#000;}
    tr.selectedRow { background:#233; }
    .muted { color:#9a9; }
  </style>
</head>
<body>
  <div class="page">
    <h1>RD03D Filtered Forward Radar, 120° Sector, 9 meter range</h1>
    <div id="radarFrame">
      <canvas id="c" width="640" height="420"></canvas>
    </div>
    <div id="alert"><span id="alertText"></span></div>
    <div id="info">Waiting for radar data...</div>

    <div id="controls">
      <button id="logButton" onclick="toggleLogging()">Stop Logging</button>
      <button onclick="clearLog()">Clear Log</button>
      <button onclick="downloadLog()">Download CSV</button>
      <button onclick="resetRadar()">Reset Radar</button>
    </div>

    <h2>Detection History</h2>
    <table id="logTable">
    <tr>
      <th>ID</th>
      <th>At (s)</th>
      <th>Class</th>
      <th>Closest Range</th>
      <th>Peak Speed</th>
      <th>Path Length</th>
      <th>Trail</th>
    </tr>
    </table>
  </div>

<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

let W = canvas.width;
let H = canvas.height;
let originX = W / 2;
let originY = H - 18;
let maxRadiusPx = H - 42;
const maxRangeMm = 9000;
const halfAngleDeg = 60;

function syncCanvasSize() {
  const rect = canvas.getBoundingClientRect();
  const newW = Math.max(320, Math.round(rect.width));
  const newH = Math.max(300, Math.round(rect.height));

  if (canvas.width !== newW || canvas.height !== newH) {
    canvas.width = newW;
    canvas.height = newH;
  }

  W = canvas.width;
  H = canvas.height;
  originX = W / 2;
  originY = H - 18;
  maxRadiusPx = H - 42;
}

window.addEventListener('resize', () => {
  syncCanvasSize();
  draw();
});

let targets = [];
let liveHistory = [[], [], []];
let logs = [];
let selectedLogIds = new Set();
let lastAlert = 0;
let loggingEnabled = true;

const LIVE_TRAIL_MAX_POINTS = 30;

function playBeep() {
  try {
    const audio = new (window.AudioContext || window.webkitAudioContext)();
    const osc = audio.createOscillator();
    const gain = audio.createGain();
    osc.type = 'sine';
    osc.frequency.setValueAtTime(800, audio.currentTime);
    gain.gain.value = 0.30;
    osc.connect(gain).connect(audio.destination);
    osc.start();
    setTimeout(() => osc.stop(), 180);
  } catch(e) {}
}

function showAlert(text) {
  const alertDiv = document.getElementById('alert');
  document.getElementById('alertText').textContent = text;
  alertDiv.style.display = 'block';
  playBeep();
  setTimeout(() => { alertDiv.style.display = 'none'; }, 1800);
}

function mmToPx(mm) {
  return (mm / maxRangeMm) * maxRadiusPx;
}

function classTextFromCode(code) {
  if (code === 1) return 'PEDESTRIAN';
  if (code === 2) return 'VEHICLE';
  if (code === 3) return 'UNKNOWN';
  return 'NONE';
}

function drawSectorArc(radiusPx, color, width = 1) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  const start = (-Math.PI / 2) - (halfAngleDeg * Math.PI / 180);
  const end   = (-Math.PI / 2) + (halfAngleDeg * Math.PI / 180);
  ctx.arc(originX, originY, radiusPx, start, end);
  ctx.stroke();
  ctx.restore();
}

function drawGrid() {
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, W, H);

  for (let m = 1; m <= 9; m++) {
    const r = mmToPx(m * 1000);
    drawSectorArc(r, '#0a0', (m === 9 ? 2 : 1));
    ctx.fillStyle = '#0a0';
    ctx.font = '13px Arial';
    ctx.fillText(m + 'm', originX + 8, originY - r + 12);
  }

  const a1 = (-90 - halfAngleDeg) * Math.PI / 180;
  const a2 = (-90 + halfAngleDeg) * Math.PI / 180;
  const edge1x = originX + Math.cos(a1) * maxRadiusPx;
  const edge1y = originY + Math.sin(a1) * maxRadiusPx;
  const edge2x = originX + Math.cos(a2) * maxRadiusPx;
  const edge2y = originY + Math.sin(a2) * maxRadiusPx;

  ctx.strokeStyle = '#0f0';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(originX, originY); ctx.lineTo(edge1x, edge1y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(originX, originY); ctx.lineTo(edge2x, edge2y); ctx.stroke();

  ctx.strokeStyle = '#055';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(originX, originY);
  ctx.lineTo(originX, originY - maxRadiusPx);
  ctx.stroke();

  drawSectorArc(mmToPx(2500), '#665500', 1);
  drawSectorArc(mmToPx(6000), '#224466', 1);

  ctx.fillStyle = '#0f0';
  ctx.beginPath();
  ctx.arc(originX, originY, 5, 0, Math.PI * 2);
  ctx.fill();
}

function drawSavedTrails() {
  logs.forEach(log => {
    if (!selectedLogIds.has(log.id)) return;
    if (!log.pinnedTrail || !log.trail || log.trail.length < 2) return;

    ctx.save();
    ctx.strokeStyle = '#ff66ff';
    ctx.lineWidth = 3;
    ctx.beginPath();

    log.trail.forEach((p, idx) => {
      const px = originX + mmToPx(p.x);
      const py = originY - mmToPx(p.y);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });

    ctx.stroke();

    const last = log.trail[log.trail.length - 1];
    const lx = originX + mmToPx(last.x);
    const ly = originY - mmToPx(last.y);
    ctx.fillStyle = '#ff66ff';
    ctx.beginPath();
    ctx.arc(lx, ly, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  });
}

function drawLiveTargets() {
  targets.forEach((t, i) => {
    const px = originX + mmToPx(t.x);
    const py = originY - mmToPx(t.y);

    liveHistory[i].forEach((p, idx) => {
      const tx = originX + mmToPx(p.x);
      const ty = originY - mmToPx(p.y);
      ctx.globalAlpha = ((idx + 1) / liveHistory[i].length) * 0.45;
      let trailColor = '#0ff';
      if (p.classification === 1) trailColor = '#00ff66';
      if (p.classification === 2) trailColor = '#ff6633';
      ctx.fillStyle = trailColor;
      ctx.fillRect(tx - 2, ty - 2, 4, 4);
    });
    ctx.globalAlpha = 1.0;

    let color = '#0ff';
    if (t.classification === 1) color = '#00ff66';
    if (t.classification === 2) color = '#ff6633';

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(px, py, 12, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(px, py, 12, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#fff';
    ctx.font = 'bold 15px Arial';
    ctx.fillText('T' + (i + 1), px + 18, py + 4);

    ctx.font = '12px Arial';
    ctx.fillText('D:' + (t.dist / 1000).toFixed(2) + 'm', px + 18, py + 20);
    ctx.fillText('S:' + t.speed.toFixed(0) + 'cm/s', px + 18, py + 34);
    ctx.fillText(classTextFromCode(t.classification), px + 18, py + 48);
  });
}

function draw() {
  syncCanvasSize();
  drawGrid();
  drawSavedTrails();
  drawLiveTargets();
}

function refreshLogTable() {
  const table = document.getElementById('logTable');

  while (table.rows.length > 1) {
    table.deleteRow(1);
  }

  const sorted = [...logs].sort((a, b) => b.id - a.id);

  sorted.forEach(item => {
    const row = table.insertRow();
    if (selectedLogIds.has(item.id)) row.classList.add('selectedRow');

    row.onclick = () => {
      if (item.pinnedTrail) {
        if (selectedLogIds.has(item.id)) selectedLogIds.delete(item.id);
        else selectedLogIds.add(item.id);
        refreshLogTable();
        draw();
      }
    };

    row.insertCell().textContent = item.id;
    row.insertCell().textContent = (item.endMs / 1000).toFixed(1);
    row.insertCell().textContent = item.classText;
    row.insertCell().textContent = (item.minDistMm / 1000).toFixed(2) + ' m';
    row.insertCell().textContent = item.peakSpeedCms + ' cm/s';
    row.insertCell().textContent = (item.pathLenMm / 1000).toFixed(2) + ' m';
    row.insertCell().textContent = item.pinnedTrail ? (selectedLogIds.has(item.id) ? 'Shown' : 'Select') : 'No saved trail';
  });
}

function toggleLogging() {
  const newValue = !loggingEnabled;
  fetch('/setLogging?enabled=' + (newValue ? '1' : '0'), { method: 'POST' })
    .then(r => r.text())
    .then(() => {
      loggingEnabled = newValue;
      const btn = document.getElementById('logButton');
      btn.textContent = loggingEnabled ? 'Stop Logging' : 'Start Logging';
      btn.style.background = loggingEnabled ? '#f00' : '';
      btn.style.color = loggingEnabled ? '#fff' : '';
    });
}

function clearLog() {
  fetch('/clearLogs', { method: 'POST' })
    .then(r => r.text())
    .then(() => {
      selectedLogIds = new Set();
      fetchLogs();
    });
}

function downloadLog() {
  if (logs.length === 0) {
    alert('Log is empty');
    return;
  }

  let csv = 'ID,At_s,Class,ClosestRange_m,PeakSpeed_cm_s,PathLength_m,PinnedTrail\n';
  const sorted = [...logs].sort((a, b) => a.id - b.id);
  sorted.forEach(item => {
    csv += `${item.id},${(item.endMs / 1000).toFixed(1)},${item.classText},${(item.minDistMm / 1000).toFixed(2)},${item.peakSpeedCms},${(item.pathLenMm / 1000).toFixed(2)},${item.pinnedTrail ? 'YES' : 'NO'}\n`;
  });

  const blob = new Blob([csv], {type: 'text/csv'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'RD03D_detection_history.csv';
  a.click();
  URL.revokeObjectURL(url);
}

function resetRadar() {
  fetch('/resetRadar', { method: 'POST' })
    .then(r => r.text())
    .then(msg => showAlert(msg));
}

function fetchLogs() {
  fetch('/logs')
    .then(r => r.json())
    .then(data => {
      logs = data.logs || [];
      const validIds = new Set(logs.map(x => x.id));
      selectedLogIds.forEach(id => {
        if (!validIds.has(id)) selectedLogIds.delete(id);
      });
      refreshLogTable();
      draw();
    });
}

setInterval(() => {
  fetch('/data')
    .then(r => r.json())
    .then(data => {
      targets = data.targets || [];
      loggingEnabled = data.loggingEnabled;

      const btn = document.getElementById('logButton');
      btn.textContent = loggingEnabled ? 'Stop Logging' : 'Start Logging';
      btn.style.background = loggingEnabled ? '#f00' : '';
      btn.style.color = loggingEnabled ? '#fff' : '';

      let info = '';
      info += 'Targets: ' + targets.length + '<br>';
      info += 'Last frame: ' + data.lastFrameAgeMs + ' ms ago<br>';
      info += 'Bad frames: ' + data.badFrames + '<br>';
      info += 'Logging: ' + (loggingEnabled ? 'ON' : 'OFF') + '<br>';
      info += 'Radar resets: ' + data.radarRecoveryCount + '<br>';
      info += 'Frozen-content resets: ' + data.radarFrozenRecoveryCount + '<br>';
      info += '<span class="muted">Display: filtered confirmed targets only, 120° forward sector, 9 m range</span><br><br>';

      targets.forEach((t, i) => {
        liveHistory[i].push({x: t.x, y: t.y, classification: t.classification});
        if (liveHistory[i].length > LIVE_TRAIL_MAX_POINTS) liveHistory[i].shift();

        info += 'T' + (i + 1) + ': ' + classTextFromCode(t.classification) +
                ' | Dist ' + (t.dist / 1000).toFixed(2) + ' m' +
                ' | Speed ' + t.speed.toFixed(0) + ' cm/s' +
                ' | X ' + t.x.toFixed(0) + ' mm' +
                ' | Y ' + t.y.toFixed(0) + ' mm<br>';
      });

      document.getElementById('info').innerHTML = info;

      const knownTargets = targets.filter(t => t.classification === 1 || t.classification === 2);
      const knownMovingTargets = knownTargets.filter(t => Math.abs(t.speed) >= 6);

      if (knownMovingTargets.length > 0 && Date.now() - lastAlert > 5000) {
        showAlert(knownMovingTargets.length + ' CLASSIFIED TARGET(S) DETECTED');
        lastAlert = Date.now();
      }

      draw();
    })
    .catch(err => {
      document.getElementById('info').innerHTML = 'Fetch error: ' + err;
    });
}, 200);

setInterval(fetchLogs, 1500);
fetchLogs();
draw();
</script>
</body>
</html>
)rawliteral";

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n===== RD03D 9m Forward Sector Radar =====");
  Serial.println("RD03D pins: RX=GPIO7 TX=GPIO6");

  rd.begin(RadarSerial);
  rd.initMultiTarget();
  delay(500);
  rd.initMultiTarget();

  memset(lastRawSnapshot, 0, sizeof(lastRawSnapshot));
  lastRadarContentChangeMillis = millis();
  lastRadarRecoveryMillis = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected");
  Serial.print("Open on any device: http://");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;

    doc["badFrames"] = totalBadFrames;
    doc["radarRecoveryCount"] = radarRecoveryCount;
    doc["radarFrozenRecoveryCount"] = radarFrozenRecoveryCount;
    doc["loggingEnabled"] = loggingEnabled;

    if (lastFrameMillis == 0) doc["lastFrameAgeMs"] = -1;
    else doc["lastFrameAgeMs"] = millis() - lastFrameMillis;

    JsonArray arr = doc["targets"].to<JsonArray>();
    for (int i = 0; i < 3; i++) {
      if (webTargets[i].active) {
        JsonObject t = arr.add<JsonObject>();
        t["id"] = i + 1;
        t["x"] = webTargets[i].x;
        t["y"] = webTargets[i].y;
        t["speed"] = webTargets[i].speed;
        t["dist"] = webTargets[i].dist;
        t["pedestrianLikely"] = webTargets[i].pedestrianLikely;
        t["vehicleLikely"] = webTargets[i].vehicleLikely;
        t["classification"] = webTargets[i].classification;
      }
    }

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{\"logs\":[");
    bool firstLog = true;

    for (int i = 0; i < MAX_LOG_ITEMS; i++) {
      if (!logItems[i].valid) continue;

      if (!firstLog) response->print(",");
      firstLog = false;

      const LogItem &li = logItems[i];
      response->print("{");
      response->printf("\"id\":%lu,", (unsigned long)li.id);
      response->printf("\"startMs\":%lu,", (unsigned long)li.startMs);
      response->printf("\"endMs\":%lu,", (unsigned long)li.endMs);
      response->printf("\"slotId\":%u,", li.slotId);
      response->printf("\"classification\":%u,", li.classification);
      response->printf("\"classText\":\"%s\",", classToText(li.classification).c_str());
      response->printf("\"minDistMm\":%u,", li.minDistMm);
      response->printf("\"peakSpeedCms\":%d,", li.peakSpeedCms);
      response->printf("\"pathLenMm\":%lu,", (unsigned long)li.pathLenMm);
      response->printf("\"netDisplacementMm\":%u,", li.netDisplacementMm);
      response->printf("\"maxAxisSpanMm\":%u,", li.maxAxisSpanMm);
      response->printf("\"pinnedTrail\":%s", li.pinnedTrail ? "true" : "false");

      if (li.pinnedTrail) {
        response->print(",\"trail\":[");
        for (int p = 0; p < li.trailCount; p++) {
          if (p > 0) response->print(",");
          response->printf("{\"x\":%d,\"y\":%d}", li.trail[p].x_mm, li.trail[p].y_mm);
        }
        response->print("]");
      }

      response->print("}");
    }

    response->print("]}");
    request->send(response);
  });

  server.on("/setLogging", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("enabled")) {
      loggingEnabled = request->getParam("enabled")->value() == "1";
    }
    request->send(200, "text/plain", loggingEnabled ? "Logging enabled" : "Logging disabled");
  });

  server.on("/clearLogs", HTTP_POST, [](AsyncWebServerRequest *request) {
    clearAllLogs();
    request->send(200, "text/plain", "All logs cleared");
  });

  server.on("/resetRadar", HTTP_POST, [](AsyncWebServerRequest *request) {
    recoverRadarSensor();
    request->send(200, "text/plain", "Radar reset sent");
  });

  server.begin();
}

// ================== LOOP ==================
void loop() {
  uint32_t bytesThisCall = 0;
  bool gotFrame = rd.poll(&bytesThisCall);

  if (bytesThisCall > 0) {
    totalUartBytes += bytesThisCall;
    lastSecondUartBytes += bytesThisCall;
  }

  if (gotFrame) {
    if (radarContentChanged()) {
      lastRadarContentChangeMillis = millis();
    }

    updateTargetsFromRadar();
    updateEventTrackers();
  } else {
    finalizeExpiredTrackers();
  }

  bool radarFrameStalled =
    (lastFrameMillis > 0) &&
    ((millis() - lastFrameMillis) > RADAR_FRAME_STALL_RESET_MS) &&
    ((millis() - lastRadarRecoveryMillis) > RADAR_FRAME_RESET_COOLDOWN_MS);

  bool radarContentStalled =
    radarHasPotentialMotion() &&
    (lastFrameMillis > 0) &&
    ((millis() - lastRadarContentChangeMillis) > RADAR_CONTENT_STALL_RESET_MS) &&
    ((millis() - lastRadarRecoveryMillis) > RADAR_CONTENT_RESET_COOLDOWN_MS);

  if (radarFrameStalled || radarContentStalled) {
    if (radarContentStalled) {
      radarFrozenRecoveryCount++;
      Serial.println("Radar content stall detected");
    }
    recoverRadarSensor();
  }

  if (lastFrameMillis > 0 && millis() - lastFrameMillis > 1500) {
    clearAllTargetsAndState();
  }

  if (millis() - lastStatusMillis > 1000) {
    Serial.print("UART bytes/sec: ");
    Serial.print(lastSecondUartBytes);
    Serial.print(" | Bad frames: ");
    Serial.print(totalBadFrames);
    Serial.print(" | Radar resets: ");
    Serial.print(radarRecoveryCount);
    Serial.print(" | Frozen resets: ");
    Serial.print(radarFrozenRecoveryCount);
    Serial.print(" | Logging: ");
    Serial.println(loggingEnabled ? "ON" : "OFF");

    for (int i = 0; i < 3; i++) {
      if (webTargets[i].active) {
        Serial.print("  T");
        Serial.print(i + 1);
        Serial.print(" X:");
        Serial.print(webTargets[i].x);
        Serial.print(" Y:");
        Serial.print(webTargets[i].y);
        Serial.print(" D:");
        Serial.print(webTargets[i].dist);
        Serial.print(" S:");
        Serial.print(webTargets[i].speed);
        Serial.print(" Class:");
        Serial.println(classToText(webTargets[i].classification));
      }
    }

    lastSecondUartBytes = 0;
    lastStatusMillis = millis();
  }

  delay(2);
}