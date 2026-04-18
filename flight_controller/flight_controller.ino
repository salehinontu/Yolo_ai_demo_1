/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║          ESP32-C3 Mini Drone — Full Flight Controller        ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Sensor  : MPU6050 (I2C)                                     ║
 * ║  Motors  : 4x Brushless + ESC (X-quad layout)                ║
 * ║  Control : WiFi — open browser on phone → 192.168.4.1        ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 *  Wiring:
 *    MPU6050 SDA  → GPIO 8
 *    MPU6050 SCL  → GPIO 9
 *    MPU6050 VCC  → 3.3V  |  GND → GND
 *    ESC FL       → GPIO 4   (Front-Left,  CCW)
 *    ESC FR       → GPIO 5   (Front-Right, CW)
 *    ESC BL       → GPIO 6   (Back-Left,   CW)
 *    ESC BR       → GPIO 7   (Back-Right,  CCW)
 *
 *  X-layout (viewed from above):
 *    [FL] ── [FR]
 *      |      |
 *    [BL] ── [BR]
 *
 *  Required libraries (Arduino Library Manager):
 *    → ESPAsyncWebServer  (by lacamera or me-no-dev)
 *    → AsyncTCP           (by me-no-dev)
 *    → (AsyncUDP is built into ESP32 core — no install needed)
 *
 *  Board settings:
 *    Board : ESP32C3 Dev Module
 *    USB CDC On Boot : Enabled
 *
 *  NOTE — ESP32 Arduino core version:
 *    Core 2.x : ledcSetup() + ledcAttachPin()  ← this code uses these
 *    Core 3.x : replace with ledcAttach(pin, freq, res)
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

// ============================================================
//  PID GAINS — TUNE THESE FOR YOUR DRONE
// ============================================================
//  How to tune (one axis at a time, props OFF first):
//    1. Set KI = KD = 0
//    2. Raise KP until drone oscillates → back off to ~60%
//    3. Raise KD until oscillation dampens (starts stable)
//    4. Add a tiny KI to fix slow drift
// ============================================================
namespace PID {
    // Roll  (left ↔ right tilt)
    constexpr float ROLL_KP  = 1.4f;
    constexpr float ROLL_KI  = 0.04f;
    constexpr float ROLL_KD  = 18.0f;

    // Pitch (forward ↔ backward tilt)
    constexpr float PITCH_KP = 1.4f;
    constexpr float PITCH_KI = 0.04f;
    constexpr float PITCH_KD = 18.0f;

    // Yaw   (rotation rate)
    constexpr float YAW_KP   = 3.5f;
    constexpr float YAW_KI   = 0.02f;
    constexpr float YAW_KD   = 0.0f;

    constexpr float INTEGRAL_LIMIT = 150.0f;   // anti-windup clamp
}

// ============================================================
//  HARDWARE PINS — change to match your wiring
// ============================================================
namespace Pin {
    constexpr int SDA      = 8;
    constexpr int SCL      = 9;
    constexpr int MOTOR_FL = 4;
    constexpr int MOTOR_FR = 5;
    constexpr int MOTOR_BL = 6;
    constexpr int MOTOR_BR = 7;
}

// ============================================================
//  FLIGHT CONFIG
// ============================================================
namespace Cfg {
    constexpr int   LOOP_HZ       = 500;
    constexpr float LOOP_DT       = 1.0f / LOOP_HZ;

    constexpr float MAX_TILT_DEG  = 25.0f;    // max roll / pitch angle
    constexpr float MAX_YAW_DPS   = 150.0f;   // max yaw rate (deg/s)

    constexpr int   ESC_DISARMED  = 1000;     // pulse width when disarmed (us)
    constexpr int   THR_MIN       = 1100;     // min throttle while armed
    constexpr int   THR_MAX       = 1800;     // max throttle (leave 200 us headroom)
    constexpr int   MOTOR_MIN     = 1050;     // absolute motor floor
    constexpr int   MOTOR_MAX     = 1950;     // absolute motor ceiling

    constexpr uint32_t FAILSAFE_MS = 500;     // disarm after no signal for this long
    constexpr float    CRASH_DEG   = 65.0f;   // crash-detect: kill motors if tilted this far

    constexpr float COMP_ALPHA    = 0.98f;    // complementary filter (0 = accel only, 1 = gyro only)
}

// ============================================================
//  WIFI / NETWORK
// ============================================================
namespace Net {
    constexpr const char* SSID     = "DroneAP";
    constexpr const char* PASSWORD = "drone1234";
    constexpr uint16_t    UDP_PORT = 4210;    // also accepts UDP packets (for external apps)
}

// ============================================================
//  MPU6050 REGISTER MAP (do not change)
// ============================================================
namespace IMU {
    constexpr uint8_t ADDR         = 0x68;
    constexpr uint8_t PWR_MGMT_1   = 0x6B;
    constexpr uint8_t SMPLRT_DIV   = 0x19;
    constexpr uint8_t CONFIG       = 0x1A;
    constexpr uint8_t GYRO_CONFIG  = 0x1B;
    constexpr uint8_t ACCEL_CONFIG = 0x1C;
    constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    constexpr uint8_t GYRO_XOUT_H  = 0x43;
    constexpr uint8_t WHO_AM_I     = 0x75;
    constexpr float   ACCEL_LSB    = 16384.0f;  // ±2 g
    constexpr float   GYRO_LSB     = 131.0f;    // ±250 °/s
    constexpr int     CALIB_N      = 2000;
}

// ============================================================
//  DATA TYPES
// ============================================================
struct Vec3 { float x = 0, y = 0, z = 0; };

struct PIDCtrl {
    const float kp, ki, kd;
    float integ    = 0;
    float prevErr  = 0;

    constexpr PIDCtrl(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    float update(float setpoint, float measured, float dt) {
        float err  = setpoint - measured;
        integ     += err * dt;
        integ      = constrain(integ, -PID::INTEGRAL_LIMIT, PID::INTEGRAL_LIMIT);
        float deriv = (dt > 0.0f) ? (err - prevErr) / dt : 0.0f;
        prevErr    = err;
        return kp * err + ki * integ + kd * deriv;
    }

    void reset() { integ = 0; prevErr = 0; }
};

struct CtrlInput {
    float throttle = 0;   // 0.0 – 1.0
    float roll     = 0;   // -1.0 – 1.0  (right = positive)
    float pitch    = 0;   // -1.0 – 1.0  (forward = positive)
    float yaw      = 0;   // -1.0 – 1.0  (clockwise = positive)
    bool  armed    = false;
};

struct Attitude {
    float roll  = 0;   // degrees
    float pitch = 0;   // degrees
    float yaw   = 0;   // degrees (gyro-integrated, no absolute ref)
};

// ============================================================
//  GLOBALS
// ============================================================
PIDCtrl pidRoll (PID::ROLL_KP,  PID::ROLL_KI,  PID::ROLL_KD);
PIDCtrl pidPitch(PID::PITCH_KP, PID::PITCH_KI, PID::PITCH_KD);
PIDCtrl pidYaw  (PID::YAW_KP,   PID::YAW_KI,   PID::YAW_KD);

Vec3       gyroOffset;
Attitude   attitude;
CtrlInput  ctrl;

AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");
AsyncUDP        udp;

volatile uint32_t lastPktMs = 0;

int pwmFL = Cfg::ESC_DISARMED,  pwmFR = Cfg::ESC_DISARMED;
int pwmBL = Cfg::ESC_DISARMED,  pwmBR = Cfg::ESC_DISARMED;

constexpr int LEDC_FL = 0, LEDC_FR = 1, LEDC_BL = 2, LEDC_BR = 3;

// ============================================================
//  BROWSER CONTROL PAGE (stored in flash)
// ============================================================
const char HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Drone</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:#111;color:#fff;font-family:monospace;height:100vh;overflow:hidden;display:flex;flex-direction:column}
#hdr{text-align:center;padding:8px;font-size:16px;background:#1a1a2e;color:#e94560;letter-spacing:2px}
#bar{text-align:center;font-size:11px;padding:4px;background:#0f3460;color:#adf}
#main{flex:1;display:flex;justify-content:space-around;align-items:center;padding:10px}
.jbox{display:flex;flex-direction:column;align-items:center;gap:8px}
.jlbl{font-size:10px;color:#888;letter-spacing:1px}
.jbase{position:relative;width:160px;height:160px;border-radius:50%;background:rgba(255,255,255,.07);border:2px solid rgba(255,255,255,.2);touch-action:none}
.jthumb{position:absolute;width:54px;height:54px;border-radius:50%;background:#e94560;top:53px;left:53px;pointer-events:none}
#mid{display:flex;flex-direction:column;align-items:center;gap:12px}
#armBtn{padding:14px 24px;font-size:17px;font-family:monospace;background:#1a1a2e;border:2px solid #e94560;color:#fff;border-radius:8px;cursor:pointer;letter-spacing:1px}
#armBtn.on{background:#e94560}
#tele{font-size:10px;color:#555;text-align:center;line-height:1.6}
#thr{width:60px;text-align:center;font-size:11px;color:#e94560}
</style>
</head>
<body>
<div id="hdr">ESP32-C3 DRONE</div>
<div id="bar" id="st">Connecting...</div>
<div id="main">
  <div class="jbox">
    <div class="jbase" id="lj"><div class="jthumb" id="lt"></div></div>
    <div class="jlbl">THR ↕ &nbsp; YAW ↔</div>
  </div>
  <div id="mid">
    <button id="armBtn" onclick="toggleArm()">ARM</button>
    <div id="thr">THR 0%</div>
    <div id="tele">R:0.0 P:0.0 Y:0.0</div>
  </div>
  <div class="jbox">
    <div class="jbase" id="rj"><div class="jthumb" id="rt"></div></div>
    <div class="jlbl">PITCH ↕ &nbsp; ROLL ↔</div>
  </div>
</div>
<script>
let ws,armed=false,c={t:0,r:0,p:0,y:0};
const st=document.getElementById('bar');
function connect(){
  ws=new WebSocket('ws://'+location.hostname+'/ws');
  ws.onopen=()=>st.textContent='Connected ✓';
  ws.onclose=()=>{st.textContent='Reconnecting...';setTimeout(connect,2000)};
  ws.onmessage=e=>document.getElementById('tele').textContent=e.data;
}
function toggleArm(){
  armed=!armed;
  const b=document.getElementById('armBtn');
  b.textContent=armed?'DISARM':'ARM';
  b.classList.toggle('on',armed);
  if(!armed){c={t:0,r:0,p:0,y:0}}
}
function send(){
  if(ws&&ws.readyState===1)
    ws.send('T:'+c.t.toFixed(3)+',R:'+c.r.toFixed(3)+',P:'+c.p.toFixed(3)+',Y:'+c.y.toFixed(3)+',A:'+(armed?1:0));
  document.getElementById('thr').textContent='THR '+(c.t*100|0)+'%';
}
function joystick(baseId,thumbId,cb){
  const base=document.getElementById(baseId);
  const thumb=document.getElementById(thumbId);
  const R=80,r=27;
  let on=false;
  function move(cx,cy){
    const rect=base.getBoundingClientRect();
    let x=cx-rect.left-R, y=cy-rect.top-R;
    const d=Math.hypot(x,y),m=R-r;
    if(d>m){x=x/d*m;y=y/d*m}
    thumb.style.left=(R+x-r)+'px';
    thumb.style.top=(R+y-r)+'px';
    cb(x/m,-y/m);
  }
  function end(){on=false;thumb.style.left=(R-r)+'px';thumb.style.top=(R-r)+'px';cb(0,0)}
  base.addEventListener('touchstart',e=>{on=true;const t=e.touches[0];move(t.clientX,t.clientY);e.preventDefault()},{passive:false});
  base.addEventListener('touchmove',e=>{if(on){const t=e.touches[0];move(t.clientX,t.clientY)}e.preventDefault()},{passive:false});
  base.addEventListener('touchend',end);
  base.addEventListener('mousedown',e=>{on=true;move(e.clientX,e.clientY)});
  document.addEventListener('mousemove',e=>{if(on)move(e.clientX,e.clientY)});
  document.addEventListener('mouseup',()=>{if(on)end()});
}
// Left joystick: vertical = throttle (up=more), horizontal = yaw
joystick('lj','lt',(x,y)=>{c.y=x;c.t=Math.max(0,Math.min(1,y))});
// Right joystick: vertical = pitch, horizontal = roll
joystick('rj','rt',(x,y)=>{c.r=x;c.p=y});
setInterval(send,20);  // 50 Hz send rate
connect();
</script>
</body></html>
)rawhtml";

// ============================================================
//  IMU FUNCTIONS
// ============================================================
static void imuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU::ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool imuInit() {
    Wire.beginTransmission(IMU::ADDR);
    Wire.write(IMU::WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU::ADDR, (uint8_t)1);
    uint8_t id = Wire.read();

    if (id != 0x68 && id != 0x72 && id != 0x70) {
        Serial.printf("[IMU] NOT FOUND  WHO_AM_I=0x%02X\n", id);
        return false;
    }
    imuWrite(IMU::PWR_MGMT_1,   0x01);  // PLL with gyro X clock
    delay(10);
    imuWrite(IMU::SMPLRT_DIV,   0x00);  // 1 kHz sample rate
    imuWrite(IMU::CONFIG,        0x03);  // DLPF 44 Hz — kills vibration noise
    imuWrite(IMU::GYRO_CONFIG,   0x00);  // ±250 °/s
    imuWrite(IMU::ACCEL_CONFIG,  0x00);  // ±2 g
    Serial.printf("[IMU] OK  id=0x%02X\n", id);
    return true;
}

static void imuReadBurst(Vec3& accel, Vec3& gyro) {
    Wire.beginTransmission(IMU::ADDR);
    Wire.write(IMU::ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU::ADDR, (uint8_t)14);

    auto r16 = []() -> int16_t {
        return (int16_t)((Wire.read() << 8) | Wire.read());
    };

    int16_t ax = r16(), ay = r16(), az = r16();
    r16();                                    // temperature — skip
    int16_t gx = r16(), gy = r16(), gz = r16();

    accel.x = ax / IMU::ACCEL_LSB;
    accel.y = ay / IMU::ACCEL_LSB;
    accel.z = az / IMU::ACCEL_LSB;

    gyro.x  = (gx / IMU::GYRO_LSB) - gyroOffset.x;
    gyro.y  = (gy / IMU::GYRO_LSB) - gyroOffset.y;
    gyro.z  = (gz / IMU::GYRO_LSB) - gyroOffset.z;
}

void imuCalibrate() {
    Serial.println("[IMU] Calibrating — keep drone LEVEL and STILL...");
    Vec3 sum;
    for (int i = 0; i < IMU::CALIB_N; i++) {
        Wire.beginTransmission(IMU::ADDR);
        Wire.write(IMU::GYRO_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(IMU::ADDR, (uint8_t)6);
        sum.x += (int16_t)((Wire.read() << 8) | Wire.read()) / IMU::GYRO_LSB;
        sum.y += (int16_t)((Wire.read() << 8) | Wire.read()) / IMU::GYRO_LSB;
        sum.z += (int16_t)((Wire.read() << 8) | Wire.read()) / IMU::GYRO_LSB;
        delayMicroseconds(500);
    }
    gyroOffset.x = sum.x / IMU::CALIB_N;
    gyroOffset.y = sum.y / IMU::CALIB_N;
    gyroOffset.z = sum.z / IMU::CALIB_N;
    Serial.printf("[IMU] Gyro offset  x=%.4f  y=%.4f  z=%.4f\n",
                  gyroOffset.x, gyroOffset.y, gyroOffset.z);
}

// ============================================================
//  ATTITUDE ESTIMATION — Complementary Filter
// ============================================================
//  Short-term: trust gyro (fast, no drift over one loop)
//  Long-term:  correct with accelerometer (gravity reference)
// ============================================================
void updateAttitude(const Vec3& acc, const Vec3& gyr, float dt) {
    float aRoll  = atan2f(acc.y, acc.z) * RAD_TO_DEG;
    float aPitch = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;

    attitude.roll  = Cfg::COMP_ALPHA * (attitude.roll  + gyr.x * dt)
                   + (1.0f - Cfg::COMP_ALPHA) * aRoll;
    attitude.pitch = Cfg::COMP_ALPHA * (attitude.pitch + gyr.y * dt)
                   + (1.0f - Cfg::COMP_ALPHA) * aPitch;

    attitude.yaw  += gyr.z * dt;
    if (attitude.yaw >  180.0f) attitude.yaw -= 360.0f;
    if (attitude.yaw < -180.0f) attitude.yaw += 360.0f;
}

// ============================================================
//  ESC / MOTOR CONTROL
// ============================================================
static uint32_t usToDuty(int us) {
    // 50 Hz timer → period = 20 000 us, 16-bit resolution (0–65535)
    return (uint32_t)((us / 20000.0f) * 65535.0f);
}

static void motorWrite(int ch, int us) {
    ledcWrite(ch, usToDuty(constrain(us, Cfg::ESC_DISARMED, Cfg::MOTOR_MAX)));
}

void escInit() {
    // Setup four LEDC channels at 50 Hz
    ledcSetup(LEDC_FL, 50, 16);  ledcAttachPin(Pin::MOTOR_FL, LEDC_FL);
    ledcSetup(LEDC_FR, 50, 16);  ledcAttachPin(Pin::MOTOR_FR, LEDC_FR);
    ledcSetup(LEDC_BL, 50, 16);  ledcAttachPin(Pin::MOTOR_BL, LEDC_BL);
    ledcSetup(LEDC_BR, 50, 16);  ledcAttachPin(Pin::MOTOR_BR, LEDC_BR);

    // Send disarm pulse — required for ESC calibration / arming sequence
    uint32_t d = usToDuty(Cfg::ESC_DISARMED);
    ledcWrite(LEDC_FL, d);  ledcWrite(LEDC_FR, d);
    ledcWrite(LEDC_BL, d);  ledcWrite(LEDC_BR, d);

    Serial.println("[ESC] Arm signal sent — waiting 3 s for ESC beep sequence...");
    delay(3000);
    Serial.println("[ESC] Ready");
}

void motorsStop() {
    motorWrite(LEDC_FL, Cfg::ESC_DISARMED);
    motorWrite(LEDC_FR, Cfg::ESC_DISARMED);
    motorWrite(LEDC_BL, Cfg::ESC_DISARMED);
    motorWrite(LEDC_BR, Cfg::ESC_DISARMED);
    pwmFL = pwmFR = pwmBL = pwmBR = Cfg::ESC_DISARMED;
}

// ============================================================
//  MOTOR MIXING — X-Quadcopter
// ============================================================
//  Spin directions (viewed from above):
//    FL CCW, FR CW, BL CW, BR CCW
//
//  Equations:
//    FL = thr + pitch + roll − yaw   (CCW motor → opposes +yaw)
//    FR = thr + pitch − roll + yaw
//    BL = thr − pitch + roll + yaw
//    BR = thr − pitch − roll − yaw
// ============================================================
void mixAndWrite(float thr, float rollOut, float pitchOut, float yawOut) {
    int t = (int)map((long)(thr * 1000.0f), 0, 1000, Cfg::THR_MIN, Cfg::THR_MAX);
    int ro = (int)rollOut,  po = (int)pitchOut,  yo = (int)yawOut;

    pwmFL = constrain(t + po + ro - yo, Cfg::MOTOR_MIN, Cfg::MOTOR_MAX);
    pwmFR = constrain(t + po - ro + yo, Cfg::MOTOR_MIN, Cfg::MOTOR_MAX);
    pwmBL = constrain(t - po + ro + yo, Cfg::MOTOR_MIN, Cfg::MOTOR_MAX);
    pwmBR = constrain(t - po - ro - yo, Cfg::MOTOR_MIN, Cfg::MOTOR_MAX);

    motorWrite(LEDC_FL, pwmFL);
    motorWrite(LEDC_FR, pwmFR);
    motorWrite(LEDC_BL, pwmBL);
    motorWrite(LEDC_BR, pwmBR);
}

// ============================================================
//  FAILSAFE
// ============================================================
void failsafe() {
    ctrl = CtrlInput{};   // zero everything, armed = false
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    motorsStop();
}

inline bool isFailsafe() {
    return (millis() - lastPktMs) > Cfg::FAILSAFE_MS;
}

// ============================================================
//  COMMAND PARSER
//  Format: "T:0.500,R:0.000,P:0.000,Y:0.000,A:1"
// ============================================================
void parseCmd(const char* buf) {
    float t, r, p, y;  int a;
    if (sscanf(buf, "T:%f,R:%f,P:%f,Y:%f,A:%d", &t, &r, &p, &y, &a) == 5) {
        ctrl.throttle = constrain(t,  0.0f, 1.0f);
        ctrl.roll     = constrain(r, -1.0f, 1.0f);
        ctrl.pitch    = constrain(p, -1.0f, 1.0f);
        ctrl.yaw      = constrain(y, -1.0f, 1.0f);
        ctrl.armed    = (a == 1);
        lastPktMs     = millis();
    }
}

// ============================================================
//  WIFI — Access Point + WebSocket + UDP
// ============================================================
void wsEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type,
             void*, uint8_t* data, size_t len) {
    if (type == WS_EVT_DATA && len > 0) {
        char buf[64] = {0};
        memcpy(buf, data, min(len, sizeof(buf) - 1));
        parseCmd(buf);
    }
}

void networkInit() {
    WiFi.softAP(Net::SSID, Net::PASSWORD);
    Serial.printf("[WiFi] AP '%s'  IP %s\n", Net::SSID,
                  WiFi.softAPIP().toString().c_str());

    // Serve browser control page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", HTML);
    });

    // WebSocket for real-time control from browser
    ws.onEvent(wsEvent);
    server.addHandler(&ws);
    server.begin();
    Serial.println("[HTTP] Server started  → open 192.168.4.1 on phone");

    // UDP — for external apps (e.g. RoboRemo, custom Python script)
    if (udp.listen(Net::UDP_PORT)) {
        udp.onPacket([](AsyncUDPPacket pkt) {
            char buf[64] = {0};
            memcpy(buf, pkt.data(), min((size_t)pkt.length(), sizeof(buf) - 1));
            parseCmd(buf);
        });
        Serial.printf("[UDP] Listening on port %u\n", Net::UDP_PORT);
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(600);
    Serial.println(F("\n╔═══════════════════════════════╗"));
    Serial.println(F("║  ESP32-C3 Flight Controller   ║"));
    Serial.println(F("╚═══════════════════════════════╝"));

    Wire.begin(Pin::SDA, Pin::SCL);
    Wire.setClock(400000);   // 400 kHz fast mode

    if (!imuInit()) {
        Serial.println(F("[FATAL] MPU6050 not found — check wiring, then restart."));
        for (;;) delay(1000);
    }
    delay(100);
    imuCalibrate();

    escInit();
    networkInit();

    Serial.println(F("\n[READY] Connect phone to WiFi: DroneAP / drone1234"));
    Serial.println(F("        Open browser → http://192.168.4.1"));
    Serial.println(F("        ARM the drone on screen, then increase throttle slowly\n"));
}

// ============================================================
//  MAIN LOOP — fixed-rate 500 Hz control loop
// ============================================================
void loop() {
    static uint32_t lastUs    = micros();
    static uint32_t lastDbgMs = millis();

    // ── Rate limiter ──────────────────────────────────────────
    uint32_t nowUs = micros();
    float    dt    = (nowUs - lastUs) * 1e-6f;
    if (dt < Cfg::LOOP_DT) return;
    lastUs = nowUs;

    // Guard against huge dt spikes (e.g. first loop, blocking calls)
    dt = constrain(dt, 0.0005f, 0.005f);

    // ── Read IMU ──────────────────────────────────────────────
    Vec3 accel, gyro;
    imuReadBurst(accel, gyro);

    // ── Attitude estimation ───────────────────────────────────
    updateAttitude(accel, gyro, dt);

    // ── Failsafe: no signal → disarm ─────────────────────────
    if (isFailsafe()) {
        failsafe();
        return;
    }

    // ── Disarmed: idle motors, reset integrators ──────────────
    if (!ctrl.armed) {
        motorsStop();
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
        return;
    }

    // ── Crash detection: extreme tilt → emergency stop ────────
    if (fabsf(attitude.roll)  > Cfg::CRASH_DEG ||
        fabsf(attitude.pitch) > Cfg::CRASH_DEG) {
        Serial.printf("[CRASH] Tilt R:%.1f P:%.1f — motors killed!\n",
                      attitude.roll, attitude.pitch);
        failsafe();
        return;
    }

    // ── Compute setpoints from pilot input ────────────────────
    float spRoll  = ctrl.roll  * Cfg::MAX_TILT_DEG;
    float spPitch = ctrl.pitch * Cfg::MAX_TILT_DEG;
    float spYaw   = ctrl.yaw   * Cfg::MAX_YAW_DPS;

    // ── PID ───────────────────────────────────────────────────
    //  Roll & Pitch: angle mode  (setpoint = desired angle °)
    //  Yaw:          rate mode   (setpoint = desired rotation rate °/s)
    float outRoll  = pidRoll.update (spRoll,  attitude.roll,  dt);
    float outPitch = pidPitch.update(spPitch, attitude.pitch, dt);
    float outYaw   = pidYaw.update  (spYaw,   gyro.z,         dt);

    // ── Motor mixing & write ──────────────────────────────────
    mixAndWrite(ctrl.throttle, outRoll, outPitch, outYaw);

    // ── WebSocket telemetry (to browser display) ──────────────
    //  Broadcast at ~50 Hz (every 10 control loops) — not every loop to save CPU
    static uint8_t tele_div = 0;
    if (++tele_div >= 10) {
        tele_div = 0;
        char msg[48];
        snprintf(msg, sizeof(msg), "R:%.1f P:%.1f Y:%.1f T:%d%%",
                 attitude.roll, attitude.pitch, attitude.yaw,
                 (int)(ctrl.throttle * 100));
        ws.textAll(msg);
        ws.cleanupClients();
    }

    // ── Serial telemetry (every 500 ms) ──────────────────────
    uint32_t nowMs = millis();
    if (nowMs - lastDbgMs >= 500) {
        lastDbgMs = nowMs;
        Serial.printf("R:%5.1f P:%5.1f Y:%6.1f | T:%.2f | FL:%4d FR:%4d BL:%4d BR:%4d\n",
                      attitude.roll, attitude.pitch, attitude.yaw,
                      ctrl.throttle, pwmFL, pwmFR, pwmBL, pwmBR);
    }
}
