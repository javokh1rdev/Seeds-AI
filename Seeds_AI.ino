#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include <ESP32Servo.h>
#include <HX711.h>
#include <Preferences.h>

// ======================= SOZLAMALAR =======================
#define AP_SSID        "UrugSaralash_CAM"
#define AP_PASS        "12345678"

#define SERVO_PIN      14
#define HX_DT_PIN      15
#define HX_SCK_PIN     13

#define SERVO_IDLE     90     // kutish / urug' yo'q
#define SERVO_OK       130    // mos keldi
#define SERVO_BAD      50     // mos kelmadi

#define HX_CAL_FACTOR  420.0f // HX711 kalibrovka koeffitsiyenti (o'zingizning datchikka moslang)

// Kadr o'lchami QVGA (320x240). ROI - kadr markazidagi kvadrat (piksel).
#define FRAME_W        320
#define FRAME_H        240
#define ROI_SIDE_PX    96
#define ROI_RING_PX    5      // ROI chetidagi fon namunasi chizig'i qalinligi

// Tahlil sozlamalari
#define ANALYSIS_MS    250    // tahlil davri
#define FG_THRESH      70     // fondan farq (|dR|+|dG|+|dB|) shu qiymatdan katta bo'lsa - urug' pikseli
#define MIN_PRESENT_PCT 1.2f  // ROI ning shuncha foizidan kam bo'lsa - urug' yo'q
#define PRESENT_N      3      // urug' barqaror deb hisoblash uchun ketma-ket tahlillar soni
#define ABSENT_N       4      // urug' yo'q deb hisoblash uchun ketma-ket tahlillar soni

#define HOLD_MS        5000   // servo 50/130 burchakda shuncha millisekund turadi, so'ng 90 ga qaytadi
#define MIN_COMPACT    0.45f  // urug' piksellari ixcham dog' bo'lishi kerak (tarqoq shovqin urug' emas)
#define STABLE_N       6      // o'lchov shuncha ketma-ket tahlilda barqaror bo'lgach servo ishlaydi (~1.5 s)
#define SETTLE_MS      1500   // servo 90 ga qaytgach kamera/yorug'lik barqarorlashishi uchun kutish

// Profilga yaqinlik bahosi (kichik = yaqinroq). Masofa uch qismdan iborat:
//   rang (RGB masofasi / SCORE_COLOR), hajm (ln nisbat / SCORE_AREA), shakl (cho'zinchoqlik farqi / SCORE_ELONG)
#define SCORE_COLOR    40.0f
#define SCORE_AREA     0.45f
#define SCORE_ELONG    0.6f
#define SCORE_REJECT   2.5f   // eng yaqin profilgacha ham shundan uzoq bo'lsa - hech biriga mos emas

// fmt2rgb888 chiqishi odatda BGR tartibida. Agar ranglar teskari chiqsa 0 qiling.
#define BUF_IS_BGR     1

// Kamera barqarorlik sozlamalari
#define CAM_XCLK_HZ    16000000   // qotsa 10000000 ga tushiring
#define CAM_JPEG_Q     14         // kattaroq son = kichikroq kadr, barqarorroq (10..20)

// Urug' profillari. Boshlang'ich qiymatlar taxminiy - kalibrovka qiling.
struct SeedProfile {
  const char *name;
  float areaPct;     // ROI ichidagi hajm (%)
  uint8_t r, g, b;   // o'rtacha rang
  float elong;       // cho'zinchoqlik (uzun o'q / qisqa o'q), 1 = dumaloq
};
static SeedProfile PROFILES[3] = {
  { "Tarvuz",  6.0f,  60,  45,  35, 1.8f },
  { "Qovun",   5.0f, 200, 175, 120, 2.6f },
  { "Bodring", 3.0f, 225, 215, 175, 3.0f }
};

// Kalibrovka natijasi flash (NVS) xotiraga saqlanadi va qayta yoqilganda yuklanadi.
struct CalData {
  float areaPct, elong;
  uint8_t r, g, b;
};
static Preferences prefs;

static void loadProfiles() {
  if (!prefs.begin("urug", true)) return;
  for (int i = 0; i < 3; i++) {
    char key[8];
    snprintf(key, sizeof(key), "q%d", i);
    CalData c;
    if (prefs.getBytesLength(key) == sizeof(CalData) &&
        prefs.getBytes(key, &c, sizeof(c)) == sizeof(c)) {
      PROFILES[i].areaPct = c.areaPct;
      PROFILES[i].elong = c.elong;
      PROFILES[i].r = c.r;
      PROFILES[i].g = c.g;
      PROFILES[i].b = c.b;
    }
  }
  prefs.end();
}

static void saveProfile(int i) {
  if (i < 0 || i > 2) return;
  if (!prefs.begin("urug", false)) return;
  CalData c;
  c.areaPct = PROFILES[i].areaPct;
  c.elong = PROFILES[i].elong;
  c.r = PROFILES[i].r;
  c.g = PROFILES[i].g;
  c.b = PROFILES[i].b;
  char key[8];
  snprintf(key, sizeof(key), "q%d", i);
  prefs.putBytes(key, &c, sizeof(c));
  prefs.end();
}

// idx-chi profilga yaqinlik bahosi (kichik = yaqinroq)
static float profileScore(int idx, float area, float r, float g, float b, float elong) {
  const SeedProfile &p = PROFILES[idx];
  float dr = r - p.r, dg = g - p.g, db = b - p.b;
  float cd = sqrtf(dr * dr + dg * dg + db * db) / SCORE_COLOR;
  float ad = logf(fmaxf(area, 0.01f) / fmaxf(p.areaPct, 0.01f)) / SCORE_AREA;
  float ed = (elong - p.elong) / SCORE_ELONG;
  return sqrtf(cd * cd + ad * ad + ed * ed);
}

// ======================= KAMERA PINLARI (AI-Thinker) =======================
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ======================= GLOBAL HOLAT =======================
struct Shared {
  bool  present;
  float areaPx;
  float areaPct;
  int   r, g, b;
  int   servo;
  float weight;
  bool  hxOk;
  bool  camOk;
  char  status[112];
};

static Shared g;
static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int selType = 0;   // 0 = tanlanmagan, 1 = tarvuz, 2 = qovun, 3 = bodring
static volatile bool calibReq = false;

// Kamera faqat bitta task (camTask) orqali o'qiladi. Oxirgi JPEG kadr shu yerda saqlanadi,
// oqim va tahlil undan nusxa oladi - shu tufayli kamera drayveri hech qachon qotib qolmaydi.
#define JPG_BUF_MAX (96 * 1024)
static volatile uint32_t streamFrames = 0;   // oqimga jo'natilgan kadrlar soni
static uint8_t *frameBuf = NULL;
static size_t frameLen = 0;
static volatile uint32_t frameSeq = 0;
static SemaphoreHandle_t frameMutex = NULL;

static Servo servo;
static HX711 hx;
static bool camOk = false;
static bool hxOk  = false;
static int  servoAngle = -1;

static httpd_handle_t httpMain   = NULL;
static httpd_handle_t httpStream = NULL;

// ======================= VEB SAHIFA =======================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="uz"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Urug' saralash</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#101418;color:#e8eef2;font-family:system-ui,Arial,sans-serif;display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:12px}
h1{font-size:18px;margin:4px 0 10px}
#wrap{position:relative;width:100%;max-width:480px;aspect-ratio:4/3;background:#000;border-radius:10px;overflow:hidden}
#cam{width:100%;height:100%;display:block;object-fit:fill}
#roi{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);width:30%;height:40%;border:2px solid #00e676;box-shadow:0 0 0 9999px rgba(0,0,0,.35);pointer-events:none}
#wt{position:fixed;top:10px;right:10px;background:rgba(0,0,0,.75);border:1px solid #2a3540;padding:6px 10px;border-radius:8px;font-weight:700;font-size:15px;z-index:5}
#btns{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;width:100%;max-width:480px;margin-top:12px}
.btn{padding:14px 4px;font-size:16px;font-weight:700;border:0;border-radius:10px;background:#26323d;color:#e8eef2;cursor:pointer}
.btn.active{background:#00e676;color:#000}
#st{margin-top:12px;font-size:13px;color:#b9c6d0;text-align:center;max-width:480px;min-height:18px}
#info{margin-top:4px;font-size:12px;color:#8fa0ad;text-align:center}
#sw{display:inline-block;width:10px;height:10px;border-radius:2px;border:1px solid #555;vertical-align:middle;margin-right:3px;background:#000}
</style></head><body>
<div id="wt">0.0 g</div>
<h1>Urug' saralash</h1>
<div id="wrap"><img id="cam" alt=""><div id="roi"></div></div>
<div id="btns">
<button class="btn" id="b1" onclick="pick(1)">Tarvuz</button>
<button class="btn" id="b2" onclick="pick(2)">Qovun</button>
<button class="btn" id="b3" onclick="pick(3)">Bodring</button>
</div>
<button class="btn" style="margin-top:10px;font-size:13px;padding:10px 14px;background:#3a4a5a" onclick="calib()">Kalibrovka: shu urug'ni o'rganish</button>
<div id="st">Ulanmoqda...</div>
<div id="info">Hajm: - | RGB: - | Servo: -</div>
<script>
function $(i){return document.getElementById(i)}
function startStream(){$('cam').src='http://'+location.hostname+':81/stream?'+Date.now()}
var lastSf=-1,lastMove=Date.now();
$('cam').onerror=function(){setTimeout(startStream,2000)};
function mark(s){for(var i=1;i<=3;i++){$('b'+i).className='btn'+(i===s?' active':'')}}
function pick(t){mark(t);fetch('/select?t='+t,{cache:'no-store'}).catch(function(){})}
function calib(){fetch('/calib',{cache:'no-store'}).catch(function(){})}
function poll(){
  fetch('/data',{cache:'no-store'}).then(function(r){return r.json()}).then(function(d){
    $('wt').textContent=d.hx?(d.w.toFixed(1)+' g'):'HX711 yo\'q';
    $('roi').style.width=d.roiw+'%';$('roi').style.height=d.roih+'%';
    mark(d.sel);
    if(d.sf!==lastSf){lastSf=d.sf;lastMove=Date.now()}
    else if(d.cam&&Date.now()-lastMove>3000){lastMove=Date.now();startStream()}
    $('st').textContent=d.cam?d.status:'Kamera xatosi';
    var v=d.present?(d.px+' px ('+d.pct.toFixed(1)+'%)'):'-';
    var c=d.present?(d.r+','+d.g+','+d.b):'-';
    $('info').innerHTML='Hajm: '+v+' | RGB: <span id="sw" style="background:'+(d.present?'rgb('+c+')':'#000')+'"></span>'+c+' | Servo: '+d.servo+'°';
  }).catch(function(){}).then(function(){setTimeout(poll,500)});
}
startStream();poll();
</script></body></html>)rawliteral";

// ======================= KAMERA =======================
static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_7;   // servo PWM bilan to'qnashmasligi uchun
  config.ledc_timer   = LEDC_TIMER_3;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = CAM_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;     // 320x240
  config.jpeg_quality = CAM_JPEG_Q;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  for (int attempt = 1; attempt <= 3; attempt++) {
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) return true;
    Serial.printf("Kamera xatosi 0x%x (urinish %d)\n", err, attempt);
    esp_camera_deinit();
    delay(300);
  }
  return false;
}

// ======================= TAHLIL =======================
struct Meas {
  bool  present;
  float areaPx;
  float areaPct;
  float r, g, b;
  float elong;
  float compact;     // ixchamlik (1 ga yaqin = bitta zich dog', kichik = tarqoq shovqin)
};
static Meas gMeas;   // natija global o'zgaruvchida (Arduino prototip muammosini oldini oladi)

// Kadr markazidagi ROI ni tahlil qiladi. Kadrni kamera drayveridan emas, camTask nusxasidan oladi.
static bool measureROI() {
  Meas &m = gMeas;
  m.present = false; m.areaPx = 0; m.areaPct = 0; m.r = m.g = m.b = 0; m.elong = 1; m.compact = 0;

  if (!frameBuf || !frameMutex) return false;
  static uint8_t *anaBuf = NULL;     // JPEG nusxasi (bir marta ajratiladi, oqish yo'q)
  if (!anaBuf) {
    anaBuf = (uint8_t *)ps_malloc(JPG_BUF_MAX);
    if (!anaBuf) anaBuf = (uint8_t *)malloc(JPG_BUF_MAX);
    if (!anaBuf) return false;
  }

  xSemaphoreTake(frameMutex, portMAX_DELAY);
  size_t jpgLen = frameLen;
  if (jpgLen > 0) memcpy(anaBuf, frameBuf, jpgLen);
  xSemaphoreGive(frameMutex);
  if (jpgLen == 0) return false;

  const int W = FRAME_W;
  const int H = FRAME_H;
  const size_t need = (size_t)W * H * 3;

  uint8_t *rgb = (uint8_t *)ps_malloc(need);
  if (!rgb) rgb = (uint8_t *)malloc(need);
  if (!rgb) return false;

  bool ok = fmt2rgb888(anaBuf, jpgLen, PIXFORMAT_JPEG, rgb);
  if (!ok) {
    free(rgb);
    return false;
  }

  int side = ROI_SIDE_PX;
  if (side > W) side = W;
  if (side > H) side = H;
  const int x0 = (W - side) / 2;
  const int y0 = (H - side) / 2;
  const int x1 = x0 + side;
  const int y1 = y0 + side;

  // 1) Fon rangi: ROI chetidagi chiziq o'rtachasi
  uint32_t bgR = 0, bgG = 0, bgB = 0, bgN = 0;
  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      bool ring = (x < x0 + ROI_RING_PX) || (x >= x1 - ROI_RING_PX) ||
                  (y < y0 + ROI_RING_PX) || (y >= y1 - ROI_RING_PX);
      if (!ring) continue;
      const uint8_t *p = rgb + ((size_t)y * W + x) * 3;
#if BUF_IS_BGR
      bgB += p[0]; bgG += p[1]; bgR += p[2];
#else
      bgR += p[0]; bgG += p[1]; bgB += p[2];
#endif
      bgN++;
    }
  }
  if (bgN == 0) { free(rgb); return false; }
  const int bR = bgR / bgN, bG = bgG / bgN, bB = bgB / bgN;

  // 2) Ichki qism: fondan farq qiluvchi piksellar = urug' (rang va shakl momentlari bilan)
  uint32_t cnt = 0, inner = 0;
  uint32_t sR = 0, sG = 0, sB = 0;
  uint32_t sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
  for (int y = y0 + ROI_RING_PX; y < y1 - ROI_RING_PX; y++) {
    for (int x = x0 + ROI_RING_PX; x < x1 - ROI_RING_PX; x++) {
      const uint8_t *p = rgb + ((size_t)y * W + x) * 3;
#if BUF_IS_BGR
      int B = p[0], G = p[1], R = p[2];
#else
      int R = p[0], G = p[1], B = p[2];
#endif
      inner++;
      int d = abs(R - bR) + abs(G - bG) + abs(B - bB);
      if (d > FG_THRESH) {
        cnt++; sR += R; sG += G; sB += B;
        uint32_t lx = x - x0, ly = y - y0;
        sx += lx; sy += ly;
        sxx += lx * lx; syy += ly * ly; sxy += lx * ly;
      }
    }
  }
  free(rgb);
  if (inner == 0) return false;

  m.areaPx  = (float)cnt;
  m.areaPct = cnt * 100.0f / inner;
  m.present = (m.areaPct >= MIN_PRESENT_PCT) && cnt > 0;
  if (cnt > 0) {
    m.r = (float)sR / cnt;
    m.g = (float)sG / cnt;
    m.b = (float)sB / cnt;

    // Cho'zinchoqlik: kovariatsiya matritsasi xos qiymatlari nisbatining ildizi
    float mx = (float)sx / cnt, my = (float)sy / cnt;
    float vxx = (float)sxx / cnt - mx * mx;
    float vyy = (float)syy / cnt - my * my;
    float vxy = (float)sxy / cnt - mx * my;
    float tr = vxx + vyy;
    float det = vxx * vyy - vxy * vxy;
    float disc = sqrtf(fmaxf(0.0f, tr * tr * 0.25f - det));
    float l1 = tr * 0.5f + disc;
    float l2 = tr * 0.5f - disc;
    float e = sqrtf(fmaxf(l1, 0.25f) / fmaxf(l2, 0.25f));
    if (e < 1.0f) e = 1.0f;
    if (e > 8.0f) e = 8.0f;
    m.elong = e;

    // Ixchamlik: urug' piksellari soni / xuddi shu momentli ellips yuzi. Tarqoq shovqinda kichik bo'ladi.
    float ellArea = 4.0f * 3.14159265f * sqrtf(fmaxf(l1, 0.25f) * fmaxf(l2, 0.25f));
    m.compact = cnt / ellArea;
    if (m.compact < MIN_COMPACT) m.present = false;
  }
  return true;
}

static void setServo(int angle) {
  if (angle != servoAngle) {
    servo.write(angle);
    servoAngle = angle;
  }
}

// ======================= SARALASH LOGIKASI =======================
static void processSorting() {
  if (!measureROI()) return;
  const Meas &m = gMeas;
  static float eP = 0, eA = 0, eR = 0, eG = 0, eB = 0, eE = 1;
  static int presentN = 0, absentN = 0, lastSel = 0;
  // Saralash bosqichi: 0 = urug' kutilmoqda, 1 = servo natija burchagida ushlab turilibdi.
  // Ushlab turish tugagach servo 90 ga qaytadi va butun jarayon boshidan boshlanadi.
  static int phase = 0;
  static uint32_t holdUntil = 0;
  static uint32_t settleUntil = 0;
  static int stableN = 0;            // ketma-ket barqaror (bir-biriga yaqin) o'lchovlar soni
  static char holdMsg[112] = "";

  const int sel = selType;
  if (sel != lastSel) {
    lastSel = sel;
    presentN = 0;
    absentN = 0;
    stableN = 0;
    phase = 0;
    setServo(SERVO_IDLE);
  }

  if (m.present) {
    if (presentN == 0) {
      eA = m.areaPx; eP = m.areaPct; eR = m.r; eG = m.g; eB = m.b; eE = m.elong;
      stableN = 0;
    } else {
      // O'lchov oldingi o'rtachaga yaqinmi? (urug' qimirlamayapti, qo'l yoki soya yo'q)
      const float dr = m.r - eR, dg = m.g - eG, db = m.b - eB;
      const bool consistent = fabsf(m.areaPct - eP) <= 0.35f * eP &&
                              sqrtf(dr * dr + dg * dg + db * db) <= 35.0f &&
                              fabsf(m.elong - eE) <= 1.0f;
      if (consistent) {
        const float a = 0.4f;
        eA += a * (m.areaPx  - eA);
        eP += a * (m.areaPct - eP);
        eR += a * (m.r - eR);
        eG += a * (m.g - eG);
        eB += a * (m.b - eB);
        eE += a * (m.elong - eE);
        if (stableN < 1000) stableN++;
      } else {
        // o'lchov sakradi: qaytadan boshlaymiz
        eA = m.areaPx; eP = m.areaPct; eR = m.r; eG = m.g; eB = m.b; eE = m.elong;
        stableN = 0;
      }
    }
    if (presentN < 1000) presentN++;
    absentN = 0;
  } else {
    if (absentN < 1000) absentN++;
    presentN = 0;
    stableN = 0;
  }

  // Kalibrovka: tanlangan tur uchun joriy o'lchangan hajm, rang va shaklni profil qilib saqlash
  static uint32_t msgUntil = 0;
  static char msg[72] = "";
  if (calibReq) {
    calibReq = false;
    if (sel >= 1 && sel <= 3 && m.present && stableN >= STABLE_N) {
      SeedProfile &p = PROFILES[sel - 1];
      p.r = (uint8_t)(eR + 0.5f);
      p.g = (uint8_t)(eG + 0.5f);
      p.b = (uint8_t)(eB + 0.5f);
      p.areaPct = eP;
      p.elong = eE;
      saveProfile(sel - 1);
      snprintf(msg, sizeof(msg), "Kalibrovka saqlandi: %s", p.name);
    } else {
      snprintf(msg, sizeof(msg), "Kalibrovka: avval tur tanlang va urug'ni ramkaga qo'ying");
    }
    msgUntil = millis() + 2500;
  }

  char st[112];
  int target = servoAngle < 0 ? SERVO_IDLE : servoAngle;

  if (sel < 1 || sel > 3) {
    phase = 0;
    target = SERVO_IDLE;
    snprintf(st, sizeof(st), "Urug' turini tanlang (Tarvuz / Qovun / Bodring)");
  } else if (phase == 1) {
    // Servo natija burchagida ushlab turiladi (urug' sirpanib ketsa ham vaqt tugaguncha shu yerda)
    if ((int32_t)(millis() - holdUntil) >= 0) {
      phase = 0;           // hammasi boshidan boshlanadi
      presentN = 0;
      absentN = 0;
      stableN = 0;
      settleUntil = millis() + SETTLE_MS;
      target = SERVO_IDLE;
      snprintf(st, sizeof(st), "Servo 90 ga qaytdi - kamera barqarorlashmoqda...");
    } else {
      int left = (int)((holdUntil - millis() + 999) / 1000);
      target = servoAngle;
      snprintf(st, sizeof(st), "[%d s] %s", left, holdMsg);
    }
  } else if ((int32_t)(settleUntil - millis()) > 0) {
    // Servo qaytgach qisqa tinchlanish: yorug'lik/kadr o'zgarishi urug' deb qabul qilinmasin
    presentN = 0;
    absentN = 0;
    stableN = 0;
    target = SERVO_IDLE;
    snprintf(st, sizeof(st), "Kamera barqarorlashmoqda...");
  } else if (!m.present) {
    if (absentN >= ABSENT_N) target = SERVO_IDLE;
    snprintf(st, sizeof(st), "Urug'ni yashil ramka ichiga qo'ying - kutilmoqda");
  } else if (stableN < STABLE_N) {
    snprintf(st, sizeof(st), "Urug' tahlil qilinmoqda...");
  } else {
    // Uchala profil bilan solishtirib, eng yaqinini topamiz
    float sc[3];
    int best = 0;
    for (int i = 0; i < 3; i++) {
      sc[i] = profileScore(i, eP, eR, eG, eB, eE);
      if (sc[i] < sc[best]) best = i;
    }
    const int si = sel - 1;
    const bool match = (best == si) && (sc[si] <= SCORE_REJECT);

    static uint32_t lastDbg = 0;
    if (millis() - lastDbg > 1000) {
      lastDbg = millis();
      Serial.printf("hajm=%.1f%% rgb=%d,%d,%d shakl=%.2f | ball T=%.2f Q=%.2f B=%.2f\n",
                    eP, (int)eR, (int)eG, (int)eB, eE, sc[0], sc[1], sc[2]);
    }

    if (match) {
      target = SERVO_OK;
      snprintf(st, sizeof(st), "%s urug'i yaroqli - mos keldi", PROFILES[si].name);
    } else {
      target = SERVO_BAD;
      const bool looksLike = (best != si) && (sc[best] <= SCORE_REJECT);
      if (sel == 1) {
        if (looksLike) {
          snprintf(st, sizeof(st), "Chirigan yoki mos kelmagan tarvuz urug'i - yaroqli emas (%s urug'iga o'xshaydi)", PROFILES[best].name);
        } else {
          snprintf(st, sizeof(st), "Chirigan yoki mos kelmagan tarvuz urug'i - yaroqli emas");
        }
      } else {
        if (looksLike) {
          snprintf(st, sizeof(st), "%s urug'i mos kelmadi - yaroqli emas (%s urug'iga o'xshaydi)", PROFILES[si].name, PROFILES[best].name);
        } else {
          snprintf(st, sizeof(st), "%s urug'i mos kelmadi - yaroqli emas", PROFILES[si].name);
        }
      }
    }

    // Natija burchagiga o'tamiz va HOLD_MS ushlab turamiz
    phase = 1;
    holdUntil = millis() + HOLD_MS;
    snprintf(holdMsg, sizeof(holdMsg), "%s", st);
  }

  if (millis() < msgUntil) snprintf(st, sizeof(st), "%s", msg);

  setServo(target);

  portENTER_CRITICAL(&gMux);
  g.present = m.present;
  g.areaPx  = m.present ? eA : 0;
  g.areaPct = m.present ? eP : 0;
  g.r = m.present ? (int)(eR + 0.5f) : 0;
  g.g = m.present ? (int)(eG + 0.5f) : 0;
  g.b = m.present ? (int)(eB + 0.5f) : 0;
  g.servo = servoAngle;
  strncpy(g.status, st, sizeof(g.status) - 1);
  g.status[sizeof(g.status) - 1] = '\0';
  portEXIT_CRITICAL(&gMux);
}

// ======================= HTTP HANDLERLAR =======================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t data_handler(httpd_req_t *req) {
  Shared s;
  int sel;
  portENTER_CRITICAL(&gMux);
  s = g;
  portEXIT_CRITICAL(&gMux);
  sel = selType;

  const float roiW = ROI_SIDE_PX * 100.0f / FRAME_W;
  const float roiH = ROI_SIDE_PX * 100.0f / FRAME_H;

  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"w\":%.1f,\"hx\":%d,\"cam\":%d,\"present\":%d,\"px\":%.0f,\"pct\":%.1f,"
           "\"r\":%d,\"g\":%d,\"b\":%d,\"servo\":%d,\"sel\":%d,\"roiw\":%.1f,\"roih\":%.1f,"
           "\"sf\":%u,\"status\":\"%s\"}",
           s.weight, s.hxOk ? 1 : 0, s.camOk ? 1 : 0, s.present ? 1 : 0, s.areaPx, s.areaPct,
           s.r, s.g, s.b, s.servo, sel, roiW, roiH, (unsigned)streamFrames, s.status);

  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t select_handler(httpd_req_t *req) {
  char query[32];
  char val[8];
  int t = 0;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    if (httpd_query_key_value(query, "t", val, sizeof(val)) == ESP_OK) {
      t = atoi(val);
    }
  }
  if (t < 0 || t > 3) t = 0;
  selType = t;
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t calib_handler(httpd_req_t *req) {
  calibReq = true;
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CT  = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_HDR = "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  if (!camOk || !frameBuf || !frameMutex) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Kamera ishlamayapti");
    return ESP_FAIL;
  }

  uint8_t *local = (uint8_t *)ps_malloc(JPG_BUF_MAX);   // shu ulanish uchun nusxa buferi
  if (!local) local = (uint8_t *)malloc(JPG_BUF_MAX);
  if (!local) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Xotira yetarli emas");
    return ESP_FAIL;
  }

  esp_err_t res = httpd_resp_set_type(req, STREAM_CT);
  if (res == ESP_OK) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char hdr[128];
    uint32_t lastSeq = 0;
    uint32_t waitStart = millis();

    while (true) {
      if (frameSeq == lastSeq) {                 // yangi kadr hali yo'q
        if (millis() - waitStart > 4000) { res = ESP_FAIL; break; }
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }

      xSemaphoreTake(frameMutex, portMAX_DELAY);
      size_t len = frameLen;
      if (len > 0) memcpy(local, frameBuf, len);
      lastSeq = frameSeq;
      xSemaphoreGive(frameMutex);
      waitStart = millis();
      if (len == 0) continue;

      size_t hl = snprintf(hdr, sizeof(hdr), STREAM_HDR, (unsigned)len);
      res = httpd_resp_send_chunk(req, hdr, hl);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)local, len);
      if (res == ESP_OK) streamFrames++;
      if (res != ESP_OK) break;                  // mijoz uzildi

      vTaskDelay(pdMS_TO_TICKS(40));             // ~25 FPS chegarasi
    }
  }

  free(local);
  return res;
}

// Kamerani o'qiydigan yagona task. Kadrni darhol nusxalab, drayverga qaytaradi.
// Kamera javob bermasa, o'zi qayta ishga tushiradi.
static void camTask(void *arg) {
  int fails = 0;
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      if (++fails >= 3) {
        Serial.println("Kamera javob bermadi - qayta ishga tushirilmoqda");
        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(300));
        if (!initCamera()) {
          Serial.println("Kamera tiklanmadi - qayta yuklanmoqda");
          delay(200);
          ESP.restart();
        }
        fails = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    fails = 0;

    if (fb->format == PIXFORMAT_JPEG && fb->len > 0 && fb->len <= JPG_BUF_MAX) {
      xSemaphoreTake(frameMutex, portMAX_DELAY);
      memcpy(frameBuf, fb->buf, fb->len);
      frameLen = fb->len;
      frameSeq++;
      xSemaphoreGive(frameMutex);
    }
    esp_camera_fb_return(fb);
    vTaskDelay(1);
  }
}

static void registerGet(httpd_handle_t server, const char *uri, esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t u;
  memset(&u, 0, sizeof(u));
  u.uri = uri;
  u.method = HTTP_GET;
  u.handler = handler;
  u.user_ctx = NULL;
  httpd_register_uri_handler(server, &u);
}

static void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.lru_purge_enable = true;
  cfg.max_uri_handlers = 8;
  if (httpd_start(&httpMain, &cfg) == ESP_OK) {
    registerGet(httpMain, "/", index_handler);
    registerGet(httpMain, "/data", data_handler);
    registerGet(httpMain, "/select", select_handler);
    registerGet(httpMain, "/calib", calib_handler);
  } else {
    Serial.println("Asosiy server ishga tushmadi");
  }

  httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
  scfg.server_port = 81;
  scfg.ctrl_port = scfg.ctrl_port + 1;
  scfg.lru_purge_enable = true;
  scfg.max_open_sockets = 3;
  scfg.send_wait_timeout = 3;      // uzilgan mijoz oqimni uzoq ushlab turmasin
  scfg.stack_size = 8192;
  if (httpd_start(&httpStream, &scfg) == ESP_OK) {
    registerGet(httpStream, "/stream", stream_handler);
  } else {
    Serial.println("Stream server ishga tushmadi");
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nUrug' saralash - ishga tushmoqda");
  loadProfiles();
  memset(&g, 0, sizeof(g));
  strncpy(g.status, "Ishga tushmoqda...", sizeof(g.status) - 1);
  g.servo = SERVO_IDLE;

  // 1) Kamera (servodan oldin - LEDC to'qnashuvining oldini olish uchun)
  camOk = initCamera();
  if (camOk) {
    frameMutex = xSemaphoreCreateMutex();
    frameBuf = (uint8_t *)ps_malloc(JPG_BUF_MAX);
    if (!frameBuf) frameBuf = (uint8_t *)malloc(JPG_BUF_MAX);
    if (!frameMutex || !frameBuf) {
      camOk = false;
      Serial.println("Kadr buferi uchun xotira yetarli emas");
    } else {
      xTaskCreatePinnedToCore(camTask, "camTask", 8192, NULL, 2, NULL, 1);
    }
  }
  g.camOk = camOk;
  if (!camOk) {
    strncpy(g.status, "Kamera ishga tushmadi", sizeof(g.status) - 1);
    Serial.println("Kamera ishga tushmadi");
  }

  // 2) Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  setServo(SERVO_IDLE);
  g.servo = SERVO_IDLE;

  // 3) HX711 (ulanmagan bo'lsa ham dastur qotib qolmaydi)
  hx.begin(HX_DT_PIN, HX_SCK_PIN);
  if (hx.wait_ready_timeout(1500, 50)) {
    hx.set_scale(HX_CAL_FACTOR);
    hx.tare(10);
    hxOk = true;
  } else {
    Serial.println("HX711 topilmadi");
  }
  g.hxOk = hxOk;

  // 4) Wi-Fi Access Point
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  if (!WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4)) {
    Serial.println("softAP xatosi");
  }
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_15dBm);   // oqim kamaytirish: brownout xavfini pasaytiradi
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // 5) Veb-serverlar
  startServers();

  if (camOk) {
    strncpy(g.status, "Urug' turini tanlang (Tarvuz / Qovun / Bodring)", sizeof(g.status) - 1);
  }
}

void loop() {
  static uint32_t lastAnalysis = 0;
  static float wEma = 0;
  static bool wInit = false;

  // Vazn (bloklanmasligi uchun faqat tayyor bo'lganda o'qiladi)
  if (hxOk && hx.is_ready()) {
    float u = hx.get_units(1);
    if (isfinite(u)) {
      wEma = wInit ? (wEma * 0.7f + u * 0.3f) : u;
      wInit = true;
      float shown = (fabsf(wEma) < 0.3f) ? 0.0f : wEma;
      portENTER_CRITICAL(&gMux);
      g.weight = shown;
      portEXIT_CRITICAL(&gMux);
    }
  }

  // Kamera tahlili
  uint32_t now = millis();
  if (camOk && (now - lastAnalysis >= ANALYSIS_MS)) {
    lastAnalysis = now;
    processSorting();
  }

  // Diagnostika: Serial Monitor (115200) da har 3 soniyada holat
  static uint32_t lastDbg = 0;
  if (now - lastDbg >= 3000) {
    lastDbg = now;
    Serial.printf("kadr=%u oqim=%u heap=%u psram=%u mijoz=%d\n",
                  (unsigned)frameSeq, (unsigned)streamFrames,
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
                  WiFi.softAPgetStationNum());
  }

  delay(5);
}
