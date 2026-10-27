#include <mcp_can.h>
#include <SPI.h>
#include <EEPROM.h>

// ============================================================
//  Блок L (левая дверь)
//  - читает джойстик, шлёт флаги правому зеркалу (0x20)
//  - слушает разрешения от M (0x10)
//  - двигает левое зеркало через L298N (IN1..IN4 = пины 4..7)
// ============================================================

// ===================== пины и CAN =====================
#define CS1           10
#define INT1          2
#define SERIAL_BAUD   500000
#define M_ID          0x10UL
#define TX_ID         0x20UL
#define TX_PERIOD_MS  40
#define M_TIMEOUT_MS  500
#define MCP_BITMOD    0x05
#define REG_CANCTRL   0x0F

#define PIN_POT_V     A6
#define PIN_POT_H     A7

MCP_CAN CAN1(CS1);

// ===================== параметры =====================
const unsigned long SAVE_DELAY_MS   = 5000;   // ожидание перед записью Last/MR
const unsigned long AUTO_TIMEOUT_MS = 12000;  // таймаут движения одной оси
const unsigned long STALL_MS        = 500;    // нет изменения потенциометра
const unsigned long SENS_MS         = 10;     // период опроса потенциометров
const unsigned long CAN_RETRY_MS    = 1000;   // повтор инициализации MCP2515
const unsigned long STABLE_MS       = 3000;   // устойчивая работа для сброса аварии
const unsigned long REPEAT_WINDOW_MS= 600000; // повтор аварии в этом окне блокирует сброс

const uint8_t  TOL        = 5;     // допуск достижения цели (0..255)
const uint8_t  CHANGE_TH  = 5;     // порог "значение изменилось"
const uint8_t  DEADBAND   = 5;     // допустимый откат против направления
const uint8_t  MAX_STEP   = 15;    // макс. изменение за период опроса
const uint16_t RAW_MIN    = 20;    // сырой АЦП: ниже - обрыв/КЗ
const uint16_t RAW_MAX    = 1003;  // выше - обрыв/КЗ
const uint8_t  DEM_BAD    = 2;
const uint8_t  DEM_FAULT  = 10;

const uint8_t  FLAG_STABLE_N     = 3;  // одинаковых кадров M_ID подряд
const uint8_t  MAX_FRAMES_LOOP   = 4;  // кадров CAN за один проход цикла
const uint8_t  COUNTER_BYTE      = 2;  // счётчик кадра в M_ID

// MoveVertical(true) / MoveHorizontal(true): показание потенциометра растёт, если SIGN = +1
const int8_t SIGN_V = +1;          // true = UP
const int8_t SIGN_H = +1;          // true = RIGHT

// ===================== типы =====================
enum Slot  : uint8_t { S_M1, S_M2, S_LAST, S_MR };
enum State : uint8_t { ST_IDLE, ST_JOY, ST_GOTO_MR, ST_GOTO_M, ST_GOTO_LAST };
enum Cause : uint8_t { C_RANGE = 1, C_DIR, C_RATE, C_STALL, C_TIMEOUT };
enum Lvl   : uint8_t { L_GND, L_VCC, L_MID };
enum Motor : uint8_t { M_STOP, M_V_FWD, M_V_REV, M_H_FWD, M_H_REV };

const uint8_t AX_V = 0, AX_H = 1;

// ===================== привод левого зеркала (L298N) =====================
// IN1 (пин 4) - общий, IN2 (пин 5) - вертикаль, IN3 (пин 6) - горизонталь, IN4 (пин 7) = как IN1.
// Пины 4..7 = PD4..PD7: состояние пишется одной операцией в порт,
// режим хранится в одной переменной, поэтому два мотора одновременно включить нельзя.
//                                   STOP  V_FWD V_REV H_FWD H_REV
const uint8_t MOTOR_BITS[5] =      { 0x00, 0x20, 0xD0, 0x40, 0xB0 };
Motor motorMode = M_STOP;
bool  motorActive = false;

void setMotor(Motor m) {
  if (m == motorMode) return;
  motorMode = m;
  PORTD = (PORTD & 0x0F) | MOTOR_BITS[m];
}

void StopMotor()               { setMotor(M_STOP); }
void MoveVertical(bool dir)    { setMotor(dir ? M_V_FWD : M_V_REV); }
void MoveHorizontal(bool dir)  { setMotor(dir ? M_H_FWD : M_H_REV); }

// ===================== флаги из M_ID =====================
bool mSET, mSAVE1, mSAVE2, mM1, mM2, mSETCANCEL, mR, mP = true, mDN;

const uint16_t FLAGS_DEFAULT = 0x0080;   // только mP
uint16_t accF  = FLAGS_DEFAULT;          // принятые (устойчивые) флаги
uint16_t candF = 0;
uint8_t  candCnt = 0;
bool     haveCnt = false;
uint8_t  lastCnt = 0;
bool     haveRx = false, linkDown = false, linkLostNow = false;
bool     mLost = false;                  // M пропал, данные ещё не устойчивы
unsigned long lastRx_M = 0;

bool pR = false, pM1 = false, pM2 = false, pCancel = false;
bool pSave1 = false, pSave2 = false, prevPerm = true;

void applyFlags(uint16_t f) {
  mSET       = f & (1 << 0);
  mSAVE1     = f & (1 << 1);
  mSAVE2     = f & (1 << 2);
  mM1        = f & (1 << 3);
  mM2        = f & (1 << 4);
  mSETCANCEL = f & (1 << 5);
  mR         = f & (1 << 6);
  mP         = f & (1 << 7);
  mDN        = f & (1 << 8);
}

void resetPermissions() {
  accF = FLAGS_DEFAULT;
  applyFlags(accF);
  candCnt = 0;
  haveCnt = false;
}

// ===================== вывод =====================
// joy: бит 0 VR UP, 1 VR DOWN, 2 HR LEFT, 3 HR RIGHT, 4 признак HL/VL (для логики L)
// flags (0x20): 0..3 правое зеркало, 4 мотор левого зеркала, 5 авария или недостоверные данные M
uint8_t  joy = 0;
int8_t   joyAxis = -1;     // ось левого зеркала по HL/VL
bool     joyFwd = false;   // параметр для MoveVertical/MoveHorizontal
int8_t   joyExp = 0;       // ожидаемое направление потенциометра
uint16_t flags = 0;
unsigned long lastTx = 0;

// ===================== память (EEPROM) =====================
// Две копии по 16 байт: 0..7 данные (V,H для M1,M2,Last,MR), 8 версия,
// 9 статус (младшие 4 бита - валидность слотов, старшие - контрольная сумма).
const uint8_t COPY_SIZE = 16;
uint8_t mem[8];
uint8_t validMask = 0;
int8_t  newestIdx = -1;
uint8_t newestVer = 0;

uint8_t memCheck(const uint8_t *data, uint8_t valid, uint8_t ver) {
  uint8_t s = 0xA5;
  for (uint8_t i = 0; i < 8; i++) s += data[i];
  s += valid;
  s += ver;
  return (s ^ (s >> 4)) & 0x0F;
}

void loadMemory() {
  uint8_t d[2][8], ver[2], vm[2];
  bool ok[2];
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t base = i * COPY_SIZE;
    for (uint8_t j = 0; j < 8; j++) d[i][j] = EEPROM.read(base + j);
    ver[i] = EEPROM.read(base + 8);
    uint8_t stt = EEPROM.read(base + 9);
    vm[i] = stt & 0x0F;
    ok[i] = ((stt >> 4) == memCheck(d[i], vm[i], ver[i]));
  }
  int8_t pick = -1;
  if (ok[0] && ok[1]) pick = ((int8_t)(ver[1] - ver[0]) > 0) ? 1 : 0;
  else if (ok[0]) pick = 0;
  else if (ok[1]) pick = 1;

  if (pick < 0) {
    memset(mem, 0, sizeof(mem));
    validMask = 0; newestIdx = -1; newestVer = 0;
    Serial.println(F("MEM empty"));
    return;
  }
  memcpy(mem, d[pick], 8);
  validMask = vm[pick];
  newestIdx = pick;
  newestVer = ver[pick];
}

bool inRange(uint8_t v) { return v >= (RAW_MIN >> 2) && v <= (RAW_MAX >> 2); }

bool isValid(uint8_t s) {
  return (validMask & (1 << s)) && inRange(mem[s * 2]) && inRange(mem[s * 2 + 1]);
}

// порядок записи: данные, версия, статус (в неактуальную копию)
void writeCopy() {
  uint8_t idx = (newestIdx < 0) ? 0 : (1 - newestIdx);
  uint8_t base = idx * COPY_SIZE;
  uint8_t ver = newestVer + 1;
  for (uint8_t j = 0; j < 8; j++) EEPROM.update(base + j, mem[j]);
  EEPROM.update(base + 8, ver);
  EEPROM.update(base + 9, (memCheck(mem, validMask, ver) << 4) | validMask);
  newestIdx = idx;
  newestVer = ver;
}

void commitSlot(uint8_t s, uint8_t v, uint8_t h) {
  if ((validMask & (1 << s)) && mem[s * 2] == v && mem[s * 2 + 1] == h) return;
  mem[s * 2] = v;
  mem[s * 2 + 1] = h;
  validMask |= (1 << s);
  writeCopy();
  Serial.print(F("SAVE slot ")); Serial.print(s);
  Serial.print(F(": V=")); Serial.print(v);
  Serial.print(F(" H="));  Serial.println(h);
}

// ===================== состояние и контроль потенциометров =====================
State st = ST_IDLE;
bool fault = false, faultLocked = false, everReset = false;
unsigned long faultResetAt = 0;

uint8_t cur[2], prevVal[2], ref[2], startVal[2];
int8_t  expDir[2] = {0, 0};
bool    rangeOk = true;
uint8_t dem = 0, lastWhy = 0;
unsigned long lastSens = 0;

bool joySuccess = false;       // ручное движение с реальным изменением показаний
bool goodOn = false;
unsigned long goodSince = 0;

bool isGoto() { return st == ST_GOTO_MR || st == ST_GOTO_M || st == ST_GOTO_LAST; }

void clearDirs() { expDir[0] = expDir[1] = 0; }

void beginAxis(uint8_t ax) { ref[ax] = startVal[ax] = cur[ax]; }

bool readPots(uint8_t *val) {
  uint16_t r[2] = { (uint16_t)analogRead(PIN_POT_V), (uint16_t)analogRead(PIN_POT_H) };
  bool ok = true;
  for (uint8_t i = 0; i < 2; i++) {
    if (r[i] < RAW_MIN || r[i] > RAW_MAX) ok = false;
    val[i] = r[i] >> 2;
  }
  return ok;
}

// ===================== автодвижение (по одной оси: сначала V, потом H) =====================
Slot    autoSlot = S_LAST;
uint8_t tgt[2];
bool    axDone[2];
int8_t  autoAx = -1;
bool    autoFwd = false;
unsigned long axStart = 0, stallT = 0;
uint8_t stallRefVal = 0;

bool savePending = false;
Slot saveSel = S_LAST;
unsigned long saveAt = 0;
bool touchedLeft = false;

void saveCurrent(uint8_t s) {
  if (fault || !rangeOk) return;
  commitSlot(s, cur[AX_V], cur[AX_H]);
}

void flushSave() {
  if (!savePending) return;
  savePending = false;
  saveCurrent(saveSel);
}

void copyToLast(uint8_t sel) {
  if (fault) return;
  if (isValid(sel)) commitSlot(S_LAST, mem[sel * 2], mem[sel * 2 + 1]);
}

void enterFault(uint8_t cause, unsigned long now) {
  if (fault) return;
  fault = true;
  if (everReset && now - faultResetAt < REPEAT_WINDOW_MS) faultLocked = true;
  if (isGoto()) { clearDirs(); autoAx = -1; st = ST_IDLE; }
  savePending = false;
  joySuccess = false;
  Serial.print(F("FAULT cause="));
  switch (cause) {
    case C_RANGE:   Serial.println(F("RANGE"));     break;
    case C_DIR:     Serial.println(F("DIRECTION")); break;
    case C_RATE:    Serial.println(F("RATE"));      break;
    case C_STALL:   Serial.println(F("STALL"));     break;
    case C_TIMEOUT: Serial.println(F("TIMEOUT"));   break;
  }
  if (faultLocked) Serial.println(F("FAULT locked until reboot"));
}

void faultReset(unsigned long now) {
  fault = false;
  everReset = true;
  faultResetAt = now;
  dem = 0;
  joySuccess = false;
  Serial.println(F("FAULT cleared"));
}

// Выбор следующей оси автодвижения; autoAx = -1, если обе оси в допуске
void selectAxis(unsigned long now) {
  autoAx = -1;
  for (uint8_t ax = 0; ax < 2; ax++) {
    if (axDone[ax]) continue;
    int16_t d = (int16_t)tgt[ax] - cur[ax];
    if (abs(d) <= TOL) { axDone[ax] = true; continue; }
    autoAx = ax;
    int8_t sign = (ax == AX_V) ? SIGN_V : SIGN_H;
    autoFwd = ((d > 0) == (sign > 0));
    clearDirs();
    expDir[ax] = (d > 0) ? 1 : -1;
    beginAxis(ax);
    stallRefVal = cur[ax];
    stallT = axStart = now;
    return;
  }
}

void startAuto(State ns, uint8_t slot, unsigned long now) {
  flushSave();
  st = ns;
  autoSlot = (Slot)slot;
  for (uint8_t ax = 0; ax < 2; ax++) {
    tgt[ax] = mem[slot * 2 + ax];
    axDone[ax] = false;
  }
  autoAx = -1;
  clearDirs();
}

void goLast(unsigned long now) {
  if (isValid(S_LAST)) startAuto(ST_GOTO_LAST, S_LAST, now);
  else { clearDirs(); autoAx = -1; st = ST_IDLE; }
}

void applyJoyDirs() {
  for (uint8_t ax = 0; ax < 2; ax++) {
    int8_t want = (ax == joyAxis) ? joyExp : 0;
    if (expDir[ax] != want) {
      expDir[ax] = want;
      if (want) beginAxis(ax);
    }
  }
}

// Опрос потенциометров: диапазон, направление, скорость, штрафные очки, сброс аварии
void sampleSensors(unsigned long now) {
  if (now - lastSens < SENS_MS) return;
  lastSens = now;

  uint8_t v[2];
  bool ok = readPots(v);
  uint8_t why = ok ? 0 : C_RANGE;

  if (motorActive) {
    for (uint8_t ax = 0; ax < 2; ax++) {
      int16_t step = (int16_t)v[ax] - prevVal[ax];
      if (abs(step) > MAX_STEP && !why) why = C_RATE;
      if (expDir[ax] > 0) {
        if (v[ax] > ref[ax]) ref[ax] = v[ax];
        else if (ref[ax] - v[ax] > DEADBAND && !why) why = C_DIR;
      } else if (expDir[ax] < 0) {
        if (v[ax] < ref[ax]) ref[ax] = v[ax];
        else if (v[ax] - ref[ax] > DEADBAND && !why) why = C_DIR;
      }
    }
  }

  // ручное движение считается успешным, если показание реально сдвинулось в нужную сторону
  if (st == ST_JOY) {
    for (uint8_t ax = 0; ax < 2; ax++)
      if (expDir[ax] != 0 && abs((int16_t)ref[ax] - startVal[ax]) >= CHANGE_TH) joySuccess = true;
  }

  for (uint8_t ax = 0; ax < 2; ax++) { prevVal[ax] = v[ax]; cur[ax] = v[ax]; }
  rangeOk = ok;

  if (why) {
    lastWhy = why;
    if (dem < 250) dem += DEM_BAD;
    if (!fault && dem >= DEM_FAULT) enterFault(lastWhy, now);
  } else if (dem) {
    dem--;
  }

  if (dem == 0 && ok) {
    if (!goodOn) { goodOn = true; goodSince = now; }
  } else {
    goodOn = false;
    joySuccess = false;
  }

  if (fault && !faultLocked && joySuccess && goodOn && now - goodSince >= STABLE_MS)
    faultReset(now);
}

// ===================== джойстик =====================
const int P = A0;

const uint8_t N = 5;
const uint8_t PINS[N] = {A1, A5, A4, A2, A3};   // HL, VL, HR, VR, C
const uint8_t IDX_C = 4;
const uint8_t IDLE = 4;

// {при GND, при +5V}
const char*   TXT[IDX_C][2]  = {{"HL LEFT", "HL RIGHT"}, {"VL DOWN", "VL UP"},
                                {"HR LEFT", "HR RIGHT"}, {"VR DOWN", "VR UP"}};
const uint8_t MASK[IDX_C][2] = {{0x10, 0x10}, {0x10, 0x10},
                                {0x04, 0x08}, {0x02, 0x01}};

const uint8_t       STABLE_N      = 5;
const unsigned long SAMPLE_MS     = 2;
const unsigned long ERR_WINDOW_MS = 50;

Lvl raw[N], cand[N], accepted[N];
uint8_t candCount = 0;
bool haveAccepted = false;
unsigned long lastSample = 0;
bool pending = false;
unsigned long pendingSince = 0;

void scan() {
  bool hi[N], lo[N];
  digitalWrite(P, HIGH);
  delayMicroseconds(20);
  for (uint8_t i = 0; i < N; i++) hi[i] = digitalRead(PINS[i]);
  digitalWrite(P, LOW);
  delayMicroseconds(20);
  for (uint8_t i = 0; i < N; i++) lo[i] = digitalRead(PINS[i]);

  for (uint8_t i = 0; i < N; i++) {
    if (!hi[i] && !lo[i])    raw[i] = L_GND;
    else if (hi[i] && lo[i]) raw[i] = L_VCC;
    else                     raw[i] = L_MID;
  }
}

bool sameAs(const Lvl *a, const Lvl *b) {
  for (uint8_t i = 0; i < N; i++) if (a[i] != b[i]) return false;
  return true;
}

// Возвращает 0..3 (вход), IDLE или -1 (неконсистентно)
int8_t decode(const Lvl *s) {
  uint8_t active = 0, dir = 0;
  for (uint8_t i = 0; i < IDX_C; i++)
    if (s[i] != L_MID) { active++; dir = i; }

  if (active == 0) return (s[IDX_C] == L_MID) ? IDLE : -1;
  if (active == 1 && s[IDX_C] == ((s[dir] == L_GND) ? L_VCC : L_GND)) return dir;
  return -1;
}

void onAcceptedChange(unsigned long now) {
  int8_t dir = decode(accepted);
  if (dir < 0) {
    if (!pending) { pending = true; pendingSince = now; }
    return;
  }
  pending = false;
  joy = 0;
  joyAxis = -1;
  if (dir == IDLE) {
    Serial.println(F("IDLE"));
  } else {
    uint8_t side = (accepted[dir] == L_VCC);   // 1: RIGHT / UP
    joy = MASK[dir][side];
    if (dir == 0)      { joyAxis = AX_H; joyFwd = side; joyExp = side ? SIGN_H : -SIGN_H; }
    else if (dir == 1) { joyAxis = AX_V; joyFwd = side; joyExp = side ? SIGN_V : -SIGN_V; }
    Serial.println(TXT[dir][side]);
  }
}

// ===================== CAN =====================
bool canOk = false;
unsigned long lastCanTry = 0;

bool canInit() {
  if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) return false;
  CAN1.setMode(MCP_NORMAL);
  return true;
}

// Флаги принимаются после FLAG_STABLE_N одинаковых кадров подряд.
// Кадр с неизменившимся счётчиком считается залипшим и игнорируется.
void parseFrame(uint32_t id, uint8_t len, const uint8_t *d) {
  if (id != M_ID || len <= COUNTER_BYTE) return;
  uint8_t c = d[COUNTER_BYTE];
  if (haveCnt && c == lastCnt) return;
  haveCnt = true;
  lastCnt = c;
  lastRx_M = millis();
  haveRx = true;
  linkDown = false;

  uint16_t f = (d[0] | ((uint16_t)d[1] << 8)) & 0x01FF;
  if (candCnt > 0 && f == candF) {
    if (candCnt < 255) candCnt++;
  } else {
    candF = f;
    candCnt = 1;
  }
  if (candCnt >= FLAG_STABLE_N) {
    mLost = false;
    if (f != accF) {
      accF = f;
      applyFlags(accF);
    }
  }
}

void mcpModify(uint8_t addr, uint8_t mask, uint8_t data) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS1, LOW);
  SPI.transfer(MCP_BITMOD);
  SPI.transfer(addr);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(CS1, HIGH);
  SPI.endTransaction();
}

uint8_t txCnt = 0;

void sendFlags() {
  uint8_t buf[8] = {0};
  buf[0] = flags & 0xFF;
  buf[1] = (flags >> 8) & 0xFF;
  buf[COUNTER_BYTE] = txCnt++;          // счётчик кадра для R
  mcpModify(REG_CANCTRL, 0x10, 0x10);   // abort TX
  mcpModify(REG_CANCTRL, 0x10, 0x00);
  CAN1.sendMsgBuf(TX_ID, 0, 8, buf);
}

// ===================== машина состояний =====================
void runFsm(unsigned long now) {
  bool perm       = mR || mP;
  bool permLost   = prevPerm && !perm;
  bool rRise      = mR && !pR;
  bool rFall      = !mR && pR;
  bool cancelRise = mSETCANCEL && !pCancel;
  int8_t mSel     = (mM1 && !pM1) ? S_M1 : ((mM2 && !pM2) ? S_M2 : -1);
  bool joyAny     = (joy != 0);

  // сохранение M1/M2 по фронту
  if (mSAVE1 && !pSave1) saveCurrent(S_M1);
  if (mSAVE2 && !pSave2) saveCurrent(S_M2);

  // 1. джойстик имеет высший приоритет: прерывает автодвижение всегда,
  //    но двигает зеркало только при разрешении
  if (joyAny) {
    if (isGoto()) {
      clearDirs();
      autoAx = -1;
      st = perm ? ST_JOY : ST_IDLE;
      touchedLeft = false;
    } else if (st == ST_IDLE && perm) {
      st = ST_JOY;
      touchedLeft = false;
    }
  }

  // 2. ручное движение
  if (st == ST_JOY) {
    if (joyAny && perm) {
      if (joyAxis >= 0) { touchedLeft = true; savePending = false; }
      applyJoyDirs();
    } else {                                   // отпустили, ERR или потеря разрешения
      clearDirs();
      st = ST_IDLE;
      if (touchedLeft) {
        savePending = true;
        saveSel = mR ? S_MR : S_LAST;
        saveAt = now;
        if (!perm) flushSave();
      }
      touchedLeft = false;
    }
  }

  // 3. отложенная запись выполняется сразу при смене режима
  if (permLost || linkLostNow || rRise || rFall) flushSave();

  // 4. таймер записи Last/MR (только пока зеркало стоит)
  if (st == ST_IDLE && savePending && now - saveAt >= SAVE_DELAY_MS) flushSave();

  // 5. команды автодвижения (при аварии и при активном джойстике игнорируются)
  if (!fault && !joyAny) {
    switch (st) {
      case ST_IDLE:
        if (rRise) {
          if (isValid(S_MR)) startAuto(ST_GOTO_MR, S_MR, now);
        } else if (rFall && !linkLostNow) {
          if (isValid(S_LAST)) startAuto(ST_GOTO_LAST, S_LAST, now);
        } else if (mSel >= 0) {
          if (mR) copyToLast(mSel);
          else if (perm && isValid(mSel)) startAuto(ST_GOTO_M, mSel, now);
        }
        break;

      case ST_GOTO_MR:
        if (rFall) goLast(now);
        else if (mSel >= 0) copyToLast(mSel);
        break;

      case ST_GOTO_M:
        if (permLost || linkLostNow) goLast(now);
        else if (rRise) { if (isValid(S_MR)) startAuto(ST_GOTO_MR, S_MR, now); }
        else if (cancelRise) { clearDirs(); autoAx = -1; st = ST_IDLE; }
        else if (mSel >= 0 && mSel != autoSlot && isValid(mSel)) startAuto(ST_GOTO_M, mSel, now);
        break;

      case ST_GOTO_LAST:
        if (rRise) { if (isValid(S_MR)) startAuto(ST_GOTO_MR, S_MR, now); }
        else if (mSel >= 0 && !mR && perm && isValid(mSel)) startAuto(ST_GOTO_M, mSel, now);
        break;

      default: break;
    }
  }

  // 6. ведение автодвижения: смена оси, достижение цели, таймаут, застой
  if (isGoto()) {
    if (autoAx >= 0 && abs((int16_t)tgt[autoAx] - cur[autoAx]) <= TOL) {
      axDone[autoAx] = true;
      autoAx = -1;
      clearDirs();
    }
    if (autoAx < 0) selectAxis(now);

    if (autoAx < 0) {                          // обе оси в допуске
      clearDirs();
      st = ST_IDLE;
    } else if (now - axStart >= AUTO_TIMEOUT_MS) {
      enterFault(C_TIMEOUT, now);
    } else {
      if (abs((int16_t)cur[autoAx] - stallRefVal) > CHANGE_TH) {
        stallRefVal = cur[autoAx];
        stallT = now;
      } else if (now - stallT >= STALL_MS) {
        enterFault(C_STALL, now);
      }
    }
  }

  prevPerm = perm;
  pR = mR; pM1 = mM1; pM2 = mM2; pCancel = mSETCANCEL;
  pSave1 = mSAVE1; pSave2 = mSAVE2;
  linkLostNow = false;
}

// Единственная точка, где вызываются методы движения
void driveMotor() {
  bool perm = mR || mP;
  int8_t ax = -1;
  bool fwd = false;

  if (st == ST_JOY && perm && joyAxis >= 0) {
    ax = joyAxis;
    fwd = joyFwd;
  } else if (isGoto() && autoAx >= 0) {
    ax = autoAx;
    fwd = autoFwd;
  }

  motorActive = (ax >= 0);
  if (ax == AX_V)      MoveVertical(fwd);
  else if (ax == AX_H) MoveHorizontal(fwd);
  else                 StopMotor();
}

// ===================== setup / loop =====================
void setup() {
  PORTD &= 0x0F;           // IN1..IN4 = 0, затем делаем выходами
  DDRD  |= 0xF0;

  Serial.begin(SERIAL_BAUD);
  pinMode(P, OUTPUT);
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT);
  pinMode(INT1, INPUT_PULLUP);

  loadMemory();
  rangeOk = readPots(cur);
  prevVal[0] = cur[0]; prevVal[1] = cur[1];

  canOk = canInit();
  if (!canOk) Serial.println(F("CAN1 init fail, retrying"));
  lastCanTry = lastRx_M = lastTx = millis();
}

void loop() {
  unsigned long now = millis();

  // повторная инициализация MCP2515
  if (!canOk && now - lastCanTry >= CAN_RETRY_MS) {
    lastCanTry = now;
    canOk = canInit();
    if (canOk) Serial.println(F("CAN1 ok"));
  }

  // потеря M_ID (только если хоть один пакет уже был)
  if (haveRx && !linkDown && now - lastRx_M > M_TIMEOUT_MS) {
    linkDown = true;
    linkLostNow = true;
    mLost = true;
    resetPermissions();
  }

  // джойстик неконсистентен дольше окна - ошибка, как отпускание
  if (pending && now - pendingSince >= ERR_WINDOW_MS) {
    pending = false;
    joy = 0;
    joyAxis = -1;
    Serial.println(F("ERR"));
  }

  if (now - lastSample > SAMPLE_MS) {
    lastSample = now;
    scan();
    if (candCount > 0 && sameAs(raw, cand)) {
      if (candCount < 255) candCount++;
    } else {
      memcpy(cand, raw, sizeof(cand));
      candCount = 1;
    }
    if (candCount >= STABLE_N && (!haveAccepted || !sameAs(cand, accepted))) {
      memcpy(accepted, cand, sizeof(accepted));
      haveAccepted = true;
      onAcceptedChange(now);
    }
  }

  if (canOk) {
    uint8_t frames = 0;
    while (frames < MAX_FRAMES_LOOP && digitalRead(INT1) == LOW) {
      frames++;
      unsigned long id;
      byte len;
      byte d[8];
      if (CAN1.readMsgBuf(&id, &len, d) != CAN_OK) break;
      if (id & 0xC0000000UL) continue;
      if (len > 8) len = 8;
      parseFrame(id, len, d);
    }
  }

  sampleSensors(now);
  runFsm(now);
  driveMotor();

  // биты 0..3 - правое зеркало (джойстик, только при mR или mP),
  // бит 4 - мотор левого зеркала включён, бит 5 - авария потенциометров или недостоверные данные M
  flags = ((mR || mP) ? (joy & 0x0F) : 0)
        | (motorActive ? 0x10 : 0)
        | ((fault || mLost) ? 0x20 : 0);

  if (canOk && now - lastTx >= TX_PERIOD_MS) {
    lastTx += TX_PERIOD_MS;
    sendFlags();
  }
}
