#include <mcp_can.h>
#include <SPI.h>

//todo - MCP2
#define CS1          10
#define INT1         2
#define SERIAL_BAUD  500000
#define M_ID        0x10UL
#define R_ID        0x40UL
#define TX_ID        0x20UL
#define TX_PERIOD_MS 40
#define MCP_BITMOD    0x05
#define REG_CANCTRL   0x0F
#define REG_RXB0CTRL  0x60

MCP_CAN CAN1(CS1);
uint16_t R_flags = 0;
uint16_t M_flags = 0;
unsigned long lastRx_M = 0;
uint16_t flags = 0;
unsigned long lastTx = 0;

const int P = A0;

enum Lvl : uint8_t { L_GND, L_VCC, L_MID };
const char* LVL_NAME[] = {"GND", "+5V", "MID"};

const uint8_t N = 5;
const char*   NAMES[N] = {"HL", "VL", "HR", "VR", "C"};
const uint8_t PINS[N]  = {A1,   A5,   A4,   A2,   A3};
const uint8_t IDX_C = 4;        // индекс контрольного пина

// Названия направления для каждого входа: {при GND, при +5V}
const char* DIR_NAME[IDX_C][2] = {
  {"LEFT", "RIGHT"},   // HL
  {"DOWN", "UP"},      // VL
  {"LEFT", "RIGHT"},   // HR
  {"DOWN", "UP"}       // VR
};

const uint8_t       STABLE_N      = 5;    // одинаковых опросов подряд для антидребезга
const unsigned long SAMPLE_MS     = 2;    // период опроса
const unsigned long ERR_WINDOW_MS = 50;   // сколько ждать, пока C догонит направление

const int8_t CODE_NONE = -1;
const int8_t CODE_IDLE = 0;
const int8_t CODE_ERR  = 100;

Lvl raw[N], cand[N], accepted[N];
uint8_t candCount = 0;
bool haveAccepted = false;
unsigned long lastSample = 0;

bool pending = false;            // ждём, не станет ли состояние корректным
unsigned long pendingSince = 0;
bool errReported = false;        // ERR уже выведен, дальнейшие смены пишем сразу
int8_t lastLogged = CODE_NONE;   // что было выведено последним

void scan() {
  bool hi[N], lo[N];

  digitalWrite(P, HIGH);
  delayMicroseconds(20);
  for (uint8_t i = 0; i < N; i++) hi[i] = digitalRead(PINS[i]);

  digitalWrite(P, LOW);
  delayMicroseconds(20);
  for (uint8_t i = 0; i < N; i++) lo[i] = digitalRead(PINS[i]);

  for (uint8_t i = 0; i < N; i++) {
    if (!hi[i] && !lo[i])     raw[i] = L_GND;
    else if (hi[i] && lo[i])  raw[i] = L_VCC;
    else                      raw[i] = L_MID;
  }
}

bool sameAs(const Lvl *a, const Lvl *b) {
  for (uint8_t i = 0; i < N; i++) if (a[i] != b[i]) return false;
  return true;
}

// Возвращает код корректного состояния или CODE_NONE, если состояние неконсистентно
int8_t validCode(const Lvl *s) {
  uint8_t active = 0, dir = 0;
  for (uint8_t i = 0; i < IDX_C; i++) {
    if (s[i] != L_MID) { active++; dir = i; }
  }
  Lvl c = s[IDX_C];

  if (active == 0) return (c == L_MID) ? CODE_IDLE : CODE_NONE;
  if (active == 1) {
    Lvl expected = (s[dir] == L_GND) ? L_VCC : L_GND;
    if (c == expected) return 1 + dir * 2 + (s[dir] == L_VCC ? 1 : 0);
  }
  return CODE_NONE;
}

void printValid(const Lvl *s) {
  uint8_t dir = 0;
  bool found = false;
  for (uint8_t i = 0; i < IDX_C; i++) {
    if (s[i] != L_MID) { dir = i; found = true; }
  }
  if (!found) { Serial.println(F("IDLE")); return; }
  Serial.print(NAMES[dir]); Serial.print(F(": "));
  Serial.println(DIR_NAME[dir][s[dir] == L_VCC ? 1 : 0]);
}

void printErr(const Lvl *s) {
  uint8_t active = 0, dir = 0;
  for (uint8_t i = 0; i < IDX_C; i++) {
    if (s[i] != L_MID) { active++; dir = i; }
  }
  Lvl c = s[IDX_C];

  if (active == 0) {
    Serial.print(F("ERR: направление не выбрано, но C="));
    Serial.println(LVL_NAME[c]);
  } else if (active == 1) {
    Lvl expected = (s[dir] == L_GND) ? L_VCC : L_GND;
    Serial.print(F("ERR: ")); Serial.print(NAMES[dir]);
    Serial.print(F("=")); Serial.print(LVL_NAME[s[dir]]);
    Serial.print(F(", C=")); Serial.print(LVL_NAME[c]);
    Serial.print(F(" (ожидалось C=")); Serial.print(LVL_NAME[expected]);
    Serial.println(F(")"));
  } else {
    Serial.print(F("ERR: активно несколько:"));
    for (uint8_t i = 0; i < IDX_C; i++) {
      if (s[i] != L_MID) {
        Serial.print(' '); Serial.print(NAMES[i]);
        Serial.print('='); Serial.print(LVL_NAME[s[i]]);
      }
    }
    Serial.print(F("  C=")); Serial.println(LVL_NAME[c]);
  }
}

// Вызывается, когда устойчивое состояние изменилось
void onAcceptedChange(unsigned long now) {
  int8_t code = validCode(accepted);

  if (code != CODE_NONE) {
    // Корректное состояние: выводим сразу, ожидание отменяем
    pending = false;
    errReported = false;
    if (code != lastLogged) {
      printValid(accepted);
      lastLogged = code;
    }
  } else {
    if (errReported) {
      // Уже в режиме ошибки: новое неконсистентное состояние пишем сразу
      printErr(accepted);
    } else if (!pending) {
      // Первое неконсистентное состояние: запускаем окно ожидания
      pending = true;
      pendingSince = now;
    }
    // если pending уже идёт, таймер не перезапускаем
  }
}

void parseFrame(uint32_t id, uint8_t len, const uint8_t *d) {
  if(id==0x10)
  {
    //m flags
  }
  else if(id==0x40)
  {
    //r flags
  }
  for (uint8_t i = 0; i < F_COUNT; i++) {
    if (SIGS[i].id != id || SIGS[i].byteIdx >= len) continue;
    if ((d[SIGS[i].byteIdx] >> SIGS[i].bit) & 1) flags |= ((uint16_t)1 << i);
    else flags &= ~((uint16_t)1 << i);
  }

  if (id == ID_A && len == 8) {
    //Serial.println("KPP");
    uint8_t prnd = (d[7] >> 2) & 0x0F;
    const uint16_t prndMask = ((uint16_t)1 << F_P) | ((uint16_t)1 << F_R) | ((uint16_t)1 << F_DN);
    flags &= ~prndMask;
    switch (prnd) {
      case 0:
      case 15:
        flags |= ((uint16_t)1 << F_P);
        break;
      case 7:
        flags |= ((uint16_t)1 << F_R);
        break;
      case 5:
      case 12:
      case 8:
      case 6:
        flags |= ((uint16_t)1 << F_DN);
        break;
      default:
        break;
    }
  }
}

void mcpModify(uint8_t cs, uint8_t addr, uint8_t mask, uint8_t data) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  SPI.transfer(MCP_BITMOD);
  SPI.transfer(addr);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void abortTx() {
  mcpModify(CS1, REG_CANCTRL, 0x10, 0x10);
  mcpModify(CS1, REG_CANCTRL, 0x10, 0x00);
}

void sendFlags() {
  uint8_t buf[2];
  buf[0] = flags & 0xFF;
  buf[1] = (flags >> 8) & 0x01;
  abortTx();
  CAN1.sendMsgBuf(TX_ID, 0, 2, buf);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(P, OUTPUT);
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT);

  pinMode(INT1, INPUT_PULLUP);
  if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("CAN1 init fail"));
    while (1) { ; }
  }
  CAN1.setMode(MCP_NORMAL);

  lastTx = millis();
}

void loop() {
  unsigned long now = millis();

  // Истекло окно ожидания, а состояние всё ещё неконсистентно
  if (pending && (now - pendingSince >= ERR_WINDOW_MS)) {
    pending = false;
    errReported = true;
    printErr(accepted);
    lastLogged = CODE_ERR;
  }

  if (now - lastSample > SAMPLE_MS)
  {
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

  while (digitalRead(INT1) == LOW) {
    unsigned long id;
    byte len;
    byte d[8];
    if (CAN1.readMsgBuf(&id, &len, d) != CAN_OK) break;
    if (id & 0xC0000000UL) continue;
    if (len > 8) len = 8;
    parseFrame(id, len, d);
  }

  unsigned long now = millis();
  if (now - lastTx >= TX_PERIOD_MS) {
    lastTx += TX_PERIOD_MS;
    sendFlags();
  }
}