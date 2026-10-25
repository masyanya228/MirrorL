#include <mcp_can.h>
#include <SPI.h>

#define CS1          10
#define INT1         2
#define SERIAL_BAUD  500000
#define M_ID         0x10UL
#define R_ID        0x40UL
#define TX_ID        0x20UL
#define TX_PERIOD_MS 40
#define M_TIMEOUT_MS 500
#define MCP_BITMOD   0x05
#define REG_CANCTRL  0x0F
#define REG_RXB0CTRL  0x60

MCP_CAN CAN1(CS1);

// флаги из M_ID
bool mSET, mSAVE1, mSAVE2, mM1, mM2, mSETCANCEL, mR, mP = true, mDN;
unsigned long lastRx_M = 0;

// положение джойстика: 0 VR UP, 1 VR DOWN, 2 HR LEFT, 3 HR RIGHT, 4 левое зеркало
uint8_t joy = 0;
// флаги на отправку (joy с учётом разрешения)
uint16_t flags = 0;
unsigned long lastTx = 0;

const int P = A0;
enum Lvl : uint8_t { L_GND, L_VCC, L_MID };

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
  if (dir == IDLE) {
    Serial.println(F("IDLE"));
  } else {
    uint8_t side = (accepted[dir] == L_VCC);
    joy = MASK[dir][side];
    Serial.println(TXT[dir][side]);
  }
}

void parseFrame(uint32_t id, uint8_t len, const uint8_t *d) {
  if (id != M_ID || len < 2) return;
  uint16_t f = d[0] | ((uint16_t)d[1] << 8);
  mSET       = f & (1 << 0);
  mSAVE1     = f & (1 << 1);
  mSAVE2     = f & (1 << 2);
  mM1        = f & (1 << 3);
  mM2        = f & (1 << 4);
  mSETCANCEL = f & (1 << 5);
  mR         = f & (1 << 6);
  mP         = f & (1 << 7);
  mDN        = f & (1 << 8);
  lastRx_M = millis();
}

void resetPermissions() {
  mSET = mSAVE1 = mSAVE2 = mM1 = mM2 = mSETCANCEL = mR = mDN = false;
  mP = true;
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

void sendFlags() {
  uint8_t buf[8] = {0};
  buf[0] = flags & 0xFF;
  buf[1] = (flags >> 8) & 0xFF;
  mcpModify(REG_CANCTRL, 0x10, 0x10);   // abort TX
  mcpModify(REG_CANCTRL, 0x10, 0x00);
  CAN1.sendMsgBuf(TX_ID, 0, 8, buf);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(P, OUTPUT);
  for (uint8_t i = 0; i < N; i++) pinMode(PINS[i], INPUT);
  pinMode(INT1, INPUT_PULLUP);

  if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("CAN1 init fail"));
    while (1) {}
  }
  mcpModify(REG_RXB0CTRL, 0x04, 0x04);
  CAN1.setMode(MCP_NORMAL);
  lastRx_M = lastTx = millis();
}

void loop() {
  unsigned long now = millis();

  // потеря M_ID: сбросить разрешения, mP остаётся включённым
  if (now - lastRx_M > M_TIMEOUT_MS) resetPermissions();

  // состояние неконсистентно дольше окна — ошибка
  if (pending && now - pendingSince >= ERR_WINDOW_MS) {
    pending = false;
    joy = 0;
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

  while (digitalRead(INT1) == LOW) {
    unsigned long id;
    byte len;
    byte d[8];
    if (CAN1.readMsgBuf(&id, &len, d) != CAN_OK) break;
    if (id & 0xC0000000UL) continue;
    if (len > 8) len = 8;
    parseFrame(id, len, d);
  }

  // джойстик действует только при mR или mP
  flags = (mR || mP) ? joy : 0;

  if (now - lastTx >= TX_PERIOD_MS) {
    lastTx += TX_PERIOD_MS;
    sendFlags();
  }
}