/*
 * SmartSprout — автополив растения, мини-проект ВРИИ
 *
 * Sense : ёмкостный датчик влажности почвы v1.2 (аналоговый сигнал на GPIO34)
 * Think : скользящее среднее (окно 10 отсчётов), пересчёт в проценты влажности,
 *         пороги с гистерезисом и конечный автомат из 3 состояний:
 *         СУХО, ПОЛИВ, ВЛАЖНО
 * Act   : реле включает мини-помпу 5V, светодиоды показывают состояние:
 *         красный — сухо, синий — идёт полив, зелёный — норма
 *
 * Плата: ESP32 Dev Board (30 pin). Внешние библиотеки не нужны.
 */

// ---------- Пины ----------
const int PIN_SOIL  = 34;   // AOUT датчика (ADC1, только вход)
const int PIN_RELAY = 26;   // IN модуля реле
const int PIN_RED   = 25;   // красный светодиод — сухо
const int PIN_GREEN = 27;   // зелёный светодиод — норма
const int PIN_BLUE  = 14;   // синий светодиод — идёт полив

// Большинство 1-канальных модулей реле включаются низким уровнем (LOW).
// Если реле щёлкает наоборот, поменяй на false.
const bool RELAY_ACTIVE_LOW = true;

// ---------- Калибровка датчика ----------
// Значения АЦП (0..4095): датчик на воздухе и датчик в стакане воды.
// Ёмкостный датчик в сухой среде даёт БОЛЬШЕ, во влажной — МЕНЬШЕ.
const int RAW_DRY = 2800;   // на воздухе
const int RAW_WET = 1200;   // в воде

// ---------- Обработка ----------
const int WIN = 10;                       // окно скользящего среднего
const unsigned long SAMPLE_MS = 200;      // период опроса: 10 * 200 мс = 2 с

// Пороги с гистерезисом (в процентах влажности):
const int DRY_BELOW = 30;   // ниже 30 % — почва сухая
const int WET_ABOVE = 60;   // выше 60 % — почва достаточно влажная
                            // между 30 и 60 % состояние не меняется

// ---------- Защита помпы ----------
const unsigned long T_CONFIRM_DRY = 3000;   // почва должна быть сухой 3 с подряд, прежде чем поливать
const unsigned long T_PUMP_MAX    = 15000;  // помпа работает не дольше 15 с за один полив
const unsigned long T_RETRY       = 30000;  // если за 15 с не намокло — ждём 30 с, вода впитывается

// ---------- Конечный автомат ----------
enum State { SUKHO, POLIV, VLAZHNO };
const char* STATE_NAMES[] = { "СУХО", "ПОЛИВ", "ВЛАЖНО" };

State state = VLAZHNO;
unsigned long stateSince = 0;   // когда вошли в текущее состояние

// ---------- Скользящее среднее ----------
int buf[WIN];
int bufIdx = 0;
long bufSum = 0;

int movingAverage(int x) {
  bufSum -= buf[bufIdx];
  buf[bufIdx] = x;
  bufSum += x;
  bufIdx = (bufIdx + 1) % WIN;
  return bufSum / WIN;
}

int rawToPercent(int raw) {
  int p = map(raw, RAW_DRY, RAW_WET, 0, 100);
  return constrain(p, 0, 100);
}

void setPump(bool on) {
  digitalWrite(PIN_RELAY, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setLeds(bool red, bool green, bool blue) {
  digitalWrite(PIN_RED, red);
  digitalWrite(PIN_GREEN, green);
  digitalWrite(PIN_BLUE, blue);
}

void enterState(State s) {
  state = s;
  stateSince = millis();
  switch (s) {
    case SUKHO:   setPump(false); setLeds(true,  false, false); break;
    case POLIV:   setPump(true);  setLeds(false, false, true);  break;
    case VLAZHNO: setPump(false); setLeds(false, true,  false); break;
  }
  Serial.print("# -> ");
  Serial.println(STATE_NAMES[s]);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RELAY, OUTPUT);
  setPump(false);                       // сначала выключаем помпу, потом всё остальное
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOIL, ADC_11db);   // диапазон до ~3.3 В

  // Заполняем буфер первым замером, чтобы среднее сразу было правильным
  int first = analogRead(PIN_SOIL);
  for (int i = 0; i < WIN; i++) buf[i] = first;
  bufSum = (long)first * WIN;

  enterState(rawToPercent(first) < DRY_BELOW ? SUKHO : VLAZHNO);
}

void loop() {
  static unsigned long lastSample = 0;
  static unsigned long dryStart = 0;      // с какого момента почва сухая (в СУХО)
  static unsigned long lastPumpEnd = 0;   // когда последний раз выключили помпу
  static bool pumpedBefore = false;       // true, если прошлый полив не довёл почву до нормы

  if (millis() - lastSample < SAMPLE_MS) return;
  lastSample = millis();

  int raw = analogRead(PIN_SOIL);
  int avg = movingAverage(raw);
  int moist = rawToPercent(avg);
  unsigned long inState = millis() - stateSince;

  switch (state) {
    case VLAZHNO:
      if (moist < DRY_BELOW) enterState(SUKHO);
      break;

    case SUKHO:
      if (moist > WET_ABOVE) {            // полили вручную или датчик переставили
        enterState(VLAZHNO);
        break;
      }
      if (moist >= DRY_BELOW) {           // кратковременный скачок — сбрасываем подтверждение
        dryStart = 0;
        break;
      }
      if (dryStart == 0) dryStart = millis();
      // Поливаем, если сухость подтверждена и прошла пауза после прошлого полива
      if (millis() - dryStart >= T_CONFIRM_DRY &&
          (!pumpedBefore || millis() - lastPumpEnd >= T_RETRY)) {
        dryStart = 0;
        enterState(POLIV);
      }
      break;

    case POLIV:
      if (moist > WET_ABOVE) {
        lastPumpEnd = millis();
        pumpedBefore = false;             // полив удался, паузу ждать не нужно
        enterState(VLAZHNO);
      } else if (inState >= T_PUMP_MAX) {
        lastPumpEnd = millis();
        pumpedBefore = true;              // не намокло за 15 с — ждём T_RETRY, вода впитывается
        enterState(SUKHO);
      }
      break;
  }

  // Вывод для Serial Plotter: сырое значение (в %), сглаженное и пороги
  Serial.print("raw:");    Serial.print(rawToPercent(raw));
  Serial.print(" avg:");   Serial.print(moist);
  Serial.print(" dry:");   Serial.print(DRY_BELOW);
  Serial.print(" wet:");   Serial.print(WET_ABOVE);
  Serial.print(" pump:");  Serial.println(state == POLIV ? 100 : 0);
}
