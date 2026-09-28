#include <Arduino.h>
#include <RP2040_PWM.h>

// =====================================================================
//  Настройки (все числа, которые стоит подбирать, собраны здесь)
// =====================================================================

// ---- Моторы ----
#define N_MOTORS 10
const uint8_t motor_pins[N_MOTORS] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
const float PWM_FREQ = 20000.0f;          // 20 кГц — не слышно писка

// ---- Входное значение (условные единицы, у.е.) ----
const float VALUE_OFF = 30.0f;            // ниже — все моторы выключены
const float VALUE_MAX = 1000.0f;

// ---- Сглаживание ----
#define HISTORY_LEN   5                   // скользящее среднее для скорости
#define POSITION_LEN  3                   // среднее для позиции (центра)

// ---- Ширина колокола в зависимости от скорости ----
const float SPEED_SLOW      = 40.0f;      // у.е./с и медленнее -> узкий колокол
const float SPEED_FAST      = 200.0f;     // у.е./с и быстрее   -> широкий колокол
const float HALF_WIDTH_SLOW = 20.0f;      // захват в каждую сторону, у.е.
const float HALF_WIDTH_FAST = 100.0f;

// ---- Сила: растёт с позицией, от DUTY_MIN при 30 у.е. до DUTY_MAX при 1000 ----
const float DUTY_MIN      = 20.0f;        // %, ниже вибромотор не раскручивается
const float WEIGHT_CUTOFF = 0.135f;       // e^-2 — край захвата: дальше мотор выключен
const float DUTY_MAX      = 100.0f;       // %

// ---- Прочее ----
#define TIMEOUT_MS      2000              // нет данных столько времени -> всё выключить
#define DEBUG_PRINT_MS       1000         // как часто печатать состояние в монитор
#define DEBUG_PRINT_FAST_MS  250          // то же во время demo/test
#define PLOT_MS              40           // машинный вывод для визуализатора (25 раз/с)

// =====================================================================

RP2040_PWM* pwmInstance[N_MOTORS];
float motor_duty[N_MOTORS];

// ---- Кадр от STM32 по UART (GP0 = TX, GP1 = RX) ----
#pragma pack(push, 1)
typedef struct {
    uint8_t  header;  // 0xAA
    uint16_t value;   // 0..1000 у.е.
    uint8_t  crc;     // XOR байтов 1..2
} Frame_t;
#pragma pack(pop)

#define FRAME_SIZE sizeof(Frame_t) // 4 байта

uint8_t rx_buf[FRAME_SIZE];
uint8_t rx_idx = 0;

static uint8_t SimpleCRC8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

// ---- История значений (кольцевой буфер) ----
float history[HISTORY_LEN];
uint8_t hist_head = 0;   // куда писать следующее значение
uint8_t hist_count = 0;  // сколько значений уже накоплено

float prev_ma = 0;
uint32_t prev_ma_time = 0;
float speed = 0;         // у.е./с

uint32_t last_sample_time = 0;
bool motors_active = false;

// Для печати в монитор
float dbg_value = 0, dbg_position = 0, dbg_half_width = 0, dbg_amplitude = 0;

void pushValue(float v)
{
    history[hist_head] = v;
    hist_head = (hist_head + 1) % HISTORY_LEN;
    if (hist_count < HISTORY_LEN) hist_count++;
}

// Среднее последних n значений (или сколько есть)
float averageLast(uint8_t n)
{
    if (n > hist_count) n = hist_count;
    float sum = 0;
    for (uint8_t k = 0; k < n; k++) {
        sum += history[(hist_head + HISTORY_LEN - 1 - k) % HISTORY_LEN];
    }
    return sum / n;
}

// Зона мотора i: мотор 0 — 30..100, мотор 1 — 100..200, ..., мотор 9 — 900..1000
float zoneStart(int i) { return (i == 0) ? VALUE_OFF : i * 100.0f; }
float zoneEnd(int i)   { return (i + 1) * 100.0f; }

// Расстояние от позиции до зоны мотора (0, если позиция внутри зоны)
float distanceToZone(float position, int i)
{
    if (position < zoneStart(i)) return zoneStart(i) - position;
    if (position > zoneEnd(i))   return position - zoneEnd(i);
    return 0.0f;
}

void setMotor(int i, float duty)
{
    motor_duty[i] = duty;
    pwmInstance[i]->setPWM(motor_pins[i], PWM_FREQ, duty);
}

void allMotorsOff()
{
    for (int i = 0; i < N_MOTORS; i++) setMotor(i, 0);
    motors_active = false;
}

// Главная логика: пришло новое значение
void processSample(float value)
{
    uint32_t now = millis();
    value = constrain(value, 0.0f, VALUE_MAX);
    last_sample_time = now;

    pushValue(value);
    float position = averageLast(POSITION_LEN);
    float ma = averageLast(HISTORY_LEN);

    // Скорость = производная скользящего среднего
    if (hist_count == 1) {
        speed = 0;
        prev_ma = ma;
        prev_ma_time = now;
    } else if (now != prev_ma_time) {
        speed = fabsf(ma - prev_ma) * 1000.0f / (now - prev_ma_time);
        prev_ma = ma;
        prev_ma_time = now;
    }

    // Ширина колокола: линейно между медленной и быстрой скоростью
    float t = constrain((speed - SPEED_SLOW) / (SPEED_FAST - SPEED_SLOW), 0.0f, 1.0f);
    float half_width = HALF_WIDTH_SLOW + t * (HALF_WIDTH_FAST - HALF_WIDTH_SLOW);
    float sigma = half_width / 2.0f;   // на краю захвата колокол = e^-2 ≈ 14% силы

    // Сила: 30 у.е. -> DUTY_MIN (20%), 1000 у.е. -> 100%, между ними линейно
    float amplitude = 0;
    if (position >= VALUE_OFF) {
        amplitude = DUTY_MIN + (position - VALUE_OFF) * (DUTY_MAX - DUTY_MIN) / (VALUE_MAX - VALUE_OFF);
    }

    for (int i = 0; i < N_MOTORS; i++) {
        float d = distanceToZone(position, i);
        float weight = expf(-(d * d) / (2.0f * sigma * sigma));
        float duty = weight * amplitude;
        if (amplitude <= 0 || weight < WEIGHT_CUTOFF) {
            duty = 0;                       // за краем захвата — выключен
        } else if (duty < DUTY_MIN) {
            duty = DUTY_MIN;                // внутри захвата — не слабее порога запуска
        }
        setMotor(i, duty);
    }
    motors_active = true;

    dbg_value = value;
    dbg_position = position;
    dbg_half_width = half_width;
    dbg_amplitude = amplitude;
}

// ---- Приём кадров от STM32 ----
void readUartFrames()
{
    while (Serial1.available() > 0) {
        uint8_t b = Serial1.read();

        if (rx_idx == 0) {
            if (b == 0xAA) rx_buf[rx_idx++] = b; // ищем заголовок
        } else {
            rx_buf[rx_idx++] = b;
            if (rx_idx == FRAME_SIZE) {
                Frame_t *frame = (Frame_t*)rx_buf;
                uint8_t crc = SimpleCRC8(rx_buf + 1, FRAME_SIZE - 2);
                if (crc == frame->crc) {
                    processSample(frame->value);
                }
                rx_idx = 0;
            }
        }
    }
}

// ---- Команды с компьютера (Serial Monitor) для проверки ----
//   "450"       — значение 450
//   "demo 100"  — ездить по ряду туда-обратно со скоростью 100 у.е./с
//   "fulltest"  — полный тест всех режимов (см. test_steps ниже), можно просто "test"
//   "go 700 150" — свой тест: плавно к 700 со скоростью 150 у.е./с,
//                 стоим 5 с, затем мгновенно в 0
//   "m 9 100"   — включить один мотор (9) на 100% напрямую, до "stop"
//   "stop"      — остановить демо/тест
//   "log"       — включить/выключить печать состояния
//   "plot 1" / "plot 0" — машинный вывод для python/visualizer.py
bool log_on = true;
bool plot_on = false;
bool demo_on = false;
float demo_speed = 100;
int demo_dir = 1;

float gen_value = 0;          // значение, которое генерируют демо и тест
uint32_t gen_last_ms = 0;

// ---- Синтетический тест ----
//   MOVE   — ехать к a, скорость плавно меняется от b до c (у.е./с)
//   HOLD   — стоять на месте ms миллисекунд
//   WOBBLE — качаться вокруг a с размахом ±b и частотой c (Гц) ms миллисекунд
enum StepType { MOVE, HOLD, WOBBLE };

struct TestStep {
    StepType type;
    float a, b, c;
    uint32_t ms;
    const char *name;
};

const TestStep test_steps[] = {
    {MOVE,   1000,   35,   35,     0, "Медленно 0 -> 1000: колокол узкий, сила растёт"},
    {HOLD,      0,    0,    0,  2000, "Стоим на 1000: максимальная сила"},
    {MOVE,      0,  250,  250,     0, "Быстро 1000 -> 0: колокол широкий"},
    {HOLD,      0,    0,    0,  2000, "Стоим на 0: ниже 30 всё выключено"},
    {MOVE,    500,  120,  120,     0, "Средняя скорость 0 -> 500: средняя ширина"},
    {HOLD,      0,    0,    0,  3000, "Остановка на 500: колокол сужается"},
    {MOVE,   1000,   20,  350,     0, "Разгон 500 -> 1000 (20 -> 350): колокол расширяется"},
    {MOVE,     50,  350,   20,     0, "Торможение 1000 -> 50 (350 -> 20): колокол сужается"},
    {HOLD,      0,    0,    0,  2000, "Стоим на 50: мотор 1, слабо"},
    {MOVE,    250,  100,  100,     0, "Переезд к 250"},
    {WOBBLE,  250,   60, 0.25f, 8000, "Медленное покачивание у 250 (+-60): ширина слегка дышит"},
    {MOVE,    750,  150,  150,     0, "Переезд к 750"},
    {WOBBLE,  750,  150, 0.5f,  8000, "Быстрое покачивание у 750 (+-150): ширина пульсирует"},
    {MOVE,    850, 2000, 2000,     0, "Скачок на 850"},
    {HOLD,      0,    0,    0,  1500, "Стоим на 850"},
    {MOVE,    150, 2000, 2000,     0, "Скачок на 150"},
    {HOLD,      0,    0,    0,  1500, "Стоим на 150"},
    {MOVE,      0,  200,  200,     0, "Уход в 0: всё выключается"},
    {HOLD,      0,    0,    0,  1000, "Конец"},
};
const int TEST_STEPS_COUNT = sizeof(test_steps) / sizeof(test_steps[0]);

// ---- Свой тест: к цели с заданной скоростью, 5 с стоим, мгновенно в 0 ----
const float CUSTOM_SPEED_MIN = 5.0f;
const float CUSTOM_SPEED_MAX = 800.0f;    // не быстрее 4 моторов за 0.5 с
const uint32_t CUSTOM_HOLD_MS = 5000;

char custom_name_move[64];
char custom_name_hold[64];
TestStep custom_steps[] = {
    {MOVE,   0,   0,   0,              0, custom_name_move},
    {HOLD,   0,   0,   0, CUSTOM_HOLD_MS, custom_name_hold},
    {MOVE,   0, 1e6, 1e6,              0, "Мгновенный сброс в 0"},
    {HOLD,   0,   0,   0,           1000, "Конец"},
};

bool test_on = false;
const TestStep *cur_steps = test_steps;
int cur_steps_count = TEST_STEPS_COUNT;
const char *cur_title = "Полный тест";
int test_step = 0;
float test_from = 0;
uint32_t test_step_start = 0;

void startTestStep(int i)
{
    test_step = i;
    test_from = gen_value;
    test_step_start = millis();
    Serial.print("\n[");
    Serial.print(cur_title);
    Serial.print(" ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(cur_steps_count);
    Serial.print("] ");
    Serial.println(cur_steps[i].name);
}

void startTest(const TestStep *steps, int count, const char *title)
{
    cur_steps = steps;
    cur_steps_count = count;
    cur_title = title;
    demo_on = false;
    test_on = true;
    gen_value = 0;
    gen_last_ms = millis();
    startTestStep(0);
}

char line[32];
uint8_t line_len = 0;

void parseLine(char *s)
{
    if (strncmp(s, "demo", 4) == 0) {
        float v = strtof(s + 4, nullptr);
        if (v > 0) demo_speed = v;
        demo_on = true;
        test_on = false;
        gen_value = 0;
        demo_dir = 1;
        gen_last_ms = millis();
        Serial.print("Demo, speed = ");
        Serial.println(demo_speed);
        return;
    }
    if (strncmp(s, "fulltest", 8) == 0 || strncmp(s, "test", 4) == 0) {
        startTest(test_steps, TEST_STEPS_COUNT, "Полный тест");
        return;
    }
    if (strncmp(s, "go", 2) == 0) {
        char *end;
        float target = strtof(s + 2, &end);
        if (end == s + 2) {
            Serial.println("Формат: go <цель 0..1000> <скорость у.е./с>");
            return;
        }
        float v = strtof(end, nullptr);
        if (v <= 0) v = 100;
        target = constrain(target, 0.0f, VALUE_MAX);
        v = constrain(v, CUSTOM_SPEED_MIN, CUSTOM_SPEED_MAX);

        snprintf(custom_name_move, sizeof(custom_name_move),
                 "Плавно 0 -> %d, %d у.е./с", (int)target, (int)v);
        snprintf(custom_name_hold, sizeof(custom_name_hold),
                 "Стоим на %d, %d с", (int)target, (int)(CUSTOM_HOLD_MS / 1000));
        custom_steps[0].a = target;
        custom_steps[0].b = v;
        custom_steps[0].c = v;
        startTest(custom_steps, sizeof(custom_steps) / sizeof(custom_steps[0]), "Свой тест");
        return;
    }
    if (s[0] == 'm' && s[1] == ' ') {       // "m 9 100" — мотор 9 на 100% до "stop"
        char *end;
        long n = strtol(s + 1, &end, 10);
        float d = strtof(end, nullptr);
        if (n < 1 || n > N_MOTORS) {
            Serial.println("Формат: m <номер 1..10> <сила 0..100>");
            return;
        }
        demo_on = false;
        test_on = false;
        allMotorsOff();
        setMotor(n - 1, constrain(d, 0.0f, 100.0f));
        Serial.print("Мотор ");
        Serial.print(n);
        Serial.print(" (GP");
        Serial.print(motor_pins[n - 1]);
        Serial.print(") = ");
        Serial.print(constrain(d, 0.0f, 100.0f), 0);
        Serial.println("%. Выключить: stop");
        return;
    }
    if (strncmp(s, "plot", 4) == 0) {
        char *end;
        long v = strtol(s + 4, &end, 10);
        plot_on = (end == s + 4) ? !plot_on : (v != 0);
        return;
    }
    if (strncmp(s, "log", 3) == 0) {
        log_on = !log_on;
        Serial.println(log_on ? "Log on" : "Log off");
        return;
    }
    if (strncmp(s, "stop", 4) == 0) {
        demo_on = false;
        test_on = false;
        allMotorsOff();
        Serial.println("Stop");
        return;
    }

    char *end;
    float value = strtof(s, &end);
    if (end == s) return;              // не число
    processSample(value);
}

void readUsbCommands()
{
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (line_len > 0) {
                line[line_len] = 0;
                parseLine(line);
                line_len = 0;
            }
        } else if (c == '\b' || c == 0x7F) {   // Backspace — стереть последний символ
            if (line_len > 0) line_len--;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = c;
        }
    }
}

void runDemo(float dt)
{
    gen_value += demo_dir * demo_speed * dt;
    if (gen_value >= VALUE_MAX) { gen_value = VALUE_MAX; demo_dir = -1; }
    if (gen_value <= 0)         { gen_value = 0;         demo_dir = 1; }
}

// Возвращает true, когда текущий шаг теста закончен
bool runTestStep(float dt)
{
    const TestStep &s = cur_steps[test_step];
    uint32_t elapsed = millis() - test_step_start;

    switch (s.type) {
    case MOVE: {
        float total = fabsf(s.a - test_from);
        if (total < 0.5f) return true;
        float progress = fabsf(gen_value - test_from) / total;
        float v = s.b + (s.c - s.b) * progress;
        float dir = (s.a > test_from) ? 1.0f : -1.0f;
        gen_value += dir * v * dt;
        if ((dir > 0 && gen_value >= s.a) || (dir < 0 && gen_value <= s.a)) {
            gen_value = s.a;
            return true;
        }
        return false;
    }
    case HOLD:
        return elapsed >= s.ms;
    case WOBBLE:
        if (elapsed >= s.ms) {
            gen_value = s.a;
            return true;
        }
        gen_value = s.a + s.b * sinf(2.0f * PI * s.c * elapsed / 1000.0f);
        return false;
    }
    return true;
}

// Демо и тест: 50 значений в секунду
void runGenerators()
{
    if (!demo_on && !test_on) return;
    uint32_t now = millis();
    if (now - gen_last_ms < 20) return;
    float dt = (now - gen_last_ms) / 1000.0f;
    gen_last_ms = now;

    if (demo_on) {
        runDemo(dt);
    } else if (runTestStep(dt)) {
        if (test_step + 1 < cur_steps_count) {
            startTestStep(test_step + 1);
        } else {
            test_on = false;
            allMotorsOff();
            Serial.print("\n[");
            Serial.print(cur_title);
            Serial.println("] завершён");
            return;
        }
    }
    processSample(gen_value);
}

// Строка состояния: цифры + «картинка» ряда моторов, где символ = сила мотора
//   ' ' выкл, '.' слабо ... '@' максимум
// Машинный вывод для визуализатора, 25 раз в секунду:
//   P,<активны 0/1>,<входящее>,<позиция>,<скорость>,<ширина ±>,<сила %>,<мотор1 %>,...,<мотор10 %>
void printPlot()
{
    static uint32_t last_plot = 0;
    if (millis() - last_plot < PLOT_MS) return;
    last_plot = millis();

    char buf[40];
    snprintf(buf, sizeof(buf), "P,%d,%d,%d,%d,%d,%d",
             motors_active ? 1 : 0, (int)dbg_value, (int)dbg_position, (int)speed,
             (int)dbg_half_width, (int)dbg_amplitude);
    Serial.print(buf);
    for (int i = 0; i < N_MOTORS; i++) {
        Serial.print(',');
        Serial.print((int)(motor_duty[i] + 0.5f));
    }
    Serial.println();
}

void printState()
{
    if (plot_on) {
        printPlot();
        return;
    }
    static uint32_t last_print = 0;
    uint32_t interval = (demo_on || test_on) ? DEBUG_PRINT_FAST_MS : DEBUG_PRINT_MS;
    if (!log_on || !motors_active || millis() - last_print < interval) return;
    last_print = millis();

    const char LEVELS[] = " .:-=+*#%@";
    char bar[N_MOTORS + 1];
    for (int i = 0; i < N_MOTORS; i++) {
        int lvl = (motor_duty[i] < 1.0f) ? 0 : 1 + (int)(motor_duty[i] * 9.0f / 100.0f);
        if (lvl > 9) lvl = 9;
        bar[i] = LEVELS[lvl];
    }
    bar[N_MOTORS] = 0;

    char buf[120];
    snprintf(buf, sizeof(buf), "pos=%4d v=%4d +-%3d amp=%3d%% [%s] ",
             (int)dbg_position, (int)speed, (int)dbg_half_width, (int)dbg_amplitude, bar);
    Serial.print(buf);
    for (int i = 0; i < N_MOTORS; i++) {
        snprintf(buf, sizeof(buf), "%3d ", (int)motor_duty[i]);
        Serial.print(buf);
    }
    Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Pico started");
  Serial1.setTX(0);          // GP0
  Serial1.setRX(1);          // GP1
  Serial1.begin(115200);
  for (int i = 0; i < N_MOTORS; i++) {
    pwmInstance[i] = new RP2040_PWM(motor_pins[i], PWM_FREQ, 0);
    motor_duty[i] = 0;
  }
}

void loop() {
  readUartFrames();
  readUsbCommands();
  runGenerators();
  printState();

  // Защита: давно нет данных -> выключить всё и забыть историю
  if (motors_active && millis() - last_sample_time > TIMEOUT_MS) {
    allMotorsOff();
    hist_count = 0;
    hist_head = 0;
    Serial.println("Timeout, motors off");
  }
}
