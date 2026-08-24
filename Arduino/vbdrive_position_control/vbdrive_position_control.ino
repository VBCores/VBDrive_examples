/*
Документация к прошивке VBDrive https://github.com/VBCores/VBDrive
Библиотека для работы с cyphal https://github.com/VBCores/libcxxcanard
Библиотека для работы с VBCoreG4 https://github.com/VBCores/VBCoreG4_arduino_system 
Документация к VBCoreG4 https://docs.vbcores.ru 
*/
#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>

/* ============================================================
 *                 ПАРАМЕТРЫ УПРАВЛЕНИЯ
 * ============================================================
 * KP, KD      — коэффициенты ПД-регулятора
 * A           — амплитуда задаваемой траектории
 * FREQ        — частота задаваемой траектории
 */
#define KP    30
#define KD    0.8
#define A     0.7
#define FREQ  0.5


/* ============================================================
 *                 ТАЙМЕРЫ
 * ============================================================
 * TIM5 — генерация задающего сигнала (1 кГц)
 * TIM7 — вывод данных в Serial (200 Гц)
 * TIM3 — отправка команд на мотор (1 кГц)
 */
HardwareTimer *timer_create_func = new HardwareTimer(TIM5);
HardwareTimer *timer_show_data = new HardwareTimer(TIM7);
HardwareTimer *timer_send_command = new HardwareTimer(TIM3);


/* ============================================================
 *              НАСТРОЙКА CYPHAL / CAN (служебная часть)
 *                          НЕ МЕНЯТЬ
 * ============================================================
 */
constexpr CanardNodeID NODE_ID = 2;
constexpr CanardPortID FOC_STATE_RX_PORT_ID = 3811; // текущее состояние VBDrive
constexpr CanardPortID FOC_COMMAND_TX_PORT_ID = 2118; // 2107 + VBDrive ID

CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

static CanardTransferID command_transfer_id = 0;


/* ============================================================
 *        ПЕРЕМЕННЫЕ УПРАВЛЕНИЯ МОТОРОМ
 * ============================================================
 */
float target_angle = 0.0;
float target_vel = 0.0;
float received_angle = 0.0;


/* ============================================================
 *        ФЛАГИ
 * ============================================================
 */
volatile bool flag_send_comm = false;
volatile bool flag_show_data = false;


/* ============================================================
 *        ОБЪЯВЛЕНИЯ ФУНКЦИЙ
 * ============================================================
 */
void set_flag_send_command();
void set_flag_show_data();
void create_func();
void send_command();


/* ============================================================
 *        ПОДПИСКИ CYPHAL
 * ============================================================
 */

//На состояние привода - угол, скорость, момент и т.п
void foc_state_handler(const FocState& msg, CanardRxTransfer* transfer) {
    received_angle = msg.angle.radian;
}

//Heartbeat - сообщение, которое сигнализирует о том, что привод в сети и передает данные
void heartbeat_handler(const Heartbeat& msg, CanardRxTransfer* transfer) {
    digitalToggle(LED2);
}


/* ============================================================
 *        НАСТРОЙКА CAN И CYPHAL
 * ============================================================
 */
void can_config(int ID) {
    SystemClock_Config();

    canfd.init();
    canfd.write_default_params();
    canfd.apply_config();

    cyphal = make_cyphal<ArduinoCyphal<>>(canfd.get_hfdcan(), ID, "org.vbcores.vbdrive");

    cyphal->subscribe(FOC_STATE_RX_PORT_ID, foc_state_handler);
    cyphal->subscribe(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, heartbeat_handler);

    cyphal->begin();
}


void setup() {
    Serial.begin(115200);
    pinMode(LED2, OUTPUT);

    can_config(NODE_ID);


    // Генерация траектории — 1 кГц
    timer_create_func->pause();
    timer_create_func->setOverflow(1000, HERTZ_FORMAT);
    timer_create_func->attachInterrupt(create_func);
    timer_create_func->refresh();
    timer_create_func->resume();


    // Вывод данных — 200 Гц
    timer_show_data->pause();
    timer_show_data->setOverflow(200, HERTZ_FORMAT);
    timer_show_data->attachInterrupt(set_flag_show_data);
    timer_show_data->refresh();
    timer_show_data->resume();


    // Отправка команды — 1 кГц
    timer_send_command->pause();
    timer_send_command->setOverflow(1000, HERTZ_FORMAT);
    timer_send_command->attachInterrupt(set_flag_send_command);
    timer_send_command->refresh();
    timer_send_command->resume();
}


void loop() {
    cyphal->cyphal_loop();

    if (flag_show_data) {
        Serial.print(target_angle);
        Serial.print(" ");
        Serial.println(received_angle);

        flag_show_data = false;
    }

    if (flag_send_comm) {
        send_command();
        flag_send_comm = false;
    }
}


/* ============================================================
 *        ОТПРАВКА КОМАНДЫ МОТОРУ
 * ============================================================
 */
void send_command() {
    voltbro_foc_command_1_0 command_msg{};

    command_msg.angle.radian = target_angle;
    command_msg.position_feedback_gain.value = KP;

    command_msg.velocity.radian_per_second = target_vel;
    command_msg.velocity_feedback_gain.value = KD;

    command_msg._torque.newton_meter = 0;

    //I_kp, I_ki  лучше не трогать
    command_msg.I_kp.value = 4;
    command_msg.I_ki.value = 1600;

    cyphal->send_msg(&command_msg, FOC_COMMAND_TX_PORT_ID, &command_transfer_id);
}


int sign(float val) {
    if (val < 0) return -1;
    if (val == 0) return 0;
    return 1;
}

/* ============================================================
 *        ГЕНЕРАЦИЯ ЗАДАЮЩЕЙ ТРАЕКТОРИИ
 * ============================================================
 */
void create_func() {
    static float amplitude = A;
    static float freq = FREQ;
    static uint32_t t0 = millis();

    uint32_t time_dot = millis() - t0;
    float t = float(time_dot) / 1000.0;

    /* ----- Доступны три траектории, выберите одну, оставшиеся две должны быть закомментированы ----- */
    
    // Меандр
    target_angle = amplitude * sign(sin(2 * PI * freq * t));
    target_vel = 0;


    // Синус
    // target_angle = amplitude * sin(2 * PI * freq * t);
    // target_vel = amplitude * 2 * PI * freq * cos(2 * PI * freq * t);


    // Треугольник
    // target_angle = amplitude - (2 * amplitude / PI) * acos(cos(2 * PI * freq * t - PI / 2));

    // target_vel = (-4 * amplitude * freq) * sin(2 * PI * freq * t - PI / 2)
    //            / sqrt(1 - sq(cos(2 * PI * freq * t - PI / 2)));
}


/* ============================================================
 *        ОБРАБОТЧИКИ ТАЙМЕРОВ
 * ============================================================
 */
void set_flag_show_data() {
    flag_show_data = true;
}

void set_flag_send_command() {
    flag_send_comm = true;
}