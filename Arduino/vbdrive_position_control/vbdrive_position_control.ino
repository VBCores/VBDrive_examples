#include <utility>

#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>

#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>
#include <voltbro/foc/state_simple_1_0.h>
#include <voltbro/foc/command_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>

/* ============================================================
 *                 ПАРАМЕТРЫ УПРАВЛЕНИЯ
 * ============================================================
 * KP, KD      — коэффициенты ПД-регулятора
 * A           — амплитуда задаваемой траектории
 * FREQ        — частота задаваемой траектории
 */
#define KP    25
#define KD    0.2
#define A     0.7
#define FREQ  0.5


/* ============================================================
 *                 ТАЙМЕРЫ
 * ============================================================
 * TIM5 — генерация задающего сигнала (1 кГц)
 * TIM7 — вывод данных в Serial (200 Гц)
 * TIM3 — отправка команд на мотор (1 кГц)
 */
HardwareTimer *timer_create_func  = new HardwareTimer(TIM5);
HardwareTimer *timer_show_data    = new HardwareTimer(TIM7);
HardwareTimer *timer_send_command = new HardwareTimer(TIM3);


/* ============================================================
 *        НАСТРОЙКА CYPHAL / CAN (служебная часть)
 *        !!! НЕ МЕНЯТЬ !!!
 * ============================================================
 */
void error_handler() {
    Serial.println("Unrecoverable commands error!");
    while (1) {}
}

UtilityConfig utilities(micros, error_handler);

// Алиасы типов сообщений Cyphal
TYPE_ALIAS(HBeat,      uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(FocState,   voltbro_foc_state_simple_1_0);
TYPE_ALIAS(FOCCommand, voltbro_foc_command_1_0)

// Статусы ноды
static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE          = uavcan_node_Mode_1_0_INITIALIZATION;
static CanardNodeID NODE_ID;

constexpr uint64_t MICROS_S = 1'000'000;

// CAN / Cyphal объекты
CanFD* canfd;
FDCAN_HandleTypeDef* hfdcan1;
static bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;


/* ============================================================
 *        ПЕРЕМЕННЫЕ УПРАВЛЕНИЯ МОТОРОМ
 * ============================================================
 */
float target_angle   = 0.0;   // задаваемый угол
float target_vel     = 0.0;   // задаваемая скорость
float received_angle = 0.0;   // текущий угол с мотора


/* ============================================================
 *        ФЛАГИ ДЛЯ РАБОТЫ В loop()
 * ============================================================
 * Устанавливаются в обработчиках таймеров
 */
int flag_send_comm = 0;
int flag_show_data = 0;


/* ============================================================
 *        ОБЪЯВЛЕНИЯ ФУНКЦИЙ
 * ============================================================
 */
void heartbeat();
void set_flag_send_command();
void set_flag_show_data();
void create_func();
void send_command();


/* ============================================================
 *        ПОДПИСЧИКИ CYPHAL
 * ============================================================
 */

// Подписка на состояние мотора
class FOCStateSub : public AbstractSubscription<FocState> {
public:
    FOCStateSub(InterfacePtr interface)
        : AbstractSubscription<FocState>(interface, 3811) {} // порт состояния мотора

    void handler(const FocState::Type& msg, CanardRxTransfer*) override {
        received_angle = msg.angle.radian; // сохраняем текущий угол
        digitalToggle(LED2);               // индикация приема сообщения
    }
};
FOCStateSub* foc_state_sub = nullptr;


// Подписка на heartbeat мотора
class HBeatSub : public AbstractSubscription<HBeat> {
public:
    HBeatSub(InterfacePtr interface)
        : AbstractSubscription<HBeat>(interface, 7509) {} // порт heartbeat

    void handler(const HBeat::Type&, CanardRxTransfer*) override {
        // heartbeat принимается, но не используется
    }
};
HBeatSub* hbeat_sub = nullptr;


/* ============================================================
 *        НАСТРОЙКА CAN И CYPHAL
 * ============================================================
 */
void can_config(int ID) {

    SystemClock_Config();

    canfd = new CanFD();
    canfd->init();
    canfd->write_default_params(); // 1000k nominal / 8M data
    canfd->apply_config();
    hfdcan1 = canfd->get_hfdcan();
    canfd->default_start();

    NODE_ID = ID;

    size_t queue_len = 200;
    cyphal_interface = std::shared_ptr<CyphalInterface>(
        CyphalInterface::create_heap<G4CAN, O1Allocator>(
            NODE_ID,
            hfdcan1,
            queue_len,
            utilities
        )
    );

    hbeat_sub     = new HBeatSub(cyphal_interface);
    foc_state_sub = new FOCStateSub(cyphal_interface);

    _is_cyphal_on = true;
    CYPHAL_MODE   = uavcan_node_Mode_1_0_OPERATIONAL;
}


/* ============================================================
 *        SETUP
 * ============================================================
 */
void setup() {

    Serial.begin(115200);
    pinMode(LED2, OUTPUT);

    can_config(2);

    // Таймер генерации траектории
    timer_create_func->pause();
    timer_create_func->setOverflow(1000, HERTZ_FORMAT);
    timer_create_func->attachInterrupt(create_func);
    timer_create_func->refresh();
    timer_create_func->resume();

    // Таймер вывода данных
    timer_show_data->pause();
    timer_show_data->setOverflow(200, HERTZ_FORMAT);
    timer_show_data->attachInterrupt(set_flag_show_data);
    timer_show_data->refresh();
    timer_show_data->resume();

    // Таймер отправки команд
    timer_send_command->pause();
    timer_send_command->setOverflow(1000, HERTZ_FORMAT);
    timer_send_command->attachInterrupt(set_flag_send_command);
    timer_send_command->refresh();
    timer_send_command->resume();
}


/* ============================================================
 *        LOOP
 * ============================================================
 */
void loop() {

    // Обработка входящих/исходящих Cyphal сообщений
    cyphal_interface->loop();

    static uint32_t t_heartbeat = 0;
    uint32_t now = millis();

    // Heartbeat — обязательное сообщение Cyphal (1 Гц)
    if (now - t_heartbeat >= 1000) {
        heartbeat();
        t_heartbeat = now;
    }

    // Вывод данных по таймеру
    if (flag_show_data) {
        Serial.print(target_angle);
        Serial.print(" ");
        Serial.println(received_angle);
        flag_show_data = 0;
    }

    // Отправка команды мотору по таймеру
    if (flag_send_comm) {
        send_command();
        flag_send_comm = 0;
    }
}


/* ============================================================
 *        ОТПРАВКА КОМАНДЫ МОТОРУ
 * ============================================================
 */
void send_command() {

    FOCCommand::Type command_msg;

    command_msg.angle.radian = target_angle;
    command_msg.position_feedback_gain.value = KP;

    command_msg.velocity.radian_per_second = target_vel;
    command_msg.velocity_feedback_gain.value = KD;

    command_msg._torque.newton_meter = 0;

    command_msg.I_kp.value = 5;
    command_msg.I_ki.value = 1300;

    static CanardTransferID command_transfer_id = 0;
    cyphal_interface->send_msg<FOCCommand>(
        &command_msg,
        2118,
        &command_transfer_id
    );
}


/* ============================================================
 *        ГЕНЕРАЦИЯ ЗАДАЮЩЕЙ ТРАЕКТОРИИ
 * ============================================================
 */
int sign(float val) {
    if (val < 0) return -1;
    if (val == 0) return 0;
    return 1;
}

void create_func() {

    static float amplitude = A;
    static float freq      = FREQ;

    static uint32_t t0 = millis();
    uint32_t time_dot = millis() - t0;

    float t = float(time_dot) / 1000.0;

    // === Выберите тип траектории ===

    // Меандр
    // target_angle = amplitude * sign(sin(2 * PI * freq * t));
    // target_vel   = 0;

    // Синус
    // target_angle = amplitude * sin(2 * PI * freq * t);
    // target_vel   = amplitude * 2 * PI * freq * cos(2 * PI * freq * t);

    // Треугольник
    target_angle = amplitude - (2 * amplitude / PI) * acos(cos(2 * PI * freq * t - PI / 2));
    target_vel   = (-4 * amplitude * freq) * sin(2 * PI * freq * t - PI / 2)
                  / sqrt(1 - sq(cos(2 * PI * freq * t - PI / 2)));
}


/* ============================================================
 *        ОБРАБОТЧИКИ ТАЙМЕРОВ
 * ============================================================
 */
void set_flag_show_data() {
    flag_show_data = 1;
}

void set_flag_send_command() {
    flag_send_comm = 1;
}


/* ============================================================
 *        HEARTBEAT CYPHAL
 * ============================================================
 */
void heartbeat() {

    static CanardTransferID hbeat_transfer_id = 0;

    HBeat::Type heartbeat_msg = {
        .uptime = int(millis() / 1000),
        .health = {CYPHAL_HEALTH_STATUS},
        .mode   = {CYPHAL_MODE}
    };

    if (_is_cyphal_on) {
        cyphal_interface->send_msg<HBeat>(
            &heartbeat_msg,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            &hbeat_transfer_id,
            MICROS_S * 2
        );
    }
}
