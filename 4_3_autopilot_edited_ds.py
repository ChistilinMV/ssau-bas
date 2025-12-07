# =============================================================
# Скрипт для демонстрации коммуникации с автопилотом ArduCopter
# Автор: Владимир Добровольский
# Исправления и комментарии: Максим Чистилин
# =============================================================
import time
from pymavlink import mavutil # загрузка модуля mavutil из библиотеки pymavlink

# Константы
TARGET_SYSTEM = 1  # ID дрона
TARGET_COMPONENT = 1  # ID автопилота
GUIDED_MODE = 4  # ID режима ArduPilot 0=STABILIZE, 2=ALT_HOLD, 3=AUTO ...
TARGET_HIG = 3  # целевая высота

# отправка команд
def connect_to_autopilot(connection_string):
    """
    Подключение к SITL ArduPilot по TCP 127.0.0.1:14550
    и ожидание HEARTBEAT.
    """
    print(f"Подключаемся к автопилоту по адресу: {connection_string}")
    master = mavutil.mavlink_connection(connection_string) # Вызов конструктора класса mavlink_connection из модуля mavutil. Функция создает новый объект (master), который будет управлять всеми операциями связи по протоколу MAVLink. Объект настраивает сокет для сетевого соединения, но само подключение на этом этапе еще не установлено и данные не передаются.
    master.wait_heartbeat() # Метод переводит программу в режим блокирующего ожидания и начинает слушать открытый сетевой порт, ожидая получения сообщения типа HEARTBEAT от автопилота. HEARTBEAT — это служебное сообщение, которое автопилот и наземная станция периодически (обычно раз в секунду) отправляют для "подтверждения жизни" и обмена базовой информацией (тип системы, режим полета). Как только первое такое сообщение получено, метод завершает работу, а программа продолжает выполнение. Это гарантирует, что физическое соединение установлено и автопилот готов к обмену данными.
    print("Соединение было установлено")
    return master

def send_command(master, command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    """
    Подключение к дрону.
    При вызове send_command в нашей программе, мы передаём в качестве параметров только два аргумента, но функция command_long_send требует передачи 7 параметров. Поэтому мы используем именованные аркументы c param2 по param6 присваивая им значение 0 по умолчанию.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,  # confirmation
        param1, param2, param3, param4, param5, param6, param7
    )
    return master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

# миссия
def simple_mission():
    """
    Основная процедура миссии. Для симуляции использовался MissionPlanner SITL. 
    """
    curent_alt = 0  # высота для работы алгоритма (перенесена внутрь функции)

    # Подключение к симулятору
    print("Идет подключение к симулятору...")
    connection_string = 'tcp:127.0.0.1:14550'
    master = connect_to_autopilot(connection_string)

    # Переключаем режим
    print("Переключаем режим...", end='')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # говорит автопилоту, что используется "пользовательский режим", который задается в следующем поле
        GUIDED_MODE # Числовой код режима полета. В нашем коде он равен 4, что соответствует режиму GUIDED в автопилоте ArduPilot (ArduCopter)
    )
    print("OK")

    # Арминг
    print("Армимся...", end='')
    ack = send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=1)
    # вызов комманды с кодом 400=ARM_DISARM и параметром param1=1 заармить (включить)
    print("OK")

    # Взлёт
    print("Пытаемся взлететь...", end='')
    ack = send_command(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, param7=TARGET_HIG)
    print("OK")

    # Удержание высоты
    print("Пытаемся удерживать высоту")
    start_time = time.time()  # добавлена переменная start_time
    # Сравниваем текущую высоту с целевой умноженной на 0.98 для предотвращения бесконечного цикла из-за погрешности показаний датчиков и практической невозможности удерживать дрон на абслютной определённой высоте выше пределов точности позиционирования. Так-же контроль высоты ниже заданной позволяет учесть инерцию набора высоты.
    while curent_alt < TARGET_HIG * 0.98:
        msg = master.recv_match(
            type='GLOBAL_POSITION_INT',  # Какой тип сообщения ищем
            blocking=True,               # Блокирующий режим ожидания
            timeout=1                    # Таймаут ожидания (секунды)
            )
        if msg:
            curent_alt = msg.relative_alt / 1000.0  # исправлено relative_alt и деление на 1000.0
            print("Текущая высота:", curent_alt)
        if time.time() - start_time > 30:
            print("Ошибка времени ожидания. Прерывание")
            break

    # Блок ожидания
    print("Ждем 5 секунд...", end='')
    time.sleep(5)
    print("OK")

    # Посадка
    print("Выполняется посадка...", end='')
    ack = send_command(master, mavutil.mavlink.MAV_CMD_NAV_LAND)
    print("OK")

    # Дизарм
    print("Дизармимся...", end='')
    send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0)
    # вызов комманды с кодом 400=ARM_DISARM и параметром param1=0 дизармить (выключить)
    print("OK")

if __name__ == '__main__':
    simple_mission()