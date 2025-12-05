# =====================================================
# Скрипт для создания и загрузки миссии в полетный контроллер ArduCopter
# Автор: Дмитрий Мокачев
# Используется: pymavlink + pyproj для геодезии WGS84
# =====================================================

"""Домашнее задание
Ваша цель внести изменения в полётное задание, добавив в него дополнительные точки (waypoints) с другим параметром высоты и проверить в симуляторе работу кода
1. Установите приложение «Mission Planner»
2. Запустите симуляцию дрона из Mission Planner. Включите зеркалирование протокола Mavlink. Для этого перейдите во вкладку «Simulation», нажмите «Multirotor» и выберите стабильную версию симулятора. Зеркалирование Mavlink: CTRL+F mavlink ТСР + отметить «write access» start
3. Реализуйте в коде добавление двух дополнительных точек (waypoints), высоту полёта укажите 50 метров. Запустите код.
4. Дождитесь пока аппарат перейдет в состояние «Ready to arm». Состояние контролируется на вкладке «Data» в Mission planner.
5. Дайте команду «Arm». Дождитесь переключения аппарата из состояния Disarm в Arm. Потом дайте команду «Takeoff» для взлета (команда в контекстном меню при нажатии правой кнопки мышки).
6. Установите режим «Auto» на вкладке «Data» меню «Действие».
7. Убедитесь, что дрон выполняет загруженное вами полётное задание."""

# подключаем нужные библиотеки
from pymavlink import mavutil, mavwp  # для связи с дроном по MAVLink
from pyproj import Geod               # Точные геодезические расчеты (WGS84)
import math                           # Математические функции (гипотенуза, углы)

# Для расчета координат
geod = Geod(ellps="WGS84")  # ellps="WGS84" = стандарт Земли для GPS


def connect():
    """
    Подключение к SITL ArduPilot по TCP 127.0.0.1:14550
    и ожидание HEARTBEAT.
    """
    master = mavutil.mavlink_connection('tcp:100.70.31.246:14550')
    master.wait_heartbeat()  # ждет первое сообщение от дрона

    print(f"Подключено к системе {master.target_system}, компонент {master.target_component}")

    return master


def get_current_position(master):
    """
    Получить широту и долготу из сообщения GLOBAL_POSITION_INT.
    Возвращает (lat_deg, lon_deg).
    """
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  # Ждем сообщение
    msg = msg.to_dict()  # Преобразуем в словарь Python

    lat = msg["lat"] / 1e7  # degE7 -> градусы (например, 377123456 → 37.7123456°)
    lon = msg["lon"] / 1e7

    print(f"Текущие координаты: lat={lat:.7f}, lon={lon:.7f}")
    return lat, lon


def add_waypoint_latlon(
    wp_loader: mavwp.MAVWPLoader,  # Контейнер для точек миссии
    master: mavutil.mavfile,       # MAVLink соединение
    lat_deg: float,                # Широта в градусах
    lon_deg: float,                # Долгота в градусах
    alt_m: float,                  # Высота в метрах (относительная)
    current: int = 0,              # current=1 означает "текущая точка"
    frame: int = None,             # Система координат миссии
    command: int = None,           # Тип команды (NAV_WAYPOINT = лететь к точке)
):
    """
    Добавить точку миссии по абсолютным координатам lat/lon (в градусах).
    Координаты конвертируются в формат degE7 для MISSION_ITEM_INT.
    """
    if frame is None:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT  # относительная высота над землёй
    if command is None:
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT  # лететь к точке

    autocontinue = 1

    lat_int = int(lat_deg * 1e7)
    lon_int = int(lon_deg * 1e7)

    wp_loader.add(
        mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system,       # ID дрона
            master.target_component,    # ID автопилота
            0,                          # Временный порядковый номер точки (перезапишется)
            frame,                      # Система координат
            command,                    # Команда
            current,                    # Текущая точка да/нет
            autocontinue,               # Продолжать автоматически да/нет
            0, 0, 0, 0,                 # param1-4 (радиус, задержка и т.д. - не используем)
            lat_int, lon_int, alt_m     # Координаты и высота
        )
    )


def add_waypoint_offset_m(
    wp_loader: mavwp.MAVWPLoader,
    master: mavutil.mavfile,
    base_lat_deg: float,
    base_lon_deg: float,
    north_m: float,
    east_m: float,
    alt_m: float,
    current: int = 0,
    frame: int = None,
    command: int = None,
):
    """
    Добавить точку миссии по относительному смещению в метрах
    от базовой точки (base_lat_deg/base_lon_deg).

    north_m > 0  -> смещение на север
    east_m  > 0  -> смещение на восток
    """
    if frame is None:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    if command is None:
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

    # Длина вектора и азимут (0° = север, 90° = восток)
    distance = math.hypot(north_m, east_m)
    if distance == 0:
        azimuth_deg = 0.0
    else:
        azimuth_deg = math.degrees(math.atan2(east_m, north_m))  # arctan(y/x), x - ось севера, y - ось востока

    # Вычисляем новые координаты по геодезии WGS84
    # geod.fwd принимает (lon, lat, azimuth_deg, distance_m)
    lon_new, lat_new, _ = geod.fwd(base_lon_deg, base_lat_deg, azimuth_deg, distance)

    # Используем функцию добавления по абсолютным координатам
    add_waypoint_latlon(
        wp_loader,
        master,
        lat_new,
        lon_new,
        alt_m,
        current=current,
        frame=frame,
        command=command,
    )

    return lat_new, lon_new  # опционально можно использовать снаружи


def build_mission(master, lat_deg, lon_deg, alt_m=50.0):
    """
    Создаем список точек полета

    Пример миссии:
    • Точка 0: текущая позиция (current=1)
    • Точка 1: 100м севернее
    • Точка 2: 100м восточнее
    """
    wp = mavwp.MAVWPLoader()  # загрузчик точек миссии

    # Точка 0: текущая позиция, текущий пункт миссии
    add_waypoint_latlon(
        wp_loader=wp,
        master=master,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        alt_m=alt_m,
        current=1,  # текущий пункт
    )

    # Точка 1: смещение в метрах от текущей позиции
    add_waypoint_offset_m(
        wp_loader=wp,
        master=master,
        base_lat_deg=lat_deg,
        base_lon_deg=lon_deg,
        north_m=150.0,   # 100 м на север
        east_m=-100.0,    # 100 м на восток
        alt_m=alt_m,
        current=0,
    )

    # Точка 2: смещение в метрах от текущей позиции
    add_waypoint_offset_m(
        wp_loader=wp,
        master=master,
        base_lat_deg=lat_deg,
        base_lon_deg=lon_deg,
        north_m=150.0,   # на север
        east_m=100.0,    # на восток
        alt_m=alt_m,
        current=0,
    )
    
    # Точка 3 (доп. точка 1): на восток от старта
    add_waypoint_offset_m(
        wp_loader=wp,
        master=master,
        base_lat_deg=lat_deg,
        base_lon_deg=lon_deg,
        north_m=0.0,
        east_m=100.0,
        alt_m=alt_m,
        current=0,
    )

    # Точка 4 (доп. точка 2): возврат в стартовую точку (можно изменить смещение под свою задачу)
    add_waypoint_offset_m(
        wp_loader=wp,
        master=master,
        base_lat_deg=lat_deg,
        base_lon_deg=lon_deg,
        north_m=0.0,
        east_m=0.0,
        alt_m=alt_m,
        current=0,
    )

    return wp


def upload_mission(master, wp_loader):
    """
    Загрузить миссию в автопилот по протоколу MAVLink Mission:
    CLEAR_ALL -> COUNT -> (REQUEST / REQUEST_INT -> ITEM / ITEM_INT) * N -> ACK.
    """
    # Очистить старую миссию
    master.waypoint_clear_all_send()

    count = wp_loader.count()
    print(f"Загружаем миссию из {count} точек")
    master.waypoint_count_send(count)

    for i in range(count):
        # Ждём запрос очередного пункта
        msg = master.recv_match(
            type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
            blocking=True
        )
        seq = msg.seq
        print(f"Отправка пункта миссии seq={seq}")
        master.mav.send(wp_loader.wp(seq))

    # Ждём подтверждение
    ack = master.recv_match(type='MISSION_ACK', blocking=True)
    print(f"MISSION_ACK: {ack}")


def main():

    # 1. Подключиться к дрону
    master = connect()

    # 2. Получить текущие координаты и вывести
    lat, lon = get_current_position(master)

    # 3. Создать простую миссию с использованием новых функций
    wp_loader = build_mission(master, lat, lon, alt_m=50.0)

    # 4. Загрузить полётное задание в контроллер
    upload_mission(master, wp_loader)


if __name__ == "__main__":
    main()
