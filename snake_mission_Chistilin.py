#======================================================================
# Программа загрузки полётного задания в дрон и контроля его выполнения
# Автор Чистилин Максим 10.12.2025
#======================================================================
import sys
import time
import math
import threading

from pymavlink import mavutil, mavwp
from pyproj import Geod

from drone_monitor import DroneState, monitor_loop
from flight_control import (
    arm,
    takeoff,
    land,
    set_mode_guided,
    set_mode_auto,
    disarm,
)

# Геодезия WGS84
geod = Geod(ellps="WGS84")


def connect(connection_string: str = "tcp:127.0.0.1:14550") -> mavutil.mavlink_connection:
    """
    Подключение к SITL/дрону ArduPilot и ожидание HEARTBEAT.
    """
    master = mavutil.mavlink_connection(connection_string)
    result = master.wait_heartbeat()
    if result is None:
        return None
    print(f"Подключено к системе {master.target_system}, компонент {master.target_component}")
    return master


def wait_for_coordinates(state: DroneState, timeout: float = 15.0) -> bool:
    """
    Ожидаем, пока модуль мониторинга получит актуальные координаты.
    """
    print("Шаг 1. Ожидаем текущие координаты дрона (GPS/GLOBAL_POSITION_INT)...")
    end_time = time.time() + timeout
    while time.time() < end_time:
        if state.last_update > 0 and not (state.lat_deg == 0.0 and state.lon_deg == 0.0):
            print(
                f"Координаты получены: lat={state.lat_deg:.7f}, "
                f"lon={state.lon_deg:.7f}, alt_rel={state.alt_rel_m:.1f} м"
            )
            return True
        print(" координаты ещё не получены, ждём 1 секунду...")
        time.sleep(1.0)
    print("Не удалось получить координаты за отведённое время.")
    return False


def wait_for_altitude(
    state: DroneState,
    target_alt_m: float,
    tolerance_m: float = 1.0,
    timeout: float = 40.0,
) -> bool:
    """
    Ждем, пока относительная высота не выйдет на заданный уровень.
    """
    print(f"Ждём набора высоты ~{target_alt_m} м...")
    end_time = time.time() + timeout
    while time.time() < end_time:
        alt = state.alt_rel_m
        print(f" текущая высота: {alt:.1f} м")
        if alt >= target_alt_m - tolerance_m:
            print("Целевая высота достигнута (с учётом допуска).")
            return True
        time.sleep(1.0)
    print("Высота не достигнута за отведённое время.")
    return False


# --- Функции для работы с миссией, загрузка путевых точек полётного задания ---


def add_waypoint_latlon(
    wp_loader: mavwp.MAVWPLoader,
    master: mavutil.mavfile,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    current: int = 0,
    frame: int | None = None,
    command: int | None = None,
):
    """
    Добавить точку миссии по абсолютным координатам lat/lon (в градусах).
    """
    if frame is None:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    if command is None:
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    autocontinue = 1

    lat_int = int(lat_deg * 1e7)
    lon_int = int(lon_deg * 1e7)

    wp_loader.add(
        mavutil.mavlink.MAVLink_mission_item_int_message(
            master.target_system,
            master.target_component,
            0,          # seq перезапишет автопилот
            frame,
            command,
            current,
            autocontinue,
            0, 0, 0, 0,  # param1-4
            lat_int,
            lon_int,
            alt_m,
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
    frame: int | None = None,
    command: int | None = None,
):
    """
    Добавить точку миссии по относительному смещению в метрах
    от базовой точки (base_lat_deg/base_lon_deg).
    north_m > 0 -> север, east_m > 0 -> восток.
    """
    if frame is None:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    if command is None:
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

    distance = math.hypot(north_m, east_m)
    if distance == 0:
        azimuth_deg = 0.0
    else:
        azimuth_deg = math.degrees(math.atan2(east_m, north_m))

    lon_new, lat_new, _ = geod.fwd(base_lon_deg, base_lat_deg, azimuth_deg, distance)

    add_waypoint_latlon(
        wp_loader=wp_loader,
        master=master,
        lat_deg=lat_new,
        lon_deg=lon_new,
        alt_m=alt_m,
        current=current,
        frame=frame,
        command=command,
    )

    return lat_new, lon_new


def build_snake_mission(
    master: mavutil.mavfile,
    lat_deg: float,
    lon_deg: float,
    alt_m: float = 20.0,
) -> mavwp.MAVWPLoader:
    wp = mavwp.MAVWPLoader()

    # Точка 0: текущая позиция (home), current=1
    add_waypoint_latlon(
        wp_loader=wp,
        master=master,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        alt_m=alt_m,
        current=1,
    )

    # Точка 1: 240 м на восток (прямой участок)
    add_waypoint_offset_m(
        wp_loader=wp,
        master=master,
        base_lat_deg=lat_deg,
        base_lon_deg=lon_deg,
        north_m=0.0,
        east_m=240.0,
        alt_m=alt_m,
        current=0,
    )

    # Точка 2: ещё 50 м на восток — начало змейки (итого 290 м от старта)
    add_waypoint_offset_m(
        wp_loader=wp,
        master=master,
        base_lat_deg=lat_deg,
        base_lon_deg=lon_deg,
        north_m=0.0,
        east_m=290.0,
        alt_m=alt_m,
        current=0,
    )

    # Параметры змейки: теперь стартуем от east=290
    snake_start_east = 290.0
    snake_length = 100.0
    snake_step = 10.0
    snake_width = 50.0

    current_north = 0.0
    line_dir = +1   # +1: на восток, -1: на запад

    while current_north <= snake_width + 1e-3:
        if line_dir > 0:
            line_end_east = snake_start_east + snake_length
        else:
            line_end_east = snake_start_east - snake_length

        add_waypoint_offset_m(
            wp_loader=wp,
            master=master,
            base_lat_deg=lat_deg,
            base_lon_deg=lon_deg,
            north_m=current_north,
            east_m=line_end_east,
            alt_m=alt_m,
            current=0,
        )

        current_north += snake_step
        if current_north > snake_width + 1e-3:
            break

        add_waypoint_offset_m(
            wp_loader=wp,
            master=master,
            base_lat_deg=lat_deg,
            base_lon_deg=lon_deg,
            north_m=current_north,
            east_m=line_end_east,
            alt_m=alt_m,
            current=0,
        )

        line_dir *= -1

    # Возврат домой (0, 0)
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


def upload_mission(master: mavutil.mavfile, wp_loader: mavwp.MAVWPLoader):
    """
    Загрузка миссии, как в create_mission.py:
    CLEAR_ALL -> COUNT -> (REQUEST / REQUEST_INT -> ITEM_INT) * N -> ACK.
    """
    master.waypoint_clear_all_send()

    count = wp_loader.count()
    print(f"Загружаем миссию из {count} точек")
    master.waypoint_count_send(count)

    for i in range(count):
        msg = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
            blocking=True,
        )
        seq = msg.seq
        print(f"Отправка пункта миссии seq={seq}")
        master.mav.send(wp_loader.wp(seq))

    ack = master.recv_match(type="MISSION_ACK", blocking=True)
    print(f"MISSION_ACK: {ack}")


def main():
    # 1. Подключение
    master = connect()
    if master is None:
        print("Проверьте подключение к дрону (SITL/Mission Planner).")
        sys.exit(1)

    # 2. Мониторинг
    state = DroneState()
    stop_flag = {"stop": False}

    monitor_thread = threading.Thread(
        target=monitor_loop,
        args=(master, state, lambda: stop_flag["stop"]),
        daemon=False,
    )
    monitor_thread.start()
    print("Поток мониторинга запущен.")

    # используем блок try для обработки исключений, которые могут возникнуть при выполнении кода.
    try:
        # Ждем координаты
        if not wait_for_coordinates(state):
            sys.exit(1)

        TARGET_ALT_M = 20.0

        # Берем текущие lat/lon из DroneState
        lat0 = state.lat_deg
        lon0 = state.lon_deg

        # Останавливаем мониторинг на время загрузки миссии
        print("Останавливаем мониторинг на время загрузки миссии...")
        stop_flag["stop"] = True
        monitor_thread.join(timeout=2.0)
        print("Мониторинг остановлен.")

        # Строим и загружаем миссию «змейкой» через MAVWPLoader
        wp_loader = build_snake_mission(master, lat0, lon0, alt_m=TARGET_ALT_M)
        upload_mission(master, wp_loader)

        # Снова запускаем мониторинг
        stop_flag = {"stop": False}
        monitor_thread = threading.Thread(
            target=monitor_loop,
            args=(master, state, lambda: stop_flag["stop"]),
            daemon=False,
        )
        monitor_thread.start()
        print("Мониторинг снова запущен.")

        # GUIDED -> ARM -> TAKEOFF -> AUTO
        print("Перевод в режим GUIDED...")
        set_mode_guided(master)

        print("ARM двигателей...")
        arm(master)
        for _ in range(10):
            if state.armed:
                print(" Дрон ARM")
                break
            print(" дрон ещё не ARM, ждём...")
            time.sleep(1.0)
        else:
            print("Не удалось включить ARM, завершаем.")
            sys.exit(1)

        print(f"Взлёт до {TARGET_ALT_M} м...")
        takeoff(master, TARGET_ALT_M)

        if not wait_for_altitude(state, TARGET_ALT_M, tolerance_m=1.0, timeout=60.0):
            print("Взлёт не удался, завершаем.")
            sys.exit(1)

        print("Перевод в AUTO для выполнения миссии...")
        set_mode_auto(master)

        # Грубое ожидание завершения миссии
        MISSION_WAIT_S = 300
        start_wait = time.time()
        while time.time() - start_wait < MISSION_WAIT_S:
            print(
                f" t={int(time.time() - start_wait):03d}s "
                f"alt={state.alt_rel_m:.1f} м armed={state.armed} mode='{state.mode}'"
            )
            time.sleep(2.0)

        print("Команда LAND (посадка)...")
        land(master)

        # Ждем фактического приземления по высоте и делаем DISARM
        for _ in range(120):  # до ~120 секунд
            alt = state.alt_rel_m
            print(
                f" Посадка... alt={alt:.1f} м "
                f"armed={state.armed} mode='{state.mode}'"
            )
            if alt <= 0.5:
                print(" Высота ~0 м, отправляем DISARM...")
                disarm(master)
                break
            time.sleep(1.0)

        # Дополнительный контроль, что реально disarm-нулся
        for _ in range(10):
            if not state.armed:
                print(" Дрон DISARM, посадка завершена.")
                break
            print(" Ждём подтверждения DISARM...")
            time.sleep(1.0)

        for _ in range(60):
            print(
                f" Посадка... alt={state.alt_rel_m:.1f} м "
                f"armed={state.armed} mode='{state.mode}'"
            )
            if not state.armed:
                print(" Дрон DISARM, посадка завершена.")
                break
            time.sleep(1.0)

    finally:
        print("Останавливаем поток мониторинга...")
        stop_flag["stop"] = True
        monitor_thread.join(timeout=2.0)
        print("Поток мониторинга остановлен, программа завершается.")


if __name__ == "__main__":
    main()
