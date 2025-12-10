# drone_monitor.py
from dataclasses import dataclass
import time
from pymavlink import mavutil  # для работы с протоколом MAVLink


@dataclass
class DroneState:
    """
    Простая структура с текущим состоянием дрона.
    Обновляется только в потоке мониторинга.
    Остальной код состояние ТОЛЬКО читает.
    """
    last_update: float = 0.0  # Время последнего обновления (time.time()).
    mode: str = ""  # Строковое имя режима (можно заполнить студенту).
    armed: bool = False  # True, если двигатели ARM-нуты.
    lat_deg: float = 0.0  # Широта в градусах (WGS84, lat/1e7 из GLOBAL_POSITION_INT)
    lon_deg: float = 0.0  # Долгота в градусах (WGS84, lon/1e7)
    alt_rel_m: float = 0.0  # Относительная высота, м (relative_alt/1000 из GLOBAL_POSITION_INT)
    battery_voltage_v: float = 0.0  # Напряжение батареи, В (voltage_battery/1000 из SYS_STATUS)
    battery_remaining_pct: float = 0.0  # Остаток батареи, % (battery_remaining из SYS_STATUS)


def _handle_heartbeat(master, msg, state: DroneState) -> None:
    """
    Обновление режима и статуса ARM по HEARTBEAT
    """
    # По MAV_MODE_FLAG_SAFETY_ARMED узнаём, включен ли ARM
    state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    
    # 2. Определяем режим по custom_mode.
    #
    # Pymavlink для ArduPilot умеет строить словарь "имя режима" -> "номер режима"
    # через master.mode_mapping()
    # В сообщении HEARTBEAT номер режима лежит в поле custom_mode
    #
    # У каждого msg есть ссылка на соединение в msg._master (это тот самый master,
    # который вы передаёте в monitor_loop)
    mode_mapping = master.mode_mapping()
    if not mode_mapping:
        return
    
    # mode_mapping: {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, ...}
    # Нужно обратить этот словарь: номер -> имя.
    inv_mode_mapping = {mode_id: name for name, mode_id in mode_mapping.items()}
    mode_id = msg.custom_mode  # текущее значение режима из HEARTBEAT.
    mode_name = inv_mode_mapping.get(mode_id)
    
    if mode_name is not None:
        state.mode = mode_name
    else:
        # Если режим неизвестен маппингу (например, кастомный), хотя бы сохраняем его номер.
        state.mode = f"UNKNOWN({mode_id})"


def _handle_global_position_int(msg, state: DroneState) -> None:
    """
    Обновление координат и относительной высоты по GLOBAL_POSITION_INT
    """
    state.lat_deg = msg.lat / 1e7
    state.lon_deg = msg.lon / 1e7
    state.alt_rel_m = msg.relative_alt / 1000.0  # мм -> м


def _handle_sys_status(msg, state: DroneState) -> None:
    """
    Обновление состояния батареи по SYS_STATUS
    """
    if msg.voltage_battery > 0:
        state.battery_voltage_v = msg.voltage_battery / 1000.0  # мВ -> В
        state.battery_remaining_pct = float(msg.battery_remaining)


def monitor_loop(master: mavutil.mavlink_connection,
                 state: DroneState,
                 stop_flag_getter=lambda: False) -> None:
    """
    Цикл опроса MAVLink-сообщений и обновления DroneState
    master: подключение к SITL/дрону.
    stop_flag_getter: функция без аргументов, возвращает True для остановки цикла.
    """
    while not stop_flag_getter():
        # Ждём сообщение
        msg = master.recv_match(
            blocking=True,
            timeout=1
        )
        now = time.time()
        
        if msg is None:
            # За это время просто ничего не пришло - ждём дальше.
            continue
        
        msg_type = msg.get_type()
        if msg_type == 'HEARTBEAT':
            _handle_heartbeat(master, msg, state)
        elif msg_type == 'GLOBAL_POSITION_INT':
            _handle_global_position_int(msg, state)
        elif msg_type == 'SYS_STATUS':
            _handle_sys_status(msg, state)
        
        state.last_update = now
 