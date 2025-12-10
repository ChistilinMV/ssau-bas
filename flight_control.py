# flight_control.py
import time
from pymavlink import mavutil

def set_mode(master: mavutil.mavlink_connection, mode_name: str, timeout: float = 5.0) -> None:
    """
    Универсальная функция смены режима полёта через SET_MODE
    - mode_name: строковое имя режима, например "GUIDED" или "AUTO".
    - mode_mapping() берётся из самого автопилота (через pymavlink), так что
    код не привязан к жёстко зашитым номерам custom_mode
    """
    mode_mapping = master.mode_mapping()
    if mode_mapping is None or mode_name not in mode_mapping:
        raise ValueError(f"Режим {mode_name} недоступен в mode_mapping()")
    
    mode_id = mode_mapping[mode_name]
    # Отправляем SET_MODE с флагом MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    # и номером режима в custom_mode (flightmode number)
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    # Небольшая пауза, чтобы автопилот успел сменить режим и прислать новый HEARTBEAT
    end_time = time.time() + timeout
    while time.time() < end_time:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg is None:
            continue
        # можно реализовать проверку, что режим действительно изменился
        break

def set_mode_guided(master: mavutil.mavlink_connection) -> None:
    """
    Переводит Copter в режим GUIDED (управление с компьютера/скрипта)
    """
    set_mode(master, "GUIDED")

def set_mode_auto(master: mavutil.mavlink_connection) -> None:
    """
    Переводит Copter в режим AUTO для выполнения загруженной миссии
    """
    set_mode(master, "AUTO")

def arm(master: mavutil.mavlink_connection, force: bool = False) -> None:
    """
    ARM двигателей командой MAV_CMD_COMPONENT_ARM_DISARM
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # param1: 1 = arm, 0 = disarm
        0 if not force else 21196,  # param2: 21196 => принудит. дизарм в полёте (для справки)
        0, 0, 0, 0, 0
    )

def disarm(master: mavutil.mavlink_connection, force: bool = False) -> None:
    """
    DISARM двигателей той же командой
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # disarm
        0 if not force else 21196,
        0, 0, 0, 0, 0
    )

def takeoff(master: mavutil.mavlink_connection, alt_m: float) -> None:
    """
    Взлёт до alt_m с помощью MAV_CMD_NAV_TAKEOFF
    Предполагается, что Copter уже в режиме GUIDED и ARM
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,  # params 1-4 не используются Copter для обычного взлёта
        0,  # param5: lat, 0 = текущая
        0,  # param6: lon
        alt_m  # param7: высота (MAV_FRAME_GLOBAL_RELATIVE_ALT)
    )
    # Ждать фактическую высоту будет основная программа по данным DroneState

def land(master: mavutil.mavlink_connection) -> None:
    """
    Посадка: команда MAV_CMD_NAV_LAND (посадка в текущей точке)
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,  #
        0,
        0, 0, 0, 0,  # параметры для Copter, 0 = посадка в текущей точке
        0, 0, 0
    )
    # Фактическое «приземлился и дизармился» также может проверяться через DroneState