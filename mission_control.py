# mission_control.py
from dataclasses import dataclass
from typing import List
from pymavlink import mavutil


@dataclass
class MissionItem:
    """
    Один пункт миссии в формате MISSION_ITEM_INT
    seq - порядковый номер пункта (0..N-1)
    frame - система координат (например, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    command - команда MAV_CMD_NAV_... или другая
    current - 1 для текущей точки (обычно только у первой)
    autocontinue - 1, если автоматически переходить к следующей точке
    param1..4 - параметры команды (зависят от типа команды)
    x, y - широта/долгота * 1e7
    z - высота в метрах
    """
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int  # lat * 1e7
    y: int  # lon * 1e7
    z: float  # alt (м)


def clear_mission(master: mavutil.mavlink_connection) -> None:
    """
    Очистка миссии командой MISSION_CLEAR_ALL
    """
    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component
    )


def upload_mission(master: mavutil.mavlink_connection, items: List[MissionItem]) -> None:
    """
    Загрузка миссии по протоколу Mission Protocol:
    1) MISSION_COUNT
    2) цикл: MISSION_REQUEST_INT -> MISSION_ITEM_INT
    3) ожидание MISSION_ACK. [web:24]
    """
    count = len(items)
    if count == 0:
        return
    
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count
    )
    
    sent = 0
    while sent < count:
        msg = master.recv_match(
            type=['MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True,
            timeout=5
        )
        
        if msg is None:
            # В учебном коде просто ждём дальше; в реальном нужно делать повторы. [web:24]
            continue
        
        if msg.get_type() == 'MISSION_REQUEST_INT':
            seq = msg.seq
            # ВАЖНО: проверяем, что seq в диапазоне [0, count-1].
            if seq < 0 or seq >= count:
                print(f"Получен запрос миссии с некорректным seq={seq}, ожидаем 0..{count-1}")
                # Можно либо пропустить, либо послать MISSION_ACK с ошибкой. [web:24][web:29]
                continue
            
            item = items[seq]
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                item.x,
                item.y,
                item.z,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION  # [web:24][web:29]
            )
            sent += 1
        elif msg.get_type() == 'MISSION_ACK':
            # В реальном коде имеет смысл проверить msg.type == MAV_MISSION_ACCEPTED. [web:24][web:29]
            print("MISSION_ACK получен, загрузка миссии завершена.")
            break


def download_mission(master: mavutil.mavlink_connection) -> List[MissionItem]:
    """
    Чтение миссии по протоколу Mission Protocol:
    1) MISSION_REQUEST_LIST
    2) получение MISSION_COUNT
    3) цикл: MISSION_REQUEST_INT -> MISSION_ITEM_INT
    """
    master.mav.mission_request_list_send(
        master.target_system,
        master.target_component
    )
    
    msg = master.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=5)
    if msg is None:
        return []
    
    count = msg.count
    items: List[MissionItem] = []
    
    for seq in range(count):
        master.mav.mission_request_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        
        item_msg = master.recv_match(type=['MISSION_ITEM_INT'], blocking=True, timeout=5)
        if item_msg is None:
            continue
        
        items.append(
            MissionItem(
                seq=item_msg.seq,
                frame=item_msg.frame,
                command=item_msg.command,
                current=item_msg.current,
                autocontinue=item_msg.autocontinue,
                param1=item_msg.param1,
                param2=item_msg.param2,
                param3=item_msg.param3,
                param4=item_msg.param4,
                x=item_msg.x,
                y=item_msg.y,
                z=item_msg.z,
            )
        )
    
    return items