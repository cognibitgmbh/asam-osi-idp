import sys
from typing import Optional

import osi3.osi_lane_pb2 as lane_pb2
import osi3.osi_trafficsign_pb2 as sign_pb2

from .lane import LaneSubtype
from .road import Road

SUBTYPE_ENTRY = lane_pb2._LANE_CLASSIFICATION_SUBTYPE.values_by_name["SUBTYPE_ENTRY"].number

TRAFFIC_SIGN_TYPE_BY_NAME = sign_pb2._TRAFFICSIGN_MAINSIGN_CLASSIFICATION_TYPE.values_by_name
TYPE_SPEED_LIMIT_BEGIN = TRAFFIC_SIGN_TYPE_BY_NAME["TYPE_SPEED_LIMIT_BEGIN"].number
TYPE_SPEED_LIMIT_END = TRAFFIC_SIGN_TYPE_BY_NAME["TYPE_SPEED_LIMIT_END"].number
TYPE_SPEED_LIMIT_ZONE_BEGIN = TRAFFIC_SIGN_TYPE_BY_NAME["TYPE_SPEED_LIMIT_ZONE_BEGIN"].number
TYPE_SPEED_LIMIT_ZONE_END = TRAFFIC_SIGN_TYPE_BY_NAME["TYPE_SPEED_LIMIT_ZONE_END"].number
SPEED_SIGNS = {TYPE_SPEED_LIMIT_BEGIN,
               TYPE_SPEED_LIMIT_END,
               TYPE_SPEED_LIMIT_ZONE_BEGIN,
               TYPE_SPEED_LIMIT_ZONE_END}


def calculate_speedlimit(road: Road, mo_road_s: tuple[float, float], ignore_exit_speed_signs: bool) \
        -> Optional[int]:
    # max integer
    latest_speedlimit: int = sys.maxsize
    latest_road_s = float("-inf")
    for current_signal in road.signals:
        sign_classification = current_signal.osi_signal.main_sign.classification
        if sign_classification.type not in SPEED_SIGNS:
            continue
        if latest_road_s < current_signal.road_s[0] <= mo_road_s[0]:
            if current_signal.closest_lane.data.lane_subtype == LaneSubtype.EXIT:
                if ignore_exit_speed_signs:
                    continue
                else:
                    s_of_exit_end = road.calculate_s_of_exit_end(current_signal.closest_lane)
                    if s_of_exit_end is None:
                        continue
                    if s_of_exit_end < mo_road_s[0]:
                        continue
            latest_road_s = current_signal.road_s[0]
            if sign_classification in (TYPE_SPEED_LIMIT_END, TYPE_SPEED_LIMIT_ZONE_END):
                latest_speedlimit = sys.maxsize
            else:
                latest_speedlimit = int(sign_classification.value.value)
    return latest_speedlimit if latest_speedlimit < sys.maxsize else None
