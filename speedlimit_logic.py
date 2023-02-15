import sys
from typing import Optional

from lane import LaneSubtype
from road import RoadSignal
import osi3.osi_lane_pb2 as lane_pb2
import osi3.osi_trafficsign_pb2 as sign_pb2

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


def calculate_speedlimit(signals: list[RoadSignal], mo_road_s: tuple[float, float], ignore_exit_speed_signs: bool) \
        -> Optional[int]:
    # max integer
    latest_speedlimit: int = sys.maxsize
    latest_road_s = float("-inf")
    for current_signal in signals:
        sign_classification = current_signal.osi_signal.main_sign.classification
        if sign_classification.type not in SPEED_SIGNS:
            continue
        if ignore_exit_speed_signs and current_signal.closest_lane.data.lane_subtype == LaneSubtype.EXIT:
            continue
        if latest_road_s < current_signal.road_s[0] <= mo_road_s[0]:
            latest_road_s = current_signal.road_s[0]
            if sign_classification in (TYPE_SPEED_LIMIT_END, TYPE_SPEED_LIMIT_ZONE_END):
                latest_speedlimit = sys.maxsize
            else:
                latest_speedlimit = int(sign_classification.value.value)
    return latest_speedlimit if latest_speedlimit < sys.maxsize else None
