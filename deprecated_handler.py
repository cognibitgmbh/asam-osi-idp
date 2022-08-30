from osi3.osi_object_pb2 import MovingObject
from typing import Optional

use_deprecated_assigned_lane = True

def get_assigned_lane_id(moving_obj: MovingObject) -> Optional[int]:
    # TODO: What happens with additional assigned lanes?
    assigned_lane_id = moving_obj.moving_object_classification.assigned_lane_id
    if use_deprecated_assigned_lane:
        assigned_lane_id = moving_obj.assigned_lane_id
    if len(assigned_lane_id) == 0:
        return None
    return assigned_lane_id[0].value

def get_all_assigned_lane_ids(moving_obj: MovingObject) -> list[int]:
    assigned_lane_id = moving_obj.moving_object_classification.assigned_lane_id
    if use_deprecated_assigned_lane:
        assigned_lane_id = moving_obj.assigned_lane_id
    return [lane.value for lane in assigned_lane_id]

