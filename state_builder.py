from osi3.osi_groundtruth_pb2 import GroundTruth
from lane import LaneData

from state import MovingObjectState, State, RoadState, StationaryObstacle


def create_state(ground_truth: GroundTruth, lane_data: dict[int, LaneData]):
    m_o_list = _create_moving_object_states(ground_truth, lane_data)
    s_o_list = _create_static_obstacle_states(ground_truth)
    return State(m_o_list, s_o_list, ground_truth.host_vehicle_id)

def _create_moving_object_states(
    ground_truth: GroundTruth,
    lane_data: dict[int, LaneData],
):
    return [MovingObjectState(o, lane_data)
            for o in ground_truth.moving_object]

def _create_static_obstacle_states(ground_truth: GroundTruth):
    # TODO: Create Testcase for statonary obstacles
    return [StationaryObstacle(o.base.dimension, o.base.position) for o in ground_truth.stationary_object]
