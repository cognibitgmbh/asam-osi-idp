from osi3.osi_groundtruth_pb2 import GroundTruth
from lanegraph import LaneGraph
from road import RoadManager

from state import MovingObjectState, State, StationaryObstacle


def create_state(ground_truth: GroundTruth, lane_graph: LaneGraph, road_manager: RoadManager):
    m_o_list = _create_moving_object_states(ground_truth, lane_graph, road_manager)
    s_o_list = _create_static_obstacle_states(ground_truth)
    return State(m_o_list, s_o_list, ground_truth.host_vehicle_id.value)

def _create_moving_object_states(
    ground_truth: GroundTruth,
    lane_graph: LaneGraph,
    road_manager: RoadManager,
):
    return [MovingObjectState(o, lane_graph, road_manager)
            for o in ground_truth.moving_object]

def _create_static_obstacle_states(ground_truth: GroundTruth):
    # TODO: Create Testcase for statonary obstacles
    return [StationaryObstacle(o.base.dimension, o.base.position) for o in ground_truth.stationary_object]
