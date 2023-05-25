from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_object_pb2 import MovingObject

from .lanegraph import LaneGraph
from .road import RoadManager
from .state import MovingObjectState, State, StationaryObstacle


def create_state(ground_truth: GroundTruth, lane_graph: LaneGraph, road_manager: RoadManager, ego_id: int):
    m_o_list = _create_moving_object_states(ground_truth, lane_graph,
                                            road_manager, ego_id)
    s_o_list = _create_static_obstacle_states(ground_truth)
    return State(m_o_list, s_o_list, ground_truth.host_vehicle_id.value)

def _create_moving_object_states(
    ground_truth: GroundTruth,
    lane_graph: LaneGraph,
    road_manager: RoadManager,
    ego_id: int,
):
    ego_object: 'MovingObject | None' = next(
        (o for o in ground_truth.moving_object if o.id.value == ego_id), None)
    if ego_object is None:
        raise RuntimeError(
            f"Could not find ego vehicle (expected id: {ego_id})")
    ego_state = MovingObjectState(ego_object, lane_graph, road_manager, None)
    ego_road_id = ego_state.road_id
    return [ego_state] + [MovingObjectState(o, lane_graph, road_manager, ego_road_id)
                          for o in ground_truth.moving_object
                          if o.id.value != ego_id]


def _create_static_obstacle_states(ground_truth: GroundTruth):
    return [StationaryObstacle(o.base.dimension, o.base.position) for o in ground_truth.stationary_object]
