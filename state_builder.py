from osi3.osi_object_pb2 import MovingObject
from osi3.osi_groundtruth_pb2 import GroundTruth

from state import MovingObjectState, State, RoadState, StationaryObstacle
from deprecated_handler import get_assigned_lane_id, get_all_assigned_lane_ids


def create_state(ground_truth: GroundTruth):
    m_o_list = _create_moving_object_states(ground_truth)
    s_o_list = _create_static_obstacle_states(ground_truth)
    return State(m_o_list, s_o_list, ground_truth.host_vehicle_id)

def _create_moving_object_states(ground_truth: GroundTruth):
    return [_convert_to_moving_object_state(o) for o in ground_truth.moving_object]

def _create_static_obstacle_states(ground_truth: GroundTruth):
    # TODO: Create Testcase for statonary obstacles
    return [StationaryObstacle(o.base.dimension, o.base.position) for o in ground_truth.stationary_object]


def _convert_to_moving_object_state(m_o: MovingObject): 
    m_o_s = MovingObjectState()
    m_o_s.simulator_id = m_o.id.value
    m_o_s.object_type = m_o.type
    m_o_s.dimensions = m_o.base.dimension
    m_o_s.location = m_o.base.position
    m_o_s.velocity = m_o.base.velocity
    m_o_s.acceleration = m_o.base.acceleration
    m_o_s.yaw_angle = m_o.base.orientation.yaw
    m_o_s.pitch_angle = m_o.base.orientation.pitch
    m_o_s.roll_angle = m_o.base.orientation.roll
    m_o_s.lane_ids = get_all_assigned_lane_ids(m_o)
    light_state = m_o.vehicle_classification.light_state
    m_o_s.indicator_signal = light_state.indicator_state
    m_o_s.brake_light = light_state.brake_light_state
    m_o_s.front_fog_light = light_state.front_fog_light
    m_o_s.rear_fog_light = light_state.rear_fog_light
    m_o_s.head_light = light_state.head_light
    m_o_s.high_beam = light_state.high_beam
    m_o_s.reversing_light = light_state.reversing_light
    m_o_s.license_plate_illumination_rear = light_state.license_plate_illumination_rear
    m_o_s.emergency_vehicle_illumination = light_state.emergency_vehicle_illumination
    m_o_s.service_vehicle_illumination = light_state.service_vehicle_illumination

    #TODO: Replace 'None' with actual values
    m_o_s.heading_angle = None
    m_o_s.lane_position = None
    m_o_s.road_id = None
    m_o_s.road_s = None
    return m_o_s


