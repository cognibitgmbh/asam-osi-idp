from lane import LaneData


def do_lanes_belong_together(lane_data: dict[int, LaneData], lane_id_a: int, lane_id_b: int):
    lane_a: LaneData = lane_data[lane_id_a]
    lane_b: LaneData = lane_data[lane_id_b]
    if lane_a.left_neighbor_ids is not None and lane_id_b in lane_a.left_neighbor_ids or \
       lane_a.right_neighbor_ids is not None and lane_id_b in lane_a.right_neighbor_ids or \
       lane_b.left_neighbor_ids is not None and lane_id_a in lane_b.left_neighbor_ids or \
       lane_b.right_neighbor_ids is not None and lane_id_a in lane_b.right_neighbor_ids:
       return True
    for pairing in lane_a.osi_lane.classification.lane_pairing:
        if pairing.antecessor_lane_id.value == lane_id_b or pairing.successor_lane_id.value == lane_id_b:
            return True

    for pairing in lane_b.osi_lane.classification.lane_pairing:
        if pairing.antecessor_lane_id.value == lane_id_a or pairing.successor_lane_id.value == lane_id_a:
            return True




def assign_roadids_to_lanes(lane_data: dict[int, LaneData]):
    next_unused_road_id = 0
    road_to_lane_dict: dict[int, set[int]] = {}
    lane_to_road_dict: dict[int, int] = {}
    for current_lane_id in lane_data:
        if current_lane_id not in lane_to_road_dict:
            found_road = False
            for r_id, lanes_in_road in road_to_lane_dict.items():
                for lane_in_road in lanes_in_road:
                    if do_lanes_belong_together(lane_data, current_lane_id, lane_in_road):
                        road_to_lane_dict[r_id].add(current_lane_id)
                        lane_to_road_dict[current_lane_id] = r_id
                        found_road = True
                        break
                if found_road:
                    break
            if not found_road:
                road_to_lane_dict[next_unused_road_id] = set([current_lane_id])
                lane_to_road_dict[current_lane_id] = next_unused_road_id
                next_unused_road_id += 1
    print(f"end of function: {road_to_lane_dict}")
    print(f"end of function2: {lane_to_road_dict}")
                
                

