def get_road_z(self) -> float:
    self.host_vehicle.base.position
    ego_lane = self.lanes[self._get_ego_lane_id()]
    piece_id, t = find_lane_piece_for_coord(ego_lane , self.host_vehicle.base.position, return_progress = True)
    ego_centerline = ego_lane.classification.centerline
    return t*ego_centerline[piece_id].z + (1-t)*ego_centerline[piece_id + 1].z
    
def find_lane_piece_for_coord(lane: Lane, coordinate: Vector3d, return_progress: bool = False) -> int:
    id_closest_lane_piece = -1
    distance_closest_lane_piece = float("Inf")
    centerline = lane.classification.centerline
    for i, xyz in enumerate(centerline):
        cur_distance = euclidean_distance(coordinate, xyz)
        if cur_distance < distance_closest_lane_piece:
            distance_closest_lane_piece = cur_distance
            id_closest_lane_piece = i
    if id_closest_lane_piece == 0:
        a = id_closest_lane_piece
        b = id_closest_lane_piece + 1
    elif id_closest_lane_piece == len(centerline) -1:
        a = id_closest_lane_piece -1
        b = id_closest_lane_piece
    elif (euclidean_distance(centerline[id_closest_lane_piece-1], centerline[id_closest_lane_piece]) -
            euclidean_distance(centerline[id_closest_lane_piece-1], coordinate)
            >
            euclidean_distance(centerline[id_closest_lane_piece+1], centerline[id_closest_lane_piece])
            - euclidean_distance(centerline[id_closest_lane_piece+1], coordinate)):
        a = id_closest_lane_piece -1
        b = id_closest_lane_piece
    else:
        a = id_closest_lane_piece
        b = id_closest_lane_piece +1
    if return_progress:
        return a, calculate_piece_progress(coordinate, centerline[a], centerline[b])
    return a

def calculate_piece_progress(point:Vector3d, start: Vector3d, end: Vector3d) -> float:
    #https://stackoverflow.com/questions/61341712/calculate-projected-point-location-x-y-on-given-line-startx-y-endx-y

    l2 = euclidean_distance(start, end, ignore_z = False) ** 2
    if l2 == 0:
      raise Exception('a and b are the same points')
      
    t = ((point.x - start.x)*(end.x - start.x)  + (point.y - start.y)*(end.y - start.y)  + (point.z - start.z)*(end.z - start.z)) / l2
    t = max(0, min(1, t))
    
    #projection = Vector3d(x=a.x + t*(b.x -a.x), y=a.y + t*(b.y -a.y), z=a.z + t*(b.z -a.z))
    return t 

