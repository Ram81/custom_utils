import numpy as np

from habitat.utils.visualizations import maps


def is_on_same_floor(height, ref_floor_height, ceiling_height=0.5):
    return (
        (ref_floor_height - ceiling_height)
        <= height
        < (ref_floor_height + ceiling_height)
    )


def draw_point(sim, top_down_map, position, point_type, point_padding=2):
    t_x, t_y = maps.to_grid(
        position[2],
        position[0],
        (top_down_map.shape[0], top_down_map.shape[1]),
        sim=sim,
    )
    top_down_map[
        t_x - point_padding : t_x + point_padding + 1,
        t_y - point_padding : t_y + point_padding + 1,
    ] = point_type
    return top_down_map


def draw_bounding_box(
    sim, top_down_map, goal_object_id, ref_floor_height, line_thickness=4
):
    sem_scene = sim.semantic_annotations()
    object_id = goal_object_id

    sem_obj = None
    for object in sem_scene.objects:
        if object.id == object_id:
            sem_obj = object
            break

    center = sem_obj.aabb.center
    x_len, _, z_len = sem_obj.aabb.sizes / 2.0
    # Nodes to draw rectangle
    corners = [
        center + np.array([x, 0, z])
        for x, z in [
            (-x_len, -z_len),
            (-x_len, z_len),
            (x_len, z_len),
            (x_len, -z_len),
            (-x_len, -z_len),
        ]
        if is_on_same_floor(center[1], ref_floor_height=ref_floor_height)
    ]

    map_corners = [
        maps.to_grid(
            p[2],
            p[0],
            (
                top_down_map.shape[0],
                top_down_map.shape[1],
            ),
            sim=sim,
        )
        for p in corners
    ]

    maps.draw_path(
        top_down_map,
        map_corners,
        maps.MAP_TARGET_BOUNDING_BOX,
        line_thickness,
    )
    return top_down_map
