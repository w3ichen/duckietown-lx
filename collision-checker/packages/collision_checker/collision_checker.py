import itertools
import random
from typing import List
import numpy as np
from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            environment=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(
    environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    # You can start by rototranslating the robot_body by the robot_pose
    rototranslated_robot: List[PlacedPrimitive] = []
    # == WRITE ME ==
    for part in robot_body:
        theta_rad = np.deg2rad(robot_pose.theta_deg)
        rotated_x = part.pose.x * np.cos(theta_rad) - part.pose.y * np.sin(theta_rad)
        rotated_y = part.pose.x * np.sin(theta_rad) + part.pose.y * np.cos(theta_rad)

        new_pose = FriendlyPose(x=robot_pose.x + rotated_x, y=robot_pose.y + rotated_y, theta_deg=robot_pose.theta_deg + part.pose.theta_deg)
        new_part = PlacedPrimitive(pose=new_pose, primitive=part.primitive)
        rototranslated_robot.append(new_part)

    # Then, call check_collision_list to see if the robot collides with the environment
    collided = check_collision_list(rototranslated_robot, environment)

    return collided


def check_collision_list(
    rototranslated_robot: List[PlacedPrimitive], environment: List[PlacedPrimitive]
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    for robot, envObject in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot, envObject):
            return True

    return False

def dist(a: PlacedPrimitive, b: PlacedPrimitive) -> float:
    # Return distance between the centers
    a_x = a.pose.x
    a_y = a.pose.y
    b_x = b.pose.x
    b_y = b.pose.y
    dist = np.sqrt((b_x-a_x)**2 + (b_y-a_y)**2)
    return dist


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # Calculate the distance between centers
    centers_dist = dist(a, b)

    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        # Collides if the distance between the centers is less than the sum of the radii
        radii_sum = a.primitive.radius + b.primitive.radius
        return centers_dist < radii_sum

    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        # Use upper and lower bounds for quick collision check
        min_radius = min(abs(a.primitive.xmax - a.primitive.xmin), abs(a.primitive.ymax - a.primitive.ymin)) / 2
        max_radius = max(abs(a.primitive.xmax - a.primitive.xmin), abs(a.primitive.ymax - a.primitive.ymin)) / 2
        return centers_dist < min_radius + b.primitive.radius or centers_dist > max_radius + b.primitive.radius

    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        # Calculate half-widths and half-heights for both rectangles
        a_half_width = abs(a.primitive.xmax - a.primitive.xmin) / 2
        a_half_height = abs(a.primitive.ymax - a.primitive.ymin) / 2
        b_half_width = abs(b.primitive.xmax - b.primitive.xmin) / 2
        b_half_height = abs(b.primitive.ymax - b.primitive.ymin) / 2

        # Calculate the vectors between the centers
        vx = a.pose.x - b.pose.x
        vy = a.pose.y - b.pose.y

        # Rotate the vectors according to the orientations of the rectangles
        theta_rad = np.deg2rad(a.pose.theta_deg)
        vx_rotated = vx * math.cos(math.radians(a.pose.theta_deg)) + vy * math.sin(math.radians(a.pose.theta_deg))
        vy_rotated = -vx * math.sin(math.radians(a.pose.theta_deg)) + vy * math.cos(math.radians(a.pose.theta_deg))

        # Check for overlap along each axis
        overlap_x = abs(vx_rotated) < (a_half_width + b_half_width)
        overlap_y = abs(vy_rotated) < (a_half_height + b_half_height)

        # If there is overlap along both axes, the rectangles collide
        return overlap_x and overlap_y

    return False



