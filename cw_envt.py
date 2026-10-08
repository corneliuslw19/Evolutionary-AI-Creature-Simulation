import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import math
import creature

# -----------------------------
# Terrain generation functions
# -----------------------------



def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    def gaussian(x, y, sigma=arena_size / 4):
        return mountain_height * math.exp(-((x ** 2 + y ** 2) / (2 * sigma ** 2)))

    for _ in range(num_rocks):
        x = random.uniform(-arena_size / 2, arena_size / 2)
        y = random.uniform(-arena_size / 2, arena_size / 2)
        z = gaussian(x, y)

        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        orientation = p.getQuaternionFromEuler([
            random.uniform(0, 3.14),
            random.uniform(0, 3.14),
            random.uniform(0, 3.14)
        ])

        rock_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size]
        )

        rock_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size],
            rgbaColor=[0.5, 0.5, 0.5, 1]
        )

        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=rock_shape,
            baseVisualShapeIndex=rock_visual,
            basePosition=[x, y, z],
            baseOrientation=orientation
        )


def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    for _ in range(num_rocks):
        x = random.uniform(-arena_size / 2, arena_size / 2)
        y = random.uniform(-arena_size / 2, arena_size / 2)
        z = 0.5

        size = random.uniform(0.1, max_size)

        orientation = p.getQuaternionFromEuler([
            random.uniform(0, 3.14),
            random.uniform(0, 3.14),
            random.uniform(0, 3.14)
        ])

        rock_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size]
        )

        rock_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size],
            rgbaColor=[0.5, 0.5, 0.5, 1]
        )

        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=rock_shape,
            baseVisualShapeIndex=rock_visual,
            basePosition=[x, y, z],
            baseOrientation=orientation
        )


def make_arena(arena_size=10, wall_height=1):
    wall_thickness = 0.5

    floor_collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[arena_size / 2, arena_size / 2, wall_thickness]
    )

    floor_visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[arena_size / 2, arena_size / 2, wall_thickness],
        rgbaColor=[1, 1, 0, 1]
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=floor_collision_shape,
        baseVisualShapeIndex=floor_visual_shape,
        basePosition=[0, 0, -wall_thickness]
    )

    wall_collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2]
    )

    wall_visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2],
        rgbaColor=[0.7, 0.7, 0.7, 1]
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[0, arena_size / 2, wall_height / 2]
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[0, -arena_size / 2, wall_height / 2]
    )

    wall_collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2]
    )

    wall_visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2],
        rgbaColor=[0.7, 0.7, 0.7, 1]
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[arena_size / 2, 0, wall_height / 2]
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[-arena_size / 2, 0, wall_height / 2]
    )


## method to combine all the methods together to create complete environment
## can be called in other files to directly display environment
def build_environment():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    #creates the barricade around the mountain, adjusting this adjusts the size
    arena_size = 30 
    make_arena(arena_size=arena_size)

    #puts the mountain slight under the floor plane because its slightly block-ish at the bottom with no slope
    mountain_position = (0, 0, -1)
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))

    p.setAdditionalSearchPath("shapes/")
    p.loadURDF(
        "gaussian_pyramid.urdf",
        mountain_position,
        mountain_orientation,
        useFixedBase=1
    )


# -----------------------------
# Standalone execution (unchanged behavior)
# -----------------------------
# to test if environment displays correctly
def main():
    p.connect(p.GUI)
    build_environment()

    # generate a random creature
    cr = creature.Creature(gene_count=3)

    # save it to URDF
    with open("test.urdf", "w") as f:
        f.write(cr.to_xml())

    # load creature
    p.loadURDF("test.urdf", (0, 0, 10))

    p.setRealTimeSimulation(1)

    while True:
        time.sleep(0.01)


if __name__ == "__main__":
    main()
