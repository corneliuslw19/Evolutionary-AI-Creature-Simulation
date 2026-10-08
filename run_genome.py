import pybullet as p
import pybullet_data as pd
import creature
import time
import genome as genlib
import cw_envt
import numpy as np

# setting up pybullet
p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# default camera view
p.resetDebugVisualizerCamera(cameraDistance=13, cameraYaw=65, cameraPitch=-13, cameraTargetPosition=[0, 0, 2])

# setting up mountain environment
p.resetSimulation()
cw_envt.build_environment()  # builds arena + mountain
p.setGravity(0, 0, -10)

# load creature
c = creature.Creature(gene_count = 5)
dna = genlib.Genome.from_csv('results/batch6_elite_297.csv')
c.update_dna(dna)


with open('test.urdf', 'w') as f:
    c.get_expanded_links()
    f.write(c.to_xml())

cid = p.loadURDF('test.urdf')

p.setRealTimeSimulation(1)
c.update_position([0,0,0])

p.resetBasePositionAndOrientation(cid,[0,-7,1.5], [0,0,0,1])

#Added: stimulus settings for additional experimenting (PART B-2)
PEAK = np.array([0.0, 0.0])   # mountain top in XY
THRESH = 0.7                 # dot-product threshold


while True:
    #Added: compute stimulus each loop
    pos, orn = p.getBasePositionAndOrientation(cid)

    p_xy = np.array([pos[0], pos[1]])
    to_peak = PEAK - p_xy
    to_peak = to_peak / (np.linalg.norm(to_peak) + 1e-9)

    roll, pitch, yaw = p.getEulerFromQuaternion(orn)

    # yaw=0 ~ facing +Y
    forward = np.array([np.sin(yaw), np.cos(yaw)])
    facing_score = float(np.dot(forward, to_peak))

    c.stimulus_facing_peak = 1.0 if facing_score > THRESH else 0.0
    gate = c.stimulus_facing_peak
    for jid in range(p.getNumJoints(cid)):
        motors = c.get_motors()
        if jid >= len(motors):
            break
        m = motors[jid]
        # Adjusted: gate motor output 
        p.setJointMotorControl2(
            cid, jid,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=gate * m.get_output(),  # <-- gated
            force=1000
        )

    time.sleep(0.1)