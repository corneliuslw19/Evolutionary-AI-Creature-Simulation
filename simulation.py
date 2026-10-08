import pybullet as p
from multiprocessing import Pool
import cw_envt
import numpy as np

class Simulation: 
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        #self.physicsClientId = p.connect(p.GUI)
        self.sim_id = sim_id
        ## connects to direct to speed up process as GUI not needed
        ## change to connect GUI if need visual representation


def run_creature(self, cr, iterations=2400):  # 2400 = 10 sec at 240 fps
    pid = self.physicsClientId

    p.resetSimulation(physicsClientId=pid)
    p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)
    cw_envt.build_environment()

    xml_file = 'temp' + str(self.sim_id) + '.urdf'
    xml_str = cr.to_xml()
    with open(xml_file, 'w') as f:
        f.write(xml_str)

    cid = p.loadURDF(xml_file, physicsClientId=pid)

    p.resetBasePositionAndOrientation(
        cid, [0, -7, 1.5], [0, 0, 0, 1], physicsClientId=pid
    )

    for step in range(iterations):
        p.stepSimulation(physicsClientId=pid)

        if step % 24 == 0:
            self.update_motors(cid=cid, cr=cr)

        pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
        cr.update_position(pos)

    
    def update_motors(self, cid, cr):
        ##cid -> Creature ID in the physics engine
        ##cr is a creature object
        
        for jid in range(p.getNumJoints(cid,
                                        physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]

            gate = cr.stimulus_facing_peak  # 0 or 1
            target = gate * (0.5 * m.get_output())

            p.setJointMotorControl2(
                cid, jid,
                controlMode=p.VELOCITY_CONTROL,
                targetPosition=target,
                force=1000,
                physicsClientId=self.physicsClientId
            )
        

    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at
        """
        for cr in pop.creatures:
            self.run_creature(cr, 2400) 


class ThreadedSim():
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):# the end
                    break
                # work out the sim ind
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                            self.sims[sim_ind], 
                            pop.creatures[i], 
                            iterations]   
                )
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                # it works on a copy of the creatures, so receive them
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                # and now put those creatures back into the main 
                # self.creatures array
                new_creatures.extend(creatures)
        pop.creatures = new_creatures
