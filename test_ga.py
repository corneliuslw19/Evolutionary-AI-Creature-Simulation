# If you on a Windows machine with any Python version 
# or an M1 mac with any Python version
# or an Intel Mac with Python > 3.7
# the multi-threaded version does not work
# so instead, you can use this version. 

import unittest
import population
import simulation 
import genome 
import creature 
import numpy as np
import pybullet as p
import cw_envt

class TestGA(unittest.TestCase):

    def testBasicGA(self):
        pop = population.Population(pop_size=10, gene_count=3)
        sim = simulation.Simulation()
        p.resetSimulation()  # clear any previous simulation
        cw_envt.build_environment()  # creates arena + mountain

        
        for iteration in range(300):
            for cr in pop.creatures:
                sim.run_creature(cr, 2400)            
            #sim.eval_population(pop, 2400)

            #fitness function (climbing mountain)
            fits = [cr.get_climbing_fitness()
                    for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) 
                    for cr in pop.creatures]
            
            #print display
            print(iteration, "best height:", np.round(np.max(fits), 3),"mean height:", np.round(np.mean(fits), 3))    
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
            for i in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=0.25)
                dna = genome.Genome.grow_mutate(dna, rate=0.1)
                cr = creature.Creature(gene_count=len(p1.dna))
                cr.update_dna(dna)
                new_creatures.append(cr)
                
           
            max_fit = np.max(fits)
            for cr in pop.creatures:

                # elitism (saves every iteration)
                if cr.get_climbing_fitness() == max_fit:
                    new_cr = creature.Creature(1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr

                    filename = "elite_" + str(iteration) + ".csv"
                    genome.Genome.to_csv(cr.dna, filename)
                    break

            pop.creatures = new_creatures
                            
            self.assertNotEqual(fits[0], 0)

unittest.main()
