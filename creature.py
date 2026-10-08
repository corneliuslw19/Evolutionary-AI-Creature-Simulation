import genome 
from xml.dom.minidom import getDOMImplementation ##allows use of XML URDF 
from enum import Enum ##assign numeric values to constant values , eg. Square = 1, Circle = 2
import numpy as np
import pybullet as p

def mountain_height_fn(x, y):
    ray_start = [x, y, 50]
    ray_end   = [x, y, -10]
    hit = p.rayTest(ray_start, ray_end)[0]
    if hit[0] != -1:
        return hit[3][2]
    return 0.0

##applying use of Enum
class MotorType(Enum):
    PULSE = 1
    SINE = 2

class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        if control_waveform <= 0.5:
            self.motor_type = MotorType.PULSE
        else:
            self.motor_type = MotorType.SINE
        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0
    

    ##generates the output of Motor according to the type (PULSE or SINE)
    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)
        if self.motor_type == MotorType.PULSE:
            if self.phase < np.pi:
                output = 1
            else:
                output = -1
            
        if self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)

        return output 

class Creature:
    def __init__(self, gene_count):
        self.spec = genome.Genome.get_gene_spec()
        self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

    ##converting the genome dict into flat links and assign it to self.flat_links
    def get_flat_links(self):
        if self.flat_links == None:
            gdicts = genome.Genome.get_genome_dicts(self.dna, self.spec)
            self.flat_links = genome.Genome.genome_to_links(gdicts)
        return self.flat_links
    
    ##converts self.flat_links to proper format and assign to self.exp_links
    def get_expanded_links(self):
        self.get_flat_links()
        if self.exp_links is not None:
            return self.exp_links
        
        exp_links = [self.flat_links[0]]
        genome.Genome.expandLinks(self.flat_links[0], 
                                self.flat_links[0].name, 
                                self.flat_links, 
                                exp_links)
        self.exp_links = exp_links
        return self.exp_links

    ##the values from flat_links have been correctly converted to exp_links
    ##loops through every link to add a <link> using link.to_link_element(adom), and then for joint
    ##with the new correct format, now convert to XML
    def to_xml(self):
        self.get_expanded_links()

        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)

        robot_tag = adom.createElement("robot")

        # Add link elements
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))

        # Add joint elements (skip root)
        first = True
        for link in self.exp_links:
            if first:  # skip the root node
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))

        # Set robot name
        robot_tag.setAttribute("name", "pepe")

        # --- DEBUG: print link name and parent for every link ---
        #print("=== Link hierarchy ===")
        #for link in self.exp_links:
        #    print(f"Link name: {link.name}, Parent name: {link.parent_name}")
        #print("======================")

        # Return XML string
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()


    def get_motors(self):
        self.get_expanded_links()
        if self.motors == None:
            motors = []
            for i in range(1, len(self.exp_links)):
                l = self.exp_links[i]
                m = Motor(l.control_waveform, l.control_amp,  l.control_freq)
                motors.append(m)
            self.motors = motors 
        return self.motors 
    
    def update_position(self, pos):
        if self.start_position == None:
            self.start_position = pos
        else:
            self.last_position = pos

    #fitness function used distance travelled to determine elites
    def get_distance_travelled(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray(self.start_position)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1-p2)
        return dist 
    
    # fitness
    # takes in parameters to ensure moving towards mountain
    def get_climbing_fitness(self):
        if self.start_position is None or self.last_position is None:
            return 1e-6

        p0 = np.asarray(self.start_position)
        p1 = np.asarray(self.last_position)

        forward_progress = p1[1] - p0[1]  # +Y only
        forward_progress = max(forward_progress, 0)

        terrain_z = mountain_height_fn(p1[0], p1[1])
        dz_relative = p1[2] - terrain_z

        fitness = 0.6 * forward_progress + 0.4 * dz_relative

        return max(fitness, 1e-6)


    def update_dna(self, dna):
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
