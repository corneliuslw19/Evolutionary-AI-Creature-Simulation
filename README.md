Evolutionary AI Creature Simulation
A Python-based evolutionary artificial intelligence simulation that uses genetic algorithms and PyBullet physics to evolve 
virtual creatures towards climbing a mountain.

Project Overview
This project explores evolutionary computation, genetic algorithms and physics-based simulation. Virtual creatures are evaluated 
according to their movement and climbing performance, with stronger-performing individuals selected across successive generations.
The project was developed for the CM3020 Artificial Intelligence module at the University of London, building upon provided starter code.

Features
Genetic algorithm with elitism
Procedurally generated virtual creatures
PyBullet physics-based simulation
Mountain environment
Fitness evaluation based on movement and height
Directional sensory input
Experiments with motor control, population size and simulation duration

Technologies
Python
PyBullet
NumPy
Genetic Algorithms
Evolutionary Computation

Installation
Install Python and the required dependencies:
pip install -r requirements.txt

Running the Simulation
From the project directory, run:
python run_genome.py

This loads a saved creature genome and opens the PyBullet simulation.
The script currently uses batch6_elite_297.csv as its demonstration genome.

Experimental Findings
The project explored different fitness functions, motor-control approaches, population sizes and simulation durations.
Adjustments to the fitness function improved movement towards the mountain, while changes to motor control and sensory 
input helped guide creature behaviour.
One experiment achieved a maximum recorded height of 5.95 and mean height of 2.1.

Limitations
The evolved creatures did not consistently climb the mountain successfully. Terrain interaction, friction and 
fitness-function design remained important limitations.
Future improvements could explore more realistic terrain interaction and fitness functions that prioritise consistent 
climbing performance.

Academic Context
Developed as part of the University of London CM3020 Artificial Intelligence coursework, 
using provided starter code with additional implementation and experimentation.
