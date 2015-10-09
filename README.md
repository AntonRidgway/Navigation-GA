# Navigation-GA
C++ implementation of a genetic algorithm to solve a robot pathfinding problem, using an experimental chromosome structure. Coded in Spring 2014 for an Independent Study under Dr. Roger Wainwright at the University of Tulsa.

In this problem, a robot seeks to find a path from a starting position to some goal. We assume that the robot is roughly aware of the obstacles in its way, and structure our tests such that it must always travel from the upper-left corner of a map to the lower-right.
This project was based on a master's thesis written in 2004 by Kamran Sedighi, "Local Path-planning of an Autonomous Mobile Robot Using a Genetic Algorithm." In that work, Sedighi used a chromosome that detailed a path in either column-wise or row-wise coordinates, and provided two opportunities to switch between coordinate-systems.
The focus of my work was on performing experimentation with a new chromosome structure for the genetic algorithm, which allowed switching between coordinate systems at any point. We found that this made the pathfinding much more flexible overall.
Finally, I determined that the worst-case scenario which should still be workable for this chromosome structure was a map which required the maximum amount of travel away from the straight line between the starting position and the goal. Some simple test cases can also thwart the algorithm by requiring too much backtracking, as mentioned in the report.

When run, the genetic algorithm allows the user to set:
 - The initial population size
 - The number of generations to run before termination
 - The mutation rate of chromosomes
 - The crossover rate of chromosomes

It also provides the following operators to choose from:
 - **Roulette Selection** - Use the fractional fitness of each chromosome to make a biased random selection for crossover.
 - **Rank Selection** - Rank the fitness of each chromosome, and make a biased random selection based on the resulting ordering.
 - **Tournament Selection** - Compare the fitness of two chromosomes randomly, without bias. Biasedly allow the more fit of the two to be selected K percent of the time.
 - **N-Slice Crossover** - Generate N random, unique integers (between 0 and P-2, and N < chromosome length). Take each integer to represent a random "slice" point at which to swap alternating segments of the two parents.
 - **Local N-Slice Crossover** - The same as regular N-Slice, except that all slide points are chosen before the first collision point in the chromosome's path. The intention was that this might help fix issues with chromosomes expending too many coordinates from their path early on.
 - **Uniform Crossover** - Generate uniform-random bit strings which determine which of the two parent chromosomes each bit of the child chromosome is to be taken from.
 - **N-Point Mutation** - Invert N uniform random points in the chromosome.
 - **Limited N-Point Mutation** - The same as regular N-Point, except that all mutation points are chosen before the first collision point, as before.
 - **Gaussian N-Point Mutation** - Invert N random points, selected from a Gaussian distribution with a mean around the first collision point in the chromosome (standard deviation is one-tenth of the chromosome length).

The simulated annealing allows the testing parameters to be set at the start of each run. It allows the user to choose between an N-Point Perturbation, corresponding to an N-Point Mutation for the GA, and the Limited and Gaussian variations as described above. Note that for the purposes of these tests, the other annealing parameters were hard-coded.

Files included in this repo are:

**main**: Initial program I/O, and the main GA loop, with all GA operators. Also incorporates a simulated-annealing algorithm as a sanity check.

**Trail**: Represents a single path, as expressed by any given chromosome.

**Chromosome**: Interface for two chromosome types experimented with.

**TurnPointChrom**: Chromosome developed for this work.

**CoordinateChrom**: Sanity-check chromosome which simply specifies a fixed-length list X,Y coordinate pairs.

**GANavigation**: A Windows executable that allows users to specify parameters to the algorithm and run it on one of the supplied datasets. Test map file names should be specified relative to the executable's directory.

**results**: Contains some sample results for the worst-case testing that I focused my work on, and a report for the conclusion of the study.

**tests**: Contains some test "map" files, in a straightforward ASCII representation, to run the algorithm on. Note that all maps are square, with a size encoded in the initial line of each file.
 - Files prefixed **corners** are maps that exhibit worst-case traits for the chromosome structure we used (i.e., they require a large number of deviations from the "straight line" path to the goal).
 - Files prefixed **godugu** are sample test cases taken from a work by Jagruthi Godugu, "Development of a Benchmark for Robot Path Planning." These were meant to provide baseline tests for our algorithm.
 - Files prefixed **manikas** are taken from a work by Theodore Manikas, "Genetic Algorithms for Autonomous Robot Navigation." This work provided additional baseline tests.
 - Files prefixed **SearchSpace** are taken from the dataset used by Thomas Geisler in "Autonomous Robot Navigation System using a Genetic Algorithm with a Novel Value Encoding Technique."
 - Other files were developed by me for specific testing needs.
