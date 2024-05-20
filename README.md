# Time-of-Flight-TFG

For the first iterations, a folder has been created to make the first tests to ensure we can represent figures (in this case, squared ones).
This folder has not yet start working with the algorithm (the last commit shows the mass_center calculation but is not probed or optimized), only the representations of figures. 

The Closed Convex folder has a script named representing_environment. This script can reproduce any figure as a ToF vector, not only squared environments. 

In the testing script, there are 4 vectors with "manual" values. These vectors are obtained directly from the MCU of the robot. So these values are real values, not simulated. 
Initially, the tests were done creating the ToF vectors with the representing_environment module, but once the tests were done, the final script shows the results with the practical values.

