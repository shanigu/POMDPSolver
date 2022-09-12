# POMDPSolver
This package supports a set of POMDP solution algorithms. 
You can run the executable with 2 parameters: model name, method name.

The package supports loading models in the Cassandra format (.pomdp files). 
Alternatively, you can implement your model as a Java class. The package contains a few such examples: Network, Logistics.

The package currently supports the following solvers:
FSVI, PBVI, HSVI, VPI, Perseus, PVI, PPBVI, PPReseus, RTDP, RTBSS, FRG, FSG, VRG, IP.
