module TestPOMDPSeeker

import POMDPSeeker: SourceSeeker
using Test
using POMDPSimulators
using POMDPPolicies

@test SourceSeeker() != Nothing

@test runSimulation()

end # module
