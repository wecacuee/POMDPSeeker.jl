module TestPOMDPSeeker

import POMDPSeeker: SourceSeeker, predict_nextmap, runSimulation
using Test
using POMDPSimulators
using POMDPPolicies

@test SourceSeeker() != Nothing

@test size(predict_nextmap(rand(Float64, 256, 256), rand(Bool, 256, 256))) == (256, 256, 3)

#@test runSimulation()

end # module
