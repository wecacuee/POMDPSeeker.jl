module TestPOMDPSeeker

import POMDPSeeker: SourceSeeker
using Test
using POMDPSimulators
using POMDPPolicies

@test SourceSeeker() != Nothing


function runSimulation()
    m = SourceSeeker()

    # policy that takes a random action
    policy = RandomPolicy(m)

    for (s, a, r) in stepthrough(m, policy, "s,a,r", max_steps=10)
        @show s
        @show a
        @show r
        println()
    end
    return true
end

@test runSimulation()

end # module
