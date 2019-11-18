#!/usr/bin/env julia
using POMDPSeeker
using RobotOS


function main()
    init_node("rosjl_pomdp")

    runSimulation()
end


if ! isinteractive()
    main()
end
