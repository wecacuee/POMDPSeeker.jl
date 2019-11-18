#!/usr/bin/env julia
using RobotOS
using POMDPSeeker
using PyCall

using POMDPSimulators
using POMDPPolicies

@rosimport nav_msgs.msg: OccupancyGrid
rostypegen()
using .nav_msgs.msg

rospy = pyimport("rospy")
nav_msg = pyimport("nav_msgs.msg")

mutable struct Lastmsg
    msg::Union{OccupancyGrid,Nothing}
end

function callback(msg::OccupancyGrid, lm::Lastmsg)
    @show msg
    lm.msg = msg
end

function runSimulation()
    m = SourceSeeker(),
    solver = POMCPOWSolver(criterion=MaxUCB(20.0))
    policy = solve(solver, m)
    for (s, a, r) in stepthrough(m, policy, "s,a,r", max_steps=10)
        @show s
        @show a
        @show r
        println()
    end
    return true
end

function main()
    init_node("rosjl_pomdp")

    runSimulation()
    @show occgrid
end


if ! isinteractive()
    main()
end
