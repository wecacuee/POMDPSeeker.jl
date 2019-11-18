#!/usr/bin/env julia
using RobotOS
using POMDPSeeker
using PyCall

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


function main()
    init_node("rosjl_pomdp")
    #lm = Lastmsg(nothing)
    #sub = Subscriber{OccupancyGrid}(
    #    "/robot0/gmapping_map", callback, (lm,), queue_size=10)
    msg = rospy.wait_for_message("/robot0/gmapping_map", nav_msg.OccupancyGrid)
    occgrid = convert(OccupancyGrid, msg)
    @show occgrid
end


if ! isinteractive()
    main()
end
