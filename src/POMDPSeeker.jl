__precompile__(false)
module POMDPSeeker

using Base
using Distributions: MvNormal, Uniform
using POMCPOW
using POMDPPolicies
using POMDPSimulators
using POMDPs: POMDP, POMDPs, simulate
using Parameters: @with_kw
using PyCall: pyimport
using Random: AbstractRNG, GLOBAL_RNG
using RobotOS
using Serialization: serialize
using PyCall

@rosimport nav_msgs.msg: OccupancyGrid
rostypegen(POMDPSeeker)
using .nav_msgs.msg: OccupancyGrid

rospy = pyimport("rospy")
pynavmsg = pyimport("nav_msgs.msg")

@with_kw struct Map
    grid::Array{Float64}
    res::Float64
end
Pose = Array{Float64}

function pose2pixel(
    map::Map,
    pose::Pose;
    origin_px::Array{Int64, 1} = round.(Int64, [size(map.grid)...] / 2)
)
    pixelji = round.(Int64, pose[1:2] / map.resolution)
    return [pixelji[end], pixelji[1]] + origin_px
end


"""
Class describing the state of the system
"""
@with_kw struct State
    map::Map
    pose::Pose
    tgts::Array{Float64}
    activated::Array{Bool}
end

@with_kw struct NDBox
    shape::Tuple{Int64, Vararg{Int64}}
    min::Array{Float64}
    max::Array{Float64}
end


"""
Return a random map
"""
Base.rand(rng::AbstractRNG, ms::NDBox) = rand(rng, Uniform(ms.min[1],ms.max[1]), ms.shape)


"""
Size of the map space
"""
length(ms::NDBox) = (*)(shape...)


"""
Dimensions of map space
"""
dimensions(ms::NDBox) = 2


@with_kw struct DiscreteSpace{T}
    shape::Tuple{Int64, Vararg{Int64}}
    values::Array{T}
end


Base.rand(rng::AbstractRNG, ds::DiscreteSpace{T}) where T = rand(rng, ds.values, vs.shape)


@with_kw struct StateSpace
    mapspace::DiscreteSpace{Bool}
    posespace::NDBox
    tgtspace::NDBox
    activationspace::DiscreteSpace{Bool}
end


Base.rand(rng::AbstractRNG, ss::StateSpace) = State(
    map = rand(rng, ss.mapspace),
    pose = rand(rng, ss.posespace),
    tgts = rand(rng, ss.tgtspace),
    activated = rand(rng, ss.activationspace))


"""
Class describing the possible actions
"""
ActionSpace = NDBox
Action = Array{Float64}

"""
class holding lidar observations
"""
LidarScans = Array{Float64}

"""
Observations of the system
"""
struct Observation
    lidar::LidarScans
    rssi::Float64
end


@with_kw struct BeliefDistribution
    occupancy::Array{Float64}
    pose::MvNormal
    tgts::Array{MvNormal}
    activated::Array{Bool}
end

function Base.rand(rng::AbstractRNG, bd::BeliefDistribution)
    return State(
      map=(rand(rng, Float64, size(bd.occupancy)) .> bd.occupancy),
      pose=rand(rng, bd.pose),
      tgts=transpose(hcat([rand(rng, nd) for nd in bd.tgts]...)),
      activated=bd.activated)
end

"""
Defining POMPD
"""
@with_kw struct SourceSeeker <: POMDP{State, Action, Observation}
    map_size::Tuple{Int64, Int64} = (4000, 4000)
    pose_size::Tuple{Int64} = (3,)
    tgt_size::Tuple{Int64, Int64} = (1, 2)
    state_particles::Int64 = 100
    action_particles::Int64 = 10
    max_velocity::Vector{Float64} = [0.5, 0.5, 0.5]
    min_pose::Vector{Float64} = [-100, -100, -π]
    max_pose::Vector{Float64} = [100, 100, π]
    nscans::Int64 = 180
    laser_range::Vector{Float64} = [0.05, 80]
    gmapping_map_topic::String = "/robot0/gmapping_map"
    sim_map_publish::String = "/robot0/true_map_for_sim"
    discount::Float64 = 0.9
end

"""
Return the state space for the pomdp
"""
function POMDPs.states(p::SourceSeeker)
    return StateSpace(
        mapspace = DiscreteSpace{Bool}(p.map_size, [false, true]),
        posespace = NDBox(p.pose_size, p.min_pose, p.max_pose),
        tgtspace = NDBox(p.tgt_size, p.min_pose, p.max_pose),
        activationspace = DiscreteSpace{Bool}(p.tgt_size[1], [false, true]))
end


function POMDPs.actions(p::SourceSeeker)
    return NDBox(size(p.max_velocity), -p.max_velocity, p.max_velocity)
end


function POMDPs.observations(p::SourceSeeker)
    return NDBox((p.nscans,), [p.laser_range[1]], [p.laser_range[end]])
end


"""
Should return a distribution over next states?
"""
function _transition(pomdp::SourceSeeker, s::State, a::Action)
    new_pose = s.pose + a
    pixelij = pose2pixel(s.map, new_pose)
    if s.map[pixelij] == 1
        return s
    else
        return State(s.map, new_pose, s.tgts, s.activated)
    end
end


function _observation(pomdp::SourceSeeker, a::Action, s::State)
    return rand(GLOBAL_RNG, POMDPs.observations(pomdp))
end


"""
Computes H(X) = - ∑ₓ p(x) log(p(x))

This is not entropy in the map space but in the pixel space. We need to have
samples in the map space to compute it's entropy. The random variable {0, 1}
occupancy of a pixel. Each pixel is assumed to be a sample from the Binary
probability distribution. The occupancy value in the map is the probability of pixel being occupied.

We use this as soft measure of unknown space. Equivalent to sum(map == -1)
"""
_entropy(map::Array{Float64, 2}) = -sum(map .* log.(map))


"""
Computes H(X') - H(X)
"""
function _entropy_gain(map_new::Array{Float64, 2}, map_old::Array{Float64, 2})
    return _entropy(map_old) - _entropy(map_new) 
end

"""
Convert to map
"""
function convert_occgrid(occgrid::OccupancyGrid)
    height::Int64 = occgrid.info.height
    width::Int64 = occgrid.info.width
    #data = transpose()
    data = reshape(occgrid.data, width, height)
    unknown = data .== -1
    fdata::Array{Float64} = data
    fdata ./= maximum(fdata)
    fdata[unknown] .= 0.5
    return Map(grid=fdata, res=occgrid.info.resolution), unknown
end


function predict_nextmap(fmap::Array{Float64}, unknown::AbstractArray{Bool})
    gipt = pyimport("generative_inpainting.test")
    fipt = gipt.GLOBAL_FILL_INPAINTING
    image = UInt8.(round.(fmap * 255))
    image_3c = repeat(reshape(image, size(image)..., 1), outer=(1, 1, 3))
    unknown = UInt8.(unknown) * UInt8(255)
    unknown_3c = repeat(reshape(unknown, size(unknown)..., 1), outer=(1, 1, 3))
    predicted = fipt.predict(image_3c, unknown_3c)
    return predicted
end


mutable struct Wrap
    x
end


function set_wrapped!(msg, wrap)
    println("msg received")
    wrap.x = msg
end

function _reward(pomdp::SourceSeeker, s::State, a::Action)
end


function POMDPs.initialstate_distribution(p::SourceSeeker)
    mean_pose = (p.min_pose .+ p.max_pose) ./ 2
    return BeliefDistribution(
        occupancy=Base.rand(Float64, p.map_size...),
        pose=MvNormal(mean_pose, 1),
        tgts=[MvNormal(mean_pose[1:p.tgt_size[end]], 1)
              for i in 1:p.tgt_size[1]],
        activated=ones(p.tgt_size[1])
    )
end


function convert_to_occgrid(map::Map, unknown::AbstractArray{Bool}, resolution::Float64 = 0.05)
    occgrid = OccupancyGrid()
    #occgrid.header.stamp = get_rostime()
    occgrid.info.height, occgrid.info.width = UInt32.(size(map.grid))
    occgrid.info.resolution = map.res
    map_i8 = Int8.(round.(map.grid * 100))
    map_i8[unknown] .= -1
    occgrid.data = vcat(map_i8...)
    return occgrid
end


function POMDPs.gen(pomdp::SourceSeeker, s::State, a::Action, rng::AbstractRNG)
    # 1. Publish predicted map and action
    pub = Publisher{OccupancyGrid}(pomdp.sim_map_publish)
    predicted = predict_nextmap(occgrid, unknown)
    pub.publish(convert_to_occgrid(predicted))

    # 2. Subscribe to observation, generated map and action
    wrapped = Wrap(nothing)
    Subscriber{OccupancyGrid}(pomdp.gmapping_map_topic, set_wrapped!, (wrapped,))
    while wrapped.x == nothing
        println("sleeping ...")
        rossleep(0.1)
    end
    occgrid, unknown = convert_occgrid(wrapped.x)
    # 3. Compute the information gain if this action was taken
    rew = _entropy_gain(occgrid, s.map.grid)
    return (sp=next_state, r=rew, o=obs)
end


POMDPs.discount(p::SourceSeeker) = p.discount


function runSimulation()
    pomdp = SourceSeeker()

    hr = HistoryRecorder(max_steps=10)
    rhist = simulate(hr, pomdp, RandomPolicy(pomdp))
    println("""
        Cumulative Discounted Reward (for 1 simulation)
            Random: $(discounted_reward(rhist))""")

    solver = POMCPOWSolver(criterion=MaxUCB(20.0))
    policy = solve(solver, pomdp)
    hist = simulate(hr, pomdp, policy)
    for (s, b, a, r, sp, o) in hist
        @show s
        @show a
        @show r
        println()
    end
    println("""
        Cumulative Discounted Reward (for 1 simulation)
            Random: $(discounted_reward(rhist))
            POMCPOW: $(discounted_reward(hist))
        """)
    return true
end


function plan_next_best_action(b::BeliefDistribution)
    pomdp = SourceSeeker()
    solver = POMCPOWSolver(criterion=MaxUCB(20.0))
    policy = solve(solver, pomdp)
    return policy(b)
end


export SourceSeeker, runSimulation

end # module
