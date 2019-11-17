module POMDPSeeker


using POMDPs
using Parameters
import Distributions: MvNormal

"""
Class describing the state of the system
"""
struct State
    map::Array{Float64}
    pose::Array{Float64}
    tgts::Array{Float64}
    activated::Array{Bool}
end

struct NDBox
    shape::Tuple{Int64, Vararg{Int64}}
    min::Array{Float64}
    max::Array{Float64}
end


"""
Return a random map
"""
rand(ms::NDBox) = rand(Uniform(-ms.min,ms.max), ms.shape)


"""
Size of the map space
"""
length(ms::NDBox) = (*)(shape...)


"""
Dimensions of map space
"""
dimensions(ms::NDBox) = 2


struct DiscreteSpace{T}
    shape::Tuple{Int64, Vararg{Int64}}
    values::Array{T}
end


rand(ds::DiscreteSpace{T}) where T = rand(ds.values, vs.shape)


struct StateSpace
    mapspace::DiscreteSpace{Bool}
    posespace::NDBox
    tgtspace::NDBox
    activationspace::DiscreteSpace{Bool}
end


rand(ss::StateSpace) = State(map = rand(ss.mapspace),
                             pose = rand(ss.posespace),
                             tgts = rand(ss.tgtspace),
                             activated = rand(ss.activationspace))


"""
Class describing the possible actions
"""
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


struct BeliefDistribution
    occupancy::Array{Float64}
    pose::MvNormal
    tgts::Array{MvNormal}
    activated::Array{Bool}
end

rand(bd::BeliefDistribution) = State(
    map=(rand(Float64, size(bd.occupancy)) .> bd.occupancy),
    pose=rand(bd.pose),
    tgts=rand(bd.tgts),
    activations=rand(bd.activated))

"""
Defining POMPD
"""
@with_kw struct SourceSeeker <: POMDP{State, Action, Observation}
    map_size::Tuple{Int64, Int64} = (4000, 4000)
    pose_size::Tuple{Int64} = (3,)
    tgt_size::Tuple{Int64} = (1, 2)
    state_particles::Int64 = 100
    action_particles::Int64 = 10
    max_velocity::Vector{Float64} = [0.5, 0.5, 0.5]
    min_pose::Vector{Float64} = [-100, -100, -π]
    max_pose::Vector{Float64} = [100, 100, π]
    nscans::Int64 = 180
    laser_range::Vector{Float64} = [0.05, 80]
    state_space = StateSpace(
        mapspace = DiscreteSpace{Bool}(map_size, [false, true]),
        posespace = NDBox(pose_size, min_pose, max_pose),
        tgtspace = NDBox(tgt_size, min_pose, max_pose),
        activationspace = DiscreteSpace{Bool}(tgt_size[1], [false, true]))
    action_space = NDBox(size(max_velocity), -max_velocity, max_velocity)
    obs_space = NDBox((nscans,), laser_range[1], laser_range[end])

end

"""
Return the state space for the pomdp
"""
POMDPs.states(pomdp::SourceSeeker) = pomdp.state_space

"""
Returns a state index
"""
#POMDPs.stateindex(pomdp::SourceSeeker, s::State) = firstindex(pomdp.state_space, s)

POMDPs.actions(pomdp::SourceSeeker) = pomdp.action_space

function POMDPs.transition(pomdp::SourceSeeker, s::State, a::Action)
    new_pose = s.pose + a
    if s.map[new_pose] == 1
        return s
    else
        return State(s.map, new_pose, s.tgts, s.activated)
    end
end

POMDPs.observations(pomdp::SourceSeeker) = pomdp.obs_space

POMDPs.observation(pomdp::SourceSeeker, a::Action, s::State) = rand(pomdp.obs_space)

POMDPs.reward(pomdp::SourceSeeker, s::State, a::Action) = rand(Float64)

function POMDPs.initialstate_distribution(pomdp::SourceSeeker)
    mean_pose = (pomdp.min_pose .+ pomdp.max_pose) ./ 2
    return BeliefDistribution(
        occupancy=rand(Float64, pomdp.map_size),
        pose=MvNormal(mean_pose, 1),
        tgts=[MvNormal(mean_pose[1:tgt_size[end]], 1)
              for i in 1:tgt_size[1]],
        activated=ones(tgt_size[1])
    )
end

end # module
