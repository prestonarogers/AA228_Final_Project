using POMDPs
using Distributions: Normal
using Random
import POMDPs: initialstate_distribution, actions, gen, discount, isterminal
using POMDPModelTools # for Deterministic
using BasicPOMCP
using POMDPSimulators
using Parameters


mutable struct RCBoatProblem <: POMDPs.POMDP{AbstractArray,Tuple,AbstractArray}
    # To simplify things, define every parameter related to the problem in this struct!
    stateSpace::AbstractArray  # the entire GridWorld
    actionSpace::AbstractArray  # all possible actions takeable (checks legallity later)
    rockPositions::AbstractArray  # where are the rocks?

    state::AbstractArray # The state that includes all of the object locations

    homePosition::Tuple  # where am I trying to reach?

    rockReward::Float64  # reward for hitting a rock (should be negative!)

    movementReward::Float64  # for each action I take, there should be a cost (should be negative!)
    reachingHomeReward::Float64  # reward for reaching home (should be highly positive!)

    gettingCaughtDist::Float64  # Euclidean distance that will make us get caught to the police
    discountFactor::Float64  # discount factor (gamma)
end

abstract type collideableObjects end

@with_kw mutable struct rcBoat 
    width::Int64 = 3
    height::Int64 = 3

    # We do not have a default position
    position::Tuple = (0,0)

    reward::Float64 = 0.0
end

@with_kw mutable struct policeBoat
    width::Int64 = 3
    height::Int64 = 3

    # We do not have a default position
    position::Tuple = (0,0)

    reward::Float64 = -100.0 # reward for getting caught to a police boat (should be highly negative!)
end

@with_kw mutable struct sailBoat
    width::Int64 = 5
    height::Int64 = 5

    # We do not have a default position
    position::Tuple = (0,0)

    reward::Float64 = -20.0
end

@with_kw mutable struct pond
    width::Int64 = 20
    height::Int64 = 20
end
