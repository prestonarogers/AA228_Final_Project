using POMDPs
using Distributions: Normal
using Random
import POMDPs: initialstate_distribution, actions, gen, discount, isterminal
using POMDPModelTools # for Deterministic
using BasicPOMCP
using POMDPSimulators
#Random.seed!(1);



mutable struct RCBoatProblem <: POMDPs.POMDP{AbstractArray,Tuple,AbstractArray}
    # To simplify things, define every parameter related to the problem in this struct!
    stateSpace::AbstractArray  # the entire GridWorld
    actionSpace::AbstractArray  # all possible actions takeable (checks legallity later)
    rockPositions::AbstractArray  # where are the rocks?

    homePosition::Tuple  # where am I trying to reach?
    RC_initialPosition::Tuple  # where did the boat start at?
    police_initialPosition::Tuple  # where did the police start at?

    rockReward::Float64  # reward for hitting a rock (should be negative!)
    policeReward::Float64  # reward for getting caught to a police boat (should be highly negative!)
    movementReward::Float64  # for each action I take, there should be a cost (should be negative!)
    reachingHomeReward::Float64  # reward for reaching home (should be highly positive!)

    gettingCaughtDist::Float64  # Eucledean distance that will make us get caught to the police
    discountFactor::Float64  # discount factor (gamma)
end


# All possible states and actions
allStates = [(i,j) for i in collect(1:20) for j in collect(1:20)];
allActions = [(i,j) for i in collect(-1:1) for j in collect(-1:1)];

# Define how many rocks there will be on the GridWorld, and place them randomly
num_of_rocks_to_create = 30
rocks = rand(allStates, num_of_rocks_to_create)

pomdp = RCBoatProblem(allStates, allActions, rocks, (21,20), (5,5), (10,10), -5, -1000, -1, 10000, 10, 0.8)
discount(p::RCBoatProblem) = p.discountFactor
isterminal(p::RCBoatProblem, s::AbstractArray) = isequal(s[1],p.homePosition)
initialstate_distribution(p::RCBoatProblem) = Deterministic([p.RC_initialPosition, p.police_initialPosition]);


# Calculate the Euclidean distance between point a and b.
function EuclideanDistance(a,b)
    return sqrt((a[1]-b[1])^2 + (a[2]-b[2])^2)
end

# Check if the action is legal (does not go outside the limits of the lake).
function isActionLegal(p::RCBoatProblem, s::AbstractArray, a::Tuple)
    RC_state_next = Tuple(collect(s[1])+collect(a))  # just add s and a
    if !(RC_state_next in p.stateSpace)  # if not in stateSpace
        return false
    else
        return true
    end
end

# All possible action, regardless of state (Throws error if you don't put this line).
actions(p::RCBoatProblem) = allActions

# Legal possible actions takeable in a certain state.
function actions(p::RCBoatProblem,s::AbstractArray)
    allowedActions = []
    for a in p.actionSpace
        if isActionLegal(p,s,a)
            push!(allowedActions,a)
        end
    end
    return allowedActions
end

noise(x) = ceil(Int, abs(x - 5)/sqrt(2) + 1e-2)

function gen(p::RCBoatProblem, s::AbstractArray, a::Tuple, rng::AbstractRNG)
    # generate next state
    RC_state_now = s[1]
    police_state_now = s[2]

    RC_state_next = Tuple(collect(s[1])+collect(a))  # just add s and a
    police_state_next = police_state_now

    # sp (next state) is an Array <: AbstractArray of Tuples.
    # First element of this Array should always be the RC boat.
    sp = [RC_state_next, police_state_next]


    # generate observation
    o = []

    # generate reward

    EuclDistReward = EuclideanDistance(p.homePosition,RC_state_now)

    if RC_state_next in p.rockPositions  # hitting rock reward
        r = p.rockReward - EuclDistReward
    elseif isequal(RC_state_next, p.homePosition)  # reaching home reward
        r = p.reachingHomeReward - EuclDistReward
    elseif isequal(RC_state_next, police_state_next)  # hitting a police (need to make it "entering perimeter" instead -- use p.gettingCaughtDist).
        r = p.policeReward - EuclDistReward
    else
        r = p.movementReward - EuclDistReward  # just the reward for making a movement
    end

    return (sp=sp, o=o, r=r)

end



solver = POMCPSolver(tree_queries=1000, c=10)
planner = solve(solver, pomdp);

for (s,a,r,sp,o) in stepthrough(pomdp, planner, "s,a,r,sp,o")
    @show (s,a,r,sp,o)
end


## This is for debugging. It will print the optimal action for a single state.
# b = initialstate_distribution(pomdp)
# a = action(planner, b)
# println("Action $a selected for $b.")
