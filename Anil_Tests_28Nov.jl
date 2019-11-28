using POMDPs
using Distributions: Normal
using Random
import POMDPs: initialstate_distribution, actions, gen, discount, isterminal
using POMDPModelTools # for Deterministic
using BasicPOMCP
using POMDPSimulators
#Random.seed!(1);

include("environment_model_28Nov.jl")

# All possible states and actions
allStates = [(i,j) for i in collect(1:50) for j in collect(1:50)];
allActions = [(i,j) for i in collect(-1:1) for j in collect(-1:1)];

# Define how many rocks there will be on the GridWorld, and place them randomly
num_of_rocks_to_create = 1000
rocks = rand(allStates, num_of_rocks_to_create)

total_policeBoats = 1
total_sailBoats = 1

initialPosition = (5,5)
myPond = pond(height=20, width=20)

# Create the starting state array.
starting_state, corresponding_objects = createEnvironment(myPond, initialPosition, total_policeBoats, total_sailBoats)

pomdp = RCBoatProblem(allStates, allActions, rocks, (50,48), (5,5), starting_state, corresponding_objects, -5, -1, 10000, 10, 0.8)
discount(p::RCBoatProblem) = p.discountFactor
isterminal(p::RCBoatProblem, s::AbstractArray) = isequal(s[1],p.homePosition)
initialstate_distribution(p::RCBoatProblem) = Deterministic(collect(p.state));


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
    RC_state_now = s[1]

    # sp (next state) is an Array <: AbstractArray of Tuples.
    # First element of this Array should always be the RC boat.
    sp = transitionModel(p, p.corresponding_objects, s, a)


    # generate observation
    o = []

    # generate reward

    EuclDistReward = EuclideanDistance(p.homePosition,RC_state_now)

    A_s = actions(p, s)

    # IF our action is in our action space, this negative reward will not apply
    r = any(x->x==a, A_s) ? 0.0 : -1000

    r = r-EuclDistReward
    r = r + rockCollisionReward(corresponding_objects[1], sp[1], p.rockPositions, p.rockReward)

    # We skip over the first object because that is our RC boat.
    for obj_index in 2:length(sp)
        if collisionDetected(p.corresponding_objects[1], sp[1], p.corresponding_objects[obj_index], sp[obj_index])
            r = r + corresponding_objects[obj_index].reward
        end
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
