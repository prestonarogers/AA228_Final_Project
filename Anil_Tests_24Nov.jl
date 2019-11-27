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

    state::AbstractArray # The state that includes all of the object locations

    homePosition::Tuple  # where am I trying to reach?

    rockReward::Float64  # reward for hitting a rock (should be negative!)

    movementReward::Float64  # for each action I take, there should be a cost (should be negative!)
    reachingHomeReward::Float64  # reward for reaching home (should be highly positive!)

    gettingCaughtDist::Float64  # Eucledean distance that will make us get caught to the police
    discountFactor::Float64  # discount factor (gamma)
end

myPond = pond(height=20, width=20)

# All possible states and actions
allStates = [(i,j) for i in collect(1:pond.height) for j in collect(1:pond.width)];
allActions = [(i,j) for i in collect(-1:1) for j in collect(-1:1)];

# Define how many rocks there will be on the GridWorld, and place them randomly
total_rocks = 5
total_policeBoats = 1
total_sailBoats = 3

homePosition = (21,20) # where am I trying to reach?
initialPosition = (5,5) # where did the boat start at?

# Create the starting state array.
starting_state = createEnvironment(pond, total_policeBoats, total_sailBoats)

# Define how many rocks there will be on the GridWorld, and place them randomly
num_of_rocks_to_create = 30
rocks = rand(allStates, num_of_rocks_to_create)

pomdp = RCBoatProblem(allStates, allActions, rocks, starting_state, homePosition, -5, -1, 10000, 10, 0.8)
discount(p::RCBoatProblem) = p.discountFactor
isterminal(p::RCBoatProblem, s::AbstractArray) = isequal(s[1].position,p.homePosition)
initialstate_distribution(p::RCBoatProblem) = Deterministic([p.RC_initialPosition, p.police_initialPosition]);


# Calculate the Euclidean distance between point a and b.
function EuclideanDistance(a,b)
    return sqrt((a[1]-b[1])^2 + (a[2]-b[2])^2)
end

# Check if the action is legal (does not go outside the limits of the lake).
function isActionLegal(p::RCBoatProblem, pond::pond, s::AbstractArray, a::Tuple)
    # The first object in our state is our rcBoat
    RC_state_next = rcBoat(position=Tuple(collect(s[1].position)+collect(a))  # just add s and a

    # If the rcBoat would collide with the pond in this scenario return false.
    if wallCollisionDetected(pond, RC_state_next)
        return false
    else
        return true
    end
end

# All possible action, regardless of state (Throws error if you don't put this line).
actions(p::RCBoatProblem, myPond) = allActions

# Legal possible actions takeable in a certain state.
function actions(p::RCBoatProblem, s::AbstractArray)
    global pond

    allowedActions = []
    for a in p.actionSpace
        if isActionLegal(p, pond, s,a)
            push!(allowedActions,a)
        end
    end
    return allowedActions
end

noise(x) = ceil(Int, abs(x - 5)/sqrt(2) + 1e-2)

function gen(p::RCBoatProblem, s::AbstractArray, a::Tuple, rng::AbstractRNG)
    global pond

    # generate next state
    sp = transitionModel(p, s, a)

    # generate observation
    o = []

    # generate reward
    RC_state_now = s[1].position
    EuclDistReward = EuclideanDistance(p.homePosition,RC_state_now)

    r = -1.*EuclDistReward
    r = r + rockCollisionReward(rcBoat::rcBoat, p.rockPositions, p.rockReward)

    # We skip over the first object because that is our RC boat.
    for object in p.sp[2:end]
        if collisionDetected(p.state[1], object)
            r = r + object.reward
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


# Work on gen function so that it works with the structs
