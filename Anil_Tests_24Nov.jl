
#Random.seed!(1);

include("environment_model.jl")

println("HEre 0")
myPond = pond(height=20, width=20)
println("HEre -1")
# All possible states and actions
allStates = [(i,j) for i in collect(1:myPond.height) for j in collect(1:myPond.width)];
allActions = [(i,j) for i in collect(-1:1) for j in collect(-1:1)];

# Define how many rocks there will be on the GridWorld, and place them randomly
total_policeBoats = 1
total_sailBoats = 1

homePosition = (20,20) # where am I trying to reach?
initialPosition = (5,5) # where did the boat start at?
println("HEre 1")
# Create the starting state array.

starting_state = createEnvironment(myPond, initialPosition, total_policeBoats, total_sailBoats)
println("HEre 1")
# Define how many rocks there will be on the GridWorld, and place them randomly
num_of_rocks_to_create = 30
rocks = rand(allStates, num_of_rocks_to_create)

pomdp = RCBoatProblem(allStates, allActions, rocks, starting_state, homePosition, -5, -1, 10000, 10, 0.8)
discount(p::RCBoatPro2lem) = p.discountFactor
println("HEre 3")
isterminal(p::RCBoatProblem, s::AbstractArray) = isequal(s[1].position,p.homePosition)
initialstate_distribution(p::RCBoatProblem) = Deterministic(p.state);
println("HEre 4")

function gen(p::RCBoatProblem, s::AbstractArray, a::Tuple, rng::AbstractRNG)
    global myPond

    # generate next state
    sp = transitionModel(p, s, a)

    # generate observation
    o = observationModel(sp)

    # generate reward
    RC_state_now = s[1].position
    EuclDistReward = EuclideanDistance(p.homePosition,RC_state_now)

    r = -1.*EuclDistReward
    r = r + rockCollisionReward(sp[1], p.rockPositions, p.rockReward)

    # We skip over the first object because that is our RC boat.
    for object in sp[2:end]
        if collisionDetected(s[1], object)
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
