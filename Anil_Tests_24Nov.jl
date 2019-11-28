
#Random.seed!(1);

include("environment_model.jl")


myPond = pond(height=20, width=20)

# All possible states and actions
allStates = [(i,j) for i in collect(1:myPond.height) for j in collect(1:myPond.width)];
allActions = [(i,j) for i in collect(-1:1) for j in collect(-1:1)];

# Define how many rocks there will be on the GridWorld, and place them randomly
total_policeBoats = 1
total_sailBoats = 1

homePosition = (20,20) # where am I trying to reach?
initialPosition = (5,5) # where did the boat start at?

# Create the starting state array.
starting_state = createEnvironment(myPond, initialPosition, total_policeBoats, total_sailBoats)

# Define how many rocks there will be on the GridWorld, and place them randomly
num_of_rocks_to_create = 30
rocks = rand(allStates, num_of_rocks_to_create)

pomdp = RCBoatProblem(allStates, allActions, rocks, starting_state, homePosition, -5, -1, 10000, 10, 0.8)
discount(p::RCBoatProblem) = p.discountFactor
isterminal(p::RCBoatProblem, s::AbstractArray) = isequal(s[1],p.homePosition)
initialstate_distribution(p::RCBoatProblem) = Deterministic(Tuple{Int, Int}[p.state[object_index].position for object_index in 1:length(p.state)]);

function gen(p::RCBoatProblem, s::AbstractArray, a::Tuple, rng::AbstractRNG)
    global myPond

    state = deepcopy(p.state)
    for object_index in 1:length(state)
        state[object_index].position = s[object_index]
    end

    A_s = actions(p, state)

    # generate next state
    next_state = transitionModel(p, state, a)
    #next_state = deepcopy(state)
    #next_state[1].position = Tuple(collect(next_state[1].position)+collect(a))
    #randomAction = collect(rand(p.actionSpace))
    #next_state[2].position = Tuple(collect(next_state[2].position)+randomAction)

    sp = Tuple{Int, Int}[]
    for object in next_state
        push!(sp, object.position)
    end

    # generate observation
    next_observation = observationModel(next_state)

    o = Tuple{Int, Int}[]
    for obs in next_observation
        push!(o, obs.position)
    end

    # generate reward
    RC_state_now = sp[1]
    EuclDistReward = EuclideanDistance(p.homePosition,RC_state_now)

    # Assign the state variable to our next_state
    p.state = next_state

    # IF our action is in our action space, this negative reward will not apply
    r = any(x->x==a, A_s) ? 0.0 : -1000

    r = r-EuclDistReward
    r = r + rockCollisionReward(next_state[1], p.rockPositions, p.rockReward)

    # We skip over the first object because that is our RC boat.
    for object in state[2:end]
        if collisionDetected(next_state[1], object)
            r = r + object.reward
        end
    end
#println((s=s, sp=sp, o=o, r=r))
    return (sp=sp, o=o, r=r)
end
# State is not in state-space
solver = POMCPSolver(tree_queries=1000, c=10)
planner = solve(solver, pomdp);

k=1
for (s,a,r,sp,o) in stepthrough(pomdp, planner, "s,a,r,sp,o")
    global k
    println("HELLOOOOOOOOOOOOOOO")
    @show (s,a,r,sp,o)
    k = k + 1
end


## This is for debugging. It will print the optimal action for a single state.
# b = initialstate_distribution(pomdp)
# a = action(planner, b)
# println("Action $a selected for $b.")


# Work on gen function so that it works with the structs
