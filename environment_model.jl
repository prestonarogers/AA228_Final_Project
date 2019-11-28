# Below is used to create the multivariate gaussians
using Distributions

include("object_structs.jl")

# We want to return a somewhat random observation based
# on the input parameters
function observationModel(s_prime)
    # In this case, the observation is independent of
    # the action taken given s_prime

    # Create the observation variable
    observation = deepcopy(s_prime)

    # The actual new position of our boat
    a = collect(s_prime[1].position)

    for object_index in 2:length(s_prime)
        # The actual new position of the police boat
        b = collect(s_prime[object_index].position)

        euclidean_dist = EuclideanDistance(a,b)

        #object_var = max(euclidean_dist/20,2).*ones(2)

        object_var = 0 .* ones(2)

        # The normal distribution for picking the observed
        # police-boat location
        object_normal = MvNormal(b, object_var)

        # Obtain sample from the distribution
        object_obs = rand(object_normal, 1)

        object_obs_floor = Tuple(trunc.(Int, object_obs))

        observation[object_index].position = object_obs_floor
    end

    rc_var = zeros(2)

    rc_normal = MvNormal(a, rc_var)

    rc_obs = rand(rc_normal, 1)

    rc_obs_floor = Tuple(trunc.(Int, rc_obs))

    # Set the rcBoat's position in the observation array to the sample
    observation[1].position = rc_obs_floor

    # Return the complete observation
    return observation
end

# Below is the transition model (which depends on the police direction)
function transitionModel(p::RCBoatProblem, s, a)
    global myPond

    s_prime = deepcopy(s)

    # Below moves the rc boat
    A_s = actions(p, s)

    move = 0

    if rand()< 0.7
        move = a
    else
        A_s_minus_a = filter!(x->x!=a, A_s)

        # We must make sure the range is non-empty
        if length(A_s_minus_a) > 0
            move = rand(A_s_minus_a)
        else
            move = a
        end
    end

    s_prime[1].position = Tuple(collect(s[1].position)+collect(move))

    # Below moves all of the other objects
    for object_index in 2:length(s)
        # If the object is a policeBoat, it will move with some probability toward us.
        if typeof(s[object_index])==policeBoat
            wall_collision = true

            police_x = s[object_index].position[1]
            police_y = s[object_index].position[2]
            # Keep looking for a new position until a collision doesn't occur
            while wall_collision
                s_prime[object_index].position = (police_x, police_y)

                # Police boats will move toward the rc boat.
                police_traj_x = trunc(Int, sign(s[1].position[1]-police_x))
                police_traj_y = trunc(Int, sign(s[1].position[2]-police_y))

                random_number = rand()

                if random_number < 0.3
                    police_new_x = police_x+police_traj_x
                    police_new_y = police_y+police_traj_y

                    s_prime[object_index].position = (police_new_x, police_new_y)
                else
                    police_new_x = police_x+rand(-1:1)
                    police_new_y = police_y+rand(-1:1)

                    s_prime[object_index].position = (police_new_x, police_new_y)
                end

                wall_collision = wallCollisionDetected(myPond, s_prime[object_index])
            end
        else
            wall_collision = true

            object_x = s[object_index].position[1]
            object_y = s[object_index].position[2]
            while wall_collision
                s_prime[object_index].position = (object_x, object_y)
                # Objects other than the police boat will move randomly
                object_new_x = object_x+rand(-1:1)
                object_new_y = object_y+rand(-1:1)

                s_prime[object_index].position = (object_new_x, object_new_y)

                wall_collision = wallCollisionDetected(myPond, s_prime[object_index])
            end
        end
    end

    return s_prime
end

function createEnvironment(myPond::pond, initialPosition, total_policeBoats, total_sailBoats)
    # Initialize the state array
    state = []

    # All the possible x and y positions
    pond_x = collect(1:myPond.width)
    pond_y = collect(1:myPond.height)

    wallCollision = true
    collision = true

    # Create the RC boat
    myRcBoat = rcBoat(position=initialPosition)

    push!(state, myRcBoat)

    for i in 1:total_policeBoats
        # Create the policeBoat variable.
        policeBoat_temp = policeBoat(position=(0,0))

        wallCollision = true
        collision = true

        while(wallCollision || collision)
            pos_x = rand(pond_x)
            pos_y = rand(pond_y)

            policeBoat_temp.position = (pos_x,pos_y)

            wallCollision = wallCollisionDetected(myPond,policeBoat_temp)

            # Reset collision variable to false.
            collision = false

            # Iterate through the objects in the state and make sure we
            # don't collide with any.
            for object in state
                # If we haven't collided with anything yet, run this code
                if !collision
                    collision = collisionDetected(object, policeBoat_temp)
                end
            end
        end

        push!(state, policeBoat_temp)
    end

    for i in 1:total_sailBoats
        # Create the sailBoat variable.
        sailBoat_temp = sailBoat(position=(0,0))

        wallCollision = true
        collision = true

        while(wallCollision || collision)
            pos_x = rand(pond_x)
            pos_y = rand(pond_y)

            sailBoat_temp.position = (pos_x,pos_y)

            wallCollision = wallCollisionDetected(myPond,sailBoat_temp)

            # Reset collision variable to false.
            collision = false

            # Iterate through the objects in the state and make sure we
            # don't collide with any.
            for object in state
                # If we haven't collided with anything yet, run this code
                if !collision
                    collision = collisionDetected(object, sailBoat_temp)
                end
            end
        end

        push!(state, sailBoat_temp)
    end

    return state
end

function wallCollisionDetected(myPond::pond, object)
    # If any of these conditions are satisfied, things aren't gucci.
    if object.position[1]-(object.width-1)/2 < 1
        return true
    end
    if object.position[1]+(object.width-1)/2 > myPond.width
        return true
    end
    if object.position[2]-(object.height-1)/2 < 1
        return true
    end
    if object.position[2]+(object.height-1)/2 > myPond.height
        return true
    end

    return false
end

function collisionDetected(object1, object2)
    # We want the positions to be based on the top left corners
    # of the object for this collision detected function.
    object1_top_left_x = object1.position[1]-(object1.width-1)/2
    object1_top_left_y = object1.position[2]+(object1.height-1)/2

    object2_top_left_x = object2.position[1]-(object2.width-1)/2
    object2_top_left_y = object2.position[2]+(object2.height-1)/2

    # If the top and bottom bounds are inside the other, and the
    # left and right bounds are also inside the other, we have
    # a collision.
    return !(
        ((object1_top_left_y + object1.height) < object2_top_left_y) ||
        (object1_top_left_y > (object2_top_left_y + object2.height)) ||
        ((object1_top_left_x + object1.width) < object2_top_left_x) ||
        (object1_top_left_x > (object2_top_left_x + object2.width))
    )
end


function rockCollisionReward(myRcBoat::rcBoat, rocks, rockReward)
    rcBoat_top_left_x = trunc(Int, myRcBoat.position[1]-(myRcBoat.width-1)/2)
    rcBoat_top_left_y = trunc(Int, myRcBoat.position[2]+(myRcBoat.height-1)/2)

    # Create an array of tuples for all the squares the rcBoat takes up
    rcBoat_squares = [(i,j) for i in collect(rcBoat_top_left_y:(rcBoat_top_left_y+myRcBoat.height)) for j in collect(rcBoat_top_left_x:(rcBoat_top_left_x+myRcBoat.width))];

    for rcBoat_square in rcBoat_squares
        for rock in rocks
            if rock==rcBoat_square
                return rockReward
            end
        end
    end

    return 0.0
end

# Calculate the Euclidean distance between point a and b.
function EuclideanDistance(a,b)
    return sqrt((a[1]-b[1])^2 + (a[2]-b[2])^2)
end

# Check if the action is legal (does not go outside the limits of the lake).
function isActionLegal(p::RCBoatProblem, myPond::pond, s, a)
    # The first object in our state is our rcBoat
    RC_state_next = rcBoat(position=Tuple(collect(s[1].position)+collect(a)))  # just add s and a

    # If the rcBoat would collide with the pond in this scenario return false.
    if wallCollisionDetected(myPond, RC_state_next)
        return false
    else
        return true
    end
end

#noise(x) = ceil(Int, abs(x - 5)/sqrt(2) + 1e-2)

# All possible action, regardless of state (Throws error if you don't put this line).
actions(p::RCBoatProblem) = allActions

# Legal possible actions takeable in a certain state.
function actions(p::RCBoatProblem, s::AbstractArray)
    global myPond

    allowedActions = []
    for a in p.actionSpace
        if isActionLegal(p, myPond, s, a)
            push!(allowedActions,a)
        end
    end
    return allowedActions
end
