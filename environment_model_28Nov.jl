# Below is used to create the multivariate gaussians
using Distributions

include("object_structs_28Nov.jl")

# We want to return a somewhat random observation based
# on the input parameters
function observationModel(corresponding_objects, s_prime)
    # In this case, the observation is independent of
    # the action taken given s_prime

    # Create the observation variable
    observation = deepcopy(s_prime)

    # The actual new position of our boat
    a = collect(s_prime[1])

    for object_index in 2:length(s_prime)
        # The actual new position of the police boat
        b = collect(s_prime[object_index])

        euclidean_dist = EuclideanDistance(a,b)

        #object_var = max(euclidean_dist/20,2).*ones(2)

        object_var = 0 .* ones(2)

        # The normal distribution for picking the observed
        # police-boat location
        object_normal = MvNormal(b, object_var)

        # Obtain sample from the distribution
        object_obs = rand(object_normal, 1)

        object_obs_floor = Tuple(trunc.(Int, object_obs))

        observation[object_index] = object_obs_floor
    end

    rc_var = zeros(2)

    rc_normal = MvNormal(a, rc_var)

    rc_obs = rand(rc_normal, 1)

    rc_obs_floor = Tuple(trunc.(Int, rc_obs))

    # Set the rcBoat's position in the observation array to the sample
    observation[1] = rc_obs_floor

    # Return the complete observation
    return observation
end

# Below is the transition model (which depends on the police direction)
function transitionModel(p::RCBoatProblem, corresponding_objects, s, a)
    global myPond

    s_prime = deepcopy(s)

    # Below moves the rc boat
    A_s = actions(p, s)

    move = 0

    if rand()< 1
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

    s_prime[1] = Tuple(collect(s[1])+collect(move))

    # Below moves all of the other objects
    for object_index in 2:length(s)
        # If the object is a policeBoat, it will move with some probability toward us.
        if typeof(corresponding_objects[object_index])==policeBoat
            wall_collision = true

            police_x = s[object_index][1]
            police_y = s[object_index][2]
            # Keep looking for a new position until a collision doesn't occur
            while wall_collision
                s_prime[object_index] = (police_x, police_y)

                # Police boats will move toward the rc boat.
                police_traj_x = trunc(Int, sign(s[1][1]-police_x))
                police_traj_y = trunc(Int, sign(s[1][2]-police_y))

                random_number = rand()

                if random_number < 0.3
                    police_new_x = police_x+police_traj_x
                    police_new_y = police_y+police_traj_y

                    s_prime[object_index] = (police_new_x, police_new_y)
                else
                    police_new_x = police_x+rand(-1:1)
                    police_new_y = police_y+rand(-1:1)

                    s_prime[object_index] = (police_new_x, police_new_y)
                end

                wall_collision = wallCollisionDetected(myPond, corresponding_objects[object_index], s_prime[object_index])
            end
        else
            wall_collision = true

            object_x = s[object_index][1]
            object_y = s[object_index][2]
            while wall_collision
                s_prime[object_index] = (object_x, object_y)
                # Objects other than the police boat will move randomly
                object_new_x = object_x+rand(-1:1)
                object_new_y = object_y+rand(-1:1)

                s_prime[object_index] = (object_new_x, object_new_y)

                wall_collision = wallCollisionDetected(myPond, corresponding_objects[object_index], s_prime[object_index])
            end
        end
    end

    return s_prime
end

function createEnvironment(myPond::pond, initialPosition, total_policeBoats, total_sailBoats)
    # Initialize the positions array (our state)
    state = Tuple{Int, Int}[]

    # Create the coresponding objects array
    corresponding_objects = []

    # All the possible x and y positions
    pond_x = collect(1:myPond.width)
    pond_y = collect(1:myPond.height)

    wallCollision = true
    collision = true

    # Create the RC boat
    rcBoatObject = rcBoat()

    push!(corresponding_objects, rcBoatObject)
    push!(state, initialPosition)

    for i in 1:total_policeBoats
        # Create the policeBoat object and it it to the array
        policeBoatObject = policeBoat()
        push!(corresponding_objects, policeBoatObject)

        policeBoat_temp_pos = (0,0)

        wallCollision = true
        collision = true

        while(wallCollision || collision)
            pos_x = rand(pond_x)
            pos_y = rand(pond_y)

            policeBoat_temp_pos = (pos_x,pos_y)

            wallCollision = wallCollisionDetected(myPond,policeBoatObject,policeBoat_temp_pos)

            # Reset collision variable to false.
            collision = false

            # Iterate through the objects in the state and make sure we
            # don't collide with any.
            for obj_index in 1:length(state)
                # If we haven't collided with anything yet, run this code
                if !collision
                    collision = collisionDetected(corresponding_objects[obj_index], state[obj_index], policeBoatObject, policeBoat_temp_pos)
                end
            end
        end

        push!(state, policeBoat_temp_pos)
    end

    for i in 1:total_sailBoats
        # Create the policeBoat object and it it to the array
        sailBoatObject = sailBoat()
        push!(corresponding_objects, sailBoatObject)

        sailBoat_temp_pos = (0,0)

        wallCollision = true
        collision = true

        while(wallCollision || collision)
            pos_x = rand(pond_x)
            pos_y = rand(pond_y)

            sailBoat_temp_pos = (pos_x,pos_y)

            wallCollision = wallCollisionDetected(myPond,sailBoatObject,sailBoat_temp_pos)

            # Reset collision variable to false.
            collision = false

            # Iterate through the objects in the state and make sure we
            # don't collide with any.
            for obj_index in 1:length(state)
                # If we haven't collided with anything yet, run this code
                if !collision
                    collision = collisionDetected(corresponding_objects[obj_index], state[obj_index], sailBoatObject, sailBoat_temp_pos)
                end
            end
        end

        push!(state, sailBoat_temp_pos)
    end

    return state, corresponding_objects
end

function wallCollisionDetected(myPond::pond, obj, pos)
    # If any of these conditions are satisfied, things aren't gucci.
    if pos[1]-(obj.width-1)/2 < 1
        return true
    end
    if pos[1]+(obj.width-1)/2 > myPond.width
        return true
    end
    if pos[2]-(obj.height-1)/2 < 1
        return true
    end
    if pos[2]+(obj.height-1)/2 > myPond.height
        return true
    end

    return false
end

function collisionDetected(obj1, obj1_pos, obj2, obj2_pos)
    # We want the positions to be based on the top left corners
    # of the object for this collision detected function.
    obj1_top_left_x = obj1_pos[1]-(obj1.width-1)/2
    obj1_top_left_y = obj1_pos[2]+(obj1.height-1)/2

    obj2_top_left_x = obj2_pos[1]-(obj2.width-1)/2
    obj2_top_left_y = obj2_pos[2]+(obj2.height-1)/2

    # If the top and bottom bounds are inside the other, and the
    # left and right bounds are also inside the other, we have
    # a collision.
    return !(
        ((obj1_top_left_y + obj1.height) < obj2_top_left_y) ||
        (obj1_top_left_y > (obj2_top_left_y + obj2.height)) ||
        ((obj1_top_left_x + obj1.width) < obj2_top_left_x) ||
        (obj1_top_left_x > (obj2_top_left_x + obj2.width))
    )
end


function rockCollisionReward(myRcBoat::rcBoat, pos, rocks, rockReward)
    rcBoat_top_left_x = trunc(Int, pos[1]-(myRcBoat.width-1)/2)
    rcBoat_top_left_y = trunc(Int, pos[2]+(myRcBoat.height-1)/2)

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
    RC_state_next = rcBoat(position=Tuple(collect(s[1])+collect(a)))  # just add s and a

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
