# Below is used to create the multivariate gaussians
using Distributions

include("Anil_Tests_24Nov.jl")

# We want to return a somewhat random observation based
# on the input parameters
function observationModel(s_prime)
    # In this case, the observation is independent of
    # the action taken given s_prime

    # The actual new position of our boat
    a = collect(s_prime[1])

    # The actual new position of the police boat
    b = collect(s_prime[2])

    euclidean_dist = EuclideanDistance(a,b)

    police_var = max(euclidean_dist/10,2).*ones(2)

    # The normal distribution for picking the observed
    # police-boat location
    police_normal = MvNormal(b, police_var)

    rc_var = ones(2)

    rc_normal = MvNormal(a, rc_var)

    # Obtain samples from the two distributions
    police_obs = rand(police_normal, 1)

    rc_obs = rand(rc_normal, 1)

    rc_obs_floor = Tuple(trunc.(Int, rc_obs))
    police_obs_floor = Tuple(trunc.(Int, police_obs))

    # Return the complete observation
    return (rc_obs_floor, police_obs_floor)
end

# Below is the transition model (which depends on the police direction)
function transitionModel(p::RCBoatProblem, s, a)
    s_prime = copy(s)

    # Below moves the rc boat
    A_s = actions(p,collect(s[1]))

    move = 0

    if rand()< 0.7
        move = a
    else
        A_s_minus_a = filter!(x->x!=a, A_s)

        move = rand(A_s_minus_a)
    end

    s_prime[1].position = Tuple(collect(s[1])+collect(move))

    # Below moves all of the other objects
    for object_index in length(s[2:end])
        # If the object is a policeBoat, it will move with some probability toward us.
        if typeof(s[object_index])==policeBoat
            wall_collision = true

            # Keep looking for a new position until a collision doesn't occue
            while wall_collision
                # Police boats will move toward the rc boat.
                police_traj_x = trunc(Int, sign(s[1].position[1]-s[object_index].position[1]))
                police_traj_y = trunc(Int, sign(s[1].position[2]-s[object_index].position[2]))

                if rand() < 0.4
                    police_new_x = s[object_index].position[1]+police_traj_x
                    police_new_y = s[object_index].position[2]+police_traj_y

                    s_prime[object_index].position = (police_new_x, police_new_y)
                else
                    police_new_x = s[object_index].position[1]+rand(-1:1)
                    police_new_y = s[object_index].position[2]+rand(-1:1)

                    s_prime[object_index].position = (police_new_x, police_new_y)
                end

                wall_collision = wallCollisionDetected(pond, s_prime[object_index])
            end
        else
            wall_collision = true
            while wall_collision
                # Objects other than the police boat will move randomly
                object_new_x = s[object_index].position[1]+rand(-1:1)
                object_new_y = s[object_index].position[2]+rand(-1:1)

                s_prime[object_index].position = (object_new_x, object_new_y)

                wall_collision = wallCollisionDetected(pond, s_prime[object_index])
            end
        end
    end

    return s_prime
end

# Below is an example of how the transition model
# can be used
s_prime = transitionModel(pomdp, ((2,1),(9,3)), (1,1), π)
println(s_prime)

# Below is an example of how the observation model
# can be used
observation = observationModel(s_prime)
println(observation)


function createEnvironment(pond::pond, initialPosition, total_policeBoats, total_sailBoats)
    # Initialize the state array
    state = []

    # All the possible x and y positions
    pond_x = collect(1:pond.width)
    pond_y = collect(1:pond.height)

    wallCollision = true
    collision = true

    # Create the RC boat
    rcBoat = rcBoat(position=initialPosition)

    push!(state, rcBoat)

    for i in 1:total_policeBoats
        # Create the policeBoat variable.
        policeBoat = nothing

        wallCollision = true
        collision = true

        while(wallCollision || collision)


            pos_x = rand(pond_x)
            pos_y = rand(pond_y)

            policeBoat = policeBoat(position=(pos_x,pos_y))

            wallCollision = wallCollisionDetected(pond,policeBoat)

            # Reset collision variable to false.
            collision = false

            # Iterate through the objects in the state and make sure we
            # don't collide with any.
            for object in state
                # If we haven't collided with anything yet, run this code
                if !collision
                    collision = collisionDetected(object, policeBoat)
                end
            end
        end

        push!(state, policeBoat)
    end

    for i in 1:total_sailBoats
        # Create the sailBoat variable.
        sailBoat = nothing

        wallCollision = true
        collision = true

        while(wallCollision || collision)


            pos_x = rand(pond_x)
            pos_y = rand(pond_y)

            sailBoat = sailBoat(position=(pos_x,pos_y))

            wallCollision = wallCollisionDetected(pond,sailBoat)

            # Reset collision variable to false.
            collision = false

            # Iterate through the objects in the state and make sure we
            # don't collide with any.
            for object in state
                # If we haven't collided with anything yet, run this code
                if !collision
                    collision = collisionDetected(object, sailBoat)
                end
            end
        end

        push!(state, sailBoat)
    end

    return state
end

function wallCollisionDetected(pond::pond, object::collideableObjects)
    # If any of these conditions are satisfied, things aren't gucci.
    if object.position[1]-(object.width-1)/2 < 1
        return true
    end
    if object.position[1]+(object.width-1)/2 > pond.width
        return true
    end
    if object.position[2]+(object.height-1)/2 < 1
        return true
    end
    if object.position[2]+(object.height-1)/2 > pond.height
        return true
    end

    return false
end

function collisionDetected(object1::collideableObjects, object2::collideableObjects)
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


function rockCollisionReward(rcBoat::rcBoat, rocks, rockReward)
    rcBoat_top_left_x = trunct(Int, rcBoat.position[1]-(rcBoat.width-1)/2)
    rcBoat_top_left_y = trunc(Int, rcBoat.position[2]+(rcBoat.height-1)/2)

    # Create an array of tuples for all the squares the rcBoat takes up
    rcBoat_squares = [(i,j) for i in collect(rcBoat_top_left_y:(rcBoat_top_left_y+rcBoat.height)) for j in collect(rcBoat_top_left_x:(rcBoat_top_left_x+rcBoat.width))];

    for rcBoat_square in rcBoat_squares
        for rock in rocks
            if rock==rcBoat_square
                return rockReward
            end
        end
    end

    return 0.0
end
