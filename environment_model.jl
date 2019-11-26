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
function transitionModel(p::RCBoatProblem, s, a, police_dir)
    A_s = actions(p,collect(s))

    move = 0

    if rand()< 0.7
        move = a
    else
        A_s_minus_a = filter!(x->x!=a, A_s)

        move = rand(A_s_minus_a)
    end

    rc_new_pos = Tuple(collect(s[1])+collect(move))

    police_new_x = trunc(Int, s[2][1]+2*cos(police_dir))
    police_new_y = trunc(Int, s[2][2]+2*sin(police_dir))

    police_new_pos = (police_new_x, police_new_y)

    return (rc_new_pos, police_new_pos)
end

# Below is an example of how the transition model
# can be used
s_prime = transitionModel(pomdp, ((2,1),(9,3)), (1,1), π)
println(s_prime)

# Below is an example of how the observation model
# can be used
observation = observationModel(s_prime)
println(observation)
