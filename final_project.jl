using POMDPs
using Distributions
using DelimitedFiles

#=
Note: The following numbers are ascribed to objects-
0 - water
1 - ME! (rc boat)
2 - rocks
3 - sail-boats
4 - police-boats
5 - HOME!
=#

function goToYourHome()
    # Below creates the pond matrix
    pond, home_pos, sail_dict, police_dict, current_dir = initializePond()

    total_sailboats = length(keys(sail_dict))
    total_policeboats = length(keys(police_dict))

    sail_dir_dict, police_dir_dict = establishDirections(current_dir, sail_dict, police_dict)
global pond = pond
    home_found = false
global old_pond = copy(pond)

end

function obtainPondMatrix(pond_filename)
    # Ascertain the default environment
    # (This includes information of rock locations and pond size)
    pond_CSV = CSV.read(pwd()*"\\"*pond_filename, delim=",")
    pond = convert(Matrix{Float64}, pond_CSV)

    # This will obtain the width and height of the pond matrix
    pond_height = size(pond, 1)
    pond_width = size(pond, 2)

    return pond, pond_height, pond_width
end

    while !home_established
        # We will always place it on the right side of the pond
        home_pos[1] = convert(Int64, rand(trunc(Int64,(1/4)*pond_height):trunc(Int64,(3/4)*pond_height)))
        home_pos[2] = convert(Int64, pond_width)

        if pond[home_pos[1],home_pos[2]] == 0
            pond[home_pos[1],home_pos[2]] = 5
            home_established = true
        end
    end

    # We will use a radians value to dictate the current direction
    current_dir = rand()

    return pond, home_pos, sail_dict, police_dict, current_dir
end

function establishDirections(current_dir, sail_dict, police_dict)
    dir_distribution =  Beta(1+(10*current_dir),1+(10*(1-current_dir)))
    # Obtain dictionary of sailboat trajectories
    sail_dir_dict = Dict()

    for key in keys(sail_dict)
        dir = rand(dir_distribution)
        push!(sail_dir_dict,(key=>dir))
    end

    # Obtain dictionary of policeboat trajectories
    police_dir_dict = Dict()

    for key in keys(police_dict)
        dir = rand(dir_distribution)
        push!(police_dir_dict,(key=>dir))
    end

    return sail_dir_dict, police_dir_dict
end

goToYourHome()
