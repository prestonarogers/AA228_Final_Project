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
pond = 0
old_pond = 0
new_pond = 0
function goToYourHome()
    # Below creates the pond matrix
    pond, home_pos, sail_dict, police_dict, current_dir = initializePond()

    total_sailboats = length(keys(sail_dict))
    total_policeboats = length(keys(police_dict))

    sail_dir_dict, police_dir_dict = establishDirections(current_dir, sail_dict, police_dict)
global pond = pond
    home_found = false
global old_pond = copy(pond)
    # Below shows what would happen to the positions after two time steps
    for i in 1:10
        writedlm( "pond_"*string(i)*".csv",pond, ',')
        pond_height = size(pond, 1)
        pond_width = size(pond, 2)
        for key in keys(sail_dict)
            # Calculate the new position components for this sailboat
            new_y_pos = sail_dict[key][1]+2*sin(sail_dir_dict[key]*(2*pi))
            new_x_pos = sail_dict[key][2]+2*cos(sail_dir_dict[key]*(2*pi))

            size = 5
            margin = convert(Int64, (size-1)/2)

            new_y_trunc = trunc(Int64, new_y_pos)
            new_x_trunc = trunc(Int64, new_x_pos)

            if !(new_y_trunc-margin >= 1) || !(new_y_trunc+margin <= pond_height) || !(new_x_trunc-margin >= 1) || !(new_x_trunc+margin <= pond_width)
                continue
            end

            move_enabled = true

            # Clear out the preoccupied spots
            for y in (trunc(Int64, sail_dict[key][1])-margin):(trunc(Int64, sail_dict[key][1])+margin)
                for x in (trunc(Int64, sail_dict[key][2])-margin):(trunc(Int64, sail_dict[key][2])+margin)
                    pond[y,x] = 0
                end
            end

            # Make sure no other objects are there
            for y in (new_y_trunc-margin):(new_y_trunc+margin)
                for x in (new_x_trunc-margin):(new_x_trunc+margin)
                    if pond[y,x] > 0
                        print("Disatisfied at:")
                        print(y)
                        print(" ")
                        println(x)
                        move_enabled = false
                    end
                end
            end

            if !move_enabled
                for y in (trunc(Int64, sail_dict[key][1])-margin):(trunc(Int64, sail_dict[key][1])+margin)
                    for x in (trunc(Int64, sail_dict[key][2])-margin):(trunc(Int64, sail_dict[key][2])+margin)
                        pond[y,x] = 3
                    end
                end

                continue
            end

            for y in (new_y_trunc-margin):(new_y_trunc+margin)
                for x in (new_x_trunc-margin):(new_x_trunc+margin)
                    pond[y,x] = 3
                end
            end

            sail_dict[key]=[new_y_pos,new_x_pos]
        end
        for key in keys(police_dict)
            # Calculate the new position components for this policeboat
            new_y_pos = police_dict[key][1]+2*sin(police_dir_dict[key]*(2*pi))
            new_x_pos = police_dict[key][2]+2*cos(police_dir_dict[key]*(2*pi))

            size = 3
            margin = convert(Int64, (size-1)/2)

            new_y_trunc = trunc(Int64, new_y_pos)
            new_x_trunc = trunc(Int64, new_x_pos)

            if !(new_y_trunc-margin >= 1) || !(new_y_trunc+margin <= pond_height) || !(new_x_trunc-margin >= 1) || !(new_x_trunc+margin <= pond_width)
                continue
            end

            move_enabled = true

            # Clear out the preoccupied spots
            for y in (trunc(Int64, police_dict[key][1])-margin):(trunc(Int64, police_dict[key][1])+margin)
                for x in (trunc(Int64, police_dict[key][2])-margin):(trunc(Int64, police_dict[key][2])+margin)
                    pond[y,x] = 0
                end
            end

            # Make sure no other objects are there
            for y in (new_y_trunc-margin):(new_y_trunc+margin)
                for x in (new_x_trunc-margin):(new_x_trunc+margin)
                    if pond[y,x] > 0
                        print("Disatisfied at:")
                        print(y)
                        print(" ")
                        println(x)
                        move_enabled = false
                    end
                end
            end

            if !move_enabled
                for y in (trunc(Int64, police_dict[key][1])-margin):(trunc(Int64, police_dict[key][1])+margin)
                    for x in (trunc(Int64, police_dict[key][2])-margin):(trunc(Int64, police_dict[key][2])+margin)
                        pond[y,x] = 4
                    end
                end

                continue
            end

            for y in (new_y_trunc-margin):(new_y_trunc+margin)
                for x in (new_x_trunc-margin):(new_x_trunc+margin)
                    pond[y,x] = 4
                end
            end

            police_dict[key]=[new_y_pos,new_x_pos]
        end
    end
    global new_pond = copy(pond)
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


# Below initializes the position of obstacles
function initializePond()
    pond_height = 25
    pond_width = 25

    pond = zeros(pond_height, pond_width)

    # Below creates the locations of the rocks

    size = 3

    rocks_remaining = rand(1:10)

    while rocks_remaining > 0
        # Pick a random x and y location for the total_sailboat
        pos_y = rand(1:pond_height)
        pos_x = rand(1:pond_width)

        margin = convert(Int, ((size-1)/2))

        # Obtain the relevant region of pond as a matrix
        # If the positions aren't within our matrix, don't put a rock there!
        if !(pos_y-margin >= 1) || !(pos_y+margin <= pond_height) || !(pos_x-margin >= 1) || !(pos_x+margin <= pond_width)
            continue
        end
        region = pond[(pos_y-margin):(pos_y+margin),(pos_x-margin):(pos_x+margin)]

        # If there is only water at this location, we can add a policeboat here.
        for y in (pos_y-margin):(pos_y+margin)
            for x in (pos_x-margin):(pos_x+margin)
                pond[y,x] = 2
            end
        end

        # Now we decrement the rocks_remaining variable
        rocks_remaining -= 1
    end


    # Here we will generate random starting locations for sail-boats
    # The sailboat will have a width and height of 5 squares
    size = 5
    water = zeros(size, size)

    sail_dict = Dict()

    sailboats_remaining = 2

    while sailboats_remaining > 0
        # Pick a random x and y location for the total_sailboat
        pos_y = rand(1:pond_height)
        pos_x = rand(1:pond_width)

        margin = convert(Int, ((size-1)/2))

        # Obtain the relevant region of pond as a matrix
        # If the positions aren't within our matrix, don't put a sailboat there!
        if !(pos_y-margin >= 1) || !(pos_y+margin <= pond_height) || !(pos_x-margin >= 1) || !(pos_x+margin <= pond_width)
            continue
        end

        region = pond[(pos_y-margin):(pos_y+margin),(pos_x-margin):(pos_x+margin)]

        # If there is only water at this location, we can add a sailboat here.
        if isequal(region,water)
            for y in (pos_y-margin):(pos_y+margin)
                for x in (pos_x-margin):(pos_x+margin)
                    pond[y,x] = 3
                end
            end

            # Add this sailboat to our dictionary of sailboats
            push!(sail_dict, (sailboats_remaining => [pos_y,pos_x]))

            # Now we decrement the sailboats_remaining variable
            sailboats_remaining -= 1
        end
    end

    # Here we will generate random starting locations for police-boats
    # The sailboat will have a width and height of 3 squares
    size = 3
    water = zeros(size, size)

    police_dict = Dict()

    policeboats_remaining = 1

    while policeboats_remaining > 0
        # Pick a random x and y location for the total_sailboat
        pos_y = rand(1:pond_height)
        pos_x = rand(1:pond_width)

        margin = convert(Int, ((size-1)/2))

        # Obtain the relevant region of pond as a matrix
        # If the positions aren't within our matrix, don't put a policeboat there!
        if !(pos_y-margin >= 1) || !(pos_y+margin <= pond_height) || !(pos_x-margin >= 1) || !(pos_x+margin <= pond_width)
            continue
        end
        region = pond[(pos_y-margin):(pos_y+margin),(pos_x-margin):(pos_x+margin)]

        # If there is only water at this location, we can add a policeboat here.
        if isequal(region,water)
            for y in (pos_y-margin):(pos_y+margin)
                for x in (pos_x-margin):(pos_x+margin)
                    pond[y,x] = 4
                end
            end

            # Add this sailboat to our dictionary of sailboats
            push!(police_dict, (policeboats_remaining => [pos_y,pos_x]))

            # Now we decrement the policeboats_remaining variable
            policeboats_remaining -= 1
        end
    end

    # Below establishes the square that references HOME
    home_established = false

    # Initialize the home position variables
    home_pos = zeros(Int64, 2)

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
