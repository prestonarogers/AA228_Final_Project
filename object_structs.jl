using Parameters

mutable struct collideableObjects
    width::Int64
    height::Int64

    position::Tuple

    reward::Float64
end

@with_kw mutable struct rcBoat <: collideableObjects
    width::Int64 = 3
    height::Int64 = 3

    # We do not have a default position
    position::Tuple

    reward::Float64 = 0
end

@with_kw mutable struct policeBoat <: collideableObjects
    width::Int64 = 3
    height::Int64 = 3

    # We do not have a default position
    position::Tuple

    reward::Float64 = -100 # reward for getting caught to a police boat (should be highly negative!)
end

@with_kw mutable struct sailBoat <: collideableObjects
    width::Int64 = 5
    height::Int64 = 5

    # We do not have a default position
    position::Tuple

    reward::Float64 = -20
end

@with_kw mutable struct rock <: collideableObjects
    width::Int64 = 3
    height::Int64 = 3

    # We do not have a default position
    position::Tuple

    reward::Float64 = -10 # reward for hitting a rock (should be negative!)
end

@with_kw mutable struct pond <: collideableObjects
    width::Int64 = 20
    height::Int64 = 20
end
