# Interface to define custom state types

"""
    posf(state)
returns the coordinates of the state in the Frenet frame. 
The return type is expected to be `Frenet`.
"""
function posf end

"""
    posg(state)
returns the coordinates of the state in the global (world) frame.
The return type is expected to be a VecSE2.
"""
function posg end 

"""
    vel(state)
returns the norm of the longitudinal velocity.
"""
function vel end

"""
    velf(state)
returns the velocity of the state in the Frenet frame.
"""
function velf end 

"""
    velg(state)
returns the velocity of the state in the global (world) frame.
"""
function velg end
