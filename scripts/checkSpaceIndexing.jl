# check state iterator() and state_index()
# include("pedXingPOMDP.jl")

# check state space
function checkStateSpace(pomdp::PedXingPOMDP)
    println("\n\nChecking state space...")
    for (i, s) in enumerate(iterator(states(pomdp)))
        # println("\niterator (i, s) = (", i, ", ", s, ")")
        # println("state_index = ", state_index(pomdp, s))
        @assert i == state_index(pomdp, s)
    end
    println("iterator(states) and state_index() align.")
end

# check action space
function checkActionSpace(pomdp::PedXingPOMDP)
    println("\nChecking action space...")
    for (i, a) in enumerate(iterator(actions(pomdp)))
        # println("\niterator (i, a) = (", i, ", ", a, ")")
        # println("action_index = ", action_index(pomdp, a))
        @assert i == action_index(pomdp, a)
    end
    println("iterator(actions) and action_index() align.")
end

# check observation space
function checkObservationSpace(pomdp::PedXingPOMDP)
    println("\nChecking observation space...")
    for (i, o) in enumerate(iterator(observations(pomdp)))
        # println("\niterator (i, o) = (", i, ", ", o, ")")
        # println("obs_index = ", obs_index(pomdp, o))
        @assert i == obs_index(pomdp, o)
    end
    println("iterator(observations) and obs_index() align.")
end
