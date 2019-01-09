using POMDPs, POMDPModelTools, POMDPPolicies, BeliefUpdaters, GridInterpolations, QMDP, Random

println("\n\nLoading model...")

##############
## Model POMDP
##############

# CONSTANTS
const CROSSING = true
const NOT_CROSSING = false
const STREET = true
const CROSSWALK = true
const SIDEWALK = false
const DISTRACTED = 1
const STOPPED    = 2
const WALKING    = 3

# STATE
mutable struct PedXingState
    v::Float64     # velocity
    d::Float64     # distance to crosswalk
    c::Bool        # pedestrian in crosswalk
    pv::Int        # pedestrian posture
    aPrev::Float64 # previous accel command
end
function Base.copy!(s1::PedXingState, s2::PedXingState) 
    s1.v = s2.v
    s1.d = s2.d
    s1.c = s2.c
    s1.pv = s2.pv
    s1.aPrev = s2.aPrev
    s1
end
Base.isequal(s1::PedXingState, s2::PedXingState) = s1.v == s2.v && s1.d == s2.d && s1.c == s2.c && s1.pv == s2.pv && s1.aPrev == s2.aPrev

# POMDP
struct PedXingPOMDP <: POMDP{PedXingState, Float64, PedXingState} # state PedXingState, action Float64, observation PedXingState
    max_a::Float64      # max acceleration [m/s^2]
    min_a::Float64      # min acceleration [m/s^2]
    step_a::Float64     # acceleration increments [m/s^2]
    max_v::Float64      # max velocity [m/s]
    step_v::Float64     # discretization step size for velocity [m/s]
    max_d::Float64      # distance to crosswalk [m]
    step_d::Float64     # discretization step size for distance to crosswalk [m]
    num_pv::Int         # max number of pedestrian postures
    r_safety::Float64   # penalty for hitting pedestrian
    r_yield::Float64    # penalty for not exhibiting due care to yield to pedestrian
    r_buffer::Float64   # in denominator for distance so don't get -Inf penalty [m]
    r_efficiency::Float64  # reward for efficiency
    r_smooth::Float64   # reward for smoothness
    r_comfort::Float64  # reward for comfort (i.e. low accelerations)
    r_buffer2::Float64  # buffer on terminal penalty
    min_velocity::Float64 #minimum velocity that draws penalty
    r_min_velocity::Float64 #penalty to encourage speeding up after car passes
    dt::Float64         # time step (default 0.05 sec)
    gamma::Float64      # discount factor (default 0.9)
    grid::GridInterpolations.RectangleGrid # grid for interpolating
    sim_maxxt::Float64  # for simulation: maximum time for pedestrian to cross street [s]
    sim_xtime::Float64  # for simulation: current duration pedestrian crossing [s]
    sim_vdist::Float64  # for simulation: vehicle distance when pedestrian appears [m]
end
# default constructor - use key worded arguments to change any of the values passed in 
function PedXingPOMDP(;ua::Float64  =  3.0, # max_a
                       la::Float64  =-10.0, # min_a
                       sa::Float64  =  0.5, # step_a
                       mv::Float64  = 10.0, # max_v
                       sv::Float64  =  0.5, # step_v
                       md::Float64  = 40.0, # max_d
                       sd::Float64  =  1.0, # step_d
                       npv::Int     =    3, # num_pv
                       rsa::Float64 =  3.0, #r_safety (eta, terminal penalty on being past crosswalk while driver is walking)
                       ry::Float64  =  0.01, #r_yield (zeta, units of s^2/m)
                       rb::Float64  =  3.0, # r_buffer (units of meters)
                       re::Float64  =  0.1, # r_efficiency lambda, units of s/m)
                       rsm::Float64 =  0.1, # r_smooth
                       rcmf::Float64=  0.0, # r_comfort
                       rb2::Float64 =  3.0,  # buffer on terminal penalty (units of meters)
                       minv::Float64 = 2.0,  # min velocity that draws penalty
                       rminv::Float64 = 10.0, #penalty on not speeding up when not crossing. 
                       dt::Float64  = 0.05, # dt
                       g::Float64   =  0.9, # gamma
                       mt::Float64  =  5.0, # sim_maxxt
                       xt::Float64  =  0.0, # sim_xtime
                       vd::Float64  = 20.0) # sim_vdist
    grid = RectangleGrid(0:sv:mv, -sd:sd:md) # start d at -sd to check isDone
    return PedXingPOMDP(ua, la, sa, mv, sv, md, sd, npv, rsa, ry, rb, re, rsm, rcmf, rb2, minv, rminv, dt, g, grid, mt, xt, vd)
end
function incrementSimXT(pomdp::PedXingPOMDP)
    pomdp.sim_xtime += pomdp.dt
end

# DISCOUNT FACTOR
POMDPs.discount(pomdp::PedXingPOMDP) = pomdp.gamma

# TERMINAL CONDITION
function POMDPs.isterminal(pomdp::PedXingPOMDP, s::PedXingState)
    if (s.d < 0.0)
        return true
    end
    return false
end

## SPACES
function POMDPs.states(pomdp::PedXingPOMDP)
    s = PedXingState[] # initialize an array of PedXingStates
    for d=-pomdp.step_d:pomdp.step_d:pomdp.max_d, v=0:pomdp.step_v:pomdp.max_v, c=0:1, pv=1:pomdp.num_pv, ap=pomdp.min_a:pomdp.step_a:pomdp.max_a
        push!(s, PedXingState(v, d, c, pv, ap))
    end
    return s
end
POMDPs.n_states(pomdp::PedXingPOMDP) = length(0:pomdp.step_v:pomdp.max_v)*length(-pomdp.step_d:pomdp.step_d:pomdp.max_d)*2*pomdp.num_pv*length(pomdp.min_a:pomdp.step_a:pomdp.max_a)
function POMDPs.stateindex(pomdp::PedXingPOMDP, state::PedXingState)
    sv = Int(state.v / pomdp.step_v) + 1
    sd = Int(state.d / pomdp.step_d) + 1 + Int(pomdp.step_d)
    sc = Int(state.c + 1)
    pv = state.pv
    ap = round(Int, state.aPrev / pomdp.step_a + 1 + abs(pomdp.min_a / pomdp.step_a))
    s2i = LinearIndices((length(pomdp.min_a:pomdp.step_a:pomdp.max_a), pomdp.num_pv, 2, length(0:pomdp.step_v:pomdp.max_v), length(-pomdp.step_d:pomdp.step_d:pomdp.max_d)))
    index = s2i[ap, pv, sc, sv, sd]
    return index
end

# ACTION SPACE
POMDPs.actions(pomdp::PedXingPOMDP) = collect(pomdp.min_a:pomdp.step_a:pomdp.max_a) # default
POMDPs.n_actions(pomdp::PedXingPOMDP) = length(collect(pomdp.min_a:pomdp.step_a:pomdp.max_a))
function POMDPs.actionindex(pomdp::PedXingPOMDP, action::Float64)
    a = round(Int, action / pomdp.step_a + 1 + abs(pomdp.min_a / pomdp.step_a))
    return a
end

# OBSERVATION SPACE
function POMDPs.observations(pomdp::PedXingPOMDP)
    o = PedXingState[] # initialize an array of PedXingStates
    for d=-pomdp.step_d:pomdp.step_d:pomdp.max_d, v=0:pomdp.step_v:pomdp.max_v, c=0:1, pv=1:pomdp.num_pv, ap=pomdp.min_a:pomdp.step_a:pomdp.max_a
        push!(o, PedXingState(v, d, c, pv, ap))
    end
    return o
end
POMDPs.n_observations(pomdp::PedXingPOMDP) = length(0:pomdp.step_v:pomdp.max_v)*length(-pomdp.step_d:pomdp.step_d:pomdp.max_d)*2*pomdp.num_pv*length(pomdp.min_a:pomdp:step_a:pomdp.max_a)
function POMDPs.obsindex(pomdp::PedXingPOMDP, obs::PedXingState)
    sv = Int(obs.v / pomdp.step_v) + 1
    sd = Int(obs.d / pomdp.step_d) + 1 + Int(pomdp.step_d)
    sc = Int(obs.c + 1)
    pv = obs.pv
    ap = round(Int, obs.aPrev / pomdp.step_a + 1 + abs(pomdp.min_a / pomdp.step_a))
    s2i = LinearIndices((length(pomdp.min_a:pomdp.step_a:pomdp.max_a), pomdp.num_pv, 2, length(0:pomdp.step_v:pomdp.max_v), length(-pomdp.step_d:pomdp.step_d:pomdp.max_d)))
    index = s2i[ap, pv, sc, sv, sd]
    return index
end

## MODELS
# REWARD
function POMDPs.reward(pomdp::PedXingPOMDP, s::PedXingState, a::Float64, sp::PedXingState)
    # initialize
    r = 0

    ## safety
    if (s.c == CROSSING) && (sp.d < pomdp.r_buffer2)
       r -= pomdp.r_safety
    end

    ## legality
    if (s.c == CROSSING)
       r -= pomdp.r_yield*(s.v^2)/(s.d+pomdp.r_buffer)
    end

    ## mobility
    if (s.c == NOT_CROSSING)
        r += pomdp.r_efficiency*s.v # lambda * speed
    end

    if (s.c == NOT_CROSSING) && (sp.v < pomdp.min_velocity)
        r-= pomdp.r_min_velocity
    end



    #r -= pomdp.r_smooth*(a - s.aPrev)^2 # smoothness
    r -= pomdp.r_comfort*(a)^2 #penalize large acceleration



    if (false)
        println("\n current state: $s, action: $a")
        println("reward: $r")
    end

    return r
end

# TRANSITION
mutable struct TransDistribution
    nextstates::Vector{PedXingState} # the states s' in the distribution
    probs::Vector{Float64} # the probability corresponding to each state s'
    pomdp::PedXingPOMDP
end
function POMDPs.support(d::TransDistribution)
    return d.nextstates
end
function POMDPs.pdf(d::TransDistribution, s::PedXingState)
    for (i, sp) in enumerate(d.nextstates)
        if isequal(s, sp)
            return d.probs[i]
        end
    end   
    return 0.0
end
function POMDPs.transition(pomdp::PedXingPOMDP, s::PedXingState, action::Float64)
    nextstates = PedXingState[]
    probs = Float64[]

    # add actuation delay if a=-10
    a = action
    if (a == -10.0)
        a = -9.9
    end

    # point mass distance
    x = max(0.0, s.v*pomdp.dt + 0.5*a*(pomdp.dt^2)) # assume x0=0 & don't go backwards
    d = s.d - x # convert to distance to crosswalk

    # point mass velocity
    v = max(0.0, s.v + a*pomdp.dt) # always positive

    # determine if reached crosswalk, otherwise interpolate grid
    if (isterminal(pomdp,s)) # vehicle within crosswalk
        push!(nextstates, s)
        push!(probs, 1.0)
    else
        xp = [0.0, 0.0]
        (grid_index, grid_probs) = interpolants(pomdp.grid, [v, d])
        for i in 1:length(grid_index)
            ind2x!(pomdp.grid, grid_index[i], xp)
            (sp_v, sp_d) = (xp[1], xp[2])
            p_xing = 1.0 # ped stays in crosswalk
            if (s.c == SIDEWALK)
                if (s.pv == DISTRACTED)
                    p_xing = 0.5
                elseif (s.pv == STOPPED)
                    p_xing = 0.523 * (max(0.0,sp_d+pomdp.r_buffer)/(pomdp.max_d+pomdp.r_buffer))
                elseif (s.pv == WALKING)
                    p_xing = 0.867
                end
            end
            p_noxing = 1.0 - p_xing

            # pedestrian on SIDEWALK in next state
            push!(nextstates, PedXingState(sp_v, sp_d,  SIDEWALK, s.pv, action))
            push!(probs, grid_probs[i]*p_noxing)
            # pedestrian in CROSSWALK in next state
            push!(nextstates, PedXingState(sp_v, sp_d, CROSSWALK, s.pv, action))
            push!(probs, grid_probs[i]*p_xing)
        end
    end

    if (false)
        println("\ncurrent state: $s, action: $a")
        println("point mass: v=$v, d=$d")
        println("nextstates: ", nextstates)
        println("probs: ", probs)
    end

    return TransDistribution(nextstates, probs, pomdp)
end

# OBSERVATION
mutable struct ObsDistribution
    obs::Vector{PedXingState}  # entire observation space
    probs::Vector{Float64} # probability of each observation
    pomdp::PedXingPOMDP
end
function POMDPs.support(d::ObsDistribution)
    return d.obs
end
function POMDPs.pdf(d::ObsDistribution, o::PedXingState)
    for (i, op) in enumerate(d.obs)
        if isequal(o, op)
            return d.probs[i]
        end
    end   
    return 0.0
end
function POMDPs.observation(pomdp::PedXingPOMDP, action::Float64, s::PedXingState)
    obs = PedXingState[]
    probs = Float64[]

    # determine if reached crosswalk, otherwise interpolate grid
    if (isterminal(pomdp,s)) # vehicle within crosswalk
        push!(obs, s)
        push!(probs, 1.0)
    else
        xp = [0.0, 0.0]
        (grid_index, grid_probs) = interpolants(pomdp.grid, [s.v, s.d])
        for i in 1:length(grid_index)
            ind2x!(pomdp.grid, grid_index[i], xp)
            (sp_v, sp_d) = (xp[1], xp[2])
            p_xing = 0.95 # obs ped stays in crosswalk
            if (s.c == SIDEWALK)
                if (s.pv == DISTRACTED)
                    p_xing = 0.10
                elseif (s.pv == WALKING)
                    p_xing = 0.05
                elseif (s.pv == STOPPED)
                    p_xing = 0.05
                end
            end
            p_noxing = 1.0 - p_xing

            # pedestrian on SIDEWALK in next state
            push!(obs, PedXingState(sp_v, sp_d,  SIDEWALK, s.pv, action))
            push!(probs, grid_probs[i]*p_noxing)
            # pedestrian in CROSSWALK in next state
            push!(obs, PedXingState(sp_v, sp_d, CROSSWALK, s.pv, action))
            push!(probs, grid_probs[i]*p_xing)
            #for a in actions(pomdp)
            #    # pedestrian on SIDEWALK in next state
            #    push!(obs, PedXingState(sp_v, sp_d,  SIDEWALK, s.pv, a))
            #    push!(probs, grid_probs[i]*p_noxing)
            #    # pedestrian in CROSSWALK in next state
            #    push!(obs, PedXingState(sp_v, sp_d, CROSSWALK, s.pv, a))
            #    push!(probs, grid_probs[i]*p_xing)
            #end
        end
    end

    # normalize
    probs = probs/sum(probs)

    if (false)
        println("\ncurrent state: $s, action: $a")
        println("obs: ", obs)
        println("probs: ", probs)
    end

    return ObsDistribution(obs, probs, pomdp)
    #return ObsDistribution([s], [1.0])
end

# initial distribution
POMDPs.initialstate_distribution(pomdp::PedXingPOMDP) = transition(pomdp, PedXingState(0.0,pomdp.max_d,SIDEWALK,1,0.0), 0.0);

# belief over s.v
function getVehSpeedBelief(b::DiscreteBelief)
    n = length(b.pomdp.grid.cutPoints[1])
    pv = zeros(n)
    for (i, s) in enumerate(iterator(b))
        pv[Int(s.v / b.pomdp.step_v) + 1] += b.b[i]
    end
    pv = pv / sum(pv) # normalize
    return DiscreteBelief(b.pomdp, b.pomdp.grid.cutPoints[1], pv)
end

# belief over s.d
function getVehDistBelief(b::DiscreteBelief)
    n = length(b.pomdp.grid.cutPoints[2])
    pd = zeros(n)
    for (i, s) in enumerate(iterator(b))
        pd[Int(s.d / b.pomdp.step_d) + 2] += b.b[i]
    end
    pd = pd / sum(pd) # normalize
    return DiscreteBelief(b.pomdp, b.pomdp.grid.cutPoints[2], pd)
end

# belief over s.c
function getPedBelief(b::DiscreteBelief)
    n = 2
    pc = zeros(n)
    for (i, s) in enumerate(iterator(b))
        pc[Int(s.c + 1)] += b.b[i]
    end
    pc = pc / sum(pc) # normalize
    return DiscreteBelief(b.pomdp, [SIDEWALK, CROSSWALK], pc)
end

# belief over s.pv
function getPedPostureBelief(b::DiscreteBelief)
    n = b.pomdp.num_pv
    ppv = zeros(n)
    for (i, s) in enumerate(iterator(b))
        ppv[s.pv] += b.b[i]
    end
    ppv = ppv / sum(ppv) # normalize
    return DiscreteBelief(b.pomdp, collect(1:n), ppv)
end

# belief over previous action
function getAPrevBelief(b::DiscreteBelief)
    n = n_actions(b.pomdp)
    pa = zeros(n)
    for (i, s) in enumerate(iterator(b))
        ndx = round(Int, state.aPrev / pomdp.step_a + 1 + abs(pomdp.min_a / pomdp.step_a))
        pa[ndx] += b.b[i]
    end
    pa = pa / sum(pa) # normalize
    return DiscreteBelief(b.pomdp, action(b.pomdp), pa)
end

########################
# simulation functions #
function POMDPs.rand(rng::AbstractRNG, d::TransDistribution)
    if ((any(s -> s.d <= d.pomdp.sim_vdist, iterator(d))) && (d.pomdp.sim_xtime <= d.pomdp.sim_maxxt))
        incrementSimXT(d.pomdp)
        return rand(rng, filter!(x -> x.c==true , iterator(d)))
    else
        return rand(rng, filter!(x -> x.c==false, iterator(d)))
    end
end
function POMDPs.rand(rng::AbstractRNG, d::ObsDistribution)
    if ((any(s -> s.d <= d.pomdp.sim_vdist, iterator(d))) && (d.pomdp.sim_xtime <= d.pomdp.sim_maxxt))
        return rand(rng, filter!(x -> x.c==true , iterator(d)))
    else
        return rand(rng, filter!(x -> x.c==false, iterator(d)))
    end
end

mutable struct PedXingBelief
    pomdp::PedXingPOMDP
    state_list::Vector{Bool} # {SIDEWALK, CROSSWALK}
    b::Vector{Float64}
    s::PedXingState # fully observed state
end
Base.length(b::PedXingBelief) = length(b.b)
function Base.fill!(b::PedXingBelief, x::Float64)
    fill!(b.b, x)
end
POMDPs.iterator(b::PedXingBelief) = b.state_list
Base.isequal(b1::PedXingBelief, b2::PedXingBelief) = b1.state_list == b2.state_list && b1.b == b2.b && b1.s == b2.s

function POMDPs.initialize_belief(pomdp::PedXingPOMDP, s::PedXingState) # uniform belief {SIDEWALK, CROSSWALK}
    return PedXingBelief(pomdp, [SIDEWALK, CROSSWALK], [0.5, 0.5], s)
end

mutable struct QMDPPolicy <: Policy
    alphas::Matrix{Float64} # matrix of alpha vectors |S|x|A|
    action_map::Vector{Any} # maps indices to actions
    pomdp::POMDP            # models for convenience
end
# default constructor
function QMDPPolicy(pomdp::POMDP)
    ns = n_states(pomdp)
    na = n_actions(pomdp)
    alphas = zeros(ns, na)
    am = Any[]
    space = ordered_actions(pomdp)
    for a in iterator(space)
        push!(am, a)
    end
    return QMDPPolicy(alphas, am, pomdp)
end

mutable struct PedXingUpdater <: Updater end
POMDPs.updater(p::QMDPPolicy) = PedXingUpdater()

function POMDPs.update(up::PedXingUpdater, bold::PedXingBelief, a::Float64, o::PedXingState)
    pomdp = bold.pomdp
    bnew = initialize_belief(pomdp, o) # {SIDEWALK, CROSSWALK}
    # ensure belief state sizes match
    @assert length(bold) == length(bnew)
    # initialize belief
    fill!(bnew, 0.0)
    # iterate through each state in belief vector
    for (i, spc) in enumerate(iterator(bnew))
        # create full state
        sp = PedXingState(o.v, o.d, spc, o.pv, o.aPrev)
        # get the distributions
        od = observation(pomdp, a, sp)
        # get prob of observation o from current distribution
        probo = pdf(od, o)
        # if observation prob is 0.0, then skip rest of update b/c bnew[i] is zero
        if probo == 0.0
            continue
        end
        b_sum = 0.0 # belief for state spc
        for (j, sc) in enumerate(iterator(bnew))
            s = PedXingState(o.v, o.d, sc, o.pv, o.aPrev)
            td = transition(pomdp, s, a)
            pp = pdf(td, sp)
            b_sum += pp * bold.b[j]
        end
        bnew.b[i] = probo * b_sum
    end
    norm = sum(bnew.b)
    # if norm is zero, the update was invalid - reset to uniform
    if norm == 0.0
        logerr(string("Invalid update for : ", bold.b, " ", a, " ", o))
        if isinteractive(); println("Invalid update for : ", bold.b, " ", a, " ", o); end
        u = 1.0/length(bnew)
        fill!(bnew, u)
    else
        for i = 1:length(bnew); bnew.b[i] /= norm; end
    end
    #loginfo(string("bnew: [", bnew.b, "]"))
    return bnew
end

function POMDPs.action(policy::QMDPPolicy, b::PedXingBelief)
    s = b.s
    pomdp = b.pomdp
    c0_ind = stateindex(pomdp, PedXingState(s.v, s.d, SIDEWALK , s.pv, s.aPrev))
    c1_ind = stateindex(pomdp, PedXingState(s.v, s.d, CROSSWALK, s.pv, s.aPrev))
    alphas = policy.alphas[[c0_ind,c1_ind],:]
    util = alphas'*b.b
    ihi = indmax(util)
    return policy.action_map[ihi]
end

println("Done.")

# verify model
include("checkSpaceIndexing.jl") # checks {state, action, observation} indexing between {SPACE}_index() and respective iterator()
function debugPedXingPOMDP(pomdp::PedXingPOMDP)
    checkStateSpace(pomdp)
    checkActionSpace(pomdp)
    checkObservationSpace(pomdp)

    println("\nChecking transition probabilities...")
    trans_prob_consistency_check(pomdp) # check the transition probabilities
    println("\nChecking observation probabilities...")
    obs_prob_consistency_check(pomdp) # checks the observation probabilities
    # probability_check(pomdp) # checks that both observation and transition functions give probs that sum to unity
    println("Probability check complete.")
end
