module StreetNetworks

using Graphs
using HDF5

using AutomotiveDrivingModels.Curves

include("RoadNetwork.jl")
using .RoadNetwork


# ===============================================================================

export TILE_WIDTH, TILE_HEIGHT, EASTING_BASE, NORTHING_BASE
export DEFAULT_LANE_WIDTH
export Pos3D, StreetNode, StreetLane, StreetSegment, NetworkTile, StreetNetwork
export lla2xyz, ll2utm, utm2tileindex_east, utm2tileindex_north, utm2tileindex
export tile_center_north, tile_center_east, tile_lowerbound_north, tile_lowerbound_east
export tile_upperbound_north, tile_upperbound_east, tile_bounds, tile_center
export get_tile, get_tile!, add_tile!, has_tile, same_tile
export has_segment, get_segment, get_segment!
export get_lane, has_lane, add_lane!, has_next_lane, next_lane, has_prev_lane, prev_lane
export next_lanetag, prev_lanetag
export next_node, prev_node, add_node
export same_lane, same_lane_and_tile
export closest_node_to_extind, closest_node_above_extind
export neighbor_north, neighbor_east, neighbor_south, neighbor_west
export has_neighbor_east, has_neighbor_south, has_neighbor_west, extisting_neighbor_tiles, extisting_neighbor_tiles_inclusive
export TilePoint2DProjectionResult
export project_point_to_tile, project_point_to_streetmap
export distance_to_lane_split, distance_to_lane_merge, distance_to_lane_end, distance_to_node
export frenet_distance_between_points
export num_lanes_on_sides, marker_distances
export rndf2streetnetwork, curves2streetnetwork
export curves2rndf

export LaneID, WaypointID, LaneTag

import Base: isequal, ==

# ===============================================================================

const TILE_WIDTH    = 250 # [m]
const TILE_HEIGHT   = 250 # [m]
const EASTING_BASE  = 0.0 # [m], the easting of the center of tile (0,0)
const NORTHING_BASE = 0.0 # [m], the northing of the center of tile (0,0)

const METERS_PER_FOOT = 0.3048 # [m/ft]

const DEFAULT_LANE_WIDTH = 3.7 # [m]
const DESIRED_DISTANCE_BETWEEN_CURVE_SAMPLES = 0.5 # [m]

const SATURATION_DISTANCE_TO_LANE_END   = 100.0 # [m]
const SATURATION_DISTANCE_TO_LANE_SPLIT = 100.0 # [m]
const SATURATION_DISTANCE_TO_LANE_MERGE = 100.0 # [m]
const SATURATION_DISTANCE_TO_NODE       = 100.0 # [m]
const SATURATION_DISTANCE_BETWEEN       = 500.0 # [m]
const THRESHOLD_DISTANCE_TO_NODE        = 500.0 # [m]

# ===============================================================================

immutable Pos3D
	x :: Float64
	y :: Float64
	z :: Float64

	Pos3D(x::Float64=0.0, y::Float64=0.0, z::Float64=0.0) = new(x,y,z)
end
type StreetNode 
	id :: WaypointID
	pos :: Pos3D # (UTM-e, UTM-n, alt [m])

	extind  :: Float64 # the extind of the node in its lane curve
	d_along :: Float64 # distance along the lane, 0.0 at source node [m]

	d_merge :: Float64 # distance to the next merge [m]
	d_split :: Float64 # distance to the next split [m]
	d_end   :: Float64 # distance to the lane end [m]

	n_lanes_left :: Int # number of lanes to the left (>= 0)
	n_lanes_right :: Int # number of lanes to the right (>= 0)

	marker_dist_left :: Float64 # estimated distance to left lane marker [m]
	marker_dist_right :: Float64 # estimated distance to right lane marker [m]

	function StreetNode(id::WaypointID, pos::Pos3D;
		extind ::Float64=NaN,
		d_along::Float64=NaN, 
		d_merge::Float64=NaN, 
		d_split::Float64=NaN,
		d_end  ::Float64=NaN,
		n_lanes_left::Int = -1,
		n_lanes_right::Int = -1,
		marker_dist_left::Float64=NaN,
		marker_dist_right::Float64=NaN,
		)

		self = new()
		self.id = id
		self.pos = pos
		self.extind = extind
		self.d_along = d_along
		self.d_merge = d_merge
		self.d_split = d_split
		self.d_end   = d_end
		self.n_lanes_left = n_lanes_left
		self.n_lanes_right = n_lanes_right
		self.marker_dist_left = marker_dist_left
		self.marker_dist_right = marker_dist_right
		self
	end
end
type StreetLane
	# an ordered list of centerline nodes comprising a lane
	id :: Int
	width :: Float64 # [m]
	boundary_left :: Symbol # TODO(tim): make this a LaneBoundary type?
	boundary_right :: Symbol # ∈ {:double_yellow, :solid_yellow, :solid_white, :broken_white, :unknown}
	nodes :: Vector{StreetNode} # only includes nodes in this lane
	curve :: Curve # this curve stretches from 1 node before source to 1 node after end, if applicable
	               # x = 0.0 occurs at source node and is measured positive along the centerline

	has_leading_node :: Bool # true iff curve includes a source node leading into this lane
	has_trailing_node :: Bool # true iff curve includes a destination node leaving this lane
end
type StreetSegment
	# an ordered list of lanes forming a single road with a common direction
	id :: Int  # > 0
	lanes :: Dict{Int, StreetLane}

	StreetSegment(id::Int, lanes::Dict{Int, StreetLane} = Dict{Int,StreetLane}()) = new(id, lanes)
end
type NetworkTile
	index_e :: Int
	index_n :: Int
	segments :: Dict{Int, StreetSegment} # maps segment id to segment

	function NetworkTile(
		index_e::Int, 
		index_n::Int, 
		segments::Dict{Int,StreetSegment} = Dict{Int, StreetSegment}()
		)

		new(index_e, index_n, segments)
	end
end
type StreetNetwork
	tile_dict :: Dict{(Int,Int),NetworkTile}
	graph :: Graph{StreetNode,Edge{StreetNode}}

	function StreetNetwork(
		tile_dict = Dict{(Int,Int),NetworkTile}(),
		graph::Graph{StreetNode,Edge{StreetNode}}=graph(StreetNode[],Edge{StreetNode}[])
		)
		new(tile_dict, graph)
	end
end

function Base.deepcopy(node::StreetNode)
	StreetNode(node.id, node.pos,
			   extind = node.extind,
			   d_along = node.d_along,
			   d_merge = node.d_merge,
			   d_split = node.d_split,
			   d_end   = node.d_end,
			   n_lanes_left = node.n_lanes_left,
			   n_lanes_right = node.n_lanes_right,
			   marker_dist_left = node.marker_dist_left,
			   marker_dist_right = node.marker_dist_right)
end
function Base.deepcopy(lane::StreetLane)
	StreetLane(
		lane.id,
		lane.width,
		lane.boundary_left,
		lane.boundary_right,
		deepcopy(lane.nodes),
		deepcopy(lane.curve),
		lane.has_leading_node,
		lane.has_trailing_node
		)
end
Base.deepcopy(seg::StreetSegment) = StreetSegment(seg.id, deepcopy(seg.lanes))
Base.deepcopy(tile::NetworkTile) = NetworkTile(tile.index_e, tile.index_n, deepcopy(tile.segments))

immutable LaneTag
	index_e :: Int
	index_n :: Int
	segment :: Int
	lane    :: Int

	LaneTag(index_e::Int, index_n::Int, segment::Int, lane::Int) = new(index_e, index_n, segment, lane)
	LaneTag(tile::NetworkTile, laneid::LaneID) = new(tile.index_e, tile.index_n, int(laneid.segment), int(laneid.lane))
	LaneTag(tile::NetworkTile, segid::Int, laneid::Int) = new(tile.index_e, tile.index_n, segid, laneid)
	function LaneTag(sn::StreetNetwork, lane::StreetLane)
		tile = get_tile(sn, lane.nodes[1].pos.x, lane.nodes[1].pos.y)
		tile_segments = collect(values(tile.segments))
		seg_ind = findfirst(s->in(lane, values(s.lanes)), tile_segments)
		seg = tile_segments[seg_ind]
		new(tile.index_e, tile.index_n, seg.id, lane.id)
	end
end
Base.convert(::Type{LaneID}, tag::LaneTag) = LaneID(tag.segment, tag.lane)

type TilePoint2DProjectionResult
	curvept    :: CurvePt
	extind     :: Float64
	tile       :: NetworkTile
	laneid     :: LaneID
	sqdist     :: Float64
	successful :: Bool

	function TilePoint2DProjectionResult()
		self = new()
		self.successful = false
		self
	end
	function TilePoint2DProjectionResult(
		curvept::CurvePt,
		extind::Float64,
		tile :: NetworkTile,
		laneid::LaneID,
		sqdist :: Float64,
		successful :: Bool
		)

		new(curvept, extind, tile, laneid, sqdist, successful)
	end
end

function Base.show(io::IO, proj::TilePoint2DProjectionResult)
	println(io, "TilePoint2DProjectionResult:")
	println(io, "\tcurvept:    ", proj.curvept)
	println(io, "\textind:     ", proj.extind)
	# println(io, "\ttile:       ", proj.tile)
	println(io, "\tlaneid:     ", proj.laneid)
	println(io, "\tsqdist:     ", proj.sqdist)
	println(io, "\tsuccessful: ", proj.successful)
end

==(A::LaneTag, B::LaneTag) = A.index_e == B.index_e && 
							 A.index_n == B.index_n &&
							 A.segment == B.segment &&
							 A.lane    == B.lane
isequal(A::LaneTag, B::LaneTag) = A.index_e == B.index_e && 
							      A.index_n == B.index_n &&
							      A.segment == B.segment &&
							      A.lane    == B.lane
same_tile(A::LaneTag, B::LaneTag) = A.index_e == B.index_e && A.index_n == B.index_n

# ===============================================================================

function lla2xyz( lat::Real, lon::Real, alt::Real )
	# convert a point in geo (lat, lon, alt) to a point in ECEX (x,y,z)

	# WGS84
	const a = 6378137.0 # equatorial radius [m]
	const b = 6356752.3142 # polar radius [m]
	const eccentricity = sqrt(1.0-(b/a)^2) # eccentricity

	RN = a / (1 - eccentricity^2*sin(lat)^2)^0.5

	x = (RN + alt)*cos(lat)*cos(lon)
	y = (RN + alt)*cos(lat)*sin(lon)
	z = (RN*(1-eccentricity^2)+alt)*sin(lat)

	return Vec3E(x,y,z)
end
function ll2utm( lat::Real, lon::Real, zone::Integer=-1; map_datum::Symbol=:WGS_84)
	# see DMATM 8358.2 by the Army
	# code verified using their samples

	if abs(lat) > pi/2 || abs(lon) > pi
		warn("ll2utm: lat and lon may not be in radians!")
	end

	# input latitude and longitude in radians
	while lon ≤ -π
		lon += 2π
	end
	while lon ≥ π
		lon -= 2π
	end

	const LATITUDE_LIMIT_NORTH = deg2rad(84)
	const LATITUDE_LIMIT_SOUTH = deg2rad(-80)
	if lat > LATITUDE_LIMIT_NORTH || lat < LATITUDE_LIMIT_SOUTH
		error("latitude $(rad2deg(lat)) is out of limits!")
	end


    if map_datum == :WGS_84
        const a = 6378137.0 # equatorial radius [m]
        const b = 6356752.314245 # polar radius [m]
    elseif map_datum == :INTERNATIONAL
        const a = 6378388.0
        const b = 6356911.94613
    else
        # default to WGS 84
        const a = 6378137.0
        const b = 6356752.314245
    end

	const FN = 0.0      # false northing, zero in the northern hemisphere
	const FE = 500000.0 # false easting

	const ko = 0.9996   # central scale factor

	zone_centers = -177.0*pi/180 + 6.0*pi/180*[0:59] # longitudes of the zone centers
	if zone == -1
		zone = indmin(map(x->abs(lon - x), zone_centers)) # index of min zone center
	end
	long0 = zone_centers[zone]

	s  = sin(lat)
	c  = cos(lat)
	t  = tan(lat)

	n  = (a-b)/(a+b)
	e₁  = sqrt((a^2-b^2)/a^2) # first eccentricity
	ep = sqrt((a^2-b^2)/b^2) # second eccentricity
	nu = a/(1-e₁^2*s^2)^0.5   # radius of curvature in the prime vertical
	dh = lon - long0         # longitudinal distance from central meridian

	A  = a*(1 - n + 5.0/4*(n^2-n^3) + 81.0/64*(n^4-n^5))
	B  = 3.0/2*a*(n-n^2 + 7.0/8*(n^3-n^4) + 55.0/64*n^5)
	C  = 15.0/16*a*(n^2-n^3 + 3.0/4*(n^4-n^5))
	D  = 35.0/48*a*(n^3-n^4 + 11.0/16.0*n^5)
	E  = 315.0/512*a*(n^4-n^5)
	S  = A*lat - B*sin(2*lat) + C*sin(4*lat) - D*sin(6*lat) + E*sin(8*lat)

	T1 = S*ko
	T2 = nu*s*c*ko/2
	T3 = nu*s*c^3*ko/24  * (5  -    t^2 + 9*ep^2*c^2 + 4*ep^4*c^4)
	T4 = nu*s*c^5*ko/720 * (61 - 58*t^2 + t^4 + 270*ep^2*c^2 - 330*(t*ep*c)^2
		                       + 445*ep^4*c^4 + 324*ep^6*c^6 - 680*t^2*ep^4*c^4
		                       + 88*ep^8*c^8  - 600*t^2*ep^6*c^6 - 192*t^2*ep^8*c^8)
	T5 = nu*s*c^7*ko/40320 * (1385 - 3111*t^2 + 543*t^4 - t^6)
	T6 = nu*c*ko
	T7 = nu*c^3*ko/6   * (1 - t^2 + ep^2*c^2)
	T8 = nu*c^5*ko/120 * (5 - 18*t^2 + t^4 + 14*ep^2*c^2 - 58*t^2*ep^2*c^2 + 13*ep^4*c^4
		                    + 4*ep^6*c^6 - 64*t^2*ep^4*c^4 - 24*t^2*ep^6*c^6)
	T9 = nu*c^7*ko/5040 * (61 - 479*t^2 + 179*t^4 - t^6)

	E = FE + (dh*T6 + dh^3*T7 + dh^5*T8 + dh^7*T9)        # easting  [m]
	N = FN + (T1 + dh^2*T2 + dh^4*T3 + dh^6*T4 + dh^8*T5) # northing [m]

    return (E,N,zone)
end

# NOTE(tim): returns the smallest in magnitude signed angle a must be rotated by to get to b
_signed_dist_btw_angles(a::Real, b::Real) = atan2(sin(b-a), cos(b-a))

utm2tileindex_east(utm_easting::Real) = int(round((utm_easting  - EASTING_BASE)  / TILE_WIDTH))
utm2tileindex_north(utm_northing::Real) = int(round((utm_northing - NORTHING_BASE) / TILE_HEIGHT))
utm2tileindex(utm_easting::Real, utm_northing::Real) = (utm2tileindex_east(utm_easting), utm2tileindex_north(utm_northing))

tile_center_north(index_n::Int)     =  index_n      * TILE_HEIGHT + NORTHING_BASE
tile_lowerbound_north(index_n::Int) = (index_n-0.5) * TILE_HEIGHT + NORTHING_BASE
tile_upperbound_north(index_n::Int) = (index_n+0.5) * TILE_HEIGHT + NORTHING_BASE
tile_center_east(index_e::Int)      =  index_e      * TILE_WIDTH  + EASTING_BASE
tile_lowerbound_east(index_e::Int)  = (index_e-0.5) * TILE_WIDTH  + EASTING_BASE
tile_upperbound_east(index_e::Int)  = (index_e+0.5) * TILE_WIDTH  + EASTING_BASE
function tile_bounds(tile::NetworkTile)
	index_e, index_n = tile.index_e, tile.index_n
	min_e = tile_lowerbound_east(index_e)
    max_e = tile_upperbound_east(index_e)
    min_n = tile_lowerbound_north(index_n)
    max_n = tile_upperbound_north(index_n)
    (min_e, max_e, min_n, max_n)
end
tile_center(tile::NetworkTile) = (tile_center_east(tile.index_e), tile_center_north(tile.index_n))

function get_tile(sn::StreetNetwork, index_e::Int, index_n::Int)
	key = (index_e, index_n)
	if !haskey(sn.tile_dict, key)
		error("tile not found: ", key)
	end
	retval = sn.tile_dict[key]
	@assert(retval.index_e == index_e && retval.index_n == index_n)
	retval
end
function get_tile(sn::StreetNetwork, easting::Float64, northing::Float64)
	index_e, index_n = utm2tileindex(easting, northing)
	get_tile(sn, index_e, index_n)	
end
get_tile(sn::StreetNetwork, node::StreetNode) = get_tile(sn, node.pos.x, node.pos.y)
get_tile(sn::StreetNetwork, tag::LaneTag) = get_tile(sn, tag.index_e, tag.index_n)
function get_tile!(sn::StreetNetwork, index_e::Int, index_n::Int)
	key = (index_e, index_n)
	if !haskey(sn.tile_dict, key)
		sn.tile_dict[key] = NetworkTile(index_e, index_n, Dict{Int, StreetSegment}())
	end
	retval = sn.tile_dict[key]
	@assert(retval.index_e == index_e && retval.index_n == index_n)
	retval
end
function get_tile!(sn::StreetNetwork, easting::Float64, northing::Float64)
	index_e, index_n = utm2tile(easting, northing)
	get_tile!(sn, index_e, index_n)	
end
function add_tile!(sn::StreetNetwork, tile::NetworkTile; override=false)
	key = (tile.index_e, tile.index_n)
	if haskey(sn.tile_dict, key) && !override
		error("tile already exists: ", key)
	end
	sn.tile_dict[key] = tile
	nothing
end
has_tile(sn::StreetNetwork, index_e::Int, index_n::Int) = haskey(sn.tile_dict, (index_e, index_n))

has_segment(tile::NetworkTile, segment_id::Integer) = haskey(tile.segments, segment_id)
function get_segment(tile::NetworkTile, segment_id::Int)
	if !haskey(tile.segments, segment_id)
		error("segment not found in tile")
	end
	retval = tile.segments[segment_id]
	@assert(retval.id == segment_id)
	retval
end
get_segment(sn::StreetNetwork, tag::LaneTag) = get_segment(get_tile(sn, tag), tag.segment)
function get_segment!(tile::NetworkTile, segment_id::Int)
	if !haskey(tile.segments, segment_id)
		segment = StreetSegment(segment_id, Dict{Int, StreetLane}())
		tile.segments[segment_id] = segment
		return segment
	end
	retval = tile.segments[segment_id]
	@assert(retval.id == segment_id)
	retval
end

has_lane(segment::StreetSegment, id::Integer) = haskey(segment.lanes, id)
has_lane(segment::StreetSegment, lane::StreetLane) = haskey(segment.lanes, lane.id)
has_lane(tile::NetworkTile, id::LaneID) = haskey(tile.segments, id.segment) && has_lane(tile.segments[id.segment], id.lane)
has_lane(tile::NetworkTile, id::WaypointID) = haskey(tile.segments, id.segment) && has_lane(tile.segments[id.segment], id.lane)
function has_lane(sn::StreetNetwork, tag::LaneTag)
	if has_tile(sn, tag.index_e, tag.index_n)
		tile = sn.tile_dict[(tag.index_e, tag.index_n)]
		return haskey(tile.segments, tag.segment) && has_lane(tile.segments[tag.segment], tag.lane)
	end
	return false
end
get_lane(tile::NetworkTile, id::LaneID) = tile.segments[id.segment].lanes[id.lane]
get_lane(tile::NetworkTile, id::WaypointID) = tile.segments[id.segment].lanes[id.lane]
get_lane(sn::StreetNetwork, tag::LaneTag) = sn.tile_dict[(tag.index_e, tag.index_n)].segments[tag.segment].lanes[tag.lane]
get_lane(sn::StreetNetwork, node::StreetNode) = get_lane(get_tile(sn, node.pos.x, node.pos.y), node.id)
get_lane(proj::TilePoint2DProjectionResult) = get_lane(proj.tile, proj.laneid)
function add_lane!(segment::StreetSegment, lane::StreetLane; override=false)
	if !override && haskey(segment.lanes, lane.id)
		error("segment already has lane: ", lane)
	end
	segment.lanes[lane.id] = lane
	nothing
end
function has_next_lane(sn::StreetNetwork, lane::StreetLane)
	if isempty(lane.nodes)
		return false
	end
	out_degree(lane.nodes[end], sn.graph) > 0
end
function next_lane(sn::StreetNetwork, lane::StreetLane)
	last_node = lane.nodes[end]
	next_node = _next_node(sn.graph, last_node)
	if next_node === last_node
		error("no next lane!")
	end
	get_lane(sn, next_node)
end
function next_lanetag(sn::StreetNetwork, lanetag::LaneTag)
	# retuns the current one if the next does not exist
	lane = get_lane(sn, lanetag)
	if has_next_lane(sn, lane)
		return LaneTag(sn, next_lane(sn, lane))
	end
	return lanetag
end
has_prev_lane(sn::StreetNetwork, lane::StreetLane) = !isempty(lane.nodes) && in_degree(lane.nodes[1], sn.graph) > 0
function prev_lane(sn::StreetNetwork, lane::StreetLane)
	first_node = lane.nodes[1]
	prev_node = _prev_node(sn.graph, first_node)
	if prev_node === first_node
		error("no previous lane!")
	end
	get_lane(sn, prev_node)
end
function prev_lanetag(sn::StreetNetwork, lanetag::LaneTag)
	# retuns the current one if the prev does not exist
	lane = get_lane(sn, lanetag)
	if has_prev_lane(sn, lane)
		return LaneTag(sn, prev_lane(sn, lane))
	end
	return lanetag
end

function _next_node(g::Graph{StreetNode,Edge{StreetNode}}, node::StreetNode)
	# given the node, find the next one if there is one
	# if there are none, return node
	# if there are multiple, pick the one in the same lane
	# if there are multiple by all in different lanes, pick the first one given by out_neighbors

	neighbors = out_neighbors(node, g)
	if isempty(neighbors)
		return node
	end

	laneid = node.id.lane
	segid  = node.id.segment

	best_node = node
	found_new = false
	for n in neighbors
		if !found_new || (n.id.lane == laneid && n.id.segment == segid)
			best_node = n
			found_new = true
		end
	end

	return best_node
end
next_node(sn::StreetNetwork, node::StreetNode) = _next_node(sn.graph, node)
function _prev_node(g::Graph{StreetNode,Edge{StreetNode}}, node::StreetNode)
	# given the node, find the previous one if there is one
	# if there are none, return node
	# if there are multiple, pick the one in the same lane
	# if there are multiple by all in different lanes, pick the first one given by out_neighbors

	neighbors = collect(in_neighbors(node, g))
	if isempty(neighbors)
		return node
	end

	laneid = node.id.lane
	best_node = node
	for n in neighbors
		if best_node === node || n.id.lane == laneid
			best_node = n
		end
	end

	return best_node
end
prev_node(sn::StreetNetwork, node::StreetNode) = _prev_node(sn.graph, node)

same_lane(a::StreetNode, b::StreetNode) = a.id.lane == b.id.lane && a.id.segment == b.id.segment
same_tile(a::StreetNode, b::StreetNode) = utm2tileindex_east(a.pos.x)  == utm2tileindex_east(b.pos.x) &&
										  utm2tileindex_north(a.pos.y) == utm2tileindex_north(b.pos.y)
same_lane_and_tile(a::StreetNode, b::StreetNode) = same_lane(a,b) && same_tile(a,b)

add_node(sn::StreetNetwork, pos::Pos3D) = add_vertex!(sn.graph, pos)
function closest_node_to_extind(lane::StreetLane, extind::Float64; sq_dist_threshold = 0.1)
	
	#=
	Do a bisection search to get the closest node to the given extind
	=#

	a = 1
	b = length(lane.nodes)

	sqdist_a = (lane.nodes[a].extind - extind)*(lane.nodes[a].extind - extind)
	sqdist_b = (lane.nodes[b].extind - extind)*(lane.nodes[b].extind - extind)

	if b == a
		return lane.nodes[a]
	elseif b == a + 1
		return sqdist_b < sqdist_a ? lane.nodes[b] : lane.nodes[a]
	end

	c = div(a+b, 2)
	sqdist_c = (lane.nodes[c].extind - extind)*(lane.nodes[c].extind - extind)

	n = 1
	while true
		if b == a
			return lane.nodes[a]
		elseif b == a + 1
			return sqdist_b < sqdist_a ? lane.nodes[b]: lane.nodes[a]
		elseif c == a + 1 && c == b - 1
			if sqdist_a < sqdist_b && sqdist_a < sqdist_c
				return lane.nodes[a]
			elseif sqdist_b < sqdist_a && sqdist_b < sqdist_c
				return lane.nodes[b]
			else
				return lane.nodes[c]
			end
		end

		left = div(a+c, 2)
		sqdist_l = (lane.nodes[left].extind - extind)*(lane.nodes[left].extind - extind)

		if sqdist_l < sq_dist_threshold
			return lane.nodes[left]
		end

		right = div(c+b, 2)
		sqdist_r = (lane.nodes[right].extind - extind)*(lane.nodes[right].extind - extind)

		if sqdist_r < sq_dist_threshold
			return lane.nodes[right]
		elseif sqdist_l < sqdist_r
			b = c
			c = left
		else
			a = c
			c = right
		end
	end

	error("invalid codepath")

	# OLD LINEAR SEARCH CODE
	# best_ind = 0
	# best_diff = Inf
	# for (i,n) in enumerate(lane.nodes)
	# 	Δ = abs(n.extind - extind)
	# 	if Δ < best_diff
	# 		best_diff = Δ
	# 		best_ind = i
	# 	end
	# end
	# return lane.nodes[best_ind]
end
function closest_node_above_extind(sn::StreetNetwork, lane::StreetLane, extind::Float64; isapprox_threshold::Float64=0.1)
	# do a linear search
	# NOTE(tim): returns last node if it is a dead-end

	# TODO(tim): implement bisection search

	if extind > lane.nodes[end].extind
		node = lane.nodes[end]
		return _next_node(sn.graph, node) 
	end

	a = 1
	b = length(lane.nodes)

	n = 1
	while true
		if b == a || b == a+1
			return lane.nodes[b]
		end

		c = div( a+b, 2 )
		fc = lane.nodes[c].extind - extind

		if 0 ≤ fc < isapprox_threshold
			return lane.nodes[c]
		elseif fc < 0.0
			a = c
		else
			b = c
		end
	end

	error("invalid codepath")

	# best_ind = 0
	# best_diff = Inf
	# for (i,n) in enumerate(lane.nodes)
	# 	Δ = n.extind - extind
	# 	if 0.0 ≤ Δ < best_diff
	# 		best_diff = Δ
	# 		best_ind = i
	# 	end
	# end
	# return lane.nodes[best_ind]
end

neighbor_north(sn::StreetNetwork, tile::NetworkTile) = get_tile(sn, tile.index_e,   tile.index_n+1)
neighbor_east(sn::StreetNetwork,  tile::NetworkTile) = get_tile(sn, tile.index_e+1, tile.index_n  )
neighbor_south(sn::StreetNetwork, tile::NetworkTile) = get_tile(sn, tile.index_e,   tile.index_n-1)
neighbor_west(sn::StreetNetwork,  tile::NetworkTile) = get_tile(sn, tile.index_e-1, tile.index_n  )
has_neighbor_north(sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e, tile.index_n+1)
has_neighbor_east( sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e+1, tile.index_n)
has_neighbor_south(sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e, tile.index_n-1)
has_neighbor_west( sn::StreetNetwork, tile::NetworkTile) = has_tile(sn, tile.index_e-1, tile.index_n)
function extisting_neighbor_tiles(sn::StreetNetwork, tile::NetworkTile)
	retval = Array(NetworkTile, 8)
	total = 0

	del_x = [-1, 0, 1,-1, 1,-1, 0, 1]
	del_y = [-1,-1,-1, 0, 0, 1, 1, 1]

	for i = 1 : 8
		dx, dy = del_x[i], del_y[i]
		if has_tile(sn, tile.index_e + dx, tile.index_n + dy)
			retval[total+=1] = get_tile(sn, tile.index_e + dx, tile.index_n + dy)
		end	
	end

	return retval[1:total]
end
function extisting_neighbor_tiles_inclusive(sn::StreetNetwork, tile::NetworkTile)
	retval = Array(NetworkTile, 9)
	retval[1] = tile
	total = 1

	del_x = [-1, 0, 1,-1, 1,-1, 0, 1]
	del_y = [-1,-1,-1, 0, 0, 1, 1, 1]

	for i = 1 : 8
		dx, dy = del_x[i], del_y[i]
		if has_tile(sn, tile.index_e + dx, tile.index_n + dy)
			retval[total+=1] = get_tile(sn, tile.index_e + dx, tile.index_n + dy)
		end	
	end

	return retval[1:total]
end

function project_point_to_tile(easting::Float64, northing::Float64, tile::NetworkTile )

	min_square_dist = Inf
	best_segment_id = -1
	best_lane_id = -1
	res_point = CurvePt(NaN,NaN,NaN,NaN,NaN,NaN)
	successful = false
	best_extind = -2

	for (segment_id, segment) in tile.segments
		for (lane_id, lane) in segment.lanes
			extind = Curves.closest_point_extind_to_curve(lane.curve, easting, northing)
			pt = Curves.curve_at(lane.curve, extind)
			de = pt.x - easting
			dn = pt.y - northing
			square_dist = de*de + dn*dn
			if square_dist < min_square_dist
				min_square_dist = square_dist
				best_segment_id = segment_id
				best_lane_id = lane_id
				res_point = pt
				successful = true
				best_extind = extind
			end
		end
	end

	if successful 
		return TilePoint2DProjectionResult(res_point, best_extind, tile, LaneID(best_segment_id, best_lane_id), 
											min_square_dist, successful)
	end
	TilePoint2DProjectionResult()
end
function project_point_to_tile(easting::Float64, northing::Float64, tile::NetworkTile, f_filter::Function)

	min_square_dist = Inf
	best_segment_id = -1
	best_lane_id = -1
	res_point = Float64[]
	successful = false
	best_extind = -2

	for (segment_id, segment) in tile.segments
		for (lane_id, lane) in segment.lanes
			extind = Curves.closest_point_extind_to_curve(lane.curve, easting, northing)
			pt = Curves.curve_at(lane.curve, extind)

			if f_filter(pt, lane)
				de = pt.x - easting
				dn = pt.y - northing
				square_dist = de*de + dn*dn
				if square_dist < min_square_dist
					min_square_dist = square_dist
					best_segment_id = segment_id
					best_lane_id = lane_id
					res_point = pt
					successful = true
					best_extind = extind
				end
			end
		end
	end

	if successful 
		return TilePoint2DProjectionResult(res_point, best_extind, tile, LaneID(best_segment_id, best_lane_id), 
											min_square_dist, successful)
	end
	TilePoint2DProjectionResult()
end
function project_point_to_streetmap(easting::Float64, northing::Float64, sn::StreetNetwork)

	index_e, index_n = utm2tileindex(easting, northing)

	tile = first(sn.tile_dict)[2]
	proj = TilePoint2DProjectionResult()

	if has_tile(sn, index_e, index_n)
		tile = get_tile(sn, index_e, index_n)
		proj = project_point_to_tile(easting, northing, tile)
	end
		
	if !proj.successful || proj.sqdist > 1.5
		# NOTE(tim): if were are already close to a centerline on this tile then
		# no need to do extra work to check neighboring tiles

		min_e, max_e, min_n, max_n = tile_bounds(tile)

		d_e_lo = easting - min_e
		d_e_hi = max_e - easting
		d_n_lo = northing - min_n
		d_n_hi = max_n - northing

		d_e_lo_sq = d_e_lo * d_e_lo
		d_e_hi_sq = d_e_hi * d_e_hi
		d_n_lo_sq = d_n_lo * d_n_lo
		d_n_hi_sq = d_n_hi * d_n_hi

		border_sq_distances = [d_e_lo_sq, d_e_hi_sq, d_n_lo_sq, d_n_hi_sq,
		                       d_e_lo_sq + d_n_lo_sq, d_e_lo_sq + d_n_hi_sq,
		                       d_e_hi_sq + d_n_lo_sq, d_e_hi_sq + d_n_hi_sq]
		borders          = [(-1, 0), ( 1,0), (0,-1), (0,1),
		                    (-1,-1), (-1,1), (1,-1), (1,1)]
		p = sortperm(border_sq_distances)
		permute!(border_sq_distances, p)
		permute!(borders, p)

		best_sqdist = proj.successful ? proj.sqdist : Inf
		i = 1
		while i ≤ 8 && best_sqdist > border_sq_distances[i]
			b = borders[i]
			# println("checking $(b[1]), $(b[2])")
			if has_tile(sn, index_e+b[1], index_n+b[2])
				proj2 = project_point_to_tile(easting, northing, get_tile(sn, index_e+b[1], index_n+b[2]))
				if proj2.sqdist < best_sqdist
					best_sqdist, proj = proj2.sqdist, proj2
				end
			end
			i += 1
		end
	end

	return proj
end
function project_point_to_streetmap(easting::Float64, northing::Float64, sn::StreetNetwork, f_filter::Function)

	index_e, index_n = utm2tileindex(easting, northing)

	tile = first(sn.tile_dict)[2]
	proj = TilePoint2DProjectionResult()

	if has_tile(sn, index_e, index_n)
		tile = get_tile(sn, index_e, index_n)
		proj = project_point_to_tile(easting, northing, tile, f_filter)
	end
		
	if !proj.successful || proj.sqdist > 1.5
		# NOTE(tim): if were are already close to a centerline on this tile then
		# no need to do extra work to check neighboring tiles

		min_e, max_e, min_n, max_n = tile_bounds(tile)

		d_e_lo = easting - min_e
		d_e_hi = max_e - easting
		d_n_lo = northing - min_n
		d_n_hi = max_n - northing

		d_e_lo_sq = d_e_lo * d_e_lo
		d_e_hi_sq = d_e_hi * d_e_hi
		d_n_lo_sq = d_n_lo * d_n_lo
		d_n_hi_sq = d_n_hi * d_n_hi

		border_sq_distances = [d_e_lo_sq, d_e_hi_sq, d_n_lo_sq, d_n_hi_sq,
		                       d_e_lo_sq + d_n_lo_sq, d_e_lo_sq + d_n_hi_sq,
		                       d_e_hi_sq + d_n_lo_sq, d_e_hi_sq + d_n_hi_sq]
		borders          = [(-1, 0), ( 1,0), (0,-1), (0,1),
		                    (-1,-1), (-1,1), (1,-1), (1,1)]
		p = sortperm(border_sq_distances)
		permute!(border_sq_distances, p)
		permute!(borders, p)

		best_sqdist = proj.successful ? proj.sqdist : Inf
		i = 1
		while i ≤ 8 && best_sqdist > border_sq_distances[i]
			b = borders[i]
			# println("checking $(b[1]), $(b[2])")
			if has_tile(sn, index_e+b[1], index_n+b[2])
				proj2 = project_point_to_tile(easting, northing, get_tile(sn, index_e+b[1], index_n+b[2]), f_filter)
				if proj2.sqdist < best_sqdist
					best_sqdist, proj = proj2.sqdist, proj2
				end
			end
			i += 1
		end
	end

	return proj
end

function _distance_to_lane_merge(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_MERGE # [m]
	)

	# NOTE(tim): follow the nodes until we get to one that has two incoming edges
	# follows next_node()
	# returns Inf if the lane ends

	lane = segment.lanes[lane_index]
	node = closest_node_above_extind(sn, lane, extind)
	dist = node.d_along - curve_at(lane.curve, extind).s

	while in_degree(node, sn.graph) < 2

	    if out_degree(node, sn.graph) == 0
	        return max_dist # no merge
	    end
	    
	    nextnode = next_node(sn, node)
	    if same_lane_and_tile(node, nextnode)
	        dist += nextnode.d_along - node.d_along
	    else
	    	# NOTE(tim): presumably the curve ends with the next node, so this is valid
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	if !(lane.curve.s[end] - node.d_along ≥ 0.0)
	    		println(lane.curve.s[end], "  ", node.d_along, "  ", node.id, "  ", nextnode.id)
	    	end
	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end
	    node = nextnode
	    if dist > max_dist
	    	return max_dist
	    end
	end
	
	return min(dist, max_dist)
end
function _distance_to_lane_split(
    sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_SPLIT # [m]
	)

	# NOTE(tim): follow the nodes until we get to one that has two outgoing edges
	# follows next_node()
	# returns Inf if the lane ends

	lane = segment.lanes[lane_index]
	node = closest_node_above_extind(sn, lane, extind)
	dist = node.d_along - curve_at(lane.curve, extind).s

	while out_degree(node, sn.graph) < 2

	    if out_degree(node, sn.graph) == 0
	        return max_dist
	    end
	    
	    nextnode = next_node(sn, node)
	    if same_lane_and_tile(node, nextnode)
	        dist += nextnode.d_along - node.d_along
	    else
	    	# NOTE(tim): presumably the curve ends with the next node, so this is valid
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	if !(lane.curve.s[end] - node.d_along ≥ 0.0)
	    		println(lane.curve.s[end], "  ", node.d_along, "  ", node.id, "  ", nextnode.id)
	    	end
	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end
	    node = nextnode
	    if dist > max_dist
	    	return max_dist
	    end
	end
	
	return min(dist, max_dist)
end
function _distance_to_lane_end(
    sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_END # [m]
	)

	# NOTE(tim): follow the nodes until we get to the end
	# follows next_node()
	# returns Inf if it gets back to a node is has previously seen (loop)

	lane = segment.lanes[lane_index]
	firstnode = node = closest_node_above_extind(sn, lane, extind)
	dist = node.d_along - curve_at(lane.curve, extind).s

	while out_degree(node, sn.graph) != 0
	    
	    nextnode = next_node(sn, node)
	    if nextnode === firstnode
	    	return max_dist
	    end

	    if same_lane_and_tile(node, nextnode)
	        dist += nextnode.d_along - node.d_along
	    else
	    	# NOTE(tim): presumably the curve ends with the next node, so this is valid
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	if !(lane.curve.s[end] - node.d_along ≥ 0.0)
	    		println(lane.curve.s[end], "  ", node.d_along, "  ", node.id, "  ", nextnode.id)
	    	end
	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end
	    node = nextnode

	    if dist > max_dist
	    	return max_dist
	    end
	end
	
	min(dist, max_dist)
end

function distance_to_lane_merge(
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
	max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_MERGE
	)

	lane = segment.lanes[lane_index]
	node = closest_node_to_extind(lane, extind)
	if node.extind > extind
        min(node.d_along - curve_at(lane.curve, extind).s + node.d_merge, max_dist)
    else
        min(curve_at(lane.curve, extind).s - node.d_along + node.d_merge, max_dist)
    end
end
function distance_to_lane_split(
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
    max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_SPLIT
	)

	lane = segment.lanes[lane_index]
	node = closest_node_to_extind(lane, extind)
    if node.extind > extind
        min(node.d_along - curve_at(lane.curve, extind).s + node.d_split, max_dist)
    else
        min(curve_at(lane.curve, extind).s - node.d_along + node.d_split, max_dist)
    end
end
function distance_to_lane_end(
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64;
    max_dist   :: Float64 = SATURATION_DISTANCE_TO_LANE_END
	)

	lane = segment.lanes[lane_index]
	node = closest_node_to_extind(lane, extind)
    if node.extind > extind
        min(node.d_along - curve_at(lane.curve, extind).s + node.d_end, max_dist)
    else
        min(curve_at(lane.curve, extind).s - node.d_along + node.d_end, max_dist)
    end
end
function distance_to_node(
	sn         :: StreetNetwork,
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64,
	target     :: StreetNode;
	max_dist   :: Float64 = THRESHOLD_DISTANCE_TO_NODE # [m]
	)

	# TODO(tim): currently only follows primary search path
	#            how should we handle offramps?

	lane = segment.lanes[lane_index]
	node = closest_node_to_extind(lane, extind)
	dist = node.d_along - curve_at(lane.curve, extind).s

	if hypot(node.pos.x - target.pos.x, node.pos.y - target.pos.y) > max_dist
		# NOTE(tim): if out of circular range cannot possibly be within max_dist
		return max_dist
	end

	if node === target
		return dist ≥ 0.0 ? dist : max_dist
	end

	while out_degree(node, sn.graph) > 0
	    
	    nextnode = next_node(sn, node)
	    if same_lane_and_tile(node, nextnode)
	        dist += nextnode.d_along - node.d_along
	    else
	    	tile = get_tile(sn, node)
	    	@assert(has_lane(tile, node.id))
	    	lane = get_lane(tile, node.id)

	    	@assert(lane.curve.s[end] - node.d_along ≥ 0.0)
	    	dist += lane.curve.s[end] - node.d_along
	    end
	    node = nextnode
	    if dist > max_dist
	    	return max_dist
	    end

	    if nextnode === target
	    	return dist
	    end
	end
	
	return max_dist
end
function frenet_distance_between_points(
	sn         :: StreetNetwork,
	A_east     :: Float64,
	A_north    :: Float64,
	B_east     :: Float64,
	B_north    :: Float64,
	max_dist   :: Float64 = SATURATION_DISTANCE_BETWEEN # [m]
	)

	# NOTE(tim): only works for B downstream from A

	# find closest point to start
	# run along lane curve until we can project B to the same curve

	projA = project_point_to_streetmap(A_east, A_north, sn)
	if projA.successful

		A_s, A_d = Curves.pt_to_frenet_xy(projA.curvept, A_east, A_north)

		# println("A: ", (A_s, A_d))

		lane = get_lane(projA)
		search_dist = 0.0
		finished = false

		# println(search_dist, "  ", lane.id, "  ", finished)

		while !finished

			extind = Curves.closest_point_extind_to_curve(lane.curve, B_east, B_north)
			# println("extind: ", extind)
			# println("s_length: ", s_length)
			# println(lane.curve.x[1])
			# println(lane.curve.y[1])
			# println(lane.curve.x[end])
			# println(lane.curve.y[end])
			if !is_extind_at_curve_end(lane.curve, extind)

				# println("\t1")

				ptC = Curves.curve_at(lane.curve, extind)
				s, d = Curves.pt_to_frenet_xy(ptC, B_east, B_north)
				Δs = s - A_s + search_dist
				Δd = d - A_d
				return (Δs, Δd)
			elseif has_next_lane(sn, lane)
				search_dist += lane.curve.s[end]
				lane = next_lane(sn, lane)
				finished = search_dist > max_dist
				# println("\t2: ", search_dist, "  ", lane.id, "  ", finished)
			else
				# println("\t3")
				finished = true
			end
		end
	end

	(NaN, NaN)
end

function num_lanes_on_sides(
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64
	)

	lane = segment.lanes[lane_index]
	node = closest_node_to_extind(lane, extind)

	@assert(node.n_lanes_left ≥ 0)
	@assert(node.n_lanes_right ≥ 0)

	(node.n_lanes_left, node.n_lanes_right)
end
function marker_distances(
	segment    :: StreetSegment,
	lane_index :: Int,
	extind     :: Float64
	)

	lane = segment.lanes[lane_index]
	node = closest_node_to_extind(lane, extind)
	(node.marker_dist_left, node.marker_dist_right)
end

function rndf2streetnetwork(rndf::RNDF; verbosity::Int=0, convert_ll2utm::Bool=true)
	# convert the given rndf to a street network

	verbosity < 1 || println("RNDF 2 STREET NETWORK")
	starttime = time()

	sn = StreetNetwork()
	G = sn.graph

	const EDGE_DIST_THRESHOLD = 50.0

	node_map = Dict{WaypointID, StreetNode}()
	lane_ids = Dict{LaneID, Vector{Int}}()

	# build the road network graph
	verbosity < 1 || @printf("BUILDING GRAPH [%.2f]\n", time()-starttime)
	for segment in values(rndf.segments)
		for lane in values(segment.lanes)
			@assert(lane.id.segment == segment.id)
			ids = sort!(collect(keys(lane.waypoints))::Vector{Int})
			for (waypoint_index,lla) in lane.waypoints
				lat, lon = lla.lat, lla.lon
				if convert_ll2utm
					eas, nor = ll2utm(deg2rad(lat), deg2rad(lon))
				else
					eas, nor = lat, lon
				end
				pos = Pos3D(eas,nor,0.0)
				node = StreetNode(WaypointID(segment.id, lane.id.lane, waypoint_index), pos)
				node_map[WaypointID(segment.id, lane.id.lane, waypoint_index)] = node
				add_vertex!(G, node)
			end

			# starting with a root node, find the next closest node to either end of the chain
			# to build the path
			root_bot = 1 # [index in ids]
			root_top = 1 # [index in ids]
			ids_yet_to_take = Set([2:length(ids)])
			new_order = Int[1]
			while !isempty(ids_yet_to_take)
				closest_node_to_bot = -1
				best_dist_bot = Inf
				closest_node_to_top = -1
				best_dist_top = Inf

				nodeT = node_map[WaypointID(segment.id, lane.id.lane, ids[root_top])]
				nodeB = node_map[WaypointID(segment.id, lane.id.lane, ids[root_bot])]
				Tn, Te = nodeT.pos.x, nodeT.pos.y
				Bn, Be = nodeB.pos.x, nodeB.pos.y

				for id in ids_yet_to_take
					node = node_map[WaypointID(segment.id, lane.id.lane, ids[id])]
					index_n, index_e = node.pos.x, node.pos.y
					dnT, deT = Tn-index_n, Te-index_e
					dnB, deB = Bn-index_n, Be-index_e
					distT = sqrt(dnT*dnT + deT*deT)
					distB = sqrt(dnB*dnB + deB*deB)
					if distT < best_dist_top
						best_dist_top, closest_node_to_top = distT, id
					end
					if distB < best_dist_bot
						best_dist_bot, closest_node_to_bot = distB, id
					end
				end

				@assert(min(best_dist_top, best_dist_bot) < EDGE_DIST_THRESHOLD)
				if best_dist_bot < best_dist_top
					# node = node_map[WaypointID(segment.id, lane.id.lane, ids[closest_node_to_bot])]
					# add_edge!(G, node, nodeB)
					root_bot = closest_node_to_bot
					delete!(ids_yet_to_take, root_bot)
					unshift!(new_order, closest_node_to_bot)
				else
					# node = node_map[WaypointID(segment.id, lane.id.lane, ids[closest_node_to_top])]
					# add_edge!(G, nodeT, node)
					root_top = closest_node_to_top
					delete!(ids_yet_to_take, root_top)
					push!(new_order, closest_node_to_top)
				end
			end

			# infer direction from whether ids are generally increasing or decreasing
			ids_generally_increasing = sum([(ids[new_order[i]] - ids[new_order[i-1]] > 0.0) for i in 2:length(ids)]) > 0.0
			node = node_map[WaypointID(segment.id, lane.id.lane, ids[new_order[1]])]

			local i = 2
			while i ≤ length(new_order)
				nextnode = node_map[WaypointID(segment.id, lane.id.lane, ids[new_order[i]])]
				ids_generally_increasing ? add_edge!(G, node, nextnode) : add_edge!(G, nextnode, node)
				node = nextnode
				i += 1
			end

			# if this is a cycle - connect it
			# TODO(tim): use direction to infer whether this is correct
			const CYCLE_CONNECTION_THRESHOLD = 6.25 # [m]
			nodeT = node_map[WaypointID(segment.id, lane.id.lane, ids[root_top])]
			nodeB = node_map[WaypointID(segment.id, lane.id.lane, ids[root_bot])]
			if dist(nodeT, nodeB) < CYCLE_CONNECTION_THRESHOLD
				if ids_generally_increasing
					add_edge!(G, nodeT, nodeB)
				else
					add_edge!(G, nodeB, nodeT)
				end
			end

			@assert(length(new_order) == length(ids))
			lane_ids[lane.id] = ids[new_order]
		end
	end

	# add exit edges
	verbosity < 1 || @printf("ADDING EXIT EDGES [%.2f]\n", time()-starttime)
	for segment in values(rndf.segments)
		for lane in values(segment.lanes)
			for (exitnode, entry) in lane.exits
				exit_waypoint = WaypointID(segment.id, lane.id.lane, exitnode)
				if haskey(node_map, exit_waypoint)
					add_edge!(G, node_map[exit_waypoint], node_map[entry])
				end
			end
		end
	end

	# add nodes to tiles
	verbosity < 1 || @printf("ADDING NODES TO TILES [%.2f]\n", time()-starttime)
	for segment in values(rndf.segments)
		for lane in values(segment.lanes)
			# keep the curve in the tile
			ids = lane_ids[lane.id]

			first_node_index = 1
			first_node = node_map[WaypointID(segment.id, lane.id.lane, ids[first_node_index])]
			index_e, index_n = utm2tileindex(first_node.pos.x, first_node.pos.y)
			tile = get_tile!(sn, index_e, index_n)
			n_pts_in_lane = length(lane.waypoints)
			@assert(n_pts_in_lane == length(ids))
			
			while first_node_index ≤ n_pts_in_lane
				# find the last node in this lane that is still in the tile
				final_node_index = findfirst(index->begin
													node = node_map[WaypointID(segment.id, lane.id.lane, ids[index])]
													node_e, node_n = utm2tileindex(node.pos.x, node.pos.y)
													return (index_e != node_e) || (index_n != node_n)
												end, 
											[(first_node_index+1) : n_pts_in_lane])
				if final_node_index == 0
					final_node_index = n_pts_in_lane
				else
					final_node_index += first_node_index-1
				end

				nodes = map(index->node_map[WaypointID(segment.id, lane.id.lane, ids[index])], [first_node_index:final_node_index])

				streetsegment = get_segment!(tile, segment.id)

				if has_lane(streetsegment, lane.id.lane)
					# need to stitch lanes together
					streetlane = streetsegment.lanes[lane.id.lane]

					if dist(nodes[end], streetlane.nodes[1]) < dist(streetlane.nodes[end], nodes[1])
						streetlane.nodes = [nodes, streetlane.nodes]
					else
						streetlane.nodes = [streetlane.nodes, nodes]
					end
				else
					width = isnan(lane.width) ? DEFAULT_LANE_WIDTH : lane.width*METERS_PER_FOOT
					streetlane = StreetLane(lane.id.lane, width, lane.boundary_left, lane.boundary_right, nodes, Curve(), false, false)
					add_lane!(streetsegment, streetlane)
				end

				first_node_index = final_node_index+1
				if first_node_index ≤ n_pts_in_lane
					node = node_map[WaypointID(segment.id, lane.id.lane, ids[first_node_index])]
					index_e, index_n = utm2tileindex(node.pos.x, node.pos.y)
					tile = get_tile!(sn, index_e, index_n)
				end
			end
		end
	end

	# attempt to connect hanging edges
	verbosity < 1 || @printf("CONNECTING HANGING EDGES [%.2f]\n", time()-starttime)

	const THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT = 30.0 # [m]
	const THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE  = deg2rad(20) # [rad]
	const THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 = deg2rad(5) # [rad]

	for node in vertices(sn.graph)
		if out_degree(node, sn.graph) == 0 && in_degree(node, sn.graph) > 0
			# NOTE(tim): this is an ending node
			prevnode = _prev_node(sn.graph, node)
			θ = atan2(node.pos.y - prevnode.pos.y, node.pos.x - prevnode.pos.x)
			E = [node.pos.x, node.pos.y]
			vE = [cos(θ), sin(θ)]

			best_node = node
			best_score = Inf

			tile = get_tile(sn, node)
			for seg in values(tile.segments)
				for lane in values(seg.lanes)
					if (seg.id == node.id.segment && lane.id == node.id.lane) || length(lane.nodes) < 2
						continue
					end

					for (i,testnode) in enumerate(lane.nodes)

						θ2 = i > 1 ? atan2(testnode.pos.y - lane.nodes[i-1].pos.y, testnode.pos.x - lane.nodes[i-1].pos.x) :
									 atan2(lane.nodes[i+1].pos.y - testnode.pos.y, lane.nodes[i+1].pos.x - testnode.pos.x)
						vT = [cos(θ2), sin(θ2)]
						T = [testnode.pos.x, testnode.pos.y]
						A = T - E

						c_p = dot(A,vE)
						if c_p > 0.0
							proj = copy(vE).*c_p
							perp_dist = norm(A - proj,2)
							line_dist = norm(proj)
							# NOTE(tim): angle between start and end orientation							
							Δθ = abs(_signed_dist_btw_angles(θ,θ2))

							# NOTE(tim): angle between dangling edge and projection
							θ3 = atan2(testnode.pos.y - node.pos.y, testnode.pos.x - node.pos.x)
							Δ3 = abs(_signed_dist_btw_angles(θ,θ3))

							score = Δθ + 10.0*Δ3
							
							if line_dist < THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT &&
								Δθ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE && 
								Δ3 < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 &&
								score < best_score

								best_score = score
								best_node = testnode
							end
						end
					end
				end
			end

			if !(node === best_node)
				verbosity > 1 || println("ADDED END NODE HANGING EDGE ", node.pos.x, "  ", node.pos.y)
				add_edge!(G, node, best_node)
			end
		end
		if in_degree(node, sn.graph) == 0 && out_degree(node, sn.graph) > 0
			# NOTE(tim): this is a starting node
			nextnode = _next_node(sn.graph, node)
			θ = atan2(nextnode.pos.y - node.pos.y, nextnode.pos.x - node.pos.x)
			E = [node.pos.x, node.pos.y]
			vE = -[cos(θ), sin(θ)]

			best_node = node
			best_perp_dist = Inf

			tile = get_tile(sn, node)
			for seg in values(tile.segments)
				for lane in values(seg.lanes)
					if (seg.id == node.id.segment && lane.id == node.id.lane) || length(lane.nodes) < 2
						continue
					end

					for (i,testnode) in enumerate(lane.nodes)

						θ2 = i > 1 ? atan2(testnode.pos.y - lane.nodes[i-1].pos.y, testnode.pos.x - lane.nodes[i-1].pos.x) :
									 atan2(lane.nodes[i+1].pos.y - testnode.pos.y, lane.nodes[i+1].pos.x - testnode.pos.x)
						vT = [cos(θ2), sin(θ2)]
						T = [testnode.pos.x, testnode.pos.y]
						A = T - E

						c_p = dot(A,vE)
						if c_p > 0.0
							proj = copy(vE).*c_p
							perp_dist = norm(A - proj,2)
							line_dist = norm(proj)
							Δθ = abs(_signed_dist_btw_angles(θ,θ2))

							θ3 = atan2(node.pos.y - testnode.pos.y, node.pos.x - testnode.pos.x)
							Δ3 = abs(_signed_dist_btw_angles(θ,θ3))

							score = Δθ + 10.0*Δ3
							
							if line_dist < THRESHOLD_HANGING_EDGE_LINE_DIST_TO_CONNECT &&
								Δθ < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE && 
								Δ3 < THRESHOLD_HANGING_EDGE_ANGLE_DIFFERENCE2 &&
								score < best_score

								best_score = score
								best_node = testnode
							end
						end
					end
				end
			end

			if !(node === best_node)
				verbosity > 1 || println("ADDED START NODE HANGING EDGE ", node.pos.x, "  ", node.pos.y)
				add_edge!(G, best_node, node)
			end
		end
	end

	# check that each node from the graph is in a matching lane in its tile
	for node in vertices(sn.graph)
		tile = get_tile(sn, node)
		# @assert(has_lane(tile, node.id))
		if !has_lane(tile, node.id)
			println("MISSING ", node.id)
		end
	end

	# check that each lane is non-empty
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)
				@assert(!isempty(lane.nodes))
			end
		end
	end

	# compute curves for segments
	# TODO(tim): maybe do this across tile boundaries for the entire lane at a time
	verbosity < 1 || @printf("COMPUTE CURVES [%.2f]\n", time()-starttime)
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)
				nodes = lane.nodes
				node_prev = prev_node(sn, nodes[1])
				node_next = next_node(sn, nodes[end])
				has_prev = node_prev != nodes[1]
				has_next = node_next != nodes[end]

				n_nodes =  length(nodes) + has_prev + has_next
				if n_nodes < 2
					continue
				end

				pts = Array(Float64, 2, n_nodes)
				total = 0
				if has_prev
					total += 1
					pts[1,total] = node_prev.pos.x
					pts[2,total] = node_prev.pos.y
				end
				for node in nodes
					total += 1
					pts[1,total] = node.pos.x
					pts[2,total] = node.pos.y
				end
				if has_next
					total += 1
					pts[1,total] = node_next.pos.x
					pts[2,total] = node_next.pos.y
				end

				if total > 1
					lane.curve = fit_curve(pts, lane.id, DESIRED_DISTANCE_BETWEEN_CURVE_SAMPLES)
					if has_prev
						# adjust s so dist to first node is 0.0
						c = lane.curve
						extind = Curves.closest_point_extind_to_curve(c, pts[1,2], pts[2,2])
						pt = Curves.curve_at(c, extind)
						lane.curve = Curve(lane.id, c.s - pt.s, c.x, c.y, c.t, c.k, c.k_d)
					end
					lane.has_leading_node = has_prev
					lane.has_trailing_node = has_next
				else
					println("LANE WITH ONE POINT!")
				end
			end
		end
	end

	# fix the extind and d_along for each waypoint
	verbosity < 1 || @printf("COMPUTE NODE EXTIND AND D_ALONG [%.2f]\n", time()-starttime)
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)

				guess = 1.0
				for node in lane.nodes
					extind = closest_point_extind_to_curve_guess(lane.curve, node.pos.x, node.pos.y, guess)
					node.extind = extind
					node.d_along = curve_at(lane.curve, extind).s
					@assert(!isnan(node.d_along))
				end
			end
		end
	end

	# compute remaining values for each waypoint
	verbosity < 1 || @printf("COMPUTE NODE D_ VALUES [%.2f]\n", time()-starttime)
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)
				for node in lane.nodes
					node.d_end   = _distance_to_lane_end(sn, seg, lane.id, node.extind)
					node.d_split = _distance_to_lane_split(sn, seg, lane.id, node.extind)
					node.d_merge = _distance_to_lane_merge(sn, seg, lane.id, node.extind)
					_calc_num_lanes_on_side!(sn, tile, node)

					@assert(node.n_lanes_left ≥ 0)
					@assert(node.n_lanes_left ≥ 0)
					@assert(node.marker_dist_left > 0.0)
					@assert(node.marker_dist_right > 0.0)
				end
			end
		end
	end

	verbosity < 1 || @printf("DONE [%.2f]\n", time()-starttime)
	sn
end

function _calc_num_lanes_on_side!(sn::StreetNetwork)
	for tile in values(sn.tile_dict)
		for seg in values(tile.segments)
			for lane in values(seg.lanes)
				for node in lane.nodes
					_calc_num_lanes_on_side!(sn, tile, node)
				end
			end
		end
	end

	sn
end
function _calc_num_lanes_on_side!(sn::StreetNetwork, center_tile::NetworkTile, node::StreetNode)

	const THRESHOLD_PROJECTION_DELTA_ALONG_LANE = 0.5 # [m]
	const THRESHOLD_PROJECTION_DELTA_PERP_LANE = 20.0 # [m]
	const THRESHOLD_FLOW_ANGLE_DIFFERENCE = deg2rad(20) # [rad] how parallel neighboring lane needs to be to be considered a lane

	const LANE_SEP_MIN = 2.5 # min distance lanes should be apart [m]
	const LANE_SEP_MAX = 5.0 # max distance lanes should be apart [m]
	const N_DIST_DISCRETIZATIONS = 6

	const DIST_DISCRETIZATION = linspace(LANE_SEP_MIN/2,LANE_SEP_MAX/2,N_DIST_DISCRETIZATIONS)

	seg_id = int(node.id.segment)
	lane_id = int(node.id.lane)
	current_lanetag = LaneTag(center_tile, seg_id, lane_id)

	x, y = node.pos.x, node.pos.y
	θ   = curve_at(get_lane(center_tile, node.id).curve, node.extind).θ
	# unit_v = [cos(θ), sin(θ)]
	p_orig = [x,y]

	# project out left a series of discretizations
	# project out right a series of discretizations

	# LEFT
	unit_left = [cos(θ+π/2), sin(θ+π/2)]
	dist_add = 0.0
	finished = false
	active_lanetag = current_lanetag
	node.n_lanes_left = 0
	node.marker_dist_left = DEFAULT_LANE_WIDTH/2

	lanes_seen = Set{LaneTag}()
	push!(lanes_seen, active_lanetag)
	push!(lanes_seen, next_lanetag(sn, active_lanetag))
	push!(lanes_seen, prev_lanetag(sn, active_lanetag))

	while !finished
		finished = true
		for dist in DIST_DISCRETIZATION
			pt = p_orig + (dist+dist_add)*unit_left
			proj = project_point_to_streetmap(pt[1], pt[2], sn)
			if proj.successful
				lanetag = LaneTag(proj.tile, proj.laneid)
				if !in(lanetag, lanes_seen) # found a new one!
					perp_dist = hypot(x - proj.curvept.x, y - proj.curvept.y)
					if LANE_SEP_MIN < perp_dist-dist_add < LANE_SEP_MAX
						node.n_lanes_left += 1
						if node.n_lanes_left == 1
							node.marker_dist_left = perp_dist/2
						end
						finished = false

						active_lanetag = lanetag
						push!(lanes_seen, active_lanetag)
						push!(lanes_seen, next_lanetag(sn, active_lanetag))
						push!(lanes_seen, prev_lanetag(sn, active_lanetag))

						dist_add += perp_dist
					end
					break
				end
			end
		end
	end

	# RIGHT
	unit_right = [cos(θ-π/2), sin(θ-π/2)]
	dist_add = 0.0
	finished = false
	active_lanetag = current_lanetag
	node.n_lanes_right = 0
	node.marker_dist_right = DEFAULT_LANE_WIDTH/2

	lanes_seen = Set{LaneTag}()
	push!(lanes_seen, active_lanetag)
	push!(lanes_seen, next_lanetag(sn, active_lanetag))
	push!(lanes_seen, prev_lanetag(sn, active_lanetag))

	while !finished
		finished = true
		for dist in DIST_DISCRETIZATION
			pt = p_orig + (dist+dist_add)*unit_right
			proj = project_point_to_streetmap(pt[1], pt[2], sn)
			if proj.successful
				lanetag = LaneTag(proj.tile, proj.laneid)
				if !in(lanetag, lanes_seen) # found a new one!
					perp_dist = hypot(x-proj.curvept.x, y-proj.curvept.y)
					if LANE_SEP_MIN < perp_dist-dist_add < LANE_SEP_MAX
						node.n_lanes_right += 1
						if node.n_lanes_right == 1
							node.marker_dist_right = perp_dist/2
						end
						finished = false

						active_lanetag = lanetag
						push!(lanes_seen, active_lanetag)
						push!(lanes_seen, next_lanetag(sn, active_lanetag))
						push!(lanes_seen, prev_lanetag(sn, active_lanetag))

						dist_add += perp_dist
					end
					break
				end
			end
		end
	end

	# NOTE(tim)
	#   for each node, find all lanes in the neighboring tiles that:
	#      1 - share the same segment but are in a different lane
	#      2 - project to within THRESHOLD_PROJECTION_DELTA_ALONG_LANE of this curve
	#      3 - are within THRESHOLD_FLOW_ANGLE_DIFFERENCE of being parallel
	
	#   then, run out left and right grabbing nodes in order of lane-spaces 

	# candidate_pts = (Bool, Float64)[] # (on_left, dist_perp)

	# for tile in extisting_neighbor_tiles_inclusive(sn, center_tile)
	# 	if has_segment(tile, seg_id)
	# 		seg = get_segment(tile, seg_id)
	# 		for lane in values(seg.lanes)
	# 			if lane.id != lane_id && !in(lane.id, lanes_seen)

	# 				# NOTE(tim): find the point on their curve that is closest
	# 				extind = closest_point_extind_to_curve(lane.curve, x, y)
	# 				pt_vec = curve_at(lane.curve, extind)

	# 				px, py, pθ = pt_vec[XIND], pt_vec[YIND], pt_vec[TIND]

	# 				# NOTE(tim): project to our local linearly approximated curve
	# 				vA = [px-x,py-y]
	# 				proj_loc = unit_v.*dot(vA,unit_v)
	# 				proj_dist_along = norm(proj_loc, 2)
	# 				proj_dist_perp  = norm(vA - proj_loc, 2)

	# 				if proj_dist_along < THRESHOLD_PROJECTION_DELTA_ALONG_LANE && 
	# 					proj_dist_perp < THRESHOLD_PROJECTION_DELTA_PERP_LANE && 
	# 				 	abs(_signed_dist_btw_angles(θ, pθ)) < THRESHOLD_FLOW_ANGLE_DIFFERENCE

	# 					push!(lanes_seen, lane.id)
	# 					push!(candidate_pts, (unit_v[1]*vA[2] > unit_v[2]*vA[1], proj_dist_perp))
	# 				end
	# 			end
	# 		end
	# 	end
	# end

	# n_lanes_left = 0
	# marker_dist_left = DEFAULT_LANE_WIDTH/2
	# dist_base = 0.0 # [m]
	# while true

	# 	threshold_lo = dist_base + LANE_SEP_MIN
	# 	threshold_hi = dist_base + LANE_SEP_MAX

	# 	best_dist = Inf

	# 	for (on_left, proj_dist_perp) in candidate_pts
	# 		if on_left && (threshold_lo < proj_dist_perp < threshold_hi) && proj_dist_perp < best_dist
	# 			best_dist = proj_dist_perp
	# 		end
	# 	end

	# 	if !isinf(best_dist)
	# 		n_lanes_left += 1
	# 		dist_base = best_dist

	# 		if n_lanes_left == 1
	# 			marker_dist_left = best_dist / 2
	# 		end
	# 	else
	# 		break
	# 	end
	# end

	# n_lanes_right = 0
	# marker_dist_right = DEFAULT_LANE_WIDTH/2
	# dist_base = 0.0 # [m]
	# while true

	# 	threshold_lo = dist_base + LANE_SEP_MIN
	# 	threshold_hi = dist_base + LANE_SEP_MAX

	# 	best_dist = Inf

	# 	for (on_left, proj_dist_perp) in candidate_pts
	# 		if !on_left && threshold_lo < proj_dist_perp < threshold_hi && proj_dist_perp < best_dist
	# 			best_dist = proj_dist_perp
	# 		end
	# 	end

	# 	if !isinf(best_dist)
	# 		n_lanes_right += 1
	# 		dist_base = best_dist
			
	# 		if n_lanes_right == 1
	# 			marker_dist_right = best_dist / 2
	# 		end
	# 	else
	# 		break
	# 	end
	# end


	# node.n_lanes_left = n_lanes_left
	# node.n_lanes_right = n_lanes_right
	# node.marker_dist_left = marker_dist_left
	# node.marker_dist_right = marker_dist_right

	node
end

function curves2streetnetwork(curves::CurveSet; verbosity::Int=0)
	rndf = curves2rndf(curves)
	rndf2streetnetwork(rndf, verbosity=verbosity, convert_ll2utm = false)
end
function curves2rndf(curves::CurveSet; node_separation::Real=5.0)

	seg = RNDF_Segment(1)

	rndf = RNDF()
	RoadNetwork.add_segment!(rndf, seg)

	nlanes = length(curves)
	for i = 1 : nlanes
		lane = RNDF_Lane(LaneID(1,i))
		seg.lanes[i] = lane

		curve = curves[i]
		lane.waypoints[1] = RoadNetwork.LatLonAlt(curve.x[1], curve.y[1])
		s_last = curve.s[1]
		waypoint_count = 1

		for j = 2 : int(length(curve))
			if (curve.s[j] - s_last > node_separation) || j == length(curve)
				waypoint_count += 1
				lane.waypoints[waypoint_count] = RoadNetwork.LatLonAlt(curve.x[j], curve.y[j])
				s_last = curve.s[j]
			end
		end
	end

	rndf
end

dist(A::StreetNode, B::StreetNode) = hypot(A.pos.x - B.pos.x, A.pos.y - B.pos.y)

end # end module