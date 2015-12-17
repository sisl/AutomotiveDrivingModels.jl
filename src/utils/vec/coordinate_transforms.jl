export
    AbstractCoordinate,
    LatLonAlt,
    ECEF,
    UTM,
    ENU,

    check_is_in_radians,
    ensure_lon_between_pies,
    deg_rad_min_to_degrees,

    GeodeticDatum,
    eccentricity


abstract AbstractCoordinate

"""
The latitude, longitude, altitude coordinate system
"""
type LatLonAlt <: AbstractCoordinate
    lat::Float64 # [rad] ϕ, angle between the equatorial plane and the straight line that passes through the point and through the center of the Earth
    lon::Float64 # [rad] λ, angle (typically east) from a reference meridian to another meridian that passes through that point
    alt::Float64 # [m] above reference ellipsoid
end
function check_is_in_radians(lla::LatLonAlt)
    if abs(lla.lat) > π/2 || abs(lla.lon) > π
        warn("lat and lon may not be in radians!")
    end
    nothing
end
function ensure_lon_between_pies(lla::LatLonAlt)
    lon = lla.lon

    while lon ≤ -π
        lon += 2π
    end
    while lon ≥ π
        lon -= 2π
    end

    LatLonAlt(lla.lat, lon, lla.alt)
end
deg_min_sec_to_degrees(deg::Real, min::Real, sec::Real) = convert(Float64, deg + min/60.0 + sec/3600.0)
function degrees_to_deg_min_sec(degrees::Real)
    degrees = convert(Float64, degrees)
    deg, remainder = divrem(degrees, 1.0)
    min, remainder = divrem(remainder*60.0, 1.0)
    sec = remainder * 60.0
    (convert(Int, deg), convert(Int, min), sec)
end

Base.convert(::Type{VecE3}, lla::LatLonAlt) = VecE3(lla.lat, lla.lon, lla.alt)
Base.convert(::Type{LatLonAlt}, p::VecE3) = LatLonAlt(p.x, p.y, p.z)

"""
The Earth-centered, Earth-fixed coordinate system
"""
type ECEF <: AbstractCoordinate
    x::Float64 # [m]
    y::Float64 # [m]
    z::Float64 # [m]
end
Base.convert(::Type{VecE3}, p::ECEF) = VecE3(p.x, p.y, p.z)
Base.convert(::Type{ECEF}, p::VecE3) = ECEF(p.x, p.y, p.z)

"""
Universal Transverse Mercator coordinate system
"""

const UTM_LATITUDE_LIMIT_NORTH = deg2rad(84)
const UTM_LATITUDE_LIMIT_SOUTH = deg2rad(-80)

type UTM <: AbstractCoordinate
    e::Float64 # [m]
    n::Float64 # [m]
    u::Float64 # [m] above reference ellipsoid
    zone::Int # UTM zone (∈ 1:60)
end
Base.convert(::Type{VecE3}, p::UTM) = VecE3(p.e, p.n, p.u)

"""
East North Up coordinate system
relative to some point on the surface of the earth
"""
type ENU <: AbstractCoordinate
    e::Float64 # [m]
    n::Float64 # [m]
    u::Float64 # [m] above reference location
end
Base.convert(::Type{VecE3}, p::ENU) = VecE3(p.e, p.n, p.u)
Base.convert(::Type{ENU}, p::VecE3) = ENU(p.x, p.y, p.z)

###################

"""
Defines a reference ellipsoid used to map a position on the Earth
relative to said ellipsoid
"""
type GeodeticDatum
    a::Float64 # equitorial radius
    b::Float64 # polar radius
end

function eccentricity(datum::GeodeticDatum)
    d = datum.b/datum.a
    sqrt(1.0-d*d)
end
function radius_of_curvature_in_the_meridian(datum::GeodeticDatum, ϕ::Float64)

    # ϕ = meridian angle (latitude)

    a = datum.a
    e = eccentricity(datum)
    e² = e*e
    sin²ϕ = sin(ϕ)^2
    ρ = a*(1-e²) / ((1-e²*sin²ϕ)^1.5)

    ρ
end
function radius_of_curvature_in_the_prime_vertical(datum::GeodeticDatum, ϕ::Float64)

    # ϕ = meridian angle (latitude)

    a = datum.a
    e = eccentricity(datum)
    e² = e*e
    sin²ϕ = sin(ϕ)^2
    ν = a / sqrt(1 - e²*sin²ϕ)

    ν
end

const WGS_84        = GeodeticDatum(6378137.0, 6356752.314245)
const INTERNATIONAL = GeodeticDatum(6378388.0, 6356911.94613)

###################

function Base.convert(::Type{ECEF}, lla::LatLonAlt, datum::GeodeticDatum=WGS_84)

    lat = lla.lat
    lon = lla.lon
    alt = lla.alt

    e = eccentricity(datum)
    e² = e*e

    slat = sin(lat)
    clat = cos(lat)
    slon = sin(lon)
    clon = cos(lon)

    RN = a / sqrt(1.0 - e²*slat*slat)

    x = (RN + alt)*clat*clon
    y = (RN + alt)*clat*slon
    z = (RN*(1.0-e²)+alt)*slat

    ECEF(x,y,z)
end
function Base.convert(::Type{LatLonAlt}, ecef::ECEF, datum::GeodeticDatum=WGS_84)

    # http://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html

    #=
    Convert Earth-Centered-Earth-Fixed (ECEF) to lat, Lon, Altitude
    Input is x, y, z in meters
    Returns lat and lon in radians, and altitude in meters
    =#

    a = datum.a
    b = datum.b
    e = eccentricity(datum)
    e2 = e*e
    a1 = a*e2
    a2 = a1*a1
    a3 = a1*e2/2
    a4 = 2.5*a2
    a5 = a1+a3
    a6 = 1.0-e2

    x = ecef.x
    y = ecef.y
    z = ecef.z

    geo = LatLonAlt(NaN,NaN,NaN)

    zp = abs(z)
    w2 = x*x + y*y
    w = sqrt(w2)
    r2 = w2 + z*z
    r = sqrt(r2)
    geo.lon = atan2(y, x)     # Lon (final)
    s2 = z*z/r2
    c2 = w2/r2
    u = a2/r
    v = a3 - a4/r
    if c2 > 0.3
        s = ( zp/r )*( 1.0 + c2*( a1 + u + s2*v )/r )
        geo.lat = asin( s )      # Lat
        ss = s*s
        c = sqrt( 1.0 - ss )
    else
        c = ( w/r )*( 1.0 - s2*( a5 - u - c2*v )/r );
        geo.lat = acos( c )      # Lat
        ss = 1.0 - c*c
        s = sqrt( ss )
    end
    g = 1.0 - e2*ss;
    rg = a/sqrt( g );
    rf = a6*rg;
    u = w - rg*c;
    v = zp - rf*s;
    f = c*u + s*v;
    m = c*v - s*u;
    p = m/( rf/g + f );
    geo.lat = geo.lat + p     # Lat
    geo.alt = f + m*p/2.0     # Altitude
    if z < 0.0
        geo.lat *= -1.0     # Lat
    end

    geo
end

function Base.convert(::Type{UTM}, lla::LatLonAlt, datum::GeodeticDatum=WGS_84, zone::Integer=-1)
    # see DMATM 8358.2 by the Army
    # code verified using their examples

    check_is_in_radians(lla)
    lla = ensure_lon_between_pies(lla)

    lat = lla.lat
    lon = lla.lon

    if lat > UTM_LATITUDE_LIMIT_NORTH || lat < UTM_LATITUDE_LIMIT_SOUTH
        error("latitude $(rad2deg(lat)) is out of limits!")
    end

    const FN = 0.0      # false northing, zero in the northern hemisphere
    const FE = 500000.0 # false easting
    const ko = 0.9996   # central scale factor

    zone_centers = -177.0*pi/180 + 6.0*pi/180*collect(0:59) # longitudes of the zone centers
    if zone == -1
        zone = indmin(map(x->abs(lon - x), zone_centers)) # index of min zone center
    end
    long0 = zone_centers[zone]

    s  = sin(lat)
    c  = cos(lat)
    t  = tan(lat)

    a = datum.a
    b = datum.b

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
    U = lla.alt

    UTM(E, N, U, zone)
end
function Base.convert(::Type{LatLonAlt}, utm::UTM, datum::GeodeticDatum=WGS_84)
    # see DMATM 8358.2 by the Army
    # code verified using their examples

    zone = utm.zone
    @assert(1 ≤ zone ≤ 60)
    λₒ = deg2rad(6.0*zone - 183) # longitude of the origin of the projection

    kₒ = 0.9996 # central scale factor
    a = datum.a
    b = datum.b

    e = eccentricity(datum)
    e² = e*e
    e⁴ = e²*e²
    e⁶ = e⁴*e²

    eₚ = sqrt((a^2-b^2)/b^2) # second eccentricity
    eₚ² = eₚ*eₚ
    eₚ⁴ = eₚ²*eₚ²
    eₚ⁶ = eₚ⁴*eₚ²
    eₚ⁸ = eₚ⁴*eₚ⁴

    N = utm.n
    arc = N / kₒ
    μ = arc / (a * (1 - e²/4 - 3*e⁴/64 - 5*e⁶/256))
    ei = (1 - sqrt(1 - e²)) / (1 + sqrt(1 - e²))
    ca = 3*ei/2 - 27*ei^3 / 32.0
    cb = 21 * ei^2 / 16 - 55*ei^4 / 32
    cc = 151 * ei^3 / 96
    cd = 1097 * ei^4 / 512
    ϕₚ = μ + ca * sin(2μ) + cb*sin(4μ) + cc*sin(6μ) + cd*sin(8μ)

    # const FN = 0.0      # false northing, zero in the northern hemisphere
    E = utm.e
    FE = 500000.0 # false easting
    ΔE = E - FE
    ΔE² = ΔE*ΔE
    ΔE³ = ΔE²*ΔE
    ΔE⁴ = ΔE²*ΔE²
    ΔE⁵ = ΔE³*ΔE²
    ΔE⁶ = ΔE³*ΔE³
    ΔE⁷ = ΔE⁴*ΔE³
    ΔE⁸ = ΔE⁴*ΔE⁴

    tanϕₚ = tan(ϕₚ)
    tan²ϕₚ = tanϕₚ*tanϕₚ
    tan⁴ϕₚ = tan²ϕₚ*tan²ϕₚ
    tan⁶ϕₚ = tan⁴ϕₚ*tan²ϕₚ

    cosϕₚ = cos(ϕₚ)
    cos²ϕₚ = cosϕₚ*cosϕₚ
    cos⁴ϕₚ = cos²ϕₚ*cos²ϕₚ
    cos⁶ϕₚ = cos⁴ϕₚ*cos²ϕₚ
    cos⁸ϕₚ = cos⁴ϕₚ*cos⁴ϕₚ

    kₒ² = kₒ*kₒ
    kₒ³ = kₒ²*kₒ
    kₒ⁴ = kₒ²*kₒ²
    kₒ⁵ = kₒ³*kₒ²
    kₒ⁶ = kₒ³*kₒ³
    kₒ⁷ = kₒ⁴*kₒ³
    kₒ⁸ = kₒ⁴*kₒ⁴

    ν = radius_of_curvature_in_the_prime_vertical(datum, ϕₚ)
    ν² = ν*ν
    ν³ = ν²*ν
    ν⁴ = ν²*ν²
    ν⁵ = ν³*ν²
    ν⁶ = ν³*ν³
    ν⁷ = ν⁴*ν³
    ν⁸ = ν⁴*ν⁴

    ρ = radius_of_curvature_in_the_meridian(datum, ϕₚ)

    T10 = tanϕₚ/(2ρ*ν*kₒ²)
    T11 = tanϕₚ/(24ρ*ν³*kₒ⁴) * (5.0 + 3*tan²ϕₚ + eₚ²*cos²ϕₚ - 4*eₚ⁴*cosϕₚ - 9*tan²ϕₚ*eₚ²*cos²ϕₚ)
    T12 = tanϕₚ/(720ρ*ν⁵*kₒ⁶) * (61.0 + 90*tan²ϕₚ + 46.0*eₚ²*cos²ϕₚ + 45*tan⁴ϕₚ - 252*tan²ϕₚ*eₚ²*cos²ϕₚ -
                                   3*eₚ⁴*cos⁴ϕₚ + 100*eₚ⁶*cos⁶ϕₚ - 66*tan²ϕₚ*eₚ⁴*cos⁴ϕₚ -
                                   90*tan⁴ϕₚ*eₚ²*cos²ϕₚ + 88*eₚ⁸*cos⁸ϕₚ + 225*tan⁴ϕₚ*eₚ⁴*cos⁴ϕₚ +
                                   84*tan²ϕₚ*eₚ⁶*cos⁶ϕₚ - 192*tan²ϕₚ*eₚ⁸*cos⁸ϕₚ)
    T13 = tanϕₚ/(40320ρ*ν⁷*kₒ⁸) * (1385.0 + 3633*tan²ϕₚ + 4095*tan⁴ϕₚ + 1575*tan⁶ϕₚ)

    T14 = 1.0/(ν*cosϕₚ*kₒ)
    T15 = (1.0 + 2*tan²ϕₚ + eₚ²*cos²ϕₚ) / (6*(ν³*cosϕₚ*kₒ³))
    T16 = (5.0 + 6*eₚ²*cos²ϕₚ + 28*tan²ϕₚ - 3*eₚ⁴*cos⁴ϕₚ + 8*tan²ϕₚ*eₚ²*cos²ϕₚ +
           24*tan⁴ϕₚ - 4*eₚ⁶*cos⁶ϕₚ + 4*tan²ϕₚ*eₚ⁴*cos⁴ϕₚ + 24*tan²ϕₚ*eₚ⁶*cos⁶ϕₚ) /
            (120*(ν⁵*cosϕₚ*kₒ⁵))
    T17 = (61.0 + 662*tan²ϕₚ + 1320*tan⁴ϕₚ + 720*tan⁶ϕₚ)/(5040*(ν⁷*cosϕₚ*kₒ⁷))

    ϕ = ϕₚ - ΔE²*T10 + ΔE⁴*T11 - ΔE⁶*T12 + ΔE⁸*T13
    λ = λₒ + ΔE*T14 - ΔE³*T15 + ΔE⁵*T16 - ΔE⁷*T17
    alt = utm.u

    LatLonAlt(ϕ, λ, alt)
end

function Base.convert(::Type{ECEF}, neu::ENU, reference::LatLonAlt)

    # http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates

    ϕ = reference.lat
    λ = reference.lon

    sϕ, cϕ = sin(ϕ), cos(ϕ)
    sλ, cλ = sin(λ), cos(λ)

    R = [-sλ  -cλ*sϕ  cλ*cϕ;
          cλ  -sλ*sϕ  sλ*cϕ;
         0.0      cϕ     sϕ]

    p = [neu.x, neu.y, neu.z]
    ecef = R*p

    ECEF(ecef[1], ecef[2], ecef[3])
end
function Base.convert(::Type{ENU}, ecef::ECEF, reference::LatLonAlt)

    # http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates

    ϕ = reference.lat
    λ = reference.lon

    sϕ, cϕ = sin(ϕ), cos(ϕ)
    sλ, cλ = sin(λ), cos(λ)

    R = [-sλ      cλ     0.0;
         -cλ*sϕ  -sλ*sϕ  cϕ;
          cλ*cϕ   sλ*cϕ  sϕ]

    p = [ecef.x, ecef.y, ecef.z]
    enu = R*p

    ENU(enu[1], enu[2], enu[3])
end

# lla = LatLonAlt(deg2rad(deg_min_sec_to_degrees(73, 0, 0)),
#                 deg2rad(deg_min_sec_to_degrees(45, 0, 0)),
#                 0.0)
# utm = convert(UTM, lla, INTERNATIONAL)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f → ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
# @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)

# lla = LatLonAlt(deg2rad(deg_min_sec_to_degrees( 30, 0, 0)),
#                 deg2rad(deg_min_sec_to_degrees(102, 0, 0)),
#                 0.0)
# utm = convert(UTM, lla, INTERNATIONAL)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f → ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
# @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)

# utm = UTM(210577.93, 3322824.35, 0.0, 48)
# lla_target = LatLonAlt(deg2rad(deg_min_sec_to_degrees(30, 0, 6.489)),
#                        deg2rad(deg_min_sec_to_degrees(101, 59, 59.805)), # E
#                        0.0)
# lla = convert(LatLonAlt, utm, INTERNATIONAL)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f ← ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
# @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f (should be this)\n", degrees_to_deg_min_sec(rad2deg(lla_target.lat))..., degrees_to_deg_min_sec(rad2deg(lla_target.lon))...)


# utm = UTM(789411.59, 3322824.08, 0.0, 47)
# lla_target = LatLonAlt(deg2rad(deg_min_sec_to_degrees(30, 0, 6.489)),
#                        deg2rad(deg_min_sec_to_degrees(101, 59, 59.805)), # E
#                        0.0)
# lla = convert(LatLonAlt, utm, INTERNATIONAL)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f ← ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
# @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f (should be this)\n", degrees_to_deg_min_sec(rad2deg(lla_target.lat))..., degrees_to_deg_min_sec(rad2deg(lla_target.lon))...)

# utm = UTM(200000.00, 1000000.00, 0.0, 31)
# lla_target = LatLonAlt(deg2rad(deg_min_sec_to_degrees(9, 2, 10.706)),
#                        deg2rad(deg_min_sec_to_degrees(0, 16, 17.099)), # E
#                        0.0)
# lla = convert(LatLonAlt, utm, INTERNATIONAL)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f ← ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
# @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)
# @printf("%3d %1d %2.3f  %3d %1d %2.3f (should be this)\n", degrees_to_deg_min_sec(rad2deg(lla_target.lat))..., degrees_to_deg_min_sec(rad2deg(lla_target.lon))...)