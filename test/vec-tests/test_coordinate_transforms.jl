let
    @test degrees_to_deg_min_sec(30.0) == (30,0,0)

    lla = LatLonAlt(deg2rad(deg_min_sec_to_degrees(73, 0, 0)),
                    deg2rad(deg_min_sec_to_degrees(45, 0, 0)),
                    0.0)
    utm = convert(UTM, lla, INTERNATIONAL)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f → ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
    # @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)

    lla = LatLonAlt(deg2rad(deg_min_sec_to_degrees( 30, 0, 0)),
                    deg2rad(deg_min_sec_to_degrees(102, 0, 0)),
                    0.0)
    utm = convert(UTM, lla, INTERNATIONAL)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f → ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
    # @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)

    utm = UTM(210577.93, 3322824.35, 0.0, 48)
    lla_target = LatLonAlt(deg2rad(deg_min_sec_to_degrees(30, 0, 6.489)),
                           deg2rad(deg_min_sec_to_degrees(101, 59, 59.805)), # E
                           0.0)
    lla = convert(LatLonAlt, utm, INTERNATIONAL)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f ← ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
    # @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f (should be this)\n", degrees_to_deg_min_sec(rad2deg(lla_target.lat))..., degrees_to_deg_min_sec(rad2deg(lla_target.lon))...)
    @test isapprox(lla.lat, lla_target.lat, atol=1e-8)
    @test isapprox(lla.lon, lla_target.lon, atol=1e-8)


    utm = UTM(789411.59, 3322824.08, 0.0, 47)
    lla_target = LatLonAlt(deg2rad(deg_min_sec_to_degrees(30, 0, 6.489)),
                           deg2rad(deg_min_sec_to_degrees(101, 59, 59.805)), # E
                           0.0)
    lla = convert(LatLonAlt, utm, INTERNATIONAL)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f ← ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
    # @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f (should be this)\n", degrees_to_deg_min_sec(rad2deg(lla_target.lat))..., degrees_to_deg_min_sec(rad2deg(lla_target.lon))...)
    @test isapprox(lla.lat, lla_target.lat, atol=1e-8)
    @test isapprox(lla.lon, lla_target.lon, atol=1e-8)

    utm = UTM(200000.00, 1000000.00, 0.0, 31)
    lla_target = LatLonAlt(deg2rad(deg_min_sec_to_degrees(9, 2, 10.706)),
                           deg2rad(deg_min_sec_to_degrees(0, 16, 17.099)), # E
                           0.0)
    lla = convert(LatLonAlt, utm, INTERNATIONAL)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f ← ", degrees_to_deg_min_sec(rad2deg(lla.lat))..., degrees_to_deg_min_sec(rad2deg(lla.lon))...)
    # @printf("%2d %7.2f  %6.2f\n", utm.zone, utm.n, utm.e)
    # @printf("%3d %1d %2.3f  %3d %1d %2.3f (should be this)\n", degrees_to_deg_min_sec(rad2deg(lla_target.lat))..., degrees_to_deg_min_sec(rad2deg(lla_target.lon))...)
    @test isapprox(lla.lat, lla_target.lat, atol=1e-8)
    @test isapprox(lla.lon, lla_target.lon, atol=1e-8)

    buf = IOBuffer()
    print(buf,  lla)
    close(buf)

    # should NOT create a warning
    check_is_in_radians(LatLonAlt(0.1,0.1,0.1))

    lla = LatLonAlt(0.1,0.2-2π,0.3)
    @test isapprox(ensure_lon_between_pies(lla).lon, 0.2, atol=1e-10)
    lla = LatLonAlt(0.1,0.2+2π,0.3)
    @test isapprox(ensure_lon_between_pies(lla).lon, 0.2, atol=1e-10)

    @test isapprox(get_earth_radius(0.0), 6378137.0, atol=0.1)
    @test isapprox(get_earth_radius(π/2), 6356752.3, atol=0.1)

    v3 = convert(VecE3, ECEF(0.1,0.2,0.3))
    @test v3.x == 0.1
    @test v3.y == 0.2
    @test v3.z == 0.3

    ecef = convert(ECEF, VecE3(0.1,0.2,0.3))
    @test ecef.x == 0.1
    @test ecef.y == 0.2
    @test ecef.z == 0.3

    v3 = convert(VecE3, UTM(0.1,0.2,0.3,1))
    @test v3.x == 0.1
    @test v3.y == 0.2
    @test v3.z == 0.3

    utm = UTM(0.1,0.2,0.3,1) + UTM(0.2,0.3,0.4,1)
    @test utm.e ≈ 0.3
    @test utm.n ≈ 0.5
    @test utm.u ≈ 0.7
    @test utm.zone == 1

    utm = UTM(0.1,0.2,0.3,1) - UTM(0.2,0.3,0.4,1)
    @test utm.e ≈ -0.1
    @test utm.n ≈ -0.1
    @test utm.u ≈ -0.1
    @test utm.zone == 1

    v3 = convert(VecE3, ENU(0.1,0.2,0.3))
    @test v3.x == 0.1
    @test v3.y == 0.2
    @test v3.z == 0.3

    enu = convert(ENU, VecE3(0.1,0.2,0.3))
    @test enu.e == 0.1
    @test enu.n == 0.2
    @test enu.u == 0.3

    enu = ENU(0.1,0.2,0.3) + ENU(0.2,0.3,0.4)
    @test enu.e ≈ 0.3
    @test enu.n ≈ 0.5
    @test enu.u ≈ 0.7

    enu = ENU(0.1,0.2,0.3) - ENU(0.2,0.3,0.4)
    @test enu.e ≈ -0.1
    @test enu.n ≈ -0.1
    @test enu.u ≈ -0.1

    for (lla, ecef) in [(LatLonAlt(deg2rad( 0),deg2rad( 0),  0.0), ECEF(6378137.0,0.0,0.0)),
                        (LatLonAlt(deg2rad(90),deg2rad( 0),  0.0), ECEF(0.0,0.0,6356752.31)),
                        (LatLonAlt(deg2rad(20),deg2rad(40),110.0), ECEF(4593156.33, 3854115.78, 2167734.41))]

        lla2 = convert(LatLonAlt, ecef)
        @test isapprox(lla2.lat, lla.lat, atol=1e-2)
        @test isapprox(lla2.lon, lla.lon, atol=1e-2)
        @test isapprox(lla2.alt, lla.alt, atol=1e-2)

        ecef2 = convert(ECEF, lla)
        @test isapprox(ecef2.x, ecef.x, atol=1.0)
        @test isapprox(ecef2.y, ecef.y, atol=1.0)
        @test isapprox(ecef2.z, ecef.z, atol=1.0)
    end
end