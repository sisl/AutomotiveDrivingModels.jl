@with_kw mutable struct Tim2DDriver <: DriverModel{LatLonAccel}
    
    mlon::LaneFollowingDriver = IntelligentDriverModel()
    mlat::LateralDriverModel = ProportionalLaneTracker()
    mlane::LaneChangeModel = TimLaneChanger()
end
get_name(::Tim2DDriver) = "Tim2DDriver"
function set_desired_speed!(model::Tim2DDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    set_desired_speed!(model.mlane, v_des)
    model
end

Base.rand(driver::Tim2DDriver) = LatLonAccel(rand(driver.mlat), rand(driver.mlon).a)
Distributions.pdf(driver::Tim2DDriver, a::LatLonAccel) = pdf(driver.mlat, a.a_lat) * pdf(driver.mlon, a.a_lon)
Distributions.logpdf(driver::Tim2DDriver, a::LatLonAccel) = logpdf(driver.mlat, a.a_lat) * logpdf(driver.mlon, a.a_lon)

