type LatLonSeparableDriver <: DriverModel{LatLonAccel}
    model_lat::LateralDriverModel
    model_lon::LongitudinalDriverModel
end