export
    COLOR_ASPHALT,
    COLOR_LANE_MARKINGS_WHITE,
    COLOR_LANE_MARKINGS_YELLOW,
    COLOR_CAR_EGO,
    COLOR_CAR_OTHER,
    MONOKAI

const COLOR_ASPHALT       = colorant"0x708090FF"
const COLOR_LANE_MARKINGS_WHITE = colorant"0xDFDFDFFF"
const COLOR_LANE_MARKINGS_YELLOW = colorant"0xFFEF00FF"
const COLOR_CAR_EGO       = colorant"0x79ABFFFF" # bluish
const COLOR_CAR_OTHER     = colorant"0xFF007FFF" # reddish

const MONOKAI = Dict(
            "foreground" => colorant"0xCFBFAD",
            "background" => colorant"0x272822",
            "color1"     => colorant"0x52E3F6", # light blue
            "color2"     => colorant"0xA7EC21", # light green
            "color3"     => colorant"0xFF007F", # red
            "color4"     => colorant"0xF9971F", # orange
            "color5"     => colorant"0x79ABFF", # cobalt
        ),

function Vec.lerp(a::Colorant, b::Colorant, t::Real)

    ra = red(a)
    rb = red(b)
    ga = green(a)
    gb = green(b)
    ba = blue(a)
    bb = blue(b)
    aa = alpha(a)
    ab = alpha(b)

    r = ra + (rb - ra)*t
    g = ga + (gb - ga)*t
    b = ba + (bb - ba)*t
    a = aa + (ab - aa)*t

    RGBA(r,g,b,a)
end