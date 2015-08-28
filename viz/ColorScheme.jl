module ColorScheme

using Reexport
@reexport using Colors

export 
	COLOR_ASPHALT,
	COLOR_LANE_MARKINGS,
	COLOR_CAR_EGO,
	COLOR_CAR_OTHER	,

	getcolorscheme

const COLOR_ASPHALT       = RGBA(0x70,0x80,0x90,0xFF)
const COLOR_LANE_MARKINGS = RGBA(0xDF,0xDF,0xDF,0xFF)
const COLOR_CAR_EGO       = RGBA(0x79,0xAB,0xFF,0xFF) # bluish
const COLOR_CAR_OTHER     = RGBA(0xFF,0x00,0x7F,0xFF) # reddish

const COLOR_SCHEMES = [
		"plain"    => [
			"foreground" => RGB(0x00,0x00,0x00), # black
			"background" => RGB(0xFF,0xFF,0xFF), # white
			"color1"     => RGB(0xFF,0x00,0x00), # red
			"color2"     => RGB(0x00,0xFF,0x00), # green
			"color3"     => RGB(0x00,0x00,0xFF), # blue
			"color4"     => RGB(0xFF,0x00,0xFF), # purple
			"color5"     => RGB(0xFF,0x80,0x00), # orange
		],


		"monokai" => [
			"foreground" => RGB(0xCF,0xBF,0xAD),
			"background" => RGB(0x27,0x28,0x22),
			"color1"     => RGB(0x52,0xE3,0xF6), # light blue
			"color2"     => RGB(0xA7,0xEC,0x21), # light green
			"color3"     => RGB(0xFF,0x00,0x7F), # red
			"color4"     => RGB(0xF9,0x97,0x1F), # orange
			"color5"     => RGB(0x79,0xAB,0xFF), # cobalt
		],

		"3024_day" => [
			"foreground" => RGB(0x4A,0x45,0x43),
			"background" => RGB(0xF7,0xF7,0xF7),
			"color1"     => RGB(0x80,0x7D,0x7C),
			"color2"     => RGB(0xA1,0x6A,0x94),
			"color3"     => RGB(0xDB,0x2D,0x20),
			"color4"     => RGB(0x01,0xA0,0xE4),
			"color5"     => RGB(0xED,0x0C,0x8C),
		],
	]

function getcolorscheme(name::String)
	global COLOR_SCHEMES
	if !haskey(COLOR_SCHEMES, name)
		warn("Color Scheme " * name * " does not exist!")
		name = "plain"
	end
	return COLOR_SCHEMES[name]
end

# --------------------------------

# distinguishable_colors(n::Integer,seed::Color)
# linspace(c1::Color, c2::Color, n=100)

end # module