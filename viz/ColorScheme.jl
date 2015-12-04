module ColorScheme

using Reexport
@reexport using Colors

export
	COLOR_ASPHALT,
	COLOR_LANE_MARKINGS,
	COLOR_CAR_EGO,
	COLOR_CAR_OTHER,

	getcolorscheme,
	hexcolor,
	lerp_color

hexcolor(r::UInt8, g::UInt8, b::UInt8) = RGB(r/255.0, g/255.0, b/255.0)
hexcolor(r::UInt8, g::UInt8, b::UInt8, a::UInt8) = RGBA(r/255.0, g/255.0, b/255.0, a/255.0)
function hexcolor(html::UInt32)
	r = uint8(html >> 24)
	g = uint8(html >> 16)
	b = uint8(html >>  8)
	a = uint8(html)
	hexcolor(r,g,b,a)
end


const COLOR_ASPHALT       = hexcolor(0x70,0x80,0x90,0xFF)
const COLOR_LANE_MARKINGS = hexcolor(0xDF,0xDF,0xDF,0xFF)
const COLOR_CAR_EGO       = hexcolor(0x79,0xAB,0xFF,0xFF) # bluish
const COLOR_CAR_OTHER     = hexcolor(0xFF,0x00,0x7F,0xFF) # reddish

const COLOR_SCHEMES = Dict(
		"plain"    => Dict(
			"foreground" => hexcolor(0x00,0x00,0x00), # black
			"background" => hexcolor(0xFF,0xFF,0xFF), # white
			"color1"     => hexcolor(0xFF,0x00,0x00), # red
			"color2"     => hexcolor(0x00,0xFF,0x00), # green
			"color3"     => hexcolor(0x00,0x00,0xFF), # blue
			"color4"     => hexcolor(0xFF,0x00,0xFF), # purple
			"color5"     => hexcolor(0xFF,0x80,0x00), # orange
		),


		"monokai" => Dict(
			"foreground" => hexcolor(0xCF,0xBF,0xAD),
			"background" => hexcolor(0x27,0x28,0x22),
			"color1"     => hexcolor(0x52,0xE3,0xF6), # light blue
			"color2"     => hexcolor(0xA7,0xEC,0x21), # light green
			"color3"     => hexcolor(0xFF,0x00,0x7F), # red
			"color4"     => hexcolor(0xF9,0x97,0x1F), # orange
			"color5"     => hexcolor(0x79,0xAB,0xFF), # cobalt
		),

		"3024_day" => Dict(
			"foreground" => hexcolor(0x4A,0x45,0x43),
			"background" => hexcolor(0xF7,0xF7,0xF7),
			"color1"     => hexcolor(0x80,0x7D,0x7C),
			"color2"     => hexcolor(0xA1,0x6A,0x94),
			"color3"     => hexcolor(0xDB,0x2D,0x20),
			"color4"     => hexcolor(0x01,0xA0,0xE4),
			"color5"     => hexcolor(0xED,0x0C,0x8C),
		),
	)

function getcolorscheme(name::AbstractString)
	global COLOR_SCHEMES
	if !haskey(COLOR_SCHEMES, name)
		warn("Color Scheme " * name * " does not exist!")
		name = "plain"
	end
	return COLOR_SCHEMES[name]
end
function lerp_color(a::Colorant, b::Colorant, t::Real)

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

# --------------------------------

# distinguishable_colors(n::Integer,seed::Color)
# linspace(c1::Color, c2::Color, n=100)

end # module