// Package box is a port of https://github.com/domasx2/gamejs-box2d-car-example
package box

import (
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
)

const box2dScale = 1.0 / 30.0

const box2dScale2 = 30.0

func box2dToPixel(vec box2d.B2Vec2) pixel.Vec {
	return pixel.V(vec.X, vec.Y)
}

func pixelToBox2d(vec pixel.Vec) box2d.B2Vec2 {
	return box2d.MakeB2Vec2(vec.X, vec.Y)
}

func radToDeg(radians float64) float64 {
	return radians * (180 / math.Pi)
}

func degToRad(degrees float64) float64 {
	return degrees * (math.Pi / 180)
}
