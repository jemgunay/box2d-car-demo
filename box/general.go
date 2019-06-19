// Package box is a port of https://github.com/domasx2/gamejs-box2d-car-example
package box

import (
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
)

const (
	worldToBox2d = 1.0 / 30.0
	box2dToWorld = 1.0 / worldToBox2d
)

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

func normaliseRadians(radians float64) float64 {
	radians = math.Mod(radians, 2.0*math.Pi)
	if radians < 0 {
		radians += 2.0 * math.Pi
	}
	return radians
}

func rotate(vec box2d.B2Vec2, angle float64) box2d.B2Vec2 {
	angle = normaliseRadians(angle)
	vX := vec.X*math.Cos(angle) - vec.Y*math.Sin(angle)
	vY := vec.X*math.Sin(angle) + vec.Y*math.Cos(angle)

	return box2d.MakeB2Vec2(vX, vY)
}
