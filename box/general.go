// Package box implements generic Box2D utils and objects.
package box

import (
	"image/color"
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
)

const (
	// WorldToBox2D is used to scale from the world co-ordinate system to the Box2D co-ordinate system.
	WorldToBox2D = 1.0 / 30.0
	// Box2DToWorld is used to scale from the Box2D co-ordinate system to the world co-ordinate system.
	Box2DToWorld = 1.0 / WorldToBox2D
)

// Categories for defining collision groups.
const (
	WallCategory  uint16 = 0x0001
	CarCategory   uint16 = 0x0002
	CrateCategory uint16 = 0x0004
)

// MainGround is the world ground used to simulate top-down friction.
var MainGround *Ground

// DrawRectBody draws a rectangle shape.
func DrawRectBody(imd *imdraw.IMDraw, pos, size pixel.Vec, angle float64, colour color.Color) {
	// convert to pixel vector and scale to real world co-ordinates
	posCentre := pos.Scaled(Box2DToWorld)
	// offset from centre to bottom left
	posOffset := posCentre.Sub(size.Scaled(0.5))

	// create sprite
	imd.Color = colour
	imd.Push(
		pixel.V(posOffset.X, posOffset.Y),
		pixel.V(posOffset.X, posOffset.Y+size.Y),
		pixel.V(posOffset.X+size.X, posOffset.Y+size.Y),
		pixel.V(posOffset.X+size.X, posOffset.Y),
	)

	// orientate imd
	imd.SetMatrix(pixel.IM.Rotated(posCentre, angle))
	imd.Polygon(0)
}

// DrawCircleBody draws a circle shape.
func DrawCircleBody(imd *imdraw.IMDraw, pos pixel.Vec, radius float64, colour color.Color) {
	imd.Color = colour
	imd.Push(pos)
	imd.Circle(radius, 0)
}

// ToPixelVec converts a Box2D vector to a pixel vector.
func ToPixelVec(vec box2d.B2Vec2) pixel.Vec {
	return pixel.V(vec.X, vec.Y)
}

// ToBox2DVec converts a pixel vector to a Box2D vector.
func ToBox2DVec(vec pixel.Vec) box2d.B2Vec2 {
	return box2d.MakeB2Vec2(vec.X, vec.Y)
}

// RadToDeg converts an angle in radians to degrees.
func RadToDeg(radians float64) float64 {
	return radians * (180.0 / math.Pi)
}

// DegToRad converts an angle in degrees to radians.
func DegToRad(degrees float64) float64 {
	return degrees * (math.Pi / 180.0)
}

// NormaliseRadians normalises an angle in radians between 0 and 2pi.
func NormaliseRadians(radians float64) float64 {
	radians = math.Mod(radians, 2.0*math.Pi)
	if radians < 0 {
		radians += 2.0 * math.Pi
	}
	return radians
}

// Rotate rotates a point by angle in radians.
func Rotate(vec pixel.Vec, angle float64) pixel.Vec {
	angle = NormaliseRadians(angle)
	vX := vec.X*math.Cos(angle) - vec.Y*math.Sin(angle)
	vY := vec.X*math.Sin(angle) + vec.Y*math.Cos(angle)

	return pixel.V(vX, vY)
}
