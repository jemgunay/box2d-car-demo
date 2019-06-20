// Package box is a port of https://github.com/domasx2/gamejs-box2d-car-example
package box

import (
	"image/color"
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
	"github.com/faiface/pixel/pixelgl"
)

const (
	worldToBox2d = 1.0 / 30.0
	box2dToWorld = 1.0 / worldToBox2d
)

func drawRectBody(win *pixelgl.Window, pos, size pixel.Vec, angle float64, colour color.Color) {
	// convert to pixel vector and scale to real world co-ordinates
	posCentre := pos.Scaled(box2dToWorld)
	// offset from centre to bottom left
	posOffset := posCentre.Sub(size.Scaled(0.5))

	// create sprite
	sprite := imdraw.New(nil)
	sprite.Color = colour
	sprite.Push(
		pixel.V(posOffset.X, posOffset.Y),
		pixel.V(posOffset.X, posOffset.Y+size.Y),
		pixel.V(posOffset.X+size.X, posOffset.Y+size.Y),
		pixel.V(posOffset.X+size.X, posOffset.Y),
	)

	// orientate sprite
	sprite.SetMatrix(pixel.IM.Rotated(posCentre, angle))
	sprite.Polygon(0)
	sprite.Draw(win)
}

func box2dToPixel(vec box2d.B2Vec2) pixel.Vec {
	return pixel.V(vec.X, vec.Y)
}

func pixelToBox2d(vec pixel.Vec) box2d.B2Vec2 {
	return box2d.MakeB2Vec2(vec.X, vec.Y)
}

func radToDeg(radians float64) float64 {
	return radians * (180.0 / math.Pi)
}

func degToRad(degrees float64) float64 {
	return degrees * (math.Pi / 180.0)
}

func normaliseRadians(radians float64) float64 {
	radians = math.Mod(radians, 2.0*math.Pi)
	if radians < 0 {
		radians += 2.0 * math.Pi
	}
	return radians
}

// rotate point by angle in radians
func rotate(vec pixel.Vec, angle float64) pixel.Vec {
	angle = normaliseRadians(angle)
	vX := vec.X*math.Cos(angle) - vec.Y*math.Sin(angle)
	vY := vec.X*math.Sin(angle) + vec.Y*math.Cos(angle)

	return pixel.V(vX, vY)
}
