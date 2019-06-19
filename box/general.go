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

//func drawRectBody(body *box2d.B2Body, ) {
//	// convert to pixel vector and scale to real world co-ordinates
//	posCentre := box2dToPixel(body.GetPosition()).Scaled(box2dToWorld)
//	// offset from centre to bottom left
//	posOffset := posCentre.Sub(size.Scaled(0.5))
//
//	sprite := imdraw.New(nil)
//	sprite.Color = c.colour
//	sprite.Push(
//		pixel.V(posOffset.X, posOffset.Y),
//		pixel.V(posOffset.X, posOffset.Y+c.size.Y),
//		pixel.V(posOffset.X+c.size.X, posOffset.Y+c.size.Y),
//		pixel.V(posOffset.X+c.size.X, posOffset.Y),
//	)
//
//	sprite.SetMatrix(pixel.IM.Rotated(posCentre, c.body.GetAngle()))
//	sprite.Polygon(0)
//	sprite.Draw(win)
//}

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

func rotate(vec pixel.Vec, angle float64) pixel.Vec {
	angle = normaliseRadians(angle)
	vX := vec.X*math.Cos(angle) - vec.Y*math.Sin(angle)
	vY := vec.X*math.Sin(angle) + vec.Y*math.Cos(angle)

	return pixel.V(vX, vY)
}
