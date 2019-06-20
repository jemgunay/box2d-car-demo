package box

import (
	"image/color"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
)

type Wall struct {
	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	size pixel.Vec
	colour color.Color
}

func NewWall(world *box2d.B2World, pos, size pixel.Vec) *Wall {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_staticBody
	bodyDef.Position = box2d.MakeB2Vec2(pos.X*worldToBox2d, pos.Y*worldToBox2d)
	bodyDef.Angle = 0

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*worldToBox2d, size.Y*0.5*worldToBox2d)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Shape = shape
	fixDef.Restitution = 0.4

	// create body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	return &Wall{
		bodyDef: bodyDef,
		body:    body,

		size: size,
		colour: pixel.RGB(0.2, 0.1, 0.5),
	}
}

func (w *Wall) Draw(win *pixelgl.Window) {
	drawRectBody(win, box2dToPixel(w.body.GetPosition()), w.size, w.body.GetAngle(), w.colour)
}
