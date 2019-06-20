package box

import (
	"image/color"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
)

type Crate struct {
	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	size pixel.Vec
	colour color.Color
}

func NewCrate(world *box2d.B2World, pos, size pixel.Vec) *Crate {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = box2d.MakeB2Vec2(pos.X*worldToBox2d, pos.Y*worldToBox2d)

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*worldToBox2d, size.Y*0.5*worldToBox2d)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Shape = shape
	fixDef.Density = 1.0
	fixDef.Friction = 0.3
	fixDef.Restitution = 0.4

	// create body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	return &Crate{
		bodyDef: bodyDef,
		body:    body,

		size: size,
		colour: pixel.RGB(210.0/255.0, 105.0/255.0, 30.0/255.0),
	}
}

func (c *Crate) Draw(win *pixelgl.Window) {
	drawRectBody(win, box2dToPixel(c.body.GetPosition()), c.size, c.body.GetAngle(), c.colour)
}
