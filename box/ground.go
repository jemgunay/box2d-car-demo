package box

import (
	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
)

// Ground is a ground for applying top-down friction to dynamic bodies.
type Ground struct {
	bodyDef *box2d.B2BodyDef
	Body    *box2d.B2Body
}

// NewGround creates and initialises a new friction ground.
func NewGround(world *box2d.B2World, pos, size pixel.Vec) *Ground {
	// create rigid Body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_staticBody
	bodyDef.Position = box2d.MakeB2Vec2(pos.X*WorldToBox2D, pos.Y*WorldToBox2D)

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*WorldToBox2D, size.Y*0.5*WorldToBox2D)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Shape = shape
	fixDef.UserData = "ground"
	fixDef.Filter.GroupIndex = -1

	// create Body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	return &Ground{
		bodyDef: bodyDef,
		Body:    body,
	}
}
