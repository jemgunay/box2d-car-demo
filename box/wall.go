package box

import (
	"image/color"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
)

// Wall is an immovable physics-based wall.
type Wall struct {
	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	size   pixel.Vec
	colour color.Color
}

// NewWall creates and initialises a new wall.
func NewWall(world *box2d.B2World, pos, size pixel.Vec) *Wall {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_staticBody
	bodyDef.Position = box2d.MakeB2Vec2(pos.X*WorldToBox2D, pos.Y*WorldToBox2D)
	bodyDef.Angle = 0

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*WorldToBox2D, size.Y*0.5*WorldToBox2D)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Shape = shape
	fixDef.Restitution = 0.4
	fixDef.UserData = "wall"

	// create body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	return &Wall{
		bodyDef: bodyDef,
		body:    body,

		size:   size,
		colour: pixel.RGB(0.2, 0.1, 0.5),
	}
}

// Draw draws the wall.
func (w *Wall) Draw(win *pixelgl.Window) {
	DrawRectBody(win, ToPixelVec(w.body.GetPosition()), w.size, w.body.GetAngle(), w.colour)
}
