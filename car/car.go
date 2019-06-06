// Package car is a port of https://github.com/domasx2/gamejs-box2d-car-example
package car

import "github.com/ByteArena/box2d"

type state uint

const (
	SteerNone state = iota
	SteerLeft
	SteerRight

	AccNone state = iota
	AccAccelerate
	AccBrake
)

type Car struct {
	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	SteerState      state
	AccelerateState state

	maxSteerAngle float64
	maxSpeed      float64
	power         float64
	wheelAngle    float64

	wheels []*Wheel
}

func NewCar(w *box2d.B2World) *Car {
	width, length := 2.0, 4.0
	x, y := 10.0, 10.0

	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = box2d.MakeB2Vec2(x, y)
	bodyDef.Angle = 0
	bodyDef.LinearDamping = 0.15
	bodyDef.AngularDamping = 0.3

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(width/2, length/2)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Density = 1.0
	fixDef.Friction = 0.3
	fixDef.Restitution = 0.4
	fixDef.Shape = shape

	// create body
	body := w.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	car := &Car{
		bodyDef: box2d.NewB2BodyDef(),
		body:    body,

		maxSteerAngle: 20,
		maxSpeed:      60,
		power:         60,

		SteerState:      SteerNone,
		AccelerateState: AccNone,
	}

	// top left
	car.AddWheel(w, -1, -1.2, 0.4, 0.8, true, true)
	// top right
	car.AddWheel(w, 1, -1.2, 0.4, 0.8, true, true)
	// back left
	car.AddWheel(w, -1, 1.2, 0.4, 0.8, false, false)
	// back right
	car.AddWheel(w, 1, 1.2, 0.4, 0.8, false, false)

	return car
}

type Wheel struct {
	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	width, length float64
	x, y          float64

	powered, revolving bool
}

func (w *Wheel) setAngle(angle float64) box2d.B2Vec2 {
	w.body.SetTransform(w.body.GetPosition(), angle)
}

func (w *Wheel) getLocalVelocity(carBody *box2d.B2Body) box2d.B2Vec2 {
	return carBody.GetLocalVector(carBody.GetLinearVelocityFromLocalPoint(box2d.MakeB2Vec2(w.x, w.y)))
}


func (w *Wheel) getDirectionVector(carBody *box2d.B2Body) box2d.B2Vec2 {
	//return vectors.rotate((this.getLocalVelocity()[1]>0) ? [0, 1]:[0, -1] , this.body.GetAngle()) ;
	var dirVec box2d.B2Vec2
	if w.getLocalVelocity(carBody).X > 0 {
		dirVec = box2d.MakeB2Vec2(0, 1)
	} else {
		dirVec = box2d.MakeB2Vec2(1, 0)
	}
	// https://github.com/GameJs/gamejs/blob/master/src/gamejs/math/vectors.js#L85
	return dirVec.
}

func (w *Wheel) getKillVelocityVector() {

}

func (c *Car) AddWheel(w *box2d.B2World, x, y, width, length float64, powered, revolving bool) {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = c.body.GetWorldPoint(box2d.MakeB2Vec2(x, y))
	bodyDef.Angle = c.body.GetAngle()
	bodyDef.LinearDamping = 0.15
	bodyDef.AngularDamping = 0.3

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(width/2, length/2)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Density = 1.0
	// disable collision responses
	fixDef.IsSensor = true
	fixDef.Shape = shape

	// create body
	body := w.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	if revolving {
		jointDef := box2d.MakeB2RevoluteJointDef()
		jointDef.Initialize(c.body, body, body.GetWorldCenter())
		jointDef.EnableMotor = true
		w.CreateJoint(&jointDef)
	} else {
		jointDef := box2d.MakeB2PrismaticJointDef()
		jointDef.Initialize(c.body, body, body.GetWorldCenter(), box2d.MakeB2Vec2(1, 0))
		jointDef.EnableLimit = true
		jointDef.LowerTranslation = 0
		jointDef.UpperTranslation = 0
		w.CreateJoint(&jointDef)
	}

	wheel := &Wheel{
		bodyDef: box2d.NewB2BodyDef(),
		body:    body,

		width:  width,
		length: length,
		x:      x,
		y:      y,

		powered:   powered,
		revolving: revolving,
	}

	c.wheels = append(c.wheels, wheel)
}

func (c *Car) Update(dt float64) {
	// kill sideways velocity for all wheels
	for _, w := range c.wheels {

	}
}

func (c *Car) Draw() {

}
