package box

import (
	"image/color"
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
)

type (
	steerState uint
	accState   uint
)

const (
	SteerNone steerState = iota
	SteerLeft
	SteerRight
)
const (
	AccForwards accState = iota
	AccReverse
)

type Car struct {
	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	SteerState   steerState
	AccState     accState
	Breaking     bool
	Accelerating bool

	size          pixel.Vec
	colour        color.Color
	maxSteerAngle float64
	maxSpeed      float64
	power         float64
	wheelAngle    float64

	wheels []*Wheel
}

func NewCar(world *box2d.B2World, pos, size pixel.Vec) *Car {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = box2d.MakeB2Vec2(pos.X*worldToBox2d, pos.Y*worldToBox2d)
	bodyDef.Angle = 0
	bodyDef.LinearDamping = 0.15
	bodyDef.AngularDamping = 0.3

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*worldToBox2d, size.Y*0.5*worldToBox2d)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Density = 1.0
	fixDef.Friction = 0.3
	fixDef.Restitution = 0.4
	fixDef.Shape = shape

	// create body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	car := &Car{
		bodyDef: bodyDef,
		body:    body,

		size:          size,
		colour:        pixel.RGB(0.5, 0.5, 0.5),
		maxSteerAngle: 20,
		maxSpeed:      40,
		power:         20,

		SteerState: SteerNone,
	}

	// offset wheels to the ends of the car body
	wheelDeltaY := (size.Y * 0.6) / 2.0
	// scale wheel size relative to car body size
	wheelSize := size.Scaled(0.2)

	// top left
	car.AddWheel(world, pixel.V(-size.X/2.0, -wheelDeltaY), wheelSize, false, false)
	// top right
	car.AddWheel(world, pixel.V(size.X/2.0, -wheelDeltaY), wheelSize, false, false)
	// back left
	car.AddWheel(world, pixel.V(-size.X/2.0, wheelDeltaY), wheelSize, true, true)
	// back right
	car.AddWheel(world, pixel.V(size.X/2.0, wheelDeltaY), wheelSize, true, true)

	return car
}

// returns car's velocity vector relative to the car
func (c *Car) getLocalVelocity() box2d.B2Vec2 {
	return c.body.GetLocalVector(c.body.GetLinearVelocityFromLocalPoint(box2d.MakeB2Vec2(0, 0)))
}

func (c *Car) AddWheel(world *box2d.B2World, relativePos, size pixel.Vec, powered, revolving bool) {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = c.body.GetWorldPoint(box2d.MakeB2Vec2(relativePos.X*worldToBox2d, relativePos.Y*worldToBox2d))
	bodyDef.Angle = c.body.GetAngle()
	bodyDef.LinearDamping = 0.15
	bodyDef.AngularDamping = 0.3

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*worldToBox2d, size.Y*0.5*worldToBox2d)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Density = 1.0
	// disable collision responses
	fixDef.IsSensor = true
	fixDef.Shape = shape

	// create body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	if revolving {
		jointDef := box2d.MakeB2RevoluteJointDef()
		jointDef.Initialize(c.body, body, body.GetWorldCenter())
		jointDef.EnableMotor = true
		world.CreateJoint(&jointDef)
	} else {
		jointDef := box2d.MakeB2PrismaticJointDef()
		jointDef.Initialize(c.body, body, body.GetWorldCenter(), box2d.MakeB2Vec2(1, 0))
		jointDef.EnableLimit = true
		jointDef.LowerTranslation = 0
		jointDef.UpperTranslation = 0
		world.CreateJoint(&jointDef)
	}

	wheel := &Wheel{
		parentCar: c,

		bodyDef: box2d.NewB2BodyDef(),
		body:    body,

		pos:       relativePos,
		size:      size,
		colour:    pixel.RGB(0.3, 0.3, 0.3),
		powered:   powered,
		revolving: revolving,
	}

	c.wheels = append(c.wheels, wheel)
}

// get speed in kilometers per hour
func (c *Car) getSpeedKMH() float64 {
	velocity := c.body.GetLinearVelocity()
	return (velocity.Length() / 1000.0) * 3600.0
}

// set speed in kilometers per hour
func (c *Car) setSpeedKMH(speed float64) {
	vel := box2dToPixel(c.body.GetLinearVelocity())
	vel = vel.Unit()
	vel = vel.Scaled((speed * 1000.0) / 3600.0)
	c.body.SetLinearVelocity(pixelToBox2d(vel))
}

func (c *Car) Update(dt float64) {
	// kill sideways velocity for all wheels
	for _, w := range c.wheels {
		w.killSidewaysVelocity()
	}

	// calculate the change in wheel's angle for this update, assuming the wheel will reach is maximum angle from zero
	// in 200 ms
	incr := (c.maxSteerAngle / 200.0) * dt
	if c.SteerState == SteerRight {
		// increment angle without going over max steer
		c.wheelAngle = math.Min(math.Max(c.wheelAngle, 0)+incr, c.maxSteerAngle)
	} else if c.SteerState == SteerLeft {
		// decrement angle without going over max steer
		c.wheelAngle = math.Max(math.Min(c.wheelAngle, 0)-incr, -c.maxSteerAngle)
	} else {
		c.wheelAngle = 0
	}

	var baseVec pixel.Vec
	if c.Accelerating && c.getSpeedKMH() < c.maxSpeed {
		if c.AccState == AccForwards {
			// forwards
			baseVec = pixel.V(0, 1)
		} else {
			// reverse
			baseVec = pixel.V(0, -0.8)
		}
	}

	if c.Breaking {
		if c.getLocalVelocity().Y > 0 {
			baseVec = pixel.V(0, -1)
		} else if c.getLocalVelocity().Y < 0 {
			baseVec = pixel.V(0, 1)
		}
	}

	// multiply by engine power, which gives us a force vector relative to the wheel
	forceVec := pixelToBox2d(baseVec.Scaled(c.power))

	for _, wheel := range c.wheels {
		// update revolving wheels
		if wheel.revolving {
			wheel.setAngle(degToRad(c.wheelAngle))
		}
		// apply force to each powered wheel
		if wheel.powered {
			pos := wheel.body.GetWorldCenter()
			wheel.body.ApplyForce(wheel.body.GetWorldVector(forceVec), pos, true)
		}
	}

	// if going very slow, stop in order to prevent endless sliding
	if !c.Accelerating && c.getSpeedKMH() < 1 {
		c.setSpeedKMH(0)
	}
}

func (c *Car) Draw(win *pixelgl.Window) {
	drawRectBody(win, box2dToPixel(c.body.GetPosition()), c.size, c.body.GetAngle(), c.colour)

	// draw wheels
	for _, wheel := range c.wheels {
		drawRectBody(win, box2dToPixel(wheel.body.GetPosition()), wheel.size, wheel.body.GetAngle(), wheel.colour)
	}
}

type Wheel struct {
	parentCar *Car

	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	pos, size          pixel.Vec
	colour             color.Color
	powered, revolving bool
}

func (w *Wheel) setAngle(angle float64) {
	w.body.SetTransform(w.body.GetPosition(), w.parentCar.body.GetAngle()-angle)
}

func (w *Wheel) getLocalVelocity(carBody *box2d.B2Body) box2d.B2Vec2 {
	return carBody.GetLocalVector(carBody.GetLinearVelocityFromLocalPoint(box2d.MakeB2Vec2(w.pos.X, w.pos.Y)))
}

// returns a world unit vector pointing in the direction this wheel is moving
func (w *Wheel) getDirectionVector() pixel.Vec {
	var dirVec pixel.Vec
	if w.getLocalVelocity(w.parentCar.body).Y > 0 {
		dirVec = pixel.V(0, 1)
	} else {
		dirVec = pixel.V(0, -1)
	}
	// https://github.com/GameJs/gamejs/blob/master/src/gamejs/math/vectors.js#L85
	return rotate(dirVec, w.body.GetAngle())
}

// substracts sideways velocity from this wheel's velocity vector and returns the remaining front-facing velocity vector.
func (w *Wheel) getKillVelocityVector() box2d.B2Vec2 {
	sidewaysAxis := w.getDirectionVector()
	dotProd := box2dToPixel(w.body.GetLinearVelocity()).Dot(sidewaysAxis)
	return pixelToBox2d(sidewaysAxis.Scaled(dotProd))
}

// removes all sideways velocity from this wheel's velocity
func (w *Wheel) killSidewaysVelocity() {
	kv := w.getKillVelocityVector()
	w.body.SetLinearVelocity(kv)
}
