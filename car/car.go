// Package car is a port of https://github.com/domasx2/gamejs-box2d-car-example
package car

import (
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
	"github.com/faiface/pixel/pixelgl"
)

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

	width, length float64
	maxSteerAngle float64
	maxSpeed      float64
	power         float64
	wheelAngle    float64

	wheels []*Wheel
}

func NewCar(w *box2d.B2World, width, length float64) *Car {
	x, y := 100.0, 100.0

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
		bodyDef: bodyDef,
		body:    body,

		width:         width,
		length:        length,
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

func (w *Wheel) setAngle(angle float64) {
	w.body.SetTransform(w.body.GetPosition(), angle)
}

func (w *Wheel) getLocalVelocity(carBody *box2d.B2Body) box2d.B2Vec2 {
	return carBody.GetLocalVector(carBody.GetLinearVelocityFromLocalPoint(box2d.MakeB2Vec2(w.x, w.y)))
}

/*func normaliseRadians(radians float64) float64 {
	r := radians % float64(2*math.Pi)
	if r<0 {
		r+= 2*math.Pi
	}
	return r
}*/

func rotate(vec box2d.B2Vec2, angle float64) box2d.B2Vec2 {
	//angle=angles.normaliseRadians(angle);
	vX := vec.X*math.Cos(angle) - vec.Y*math.Sin(angle)
	vY := vec.X*math.Sin(angle) - vec.Y*math.Cos(angle)

	return box2d.MakeB2Vec2(vX, vY)
}

// returns a world unit vector pointing in the direction this wheel is moving
func (w *Wheel) getDirectionVector(carBody *box2d.B2Body) box2d.B2Vec2 {
	//return vectors.rotate((this.getLocalVelocity()[1]>0) ? [0, 1]:[0, -1] , this.body.GetAngle()) ;
	var dirVec box2d.B2Vec2
	if w.getLocalVelocity(carBody).X > 0 {
		dirVec = box2d.MakeB2Vec2(0, 1)
	} else {
		dirVec = box2d.MakeB2Vec2(1, 0)
	}
	// https://github.com/GameJs/gamejs/blob/master/src/gamejs/math/vectors.js#L85
	return rotate(dirVec, w.body.GetAngle())
}

// substracts sideways velocity from this wheel's velocity vector and returns the remaining front-facing velocity vector.
func (w *Wheel) getKillVelocityVector(carBody *box2d.B2Body) box2d.B2Vec2 {
	vel := w.body.GetLinearVelocity()
	sidewaysAxis := box2dToPixel(w.getDirectionVector(carBody))
	dotProd := box2dToPixel(vel).Dot(sidewaysAxis)
	return box2d.MakeB2Vec2(sidewaysAxis.X*dotProd, sidewaysAxis.Y*dotProd)
}

// removes all sideways velocity from this wheel's velocity
func (w *Wheel) killSidewaysVelocity(carBody *box2d.B2Body) {
	kv := w.getKillVelocityVector(carBody)
	w.body.SetLinearVelocity(kv)
}

// returns car's velocity vector relative to the car
func (c *Car) getLocalVelocity() box2d.B2Vec2 {
	return c.body.GetLocalVector(c.body.GetLinearVelocityFromLocalPoint(box2d.MakeB2Vec2(0, 0)))
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

// get speed in kilometers per hour
func (c *Car) getSpeedKMH() float64 {
	velocity := c.body.GetLinearVelocity()
	len := velocity.Length()
	return (len / 1000) * 3600
}

// set speed in kilometers per hour
func (c *Car) setSpeed(speed float64) {
	vel := box2dToPixel(c.body.GetLinearVelocity())
	vel.Unit().Scaled((speed * 1000) / 3600)
	c.body.SetLinearVelocity(pixelToBox2d(vel))
}

func (c *Car) Update(dt float64) {
	// kill sideways velocity for all wheels
	for _, w := range c.wheels {
		w.killSidewaysVelocity(c.body)
	}

	// calculate the change in wheel's angle for this update, assuming the wheel will reach is maximum angle from zero
	// in 200 ms
	incr := (c.maxSteerAngle / 200) * dt
	if c.SteerState == SteerRight {
		// increment angle without going over max steer
		c.wheelAngle = math.Min(math.Max(c.wheelAngle, 0)+incr, c.maxSteerAngle)
	} else if c.SteerState == SteerLeft {
		// decrement angle without going over max steer
		c.wheelAngle = math.Max(math.Min(c.wheelAngle, 0)-incr, -c.maxSteerAngle)
	} else {
		c.wheelAngle = 0
	}

	// update revolving wheels
	for _, w := range c.wheels {
		if w.revolving {
			w.setAngle(c.wheelAngle)
		}
	}

	var baseVec box2d.B2Vec2
	if (c.AccelerateState == AccAccelerate) && (c.getSpeedKMH() < c.maxSpeed) {
		baseVec = box2d.MakeB2Vec2(0, -1)
	} else if c.AccelerateState == AccBrake {
		if c.getLocalVelocity().X < 0 {
			baseVec = box2d.MakeB2Vec2(0, 1.3)
		} else {
			baseVec = box2d.MakeB2Vec2(0, 0.7)
		}
	} else {
		baseVec = box2d.MakeB2Vec2(0, 0)
	}

	fVec := box2d.MakeB2Vec2(c.power*baseVec.X, c.power*baseVec.Y)
	for _, w := range c.wheels {
		if w.powered {
			pos := w.body.GetWorldCenter()
			w.body.ApplyForce(w.body.GetWorldVector(fVec), pos, true)
		}
	}

	if c.getSpeedKMH() < 4 && c.AccelerateState == AccNone {
		c.setSpeed(0)
	}
}

const box2dScale = 1

func (c *Car) Draw(win *pixelgl.Window) {
	pos := box2dToPixel(c.body.GetPosition())
	x := pos.X  / box2dScale * 2
	y := pos.Y  / box2dScale * 2
	x1 := (pos.X + c.width/2) / box2dScale * 2
	y1 := (pos.Y + c.length/2) / box2dScale * 2
	x2 := (pos.X - c.width/2) / box2dScale * 2
	y2 := (pos.Y - c.length/2) / box2dScale * 2
	rot := c.body.GetAngle() * (180 / math.Pi)

	carBodySprite := imdraw.New(nil)
	carBodySprite.Color = pixel.RGB(0.5, 0.5, 0.5)
	carBodySprite.Push(
		pixel.V(x1, y1),
		pixel.V(x1, y2),
		pixel.V(x2, y2),
		pixel.V(x2, y1),
	)

	carBodySprite.SetMatrix(pixel.IM.Rotated(pixel.V(x, y), rot))

	carBodySprite.Polygon(0)
	carBodySprite.Draw(win)
}

func box2dToPixel(vec box2d.B2Vec2) pixel.Vec {
	return pixel.V(vec.X, vec.Y)
}

func pixelToBox2d(vec pixel.Vec) box2d.B2Vec2 {
	return box2d.MakeB2Vec2(vec.X, vec.Y)
}
