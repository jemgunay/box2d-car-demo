package box

import (
	"image/color"
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
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
	AccNone accState = iota
	AccAccelerate
	AccBrake
)

type Car struct {
	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body

	SteerState      steerState
	AccelerateState accState

	size          pixel.Vec
	colour        color.Color
	maxSteerAngle float64
	maxSpeed      float64
	power         float64
	wheelAngle    float64

	wheels []*Wheel
}

func NewCar(w *box2d.B2World, pos, size pixel.Vec) *Car {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = box2d.MakeB2Vec2(pos.X*worldToBox2d, pos.Y*worldToBox2d)
	bodyDef.Angle = degToRad(180)
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
	body := w.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	car := &Car{
		bodyDef: bodyDef,
		body:    body,

		size:          size,
		colour:        pixel.RGB(0.5, 0.5, 0.5),
		maxSteerAngle: 20,
		maxSpeed:      20,
		power:         20,

		SteerState:      SteerNone,
		AccelerateState: AccNone,
	}

	// offset wheels to the ends of the car body
	wheelDeltaY := (size.Y * 0.6) / 2.0
	// scale wheel size relative to car body size
	wheelSize := size.Scaled(0.2)

	// top left
	car.AddWheel(w, pixel.V(-size.X/2.0, -wheelDeltaY), wheelSize, true, true)
	// top right
	car.AddWheel(w, pixel.V(size.X/2.0, -wheelDeltaY), wheelSize, true, true)
	// back left
	car.AddWheel(w, pixel.V(-size.X/2.0, wheelDeltaY), wheelSize, false, false)
	// back right
	car.AddWheel(w, pixel.V(size.X/2.0, wheelDeltaY), wheelSize, false, false)

	return car
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
func (w *Wheel) getDirectionVector(carBody *box2d.B2Body) box2d.B2Vec2 {
	//return vectors.rotate((this.getLocalVelocity()[1]>0) ? [0, 1]:[0, -1] , this.body.GetAngle()) ;
	var dirVec box2d.B2Vec2
	if w.getLocalVelocity(carBody).Y > 0 {
		dirVec = box2d.MakeB2Vec2(0, 1)
	} else {
		dirVec = box2d.MakeB2Vec2(0, -1)
	}
	// https://github.com/GameJs/gamejs/blob/master/src/gamejs/math/vectors.js#L85
	return rotate(dirVec, w.body.GetAngle())
}

// substracts sideways velocity from this wheel's velocity vector and returns the remaining front-facing velocity vector.
func (w *Wheel) getKillVelocityVector() box2d.B2Vec2 {
	vel := w.body.GetLinearVelocity()
	sidewaysAxis := box2dToPixel(w.getDirectionVector(w.parentCar.body))
	dotProd := box2dToPixel(vel).Dot(sidewaysAxis)
	return box2d.MakeB2Vec2(sidewaysAxis.X*dotProd, sidewaysAxis.Y*dotProd)
}

// removes all sideways velocity from this wheel's velocity
func (w *Wheel) killSidewaysVelocity() {
	kv := w.getKillVelocityVector()
	w.body.SetLinearVelocity(kv)
}

func (w *Wheel) Draw(win *pixelgl.Window) {
	// convert to pixel vector and scale to real world co-ordinates
	posCentre := box2dToPixel(w.body.GetPosition()).Scaled(box2dToWorld)
	// offset from centre to bottom left
	posOffset := posCentre.Sub(w.size.Scaled(0.5))

	wheelSprite := imdraw.New(nil)
	wheelSprite.Color = w.colour
	wheelSprite.Push(
		pixel.V(posOffset.X, posOffset.Y),
		pixel.V(posOffset.X, posOffset.Y+w.size.Y),
		pixel.V(posOffset.X+w.size.X, posOffset.Y+w.size.Y),
		pixel.V(posOffset.X+w.size.X, posOffset.Y),
	)

	wheelSprite.SetMatrix(pixel.IM.Rotated(posCentre, w.body.GetAngle()))
	wheelSprite.Polygon(0)
	wheelSprite.Draw(win)
}

// returns car's velocity vector relative to the car
func (c *Car) getLocalVelocity() box2d.B2Vec2 {
	return c.body.GetLocalVector(c.body.GetLinearVelocityFromLocalPoint(box2d.MakeB2Vec2(0, 0)))
}

func (c *Car) AddWheel(world *box2d.B2World, pos, size pixel.Vec, powered, revolving bool) {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = c.body.GetWorldPoint(box2d.MakeB2Vec2(pos.X*worldToBox2d, pos.Y*worldToBox2d))
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

		pos:       pos,
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
func (c *Car) setSpeed(speed float64) {
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

	var baseVec box2d.B2Vec2
	// if accelerator is pressed down and speed limit has not been reached, go forwards
	if (c.AccelerateState == AccAccelerate) && (c.getSpeedKMH() < c.maxSpeed) {
		baseVec = box2d.MakeB2Vec2(0, -1)
	} else if c.AccelerateState == AccBrake {
		if c.getLocalVelocity().Y < 0 {
			// braking, but still moving forwards - increased force
			baseVec = box2d.MakeB2Vec2(0, 1.3)
		} else {
			// going in reverse - less force
			baseVec = box2d.MakeB2Vec2(0, 0.7)
		}
	}

	fVec := box2d.MakeB2Vec2(c.power*baseVec.X, c.power*baseVec.Y)
	for _, wheel := range c.wheels {
		// update revolving wheels
		if wheel.revolving {
			wheel.setAngle(degToRad(c.wheelAngle))
		}
		// apply force to each wheel
		if wheel.powered {
			pos := wheel.body.GetWorldCenter()
			wheel.body.ApplyForce(wheel.body.GetWorldVector(fVec), pos, true)
		}
	}

	// if going very slow, stop in order to prevent endless sliding
	if c.getSpeedKMH() < 4 && c.AccelerateState == AccNone {
		c.setSpeed(0)
	}
}

func (c *Car) Draw(win *pixelgl.Window) {
	// convert to pixel vector and scale to real world co-ordinates
	posCentre := box2dToPixel(c.body.GetPosition()).Scaled(box2dToWorld)
	// offset from centre to bottom left
	posOffset := posCentre.Sub(c.size.Scaled(0.5))

	carBodySprite := imdraw.New(nil)
	carBodySprite.Color = c.colour
	carBodySprite.Push(
		pixel.V(posOffset.X, posOffset.Y),
		pixel.V(posOffset.X, posOffset.Y+c.size.Y),
		pixel.V(posOffset.X+c.size.X, posOffset.Y+c.size.Y),
		pixel.V(posOffset.X+c.size.X, posOffset.Y),
	)

	carBodySprite.SetMatrix(pixel.IM.Rotated(posCentre, c.body.GetAngle()))
	carBodySprite.Polygon(0)
	carBodySprite.Draw(win)

	// draw wheels
	for _, wheel := range c.wheels {
		wheel.Draw(win)
	}
}
