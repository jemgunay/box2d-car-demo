// Package car is a port of https://github.com/domasx2/gamejs-box2d-car-example
package car

import (
	"image/color"
	"math"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
	"github.com/lucasb-eyer/go-colorful"

	"github.com/jemgunay/box2d-car-demo/box"
)

// SteerState determines the direction the car is steering in.
type SteerState uint

// The possible vehicle steering states.
const (
	SteerNone SteerState = iota
	SteerLeft
	SteerRight
)

// AccDirectionState determines the direction the car should accelerate in.
type AccDirectionState string

// The possible acceleration direction states.
const (
	Forwards AccDirectionState = "forwards"
	Reverse  AccDirectionState = "backwards"
)

// HealthState determines the health state of the car and how it can be interacted with based on damage taken.
type HealthState uint

// The possible vehicle health states.
const (
	Healthy HealthState = iota
	Destroyed
	Detached
)

// Car is the base for a drivable physics-based car.
type Car struct {
	bodyDef       *box2d.B2BodyDef
	body          *box2d.B2Body
	frictionJoint *box2d.B2FrictionJoint

	steerState        SteerState
	accDirectionState AccDirectionState
	healthState       HealthState
	Braking           bool
	Accelerating      bool

	size          pixel.Vec
	colour        color.Color
	health        float64
	maxSteerAngle float64
	maxSpeed      float64
	power         float64
	wheelAngle    float64

	wheels []*Wheel
}

// NewCar creates and initialises a new car.
func NewCar(world *box2d.B2World, pos, size pixel.Vec) *Car {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = box2d.MakeB2Vec2(pos.X*box.WorldToBox2D, pos.Y*box.WorldToBox2D)
	bodyDef.Angle = 0
	bodyDef.LinearDamping = 0.15
	bodyDef.AngularDamping = 0.3

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*box.WorldToBox2D, size.Y*0.5*box.WorldToBox2D)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Density = 4.0
	fixDef.Friction = 0.3
	fixDef.Restitution = 0.4
	fixDef.Shape = shape
	fixDef.UserData = "car"
	fixDef.Filter.CategoryBits = box.CarCategory
	fixDef.Filter.MaskBits = box.WallCategory | box.CrateCategory
	//fixDef.Filter.GroupIndex = -4

	// create body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	car := &Car{
		bodyDef: bodyDef,
		body:    body,

		size:          size,
		maxSteerAngle: 20,
		maxSpeed:      40,
		power:         50,
		health:        100,
		colour:        pixel.RGB(0.5, 0.5, 0.5),

		steerState:        SteerNone,
		accDirectionState: Forwards,
		healthState:       Healthy,
	}

	// offset wheels to the ends of the car body
	wheelDeltaY := (size.Y * 0.6) / 2.0
	// scale wheel size relative to car body size
	wheelSize := size.Scaled(0.2)

	// bottom left
	car.AddWheel(world, pixel.V(-size.X/2.0, -wheelDeltaY), wheelSize, false, fixedRevolve)
	// bottom right
	car.AddWheel(world, pixel.V(size.X/2.0, -wheelDeltaY), wheelSize, false, fixedRevolve)
	// top left
	car.AddWheel(world, pixel.V(-size.X/2.0, wheelDeltaY), wheelSize, true, standardRevolve)
	// top right
	car.AddWheel(world, pixel.V(size.X/2.0, wheelDeltaY), wheelSize, true, standardRevolve)

	// create contact listener to process damage upon collisions
	world.SetContactListener(&carContactListener{
		car: car,
	})
	return car
}

type carContactListener struct {
	*box2d.B2ContactFilter
	car *Car
}

func (c *carContactListener) BeginContact(contact box2d.B2ContactInterface) {}

func (c *carContactListener) EndContact(contact box2d.B2ContactInterface) {}

func (c *carContactListener) PreSolve(contact box2d.B2ContactInterface, oldManifold box2d.B2Manifold) {
}

func (c *carContactListener) PostSolve(contact box2d.B2ContactInterface, impulse *box2d.B2ContactImpulse) {
	if contact.GetFixtureA().GetUserData() == "car" || contact.GetFixtureB().GetUserData() == "car" {
		absImpulse := math.Abs(impulse.TangentImpulses[0])
		// check impulse exceeds minimum required to cause damage
		if absImpulse > 3 && c.car.health > 0 {
			c.car.health -= absImpulse
			if c.car.health < 0 {
				c.car.health = 0
			}
		}
	}
}

// Destroy destroys the car, causing the wheels to disconnect.
func (c *Car) Destroy(world *box2d.B2World) {
	c.health = 0
	c.healthState = Destroyed

	filterData := c.body.GetFixtureList().GetFilterData()
	filterData.MaskBits = filterData.MaskBits | box.CarCategory
	c.body.GetFixtureList().SetFilterData(filterData)

	jointDef := box2d.MakeB2FrictionJointDef()
	jointDef.Initialize(c.body, box.MainGround.Body, c.body.GetWorldCenter())
	jointDef.MaxForce = 10.0
	jointDef.MaxTorque = 5.0
	world.CreateJoint(&jointDef)

	for _, wheel := range c.wheels {
		if wheel.healthState != Healthy {
			continue
		}

		filterData := wheel.body.GetFixtureList().GetFilterData()
		filterData.MaskBits = filterData.MaskBits | box.CarCategory
		wheel.body.GetFixtureList().SetFilterData(filterData)

		// create friction joint to simulate top down friction
		jointDef := box2d.MakeB2FrictionJointDef()
		jointDef.Initialize(wheel.body, box.MainGround.Body, wheel.body.GetWorldCenter())
		jointDef.MaxForce = 0.2
		jointDef.MaxTorque = 0.2
		world.CreateJoint(&jointDef)

		world.DestroyJoint(wheel.joint)

		wheel.healthState = Detached
	}
}

// AddWheel creates a wheel and joins it to the parent car.
func (c *Car) AddWheel(world *box2d.B2World, relativePos, size pixel.Vec, powered bool, revolveType wheelRevolveType) {
	// create rigid body definition
	bodyDef := box2d.NewB2BodyDef()
	bodyDef.Type = box2d.B2BodyType.B2_dynamicBody
	bodyDef.Position = c.body.GetWorldPoint(box2d.MakeB2Vec2(relativePos.X*box.WorldToBox2D, relativePos.Y*box.WorldToBox2D))
	bodyDef.Angle = c.body.GetAngle()
	bodyDef.LinearDamping = 0.15
	bodyDef.AngularDamping = 0.3

	// create fixture shape
	shape := box2d.NewB2PolygonShape()
	shape.SetAsBox(size.X*0.5*box.WorldToBox2D, size.Y*0.5*box.WorldToBox2D)

	// create fixture
	fixDef := box2d.MakeB2FixtureDef()
	fixDef.Density = 1.0
	fixDef.Friction = 0.3
	fixDef.Restitution = 0.4
	fixDef.Shape = shape
	fixDef.UserData = "wheel"
	fixDef.Filter.CategoryBits = box.CarCategory
	fixDef.Filter.MaskBits = box.WallCategory | box.CrateCategory

	// create body
	body := world.CreateBody(bodyDef)
	body.CreateFixtureFromDef(&fixDef)

	wheel := &Wheel{
		parentCar: c,

		bodyDef: box2d.NewB2BodyDef(),
		body:    body,

		pos:         relativePos,
		size:        size,
		colour:      pixel.RGB(0.3, 0.3, 0.3),
		powered:     powered,
		revolveType: revolveType,
	}

	// joint wheel to car body
	if revolveType == fixedRevolve {
		jointDef := box2d.MakeB2PrismaticJointDef()
		jointDef.Initialize(c.body, body, body.GetWorldCenter(), box2d.MakeB2Vec2(1, 0))
		jointDef.EnableLimit = true
		jointDef.LowerTranslation = 0
		jointDef.UpperTranslation = 0
		wheel.joint = world.CreateJoint(&jointDef)
	} else {
		jointDef := box2d.MakeB2RevoluteJointDef()
		jointDef.Initialize(c.body, body, body.GetWorldCenter())
		jointDef.EnableMotor = true
		wheel.joint = world.CreateJoint(&jointDef)
	}

	c.wheels = append(c.wheels, wheel)
}

// returns car's velocity vector relative to the car
func (c *Car) getLocalVelocity() box2d.B2Vec2 {
	return c.body.GetLocalVector(c.body.GetLinearVelocityFromLocalPoint(box2d.MakeB2Vec2(0, 0)))
}

// GetSpeedKMH gets the car's speed in kilometers per hour.
func (c *Car) GetSpeedKMH() float64 {
	velocity := c.body.GetLinearVelocity()
	return (velocity.Length() / 1000.0) * 3600.0
}

// set speed in kilometers per hour
func (c *Car) setSpeedKMH(speed float64) {
	vel := box.ToPixelVec(c.body.GetLinearVelocity())
	vel = vel.Unit().Scaled((speed * 1000.0) / 3600.0)
	c.body.SetLinearVelocity(box.ToBox2DVec(vel))
}

// ToggleDirection toggles the direction the car should accelerate in, between forwards and backwards.
func (c *Car) ToggleDirection() AccDirectionState {
	if c.accDirectionState == Forwards {
		c.accDirectionState = Reverse
	} else {
		c.accDirectionState = Forwards
	}
	return c.accDirectionState
}

// SetSteerState sets the car's steer state.
func (c *Car) SetSteerState(state SteerState) {
	c.steerState = state
}

// Update moves the car based on its current state.
func (c *Car) Update(world *box2d.B2World, dt float64) {
	// calculate the change in wheel's angle for this update, assuming the wheel will reach is maximum angle from zero
	// in 200 ms
	steerDelta := (c.maxSteerAngle / 200.0) * dt
	if c.steerState == SteerRight {
		// increment angle without going over max steer
		c.wheelAngle = math.Min(c.wheelAngle+steerDelta, c.maxSteerAngle)
	} else if c.steerState == SteerLeft {
		// decrement angle without going over max steer
		c.wheelAngle = math.Max(c.wheelAngle-steerDelta, -c.maxSteerAngle)
	} else if c.wheelAngle < 0 {
		// ease wheels from left to centre
		c.wheelAngle = math.Min(c.wheelAngle+steerDelta, 0)
	} else if c.wheelAngle > 0 {
		// ease wheels from right to centre
		c.wheelAngle = math.Max(c.wheelAngle-steerDelta, 0)
	}

	var baseVec pixel.Vec
	// handle acceleration
	if c.Accelerating && c.GetSpeedKMH() < c.maxSpeed && c.health > 0 {
		if c.accDirectionState == Forwards {
			// forwards
			baseVec = pixel.V(0, 1)
		} else {
			// reverse (slower)
			baseVec = pixel.V(0, -0.6)
		}
	}

	// handle braking
	if c.Braking {
		if c.getLocalVelocity().Y > 0 {
			baseVec = pixel.V(0, -1)
		} else if c.getLocalVelocity().Y < 0 {
			baseVec = pixel.V(0, 1)
		}
	}

	// multiply by engine power, which gives us a force vector relative to the wheel
	forceVec := box.ToBox2DVec(baseVec.Scaled(c.power * dt / 10.0))

	for _, wheel := range c.wheels {
		if wheel.healthState != Healthy {
			continue
		}

		// kill sideways velocity for all wheels
		wheel.killSidewaysVelocity()

		// update revolving wheels
		if wheel.revolveType == standardRevolve {
			wheel.setAngle(box.DegToRad(c.wheelAngle))
		} else if wheel.revolveType == inverseRevolve {
			wheel.setAngle(box.DegToRad(-c.wheelAngle))
		}
		// apply force to each powered wheel
		if wheel.powered {
			pos := wheel.body.GetWorldCenter()
			wheel.body.ApplyForce(wheel.body.GetWorldVector(forceVec), pos, true)
		}
	}

	// if going very slow, stop in order to prevent endless sliding
	if !c.Accelerating && c.GetSpeedKMH() < 1 {
		c.setSpeedKMH(0)
	}

	// process health state
	if c.healthState != Destroyed && c.health <= 0 {
		// destroy car if health not above 0
		c.Destroy(world)
	}

	// update colour to reflect health
	c.colour = colorful.Hsl(c.health*1.2, 1, 0.5)
}

// Draw draws the car and its wheels.
func (c *Car) Draw(win *pixelgl.Window) {
	// draw wheels
	for _, wheel := range c.wheels {
		box.DrawRectBody(win, box.ToPixelVec(wheel.body.GetPosition()), wheel.size, wheel.body.GetAngle(), wheel.colour)
	}

	box.DrawRectBody(win, box.ToPixelVec(c.body.GetPosition()), c.size, c.body.GetAngle(), c.colour)
}

type wheelRevolveType uint

// The wheel rotation direction states.
const (
	fixedRevolve wheelRevolveType = iota
	standardRevolve
	inverseRevolve
)

// Wheel is a physics-based vehicle wheel.
type Wheel struct {
	parentCar *Car

	bodyDef *box2d.B2BodyDef
	body    *box2d.B2Body
	joint   box2d.B2JointInterface

	pos, size   pixel.Vec
	colour      color.Color
	powered     bool
	revolveType wheelRevolveType
	healthState HealthState
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
	return box.Rotate(dirVec, w.body.GetAngle())
}

// substracts sideways velocity from this wheel's velocity vector and returns the remaining front-facing velocity vector.
func (w *Wheel) getKillVelocityVector() box2d.B2Vec2 {
	sidewaysAxis := w.getDirectionVector()
	dotProd := box.ToPixelVec(w.body.GetLinearVelocity()).Dot(sidewaysAxis)
	return box.ToBox2DVec(sidewaysAxis.Scaled(dotProd))
}

// removes all sideways velocity from this wheel's velocity
func (w *Wheel) killSidewaysVelocity() {
	kv := w.getKillVelocityVector()
	w.body.SetLinearVelocity(kv)
}
