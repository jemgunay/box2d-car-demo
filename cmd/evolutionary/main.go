// Package main sets up the evolutionary world.
package main

import (
	"fmt"
	"image/color"
	"math/rand"
	"sort"
	"time"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
	"github.com/jemgunay/box2d-car-demo/genetics"

	"github.com/jemgunay/box2d-car-demo/box"
	"github.com/jemgunay/box2d-car-demo/car"
)

func main() {
	pixelgl.Run(start)
}

const (
	Nothing genetics.Option = iota
	Left
	Right
	Brake
)

func start() {
	rand.Seed(time.Now().UnixNano())

	population, err := genetics.NewPopulation(10, 10, []genetics.Option{Nothing, Left, Right, Brake})
	if err != nil {
		fmt.Printf("failed to create initial population: %s\n", err)
		return
	}

	// create window config
	cfg := pixelgl.WindowConfig{
		Title:     "Car Evolution",
		Bounds:    pixel.R(0, 0, 1080, 720),
		VSync:     false,
		Resizable: true,
	}

	// create window
	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		fmt.Printf("failed create new window: %s\n", err)
		return
	}

	// create Box2D world
	world := box2d.MakeB2World(box2d.MakeB2Vec2(0, 0))
	winCentre := win.Bounds().Center()

	// enable contact filter
	world.SetContactFilter(&box2d.B2ContactFilter{})

	// create ground
	box.MainGround = box.NewGround(&world, winCentre, win.Bounds().Size())

	// create car
	mainCar := car.NewCar(&world, winCentre, pixel.V(38, 80))

	// create wall props
	walls := []*box.Wall{
		box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Min.Y), pixel.V(win.Bounds().W(), 30)),
		box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Max.Y), pixel.V(win.Bounds().W(), 30)),
		box.NewWall(&world, pixel.V(win.Bounds().Min.X, winCentre.Y), pixel.V(30, win.Bounds().H())),
		box.NewWall(&world, pixel.V(win.Bounds().Max.X, winCentre.Y), pixel.V(30, win.Bounds().H())),
	}

	// create crates
	crateSize := pixel.V(50, 50)
	crates := []*box.Crate{
		box.NewCrate(&world, pixel.V(winCentre.X, win.Bounds().Min.Y+250), crateSize),
		box.NewCrate(&world, pixel.V(winCentre.X-30, win.Bounds().Min.Y+190), crateSize),
		box.NewCrate(&world, pixel.V(winCentre.X+30, win.Bounds().Min.Y+190), crateSize),
		box.NewCrate(&world, pixel.V(winCentre.X-60, win.Bounds().Min.Y+130), crateSize),
		box.NewCrate(&world, pixel.V(winCentre.X+60, win.Bounds().Min.Y+130), crateSize),
		box.NewCrate(&world, pixel.V(winCentre.X, win.Bounds().Min.Y+130), crateSize),
	}

	const (
		realtimeWorldStep    = 1000.0
		acceleratedWorldStep = 10.0
	)
	worldStepScaler := acceleratedWorldStep
	targetPos := pixel.V(0, 0)

	// main game render and physics step loop
	done := make(chan struct{})
	go func() {
		// limit update cycles FPS
		frameRateLimiter := time.Tick(time.Second / 120)
		prevTimestamp := time.Now()

		for !win.Closed() {
			dt := float64(time.Since(prevTimestamp).Nanoseconds()) / 1e6
			prevTimestamp = time.Now()

			// handle keyboard input
			if win.JustPressed(pixelgl.KeyEscape) {
				close(done)
				return
			}

			mainCar.Update(&world, dt)

			world.Step(dt/worldStepScaler, 8, 3)
			world.ClearForces()

			// draw window
			win.Clear(color.White)
			for _, wall := range walls {
				wall.Draw(win)
			}
			for _, crate := range crates {
				crate.Draw(win)
			}
			box.DrawCircleBody(win, targetPos, 20.0, pixel.RGB(224, 187, 228))
			mainCar.Draw(win)
			win.Update()

			<-frameRateLimiter
		}
	}()

	// perform evolution iterations
	go func() {
		fmt.Printf("0)\n%s\n\n", population)
		for i := 0; i < 30; i++ {
			population.PerformSelection()
			fmt.Printf("%d)\n%s\n", population.Iteration, population)

			population.FitnessSum = 0
			for _, s := range population.Sequences {
				// reset car for this sequence
				mainCar = car.NewCar(&world, winCentre, pixel.V(38, 80))

				// run sequence through fitness function
				for _, v := range s.Data {
					switch v {
					case Nothing:
						// do nothing/stop accelerating
					case Left:
						// forwards and left
						mainCar.Accelerating = true
						mainCar.SetSteerState(car.SteerLeft)
					case Right:
						// forwards and right
						mainCar.Accelerating = true
						mainCar.SetSteerState(car.SteerRight)
					case Brake:
						// brake
						mainCar.Braking = true
					}

					time.Sleep(time.Millisecond * 1000 / time.Duration(worldStepScaler)) // 100ms

					// reset states
					mainCar.SetSteerState(car.SteerNone)
					mainCar.Accelerating = false
					mainCar.Braking = false
				}

				// determine fitness of sequence
				s.FitnessValue = mainCar.Pos().To(targetPos).Normal().Len()
				population.FitnessSum += s.FitnessValue
			}
		}

		// output ordered results
		sort.Slice(population.Sequences, func(i, j int) bool {
			return population.Sequences[i].FitnessValue < population.Sequences[j].FitnessValue
		})
		for _, s := range population.Sequences {
			fmt.Printf("%v [%v]\n", s.Data, s.FitnessValue)
		}
	}()

	<-done
}
