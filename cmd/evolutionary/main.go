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
	"github.com/faiface/pixel/imdraw"
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
	Forward
	Left
	Right
	Brake
)

func start() {
	rand.Seed(time.Now().UnixNano())

	population, err := genetics.NewPopulation(10, 10, []genetics.Option{Nothing, Forward, Left, Right, Brake})
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
	initialCarPos := pixel.V(winCentre.X, 100)
	mainCar := car.NewCar(&world, initialCarPos, pixel.V(38, 80))

	// create wall props
	walls := []*box.Wall{
		box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Min.Y), pixel.V(win.Bounds().W(), 30)),
		box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Max.Y), pixel.V(win.Bounds().W(), 30)),
		box.NewWall(&world, pixel.V(win.Bounds().Min.X, winCentre.Y), pixel.V(30, win.Bounds().H())),
		box.NewWall(&world, pixel.V(win.Bounds().Max.X, winCentre.Y), pixel.V(30, win.Bounds().H())),
	}

	targetPos := win.Bounds().Max.Sub(pixel.V(200, 200))
	const (
		realtimeWorldStep    = 10.0
		acceleratedWorldStep = 10.0
	)
	worldStepScaler := realtimeWorldStep
	imd := imdraw.New(nil)

	// main game render and physics step loop
	done := make(chan struct{})
	carDelete := make(chan struct{})
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

			// concurrently recreate car
			select {
			case <-carDelete:
				mainCar.Destroy()
				mainCar = car.NewCar(&world, initialCarPos, pixel.V(38, 80))
			default:
			}

			mainCar.Update(&world, dt)

			world.Step(dt/worldStepScaler, 8, 3)
			world.ClearForces()

			// draw window
			win.Clear(color.White)
			imd.Clear()

			for _, wall := range walls {
				wall.Draw(imd)
			}

			// draw target for car to reach
			box.DrawCircleBody(imd, targetPos, 40.0, pixel.ToRGBA(color.RGBA{204, 255, 153, 255}))

			mainCar.Draw(imd)
			imd.Draw(win)
			win.Update()

			<-frameRateLimiter
		}
	}()

	// perform evolution iterations
	go func() {
		for i := 0; i < 3000; i++ {
			population.PerformSelection()
			fmt.Printf("========= %d =========\n", population.Iteration)

			for _, s := range population.Sequences {
				// reset car for this sequence
				carDelete <- struct{}{}

				// run sequence through fitness function
				for _, v := range s.Data {
					switch v {
					case Nothing:
						// do nothing/stop accelerating
					case Forward:
						mainCar.Accelerating = true
						mainCar.SetSteerState(car.SteerNone)
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

					//time.Sleep(time.Millisecond * 1000 / time.Duration(worldStepScaler)) // 100ms
					time.Sleep(time.Millisecond * 5) // 100ms

					// reset states
					mainCar.SetSteerState(car.SteerNone)
					mainCar.Accelerating = false
					mainCar.Braking = false
				}

				// determine fitness of sequence
				s.FitnessValue = mainCar.Pos().Sub(targetPos).Len()
				population.FitnessSum += s.FitnessValue

				fmt.Printf("%v -> %f\n", s.Data, s.FitnessValue)
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
