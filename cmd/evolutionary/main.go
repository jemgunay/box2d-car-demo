// Package main sets up the evolutionary world.
package main

import (
	"flag"
	"fmt"
	"image/color"
	"math/rand"
	"strconv"
	"strings"
	"time"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/imdraw"
	"github.com/faiface/pixel/pixelgl"
	"github.com/jemgunay/box2d-car-demo/genetics"
	"github.com/pkg/errors"

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

var (
	populationSize = 10
	sequenceSize   = 20

	win   *pixelgl.Window
	world box2d.B2World

	mainCar                  *car.Car
	walls                    []*box.Wall
	targetPos, initialCarPos pixel.Vec

	done         = make(chan struct{})
	carResetChan = make(chan struct{})
)

func processSequenceString(sequence string) ([]genetics.Option, error) {
	if sequence == "" {
		return nil, nil
	}
	items := strings.Split(sequence, " ")
	if len(items) < 1 {
		return nil, errors.New("invalid sequence provided")
	}
	// parse each sequence element into byte
	processedSeqInput := make([]genetics.Option, 0, len(items))
	for i, item := range items {
		num, err := strconv.Atoi(item)
		if err != nil {
			return nil, fmt.Errorf("failed to process %dth sequence value (%s): %s", i+1, item, err)
		}
		processedSeqInput = append(processedSeqInput, genetics.Option(num))
	}
	return processedSeqInput, nil
}

func start() {
	// parse flags
	numIterations := flag.Int("iterations", 1000, "number of evolution iterations")
	flag.IntVar(&sequenceSize, "seq_size", sequenceSize, "sequence size of each competitor")
	sequenceInput := flag.String("seq", "", "play back a predetermined sequence (space separated integers)")
	seed := flag.Int64("seed", -1, "random number generator seed (random if not provided)")
	level := flag.Int64("level", 1, "the level to load on startup (1=basic, 2=obstacle)")
	flag.Parse()

	// process command line flag sequence input
	playbackSequence, err := processSequenceString(*sequenceInput)
	if err != nil {
		fmt.Printf("failed to process sequence flag value: %s\n", err)
		return
	}

	// seed rand generator
	if *seed == -1 {
		*seed = time.Now().UnixNano()
	}
	rand.Seed(*seed)

	// create window config
	cfg := pixelgl.WindowConfig{
		Title:     "Car Evolution",
		Bounds:    pixel.R(0, 0, 1080, 720),
		VSync:     false,
		Resizable: true,
	}

	// create window
	win, err = pixelgl.NewWindow(cfg)
	if err != nil {
		fmt.Printf("failed create new window: %s\n", err)
		return
	}

	// create Box2D world
	world = box2d.MakeB2World(box2d.MakeB2Vec2(0, 0))
	// enable contact filter
	world.SetContactFilter(&box2d.B2ContactFilter{})
	winCentre := win.Bounds().Center()

	initialCarPos = pixel.V(win.Bounds().Center().X, 100)
	targetPos = win.Bounds().Max.Sub(pixel.V(200, 200))

	// create ground
	box.MainGround = box.NewGround(&world, winCentre, win.Bounds().Size())

	// create wall props
	walls = []*box.Wall{
		box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Min.Y), pixel.V(win.Bounds().W(), 30)), // bottom
		//box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Max.Y), pixel.V(win.Bounds().W(), 30)), // top
		box.NewWall(&world, pixel.V(win.Bounds().Min.X, winCentre.Y), pixel.V(30, win.Bounds().H())), // left
		//box.NewWall(&world, pixel.V(win.Bounds().Max.X, winCentre.Y), pixel.V(30, win.Bounds().H())), // right
	}

	// create obstacle
	if *level == 2 {
		obstacleSize := pixel.V(50, 300)
		walls = append(
			walls,
			box.NewWall(&world, pixel.V(winCentre.X-250-obstacleSize.X/2, obstacleSize.Y/2), obstacleSize),
			box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Max.Y-obstacleSize.Y/2), obstacleSize),
		)

		initialCarPos = win.Bounds().Min.Add(pixel.V(125, 100))
	}

	// create car
	resetCar()

	// start main game render and physics step loop
	go stepAndDraw()

	go func() {
		// basic sequence playback
		if playbackSequence != nil {
			for {
				// execute the sequence provided via the flag
				executeSequence(playbackSequence)

				// reset car for this sequence
				carResetChan <- struct{}{}
			}
		}

		// perform evolution iterations - create initial population
		population, err := genetics.NewPopulation(populationSize, sequenceSize, []genetics.Option{Nothing, Forward, Left, Right, Brake})
		if err != nil {
			fmt.Printf("failed to create initial population: %s\n", err)
			return
		}

		for i := 0; i < *numIterations; i++ {
			population.PerformSelection()
			fmt.Printf("=================== %d ===================\n", population.Iteration)

			for _, s := range population.Sequences {
				// reset car for this sequence
				carResetChan <- struct{}{}

				// perform a series of car movements that reflect the provided sequence
				executeSequence(s.Data)

				// determine fitness of sequence, consisting of distance between car and target, as well as the final
				// velocity by the end of the sequence
				finalDist := mainCar.Pos().Sub(targetPos).Len()
				finalVel := mainCar.GetSpeedKMH() * 5
				s.FitnessValue = finalDist + finalVel
				population.FitnessSum += s.FitnessValue

				fmt.Printf("%v -> %.2f (d=%.2f, v=%.2f)\n", s.Data, s.FitnessValue, finalDist, finalVel)
			}
		}
	}()

	<-done
}

func executeSequence(sequence []genetics.Option) {
	// run sequence through fitness function
	for _, v := range sequence {
		switch v {
		case Forward:
			// forwards, no steering
			mainCar.Accelerating = true
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
		time.Sleep(time.Millisecond * 200)

		// reset states
		mainCar.SetSteerState(car.SteerNone)
		mainCar.Accelerating = false
		mainCar.Braking = false
	}
}

func resetCar() {
	if mainCar != nil {
		mainCar.Destroy()
	}
	mainCar = car.NewCar(&world, initialCarPos, pixel.V(38, 80))
}

const (
	realtimeWorldStep    = 1000.0
	acceleratedWorldStep = 10.0
)

func stepAndDraw() {
	var (
		// draw sprites onto this
		imd = imdraw.New(nil)
		// limit update cycles FPS
		frameRateLimiter = time.Tick(time.Second / 120)
		prevTimestamp    = time.Now()
		// used to control speed of simulation
		worldStepScaler = realtimeWorldStep
	)

	for !win.Closed() {
		dt := float64(time.Since(prevTimestamp).Nanoseconds()) / 1e6
		prevTimestamp = time.Now()

		// handle keyboard input
		if win.JustPressed(pixelgl.KeyEscape) {
			close(done)
			return
		}

		// concurrently recreate car to avoid nil pointer panic
		select {
		case <-carResetChan:
			resetCar()
		default:
		}

		// world step
		mainCar.Update(&world, dt)
		world.Step(dt/worldStepScaler, 8, 3)
		world.ClearForces()

		// clear window
		win.Clear(color.White)
		imd.Clear()

		// drawing to window
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
}
