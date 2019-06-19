package main

import (
	"fmt"
	"image/color"
	"time"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
	"github.com/jemgunay/box2d-car-test/box"
)

func main() {
	pixelgl.Run(func() {
		start()
	})
}

func start() {
	// create window config
	cfg := pixelgl.WindowConfig{
		Title:     "Box2D Car Test",
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

	world := box2d.MakeB2World(box2d.MakeB2Vec2(0, 0))
	//world := box2d.MakeB2World(box2d.MakeB2Vec2(0, -9.8))

	// create car
	mainCar := box.NewCar(&world, win.Bounds().Center().X, win.Bounds().Center().Y, 2, 4)

	// create wall props
	walls := []*box.Wall{
		box.NewWall(&world, win.Bounds().Center().X, win.Bounds().Min.Y+100, win.Bounds().W(), 50),
		box.NewWall(&world, win.Bounds().Min.X+300, win.Bounds().Center().Y+100, 50, win.Bounds().H()),
		box.NewWall(&world, win.Bounds().Max.X-300, win.Bounds().Center().Y+100, 50, win.Bounds().H()),
	}

	crates := []*box.Crate{
		box.NewCrate(&world, win.Bounds().Center().X, win.Bounds().Min.Y+200, 30, 30),
		box.NewCrate(&world, win.Bounds().Center().X-15, win.Bounds().Min.Y+250, 30, 30),
		box.NewCrate(&world, win.Bounds().Center().X-30, win.Bounds().Min.Y+300, 30, 30),
	}

	// limit update cycles FPS
	//frameRateLimiter := time.Tick(time.Second / 120)
	prevTimestamp := time.Now().UTC()

	// main game loop
	for !win.Closed() {
		dt := float64(time.Since(prevTimestamp).Nanoseconds())
		prevTimestamp = time.Now().UTC()

		// handle keyboard input
		if win.JustPressed(pixelgl.KeyEscape) {
			return
		}

		if win.Pressed(pixelgl.KeyA) {
			mainCar.SteerState = box.SteerLeft
		} else if win.Pressed(pixelgl.KeyD) {
			mainCar.SteerState = box.SteerRight
		} else {
			mainCar.SteerState = box.SteerNone
		}

		if win.Pressed(pixelgl.KeyS) {
			mainCar.AccelerateState = box.AccBrake
		} else if win.Pressed(pixelgl.KeyW) {
			mainCar.AccelerateState = box.AccAccelerate
		} else {
			mainCar.AccelerateState = box.AccNone
		}

		dt = (1.0 / 60.0) * 1000 // ms
		mainCar.Update(dt)

		world.Step(dt/1000.0, 8, 3)
		world.ClearForces()

		// draw window
		win.Clear(color.White)
		for _, wall := range walls {
			wall.Draw(win)
		}
		for _, crate := range crates {
			crate.Draw(win)
		}
		mainCar.Draw(win)
		win.Update()

		//<-frameRateLimiter
	}
}
