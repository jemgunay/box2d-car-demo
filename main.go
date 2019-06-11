package main

import (
	"fmt"
	"image/color"
	"time"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
	"github.com/jemgunay/box2d-car-test/car"
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

	mainCar := car.NewCar(&world, 20, 40)

	// limit update cycles FPS
	frameRateLimiter := time.Tick(time.Second / 120)
	prevTimestamp := time.Now().UTC()

	// main game loop
	for !win.Closed() {
		dt := float64(time.Since(prevTimestamp).Nanoseconds())/1000
		prevTimestamp = time.Now().UTC()

		// handle keyboard input
		if win.JustPressed(pixelgl.KeyEscape) {
			return
		}

		if win.Pressed(pixelgl.KeyA) {
			mainCar.SteerState = car.SteerLeft
		} else if win.Pressed(pixelgl.KeyD) {
			mainCar.SteerState = car.SteerRight
		} else {
			mainCar.SteerState = car.SteerNone
		}

		if win.Pressed(pixelgl.KeyS) {
			mainCar.AccelerateState = car.AccBrake
		} else if win.Pressed(pixelgl.KeyW) {
			mainCar.AccelerateState = car.AccAccelerate
		} else {
			mainCar.AccelerateState = car.AccNone
		}

		mainCar.Update(dt)

		world.Step(dt/1000, 10, 8)
		world.ClearForces()

		// draw window
		win.Clear(color.White)
		mainCar.Draw(win)
		win.Update()

		<-frameRateLimiter
	}
}
