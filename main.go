package main

import (
	"fmt"
	"image/color"
	"time"

	"github.com/ByteArena/box2d"
	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
	"github.com/jemgunay/box2d-car-demo/box"
)

func main() {
	pixelgl.Run(func() {
		start()
	})
}

func start() {
	// create window config
	cfg := pixelgl.WindowConfig{
		Title:     "Box2D Car Demo",
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
	winCentre := win.Bounds().Center()

	// create car
	mainCar := box.NewCar(&world, winCentre, pixel.V(40, 80))

	// create wall props
	walls := []*box.Wall{
		box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Min.Y), pixel.V(win.Bounds().W(), 30)),
		box.NewWall(&world, pixel.V(winCentre.X, win.Bounds().Max.Y), pixel.V(win.Bounds().W(), 30)),
		box.NewWall(&world, pixel.V(win.Bounds().Min.X, winCentre.Y), pixel.V(30, win.Bounds().H())),
		box.NewWall(&world, pixel.V(win.Bounds().Max.X, winCentre.Y), pixel.V(30, win.Bounds().H())),
	}

	crates := []*box.Crate{
		box.NewCrate(&world, pixel.V(winCentre.X, win.Bounds().Min.Y+200), pixel.V(50, 50)),
		box.NewCrate(&world, pixel.V(winCentre.X-100, win.Bounds().Min.Y+250), pixel.V(50, 50)),
		box.NewCrate(&world, pixel.V(winCentre.X+100, win.Bounds().Min.Y+300), pixel.V(50, 50)),
	}

	// limit update cycles FPS
	frameRateLimiter := time.Tick(time.Second / 120)
	prevTimestamp := time.Now().UTC()

	// main game loop
	for !win.Closed() {
		dt := float64(time.Since(prevTimestamp).Nanoseconds()) / 1e6
		prevTimestamp = time.Now().UTC()

		// handle keyboard input
		if win.JustPressed(pixelgl.KeyEscape) {
			return
		}

		mainCar.SteerState = box.SteerNone
		if win.Pressed(pixelgl.KeyA) {
			mainCar.SteerState = box.SteerLeft
		}
		if win.Pressed(pixelgl.KeyD) {
			mainCar.SteerState = box.SteerRight
		}

		if win.JustPressed(pixelgl.KeyQ) {
			if mainCar.AccState == box.AccForwards {
				mainCar.AccState = box.AccReverse
				fmt.Println("Reversing")
			} else {
				mainCar.AccState = box.AccForwards
				fmt.Println("Forwards")
			}
		}

		if win.Pressed(pixelgl.KeyW) {
			mainCar.Accelerating = true
		} else {
			mainCar.Accelerating = false
		}
		if win.Pressed(pixelgl.KeyS) {
			mainCar.Breaking = true
		} else {
			mainCar.Breaking = false
		}

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
		//camMatrix := pixel.IM.Scaled(win.Bounds().Center(), 1)
		//win.SetMatrix(camMatrix)
		win.Update()

		<-frameRateLimiter
	}
}
