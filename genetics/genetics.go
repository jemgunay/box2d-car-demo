package genetics

import (
	"math/rand"
)

type Population struct {
	Solutions []Sequence
	Iteration uint64
}

func NewPopulation(size uint32) *Population {
	p := &Population{
		Solutions: make([]Sequence, 0, size),
	}
	for i := 0; i < int(size); i++ {

	}
	return
}

func (p *Population) PerformSelection() {

	for i := 0; i < 5; i++ {
		firstParent := p.rouletteWheelSelection()
		secondParent := p.rouletteWheelSelection()

		child := Crossover(firstParent, secondParent)
		child = child.Swap(secondParent)
		child = child.Mutate()
	}
}

type Sequence struct {
	chromosome   []byte
	fitness      float64
	fitnessRatio float64
}

func NewSequence(size uint32) Sequence {
	return Sequence{
		chromosome: make([]byte, 0, size),
	}
}

func Crossover(s1, s2 Sequence) (Sequence, Sequence) {

}

func Swap(s1, s2 Sequence) Sequence {

}

func (s Sequence) Mutate(options ...byte) Sequence {

}

// TODO:
/*
1) Generate initial zeroed population
2) Evaluate fitness
   Selection - Roulette Wheel Selection
   Crossover
   Mutation

*/

func (p *Population) rouletteWheelSelection() Sequence {
	r := rand.Float64()

	var tot float64
	for _, s := range p.Solutions {
		tot += s.fitnessRatio

		if r <= tot {
			return s
		}
	}

	return 0
}
