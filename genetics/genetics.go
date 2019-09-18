package genetics

import (
	"math/rand"

	"github.com/pkg/errors"
)

type Population struct {
	Solutions []Sequence
	Iteration uint64
	options   []byte
}

func NewPopulation(populationSize, solutionSize int, options []byte) (*Population, error) {
	if len(options) < 2 {
		return nil, errors.New("number of options must be 2 or greater")
	}
	if populationSize%2 != 0 || populationSize < 4 {
		return nil, errors.New("population size must be even and greater than 4")
	}
	p := &Population{
		Solutions: make([]Sequence, 0, populationSize),
		options:   options,
	}
	// create sequences of predetermined lengths
	for i := range p.Solutions {
		p.Solutions[i] = NewSequence(solutionSize)
	}

	return p, nil
}

func (p *Population) PerformSelection() {
	p.Iteration++
	newSolutions := make([]Sequence, 0, len(p.Solutions))

	for i := 0; i < len(p.Solutions); i+=2 {
		firstParent := p.rouletteWheelSelection()
		secondParent := p.rouletteWheelSelection()

		// crossover
		c1, c2 := Crossover(firstParent, secondParent)
		//Swap(c1, c2)
		// mutate
		c1.Mutate(p.options)
		c2.Mutate(p.options)

		newSolutions[i] = c1
		newSolutions[i+1] = c1
	}

	p.Solutions = newSolutions
}

func (p *Population) rouletteWheelSelection() Sequence {
	r := rand.Float64()

	var total float64
	for _, s := range p.Solutions {
		total += s.fitnessRatio

		if r <= total {
			return s
		}
	}

	return p.Solutions[0]
}

type Sequence struct {
	data         []byte
	fitnessValue float64
	fitnessRatio float64
}

func NewSequence(size int) Sequence {
	return Sequence{
		data: make([]byte, 0, size),
	}
}

func Crossover(s1, s2 Sequence) (Sequence, Sequence) {
	size := len(s1.data)
	c1 := NewSequence(size)
	c2 := NewSequence(size)

	// TODO: scale min/max to size
	separator := randRange(2, size-2)

	for i := separator; i < size; i++ {
		if i < separator {
			// keep first half of sequences same as original sequence
			c1.data[i] = s1.data[i]
			c2.data[i] = s2.data[i]
		}
		// cross over second half of sequences
		c1.data[i] = s2.data[i]
		c2.data[i] = s1.data[i]
	}

	return c1, c2
}

func Swap(s1, s2 Sequence) {
	i1 := randRange(0, len(s1.data)-1)
	i2 := randRange(0, len(s2.data)-1)
	s1.data[i1], s2.data[i2] = s2.data[i2], s1.data[i1]
}

func (s Sequence) Mutate(options []byte) {
	randOption := options[randRange(0, len(options)-1)]
	randIndex := randRange(0, len(s.data)-1)
	s.data[randIndex] = randOption
}

// TODO:
/*
1) Generate initial zeroed population
2) Evaluate fitnessValue
   Selection - Roulette Wheel Selection
   Crossover
   Mutation
*/

func randRange(min, max int) int {
	return rand.Intn(max-min) + min
}
