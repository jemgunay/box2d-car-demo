package genetics

import (
	"fmt"
	"math/rand"
	"sort"
	"strings"

	"github.com/pkg/errors"
)

/*
1) Generate initial zeroed Sequences
2) Evaluate FitnessValue
   Selection - Roulette Wheel Selection
   Crossover
   Mutation
*/

type Option byte

type Population struct {
	Sequences      []*Sequence
	Iteration      uint64
	populationSize int
	solutionSize   int
	options        []Option
	FitnessSum     float64
}

func NewPopulation(populationSize, solutionSize int, options []Option) (*Population, error) {
	if len(options) < 2 {
		return nil, errors.New("number of options must be 2 or greater")
	}
	if populationSize%2 != 0 || populationSize < 4 {
		return nil, errors.New("Sequences size must be even and greater than 4")
	}
	p := &Population{
		Sequences:      make([]*Sequence, populationSize),
		populationSize: populationSize,
		solutionSize:   solutionSize,
		options:        options,
	}
	// create Sequences of predetermined lengths
	for i := range p.Sequences {
		p.Sequences[i] = NewSequence(solutionSize)
	}

	return p, nil
}

const ratioExponentialScale = 1.0/55.0

func (p *Population) PerformSelection() {
	p.Iteration++

	// determine fitness ratios from fitness values
	sort.Slice(p.Sequences, func(i, j int) bool {
		return p.Sequences[i].FitnessValue < p.Sequences[j].FitnessValue
	})
	for i, s := range p.Sequences {
		s.FitnessRatio = ratioExponentialScale * float64(i+1)
		//fmt.Printf("%v [%v]\n", s.Data, s.FitnessValue)
	}
	p.FitnessSum = 0

	// perform selection and apply mutations
	newSequences := make([]*Sequence, 0, len(p.Sequences))
	for i := 0; i < len(p.Sequences)/2; i++ {
		firstParent := p.rouletteWheelSelection()
		secondParent := p.rouletteWheelSelection()

		// crossover
		c1, c2 := crossover(firstParent, secondParent)
		swap(c1, c2)
		// mutate
		c1.mutate(p.options)
		//c1.mutate(p.options)
		c2.mutate(p.options)
		//c2.mutate(p.options)

		newSequences = append(newSequences, c1, c2)
	}

	p.Sequences = newSequences
}

func (p *Population) String() string {
	buf := strings.Builder{}
	for _, s := range p.Sequences {
		buf.WriteString(fmt.Sprintf("%v\n", s.Data))
	}
	return buf.String()
}

func (p *Population) rouletteWheelSelection() *Sequence {
	r := rand.Float64()

	var total float64
	for _, s := range p.Sequences {
		total += s.FitnessRatio

		if r <= total {
			return s
		}
	}

	return p.Sequences[0]
}

type Sequence struct {
	Data         []Option
	FitnessValue float64
	FitnessRatio float64
}

func NewSequence(size int) *Sequence {
	return &Sequence{
		Data: make([]Option, size),
	}
}

func crossover(s1, s2 *Sequence) (*Sequence, *Sequence) {
	size := len(s1.Data)
	c1 := NewSequence(size)
	c2 := NewSequence(size)

	// TODO: offset from centre randomly
	separator := size/2

	for i := separator; i < size; i++ {
		if i < separator {
			// keep first half of Sequences same as original sequence
			c1.Data[i] = s1.Data[i]
			c2.Data[i] = s2.Data[i]
		}
		// cross over second half of Sequences
		c1.Data[i] = s2.Data[i]
		c2.Data[i] = s1.Data[i]
	}

	return c1, c2
}

func swap(s1, s2 *Sequence) {
	i := randRange(0, len(s1.Data)-1)
	j := randRange(0, len(s2.Data)-1)
	s1.Data[i], s2.Data[j] = s2.Data[j], s1.Data[i]
}

func (s Sequence) mutate(options []Option) {
	randOption := randRange(0, len(options)-1)
	randIndex := randRange(0, len(s.Data)-1)
	s.Data[randIndex] = options[randOption]
}

func randRange(min, max int) int {
	return rand.Intn(max-min+1) + min
}
