package genetics

import (
	"fmt"
	"math/rand"
	"sort"
	"strings"

	"github.com/pkg/errors"
)

// Option represents a single encoded instruction of which a Sequence can be constructed.
type Option byte

// Population represents a population of sequences.
type Population struct {
	Sequences           []*Sequence
	Iteration           uint64
	populationSize      int
	solutionSize        int
	options             []Option
	FitnessSum          float64
	ratioExpScaleFactor float64
}

// NewPopulation initialises a new Population which is ready to be evolved.
func NewPopulation(populationSize, solutionSize int, options []Option) (*Population, error) {
	if len(options) < 2 {
		return nil, errors.New("number of options must be 2 or greater")
	}
	if populationSize%2 != 0 || populationSize < 4 {
		return nil, errors.New("sequence size must be even and greater than 4")
	}
	p := &Population{
		Sequences:      make([]*Sequence, populationSize),
		populationSize: populationSize,
		solutionSize:   solutionSize,
		options:        options,
	}
	// create sequence of predetermined lengths
	for i := range p.Sequences {
		p.Sequences[i] = NewSequence(solutionSize)
	}

	// determine scale factor for exponential fitness weighting
	for i := 0; i < populationSize; i++ {
		p.ratioExpScaleFactor += float64(i + 1)
	}
	p.ratioExpScaleFactor = 1.0 / p.ratioExpScaleFactor

	return p, nil
}

// Randomise randomises each sequence in the population, setting each Sequence Data value to a random item from the
// supported Options.
func (p *Population) Randomise() {
	for i := range p.Sequences {
		for j := range p.Sequences[i].Data {
			randOption := randRange(0, len(p.options)-2)
			p.Sequences[i].Data[j] = p.options[randOption]
		}
	}
}

// PerformSelection performs one iteration of selection in order to produce a fresh population of children from the
// previous population.
func (p *Population) PerformSelection() {
	p.Iteration++

	// determine fitness ratios from fitness values
	sort.Slice(p.Sequences, func(i, j int) bool {
		return p.Sequences[i].FitnessValue < p.Sequences[j].FitnessValue
	})
	for i, s := range p.Sequences {
		// each sequence can get selected relative to its fitness
		//s.FitnessRatio = 1 - (s.FitnessValue / p.FitnessSum)
		// more fit sequences are exponentially more likely to get selected
		s.FitnessRatio = 1 - (p.ratioExpScaleFactor * float64(i+1))
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
		c2.mutate(p.options)

		newSequences = append(newSequences, c1, c2)
	}

	p.Sequences = newSequences
}

// String formats the population data into a readable string.
func (p *Population) String() string {
	buf := strings.Builder{}
	for _, s := range p.Sequences {
		buf.WriteString(fmt.Sprintf("%v\n", s.Data))
	}
	return buf.String()
}

// Performs roulette wheel selection, where a sequence's likelihood of getting selected is proportional to its fitness.
// This means even poorly performing sequences have a chance of being selected.
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

// Sequence represents a member of a population, containing the genetic composition of that sequence and its overall
// fitness.
type Sequence struct {
	Data         []Option
	FitnessValue float64
	FitnessRatio float64
}

// NewSequence creates an initialised Sequence.
func NewSequence(size int) *Sequence {
	return &Sequence{
		Data: make([]Option, size),
	}
}

// performs crossover between two sequences, resulting in two child sequences
func crossover(s1, s2 *Sequence) (*Sequence, *Sequence) {
	size := len(s1.Data)
	c1 := NewSequence(size)
	c2 := NewSequence(size)

	// offset from centre, use 30-70% range
	offset := int(float64(len(c1.Data)) * 0.3)
	separator := randRange(offset, (len(c1.Data)-1)-offset)

	swapLeft := randRange(0, 1) == 0
	for i := 0; i < size; i++ {
		if (swapLeft && i < separator) || (!swapLeft && i > separator) {
			// cross over one half of sequences
			c1.Data[i] = s2.Data[i]
			c2.Data[i] = s1.Data[i]
		} else {
			// keep other half of sequences same as original sequences
			c1.Data[i] = s1.Data[i]
			c2.Data[i] = s2.Data[i]
		}
	}

	return c1, c2
}

// swaps a random index in one sequence with a random index in another sequence
func swap(s1, s2 *Sequence) {
	i := randRange(0, len(s1.Data)-1)
	j := randRange(0, len(s2.Data)-1)
	s1.Data[i], s2.Data[j] = s2.Data[j], s1.Data[i]
}

// randomly mutates a sequence at a single index - 50/50 chance of returning and skipping the mutation
func (s Sequence) mutate(options []Option) {
	// 50% chance of mutation
	if randRange(0, 1) == 0 {
		return
	}

	randOption := randRange(0, len(options)-1)
	randIndex := randRange(0, len(s.Data)-1)
	s.Data[randIndex] = options[randOption]
}

// inclusive random integer within a specified range
func randRange(min, max int) int {
	return rand.Intn(max-min+1) + min
}
