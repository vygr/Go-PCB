//package name
package layer

//package imports
import (
	"../mymath"
	"math"
	"os"
)

/////////////////////////
//private structure/types
/////////////////////////

type dims struct {
	width  int
	height int
}

type point struct {
	x float32
	y float32
}

type line struct {
	p1     point
	p2     point
	radius float32
}

type record struct {
	id   int
	line line
}

type aabb struct {
	minx int
	miny int
	maxx int
	maxy int
}

type bucket []record
type buckets []bucket

//layer object
type layer struct {
	width   int
	height  int
	scale   float32
	buckets buckets
	test    int
}

//layer methods

/////////////////
//private methods
/////////////////

func newlayer(dims dims, s float32) *layer {
	l := layer{}
	l.init(dims, s)
	return &l
}

func (self *layer) init(dims dims, s float32) {
	self.width = dims.width
	self.height = dims.height
	self.scale = s
	self.buckets = nil
	for i := 0; i < (self.width * self.height); i++ {
		self.buckets = append(self.buckets, bucket{})
	}
	self.test = 0
	return
}

func (self *layer) aabb(l line) aabb {
	x1, y1, x2, y2, r := l.p1.x, l.p1.y, l.p2.x, l.p2.y, l.radius
	if x1 > x2 {
		x1, x2 = x2, x1
	}
	if y1 > y2 {
		y1, y2 = y2, y1
	}
	minx := int(math.Floor(float64((x1 - r) * self.scale)))
	miny := int(math.Floor(float64((y1 - r) * self.scale)))
	maxx := int(math.Ceil(float64((x2 + r) * self.scale)))
	maxy := int(math.Ceil(float64((y2 + r) * self.scale)))
	if minx < 0 {
		minx = 0
	}
	if miny < 0 {
		miny = 0
	}
	if maxx > self.width {
		maxx = self.width
	}
	if maxy > self.height {
		maxy = self.height
	}
	if minx > maxx || miny > maxy {
		print("AABB Error:")
		os.Exit(20)
	}
	return aabb{minx, miny, maxx, maxy}
}

func (self *layer) all_buckets(bb aabb) <-chan int {
	yield := make(chan int, 128)
	go func() {
		minx, miny, maxx, maxy := bb.minx, bb.miny, bb.maxx, bb.maxy
		for y := miny; y < maxy; y++ {
			for x := minx; x < maxx; x++ {
				yield <- y*self.width + x
			}
		}
		close(yield)
	}()
	return yield
}

func (self *layer) all_not_empty_buckets(bb aabb) <-chan int {
	yield := make(chan int, 128)
	go func() {
		minx, miny, maxx, maxy := bb.minx, bb.miny, bb.maxx, bb.maxy
		for y := miny; y < maxy; y++ {
			for x := minx; x < maxx; x++ {
				b := y*self.width + x
				if len(self.buckets[b]) > 0 {
					yield <- b
				}
			}
		}
		close(yield)
	}()
	return yield
}

func lines_equal(l1 line, l2 line) bool {
	if l1.p1.x != l2.p1.x {
		return false
	}
	if l1.p1.y != l2.p1.y {
		return false
	}
	if l1.p2.x != l2.p2.x {
		return false
	}
	if l1.p2.y != l2.p2.y {
		return false
	}
	if l1.radius != l2.radius {
		return false
	}
	return true
}

func (self *layer) add_line(l line) {
	new_record := record{0, l}
	for b := range self.all_buckets(self.aabb(l)) {
		found := false
		for _, record := range self.buckets[b] {
			if lines_equal(record.line, l) {
				found = true
				break
			}
		}
		if !found {
			self.buckets[b] = append(self.buckets[b], new_record)
		}
	}
}

func (self *layer) sub_line(l line) {
	for b := range self.all_not_empty_buckets(self.aabb(l)) {
		for i := len(self.buckets[b]) - 1; i >= 0; i-- {
			if lines_equal(self.buckets[b][i].line, l) {
				self.buckets[b] = append(self.buckets[b][:i], self.buckets[b][i+1:]...)
			}
		}
	}
}

func (self *layer) hit_line(l line) bool {
	self.test += 1
	for b := range self.all_not_empty_buckets(self.aabb(l)) {
		for _, record := range self.buckets[b] {
			if record.id != self.test {
				record.id = self.test
				l1_p1_x, l1_p1_y, l1_p2_x, l1_p2_y, l1_r :=
					l.p1.x, l.p1.y,
					l.p2.x, l.p2.y,
					l.radius
				l2_p1_x, l2_p1_y, l2_p2_x, l2_p2_y, l2_r :=
					record.line.p1.x, record.line.p1.y,
					record.line.p2.x, record.line.p2.y,
					record.line.radius
				if mymath.Collide_thick_lines_2d(
					mymath.Point{l1_p1_x, l1_p1_y}, mymath.Point{l1_p2_x, l1_p2_y},
					mymath.Point{l2_p1_x, l2_p1_y}, mymath.Point{l2_p2_x, l2_p2_y},
					l1_r, l2_r) {
					return true
				}
			}
		}
	}
	return false
}

/////////////////////////
//public structures/types
/////////////////////////

type Dims struct {
	Width  int
	Height int
	Depth  int
}

//layers object
type Layers struct {
	depth  int
	layers []*layer
}

//layers methods

////////////////
//public methods
////////////////

func NewLayers(dims Dims, s float32) *Layers {
	l := Layers{}
	l.Init(dims, s)
	return &l
}

func (self *Layers) Init(dm Dims, s float32) *Layers {
	width := dm.Width
	height := dm.Height
	self.depth = dm.Depth
	self.layers = nil
	for z := 0; z < self.depth; z++ {
		self.layers = append(self.layers, newlayer(dims{width, height}, s))
	}
	return self
}

func (self *Layers) Add_line(p1, p2 mymath.Point, r float32) {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	lp1 := point{x1, y1}
	lp2 := point{x2, y2}
	for layer := range self.all_layers(z1, z2) {
		layer.add_line(line{lp1, lp2, r})
	}
}

func (self *Layers) Sub_line(p1, p2 mymath.Point, r float32) {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	lp1 := point{x1, y1}
	lp2 := point{x2, y2}
	for layer := range self.all_layers(z1, z2) {
		layer.sub_line(line{lp1, lp2, r})
	}
}

func (self *Layers) Hit_line(p1, p2 mymath.Point, r float32) bool {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	lp1 := point{x1, y1}
	lp2 := point{x2, y2}
	for layer := range self.all_layers(z1, z2) {
		if layer.hit_line(line{lp1, lp2, r}) {
			return true
		}
	}
	return false
}

/////////////////
//private methods
/////////////////

func (self *Layers) all_layers(z1, z2 float32) <-chan *layer {
	yield := make(chan *layer, 6)
	go func() {
		if z1 != z2 {
			for z := 0; z < self.depth; z++ {
				yield <- self.layers[z]
			}
		} else {
			yield <- self.layers[int(z1)]
		}
		close(yield)
	}()
	return yield
}
