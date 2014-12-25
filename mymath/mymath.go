// Copyright (C) 2014 Chris Hinsley.

//package name
package mymath

//package imports
import (
	"math"
	"math/rand"
)

/////////////////////////
//public structures/types
/////////////////////////

type Point []float32
type Points []Point
type Pointss []Points

///////////////////////////
//private utility functions
///////////////////////////

func sum(stream <-chan float32) float32 {
	t := float32(0.0)
	for v := range stream {
		t += v
	}
	return t
}

func max(stream <-chan float32) float32 {
	m := float32(-1.18E10 - 38)
	for v := range stream {
		if m < v {
			m = v
		}
	}
	return m
}

func min(stream <-chan float32) float32 {
	m := float32(+1.18E10 + 38)
	for v := range stream {
		if m > v {
			m = v
		}
	}
	return m
}

func zip_1_point(p Point) <-chan float32 {
	l := len(p)
	yield := make(chan float32, l)
	go func() {
		for i := 0; i < l; i++ {
			yield <- p[i]
		}
		close(yield)
	}()
	return yield
}

func zip_2_point(p1, p2 Point) <-chan Point {
	l1 := len(p1)
	l2 := len(p2)
	l := l1
	if l2 < l1 {
		l = l2
	}
	yield := make(chan Point, l)
	go func() {
		for i := 0; i < l; i++ {
			yield <- Point{p1[i], p2[i]}
		}
		close(yield)
	}()
	return yield
}

func collect_point(stream <-chan float32, l int) Point {
	p := make(Point, 0, l)
	for e := range stream {
		p = append(p, e)
	}
	return p
}

//////////////////
//public functions
//////////////////

//generic distance metric stuff
func Manhattan_distance(p1, p2 Point) float32 {
	yield := make(chan float32, len(p1))
	go func() {
		for p := range zip_2_point(p1, p2) {
			yield <- float32(math.Abs(float64(p[0] - p[1])))
		}
		close(yield)
	}()
	return sum(yield)
}

func Euclidean_distance(p1, p2 Point) float32 {
	yield := make(chan float32, len(p1))
	go func() {
		for p := range zip_2_point(p1, p2) {
			ps := (p[0] - p[1])
			ps *= ps
			yield <- float32(ps)
		}
		close(yield)
	}()
	return float32(math.Sqrt(float64(sum(yield))))
}

func Squared_euclidean_distance(p1, p2 Point) float32 {
	yield := make(chan float32, len(p1))
	go func() {
		for p := range zip_2_point(p1, p2) {
			ps := (p[0] - p[1])
			ps *= ps
			yield <- float32(ps)
		}
		close(yield)
	}()
	return sum(yield)
}

func Chebyshev_distance(p1, p2 Point) float32 {
	yield := make(chan float32, len(p1))
	go func() {
		for p := range zip_2_point(p1, p2) {
			yield <- float32(math.Abs(float64(p[0] - p[1])))
		}
		close(yield)
	}()
	return max(yield)
}

func Reciprical_distance(p1, p2 Point) float32 {
	d := Manhattan_distance(p1, p2)
	if d == 0.0 {
		return 1.0
	}
	return 1.0 / d
}

func Random_distance(p1, p2 Point) float32 {
	return float32(rand.Float32())
}

//generic vector stuff
func Sign(x int) int {
	if x == 0 {
		return 0
	}
	if x < 0 {
		return -1
	}
	return 1
}

func Equal(p1, p2 Point) bool {
	return Manhattan_distance(p1, p2) == 0.0
}

func Add(p1, p2 Point) Point {
	l := len(p1)
	yield := make(chan float32, l)
	go func() {
		for p := range zip_2_point(p1, p2) {
			yield <- float32(p[0] + p[1])
		}
		close(yield)
	}()
	return collect_point(yield, l)
}

func Sub(p1, p2 Point) Point {
	l := len(p1)
	yield := make(chan float32, l)
	go func() {
		for p := range zip_2_point(p1, p2) {
			yield <- float32(p[0] - p[1])
		}
		close(yield)
	}()
	return collect_point(yield, l)
}

func Scale(p Point, s float32) Point {
	l := len(p)
	yield := make(chan float32, l)
	go func() {
		for p := range zip_1_point(p) {
			yield <- p * s
		}
		close(yield)
	}()
	return collect_point(yield, l)
}

func Dot(p1, p2 Point) float32 {
	l := len(p1)
	yield := make(chan float32, l)
	go func() {
		for p := range zip_2_point(p1, p2) {
			yield <- float32(p[0] * p[1])
		}
		close(yield)
	}()
	return sum(yield)
}

func Length(p Point) float32 {
	return float32(math.Sqrt(float64(Dot(p, p))))
}

func Distance(p1, p2 Point) float32 {
	return Length(Sub(p2, p1))
}

func Distance_squared(p1, p2 Point) float32 {
	p := Sub(p2, p1)
	return Dot(p, p)
}

func Normalise(p Point) Point {
	l := Length(p)
	if l == 0.0 {
		return Scale(p, 0.0)
	}
	return Scale(p, 1.0/l)
}

func Distance_to_line(p, p1, p2 Point) float32 {
	lv := Sub(p2, p1)
	pv := Sub(p, p1)
	c1 := Dot(pv, lv)
	if c1 <= 0 {
		return Distance(p, p1)
	}
	c2 := Dot(lv, lv)
	if c2 <= c1 {
		return Distance(p, p2)
	}
	return Distance(p, Add(p1, Scale(lv, c1/c2)))
}

func Distance_squared_to_line(p, p1, p2 Point) float32 {
	lv := Sub(p2, p1)
	pv := Sub(p, p1)
	c1 := Dot(pv, lv)
	if c1 <= 0 {
		return Distance_squared(p, p1)
	}
	c2 := Dot(lv, lv)
	if c2 <= c1 {
		return Distance_squared(p, p2)
	}
	return Distance_squared(p, Add(p1, Scale(lv, c1/c2)))
}

//specific vector stuff
func Equal_2d(p1, p2 Point) bool {
	if p1[0] != p2[0] {
		return false
	}
	if p1[1] != p2[1] {
		return false
	}
	return true
}

func Equal_3d(p1, p2 Point) bool {
	if p1[0] != p2[0] {
		return false
	}
	if p1[1] != p2[1] {
		return false
	}
	if p1[2] != p2[2] {
		return false
	}
	return true
}

func Add_2d(p1, p2 Point) Point {
	x1, y1 := p1[0], p1[1]
	x2, y2 := p2[0], p2[1]
	return Point{x1 + x2, y1 + y2}
}

func Add_3d(p1, p2 Point) Point {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	return Point{x1 + x2, y1 + y2, z1 + z2}
}

func Sub_2d(p1, p2 Point) Point {
	x1, y1 := p1[0], p1[1]
	x2, y2 := p2[0], p2[1]
	return Point{x1 - x2, y1 - y2}
}

func Sub_3d(p1, p2 Point) Point {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	return Point{x1 - x2, y1 - y2, z1 - z2}
}

func Scale_2d(p Point, s float32) Point {
	x, y := p[0], p[1]
	return Point{x * s, y * s}
}

func Scale_3d(p Point, s float32) Point {
	x, y, z := p[0], p[1], p[2]
	return Point{x * s, y * s, z * s}
}

func Perp_2d(p Point) Point {
	x, y := p[0], p[1]
	return Point{y, -x}
}

func Cross_3d(p1, p2 Point) Point {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	return Point{y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - y1*x2}
}

func Dot_2d(p1, p2 Point) float32 {
	x1, y1 := p1[0], p1[1]
	x2, y2 := p2[0], p2[1]
	return x1*x2 + y1*y2
}

func Dot_3d(p1, p2 Point) float32 {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	return x1*x2 + y1*y2 + z1*z2
}

func Length_2d(p Point) float32 {
	return float32(math.Sqrt(float64(Dot_2d(p, p))))
}

func Length_3d(p Point) float32 {
	return float32(math.Sqrt(float64(Dot_3d(p, p))))
}

func Normalise_2d(p Point) Point {
	l := Length_2d(p)
	if l == 0 {
		return Point{0, 0}
	}
	x, y := p[0], p[1]
	return Point{x / l, y / l}
}

func Normalise_3d(p Point) Point {
	l := Length_3d(p)
	if l == 0 {
		return Point{0, 0, 0}
	}
	x, y, z := p[0], p[1], p[2]
	return Point{x / l, y / l, z / l}
}

func Distance_2d(p1, p2 Point) float32 {
	x1, y1 := p1[0], p1[1]
	x2, y2 := p2[0], p2[1]
	return Length_2d(Point{x2 - x1, y2 - y1})
}

func Distance_3d(p1, p2 Point) float32 {
	x1, y1, z1 := p1[0], p1[1], p1[2]
	x2, y2, z2 := p2[0], p2[1], p2[2]
	return Length_2d(Point{x2 - x1, y2 - y1, z2 - z1})
}

func Distance_squared_2d(p1, p2 Point) float32 {
	p := Sub_2d(p2, p1)
	return Dot_2d(p, p)
}

func Distance_squared_3d(p1, p2 Point) float32 {
	p := Sub_3d(p2, p1)
	return Dot_3d(p, p)
}

func Distance_to_line_2d(p, p1, p2 Point) float32 {
	lv := Sub_2d(p2, p1)
	pv := Sub_2d(p, p1)
	c1 := Dot_2d(pv, lv)
	if c1 <= 0 {
		return Distance_2d(p, p1)
	}
	c2 := Dot_2d(lv, lv)
	if c2 <= c1 {
		return Distance_2d(p, p2)
	}
	return Distance_2d(p, Add_2d(p1, Scale_2d(lv, c1/c2)))
}

func Distance_to_line_3d(p, p1, p2 Point) float32 {
	lv := Sub_3d(p2, p1)
	pv := Sub_3d(p, p1)
	c1 := Dot_3d(pv, lv)
	if c1 <= 0 {
		return Distance_3d(p, p1)
	}
	c2 := Dot_3d(lv, lv)
	if c2 <= c1 {
		return Distance_3d(p, p2)
	}
	return Distance_3d(p, Add_3d(p1, Scale_3d(lv, c1/c2)))
}

func Distance_squared_to_line_2d(p, p1, p2 Point) float32 {
	lv := Sub_2d(p2, p1)
	pv := Sub_2d(p, p1)
	c1 := Dot_2d(pv, lv)
	if c1 <= 0 {
		return Distance_squared_2d(p, p1)
	}
	c2 := Dot_2d(lv, lv)
	if c2 <= c1 {
		return Distance_squared_2d(p, p2)
	}
	return Distance_squared_2d(p, Add_2d(p1, Scale_2d(lv, c1/c2)))
}

func Distance_squared_to_line_3d(p, p1, p2 Point) float32 {
	lv := Sub_3d(p2, p1)
	pv := Sub_3d(p, p1)
	c1 := Dot_3d(pv, lv)
	if c1 <= 0 {
		return Distance_squared_3d(p, p1)
	}
	c2 := Dot_3d(lv, lv)
	if c2 <= c1 {
		return Distance_squared_3d(p, p2)
	}
	return Distance_squared_3d(p, Add_3d(p1, Scale_3d(lv, c1/c2)))
}

func Collide_lines_2d(l1_p1, l1_p2, l2_p1, l2_p2 Point) bool {
	l1_x1, l1_y1 := l1_p1[0], l1_p1[1]
	l1_x2, l1_y2 := l1_p2[0], l1_p2[1]
	l2_x1, l2_y1 := l2_p1[0], l2_p1[1]
	l2_x2, l2_y2 := l2_p2[0], l2_p2[1]
	ax := l1_x2 - l1_x1
	ay := l1_y2 - l1_y1
	bx := l2_x1 - l2_x2
	by := l2_y1 - l2_y2
	cx := l1_x1 - l2_x1
	cy := l1_y1 - l2_y1
	an := by*cx - bx*cy
	ad := ay*bx - ax*by
	bn := ax*cy - ay*cx
	bd := ay*bx - ax*by
	if (ad == 0) || (bd == 0) {
		return false
	} else {
		if ad > 0 {
			if (an < 0) || (an > ad) {
				return false
			}
		} else {
			if (an > 0) || (an < ad) {
				return false
			}
		}
		if bd > 0 {
			if (bn < 0) || (bn > bd) {
				return false
			}
		} else {
			if (bn > 0) || (bn < bd) {
				return false
			}
		}
	}
	return true
}

func Collide_thick_lines_2d(tl1_p1, tl1_p2, tl2_p1, tl2_p2 Point, tl1_r, tl2_r float32) bool {
	if Collide_lines_2d(tl1_p1, tl1_p2, tl2_p1, tl2_p2) {
		return true
	}
	radius_squared := (tl1_r + tl2_r)
	radius_squared *= radius_squared
	if Distance_squared_to_line_2d(tl2_p1, tl1_p1, tl1_p2) <= radius_squared {
		return true
	}
	if Distance_squared_to_line_2d(tl2_p2, tl1_p1, tl1_p2) <= radius_squared {
		return true
	}
	if Distance_squared_to_line_2d(tl1_p1, tl2_p1, tl2_p2) <= radius_squared {
		return true
	}
	if Distance_squared_to_line_2d(tl1_p2, tl2_p1, tl2_p2) <= radius_squared {
		return true
	}
	return false
}

//generic path stuff
func Thicken_path_2d(path Points, radius float32, capstyle, joinstyle int) Points {
	if radius == 0.0 {
		radius = 0.00000001
	}
	index := 0
	step := 1
	out_points := Points{}
	resolution := int(0.3*math.Pi*radius) + 1
	for {
		p1 := path[index]
		index += step
		p2 := path[index]
		index += step
		l2_v := Sub_2d(p2, p1)
		l2_pv := Perp_2d(l2_v)
		l2_npv := Normalise_2d(l2_pv)
		rv := Scale_2d(l2_npv, radius)
		switch {
		case capstyle == 0:
			//butt cap
			out_points = append(out_points, Sub_2d(p1, rv))
			out_points = append(out_points, Add_2d(p1, rv))
		case capstyle == 1:
			//square cap
			p0 := Add_2d(p1, Perp_2d(rv))
			out_points = append(out_points, Sub_2d(p0, rv))
			out_points = append(out_points, Add_2d(p0, rv))
		case capstyle == 2:
			//triangle cap
			out_points = append(out_points, Sub_2d(p1, rv))
			out_points = append(out_points, Add_2d(p1, Perp_2d(rv)))
			out_points = append(out_points, Add_2d(p1, rv))
		default:
			//round cap
			rvx, rvy := rv[0], rv[1]
			for i := 0; i <= resolution; i++ {
				angle := float64((float32(i) * math.Pi) / float32(resolution))
				s := float32(math.Sin(angle))
				c := float32(math.Cos(angle))
				rv := Point{rvx*c - rvy*s, rvx*s + rvy*c}
				out_points = append(out_points, Sub_2d(p1, rv))
			}
		}
		for (index != -1) && (index != len(path)) {
			p1, l1_v, l1_npv := p2, l2_v, l2_npv
			p2 = path[index]
			index += step
			l2_v = Sub_2d(p2, p1)
			l2_pv = Perp_2d(l2_v)
			l2_npv = Normalise_2d(l2_pv)
			nbv := Normalise_2d(Scale_2d(Add_2d(l1_npv, l2_npv), 0.5))
			c := Dot_2d(nbv, Normalise_2d(l1_v))
			switch {
			case (c <= 0) || (joinstyle == 0):
				//mitre join
				s := float32(math.Sin(math.Acos(float64(c))))
				bv := Scale_2d(nbv, radius/s)
				out_points = append(out_points, Add_2d(p1, bv))
			case joinstyle == 1:
				//bevel join
				out_points = append(out_points, Add_2d(p1, Scale_2d(l1_npv, radius)))
				out_points = append(out_points, Add_2d(p1, Scale_2d(l2_npv, radius)))
			default:
				//round join
				rv := Scale_2d(l1_npv, radius)
				rvx, rvy := rv[0], rv[1]
				theta := float32(math.Acos(float64(Dot_2d(l1_npv, l2_npv))))
				segs := int((theta/float32(math.Pi))*float32(resolution)) + 1
				for i := 0; i <= segs; i++ {
					angle := float64((float32(i) * theta) / float32(segs))
					s := float32(math.Sin(angle))
					c := float32(math.Cos(angle))
					rv := Point{rvx*c - rvy*s, rvx*s + rvy*c}
					out_points = append(out_points, Add_2d(p1, rv))
				}
			}
		}
		if step < 0 {
			break
		}
		step = -step
		index += step
	}
	return out_points
}
