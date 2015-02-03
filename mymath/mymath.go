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
type Points []*Point

//////////////////
//public functions
//////////////////

//generic distance metric stuff
func Manhattan_distance(pp1, pp2 *Point) float32 {
	p1 := *pp1
	p2 := *pp2
	d := float32(0.0)
	for i := 0; i < len(p1); i++ {
		d += float32(math.Abs(float64(p1[i] - p2[i])))
	}
	return d
}

func Euclidean_distance(pp1, pp2 *Point) float32 {
	p1 := *pp1
	p2 := *pp2
	d := float32(0.0)
	for i := 0; i < len(p1); i++ {
		ps := p1[i] - p2[i]
		d += ps * ps
	}
	return float32(math.Sqrt(float64(d)))
}

func Squared_euclidean_distance(pp1, pp2 *Point) float32 {
	p1 := *pp1
	p2 := *pp2
	d := float32(0.0)
	for i := 0; i < len(p1); i++ {
		ps := p1[i] - p2[i]
		d += ps * ps
	}
	return d
}

func Chebyshev_distance(pp1, pp2 *Point) float32 {
	p1 := *pp1
	p2 := *pp2
	d := float32(0.0)
	for i := 0; i < len(p1); i++ {
		d1 := float32(math.Abs(float64(p1[i] - p2[i])))
		if d1 > d {
			d = d1
		}
	}
	return d
}

func Reciprical_distance(pp1, pp2 *Point) float32 {
	d := Manhattan_distance(pp1, pp2)
	if d == 0.0 {
		return 1.0
	}
	return 1.0 / d
}

func Random_distance(pp1, pp2 *Point) float32 {
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

func Equal(pp1, pp2 *Point) bool {
	return Manhattan_distance(pp1, pp2) == 0.0
}

func Add(pp1, pp2 *Point) *Point {
	p1 := *pp1
	p2 := *pp2
	p := make(Point, len(p1), len(p1))
	for i := 0; i < len(p1); i++ {
		p[i] = p1[i] + p2[i]
	}
	return &p
}

func Sub(pp1, pp2 *Point) *Point {
	p1 := *pp1
	p2 := *pp2
	p := make(Point, len(p1), len(p1))
	for i := 0; i < len(p1); i++ {
		p[i] = p1[i] - p2[i]
	}
	return &p
}

func Scale(pp *Point, s float32) *Point {
	p := *pp
	sp := make(Point, len(p), len(p))
	for i := 0; i < len(p); i++ {
		sp[i] = p[i] * s
	}
	return &sp
}

func Dot(pp1, pp2 *Point) float32 {
	p1 := *pp1
	p2 := *pp2
	d := float32(0.0)
	for i := 0; i < len(p1); i++ {
		d += p1[i] * p2[i]
	}
	return d
}

func Length(pp *Point) float32 {
	return float32(math.Sqrt(float64(Dot(pp, pp))))
}

func Distance(pp1, pp2 *Point) float32 {
	return Length(Sub(pp2, pp1))
}

func Distance_squared(pp1, pp2 *Point) float32 {
	pp := Sub(pp2, pp1)
	return Dot(pp, pp)
}

func Norm(pp *Point) *Point {
	l := Length(pp)
	if l == 0.0 {
		return Scale(pp, 0.0)
	}
	return Scale(pp, 1.0/l)
}

func Distance_to_line(pp, pp1, pp2 *Point) float32 {
	lv := Sub(pp2, pp1)
	pv := Sub(pp, pp1)
	c1 := Dot(pv, lv)
	if c1 <= 0 {
		return Distance(pp, pp1)
	}
	c2 := Dot(lv, lv)
	if c2 <= c1 {
		return Distance(pp, pp2)
	}
	return Distance(pp, Add(pp1, Scale(lv, c1/c2)))
}

func Distance_squared_to_line(pp, pp1, pp2 *Point) float32 {
	lv := Sub(pp2, pp1)
	pv := Sub(pp, pp1)
	c1 := Dot(pv, lv)
	if c1 <= 0 {
		return Distance_squared(pp, pp1)
	}
	c2 := Dot(lv, lv)
	if c2 <= c1 {
		return Distance_squared(pp, pp2)
	}
	return Distance_squared(pp, Add(pp1, Scale(lv, c1/c2)))
}

//specific vector stuff
func Equal_2d(pp1, pp2 *Point) bool {
	p1 := *pp1
	p2 := *pp2
	if p1[0] != p2[0] {
		return false
	}
	if p1[1] != p2[1] {
		return false
	}
	return true
}

func Equal_3d(pp1, pp2 *Point) bool {
	p1 := *pp1
	p2 := *pp2
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

func Add_2d(pp1, pp2 *Point) *Point {
	p1 := *pp1
	p2 := *pp2
	return &Point{p1[0] + p2[0], p1[1] + p2[1]}
}

func Add_3d(pp1, pp2 *Point) *Point {
	p1 := *pp1
	p2 := *pp2
	return &Point{p1[0] + p2[0], p1[1] + p2[1], p1[2] + p2[2]}
}

func Sub_2d(pp1, pp2 *Point) *Point {
	p1 := *pp1
	p2 := *pp2
	return &Point{p1[0] - p2[0], p1[1] - p2[1]}
}

func Sub_3d(pp1, pp2 *Point) *Point {
	p1 := *pp1
	p2 := *pp2
	return &Point{p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]}
}

func Scale_2d(pp *Point, s float32) *Point {
	p := *pp
	return &Point{p[0] * s, p[1] * s}
}

func Scale_3d(pp *Point, s float32) *Point {
	p := *pp
	return &Point{p[0] * s, p[1] * s, p[2] * s}
}

func Perp_2d(pp *Point) *Point {
	p := *pp
	return &Point{p[1], -p[0]}
}

func Cross_3d(pp1, pp2 *Point) *Point {
	p1 := *pp1
	p2 := *pp2
	return &Point{p1[1]*p2[2] - p1[2]*p2[1], p1[2]*p2[0] - p1[0]*p2[2], p1[0]*p2[1] - p1[1]*p2[0]}
}

func Dot_2d(pp1, pp2 *Point) float32 {
	p1 := *pp1
	p2 := *pp2
	return p1[0]*p2[0] + p1[1]*p2[1]
}

func Dot_3d(pp1, pp2 *Point) float32 {
	p1 := *pp1
	p2 := *pp2
	return p1[0]*p2[0] + p1[1]*p2[1] + p1[2]*p2[2]
}

func Length_2d(pp *Point) float32 {
	return float32(math.Sqrt(float64(Dot_2d(pp, pp))))
}

func Length_3d(pp *Point) float32 {
	return float32(math.Sqrt(float64(Dot_3d(pp, pp))))
}

func Norm_2d(pp *Point) *Point {
	l := Length_2d(pp)
	if l == 0 {
		return &Point{0, 0}
	}
	p := *pp
	return &Point{p[0] / l, p[1] / l}
}

func Norm_3d(pp *Point) *Point {
	l := Length_3d(pp)
	if l == 0 {
		return &Point{0, 0, 0}
	}
	p := *pp
	return &Point{p[0] / l, p[1] / l, p[2] / l}
}

func Distance_2d(pp1, pp2 *Point) float32 {
	return Length_2d(Sub_2d(pp2, pp1))
}

func Distance_3d(pp1, pp2 *Point) float32 {
	return Length_3d(Sub_3d(pp2, pp1))
}

func Distance_squared_2d(pp1, pp2 *Point) float32 {
	pp := Sub_2d(pp2, pp1)
	return Dot_2d(pp, pp)
}

func Distance_squared_3d(pp1, pp2 *Point) float32 {
	pp := Sub_3d(pp2, pp1)
	return Dot_3d(pp, pp)
}

func Distance_to_line_2d(pp, pp1, pp2 *Point) float32 {
	lv := Sub_2d(pp2, pp1)
	pv := Sub_2d(pp, pp1)
	c1 := Dot_2d(pv, lv)
	if c1 <= 0 {
		return Distance_2d(pp, pp1)
	}
	c2 := Dot_2d(lv, lv)
	if c2 <= c1 {
		return Distance_2d(pp, pp2)
	}
	return Distance_2d(pp, Add_2d(pp1, Scale_2d(lv, c1/c2)))
}

func Distance_to_line_3d(pp, pp1, pp2 *Point) float32 {
	lv := Sub_3d(pp2, pp1)
	pv := Sub_3d(pp, pp1)
	c1 := Dot_3d(pv, lv)
	if c1 <= 0 {
		return Distance_3d(pp, pp1)
	}
	c2 := Dot_3d(lv, lv)
	if c2 <= c1 {
		return Distance_3d(pp, pp2)
	}
	return Distance_3d(pp, Add_3d(pp1, Scale_3d(lv, c1/c2)))
}

func Distance_squared_to_line_2d(pp, pp1, pp2 *Point) float32 {
	lv := Sub_2d(pp2, pp1)
	pv := Sub_2d(pp, pp1)
	c1 := Dot_2d(pv, lv)
	if c1 <= 0 {
		return Distance_squared_2d(pp, pp1)
	}
	c2 := Dot_2d(lv, lv)
	if c2 <= c1 {
		return Distance_squared_2d(pp, pp2)
	}
	return Distance_squared_2d(pp, Add_2d(pp1, Scale_2d(lv, c1/c2)))
}

func Distance_squared_to_line_3d(pp, pp1, pp2 *Point) float32 {
	lv := Sub_3d(pp2, pp1)
	pv := Sub_3d(pp, pp1)
	c1 := Dot_3d(pv, lv)
	if c1 <= 0 {
		return Distance_squared_3d(pp, pp1)
	}
	c2 := Dot_3d(lv, lv)
	if c2 <= c1 {
		return Distance_squared_3d(pp, pp2)
	}
	return Distance_squared_3d(pp, Add_3d(pp1, Scale_3d(lv, c1/c2)))
}

func Collide_lines_2d(pl1_p1, pl1_p2, pl2_p1, pl2_p2 *Point) bool {
	l1_p1 := *pl1_p1
	l1_p2 := *pl1_p2
	l2_p1 := *pl2_p1
	l2_p2 := *pl2_p2
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

func Collide_thick_lines_2d(tl1_p1, tl1_p2, tl2_p1, tl2_p2 *Point, r float32) bool {
	if Collide_lines_2d(tl1_p1, tl1_p2, tl2_p1, tl2_p2) {
		return true
	}
	r *= r
	if Distance_squared_to_line_2d(tl2_p1, tl1_p1, tl1_p2) <= r {
		return true
	}
	if Distance_squared_to_line_2d(tl2_p2, tl1_p1, tl1_p2) <= r {
		return true
	}
	if Distance_squared_to_line_2d(tl1_p1, tl2_p1, tl2_p2) <= r {
		return true
	}
	if Distance_squared_to_line_2d(tl1_p2, tl2_p1, tl2_p2) <= r {
		return true
	}
	return false
}

//generic path stuff
func Circle_lines_2d(p *Point, radius float32, resolution int) *Points {
	out_points := make(Points, resolution+1, resolution*1)
	rvx, rvy := float32(0.0), radius
	for i := 0; i <= resolution; i++ {
		angle := float64((float32(i) * math.Pi * 2.0) / float32(resolution))
		s := float32(math.Sin(angle))
		c := float32(math.Cos(angle))
		rv := &Point{rvx*c - rvy*s, rvx*s + rvy*c}
		out_points[i] = Sub_2d(p, rv)
	}
	out_points[resolution] = out_points[0]
	return &out_points
}

func Circle_triangles_2d(p *Point, radius float32, resolution int) *Points {
	out_points := make(Points, resolution*2+2, resolution*2+2)
	rvx, rvy := float32(0.0), radius
	for i := 0; i <= resolution; i++ {
		angle := float64((float32(i) * math.Pi * 2.0) / float32(resolution))
		s := float32(math.Sin(angle))
		c := float32(math.Cos(angle))
		rv := &Point{rvx*c - rvy*s, rvx*s + rvy*c}
		out_points[i*2] = p
		out_points[i*2+1] = Sub_2d(p, rv)
	}
	out_points[resolution*2] = out_points[0]
	out_points[resolution*2+1] = out_points[1]
	return &out_points
}

func Thicken_path_lines_2d(pathp *Points, radius float32, capstyle, joinstyle, resolution int) *Points {
	if radius == 0.0 {
		radius = 0.00000001
	}
	path := *pathp
	index := 0
	step := 1
	out_points := Points{}
	for {
		p1 := path[index]
		index += step
		p2 := path[index]
		index += step
		l2_v := Sub_2d(p2, p1)
		l2_pv := Perp_2d(l2_v)
		l2_npv := Norm_2d(l2_pv)
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
			rvd := *rv
			rvx, rvy := rvd[0], rvd[1]
			for i := 0; i <= resolution; i++ {
				angle := float64((float32(i) * math.Pi) / float32(resolution))
				s := float32(math.Sin(angle))
				c := float32(math.Cos(angle))
				rv := &Point{rvx*c - rvy*s, rvx*s + rvy*c}
				out_points = append(out_points, Sub_2d(p1, rv))
			}
		}
		for (index != -1) && (index != len(path)) {
			p1, l1_v, l1_npv := p2, l2_v, l2_npv
			p2 = path[index]
			index += step
			l2_v = Sub_2d(p2, p1)
			l2_pv = Perp_2d(l2_v)
			l2_npv = Norm_2d(l2_pv)
			nbv := Norm_2d(Scale_2d(Add_2d(l1_npv, l2_npv), 0.5))
			c := Dot_2d(nbv, Norm_2d(l1_v))
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
				rvd := *rv
				rvx, rvy := rvd[0], rvd[1]
				theta := float32(math.Acos(float64(Dot_2d(l1_npv, l2_npv))))
				segs := int((theta/float32(math.Pi))*float32(resolution)) + 1
				for i := 0; i <= segs; i++ {
					angle := float64((float32(i) * theta) / float32(segs))
					s := float32(math.Sin(angle))
					c := float32(math.Cos(angle))
					rv := &Point{rvx*c - rvy*s, rvx*s + rvy*c}
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
	out_points = append(out_points, out_points[0])
	return &out_points
}

func Thicken_path_triangles_2d(pathp *Points, radius float32, capstyle, joinstyle, resolution int) *Points {
	if radius == 0.0 {
		radius = 0.00000001
	}
	path := *pathp
	index := 0
	step := 1
	out_points := Points{}
	for {
		p1 := path[index]
		index += step
		p2 := path[index]
		index += step
		l2_v := Sub_2d(p2, p1)
		l2_pv := Perp_2d(l2_v)
		l2_npv := Norm_2d(l2_pv)
		rv := Scale_2d(l2_npv, radius)
		switch {
		case capstyle == 0:
			//butt cap
			out_points = append(out_points, p1)
			out_points = append(out_points, Sub_2d(p1, rv))
			out_points = append(out_points, p1)
			out_points = append(out_points, Add_2d(p1, rv))
		case capstyle == 1:
			//square cap
			p0 := Add_2d(p1, Perp_2d(rv))
			out_points = append(out_points, p0)
			out_points = append(out_points, Sub_2d(p0, rv))
			out_points = append(out_points, p0)
			out_points = append(out_points, Add_2d(p0, rv))
		case capstyle == 2:
			//triangle cap
			out_points = append(out_points, p1)
			out_points = append(out_points, Sub_2d(p1, rv))
			out_points = append(out_points, p1)
			out_points = append(out_points, Add_2d(p1, Perp_2d(rv)))
			out_points = append(out_points, p1)
			out_points = append(out_points, Add_2d(p1, rv))
		default:
			//round cap
			rvd := *rv
			rvx, rvy := rvd[0], rvd[1]
			for i := 0; i <= resolution; i++ {
				angle := float64((float32(i) * math.Pi) / float32(resolution))
				s := float32(math.Sin(angle))
				c := float32(math.Cos(angle))
				rv := &Point{rvx*c - rvy*s, rvx*s + rvy*c}
				out_points = append(out_points, p1)
				out_points = append(out_points, Sub_2d(p1, rv))
			}
		}
		for (index != -1) && (index != len(path)) {
			p1, l1_v, l1_npv := p2, l2_v, l2_npv
			p2 = path[index]
			index += step
			l2_v = Sub_2d(p2, p1)
			l2_pv = Perp_2d(l2_v)
			l2_npv = Norm_2d(l2_pv)
			nbv := Norm_2d(Scale_2d(Add_2d(l1_npv, l2_npv), 0.5))
			c := Dot_2d(nbv, Norm_2d(l1_v))
			switch {
			case (c <= 0) || (joinstyle == 0):
				//mitre join
				s := float32(math.Sin(math.Acos(float64(c))))
				bv := Scale_2d(nbv, radius/s)
				out_points = append(out_points, p1)
				out_points = append(out_points, Add_2d(p1, bv))
			case joinstyle == 1:
				//bevel join
				out_points = append(out_points, p1)
				out_points = append(out_points, Add_2d(p1, Scale_2d(l1_npv, radius)))
				out_points = append(out_points, p1)
				out_points = append(out_points, Add_2d(p1, Scale_2d(l2_npv, radius)))
			default:
				//round join
				rv := Scale_2d(l1_npv, radius)
				rvd := *rv
				rvx, rvy := rvd[0], rvd[1]
				theta := float32(math.Acos(float64(Dot_2d(l1_npv, l2_npv))))
				segs := int((theta/float32(math.Pi))*float32(resolution)) + 1
				for i := 0; i <= segs; i++ {
					angle := float64((float32(i) * theta) / float32(segs))
					s := float32(math.Sin(angle))
					c := float32(math.Cos(angle))
					rv := &Point{rvx*c - rvy*s, rvx*s + rvy*c}
					out_points = append(out_points, p1)
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
	out_points = append(out_points, out_points[0])
	out_points = append(out_points, out_points[1])
	return &out_points
}

func recursive_bezier(x1, y1, x2, y2, x3, y3, x4, y4 float32, pointsp *Points, distance_tolerance float32) *Points {
	//calculate all the mid-points of the line segments
	x12 := (x1 + x2) * 0.5
	y12 := (y1 + y2) * 0.5
	x23 := (x2 + x3) * 0.5
	y23 := (y2 + y3) * 0.5
	x34 := (x3 + x4) * 0.5
	y34 := (y3 + y4) * 0.5
	x123 := (x12 + x23) * 0.5
	y123 := (y12 + y23) * 0.5
	x234 := (x23 + x34) * 0.5
	y234 := (y23 + y34) * 0.5
	x1234 := (x123 + x234) * 0.5
	y1234 := (y123 + y234) * 0.5

	//try to approximate the full cubic curve by a single straight line
	dx := x4 - x1
	dy := y4 - y1

	d2 := float32(math.Abs(float64(((x2-x4)*dy - (y2-y4)*dx))))
	d3 := float32(math.Abs(float64(((x3-x4)*dy - (y3-y4)*dx))))

	points := *pointsp
	if (d2+d3)*(d2+d3) < distance_tolerance*(dx*dx+dy*dy) {
		points = append(points, &Point{x1234, y1234})
		return &points
	}

	//continue subdivision
	points = *recursive_bezier(x1, y1, x12, y12, x123, y123, x1234, y1234, &points, distance_tolerance)
	points = *recursive_bezier(x1234, y1234, x234, y234, x34, y34, x4, y4, &points, distance_tolerance)
	return &points
}

//create bezier path
func Bezier_path(pp1, pp2, pp3, pp4 *Point, distance_tolerance float32) *Points {
	p1, p2, p3, p4 := *pp1, *pp2, *pp3, *pp4
	points := Points{}
	points = append(points, &Point{p1[0], p1[1]})
	points = *recursive_bezier(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1], &points, distance_tolerance)
	points = append(points, &Point{p4[0], p4[1]})
	return &points
}
