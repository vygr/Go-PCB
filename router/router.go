// Copyright (C) 2014 Chris Hinsley.

//package name
package router

//package imports
import (
	"../layer"
	"../mymath"
	"fmt"
	"math/rand"
	"time"
)

type Dims struct {
	Width  int
	Height int
	Depth  int
}

type Point struct {
	X int
	Y int
	Z int
}
type Vectors []Point
type Vectorss []Vectors

type sort_point struct {
	mark float32
	node Point
}
type sort_points []sort_point

func zip_2_vectors(p1, p2 Vectors) <-chan Vectors {
	l1 := len(p1)
	l2 := len(p2)
	l := l1
	if l2 < l1 {
		l = l2
	}
	yield := make(chan Vectors, l)
	go func() {
		for i := 0; i < l; i++ {
			yield <- Vectors{p1[i], p2[i]}
		}
		close(yield)
	}()
	return yield
}

func point_equal(n1, n2 Point) bool {
	if n1.X != n2.X {
		return false
	}
	if n1.Y != n2.Y {
		return false
	}
	if n1.Z != n2.Z {
		return false
	}
	return true
}

func point_to_math_point(p Point) mymath.Point {
	return mymath.Point{float32(p.X),float32(p.Y),float32(p.Z)}
}

func vectors_contain(nodes Vectors, node Point) bool {
	for i := 0; i < len(nodes); i++ {
		if point_equal(nodes[i], node) {
			return true
		}
	}
	return false
}

func merge_point(nodes Vectors, node Point) Vectors {
	if !vectors_contain(nodes, node) {
		nodes = append(nodes, Point{node.X, node.Y, node.Z})
	}
	return nodes
}

func merge_vectors(nodes Vectors, mnodes Vectors) Vectors {
	for i := 0; i < len(mnodes); i++ {
		nodes = merge_point(nodes, mnodes[i])
	}
	return nodes
}

func copy_vectors(nodes Vectors) Vectors {
	cn := make(Vectors, len(nodes), cap(nodes))
	copy(cn, nodes)
	return cn
}

func append_sort_point(nodes sort_points, node Point, mark float32) sort_points {
	mn := sort_point{mark, node}
	return append(nodes, mn)
}

func insert_sort_point(nodes sort_points, node Point, mark float32, ad bool) sort_points {
	if ad {
		//assending
		for i := 0; i < len(nodes); i++ {
			if nodes[i].mark <= mark {
				mn := sort_point{mark, node}
				nodes = append(nodes, mn)
				copy(nodes[i+1:], nodes[i:])
				nodes[i] = mn
				return nodes
			}
		}
	} else {
		//desending
		for i := 0; i < len(nodes); i++ {
			if nodes[i].mark >= mark {
				mn := sort_point{mark, node}
				nodes = append(nodes, mn)
				copy(nodes[i+1:], nodes[i:])
				nodes[i] = mn
				return nodes
			}
		}
	}
	return append_sort_point(nodes, node, mark)
}

func optimise_paths(paths Vectorss) Vectorss {
	opt_paths := make(Vectorss, 0)
	for _, path := range paths {
		opt_path := make(Vectors, 0)
		d := mymath.Point{0, 0, 0}
		for p := range zip_2_vectors(path, path[1:]) {
			p0 := mymath.Point{float32(p[0].X), float32(p[0].Y), float32(p[0].Z)}
			p1 := mymath.Point{float32(p[1].X), float32(p[1].Y), float32(p[1].Z)}
			d1 := mymath.Normalise_3d(mymath.Sub_3d(p1, p0))
			if !mymath.Equal(d1, d) {
				opt_path = append(opt_path, p[0])
				d = d1
			}
		}
		opt_path = append(opt_path, path[len(path)-1])
		opt_paths = append(opt_paths, opt_path)
	}
	return opt_paths
}

type Nets []*Net

type Terminal struct {
	Radius float32
	Term   Point
}
type Terminals []Terminal

type Track struct {
	Radius float32
	Terms  Terminals
}

//pcb object
type Pcb struct {
	width                 int
	height                int
	depth                 int
	stride                int
	routing_flood_vectors Vectorss
	routing_path_vectors  Vectorss
	dfunc                 func(mymath.Point, mymath.Point) float32
	resolution            int
	verbosity             int
	track_gap             float32
	layers                *layer.Layers
	netlist               []*Net
	nodes                 []int
}

//pcb methods
func NewPcb(dims Dims, rfvs, rpvs Vectorss, dfunc func(mymath.Point, mymath.Point) float32,
	res, verb int, tg float32) *Pcb {
	p := Pcb{}
	p.Init(dims, rfvs, rpvs, dfunc, res, verb, tg)
	return &p
}

func (self *Pcb) Init(dims Dims, rfvs, rpvs Vectorss,
	dfunc func(mymath.Point, mymath.Point) float32, res, verb int, tg float32) {
	self.width = dims.Width
	self.height = dims.Height
	self.depth = dims.Depth
	self.routing_flood_vectors = rfvs
	self.routing_path_vectors = rpvs
	self.dfunc = dfunc
	self.resolution = res
	self.verbosity = verb
	self.track_gap = tg
	self.layers = layer.NewLayers(layer.Layers_dims{self.width, self.height, self.depth}, 1.0/float32(res))
	self.netlist = nil
	self.width *= res
	self.height *= res
	self.stride = self.width * self.height
	self.nodes = make([]int, self.stride*self.depth, self.stride*self.depth)
}

func (self *Pcb) Copy() *Pcb {
	new_pcb := Pcb{}
	new_pcb.width = self.width
	new_pcb.height = self.height
	new_pcb.depth = self.depth
	new_pcb.stride = self.stride
	new_pcb.routing_flood_vectors = self.routing_flood_vectors
	new_pcb.routing_path_vectors = self.routing_path_vectors
	new_pcb.dfunc = self.dfunc
	new_pcb.resolution = self.resolution
	new_pcb.verbosity = self.verbosity
	new_pcb.track_gap = self.track_gap
	new_pcb.layers = layer.NewLayers(layer.Layers_dims{self.width, self.height, self.depth}, 1.0/float32(self.resolution))
	new_pcb.netlist = nil
	for _, net := range self.netlist {
		new_pcb.netlist = append(new_pcb.netlist, net.Copy())
	}
	new_pcb.nodes = make([]int, self.stride*self.depth, self.stride*self.depth)
	return &new_pcb
}

func (self *Pcb) Set_node(node Point, value int) {
	self.nodes[(self.stride*node.Z)+(node.Y*self.width)+node.X] = value
}

func (self *Pcb) Get_node(node Point) int {
	return self.nodes[(self.stride*node.Z)+(node.Y*self.width)+node.X]
}

func (self *Pcb) All_nodes(vectors *Vectorss, node Point) <-chan Point {
	yield := make(chan Point, 16)
	go func() {
		vec := *vectors
		x, y, z := node.X, node.Y, node.Z
		for _, v := range vec[z%2] {
			dx, dy, dz := v.X, v.Y, v.Z
			nx := x + dx
			ny := y + dy
			nz := z + dz
			if (0 <= nx) && (nx < self.width) && (0 <= ny) && (ny < self.height) && (0 <= nz) && (nz < self.depth) {
				yield <- Point{nx, ny, nz}
			}
		}
		close(yield)
	}()
	return yield
}

func (self *Pcb) All_marked(vectors *Vectorss, node Point) <-chan sort_point {
	yield := make(chan sort_point, 16)
	go func() {
		for n := range self.All_nodes(vectors, node) {
			mark := self.Get_node(n)
			if mark != 0 {
				yield <- sort_point{float32(mark), n}
			}
		}
		close(yield)
	}()
	return yield
}

func (self *Pcb) All_not_marked(vectors *Vectorss, node Point) <-chan Point {
	yield := make(chan Point, 16)
	go func() {
		for n := range self.All_nodes(vectors, node) {
			if self.Get_node(n) == 0 {
				yield <- n
			}
		}
		close(yield)
	}()
	return yield
}

func (self *Pcb) All_nearer_sorted(vectors *Vectorss, node, goal Point,
	dfunc func(mymath.Point, mymath.Point) float32) <-chan Point {
	yield := make(chan Point, 16)
	go func() {
		gp := point_to_math_point(goal)
		distance := float32(self.Get_node(node))
		nodes := make(sort_points, 0)
		for mn := range self.All_marked(vectors, node) {
			if (distance - mn.mark) > 0 {
				mnp := point_to_math_point(mn.node)
				nodes = insert_sort_point(nodes, mn.node, dfunc(mnp, gp), false)
			}
		}
		for _, node := range nodes {
			yield <- node.node
		}
		close(yield)
	}()
	return yield
}

func (self *Pcb) All_not_shorting(gather <-chan Point, node Point, radius float32) <-chan Point {
	yield := make(chan Point, 16)
	go func() {
		np := point_to_math_point(node)
		for new_node := range gather {
			nnp := point_to_math_point(new_node)
			if !self.layers.Hit_line(np, nnp, radius) {
				yield <- new_node
			}
		}
		close(yield)
	}()
	return yield
}

func (self *Pcb) Mark_distances(vectors *Vectorss, radius float32, starts, ends Vectors) {
	distance := 1
	nodes := starts
	for _, node := range nodes {
		self.Set_node(node, distance)
	}
	for len(nodes) > 0 {
		distance++
		new_nodes := make(Vectors, 0)
		for _, node := range nodes {
			for new_node := range self.All_not_shorting(self.All_not_marked(vectors, node), node, radius) {
				self.Set_node(new_node, distance)
				new_nodes = append(new_nodes, new_node)
			}
		}
		nodes = new_nodes
		flag := false
		for _, node := range ends {
			if self.Get_node(node) == 0 {
				flag = true
			}
		}
		if !flag {
			break
		}
	}
}

func (self *Pcb) Unmark_distances() {
	self.nodes = make([]int, self.stride*self.depth, self.stride*self.depth)
}

func (self *Pcb) Add_track(t Track) {
	self.netlist = append(self.netlist, NewNet(t.Terms, t.Radius, *self))
}

func hoist_netlist(ns Nets, n int) Nets {
	new_nets := make(Nets, 0, len(ns))
	new_nets = append(new_nets, ns[n])
	for i := 0; i < len(ns); i++ {
		if i != n {
			new_nets = append(new_nets, ns[i])
		}
	}
	return new_nets
}

func (self *Pcb) Route(timeout float64) bool {
	start_time := time.Now()
	index := 0
	for index < len(self.netlist) {
		if self.netlist[index].Route() {
			index += 1
		} else {
			for {
				self.netlist[index].Reset_topology()
				if index == 0 {
					self.Shuffle_netlist()
					break
				} else {
					index -= 1
					self.netlist[index].Remove()
					if self.netlist[index].Next_topology() {
						break
					} else {
						self.netlist = hoist_netlist(self.netlist, index+1)
						for index != 0 {
							self.netlist[index].Remove()
							self.netlist[index].Reset_topology()
							index -= 1
						}
						break
					}
				}
			}
		}
		if time.Since(start_time).Seconds() > timeout {
			return false
		}
		if self.verbosity >= 1 {
			self.Print_netlist()
		}
	}
	return true
}

func (self *Pcb) Cost() int {
	sum := 0
	for _, net := range self.netlist {
		for _, path := range net.paths {
			sum += len(path)
		}
	}
	return sum
}

func shuffle_netlist(ns Nets) Nets {
	new_nets := make(Nets, 0, len(ns))
	for _, i := range rand.Perm(len(ns)) {
		new_nets = append(new_nets, ns[i])
	}
	return new_nets
}

func (self *Pcb) Shuffle_netlist() {
	for _, net := range self.netlist {
		net.Remove()
		net.Shuffle_topology()
	}
	self.netlist = shuffle_netlist(self.netlist)
}

func (self *Pcb) Print() {
	scale := 1.0 / float32(self.resolution)
	fmt.Print("[")
	fmt.Print(float32(self.width)*scale, ",")
	fmt.Print(float32(self.height)*scale, ",")
	fmt.Print(self.depth)
	fmt.Println("]")
}

func (self *Pcb) Print_netlist() {
	for _, net := range self.netlist {
		net.Print()
	}
	fmt.Println("[]")
}

//net object
type Net struct {
	pcb       Pcb
	terminals Terminals
	radius    float32
	shift     int
	paths     Vectorss
}

//net methods
func NewNet(terms Terminals, radius float32, pcb Pcb) *Net {
	n := Net{}
	n.Init(terms, radius, pcb)
	return &n
}

func scale_terminals(terms Terminals, res int) Terminals {
	for i:=0; i<len(terms); i++{
		terms[i].Radius *= float32(res)
		terms[i].Term.X *= res
		terms[i].Term.Y *= res
		terms[i].Term.Z *= res
	}
	return terms
}

func (self *Net) Init(terms Terminals, radius float32, pcb Pcb) {
	self.pcb = pcb
	self.radius = radius * float32(pcb.resolution)
	self.shift = 0
	self.paths = make(Vectorss, 0)
	self.terminals = scale_terminals(terms, pcb.resolution)
	self.Remove()
}

func copy_terminals(terms Terminals) Terminals {
	new_terms := make(Terminals, len(terms), cap(terms))
	copy(new_terms, terms)
	return new_terms
}

func (self *Net) Copy() *Net {
	new_net := Net{}
	new_net.pcb = self.pcb
	new_net.radius = self.radius
	new_net.shift = 0
	new_net.terminals = copy_terminals(self.terminals)
	new_net.paths = optimise_paths(self.paths)
	self.Remove()
	return &new_net
}

func shift_terminals(terms Terminals, n int) Terminals {
	l := len(terms)
	n = n % l
	if n < 0 {
		n += l
	}
	if n == 0 {
		return terms
	}
	return append(terms[n:l], terms[:n]...)
}

func (self *Net) Next_topology() bool {
	self.shift += 1
	self.terminals = shift_terminals(self.terminals, 1)
	if self.shift == len(self.terminals) {
		self.shift = 0
		return false
	}
	return true
}

func (self *Net) Reset_topology() {
	self.terminals = shift_terminals(self.terminals, -self.shift)
	self.shift = 0
}

func shuffle_terminals(terms Terminals) Terminals {
	new_terms := make(Terminals, 0, len(terms))
	for _, i := range rand.Perm(len(terms)) {
		new_terms = append(new_terms, terms[i])
	}
	return new_terms
}

func (self *Net) Shuffle_topology() {
	self.terminals = shuffle_terminals(self.terminals)
}

func (self *Net) Add_terminal_collision_lines() {
	for _, node := range self.terminals {
		r, x, y := node.Radius, float32(node.Term.X), float32(node.Term.Y)
		self.pcb.layers.Add_line(mymath.Point{x, y, 0}, mymath.Point{x, y, float32(self.pcb.depth)}, r)
	}
}

func (self *Net) Sub_terminal_collision_lines() {
	for _, node := range self.terminals {
		r, x, y := node.Radius, float32(node.Term.X), float32(node.Term.Y)
		self.pcb.layers.Sub_line(mymath.Point{x, y, 0}, mymath.Point{x, y, float32(self.pcb.depth)}, r)
	}
}

func (self *Net) Add_paths_collision_lines() {
	for _, path := range self.paths {
		for p := range zip_2_vectors(path, path[1:]) {
			p0 := mymath.Point{float32(p[0].X), float32(p[0].Y), float32(p[0].Z)}
			p1 := mymath.Point{float32(p[1].X), float32(p[1].Y), float32(p[1].Z)}
			self.pcb.layers.Add_line(p0, p1, self.radius)
		}
	}
}

func (self *Net) Sub_paths_collision_lines() {
	for _, path := range self.paths {
		for p := range zip_2_vectors(path, path[1:]) {
			p0 := mymath.Point{float32(p[0].X), float32(p[0].Y), float32(p[0].Z)}
			p1 := mymath.Point{float32(p[1].X), float32(p[1].Y), float32(p[1].Z)}
			self.pcb.layers.Sub_line(p0, p1, self.radius)
		}
	}
}

func (self *Net) Remove() {
	self.Sub_paths_collision_lines()
	self.Sub_terminal_collision_lines()
	self.paths = nil
	self.Add_terminal_collision_lines()
}

func (self *Net) Route() bool {
	self.paths = make(Vectorss, 0)
	self.Sub_terminal_collision_lines()
	radius := self.radius + (self.pcb.track_gap * float32(self.pcb.resolution))
	visited := make(Vectors, 0)
	for index := 1; index < len(self.terminals); index++ {
		starts := make(Vectors, 0, (len(self.terminals[0:index]) * self.pcb.depth))
		for _, t := range self.terminals[0:index] {
			for z := 0; z < self.pcb.depth; z++ {
				x, y := int(t.Term.X), int(t.Term.Y)
				starts = append(starts, Point{x, y, z})
			}
		}
		ends := make(Vectors, 0, (len(self.terminals[index : index+1]) * self.pcb.depth))
		for _, t := range self.terminals[index : index+1] {
			for z := 0; z < self.pcb.depth; z++ {
				x, y := t.Term.X, t.Term.Y
				ends = append(ends, Point{x, y, z})
			}
		}
		visited = merge_vectors(visited, starts)
		self.pcb.Mark_distances(&self.pcb.routing_flood_vectors, radius, visited, ends)
		end_nodes := make(sort_points, 0, len(ends))
		for _, node := range ends {
			mark := self.pcb.Get_node(node)
			end_nodes = insert_sort_point(end_nodes, node, float32(mark), false)
		}
		end := end_nodes[0].node
		path := make(Vectors, 0)
		path = append(path, end)
		dv := mymath.Point{0, 0, 0}
		for {
			pn := path[len(path)-1]
			if vectors_contain(visited, pn) {
				break
			}
			nearer_nodes := make(Vectors, 0)
			for node := range self.pcb.All_not_shorting(
				self.pcb.All_nearer_sorted(&self.pcb.routing_path_vectors, pn, end, self.pcb.dfunc),
				pn, radius) {
				nearer_nodes = append(nearer_nodes, node)
			}
			if len(nearer_nodes) == 0 {
				//no nearer node
				self.pcb.Unmark_distances()
				self.Remove()
				return false
			}
			next_node := nearer_nodes[0]
			if !vectors_contain(visited, next_node) {
				for i := 1; i < len(nearer_nodes); i++ {
					if vectors_contain(visited, nearer_nodes[i]) {
						next_node = nearer_nodes[i]
						break
					}
					n := nearer_nodes[i]
					dv1 := mymath.Normalise_3d(point_to_math_point(n))
					dv2 := mymath.Normalise_3d(point_to_math_point(pn))
					if mymath.Equal_3d(dv, mymath.Sub_3d(dv1, dv2)) {
						next_node = nearer_nodes[i]
					}
				}
			}
			dv1 := mymath.Normalise_3d(point_to_math_point(next_node))
			dv2 := mymath.Normalise_3d(point_to_math_point(pn))
			dv = mymath.Normalise_3d(mymath.Sub_3d(dv1, dv2))
			path = append(path, next_node)
		}
		visited = merge_vectors(visited, path)
		self.paths = append(self.paths, path)
		self.pcb.Unmark_distances()
	}
	self.paths = optimise_paths(self.paths)
	self.Add_paths_collision_lines()
	self.Add_terminal_collision_lines()
	return true
}

func (self *Net) Print() {
	scale := 1.0 / float32(self.pcb.resolution)
	fmt.Print("[")
	fmt.Print(self.radius*scale, ",")
	fmt.Print("[")
	for i, t := range self.terminals {
		r, x, y, z := t.Radius, float32(t.Term.X), float32(t.Term.Y), float32(t.Term.Z)
		fmt.Print("(")
		fmt.Print(r*scale, ",")
		fmt.Print("(")
		fmt.Print(x*scale, ",")
		fmt.Print(y*scale, ",")
		fmt.Print(z)
		fmt.Print("))")
		if i != (len(self.terminals) - 1) {
			fmt.Print(",")
		}
	}
	fmt.Print("],[")
	for i, path := range self.paths {
		fmt.Print("[")
		for j, p := range path {
			x, y, z := float32(p.X), float32(p.Y), float32(p.Z)
			fmt.Print("(")
			fmt.Print(x*scale, ",")
			fmt.Print(y*scale, ",")
			fmt.Print(z)
			fmt.Print(")")
			if j != (len(path) - 1) {
				fmt.Print(",")
			}
		}
		fmt.Print("]")
		if i != (len(self.paths) - 1) {
			fmt.Print(",")
		}
	}
	fmt.Println("]]")
}
