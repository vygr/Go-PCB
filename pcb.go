///opt/local/bin/go run
// Copyright (C) 2014 Chris Hinsley.

//package name
package main

//package imports
import (
	"./mymath"
	"./router"
	"bufio"
	"flag"
	"fmt"
	"os"
	"runtime"
	"strconv"
	"strings"
)

func gen_vectors(vec_range, x_range, y_range int) <-chan router.Point {
	yield := make(chan router.Point, 16)
	go func() {
		for y := y_range; y >= -y_range; y-- {
			for x := x_range; x >= -x_range; x-- {
				p := mymath.Point{float32(x), float32(y)}
				if mymath.Length(p) > 0.1 && mymath.Length(p) <= float32(vec_range) {
					yield <- router.Point{x, y, 0}
				}
			}
		}
		yield <- router.Point{0, 0, -1}
		yield <- router.Point{0, 0, 1}
		close(yield)
	}()
	return yield
}

func read_until(r *bufio.Reader, c byte) bool {
	for {
		b, err := r.ReadByte()
		if err != nil {
			//eof
			return true
		}
		if b == c {
			//not eof
			return false
		}
	}
}

func read_dimentions(r *bufio.Reader) router.Dims {
	eof := read_until(r, '[')
	if eof {
		os.Exit(1)
	}
	trim := "()[], "
	string, _ := r.ReadString(',')
	width, _ := strconv.ParseInt(strings.Trim(string, trim), 10, 32)
	string, _ = r.ReadString(',')
	height, _ := strconv.ParseInt(strings.Trim(string, trim), 10, 32)
	string, _ = r.ReadString(']')
	depth, _ := strconv.ParseInt(strings.Trim(string, trim), 10, 32)
	return router.Dims{int(width), int(height), int(depth)}
}

func read_terminal(r *bufio.Reader) router.Terminal {
	eof := read_until(r, '(')
	if eof {
		os.Exit(1)
	}
	trim := "()[], "
	string, _ := r.ReadString(',')
	radius, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString('(')
	string, _ = r.ReadString(',')
	x, _ := strconv.ParseInt(strings.Trim(string, trim), 10, 32)
	string, _ = r.ReadString(',')
	y, _ := strconv.ParseInt(strings.Trim(string, trim), 10, 32)
	string, _ = r.ReadString(')')
	z, _ := strconv.ParseInt(strings.Trim(string, trim), 10, 32)
	eof = read_until(r, ')')
	if eof {
		os.Exit(1)
	}
	return router.Terminal{float32(radius), router.Point{int(x), int(y), int(z)}}
}

func read_terminals(r *bufio.Reader) router.Terminals {
	eof := read_until(r, '[')
	if eof {
		os.Exit(1)
	}
	terminals := router.Terminals{}
	for {
		bytes, _ := r.Peek(1)
		if bytes[0] == ']' {
			break
		}
		terminals = append(terminals, read_terminal(r))
	}
	eof = read_until(r, ']')
	if eof {
		os.Exit(1)
	}
	return terminals
}

func read_track(r *bufio.Reader) (router.Track, bool) {
	eof := read_until(r, '[')
	if eof {
		return router.Track{}, true
	}
	bytes, _ := r.Peek(1)
	if bytes[0] == ']' {
		return router.Track{}, true
	}
	trim := "()[], "
	string, _ := r.ReadString(',')
	radius, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	terminals := read_terminals(r)
	eof = read_until(r, ']')
	if eof {
		os.Exit(1)
	}
	return router.Track{float32(radius), terminals}, false
}

//setup first board, loop for white..black..white..black...
func main() {
	runtime.GOMAXPROCS(1)

	arg_infile := os.Stdin
	var arg_t float64
	var arg_v int
	var arg_s int
	var arg_g float64
	var arg_r int
	var arg_d int
	var arg_fr int
	var arg_xr int
	var arg_yr int

	flag.Float64Var(&arg_t, "t", 600.0, "timeout in seconds, default 600")
	flag.IntVar(&arg_v, "v", 0, "verbosity level 0..1, default 0")
	flag.IntVar(&arg_s, "s", 1, "number of samples, default 1")
	flag.Float64Var(&arg_g, "g", 0.1, "track gap, default 0.1")
	flag.IntVar(&arg_r, "r", 1, "grid resolution 1..4, default 1")
	flag.IntVar(&arg_d, "d", 0, "distance metric 0..5, default 0.\n\t0 -> manhattan\n\t1 -> squared_euclidean\n\t2 -> euclidean\n\t3 -> chebyshev\n\t4 -> reciprocal\n\t5 -> random")
	flag.IntVar(&arg_fr, "fr", 2, "flood range 1..5, default 2")
	flag.IntVar(&arg_xr, "xr", 1, "even layer x range 0..5, default 1")
	flag.IntVar(&arg_yr, "yr", 1, "odd layer y range 0..5, default 1")
	flag.Parse()

	if flag.NArg() > 0 {
		//read access
		file, err := os.Open(flag.Args()[0])
		if err != nil {
			os.Exit(1)
		}
		arg_infile = file
	}
	reader := bufio.NewReader(arg_infile)

	flood_range := arg_fr
	flood_range_x_even_layer := arg_xr
	flood_range_y_odd_layer := arg_yr
	path_range := flood_range + 0
	path_range_x_even_layer := flood_range_x_even_layer + 0
	path_range_y_odd_layer := flood_range_y_odd_layer + 0

	routing_flood_vectorss := make(router.Vectorss, 0)
	routing_flood_vectors := make(router.Vectors, 0)
	for p := range gen_vectors(flood_range, flood_range_x_even_layer, flood_range) {
		routing_flood_vectors = append(routing_flood_vectors, p)
	}
	routing_flood_vectorss = append(routing_flood_vectorss, routing_flood_vectors)
	routing_flood_vectors = make(router.Vectors, 0)
	for p := range gen_vectors(flood_range, flood_range, flood_range_y_odd_layer) {
		routing_flood_vectors = append(routing_flood_vectors, p)
	}
	routing_flood_vectorss = append(routing_flood_vectorss, routing_flood_vectors)

	routing_path_vectorss := make(router.Vectorss, 0)
	routing_path_vectors := make(router.Vectors, 0)
	for p := range gen_vectors(path_range, path_range_x_even_layer, path_range) {
		routing_path_vectors = append(routing_path_vectors, p)
	}
	routing_path_vectorss = append(routing_path_vectorss, routing_path_vectors)
	routing_path_vectors = make(router.Vectors, 0)
	for p := range gen_vectors(path_range, path_range, path_range_y_odd_layer) {
		routing_path_vectors = append(routing_path_vectors, p)
	}
	routing_path_vectorss = append(routing_path_vectorss, routing_path_vectors)

	dfuncs := []func(mymath.Point, mymath.Point) float32{
		mymath.Manhattan_distance, mymath.Squared_euclidean_distance, mymath.Euclidean_distance,
		mymath.Chebyshev_distance, mymath.Reciprical_distance, mymath.Random_distance}

	dimensions := read_dimentions(reader)
	pcb := router.NewPcb(dimensions, &routing_flood_vectorss, &routing_path_vectorss,
		dfuncs[arg_d], arg_r, arg_v, float32(arg_g))
	for {
		track, eof := read_track(reader)
		if eof == true {
			break
		}
		pcb.Add_track(track)
	}

	pcb.Print()
	best_cost := 1000000000
	var best_pcb *router.Pcb
	best_pcb = nil
	for i := 0; i < arg_s; i++ {
		if !pcb.Route(arg_t) {
			pcb.Shuffle_netlist()
			continue
		}
		cost := pcb.Cost()
		print("Cost=", cost)
		if cost <= best_cost {
			print("1-------")
			best_cost = cost
			best_pcb = pcb.Copy()
		}
		pcb.Shuffle_netlist()
	}
	if best_pcb != nil {
		print("2-------")
		best_pcb.Print_netlist()
	} else {
		fmt.Println("[]")
	}
}
