///opt/local/bin/go run
// Copyright (C) 2014 Chris Hinsley.

//package name
package main

//package imports
import (
	"./router"
	"bufio"
	"flag"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
)

type tree struct {
	value    *string
	branches []*tree
}

type pin struct {
	form  *string
	angle float64
	name  *string
	x     float64
	y     float64
}

type component struct {
	name    *string
	pin_map map[string]*pin
}

type instance struct {
	name  *string
	comp  *string
	x     float64
	y     float64
	side  *string
	angle float64
}

type rule struct {
	radius float64
	gap    float64
}

type circuit struct {
	via  *string
	rule *rule
}

func peek_char(r *bufio.Reader) (byte, bool) {
	bytes, err := r.Peek(1)
	if err != nil {
		//eof
		return ' ', true
	}
	return bytes[0], false
}

func read_char(r *bufio.Reader) (byte, bool) {
	b, err := r.ReadByte()
	if err != nil {
		//eof
		return ' ', true
	}
	return b, false
}

func read_whitespace(r *bufio.Reader) {
	for {
		b, eof := peek_char(r)
		if eof {
			return
		}
		if b != ' ' && b != '\t' && b != '\r' && b != '\n' {
			return
		}
		read_char(r)
	}
}

func peek_until(r *bufio.Reader, c byte) {
	for {
		b, eof := peek_char(r)
		if eof {
			return
		}
		if b == c {
			return
		}
		read_char(r)
	}
}

func read_until(r *bufio.Reader, c byte) {
	for {
		b, eof := read_char(r)
		if eof {
			return
		}
		if b == c {
			return
		}
	}
}

func read_node_name(r *bufio.Reader) *string {
	s := ""
	for {
		b, eof := peek_char(r)
		if eof || b == '\t' || b == '\n' || b == '\r' || b == ' ' || b == ')' {
			break
		}
		s += string(b)
		read_char(r)
	}
	return &s
}

func read_string(r *bufio.Reader) *tree {
	t := tree{nil, nil}
	s := ""
	for {
		b, eof := peek_char(r)
		if eof || b == '\t' || b == '\n' || b == '\r' || b == ' ' || b == ')' {
			break
		}
		s += string(b)
		read_char(r)
	}
	t.value = &s
	return &t
}

func read_quoted_string(r *bufio.Reader) *tree {
	t := tree{nil, nil}
	s := ""
	for {
		b, eof := peek_char(r)
		if eof || b == '"' {
			break
		}
		s += string(b)
		read_char(r)
	}
	t.value = &s
	return &t
}

func read_tree(r *bufio.Reader) *tree {
	read_until(r, '(')
	read_whitespace(r)
	t := tree{read_node_name(r), nil}
	for {
		read_whitespace(r)
		b, eof := peek_char(r)
		if eof {
			break
		}
		if b == ')' {
			read_char(r)
			break
		}
		if b == '(' {
			t.branches = append(t.branches, read_tree(r))
			continue
		}
		if b == '"' {
			read_char(r)
			t.branches = append(t.branches, read_quoted_string(r))
			read_char(r)
			continue
		}
		t.branches = append(t.branches, read_string(r))
	}
	return &t
}

func search_tree(t *tree, s *string) *tree {
	if t.value != nil {
		if *t.value == *s {
			return t
		}
	}
	for _, ct := range t.branches {
		st := search_tree(ct, s)
		if st != nil {
			return st
		}
	}
	return nil
}

func print_tree(t *tree, indent int) {
	if t.value != nil {
		for i := 0; i < indent; i++ {
			fmt.Print("  ")
		}
		fmt.Print(*t.value, "\n")
	}
	for _, ct := range t.branches {
		print_tree(ct, indent+1)
	}
}

func main() {
	//command line flags and defaults etc
	arg_infile := os.Stdin
	var arg_b int
	flag.IntVar(&arg_b, "b", 0, "border gap, default 0")
	flag.Parse()

	//input reader from default stdin or given file
	if flag.NArg() > 0 {
		//read access
		file, err := os.Open(flag.Args()[0])
		if err != nil {
			os.Exit(1)
		}
		arg_infile = file
	}
	reader := bufio.NewReader(arg_infile)

	//create tree from input
	tree := read_tree(reader)

	ss := "structure"
	structure_node := search_tree(tree, &ss)
	num_layers := 0
	minx := 1000000.0
	miny := 1000000.0
	maxx := -1000000.0
	maxy := -1000000.0
	for _, struc_node := range structure_node.branches {
		if *struc_node.value == "layer" {
			num_layers++
		}
		if *struc_node.value == "boundary" {
			cords := struc_node.branches[0].branches[2:]
			for i := 0; i < len(cords); i += 2 {
				px, _ := strconv.ParseFloat(*cords[i].value, 32)
				py, _ := strconv.ParseFloat(*cords[i+1].value, 32)
				px /= 1000.0
				py /= 1000.0
				if px < minx {
					minx = px
				}
				if px > maxx {
					maxx = px
				}
				if py < miny {
					miny = py
				}
				if py > maxy {
					maxy = py
				}
			}
		}
	}

	ss = "library"
	library_node := search_tree(tree, &ss)
	component_map := map[string]*component{}
	rule_map := map[string]*rule{}
	for _, lib_node := range library_node.branches {
		if *lib_node.value == "image" {
			component_name := lib_node.branches[0].value
			component := component{component_name, map[string]*pin{}}
			for _, pin_node := range lib_node.branches[1:] {
				if *pin_node.value == "pin" {
					pin := pin{}
					pin.form = pin_node.branches[0].value
					if *pin_node.branches[1].value == "rotate" {
						pin.angle, _ = strconv.ParseFloat(*pin_node.branches[1].branches[0].value, 32)
						pin.name = pin_node.branches[2].value
						pin.x, _ = strconv.ParseFloat(*pin_node.branches[3].value, 32)
						pin.y, _ = strconv.ParseFloat(*pin_node.branches[4].value, 32)
					} else {
						pin.angle = 0.0
						pin.name = pin_node.branches[1].value
						pin.x, _ = strconv.ParseFloat(*pin_node.branches[2].value, 32)
						pin.y, _ = strconv.ParseFloat(*pin_node.branches[3].value, 32)
					}
					pin.x /= 1000.0
					pin.y /= 1000.0
					component.pin_map[*pin.name] = &pin
				}
			}
			component_map[*component_name] = &component
		}
		if *lib_node.value == "padstack" {
			rule := rule{0.5, 0.125}
			if *lib_node.branches[1].branches[0].value == "circle" {
				rule.radius, _ = strconv.ParseFloat(*lib_node.branches[1].branches[0].branches[1].value, 32)
				rule.radius /= 2000.0
			}
			if *lib_node.branches[1].branches[0].value == "path" {
				rule.radius, _ = strconv.ParseFloat(*lib_node.branches[1].branches[0].branches[1].value, 32)
				rule.radius /= 2000.0
			}
			if *lib_node.branches[1].branches[0].value == "rect" {
				minx, _ := strconv.ParseFloat(*lib_node.branches[1].branches[0].branches[1].value, 32)
				miny, _ := strconv.ParseFloat(*lib_node.branches[1].branches[0].branches[2].value, 32)
				maxx, _ := strconv.ParseFloat(*lib_node.branches[1].branches[0].branches[3].value, 32)
				maxy, _ := strconv.ParseFloat(*lib_node.branches[1].branches[0].branches[4].value, 32)
				xd := maxx - minx
				yd := maxy - miny
				r := xd
				if yd < xd {
					r = yd
				}
				rule.radius = r / 2000.0
			}
			rule_map[*lib_node.branches[0].value] = &rule
		}
	}

	ss = "placement"
	placement_tree := search_tree(tree, &ss)
	instance_map := map[string]*instance{}
	for _, node := range placement_tree.branches {
		component_name := node.branches[0].value
		for i := 1; i < len(node.branches); i++ {
			place_tree := *node.branches[i]
			instance := instance{}
			instance_name := place_tree.branches[0].value
			instance.name = instance_name
			instance.comp = component_name
			instance.x, _ = strconv.ParseFloat(*place_tree.branches[1].value, 32)
			instance.y, _ = strconv.ParseFloat(*place_tree.branches[2].value, 32)
			instance.side = place_tree.branches[3].value
			instance.angle, _ = strconv.ParseFloat(*place_tree.branches[4].value, 32)
			instance.x /= 1000.0
			instance.y /= 1000.0
			instance_map[*instance_name] = &instance
		}
	}

	all_terminals := router.Terminals{}
	for _, instance := range instance_map {
		component := component_map[*instance.comp]
		for _, pin := range component.pin_map {
			x := pin.x
			y := pin.y
			if *instance.side != "front" {
				x = -x
			}
			angle := instance.angle * (math.Pi / 180.0)
			s := math.Sin(angle)
			c := math.Cos(angle)
			px := (c*x - s*y) + instance.x
			py := (s*x + c*y) + instance.y
			pin_rule := rule_map[*pin.form]
			tp := router.Tpoint{float32(px), float32(py), 0.0}
			all_terminals = append(all_terminals, &router.Terminal{float32(pin_rule.radius), float32(pin_rule.gap), tp})
			if px < minx {
				minx = px
			}
			if px > maxx {
				maxx = px
			}
			if py < miny {
				miny = py
			}
			if py > maxy {
				maxy = py
			}
		}
	}

	ss = "network"
	network_node := search_tree(tree, &ss)
	circuit_map := map[string]*circuit{}
	for _, node := range network_node.branches {
		if *node.value == "class" {
			net_rule := rule{0.125, 0.125}
			circuit := circuit{nil, &net_rule}
			for _, class_node := range node.branches {
				if *class_node.value == "rule" {
					for _, dims := range class_node.branches {
						if *dims.value == "width" {
							net_rule.radius, _ = strconv.ParseFloat(*dims.branches[0].value, 32)
							net_rule.radius /= 2000.0
						}
						if *dims.value == "clearance" {
							net_rule.gap, _ = strconv.ParseFloat(*dims.branches[0].value, 32)
							net_rule.gap /= 2000.0
						}
					}
				}
				if *class_node.value == "circuit" {
					circuit.via = class_node.branches[0].branches[0].value
				}
			}
			for _, netname := range node.branches {
				if netname.branches == nil {
					circuit_map[*netname.value] = &circuit
				}
			}
		}
	}
	tracks := make([]router.Track, 0)
	for _, node := range network_node.branches {
		if *node.value == "net" {
			terminals := router.Terminals{}
			for _, pin := range node.branches[1].branches {
				pin_info := strings.Split(*pin.value, "-")
				instance_name := pin_info[0]
				pin_name := pin_info[1]
				instance := instance_map[instance_name]
				component := component_map[*instance.comp]
				pin := component.pin_map[pin_name]
				x := pin.x
				y := pin.y
				if *instance.side != "front" {
					x = -x
				}
				angle := instance.angle * (math.Pi / 180.0)
				s := math.Sin(angle)
				c := math.Cos(angle)
				px := (c*x - s*y) + instance.x
				py := (s*x + c*y) + instance.y
				pin_rule := rule_map[*pin.form]
				tp := router.Tpoint{float32(px), float32(py), 0.0}
				term := router.Terminal{float32(pin_rule.radius), float32(pin_rule.gap), tp}
				terminals = append(terminals, &term)
				for i, t := range all_terminals {
					if *t == term {
						all_terminals = append(all_terminals[:i], all_terminals[i+1:]...)
						break
					}
				}
			}
			circuit := circuit_map[*node.branches[0].value]
			net_rule := circuit.rule
			via_rule := rule_map[*circuit.via]
			tracks = append(tracks, router.Track{float32(net_rule.radius), float32(via_rule.radius), float32(net_rule.gap), terminals})
		}
	}
	tracks = append(tracks, router.Track{0.0, 0.0, 0.0, all_terminals})

	border := float64(arg_b)
	fmt.Print("[", int(maxx-minx+(border*2)+0.5), ",", int(maxy-miny+(border*2)+0.5), ",", num_layers, "]\n")
	for _, track := range tracks {
		fmt.Print("[")
		fmt.Print(track.Radius, ",")
		fmt.Print(track.Via, ",")
		fmt.Print(track.Gap, ",")
		fmt.Print("[")
		for i, t := range track.Terms {
			fmt.Print("(")
			fmt.Print(t.Radius, ",")
			fmt.Print(t.Gap, ",")
			fmt.Print("(")
			fmt.Print(t.Term.X-float32(minx+border), ",")
			fmt.Print(t.Term.Y-float32(miny+border), ",")
			fmt.Print(t.Term.Z)
			fmt.Print("))")
			if i != (len(track.Terms) - 1) {
				fmt.Print(",")
			}
		}
		fmt.Println("]]")
	}
}
