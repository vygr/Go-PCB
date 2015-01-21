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
	for _, layer_node := range structure_node.branches {
		if *layer_node.value == "layer" {
			num_layers++
		}
	}

	ss = "library"
	library_node := search_tree(tree, &ss)
	component_map := map[string]*component{}
	for _, image_node := range library_node.branches {
		if *image_node.value == "image" {
			component_name := image_node.branches[0].value
			component := component{component_name, map[string]*pin{}}
			for _, pin_node := range image_node.branches[1:] {
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
					component.pin_map[*pin.name] = &pin
				}
			}
			component_map[*component_name] = &component
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
			instance_map[*instance_name] = &instance
		}
	}

	minx := float32(1000000)
	miny := float32(1000000)
	maxx := float32(-1000000)
	maxy := float32(-1000000)
	all_tpoints := []router.Tpoint{}
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
			px := float32(((c*x - s*y) + instance.x) / 1000.0)
			py := float32(((s*x + c*y) + instance.y) / 1000.0)
			all_tpoints = append(all_tpoints, router.Tpoint{px, py, 0.0})
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
	rule_map := map[string]*rule{}
	for _, node := range network_node.branches {
		if *node.value == "class" {
			rule := rule{125, 125}
			for _, rule_node := range node.branches {
				if *rule_node.value == "rule" {
					for _, dims := range rule_node.branches {
						if *dims.value == "width" {
							rule.radius, _ = strconv.ParseFloat(*dims.branches[0].value, 32)
							rule.radius /= 2.0
						}
						if *dims.value == "clearance" {
							rule.gap, _ = strconv.ParseFloat(*dims.branches[0].value, 32)
							rule.gap /= 2.0
						}
					}
				}
			}
			for _, netname := range node.branches {
				if netname.branches == nil {
					rule_map[*netname.value] = &rule
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
				px := float32(((c*x - s*y) + instance.x) / 1000.0)
				py := float32(((s*x + c*y) + instance.y) / 1000.0)
				term := router.Tpoint{px, py, 0.0}
				terminals = append(terminals, &router.Terminal{0.5, term})

				for i, t := range all_tpoints {
					if t == term {
						all_tpoints = append(all_tpoints[:i], all_tpoints[i+1:]...)
						break
					}
				}
			}
			rule := rule_map[*node.branches[0].value]
			tracks = append(tracks, router.Track{float32(rule.radius) / 1000.0, float32(rule.gap) / 1000.0, 0.25, terminals})
		}
	}
	terminals := router.Terminals{}
	for _, term := range all_tpoints {
		terminals = append(terminals, &router.Terminal{0.5, term})
	}
	tracks = append(tracks, router.Track{0.0, 0.0, 0.0, terminals})

	border := float32(arg_b)
	fmt.Print("[", int(maxx-minx+(border*2)+0.5)+1, ",", int(maxy-miny+(border*2)+0.5)+1, ",", num_layers, "]\n")
	for _, track := range tracks {
		fmt.Print("[")
		fmt.Print(track.Radius, ",")
		fmt.Print(track.Gap, ",")
		fmt.Print(track.Via, ",")
		fmt.Print("[")
		for i, t := range track.Terms {
			r, x, y, z := t.Radius, t.Term.X, t.Term.Y, t.Term.Z
			fmt.Print("(")
			fmt.Print(r, ",")
			fmt.Print("(")
			fmt.Print(x-minx+border, ",")
			fmt.Print(y-miny+border, ",")
			fmt.Print(z)
			fmt.Print("))")
			if i != (len(track.Terms) - 1) {
				fmt.Print(",")
			}
		}
		fmt.Println("]]")
	}
}
