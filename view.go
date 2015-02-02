package main

import (
	"./mymath"
	"./router"
	"bufio"
	"flag"
	"fmt"
	"io/ioutil"
	"os"
	"runtime"
	"strconv"
	"strings"

	"github.com/go-gl/gl"
	glfw "github.com/go-gl/glfw3"
	"github.com/go-gl/glh"
)

const (
	margin = 2
)

//read input till given byte appears
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

//read, [width, height, depth]
func read_dimentions(r *bufio.Reader) *router.Dims {
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
	return &router.Dims{int(width), int(height), int(depth)}
}

//read, (x, y)
func read_cord(r *bufio.Reader) *router.Cord {
	eof := read_until(r, '(')
	if eof {
		os.Exit(1)
	}
	trim := "()[], "
	string, _ := r.ReadString(',')
	x, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(')')
	y, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	return &router.Cord{float32(x), float32(y)}
}

//read, [(x, y), ...]
func read_shape(r *bufio.Reader) *router.Cords {
	eof := read_until(r, '[')
	if eof {
		os.Exit(1)
	}
	cords := router.Cords{}
	for {
		bytes, _ := r.Peek(1)
		if bytes[0] == ']' {
			break
		}
		cords = append(cords, read_cord(r))
	}
	eof = read_until(r, ']')
	if eof {
		os.Exit(1)
	}
	return &cords
}

//read, (x, y, z)
func read_tpoint(r *bufio.Reader) *router.Tpoint {
	eof := read_until(r, '(')
	if eof {
		os.Exit(1)
	}
	trim := "()[], "
	string, _ := r.ReadString(',')
	x, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(',')
	y, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(')')
	z, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	return &router.Tpoint{float32(x), float32(y), float32(z)}
}

//read, [(x, y, z), ...]
func read_path(r *bufio.Reader) *router.Path {
	eof := read_until(r, '[')
	if eof {
		os.Exit(1)
	}
	path := router.Path{}
	for {
		bytes, _ := r.Peek(1)
		if bytes[0] == ']' {
			break
		}
		path = append(path, read_tpoint(r))
	}
	eof = read_until(r, ']')
	if eof {
		os.Exit(1)
	}
	return &path
}

//read, [[(x, y, z), ...], ...]
func read_paths(r *bufio.Reader) *router.Paths {
	eof := read_until(r, '[')
	if eof {
		os.Exit(1)
	}
	paths := router.Paths{}
	for {
		bytes, _ := r.Peek(1)
		if bytes[0] == ']' {
			break
		}
		paths = append(paths, *read_path(r))
	}
	eof = read_until(r, ']')
	if eof {
		os.Exit(1)
	}
	return &paths
}

//read, (radius, gap, (x, y, z), [(x, y), ...])
func read_terminal(r *bufio.Reader) *router.Terminal {
	eof := read_until(r, '(')
	if eof {
		os.Exit(1)
	}
	trim := "()[], "
	string, _ := r.ReadString(',')
	radius, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(',')
	gap, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(',')
	x, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(',')
	y, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(',')
	z, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	shape := read_shape(r)
	eof = read_until(r, ')')
	if eof {
		os.Exit(1)
	}
	return &router.Terminal{float32(radius), float32(gap), router.Tpoint{float32(x), float32(y), float32(z)}, *shape}
}

//read all terminals for one track
func read_terminals(r *bufio.Reader) *router.Terminals {
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
	return &terminals
}

//read one track
func read_track(r *bufio.Reader) (*router.Output, bool) {
	eof := read_until(r, '[')
	if eof {
		return &router.Output{}, true
	}
	bytes, _ := r.Peek(1)
	if bytes[0] == ']' {
		return &router.Output{}, true
	}
	trim := "()[], "
	string, _ := r.ReadString(',')
	radius, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(',')
	via, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	string, _ = r.ReadString(',')
	gap, _ := strconv.ParseFloat(strings.Trim(string, trim), 32)
	terminals := read_terminals(r)
	paths := read_paths(r)
	eof = read_until(r, ']')
	if eof {
		os.Exit(1)
	}
	return &router.Output{float32(radius), float32(via), float32(gap), *terminals, *paths}, false
}

func make_program(vert_file_name, frag_file_name string) gl.Program {
	vert_source, err := ioutil.ReadFile(vert_file_name)
	if err != nil {
		panic(err)
	}
	frag_source, err := ioutil.ReadFile(frag_file_name)
	if err != nil {
		panic(err)
	}
	vs := glh.Shader{gl.VERTEX_SHADER, string(vert_source)}
	fs := glh.Shader{gl.FRAGMENT_SHADER, string(frag_source)}
	return glh.NewProgram(vs, fs)
}

func draw_polygon(data mymath.Points) {
	vertex_buffer_data := make([]float32, len(data)*2, len(data)*2)
	for i := 0; i < len(data); i++ {
		pp := data[i]
		p := *pp
		vertex_buffer_data[i*2] = p[0]
		vertex_buffer_data[i*2+1] = p[1]
	}
	gl.BufferData(gl.ARRAY_BUFFER, len(vertex_buffer_data)*4, vertex_buffer_data, gl.STATIC_DRAW)
	gl.DrawArrays(gl.LINE_STRIP, 0, len(vertex_buffer_data)/2)
}

func draw_filled_polygon(data mymath.Points) {
	vertex_buffer_data := make([]float32, len(data)*2, len(data)*2)
	for i := 0; i < len(data); i++ {
		pp := data[i]
		p := *pp
		vertex_buffer_data[i*2] = p[0]
		vertex_buffer_data[i*2+1] = p[1]
	}
	gl.BufferData(gl.ARRAY_BUFFER, len(vertex_buffer_data)*4, vertex_buffer_data, gl.STATIC_DRAW)
	gl.DrawArrays(gl.TRIANGLE_STRIP, 0, len(vertex_buffer_data)/2)
}

func main() {
	runtime.LockOSThread()

	//command line flags and defaults etc
	arg_infile := os.Stdin
	var arg_s int
	var arg_o int
	flag.IntVar(&arg_s, "s", 9, "scale factor, default 9")
	flag.IntVar(&arg_o, "o", 0, "overlay modes 0..1, default 0")
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

	dimensions := read_dimentions(reader)
	pcb_width := dimensions.Width
	pcb_height := dimensions.Height
	pcb_depth := dimensions.Depth
	width := (pcb_width + (margin * 2)) * arg_s
	height := (pcb_height + (margin * 2)) * arg_s
	if arg_o == 1 {
		height *= pcb_depth
	}

	if !glfw.Init() {
		fmt.Fprintf(os.Stderr, "Can't open GLFW")
		return
	}
	glfw.WindowHint(glfw.Samples, 4)
	glfw.WindowHint(glfw.ContextVersionMajor, 3)
	glfw.WindowHint(glfw.ContextVersionMinor, 3)
	glfw.WindowHint(glfw.OpenglProfile, glfw.OpenglCoreProfile)
	glfw.WindowHint(glfw.OpenglForwardCompatible, glfw.True) // needed for macs

	window, err := glfw.CreateWindow(width, height, "PCB Viewer", nil, nil)
	if err != nil {
		fmt.Fprintf(os.Stderr, "%v\n", err)
		return
	}
	window.MakeContextCurrent()

	gl.Init()
	gl.GetError()
	window.SetInputMode(glfw.StickyKeys, 1)

	gl.ClearColor(0.0, 0.0, 0.0, 0.0)

	gl.Enable(gl.BLEND)
	gl.DepthFunc(gl.LESS)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
	gl.LineWidth(1.0)

	vertex_array := gl.GenVertexArray()
	vertex_array.Bind()

	prog := make_program("VertexShader.vert", "FragmentShader.frag")
	vert_color_id := prog.GetUniformLocation("vert_color")
	vert_aspect_id := prog.GetUniformLocation("vert_aspect")
	vert_offset_id := prog.GetUniformLocation("vert_offset")
	prog.Use()

	vert_aspect_id.Uniform2f(2.0/float32(width), -2.0/float32(height))
	vert_offset_id.Uniform2f(-1.0, 1.0)

	vertex_buffer := gl.GenBuffer()
	vertex_attrib := gl.AttribLocation(0)
	vertex_buffer.Bind(gl.ARRAY_BUFFER)
	vertex_attrib.EnableArray()
	vertex_attrib.AttribPointer(2, gl.FLOAT, false, 0, nil)

	for {
		if (window.GetKey(glfw.KeyEscape) == glfw.Press) || window.ShouldClose() {
			break
		}
		gl.Clear(gl.COLOR_BUFFER_BIT)

		tracks := []*router.Output{}
		for {
			track, eof := read_track(reader)
			if eof == true {
				break
			}
			tracks = append(tracks, track)
		}
		if len(tracks) == 0 {
			break
		}

		scale := float32(arg_s)
		border := float32(margin * arg_s)
		for _, track := range tracks {
			track.Radius *= scale
			track.Via *= scale
			track.Gap *= scale
			for _, term := range track.Terms {
				term.Radius *= scale
				term.Gap *= scale
				term.Term.X *= scale
				term.Term.Y *= scale
				term.Term.X += border
				term.Term.Y += border
				for _, cord := range term.Shape {
					cord.X *= scale
					cord.Y *= scale
					cord.X += term.Term.X
					cord.Y += term.Term.Y
				}
			}
			for _, path := range track.Paths {
				for _, node := range path {
					node.X *= scale
					node.Y *= scale
					node.X += border
					node.Y += border
				}
			}
		}

		colors := [...]float32{
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0,
			1.0, 1.0, 0.0,
			0.0, 1.0, 1.0,
			1.0, 0.0, 1.0,
		}
		for depth := pcb_depth - 1; depth > -1; depth-- {
			color := (depth % (len(colors) / 3)) * 3
			vert_color_id.Uniform4f(colors[color], colors[color+1], colors[color+2], 0.5)
			for _, track := range tracks {
				for _, path := range track.Paths {
					start := 0
					end := 0
					for end = 0; end < (len(path) - 1); end++ {
						if path[start].Z != path[end].Z {
							if path[start].Z == float32(depth) {
								points := make([]*mymath.Point, len(path[start:end]), len(path[start:end]))
								for i, cord := range path[start:end] {
									points[i] = &mymath.Point{cord.X, cord.Y}
								}
								draw_filled_polygon(mymath.Thicken_path_triangles_2d(points, track.Radius, 3, 2, 16))
							}
							start = end
						}
					}
					if len(path)-start > 1 {
						if path[start].Z == float32(depth) {
							points := make([]*mymath.Point, len(path[start:]), len(path[start:]))
							for i, cord := range path[start:] {
								points[i] = &mymath.Point{cord.X, cord.Y}
							}
							draw_filled_polygon(mymath.Thicken_path_triangles_2d(points, track.Radius, 3, 2, 16))
						}
					}
				}
			}
		}
		vert_color_id.Uniform4f(1.0, 1.0, 1.0, 1.0)
		for _, track := range tracks {
			for _, path := range track.Paths {
				for i := 0; i < (len(path) - 1); i++ {
					if path[i].Z != path[i+1].Z {
						draw_filled_polygon(mymath.Circle_triangles_2d(&mymath.Point{path[i].X, path[i].Y}, track.Via, 32))
					}
				}
			}
			for _, term := range track.Terms {
				if len(term.Shape) == 0 {
					draw_filled_polygon(mymath.Circle_triangles_2d(&mymath.Point{term.Term.X, term.Term.Y}, term.Radius, 32))
				} else {
					points := make([]*mymath.Point, len(term.Shape), len(term.Shape))
					for i, cord := range term.Shape {
						points[i] = &mymath.Point{cord.X, cord.Y}
					}
					if term.Radius != 0 {
						draw_filled_polygon(mymath.Thicken_path_triangles_2d(points, term.Radius, 3, 2, 16))
					} else {
						draw_filled_polygon(points)
					}
				}
			}
		}

		window.SwapBuffers()
		glfw.PollEvents()
	}

	for {
		if (window.GetKey(glfw.KeyEscape) == glfw.Press) || window.ShouldClose() {
			break
		}
		glfw.PollEvents()
	}

	vertex_buffer.Delete()
	prog.Delete()
	vertex_array.Delete()
	glfw.Terminate()
}
