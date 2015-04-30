package main

import (
	"errors"
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

	"github.com/go-gl/gl/v4.1-core/gl"
	glfw "github.com/go-gl/glfw/v3.0/glfw"
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

//load shader progs
func make_program(vert_file_name, frag_file_name string) uint32 {
	vert_source, err := ioutil.ReadFile(vert_file_name)
	if err != nil {
		panic(err)
	}
	frag_source, err := ioutil.ReadFile(frag_file_name)
	if err != nil {
		panic(err)
	}
	program, err := newProgram(string(vert_source) + "\x00", string(frag_source) + "\x00")
	return program
}

func newProgram(vertexShaderSource, fragmentShaderSource string) (uint32, error) {
	vertexShader, err := compileShader(vertexShaderSource, gl.VERTEX_SHADER)
	if err != nil {
		return 0, err
	}

	fragmentShader, err := compileShader(fragmentShaderSource, gl.FRAGMENT_SHADER)
	if err != nil {
		return 0, err
	}

	program := gl.CreateProgram()

	gl.AttachShader(program, vertexShader)
	gl.AttachShader(program, fragmentShader)
	gl.LinkProgram(program)

	var status int32
	gl.GetProgramiv(program, gl.LINK_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetProgramiv(program, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetProgramInfoLog(program, logLength, nil, gl.Str(log))

		return 0, errors.New(fmt.Sprintf("failed to link program: %v", log))
	}

	gl.DeleteShader(vertexShader)
	gl.DeleteShader(fragmentShader)

	return program, nil
}

func compileShader(source string, shaderType uint32) (uint32, error) {
	shader := gl.CreateShader(shaderType)

	csource := gl.Str(source)
	gl.ShaderSource(shader, 1, &csource, nil)
	gl.CompileShader(shader)

	var status int32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLength)

		log := strings.Repeat("\x00", int(logLength+1))
		gl.GetShaderInfoLog(shader, logLength, nil, gl.Str(log))

		return 0, fmt.Errorf("failed to compile %v: %v", source, log)
	}

	return shader, nil
}

//draw a line strip polygon
func draw_polygon(offsetp *mymath.Point, datap *mymath.Points) {
	data := *datap
	offset := *offsetp
	vertex_buffer_data := make([]float32, len(data)*2, len(data)*2)
	for i := 0; i < len(data); i++ {
		pp := data[i]
		p := *pp
		vertex_buffer_data[i*2] = p[0] + offset[0]
		vertex_buffer_data[i*2+1] = p[1] + offset[1]
	}
	gl.BufferData(gl.ARRAY_BUFFER, len(vertex_buffer_data)*4, gl.Ptr(vertex_buffer_data), gl.STATIC_DRAW)
	gl.DrawArrays(gl.LINE_STRIP, 0, int32(len(vertex_buffer_data)/2))
}

//draw a triangle strip polygon
func draw_filled_polygon(offsetp *mymath.Point, datap *mymath.Points) {
	data := *datap
	offset := *offsetp
	vertex_buffer_data := make([]float32, len(data)*2, len(data)*2)
	for i := 0; i < len(data); i++ {
		pp := data[i]
		p := *pp
		vertex_buffer_data[i*2] = p[0] + offset[0]
		vertex_buffer_data[i*2+1] = p[1] + offset[1]
	}
	gl.BufferData(gl.ARRAY_BUFFER, len(vertex_buffer_data)*4, gl.Ptr(vertex_buffer_data), gl.STATIC_DRAW)
	gl.DrawArrays(gl.TRIANGLE_STRIP, 0, int32(len(vertex_buffer_data)/2))
}

//create circle polygon
var circle_map map[float32]*mymath.Points

func create_filled_circle(radius float32) *mymath.Points {
	circle_points := circle_map[radius]
	if circle_points != nil {
		return circle_points
	}
	circle_points = mymath.Circle_as_tristrip(&mymath.Point{0.0, 0.0}, radius, 0, 32)
	circle_map[radius] = circle_points
	return circle_points
}

func main() {
	runtime.LockOSThread()

	//setup globals
	circle_map = map[float32]*mymath.Points{}

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

	//read dimensions of the pcb
	dimensions := read_dimentions(reader)
	pcb_width := dimensions.Width
	pcb_height := dimensions.Height
	pcb_depth := dimensions.Depth
	width := (pcb_width + (margin * 2)) * arg_s
	height := (pcb_height + (margin * 2)) * arg_s
	if arg_o == 1 {
		height *= pcb_depth
	}

	//create window
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
	window.SetInputMode(glfw.StickyKeys, 1)

	//set gl settings
	gl.Init()
	gl.GetError()
	gl.ClearColor(0.0, 0.0, 0.0, 0.0)
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
	gl.LineWidth(1.0)

	//create vertex array
	var vertex_array uint32
	gl.GenVertexArrays(1, &vertex_array)
	gl.BindVertexArray(vertex_array)

	//load shaders and get address of shader variables
	prog := make_program("VertexShader.vert", "FragmentShader.frag")
	vert_color_id := gl.GetUniformLocation(prog, gl.Str("vert_color\x00"))
	vert_scale_id := gl.GetUniformLocation(prog, gl.Str("vert_scale\x00"))
	vert_offset_id := gl.GetUniformLocation(prog, gl.Str("vert_offset\x00"))

	//use the loaded shader program
	gl.UseProgram(prog)

	//set aspect and offset for 2D drawing
	gl.Uniform2f(vert_scale_id, 2.0/float32(width), -2.0/float32(height))
	gl.Uniform2f(vert_offset_id, -1.0, 1.0)

	//setup vertex buffer ready for use
	var vertex_buffer uint32
	gl.GenBuffers(1, &vertex_buffer)
	vertex_attrib := uint32(gl.GetAttribLocation(prog, gl.Str("vert_vertex\x00")))
	gl.BindBuffer(gl.ARRAY_BUFFER, vertex_buffer)
	gl.EnableVertexAttribArray(vertex_attrib)
	gl.VertexAttribPointer(vertex_attrib, 2, gl.FLOAT, false, 0, gl.PtrOffset(0))

	for {
		//exit of ESC key or close button pressed
		glfw.PollEvents()
		if (window.GetKey(glfw.KeyEscape) == glfw.Press) || window.ShouldClose() {
			break
		}

		//load track and exit if no track loaded
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

		//scale track acording to window size
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

		//clear background
		gl.Clear(gl.COLOR_BUFFER_BIT)

		if arg_o == 0 {
			//draw paths for each layer
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
				gl.Uniform4f(vert_color_id, colors[color], colors[color+1], colors[color+2], 0.5)
				for _, track := range tracks {
					for _, path := range track.Paths {
						start := 0
						end := 0
						for end = 0; end < len(path); end++ {
							if path[start].Z != path[end].Z {
								if path[start].Z == float32(depth) {
									if end-start > 1 {
										points := make(mymath.Points, len(path[start:end]), len(path[start:end]))
										for i, cord := range path[start:end] {
											points[i] = &mymath.Point{cord.X, cord.Y}
										}
										draw_filled_polygon(&mymath.Point{0.0, 0.0},
											mymath.Thicken_path_as_tristrip(&points, track.Radius, 3, 2, 16))
									}
								}
								start = end
							}
						}
						if path[start].Z == float32(depth) {
							if end-start > 1 {
								points := make(mymath.Points, len(path[start:end]), len(path[start:end]))
								for i, cord := range path[start:end] {
									points[i] = &mymath.Point{cord.X, cord.Y}
								}
								draw_filled_polygon(&mymath.Point{0.0, 0.0},
									mymath.Thicken_path_as_tristrip(&points, track.Radius, 3, 2, 16))
							}
						}
					}
				}
			}
			//draw terminals and vias
			gl.Uniform4f(vert_color_id, 1.0, 1.0, 1.0, 1.0)
			for _, track := range tracks {
				for _, path := range track.Paths {
					for i := 0; i < (len(path) - 1); i++ {
						if path[i].Z != path[i+1].Z {
							draw_filled_polygon(&mymath.Point{path[i].X, path[i].Y},
								create_filled_circle(track.Via))
						}
					}
				}
				for _, term := range track.Terms {
					if len(term.Shape) == 0 {
						draw_filled_polygon(&mymath.Point{term.Term.X, term.Term.Y},
							create_filled_circle(term.Radius))
					} else {
						points := make(mymath.Points, len(term.Shape), len(term.Shape))
						for i, cord := range term.Shape {
							points[i] = &mymath.Point{cord.X, cord.Y}
						}
						if term.Radius != 0 {
							draw_filled_polygon(&mymath.Point{0.0, 0.0},
								mymath.Thicken_path_as_tristrip(&points, term.Radius, 3, 2, 16))
						} else {
							draw_filled_polygon(&mymath.Point{0.0, 0.0}, &points)
						}
					}
				}
			}
		} else {
			//draw paths for each layer in white
			gl.Uniform4f(vert_color_id, 1.0, 1.0, 1.0, 1.0)
			for depth := pcb_depth - 1; depth > -1; depth-- {
				yoffset := float32((pcb_height + (margin * 2)) * arg_s * depth)
				for _, track := range tracks {
					for _, path := range track.Paths {
						start := 0
						end := 0
						for end = 0; end < len(path); end++ {
							if path[start].Z != path[end].Z {
								if path[start].Z == float32(depth) {
									if end-start > 1 {
										points := make(mymath.Points, len(path[start:end]), len(path[start:end]))
										for i, cord := range path[start:end] {
											points[i] = &mymath.Point{cord.X, cord.Y}
										}
										draw_filled_polygon(&mymath.Point{0.0, yoffset},
											mymath.Thicken_path_as_tristrip(&points, track.Radius+track.Gap, 3, 2, 16))
									}
								}
								start = end
							}
						}
						if path[start].Z == float32(depth) {
							if end-start > 1 {
								points := make(mymath.Points, len(path[start:end]), len(path[start:end]))
								for i, cord := range path[start:end] {
									points[i] = &mymath.Point{cord.X, cord.Y}
								}
								draw_filled_polygon(&mymath.Point{0.0, yoffset},
									mymath.Thicken_path_as_tristrip(&points, track.Radius+track.Gap, 3, 2, 16))
							}
						}
					}
				}
				//draw terminals and vias in white
				for _, track := range tracks {
					for _, path := range track.Paths {
						for i := 0; i < (len(path) - 1); i++ {
							if path[i].Z != path[i+1].Z {
								draw_filled_polygon(&mymath.Point{path[i].X, path[i].Y + yoffset},
									create_filled_circle(track.Via+track.Gap))
							}
						}
					}
					for _, term := range track.Terms {
						if len(term.Shape) == 0 {
							draw_filled_polygon(&mymath.Point{term.Term.X, term.Term.Y + yoffset},
								create_filled_circle(term.Radius+term.Gap))
						} else {
							points := make(mymath.Points, len(term.Shape), len(term.Shape))
							for i, cord := range term.Shape {
								points[i] = &mymath.Point{cord.X, cord.Y}
							}
							draw_filled_polygon(&mymath.Point{0.0, yoffset},
								mymath.Thicken_path_as_tristrip(&points, term.Radius+term.Gap, 3, 2, 16))
						}
					}
				}
			}
			//draw paths for each layer in black
			gl.Uniform4f(vert_color_id, 0.0, 0.0, 0.0, 1.0)
			for depth := pcb_depth - 1; depth > -1; depth-- {
				yoffset := float32((pcb_height + (margin * 2)) * arg_s * depth)
				for _, track := range tracks {
					for _, path := range track.Paths {
						start := 0
						end := 0
						for end = 0; end < len(path); end++ {
							if path[start].Z != path[end].Z {
								if path[start].Z == float32(depth) {
									if end-start > 1 {
										points := make(mymath.Points, len(path[start:end]), len(path[start:end]))
										for i, cord := range path[start:end] {
											points[i] = &mymath.Point{cord.X, cord.Y}
										}
										draw_filled_polygon(&mymath.Point{0.0, yoffset},
											mymath.Thicken_path_as_tristrip(&points, track.Radius, 3, 2, 16))
									}
								}
								start = end
							}
						}
						if path[start].Z == float32(depth) {
							if end-start > 1 {
								points := make(mymath.Points, len(path[start:end]), len(path[start:end]))
								for i, cord := range path[start:end] {
									points[i] = &mymath.Point{cord.X, cord.Y}
								}
								draw_filled_polygon(&mymath.Point{0.0, yoffset},
									mymath.Thicken_path_as_tristrip(&points, track.Radius, 3, 2, 16))
							}
						}
					}
				}
				//draw terminals and vias in black
				for _, track := range tracks {
					for _, path := range track.Paths {
						for i := 0; i < (len(path) - 1); i++ {
							if path[i].Z != path[i+1].Z {
								draw_filled_polygon(&mymath.Point{path[i].X, path[i].Y + yoffset},
									create_filled_circle(track.Via))
							}
						}
					}
					for _, term := range track.Terms {
						if len(term.Shape) == 0 {
							draw_filled_polygon(&mymath.Point{term.Term.X, term.Term.Y + yoffset},
								create_filled_circle(term.Radius))
						} else {
							points := make(mymath.Points, len(term.Shape), len(term.Shape))
							for i, cord := range term.Shape {
								points[i] = &mymath.Point{cord.X, cord.Y}
							}
							if term.Radius != 0 {
								draw_filled_polygon(&mymath.Point{0.0, yoffset},
									mymath.Thicken_path_as_tristrip(&points, term.Radius, 3, 2, 16))
							} else {
								draw_filled_polygon(&mymath.Point{0.0, yoffset}, &points)
							}
						}
					}
				}
			}
		}

		//show window just drawn
		window.SwapBuffers()
	}

	//wait till exit or close button pressed, 'hold on last frame'
	for {
		glfw.PollEvents()
		if (window.GetKey(glfw.KeyEscape) == glfw.Press) || window.ShouldClose() {
			break
		}
	}

	//clean up
	gl.DeleteBuffers(1, &vertex_buffer)
	gl.DeleteVertexArrays(1, &vertex_array)
	gl.DeleteProgram(prog)
	glfw.Terminate()
}
