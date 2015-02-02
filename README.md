Go-PCB
======

Go PCB router and solver.

Example command line would be:

go run pcb.go --v 1 netlist.pcb | go run view.go

You can drop the output to a file and view it as an animation with:

go run pcb.go --v 1 netlist.pcb 1 > anim
go run view.go anim

-h or --help for help on either app.

Format of .pcb input file or stdin is:

[width, height, depth]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]

You can stop a netlist early by just putting:

[]

For example:

[width, height, depth]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
[]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...]

Format of the view.py input is similar but has the track paths appended and the gaps removed:

[width, height, depth]
[track_radius, via_radius, track_gap, [(terminal_radius, terminal_gap, (x, y, z), [(x, y), ...]), ...]...], [(x, y, z), ...]]
