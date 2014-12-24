Go-PCB
======

Go PCB router and solver

Example command line would be:

go run pcb.go --v 1 netlist.pcb | python view.py

You can drop the output to a file and view it as an animation with:

go run pcb.go --v 1 netlist.pcb 1 > anim
python view.py anim

-h or --help for help on either app.

Format of .pcb input file or stdin is:

[width, height, depth]
[track_radius, [(terminal_radius, (x, y, z)), ...]...]

You can stop a netlist early by just putting:

[]

For example:

[width, height, depth]
[track_radius, [(terminal_radius, (x, y, z)), ...]...]
[track_radius, [(terminal_radius, (x, y, z)), ...]...]
[]
[track_radius, [(terminal_radius, (x, y, z)), ...]...]
[track_radius, [(terminal_radius, (x, y, z)), ...]...]

Format of the view.py input is similar but has the track paths appended:

[width, height, depth]
[track_radius, [(terminal_radius, (x, y, z)), ...]...], [(x, y, z), ...]]
