Go-PCB
======

Go PCB router and solver.

There are two viewer apps, view.py requires aggDraw module to be installed, view_mpl.py
requires matplotlib module to be installed. The matplotlib viwer is much more heavyweight
than the former. Both are available with the Python version of this app.

Example command line would be:

go run pcb.go --v 1 netlist.pcb | python view.py

You can drop the output to a file and view it as an animation with:

go run pcb.go --v 1 netlist.pcb 1 > anim
python view.py anim

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
