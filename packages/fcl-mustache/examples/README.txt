demo1 sample program:

Demonstrates the most basic use of the mustache parser

demo2 sample program:

Demonstrates the use of the mustache parser with a CSV dataset


mustache example program:

Can be used to load a template and data, and process the result.
Output to standard output or file.
The template and JSON value can be loaded from file (using @filename), 
or their value can be specified directly on the command-line.

Example usage:

Load template from family.tmpl file, data from family.json file:

./mustache -d title="my family" -t @family.tmpl -j @family.json

Load template from family.tmpl file, data from family.csv file:

./mustache -d title="my family" -t @family.tmpl -c family.csv

Use of expressions can be enabled with the -e switch.
