convertSTL
==========

A utility for converting STL files between ASCII and binary encoding.
I wrote this to save disk space and bandwidth when handling STL files. Some modeling programs include this functionality, but it is nice to have something that is more lightweight and can be called from a command line.

USAGE 
---------
    $ ruby convertSTL.rb [filename of .stl to be converted]
or 'chmod +x' the script and run as:
    
    $ ./convertSTL.rb [filename of .stl to be converted]
The script will then translate the STL to the opposite encoding and save it as either `-ascii.stl` or `-binary.stl`

AUTHOR
-----------
Created by Chris Polis([@ChrisPolis](http://twitter.com/chrispolis)) under the MIT License
