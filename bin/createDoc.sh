#!/bin/bash

cd doc
doxygen Doxyfile 

pandoc --write=gfm -i html/index.html -o /tmp/rs.md 
tail --lines=+40 /tmp/rs.md  > ../README.md
rm /tmp/rs.md

