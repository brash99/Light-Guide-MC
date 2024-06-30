#!/bin/sh

for i in {0..11}; do
    CPU='G4WT'$i' '
    echo $CPU
    grep "$CPU" $1 | grep 'at Boundary' | wc -l
    grep "$CPU" $1 | grep Photons 
done

