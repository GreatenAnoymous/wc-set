#!/bin/bash

# Set the variables.
m=30
k=0
d=1

# Loop over the values of m.
while [ $m -le 300 ]; do

  # Loop over the values of k.
  for ((i=0; i<10; i++)); do

    # Run the executable with the specified arguments.
    ./main "$m" "$i" "$d"

  # Increment k.
 

  done

  # Increment m.
  m=$(($m+30))

done
