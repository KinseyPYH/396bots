# CS396 HW6: Generate Random 1D Creature Morphologies

This assignment create a program that generates a kinematic chain (a jointed, motorized, innervated, sensorized snake) with a random number of randomly shaped links with random sensor placement along the chain.

Links with and without sensors are respectively colored green and blue.

The video of 10 trials can be seen here: https://youtu.be/QkMwgxp1WTs

# Running the Program

The main file is search.py. I exclusively used python3.7 for this project.

You can run 1 generation of this kniematic chain (snake) (generation 0, completely random) with:

``` 

python3.7 search.py 

```

To run this multiple times as in the YouTube video linked above, I ran search.py n times with a bash script: 

```
for ((i=0; i < 10; i++)); do

  python3.7 search.py

done

```

# Citations

Bongard, Josh. “Education in Evolutionary Robotics.” Reddit, https://www.reddit.com/r/ludobots/.

Kriegman, Sam. CS396: Artifical Life, 2023, Northwestern University.