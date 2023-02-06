# HW5: Biped Stair Climber

My project is a Biped Stair Climber. It has a torso (moreso a hip), 2 thighs (left and right), 2 calves (left and right), and 2 feet (left and right). 

It consists of motors for the torsos, thighs, and feet, which move based on sensor neurons. The sensor neurons are on each link. 

The motors on the thighs, calves, and feet have different angles of movement and more force. This is intentional to somewhat mimic the behavior of humanoid bipeds, where feet have less degree of movement, and thighs and calves are stronger and have more angle of movement than feet. 

The objective is to reach the top of the 6-step flight of stairs. Originally, the fitness function simply consisted of maximising the greatest Z coordinate the robot encounters throughout the simulation. However, this result in either standing as straight as possible or jumping with no objective. Also, the stairs increase in height toward the +Y direction. Thus, the fitness function consists of two parts: the Y coordinate and the Z coordinate. 

At first, the fitness function just checked the maximum Y value and the maximum Z coordinate separately. If the child's Y and Z coordinate are greater than the respective Y and Z of the parent, then the child is more optimal. However, this caused the hill climber to discard some correct trials because, with high chance, the robot would reach the top of the stairs upside-down. Because the coordinates are chosen based on the head (and it wouldn't work if the feet were chosen, as it chose to flip upside-down with its feet extended straight upwards), a different fitness function had to be used.

The final fitness function to create the successful stair climber is a weighted fitness function between Y and Z values. Specifically, the fitness function was a singal value calculated from 0.35*(z-coordinate) + 0.65*(y-coordinate). The z-coordinate is weighed less because of a similar issue to the aforementioned issue where many more successful trials were discarded because the head may be higher on a lower step (if it were upright) compared to an up-side down robot on the top step. 

The video can be seen here: https://youtu.be/f2c54MJUxMY

# Running the Program

The main file is search.py. I exclusively used python3.7 for this project.

You can run 1 generation of this biped stair climber (generation 0, completely random) with:

``` 

python3.7 search.py 

````

To run the hillclimber, the value of ``` numberOfGenerations ``` and ``` populationSize ``` in constants.py can be changed. 

Once the hillclimber is completed, you can re-run the trial with the best fitness using:

``` 

python3.7 simulate.py GUI <successfulTrialIDNumber>

```