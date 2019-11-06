# Assignment 1.1 
Course 0368401002, Robotics & Motion Planning, TAU.

## Algorithm
The idea is to model the problem using a graph.
For every legal (i.e. "obstacle-less") x, y, z coordinates,
where 0 <= x, y, z <= 10, we propose a vertex 
corresponding to it. 

We then add edges from every vertex to every other vertex
whose coordinates differ by one, for example:
```
(2, 1, 3) -> (2, 1, 4)
(4, 5, 8) -> (3, 5, 8)
```

After that, we perform BFS from source to destination, and return the path found.

## How to run this
From the command line, run as follows:
```
python oskar.py --sx [src x] --sy [src y] 
--sz [src z]  --dx [dst x] --dy [dst y] 
--dz [dst z] -- f [file path]
```
And simply replace contents written inside the brackets with desired input.
For example:
```
python oskar.py --sx 1 --sy 7 --sz 9 --dx 5 
--dy 1 --dz 1 --f util/oskar_input.txt
```
And the output we get:
```
5 5 5 5 0 0 2 2 1 1 4 4 4 4 0 
0 5 5 0 0 5 5 3 3 4 4 3 3 5 5 
5 5 2 2 2 2 1 1 3 3 5 5 1 1 3 
3 0 0 3 3 4 4 0 0 4 4 4 4 1 1 
3 3 4 4 0 0
```
Which corresponds to the formatted path.

## Assumptions
* Source and destination vertex are legal
* There exists a path from source to destination

## Author
Adar Gutman, 316265065, TAU.

## References
http://acg.cs.tau.ac.il/courses/algorithmic-robotics/fall-2019-2020/assignments/assignment-1