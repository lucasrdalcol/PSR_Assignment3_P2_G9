
# Final Project PSR 
## General Overview
In the last project, we used all the skills acquired from the previous parts of the course *Programming Robot Systems*. The main goal of this work is to develop a simulation of the game called *"Team Hunt"* using the available tools in the *Robot Operation System* [ROS] environment.
### Team Hunt Rules
Team Hunt is a simulation of a game with three teams in which each team consists of three players. The game is about hunting a player from the opposing team, although the other teams must also be careful, as other teams do not remain passive! 
#### Rules of the game: 
A player who has been hunted by the hunting team in the form of contact / collision contributes to the score for the hunting team's account. Each game lasts 80 seconds.
#### Referee: 
The character of the referee was also implemented in the game. When a player has been hunted, he goes to a special zone for 6 seconds, then after serving a penalty, the hunted robot (player) returns to the arena to a random place.
The team with the most points wins.

## Program functionalities Overview

### Gameplay arena & simulation video
In our project, three arenas have been implemented in which the game is possible.
<br /> The videos uploaded to the YouTube platform shows a simulation of the game in the three arenas. Nevertheless, it is advisable to see a practical simulation of the gameplay recorded in each arena (see links below):
- **th_arena_1.world** [[Watch the video. Click here!]](https://youtu.be/dVea0v-LO34)
- **th_arena_2.world** [[Watch the video. Click here!]](https://youtu.be/9nZphKANAS4)
- **th_arena_3.world** [[Watch the video. Click here!]](https://youtu.be/d_NqNGz0aFM)
  
### Navigation on arenas

In the project it is possible to navigate the robot in three arenas mentioned above, and in order to illustrate the process of navigation, videos have been recorded to show how the robot moves according to the direction indicated:
- Navigation **th_arena_1.world** [[Watch the video. Click here!]](https://youtu.be/4-fUDnn4Wi0)
- Navigation **th_arena_2.world** [[Watch the video. Click here!]](https://youtu.be/C__kIk-h4oo)
- Navigation **th_arena_3.world** [[Watch the video. Click here!]](https://youtu.be/HP5tDa2EemE)
## Usage of program

### Running the program [instructions]
Open a new terminal and enter the following command to run empty environment:
```
roslaunch p_playerName_bringup gazebo.launch
```
In new terminal type following command to add a specific single player:
```
roslaunch p_playerName_bringup bringup.launch player_name:=red1 player_color:=Red
```
To automatically add multiple players, enter the following command in the terminal:
```
roslaunch p_playerName_bringup game_bringup.launch 
```
In order to see the arena on which you have added players type following command in terminal:
```
roslaunch p_playerName_bringup bringup.launch visualize:=true driver:=true 
```
To start the game, enter the following command in the terminal:
```
rosrun th_referee th_referee
```

### Driver Intelligences
We developed two intelligences to drive the robots: one of them for a robot with one camera and the other with two cameras.



## Contributors 

- Lucas Dal'Col [@lucasrdalcol](https://github.com/lucasrdalcol)
- Vinicius Campos de Oliveira Batista [@ufesvinicius](https://github.com/ufesvinicius)
- Emanuel Krzysztoń [@emanuelkrzyszton](https://github.com/emanuelkrzyszton)
- Diogo Vieira [@Kazadhum](https://github.com/Kazadhum)

