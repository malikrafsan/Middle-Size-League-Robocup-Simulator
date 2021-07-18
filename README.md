# Workspace-Team-2

## Description

> Dokumentasi dari pengerjaan Tugas Besar Magang 2 Dagozilla tim 2 divisi Programming. Berisi Langkah-langkah development serta penjelasannya. Berisi pula cara menjalankan program tugas besar ini. Untuk spesifikasi tugas besar ini dapat dilihat [di sini](https://docs.google.com/document/d/1I2hONJeM-VH9wPXQ0c4o6-VN-0rQxDKiGZR3LR6tkmg/edit).

## Author

1. Malik Akbar H. R.
2. Sarah Azka Arief
3. Rania Dwi Fadhilah
4. Vincent Prasetiya A.
5. M. Sulthan Mazaya

## Development steps

1. Initialize workspace

    ```sh
    cd TESTING_WORKSPACE
    mkdir src
    catkin_make
    ```

2. Create Package

    ```sh
    cd src
    catkin_create_pkg ssl_robocup_gazebo
    ```

3. Create directories and files

    ```
    cd ssl_robocup_gazebo
    mkdir worlds launch models plugins
    touch worlds/ssl_robocup.world launch/ssl_robocup.launch models/ssl_robocup.model plugins/ssl_robocup.plugin
    ```

4. Copy template codes to world file and launch file (copy from world database and roslaunch course)
5. Compile and source

    ```sh
    cd ../..                    # to our workspace
    catkin_make
    source devel/setup.bash
    ```

6. Modify models and integrate to world file and `package.xml`
    - Change size of models
    - Add new link to models
    - Change included models in world file from model database to modified model
    - Add execution dependencies and export to Gazebo model path
7. Try implement and integrate plugin 
    - Add plugin file used in gazebo tutorial [Model plugins](http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin) in the plugins directory
    - Modify `package.xml`, `CMakeLists.txt`, and world file to integrate the plugin
8. Implement `follow_object` plugin which implement:
    - Client of the position service at /gazebo/get_model_state
    - Calculate magnitude of distance and velocity of the robot
    - Calculate angle between robot and the ball then different orientation between them
    - Adjust orientation of the robot until the different angle is below threshold
    - If magnitude of distance is bigger than magnitude of velocity, set constant force to 3 Newton, using magnitude normalization
    - If magnitude of distance is smaller than magnitude of velocity, slow down using dropping linear velocity (deceleration)
    - Advertise service for `set_orient_and_chase` plugin
9. Implement `goal_out_detection` plugin to reset world if the ball is out of field or there is goal
    - Client of the model's position (including ball) at `/gazebo/get_model_state`
    - Get model's position that is named "ball"
    - Check whether the position is outside the side / end line, if yes, reset the world
10. Implemented service for attaching 2 model together:
    - Done at /link_attacher_node/attach and /link_attacher_node/attach
    - The args of this service is the following

    ```bash
    rosservice call /link_attacher_node/attach "model_name_1:'model_name' link_name_1:'link_name' model_name_2:'model_name' link_name_2:'link_name'"
    rosservice call /link_attacher_node/detach "model_name_1:'model_name' link_name_1:'link_name' model_name_2:'model_name' link_name_2:'link_name'"
    ```

    - Link for the turtlebot is named `rack` and for the ball its `ball`.
11. Implement `set_orient_and_chase` plugin which call set orient service at /kinetics/set_orient_n_chase.
12. Implement `move_to_coord` plugin to move robot to a spesific location
    - Client of the position service at /gazebo/get_model_state
    - Calculate magnitude of distance and velocity of the robot
    - Calculate angle between robot and the spesific location then different orientation between them
    - Adjust orientation of the robot until the different angle is below threshold
    - If magnitude of distance is bigger than magnitude of velocity, set constant force to 3 Newton, using magnitude normalization
    - If magnitude of distance is smaller than magnitude of velocity, slow down using dropping linear velocity (deceleration)
13. Implement `ball_controller` plugin to move ball and stop ball
    - Client of the position service at /gazebo/get_model_state
    - Advertise service move_ball and stop_ball
14. Implemenet `find_distance` plugin to find distance between two object
    - Client of the position service at /gazebo/get_model_state
    - Calculate position different between two objects
    - Advertise service at calculateDist
15. Implement `game_plugin`
16. Implement `behavior_tree` plugin for decision making

## How to run

1. Clone this repository
2. Change directory to `workspace-team-2`
3. Compile and source

    ```sh
    catkin_make
    source devel/setup.bash
    ```

4. Create build folder (if it isn't already created) and compile our program

    ```sh
    cd src/ssl_robocup_gazebo
    mkdir -p build
    cd build
    cmake ../
    make
    ```

    If you get error, see [Notes](#notes)
5. Run using `roslaunch`

    ``` sh
    roslaunch ssl_robocup_gazebo ssl_robocup.launch 
    ```

## Notes

- Try delete `build` and `devel` folder first and compile again from `catkin_make`
- If still error, try to change codes for include header file using relative path, something like:

    ```cpp
    #include "../../../devel/include/ssl_robocup_gazebo/MoveBall.h"
    ```
