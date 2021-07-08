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
8. Implemented follow_ball which implement:
    - Client of the position service at /gazebo/get_model_state
    - Follow the ball with naive approach (by adding a linear velocity by the distance)
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
11. Implement `pass_ball` plugin to pass the ball from one robot to another robot
    - Store ModelPtr of the model
    - Get the WorldPtr and store it
    - Get the teammate robot ModelPtr and store it
    - Find position such that the ball, the model robot, and the teammate robot are in one line, then go to that position
    - Check whether the distance between the ball and the robot is below the threshold, if below, kick (set linear velocity of) the ball 

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
5. Run using `roslaunch`
    ```
    roslaunch ssl_robocup_gazebo ssl_robocup.launch 
    ```

