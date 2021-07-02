# Workspace-Team-2

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

## How to run
1. Clone this repository
2. Change directory to `workspace-team-2`
3. Compile and source
    ```
    catkin_make
    source devel/setup.bash
    ```
4. Run using `roslaunch`
    ```
    roslaunch ssl_robocup_gazebo ssl_robocup.launch 
    ```

