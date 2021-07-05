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

## How to run
1. Clone this repository
2. Change directory to `workspace-team-2`
3. Compile and source
    ```sh
    catkin_make
    source devel/setup.bash
    ```
4. Run using `roslaunch`
    ```
    roslaunch ssl_robocup_gazebo ssl_robocup.launch 
    ```

