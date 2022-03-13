# sample_comm

## About

`sample_comm` stands for *sample communication*. This respository is created for demonstrating basic communication between nodes in ROS during the Step Sessions conducted by Robotics Club, IIITDM Kancheepuram.

## Commands

1. Display `Hello world` on console
    ```bash
    # for cpp node
    rosrun sample_comm hello_cpp

    # for python node
    rosrun sample_comm hello.py
    ```

2. To start the standard publisher node
    ```bash
    # for cpp node
    rosrun sample_comm chatter

    # for python node
    rosrun sample_comm chatter.py
    ```

3. To start the standard subscriber node
    ```bash
    # for cpp node
    rosrun sample_comm listener

    # for python node
    rosrun sample_comm listener.py
    ```