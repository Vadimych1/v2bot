# Between-Node Communication Diagram

/ TODO: change structure from [nodes] [topics] to [node 1 topics 1] [node 2 topics 2] etc
```mermaid
graph LR;
    subgraph nodes;
        direction TB;
        A[lidar];
        B[slam];
        C[pathplanner];
        D[motioncontroller];
        E[goalmanager];
        F[motorcontroller];
    end;

    subgraph topics;
        direction TB;
        AA(/lidar);
        BA(/map);
        BB(/pose);
        CA(/globalpath);
        DA(/cmdvel);
        EA(/currentgoal);
        FA(/odometry);
        FB(/velocity);
    end

    A -->|default/LidarDatatype| AA;
    B -->|slam/SLAMOffsetMap| BA;
    B -->|default/Movement| BB;
    C -->|default/NumpyArray| CA;
    D -->|default/Vector| DA;
    E -->|default/Vector| EA;
    F -->|default/Vector| FA;
    F -->|default/Vector| FB;

    AA --> B;
    
    BA --> C;
    BA --> D;
    
    BB --> C;
    BB --> D;
    BB --> E;
    
    CA --> D;

    DA --> F;

    EA --> C;

    FA --> B;
    
    FB --> D;
    FB --> C;
```

@ is ANON receive, / is TOPIC send

- lidar
/lidar (miniros/LidarDatatype)
@ping (None)

- slam
/map (slam/SLAMOffsetMap)
/pose (miniros/Movement)

- pathplanner
/globalpath (miniros/NumpyArray)

- motioncontroller
/cmdvel (miniros/Vector) (left, right, none)

- goalmanager
/currentgoal (miniros/Vector) (x, y, 0)

- motorcontroller
/odometry (miniros/Vector) (x, y, theta)
/velocity (miniros/Vector) (v, omega, 0)