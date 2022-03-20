# Roombaの実機環境でのNavigaton

* 参考ページ
TBD

* 使用ドライバ
    * [RoboticaUtnFrba/libcreate](https://github.com/RoboticaUtnFrba/libcreate)
    * [RoboticaUtnFrba/create_autonomy](https://github.com/RoboticaUtnFrba/create_autonomy)

* 構成図

```mermaid
flowchart TB
  subgraph Roomba
    direction TB
    subgraph C2[rplidar]
        direction TB
    end
    subgraph C1[ca_driver]
        direction TB
    end
  end
  subgraph PC
    direction TB
    subgraph B1[move_base]
        direction TB
    end
    subgraph B4[Rviz]
        direction TB
    end
    subgraph B3[map_server]
        direction TB
    end
    subgraph B2[amcl]
        direction TB
    end
    subgraph B5[roscore]
        direction TB
    end
  end
B1 -- /create1/cmd_vel --> C1
C2 -- /scan --> B2
C2 -- /scan --> B1
C1 -- /create1/odom --> B2
B2 -- /tf odom->base_link --> B1
B2 -- /tf map->odom --> B1
B4 -- /move_base_simple/goal --> B1
B3 -- /map --> B1
B3 -- /map --> B2
```

