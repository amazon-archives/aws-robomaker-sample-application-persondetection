# AWS RoboMaker Simulations Common utilities and helpers

### Route Manager
Node that sends a route of target goals to the robot. Sends to the move_base server.

Includes in your .launch:
```
<launch>
    ...
    <node pkg="aws_robomaker_simulation_common" type="route_manager" name="route_manager" output="screen">
      <rosparam file="$(find <your_package>)/routes/route.yaml" command="load"/> 
    </node>
    ...
</launch>
```
