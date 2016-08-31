##By using this node we can do the below things.

```
    <rosparam file="$(find back_and_forth)/config/goal_manager.yaml" command="load"/>
    <node name="goal_manager" pkg="back_and_forth" type="goal_manager" output="screen"/>
```

- Set a goal sequence in the "goal_manager.yaml", and the move_base
   from the navigation stack will follow these goals sequentially.

```
e.g.
goal_sequence:
 - [-89.8889, -41.1088, 0.91191]
 - [-94.0366, -39.0583, 2.67807]
 - [-95.5715, -48.1269, -1.45454]
```

There are three goals in the goal_sequence.
After loaded the yaml, robot will go through these three points.

####Note: If don't want goal sequence, just do not load this yaml into ros parameter server, and this node can be also functional.
(It is okay to ignore this error message: [ERROR] [1472545688.517191475]: get goal_sequence error)

 - Make a queue goals from rviz (or any topic advertiser)
   - Change the rviz 2D Navigation topic to "/new_goal_stamped"
   - This node can get the topic message from rviz, and push these goals
     into a queue, and robot will go through all the point in this queue.

####Note: The priority of goals in queue are higher than goal_sequence, which means that robot will go throuth the queue goal then the goal_sequence
####Note; The setting for queue goals will interrupt the action when doing the goal_sequence.

 - Cancel action and clear all the goal
   By calling this topic, "cancel_goal", robot will stop and rease all the
   goals in the queue.

```
e.g.
rostopic pub -1 /cancel_goal std_msgs/String "data: ''"
```
