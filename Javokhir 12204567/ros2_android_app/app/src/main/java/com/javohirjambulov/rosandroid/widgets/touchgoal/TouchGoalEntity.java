package com.javohirjambulov.rosandroid.widgets.touchgoal;

import com.javohirjambulov.rosandroid.model.entities.widgets.PublisherLayerEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import geometry_msgs.PoseStamped;



public class TouchGoalEntity extends PublisherLayerEntity {

    public TouchGoalEntity() {
        this.topic = new Topic("/goal", PoseStamped._TYPE);
        this.immediatePublish = true;
    }
}
