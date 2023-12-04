package com.javohirjambulov.rosandroid.widgets.pose;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberLayerEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import geometry_msgs.PoseWithCovarianceStamped;


/**
 * Pose entity represents a widget which subscribes
 * to a topic with message type "geometry_msgs.PoseStamped".
 * Usable in 2D widgets.
 */
public class PoseEntity extends SubscriberLayerEntity {


    public PoseEntity() {
        this.topic = new Topic("/amcl_pose", PoseWithCovarianceStamped._TYPE);
    }
}
