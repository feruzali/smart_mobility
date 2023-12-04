package com.javohirjambulov.rosandroid.widgets.path;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberLayerEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;


/**
 * Path entity represents a widget which subscribes
 * to a topic with message type "nav_msgs.Path".
 * Usable in 2D widgets.
 */
public class PathEntity extends SubscriberLayerEntity {

    public float lineWidth;
    public String lineColor;


    public PathEntity() {
        this.topic = new Topic("/move_base/TebLocalPlannerROS/local_plan", nav_msgs.Path._TYPE);
        this.lineWidth = 4;
        this.lineColor = "ff0000ff";
    }
}
