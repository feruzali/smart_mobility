package com.javohirjambulov.rosandroid.widgets.rqtplot;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberWidgetEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import org.ros.node.topic.Subscriber;



public class RqtPlotEntity extends SubscriberWidgetEntity {

    public String fieldPath;

    public RqtPlotEntity() {
        this.width = 8;
        this.height = 6;
        this.topic = new Topic("/plot", Subscriber.TOPIC_MESSAGE_TYPE_WILDCARD);
        this.fieldPath = "/pos/xy";
    }
}
