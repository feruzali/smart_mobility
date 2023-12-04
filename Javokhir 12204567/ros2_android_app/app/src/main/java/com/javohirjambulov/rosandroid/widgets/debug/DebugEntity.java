package com.javohirjambulov.rosandroid.widgets.debug;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberWidgetEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import org.ros.node.topic.Subscriber;



public class DebugEntity extends SubscriberWidgetEntity {

    public int numberMessages;


    public DebugEntity() {
        this.width = 4;
        this.height = 3;
        this.topic = new Topic("MessageToDebug", Subscriber.TOPIC_MESSAGE_TYPE_WILDCARD);
        this.numberMessages = 10;
    }
}
