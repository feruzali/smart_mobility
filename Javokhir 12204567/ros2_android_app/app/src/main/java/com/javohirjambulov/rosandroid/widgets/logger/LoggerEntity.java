package com.javohirjambulov.rosandroid.widgets.logger;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberWidgetEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;




public class LoggerEntity extends SubscriberWidgetEntity {

    public String text;
    public int rotation;


    public LoggerEntity() {
        this.width = 3;
        this.height = 1;
        this.topic = new Topic("log", std_msgs.String._TYPE);
        this.text = "A logger";
        this.rotation = 0;
    }

}
