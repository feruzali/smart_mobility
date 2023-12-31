package com.javohirjambulov.rosandroid.model.repositories.rosRepo.message;

import org.ros.internal.message.Message;


public class RosData {

    private final Topic topic;
    private final Message message;


    public RosData(Topic topic, Message message) {
        this.topic = topic;
        this.message = message;
    }


    public Topic getTopic() {
        return this.topic;
    }

    public Message getMessage() {
        return this.message;
    }
}
