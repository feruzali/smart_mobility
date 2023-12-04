package com.javohirjambulov.rosandroid.widgets.button;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.node.BaseData;

import org.ros.internal.message.Message;
import org.ros.node.topic.Publisher;

import std_msgs.Bool;



public class ButtonData extends BaseData {

    public boolean pressed;

    public  ButtonData(boolean pressed) {
        this.pressed = pressed;
    }

    @Override
    public Message toRosMessage(Publisher<Message> publisher, BaseEntity widget) {
        std_msgs.Bool message = (Bool) publisher.newMessage();
        message.setData(pressed);
        return message;
    }
}
