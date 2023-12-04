package com.javohirjambulov.rosandroid.widgets.button;

import com.javohirjambulov.rosandroid.model.entities.widgets.PublisherWidgetEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import std_msgs.Bool;




public class ButtonEntity extends PublisherWidgetEntity {

    public String text;
    public int rotation;


    public ButtonEntity() {
        this.width = 2;
        this.height = 1;
        this.topic = new Topic("btn_press", Bool._TYPE);
        this.immediatePublish = true;
        this.text = "A button";
        this.rotation = 0;
    }

}
