package com.javohirjambulov.rosandroid.widgets.camera;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberWidgetEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import sensor_msgs.Image;



public class CameraEntity extends SubscriberWidgetEntity {

    int colorScheme;
    boolean drawBehind;
    boolean useTimeStamp;


    public CameraEntity() {
        this.width = 8;
        this.height = 6;
        this.topic = new Topic("camera/image_raw", Image._TYPE);
    }
}

