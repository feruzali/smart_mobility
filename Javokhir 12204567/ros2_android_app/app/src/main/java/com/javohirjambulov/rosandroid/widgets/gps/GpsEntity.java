package com.javohirjambulov.rosandroid.widgets.gps;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberWidgetEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import sensor_msgs.NavSatFix;



public class GpsEntity extends SubscriberWidgetEntity {

    public GpsEntity() {
        this.width = 8;
        this.height = 8;
        this.topic = new Topic("gps", NavSatFix._TYPE);
    }
}

