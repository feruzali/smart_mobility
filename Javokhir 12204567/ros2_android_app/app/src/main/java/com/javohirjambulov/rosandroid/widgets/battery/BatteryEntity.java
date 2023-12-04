package com.javohirjambulov.rosandroid.widgets.battery;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberWidgetEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;



public class BatteryEntity extends SubscriberWidgetEntity {

    public boolean displayVoltage;


    public BatteryEntity() {
        this.width = 1;
        this.height = 2;
        this.topic = new Topic("battery", sensor_msgs.BatteryState._TYPE);
        this.displayVoltage = false;
    }

}
