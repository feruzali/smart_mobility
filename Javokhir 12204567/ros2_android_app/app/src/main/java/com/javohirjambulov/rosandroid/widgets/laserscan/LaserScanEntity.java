package com.javohirjambulov.rosandroid.widgets.laserscan;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberLayerEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;
import com.javohirjambulov.rosandroid.ui.opengl.visualisation.ROSColor;

import sensor_msgs.LaserScan;



public class LaserScanEntity extends SubscriberLayerEntity {

    public int pointsColor;
    public int areaColor;
    public int pointSize;
    public boolean showFreeSpace;


    public LaserScanEntity() {
        this.topic = new Topic("/scan", LaserScan._TYPE);
        this.pointsColor = ROSColor.fromHexAndAlpha("377dfa", 0.6f).toInt();
        this.areaColor = ROSColor.fromHexAndAlpha("377dfa", 0.2f).toInt();
        this.pointSize = 10;
        this.showFreeSpace = true;
    }
}
