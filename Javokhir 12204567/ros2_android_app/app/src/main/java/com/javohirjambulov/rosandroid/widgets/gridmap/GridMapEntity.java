package com.javohirjambulov.rosandroid.widgets.gridmap;

import com.javohirjambulov.rosandroid.model.entities.widgets.SubscriberLayerEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;

import nav_msgs.OccupancyGrid;



public class GridMapEntity extends SubscriberLayerEntity {
    
    public GridMapEntity() {
        this.topic = new Topic("/move_base/local_costmap/costmap", OccupancyGrid._TYPE);
    }
    
}
