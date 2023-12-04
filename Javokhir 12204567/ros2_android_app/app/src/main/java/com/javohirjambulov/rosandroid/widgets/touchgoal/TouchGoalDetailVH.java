package com.javohirjambulov.rosandroid.widgets.touchgoal;

import android.view.View;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.PublisherLayerViewHolder;

import java.util.Collections;
import java.util.List;

import geometry_msgs.PoseStamped;



public class TouchGoalDetailVH extends PublisherLayerViewHolder {

    private static final String TAG = TouchGoalDetailVH.class.getSimpleName();


    @Override
    protected void initView(View parentView) {
    }

    @Override
    protected void bindEntity(BaseEntity entity) {
        TouchGoalEntity scanEntity = (TouchGoalEntity) entity;
    }

    @Override
    protected void updateEntity(BaseEntity entity) {
        TouchGoalEntity scanEntity = (TouchGoalEntity) entity;
    }

    @Override
    public List<String> getTopicTypes() {
        return Collections.singletonList(PoseStamped._TYPE);
    }

}
