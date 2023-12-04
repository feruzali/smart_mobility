package com.javohirjambulov.rosandroid.widgets.path;

import android.view.View;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.SubscriberLayerViewHolder;

import java.util.Collections;
import java.util.List;

import nav_msgs.Path;



public class PathDetailVH extends SubscriberLayerViewHolder {


    @Override
    protected void initView(View parentView) {

    }

    @Override
    protected void bindEntity(BaseEntity entity) {

    }

    @Override
    protected void updateEntity(BaseEntity entity) {

    }

    @Override
    public List<String> getTopicTypes() {
        return Collections.singletonList(Path._TYPE);
    }
}
