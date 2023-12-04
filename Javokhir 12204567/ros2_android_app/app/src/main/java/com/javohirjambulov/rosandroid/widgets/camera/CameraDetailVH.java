package com.javohirjambulov.rosandroid.widgets.camera;

import android.view.View;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.ui.views.details.SubscriberWidgetViewHolder;

import java.util.Arrays;
import java.util.List;

import sensor_msgs.CompressedImage;
import sensor_msgs.Image;



public class CameraDetailVH extends SubscriberWidgetViewHolder {

    public static final String TAG = CameraDetailVH.class.getSimpleName();


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
        return Arrays.asList(Image._TYPE, CompressedImage._TYPE);
    }

}
