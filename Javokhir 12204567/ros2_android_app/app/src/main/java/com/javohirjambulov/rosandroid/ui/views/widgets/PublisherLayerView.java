package com.javohirjambulov.rosandroid.ui.views.widgets;

import android.content.Context;

import com.javohirjambulov.rosandroid.model.repositories.rosRepo.node.BaseData;
import com.javohirjambulov.rosandroid.ui.general.DataListener;
import com.javohirjambulov.rosandroid.ui.opengl.visualisation.VisualizationView;

import javax.microedition.khronos.opengles.GL10;



public abstract class PublisherLayerView extends LayerView implements IPublisherView{

    private DataListener dataListener;


    public PublisherLayerView(Context context) {
        super(context);
    }


    @Override
    public void publishViewData(BaseData data) {
        if(dataListener == null) return;

        data.setTopic(widgetEntity.topic);
        dataListener.onNewWidgetData(data);
    }

    @Override
    public void setDataListener(DataListener listener) {
        this.dataListener = listener;
    }

    @Override
    public void draw(VisualizationView view, GL10 gl) {}
}
