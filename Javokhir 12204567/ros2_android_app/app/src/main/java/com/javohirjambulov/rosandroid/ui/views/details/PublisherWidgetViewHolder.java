package com.javohirjambulov.rosandroid.ui.views.details;

import android.view.View;

import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;
import com.javohirjambulov.rosandroid.viewmodel.DetailsViewModel;

import java.util.List;



public abstract class PublisherWidgetViewHolder extends DetailViewHolder {

    private WidgetViewHolder widgetViewHolder;
    private PublisherViewHolder publisherViewHolder;


    public PublisherWidgetViewHolder() {
        this.widgetViewHolder = new WidgetViewHolder(this);
        this.publisherViewHolder = new PublisherViewHolder(this);
        this.publisherViewHolder.topicTypes = this.getTopicTypes();
    }


    public abstract List<String> getTopicTypes();


    @Override
    public void setViewModel(DetailsViewModel viewModel) {
        super.setViewModel(viewModel);
        publisherViewHolder.viewModel = viewModel;
    }

    public void baseInitView(View view) {
        widgetViewHolder.baseInitView(view);
        publisherViewHolder.baseInitView(view);
    }

    public void baseBindEntity(BaseEntity entity) {
        widgetViewHolder.baseBindEntity(entity);
        publisherViewHolder.baseBindEntity(entity);
    }

    public void baseUpdateEntity(BaseEntity entity) {
        widgetViewHolder.baseUpdateEntity(entity);
        publisherViewHolder.baseUpdateEntity(entity);
    }
}
