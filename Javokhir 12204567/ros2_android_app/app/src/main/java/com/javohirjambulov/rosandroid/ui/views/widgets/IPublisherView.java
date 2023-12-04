package com.javohirjambulov.rosandroid.ui.views.widgets;

import com.javohirjambulov.rosandroid.model.repositories.rosRepo.node.BaseData;
import com.javohirjambulov.rosandroid.ui.general.DataListener;


public interface IPublisherView {

    void publishViewData(BaseData data);
    void setDataListener(DataListener listener);
}
