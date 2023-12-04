package com.javohirjambulov.rosandroid.viewmodel;

import android.app.Application;

import androidx.annotation.NonNull;
import androidx.lifecycle.AndroidViewModel;
import androidx.lifecycle.LiveData;

import com.javohirjambulov.rosandroid.domain.RosDomain;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.RosData;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.node.BaseData;
import com.javohirjambulov.rosandroid.model.entities.widgets.BaseEntity;

import java.util.List;



public class VizViewModel extends AndroidViewModel {

    private static final String TAG = VizViewModel.class.getSimpleName();

    private final RosDomain rosDomain;


    public VizViewModel(@NonNull Application application) {
        super(application);

        rosDomain = RosDomain.getInstance(application);
    }

    public void updateWidget(BaseEntity widget) {
        rosDomain.updateWidget(null, widget);
    }

    public LiveData<List<BaseEntity>> getCurrentWidgets() {
        return rosDomain.getCurrentWidgets();
    }

    public LiveData<RosData> getData() {
        return this.rosDomain.getData();
    }


    public void publishData(BaseData data) {
        rosDomain.publishData(data);
    }
}
