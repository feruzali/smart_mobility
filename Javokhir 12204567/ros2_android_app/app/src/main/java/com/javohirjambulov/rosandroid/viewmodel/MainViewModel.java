package com.javohirjambulov.rosandroid.viewmodel;

import android.app.Application;

import androidx.annotation.NonNull;
import androidx.lifecycle.AndroidViewModel;
import androidx.lifecycle.LiveData;
import androidx.lifecycle.MediatorLiveData;

import com.javohirjambulov.rosandroid.model.repositories.ConfigRepository;
import com.javohirjambulov.rosandroid.model.repositories.ConfigRepositoryImpl;



public class MainViewModel extends AndroidViewModel {

    private static final String TAG = MainViewModel.class.getSimpleName();

    ConfigRepository configRepo;
    MediatorLiveData<String> configTitle;


    public MainViewModel(@NonNull Application application) {
        super(application);

        configRepo = ConfigRepositoryImpl.getInstance(getApplication());
    }


    public LiveData<String> getConfigTitle() {
        if (configTitle == null) {
            configTitle = new MediatorLiveData<>();

            configTitle.addSource(configRepo.getCurrentConfig(), configuration -> {
                if (configuration == null)
                    return;

                configTitle.setValue(configuration.name);
            });
        }

        return configTitle;
    }

    public void createFirstConfig(String name) {
        configRepo.createConfig(name);
    }
}
