package com.javohirjambulov.rosandroid.model.repositories;

import androidx.lifecycle.LiveData;

import com.javohirjambulov.rosandroid.model.entities.SSHEntity;




public interface SshRepository {

    void startSession();

    void stopSession();

    LiveData<Boolean> isConnected();

    void sendMessage(String message);

    void abort();

    LiveData<String> getOutputData();

    void updateSSH(SSHEntity ssh);

    LiveData<SSHEntity> getCurrentSSH();
}
