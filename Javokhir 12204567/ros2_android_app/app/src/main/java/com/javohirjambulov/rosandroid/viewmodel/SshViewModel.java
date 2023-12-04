package com.javohirjambulov.rosandroid.viewmodel;

import android.app.Application;

import androidx.annotation.NonNull;
import androidx.lifecycle.AndroidViewModel;
import androidx.lifecycle.LiveData;

import com.javohirjambulov.rosandroid.model.entities.SSHEntity;
import com.javohirjambulov.rosandroid.model.repositories.SshRepositoryImpl;




public class SshViewModel extends AndroidViewModel {

    SshRepositoryImpl sshRepositoryImpl;
    private final LiveData<SSHEntity> currentSSH;


    public SshViewModel(@NonNull Application application) {
        super(application);
        sshRepositoryImpl = SshRepositoryImpl.getInstance(application);
        currentSSH = sshRepositoryImpl.getCurrentSSH();
    }

    public void setSshIp(String ipString) {
        SSHEntity ssh = currentSSH.getValue();
        ssh.ip = ipString;
        sshRepositoryImpl.updateSSHConfig(ssh);
    }

    public void setSshPort(String portString) {
        int port = 22;

        try {
            port = Integer.parseInt(portString);
        } catch (NumberFormatException nfe) {
            nfe.printStackTrace();
        }

        SSHEntity ssh = currentSSH.getValue();
        ssh.port = port;
        sshRepositoryImpl.updateSSHConfig(ssh);
    }

    public void setSshUsername(String usernameString) {
        SSHEntity ssh = currentSSH.getValue();
        ssh.username = usernameString;
        sshRepositoryImpl.updateSSHConfig(ssh);
    }

    public void setSshPassword(String passwordString) {
        SSHEntity ssh = currentSSH.getValue();
        ssh.password = passwordString;
        sshRepositoryImpl.updateSSHConfig(ssh);
    }


    public void connectViaSSH() {
        sshRepositoryImpl.startSession();
    }

    public void stopSsh() {
        sshRepositoryImpl.stopSession();
    }

    public void sendViaSSH(String message) {
        sshRepositoryImpl.sendMessage(message);
    }

    public void abortAction() {sshRepositoryImpl.abort();}

    public LiveData<Boolean> isConnected() {
        return sshRepositoryImpl.isConnected();
    }

    public LiveData<String> getOutputData() {
        return sshRepositoryImpl.getOutputData();
    }

    public LiveData<SSHEntity> getSSH() {
        return sshRepositoryImpl.getCurrentSSH();
    }

}
