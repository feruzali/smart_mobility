package com.javohirjambulov.rosandroid.viewmodel;

import android.app.Application;
import android.content.Context;
import android.net.wifi.WifiManager;

import androidx.annotation.NonNull;
import androidx.lifecycle.AndroidViewModel;
import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;

import com.javohirjambulov.rosandroid.domain.RosDomain;
import com.javohirjambulov.rosandroid.model.entities.MasterEntity;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.connection.ConnectionType;
import com.javohirjambulov.rosandroid.utility.Utils;

import java.util.ArrayList;



public class MasterViewModel extends AndroidViewModel {

    private static final String TAG = MasterViewModel.class.getSimpleName();
    private static final long MIN_HELP_TIMESPAM = 30 * 1000;

    private final RosDomain rosDomain;

    private MutableLiveData<String> networkSSIDLiveData;
    private final LiveData<MasterEntity> currentMaster;

    private long lastTimeHelpShowed;


    public MasterViewModel(@NonNull Application application) {
        super(application);

        rosDomain = RosDomain.getInstance(application);
        currentMaster = rosDomain.getCurrentMaster();
    }


    public void updateHelpDisplay() {
        lastTimeHelpShowed = System.currentTimeMillis();
    }

    public boolean shouldShowHelp() {
        return System.currentTimeMillis() - lastTimeHelpShowed >= MIN_HELP_TIMESPAM;
    }

    public void setMasterIp(String ipString) {
        MasterEntity master = currentMaster.getValue();

        if (master == null) return;

        master.ip = ipString;
        rosDomain.updateMaster(master);
    }

    public void setMasterPort(String portString) {
        int port = Integer.parseInt(portString);
        MasterEntity master = currentMaster.getValue();
        if (master == null) return;

        master.port = port;
        rosDomain.updateMaster(master);
    }

    public void setMasterDeviceIp(String deviceIpString) {
        rosDomain.setMasterDeviceIp(deviceIpString);
    }

    public void connectToMaster() {
        setWifiSSID();
        rosDomain.connectToMaster();
    }

    public void disconnectFromMaster() {
        rosDomain.disconnectFromMaster();
    }

    public LiveData<MasterEntity> getMaster() {
        return rosDomain.getCurrentMaster();
    }

    public LiveData<ConnectionType> getRosConnection() {
        return rosDomain.getRosConnection();
    }

    public String setDeviceIp(String deviceIp){
        return deviceIp;
    }

    public LiveData<String> getCurrentNetworkSSID(){
        if (networkSSIDLiveData == null) {
            networkSSIDLiveData = new MutableLiveData<>();
        }

        setWifiSSID();

        return networkSSIDLiveData;
    }

    public ArrayList<String> getIPAddressList() {
        return Utils.getIPAddressList(true);
    }

    public String getIPAddress() {return Utils.getIPAddress(true); }

    private void setWifiSSID() {
        WifiManager wifiManager = (WifiManager) getApplication().getApplicationContext()
                .getSystemService(Context.WIFI_SERVICE);
        String ssid = Utils.getWifiSSID(wifiManager);

        if (ssid == null) {
            ssid = "None";
        }

        networkSSIDLiveData.postValue(ssid);
    }

}
