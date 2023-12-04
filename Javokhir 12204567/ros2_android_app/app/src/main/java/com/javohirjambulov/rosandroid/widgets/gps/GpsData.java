package com.javohirjambulov.rosandroid.widgets.gps;

import com.javohirjambulov.rosandroid.model.repositories.rosRepo.node.BaseData;

import sensor_msgs.NavSatFix;




public class GpsData extends BaseData {

    private NavSatFix navSatFix;

    public GpsData(NavSatFix navSatFix) {
        this.navSatFix = navSatFix;
    }

    public double getLat() {
        return navSatFix.getLatitude();
    }

    public double getLon() {
        return navSatFix.getLongitude();
    }
}
