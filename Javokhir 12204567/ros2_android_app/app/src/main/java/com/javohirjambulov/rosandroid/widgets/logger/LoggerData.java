package com.javohirjambulov.rosandroid.widgets.logger;

import com.javohirjambulov.rosandroid.model.repositories.rosRepo.node.BaseData;




public class LoggerData extends BaseData {

    public String data;

    public LoggerData(std_msgs.String message) {
        this.data = message.getData();
    }
}
