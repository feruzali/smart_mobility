package com.javohirjambulov.rosandroid.ui.views.widgets;

import org.ros.internal.message.Message;



public interface ISubscriberView {

     void onNewMessage(Message message);
}
