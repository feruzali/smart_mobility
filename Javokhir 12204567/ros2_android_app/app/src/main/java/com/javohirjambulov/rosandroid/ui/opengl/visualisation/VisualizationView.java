/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.javohirjambulov.rosandroid.ui.opengl.visualisation;

import android.content.Context;
import android.graphics.PixelFormat;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.view.MotionEvent;

import com.google.common.collect.Lists;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.RosData;
import com.javohirjambulov.rosandroid.model.repositories.rosRepo.message.Topic;
import com.javohirjambulov.rosandroid.ui.opengl.layer.CameraControl;
import com.javohirjambulov.rosandroid.ui.views.widgets.IPublisherView;
import com.javohirjambulov.rosandroid.ui.views.widgets.ISubscriberView;
import com.javohirjambulov.rosandroid.ui.views.widgets.LayerView;

import org.ros.internal.message.Message;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;



public class VisualizationView extends GLSurfaceView {

    public static String TAG = VisualizationView.class.getSimpleName();

    private XYOrthographicCamera camera;
    private CameraControl cameraControl;
    private List<LayerView> layers;
    private XYOrthographicRenderer renderer;


    public VisualizationView(Context context) {
        super(context);
        init();
    }

    public VisualizationView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }


    private void init() {
        this.layers = new ArrayList<>();
        this.renderer = new XYOrthographicRenderer(this);

        setDebugFlags(DEBUG_CHECK_GL_ERROR);
        setZOrderMediaOverlay(true);
        getHolder().setFormat(PixelFormat.TRANSLUCENT);
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        setRenderer(renderer);
        setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);

        //frameTransformTree = new FrameTransformTree();
        camera = new XYOrthographicCamera();

        //camera = new XYOrthographicCamera(frameTransformTree);
        cameraControl = new CameraControl(this);
        cameraControl.init(true, true, true);
        camera.jumpToFrame("map");
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        for (LayerView layer : Lists.reverse(layers)) {
            if (layer.onTouchEvent(this, event)) {
                this.requestRender();
                return true;
            }
        }

        if(cameraControl.onTouchEvent(event)) {
            this.requestRender();
            return true;
        }

        return super.onTouchEvent(event);
    }

    public XYOrthographicRenderer getRenderer() {
        return renderer;
    }

    public XYOrthographicCamera getCamera() {
        return camera;
    }

    public List<LayerView> getLayers() {
        return Collections.unmodifiableList(layers);
    }


    public void addLayer(LayerView layer) {
        layer.setParentView(this);
        layers.add(layer);

        if (layer instanceof IPublisherView) {
            layer.setFrame(camera.getFrame());
        }
    }

    public void onNewData(RosData data) {
        Message message = data.getMessage();
        Topic topic = data.getTopic();
        boolean dirtyView = false;

        // Forward message to sub layers
        for(LayerView layer: getLayers()) {
            if (layer instanceof ISubscriberView) {
                if (layer.getWidgetEntity().topic.equals(topic)) {
                    ((ISubscriberView)layer).onNewMessage(message);
                    dirtyView = true;
                }
            }
        }

        if (dirtyView) {
            this.requestRender();
        }
    }

}