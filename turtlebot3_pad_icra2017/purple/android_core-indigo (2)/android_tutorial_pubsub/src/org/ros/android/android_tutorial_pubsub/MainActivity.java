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

package org.ros.android.android_tutorial_pubsub;

import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;

import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 * @author rwjung@robotis.com (Leon Ryuwoon Jung)
 */
public class MainActivity extends RosActivity {

  private RosTextView<std_msgs.String> rosTextView;
  private Talker talker;

  public ImageButton imageButtonBurger;
  public ImageButton imageButtonCoffee;
  public ImageButton imageButtonWaffle;

  public MainActivity() {
    // The RosActivity constructor configures the notification title and ticker
    // messages.
    super("Pubsub Tutorial", "Pubsub Tutorial");
  }

  @SuppressWarnings("unchecked")
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);

    imageButtonBurger = (ImageButton)findViewById(R.id.button_burger);
    imageButtonCoffee = (ImageButton)findViewById(R.id.button_coffee);
    imageButtonWaffle = (ImageButton)findViewById(R.id.button_waffle);

    setupMessageButton();

    rosTextView = (RosTextView<std_msgs.String>) findViewById(R.id.text);
    rosTextView.setTopicName("/tb3p/pad_status");
    rosTextView.setMessageType(std_msgs.String._TYPE);
    rosTextView.setMessageToStringCallable(new MessageCallable<String, std_msgs.String>() {
      @Override
      public String call(std_msgs.String message) {
        return message.getData();
      }
    });

  }

  private void setupMessageButton()
  {
    imageButtonBurger.setOnClickListener(new View.OnClickListener()
    {
      @Override
      public void onClick(View v)
      {
        talker.button_pressed[0] = true;
      }
    });

    imageButtonCoffee.setOnClickListener(new View.OnClickListener()
    {
      @Override
      public void onClick(View v)
      {
        talker.button_pressed[1] = true;
      }
    });

    imageButtonWaffle.setOnClickListener(new View.OnClickListener()
    {
      @Override
      public void onClick(View v)
      {
        talker.button_pressed[2] = true;
      }
    });
  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
    talker = new Talker();

    // At this point, the user has already been prompted to either enter the URI
    // of a master to use or to start a master locally.

    // The user can easily use the selected ROS Hostname in the master chooser
    // activity.
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
    nodeConfiguration.setMasterUri(getMasterUri());
    nodeMainExecutor.execute(talker, nodeConfiguration);
    // The RosTextView is also a NodeMain that must be executed in order to
    // start displaying incoming messages.
    nodeMainExecutor.execute(rosTextView, nodeConfiguration);
  }
}
