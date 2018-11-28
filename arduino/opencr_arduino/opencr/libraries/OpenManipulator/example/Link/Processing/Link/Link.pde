/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Hye-Jong KIM */

/**
 * this code is compatible with open_manipulator_chain.ino
**/

// Multiple Window
ChildApplet child;

// Control Interface
import controlP5.*;

// Init serial
import processing.serial.*;

// Shape variables
PShape base, link0, link1, link2, link3, link4, link5, link6, link7, tool;
PShape ctrl_base, ctrl_link0, ctrl_link1, ctrl_link2, ctrl_link3, ctrl_link4, ctrl_link5, ctrl_link6, ctrl_link7, ctrl_tool;
// Model pose
float model_rot_x, model_rot_z, model_trans_x, model_trans_y, model_trans_z, model_scale_factor;

// World pose
float world_rot_x, world_rot_y;

// Serial variable
Serial opencr_port;

// Angle variable
float[] joint_angle = new float[11];
float[] current_joint_angle = new float[11];

int tabFlag = 1;

float[] visual_target_pose_x = new float[50];
float[] visual_target_pose_y = new float[50];
float[] visual_target_pose_z = new float[50];

/*******************************************************************************
* Setting window size
*******************************************************************************/
void settings()
{
  size(900, 900, OPENGL);
}

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  surface.setTitle("OpenManipulator Link");
  child = new ChildApplet();

  initShape();
  initView();

  connectOpenCR(0); // It is depend on laptop enviroments.
}

/*******************************************************************************
* Draw (loop function)
*******************************************************************************/
void draw()
{
  setWindow();

  drawTitle();
  drawWorldFrame();

  if(tabFlag == 1)
  {
    drawManipulator();
  }
  drawCurrentManipulator();
}

/*******************************************************************************
* Connect OpenCR
*******************************************************************************/
void connectOpenCR(int port_num)
{
  printArray(Serial.list());

  String port_name = Serial.list()[port_num];
  opencr_port = new Serial(this, port_name, 57600);
  opencr_port.bufferUntil('\n');
}

/*******************************************************************************
* Serial Interrupt
*******************************************************************************/
void serialEvent(Serial opencr_port)
{
  String opencr_string = opencr_port.readStringUntil('\n');
  opencr_string = trim(opencr_string);

  String[] cmd = split(opencr_string, ',');
  
  float[] received_joint_angle = new float[11];
  
  if (cmd[0].equals("angle"))
  {
    print("received joint ");
    for (int cmd_cnt = 1; cmd_cnt < cmd.length; cmd_cnt++)
    {
      received_joint_angle[cmd_cnt-1] = float(cmd[cmd_cnt]);
      if(cmd_cnt < 4)
        print(cmd_cnt + ": " + cmd[cmd_cnt] + "  ");
    }
    println("");
  }
  else
  {
    //println("Other Serial");
  }
  
  for(int i=0; i < 3; i++)
  {
    current_joint_angle[i] = received_joint_angle[i];
  }
  current_joint_angle[3] = (current_joint_angle[1] - current_joint_angle[2]);
  current_joint_angle[4] = -PI - (current_joint_angle[1] - current_joint_angle[2]);
  current_joint_angle[5] = -PI - (current_joint_angle[1] - current_joint_angle[2]);
  current_joint_angle[6] = -180*PI/180 - current_joint_angle[2];
  current_joint_angle[7] = current_joint_angle[1];
  current_joint_angle[8] = -15*PI/180 - current_joint_angle[1];
  current_joint_angle[9] = current_joint_angle[2] - 195*PI/180;
  current_joint_angle[10] = +90*PI/180 - current_joint_angle[2];  
}

/*******************************************************************************
* Init viewpoint and camera
*******************************************************************************/
void initView()
{
  float camera_y = height/2.0;
  float fov = 200/float(width) * PI/2;
  float camera_z = camera_y / tan(fov / 2.0);
  float aspect = float(width)/float(height);

  perspective(fov, aspect, camera_z/10.0, camera_z*10.0);

  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0, height/2.0-500, height/2.0 * 4,
         width/2-100, height/2, 0,
         0, 1, 0);
}

/*******************************************************************************
* Get shape
*******************************************************************************/
void initShape()
{
  base        = loadShape("meshes/base.obj");
  link0       = loadShape("meshes/link0.obj");
  link1       = loadShape("meshes/link1.obj");
  link2       = loadShape("meshes/link2.obj");
  link3       = loadShape("meshes/link3.obj");
  link4       = loadShape("meshes/link4.obj");
  link5       = loadShape("meshes/link5.obj");
  link6       = loadShape("meshes/link6.obj");
  link7       = loadShape("meshes/link7.obj");
  tool        = loadShape("meshes/tool.obj");

  ctrl_base        = loadShape("meshes/base.obj");
  ctrl_link0       = loadShape("meshes/link0.obj");
  ctrl_link1       = loadShape("meshes/link1.obj");
  ctrl_link2       = loadShape("meshes/link2.obj");
  ctrl_link3       = loadShape("meshes/link3.obj");
  ctrl_link4       = loadShape("meshes/link4.obj");
  ctrl_link5       = loadShape("meshes/link5.obj");
  ctrl_link6       = loadShape("meshes/link6.obj");
  ctrl_link7       = loadShape("meshes/link7.obj");
  ctrl_tool        = loadShape("meshes/tool.obj");

  ctrl_base.setFill(color(200));
  ctrl_link0.setFill(color(200));
  ctrl_link1.setFill(color(200));
  ctrl_link2.setFill(color(200));
  ctrl_link3.setFill(color(200));
  ctrl_link4.setFill(color(200));
  ctrl_link5.setFill(color(200));
  ctrl_link6.setFill(color(200));
  ctrl_link7.setFill(color(200));
  ctrl_tool.setFill(color(200));

  joint_angle[0] = 0*PI/180;
  joint_angle[1] = -90.0*PI/180;
  joint_angle[2] = -160*PI/180;
  current_joint_angle[0] = 0*PI/180;
  current_joint_angle[1] = -90.0*PI/180;
  current_joint_angle[2] = -160*PI/180;
  passiveJointSet();
}

/*******************************************************************************
* passiveJointSet
*******************************************************************************/

void passiveJointSet()
{
  joint_angle[3] = (joint_angle[1] - joint_angle[2]);
  joint_angle[4] = -PI - (joint_angle[1] - joint_angle[2]);
  joint_angle[5] = -PI - (joint_angle[1] - joint_angle[2]);
  joint_angle[6] = -180*PI/180 - joint_angle[2];
  joint_angle[7] = joint_angle[1];
  joint_angle[8] = -15*PI/180 - joint_angle[1];
  joint_angle[9] = joint_angle[2] - 195*PI/180;
  joint_angle[10] = +90*PI/180 - joint_angle[2];  
  
  current_joint_angle[3] = (current_joint_angle[1] - current_joint_angle[2]);
  current_joint_angle[4] = -PI - (current_joint_angle[1] - current_joint_angle[2]);
  current_joint_angle[5] = -PI - (current_joint_angle[1] - current_joint_angle[2]);
  current_joint_angle[6] = -180*PI/180 - current_joint_angle[2];
  current_joint_angle[7] = current_joint_angle[1];
  current_joint_angle[8] = -15*PI/180 - current_joint_angle[1];
  current_joint_angle[9] = current_joint_angle[2] - 195*PI/180;
  current_joint_angle[10] = +90*PI/180 - current_joint_angle[2];  
}

/*******************************************************************************
* initAngleCoefficient
*******************************************************************************/

/*******************************************************************************
* Set window characteristic
*******************************************************************************/
void setWindow()
{
  lights();
  smooth();
  background(30);

  translate(width/2, height/2, 0);

  rotateX(radians(90));
  rotateZ(radians(140));
}

/*******************************************************************************
* Draw sphere
*******************************************************************************/
void drawSphere(int x, int y, int z, int r, int g, int b, int size)
{
  pushMatrix();
  translate(x,y,z);
  stroke(r,g,b);
  sphere(size);
  popMatrix();
}

void saveSpherePose()
{
  for (int i=0; i< visual_target_pose_x.length - 1; i++)
  {
    visual_target_pose_x[i] = visual_target_pose_x[i + 1];
    visual_target_pose_y[i] = visual_target_pose_y[i + 1];
    visual_target_pose_z[i] = visual_target_pose_z[i + 1];
  }

  visual_target_pose_x[visual_target_pose_x.length - 1] = modelX(0,0,0);
  visual_target_pose_y[visual_target_pose_y.length - 1] = modelY(0,0,0);
  visual_target_pose_z[visual_target_pose_z.length - 1] = modelZ(0,0,0);
}

void drawSphereAfterEffect()
{
  for(int i = 0; i < visual_target_pose_x.length; i ++)
  { 
    pushMatrix();
    rotateZ(radians(-140));
    rotateX(radians(-90));
    translate(-width/2 , -height/2, 0);
    
    translate(visual_target_pose_x[i], visual_target_pose_y[i], visual_target_pose_z[i]);
    stroke(255,255,255);
    sphere(1.5);
    popMatrix();
  }
}

/*******************************************************************************
* Draw title
*******************************************************************************/
void drawTitle()
{
  scale(1 + model_scale_factor);
  pushMatrix();
  rotateX(radians(60));
  rotateZ(radians(180));
  textSize(60);
  fill(255,204,102);
  text("OpenManipulator Link", -450,75,0);
  textSize(10);
  fill(102,255,255);
  text("Move manipulator 'Q,A','W,S','E,D'", -450,120,0);
  text("Initial view 'I'", -450,150,0);
  popMatrix();
}

/*******************************************************************************
* Draw manipulator
*******************************************************************************/
void drawManipulator()
{
  scale(1 + model_scale_factor);

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);
  
  shape(base);
  drawLocalFrame();
  translate(0, 0, 6.79972);
  
  rotateZ(-joint_angle[0]);
  shape(link0);
  drawLocalFrame();
  translate(0, 0, 44.99960);
  
  rotateY(joint_angle[1]);
  shape(link1);
  drawLocalFrame();
  popMatrix();

///////////////////////////////////////////

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);
  
  //shape(base);
  translate(0, 0, 6.79972);
  
  rotateZ(-joint_angle[0]);
  //shape(link0);
  translate(0, 0, 44.99960);
  
  rotateY(joint_angle[2]);
  shape(link2);
  drawLocalFrame();
  translate(50, 0, 0);
  
  rotateY(joint_angle[3]);
  shape(link3);
  drawLocalFrame();
  translate(200, 0, 0);
  
  rotateY(joint_angle[4]);
  shape(link4);
  drawLocalFrame();
  translate(250, 0, 0);
  
  rotateY(joint_angle[6]);
  shape(tool);
  drawLocalFrame();
  popMatrix();

///////////////////////////////////////////
  
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);
  
  //shape(base);
  translate(0, 0, 6.79972);
  
  rotateZ(-joint_angle[0]);
  //shape(link0);
  translate(0, 0, 44.99960);
  rotateY(25*PI/180);
  translate(-50, 0, 0);
  rotateY(-25*PI/180);
  
  rotateY(joint_angle[7]);  
  shape(link5);
  drawLocalFrame();
  translate(200, 0, 0);
    
  rotateY(joint_angle[8]);
  shape(link6);
  drawLocalFrame();
  rotateY(40*PI/180);
  translate(50, 0, 0);
  rotateY(-80*PI/180);
  translate(50, 0, 0);
  rotateY(40*PI/180);
  
  rotateY(joint_angle[9]);
  rotateY(30*PI/180);
  shape(link7);
  drawLocalFrame();

  popMatrix();


}

void drawCurrentManipulator()
{
////////////////////////Current manipulator///////////////////////////////

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);
  
  shape(ctrl_base);
  drawLocalFrame();
  translate(0, 0, 6.79972);
  
  rotateZ(-current_joint_angle[0]);
  shape(ctrl_link0);
  drawLocalFrame();
  translate(0, 0, 44.99960);
  
  rotateY(current_joint_angle[1]);
  shape(ctrl_link1);
  drawLocalFrame();
  popMatrix();

///////////////////////////////////////////

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);
  
  //shape(base);
  translate(0, 0, 6.79972);
  
  rotateZ(-current_joint_angle[0]);
  //shape(link0);
  translate(0, 0, 44.99960);
  
  rotateY(current_joint_angle[2]);
  shape(ctrl_link2);
  drawLocalFrame();
  translate(50, 0, 0);
  
  rotateY(current_joint_angle[3]);
  shape(ctrl_link3);
  drawLocalFrame();
  translate(200, 0, 0);
  
  rotateY(current_joint_angle[4]);
  shape(ctrl_link4);
  drawLocalFrame();
  translate(250, 0, 0);
  
  rotateY(current_joint_angle[6]);
  shape(ctrl_tool);
  drawLocalFrame();

  translate(38.67882, 7.0, -23.37315);
  //translate(0.0, 0.0, 0.0);
  drawSphere(0, -7, 0, 100, 100, 100, 10);
  saveSpherePose();
  popMatrix();

///////////////////////////////////////////
  
  pushMatrix();
  translate(-model_trans_x, -model_trans_y, -model_trans_z);
  rotateX(model_rot_x);
  rotateZ(model_rot_z);
  
  //shape(base);
  translate(0, 0, 6.79972);
  
  rotateZ(-current_joint_angle[0]);
  //shape(link0);
  translate(0, 0, 44.99960);
  rotateY(25*PI/180);
  translate(-50, 0, 0);
  rotateY(-25*PI/180);
  
  rotateY(current_joint_angle[7]);  
  shape(ctrl_link5);
  drawLocalFrame();
  translate(200, 0, 0);
    
  rotateY(current_joint_angle[8]);
  shape(ctrl_link6);
  drawLocalFrame();
  rotateY(40*PI/180);
  translate(50, 0, 0);
  rotateY(-80*PI/180);
  translate(50, 0, 0);
  rotateY(40*PI/180);
  
  rotateY(current_joint_angle[9]);
  rotateY(30*PI/180);
  shape(ctrl_link7);
  drawLocalFrame();

  popMatrix();

  drawSphereAfterEffect();
}

/*******************************************************************************
* Draw world frame
*******************************************************************************/
void drawWorldFrame()
{
  strokeWeight(5);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 100, 0, 0);

  strokeWeight(5);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, -100, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(5);
  line(0, 0, 0, 0, 0, 100);
}

/*******************************************************************************
* Draw local frame
*******************************************************************************/
void drawLocalFrame()
{
  strokeWeight(5);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 50, 0, 0);

  strokeWeight(5);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, -50, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(5);
  line(0, 0, 0, 0, 0, 50);
}


/*******************************************************************************
* Mouse drag event
*******************************************************************************/
void mouseDragged()
{
  world_rot_x -= (mouseX - pmouseX) * 2.0;
  world_rot_y -= (mouseY - pmouseY) * 2.0;

  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0 + world_rot_x, height/2.0-500 + world_rot_y, height/2.0 * 4,
         width/2-100, height/2, 0,
         0, 1, 0);
}

/*******************************************************************************
* Mouse wheel event
*******************************************************************************/
void mouseWheel(MouseEvent event) {
  float e = event.getCount() * 0.01;
  model_scale_factor += e;
}

/*******************************************************************************
* Key press event
*******************************************************************************/
void keyPressed()
{
  if      (key == 'q') model_trans_x      -= 0.050 * 1000;
  else if (key == 'a') model_trans_x      += 0.050 * 1000;
  else if (key == 'w') model_trans_y      += 0.050 * 1000;
  else if (key == 's') model_trans_y      -= 0.050 * 1000;
  else if (key == 'e') model_trans_z      -= 0.050 * 1000;
  else if (key == 'd') model_trans_z      += 0.050 * 1000;
  else if (key == 'i') 
  {
    model_trans_x = model_trans_y = model_trans_z = model_scale_factor = world_rot_x = world_rot_y = 0.0;
    camera(width/2.0, height/2.0-500, height/2.0 * 4,
           width/2-100, height/2, 0,
           0, 1, 0);
  }
}

/*******************************************************************************
* Controller Window
*******************************************************************************/
class ChildApplet extends PApplet
{
  ControlP5 cp5;

  Textlabel headLabel;

  Knob joint0, joint1, joint2;

  float[] send_joint_angle = new float[3];
  int set_suction = 0;
  
  boolean onoff_flag = false;
  int motion_num = 0;

  public ChildApplet()
  {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings()
  {
    size(400, 600);
    smooth();
  }
  public void setup()
  {
    surface.setTitle("Control Interface");

    cp5 = new ControlP5(this);

/*******************************************************************************
* Init Tab
*******************************************************************************/
    cp5.addTab("Task Space Control")
       .setColorBackground(color(188, 188, 90))
       .setColorLabel(color(255))
       .setColorActive(color(0,128,0))
       ;

    cp5.addTab("Hand Teaching")
       .setColorBackground(color(100, 160, 0))
       .setColorLabel(color(255))
       .setColorActive(color(0,0,255))
       ;

    cp5.addTab("Motion")
       .setColorBackground(color(0, 160, 100))
       .setColorLabel(color(255))
       .setColorActive(color(0,0,255))
       ;

    cp5.getTab("default")
       .activateEvent(true)
       .setLabel("Joint Space Control")
       .setId(1)
       ;

    cp5.getTab("Task Space Control")
       .activateEvent(true)
       .setId(2)
       ;

    cp5.getTab("Hand Teaching")
       .activateEvent(true)
       .setId(3)
       ;

    cp5.getTab("Motion")
       .activateEvent(true)
       .setId(4)
       ;

/*******************************************************************************
* Init Joint Space Controller
*******************************************************************************/
    headLabel = cp5.addTextlabel("Label")
                   .setText("Controller for OpenManipulator Link")
                   .setPosition(10,20)
                   .setColorValue(0xffffff00)
                   .setFont(createFont("arial",20))
                   ;

    cp5.addToggle("Controller_OnOff")
       .setPosition(0,50)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

    joint0 = cp5.addKnob("joint0")
             .setRange(-169.0*PI/180 , 169.0*PI/180)
             .setValue(0)
             .setPosition(30,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint1 = cp5.addKnob("joint1")
             .setRange(-119.0*PI/180 , -1.0*PI/180)
             .setValue(3.14/2)
             .setPosition(150,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint2 = cp5.addKnob("joint2")
             .setRange(-169.0*PI/180 , -91.0*PI/180)
             .setValue(3.14)
             .setPosition(270,140)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    cp5.addButton("Origin")
       .setValue(0)
       .setPosition(0,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Basic")
       .setValue(0)
       .setPosition(320,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Send_Joint_Angle")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Suction_OnOff")
       .setPosition(0,520)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

/*******************************************************************************
* Init Task Space Controller
*******************************************************************************/
    cp5.addButton("Forward")
       .setValue(0)
       .setPosition(150,150)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Back")
       .setValue(0)
       .setPosition(150,350)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Left")
       .setValue(0)
       .setPosition(50,250)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Right")
       .setValue(0)
       .setPosition(250,250)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Set")
       .setValue(0)
       .setPosition(170,270)
       .setSize(60,60)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Up")
       .setValue(0)
       .setPosition(50,450)
       .setSize(100,100)
       .setFont(createFont("arial",15))
       ;

   cp5.addButton("Down")
      .setValue(0)
      .setPosition(250,450)
      .setSize(100,100)
      .setFont(createFont("arial",15))
      ;

   cp5.addToggle("Suction")
      .setPosition(165,480)
      .setSize(70,70)
      .setMode(Toggle.SWITCH)
      .setFont(createFont("arial",10))
      .setColorActive(color(196, 196, 196))
      .setColorBackground(color(255, 255, 153))
      ;

/*******************************************************************************
* Init Hand Teaching Controller
*******************************************************************************/
    cp5.addToggle("Dynamixel_OnOff")
       .setPosition(0,130)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;
       
    cp5.addButton("Motion_Clear")
       .setValue(0)
       .setPosition(0,210)
       .setSize(200,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Make_Joint_Pose")
       .setValue(0)
       .setPosition(200,210)
       .setSize(200,100)
       .setFont(createFont("arial",15))
       ;


    cp5.addButton("Motion_Start")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,80)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Motion_Repeat")
       .setPosition(0,490)
       .setSize(400,80)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

/*******************************************************************************
* Init Motion
*******************************************************************************/
    cp5.addButton("Start")
       .setValue(0)
       .setPosition(0,200)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Stop")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

/*******************************************************************************
* Set Tap UI
*******************************************************************************/
    cp5.getController("Label").moveTo("global");
    cp5.getController("Controller_OnOff").moveTo("global");

    cp5.getController("Forward").moveTo("Task Space Control");
    cp5.getController("Back").moveTo("Task Space Control");
    cp5.getController("Left").moveTo("Task Space Control");
    cp5.getController("Right").moveTo("Task Space Control");
    cp5.getController("Set").moveTo("Task Space Control");
    cp5.getController("Up").moveTo("Task Space Control");
    cp5.getController("Down").moveTo("Task Space Control");
    cp5.getController("Suction").moveTo("Task Space Control");

    cp5.getController("Dynamixel_OnOff").moveTo("Hand Teaching");
    cp5.getController("Motion_Clear").moveTo("Hand Teaching");
    cp5.getController("Make_Joint_Pose").moveTo("Hand Teaching");
    cp5.getController("Motion_Start").moveTo("Hand Teaching");
    cp5.getController("Motion_Repeat").moveTo("Hand Teaching");

    cp5.getController("Start").moveTo("Motion");
    cp5.getController("Stop").moveTo("Motion");
  }

  public void draw()
  {
    background(0);
  }

  void controlEvent(ControlEvent theControlEvent) {
    if (theControlEvent.isTab()) {
      println("got an event from tab : "+theControlEvent.getTab().getName()+" with id "+theControlEvent.getTab().getId());
      tabFlag = theControlEvent.getTab().getId();
    }
  }

/*******************************************************************************
* Init Function of Joint Space Controller
*******************************************************************************/
  void Controller_OnOff(boolean flag)
  {
    onoff_flag = flag;
    if (onoff_flag)
    {
      joint0.setValue(current_joint_angle[0]);
      joint1.setValue(current_joint_angle[1]);
      joint2.setValue(current_joint_angle[2]);

      opencr_port.write("om"   + ',' +
                        "ready" + '\n');
      println("OpenManipulator Link Ready!!!");
    }
    else
    {
      opencr_port.write("om"  + ',' +
                        "end"  + '\n');
      println("OpenManipulator Link End...");
    }
    println(onoff_flag);
  }

  void joint0(float angle)
  {
    joint_angle[0] = angle;   
    passiveJointSet();
  }

  void joint1(float angle)
  {
    joint_angle[1] = angle;
    passiveJointSet();
  }

  void joint2(float angle)
  {
    joint_angle[2] = angle;
    passiveJointSet();
  }

  public void Origin(int theValue)
  {
    if (onoff_flag)
    {
      send_joint_angle[0] = 0.0;
      send_joint_angle[1] = -90.0*PI/180.0;
      send_joint_angle[2] = -160.0*PI/180.0;
      
      for(int i=0; i<3; i++)
      {
        joint_angle[i] = send_joint_angle[i];
      }
      passiveJointSet();
            
      opencr_port.write("joint"            + ',' +
                        send_joint_angle[0] + ',' +
                        send_joint_angle[1] + ',' +
                        send_joint_angle[2] + '\n');
      println("send joint " + send_joint_angle[0] + ", " + send_joint_angle[1]  + ", " + send_joint_angle[2]  + "  ");                    
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Basic(int theValue)
  {
    if (onoff_flag)
    {
      send_joint_angle[0] = 0.0;
      send_joint_angle[1] = -90.0*PI/180.0;
      send_joint_angle[2] = -135.0*PI/180.0;
      
      for(int i=0; i<3; i++)
      {
        joint_angle[i] = send_joint_angle[i];
      }
      passiveJointSet();
      
      opencr_port.write("joint"            + ',' +
                        send_joint_angle[0] + ',' +
                        send_joint_angle[1] + ',' +
                        send_joint_angle[2] + '\n');
      println("send joint " + send_joint_angle[0] + ", " + send_joint_angle[1]  + ", " + send_joint_angle[2]  + "  ");  
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Send_Joint_Angle(int theValue)
  {
    if (onoff_flag)
    {
      for(int i=0; i<3; i++)
      {
      send_joint_angle[i] = joint_angle[i];
      }

      opencr_port.write("joint"            + ',' +
                        send_joint_angle[0] + ',' +
                        send_joint_angle[1] + ',' +
                        send_joint_angle[2] + '\n');
      println("send joint " + send_joint_angle[0] + ", " + send_joint_angle[1]  + ", " + send_joint_angle[2]  + "  ");  
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Suction_OnOff(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("suction"  + ',' +
                          "on" + '\n');
        println("send suction on");
      }
      else
      {
        opencr_port.write("suction"  + ',' +
                          "off" + '\n');
        println("send suction off");
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

/*******************************************************************************
* Init Function of Task Space Controller
*******************************************************************************/
  public void Forward(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "forward" + '\n');
      println("send Move Forward");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Back(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "back"    + '\n');
      println("send Move Back");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Left(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "left"    + '\n');
      println("send Move Left");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Right(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "right"   + '\n');
      println("send Move Right");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Up(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "up"      + '\n');
      println("send Move Up");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Down(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("task"    + ',' +
                        "down"    + '\n');
      println("send Move Down");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Set(int theValue)
  {
    if (onoff_flag)
    {
      send_joint_angle[0] = 0.0;
      send_joint_angle[1] = -90.0  * PI/180.0;
      send_joint_angle[2] = -160.0 * PI/180.0;
      
      for(int i=0; i<3; i++)
      {
        current_joint_angle[i] = send_joint_angle[i];
      }
      passiveJointSet();

      opencr_port.write("joint"            + ',' +
                        send_joint_angle[0] + ',' +
                        send_joint_angle[1] + ',' +
                        send_joint_angle[2] + '\n');
      println("send joint " + send_joint_angle[0] + ", " + send_joint_angle[1]  + ", " + send_joint_angle[2]  + "  ");  
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Suction(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("suction"  + ',' +
                          "on" + '\n');
        println("send suction on");
      }
      else
      {
        opencr_port.write("suction"  + ',' +
                          "off" + '\n');
        println("send suction on");
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

/*******************************************************************************
* Init Function of Hand Teaching Controller
*******************************************************************************/
  void Dynamixel_OnOff(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("motor"  + ',' +
                          "disable"     + '\n');
        println("send motor disable");
      }
      else
      {
        opencr_port.write("motor"  + ',' +
                          "enable"      + '\n');
        println("send motor enable");
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }
  
  public void Motion_Clear(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("get" + ',' +
                        "clear"  + '\n');
      println("send get clear");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Make_Joint_Pose(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("get"      + ',' +
                        "pose"     + ',' +
                        motion_num + '\n');
      println("send get pose");
      motion_num++;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Motion_Start(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("hand"     + ',' +
                        "once"  + '\n');
      println("send Motion Start!!!");

      motion_num = 0;
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Motion_Repeat(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        opencr_port.write("hand"    + ',' +
                          "repeat"  + '\n');
        println("send hand repeat");
      }
      else
      {
        opencr_port.write("hand"  + ',' +
                          "stop"  + '\n');
        println("send hand stop");
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Start(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("motion"  + ',' +
                        "start"   + '\n');
      println("send Motion Start!!!");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Stop(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("motion"  + ',' +
                        "stop"    + '\n');
      println("send Motion Stop!!!");
    }
    else
    {
      println("Please, Set On Controller");
    }
  }
}
