import tensorflow as tf
import tensorflow_datasets as tfds
import rlds
from rlds import transformations
from rlds import rlds_types
import tf_agents
from tf_agents.policies import py_tf_eager_policy
from tf_agents.trajectories import time_step as ts
from IPython import display
from PIL import Image
import numpy as np
import math
import time
import datetime
import json
import camera_snapshot
from widowx import WidowX
import copy

#########################################################################
# Functions extracted from DeepMind sample code:
# robotics_open_x_embodiment_and_rt_x_oss_Minimal_example_for_running_inference_using_RT_1_X_TF_using_tensorflow_datasets.ipynb
#########################################################################
def as_gif(images, rbt=False):
  # Render the images as the gif:
  if rbt:
    filenm = '/tmp/temprbt.gif'
  else:
    filenm = '/tmp/temp.gif'

  images[0].save(filenm, save_all=True, append_images=images[1:], duration=1000, loop=0)
  gif_bytes = open(filenm,'rb').read()
  return gif_bytes

def resize(image):
    image = tf.image.resize_with_pad(image, target_width=320, target_height=256)
    image = tf.cast(image, tf.uint8)
    return image

#########################################################################
##########################
# Code shared with joystick & RT1 script (by copying)
##########################
# a cross-over interface between joystick & widowx.py, deals with move_mode
class widowx_client():

  def __init__(self):
      self.move_mode = 'Absolute'
      self.gripper_fully_open_closed = None

  def move(self, robot_arm, vx, vy, vz, vg, vr, goc):
        initial_time = robot_arm.millis()
        if (vr and self.move_mode != 'Absolute'):
            robot_arm.moveServoWithSpeed(robot_arm.IDX_ROT, vr, initial_time)
        if (vx or vy or vz or vg):
          if (self.move_mode == 'Relative'):
            fvx = min(max(-1.75, (float(vx) / 127.0 * 1.75)), 1.75)
            fvy = min(max(-1.75, (float(vy) / 127.0 * 1.75)), 1.75)
            fvz = min(max(-1.75, (float(vz) / 127.0 * 1.75)), 1.75)
            fvg = min(max(-1.4, (float(vg) / 255.0 * 1.4)), 1.4)
            fvr = min(max(-1.4, (float(vr) / 255.0 * 1.4)), 1.4)
            robot_arm.movePointWithSpeed(fvx, fvy, fvz, fvg, initial_time)
            gripper_fully_open_closed = robot_arm.moveGrip(goc)
            wdw.set_move_mode('Relative')
            if vr != None:
              rot = fvr + robot_arm.getServoAngle(robot_arm.IDX_ROT)
              print("move vr:", fvr, robot_arm.current_angle[robot_arm.IDX_ROT], rot)
              try:
                robot_arm.moveServo2Angle(robot_arm.IDX_ROT, rot)  # servo id 5 / idx 4: rotate gripper to angle
              except:
                pass
            if goc != None:
              robot_arm.openCloseGrip(goc)
          elif (self.move_mode == 'Absolute'):
            robot_arm.moveArmGammaController(vx, vy, vz, vg)
            angle = (float(vr) / 256.0) * math.pi
            robot_arm.moveServoWithSpeed(robot_arm.IDX_ROT, angle, initial_time)
            gripper_fully_open_closed = robot_arm.openCloseGrip(goc)
          # elif (moveOption == USER_FRIENDLY):
            # used by joystick controller only
            # moveArmWithSpeed(vx, vy, vz, vg, initial_time)

  def set_move_mode(self, mode):
        # AKA:      absolute point         relative point
        if mode != 'Absolute' and mode != 'Relative':
          print("illegal move mode:", mode)
          return [True, None]
        if self.move_mode != mode:
          self.move_mode = mode
          return [True, None]
        return [True, None]
##########################
###################################################3
def read_config():
    with open('rt1_widowx_config.json') as config_file:
      config_json = config_file.read()
    config = json.loads(config_json)
    return config

def get_initial_state(config):
    init_state = json.loads(config["initial_state"])
    print("init_state", init_state)
    return init_state

###################################################3
# Start code for dataset trajectory example derived from:
# robotics_open_x_embodiment_and_rt_x_oss_Minimal_example_for_running_inference_using_RT_1_X_TF_using_tensorflow_datasets.ipynb
# The original code uses the dataset to get one episode of images
# and state to get "gt - ground truth".  These images and state are
# then run with the model to get "predicted actions". No real robot
# arm was required.
#
# For this application, we run predicted actions, get images and states 
# using a real robot.
###################################################3
# some globals to make some of the below code easier to understand
x = 0
y = 1
z = 2
config = read_config()
saved_model_path = config["saved_model_path"]
run_tf = True
#######################################

# Load TF model checkpoint
if run_tf:
  gpus = tf.config.list_physical_devices('GPU')
  if gpus:
    # Restrict TensorFlow to only use the first GPU
    try:
      tf.config.set_visible_devices(gpus[0], 'GPU')
      logical_gpus = tf.config.list_logical_devices('GPU')
      print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPU")
    except RuntimeError as e:
      # Visible devices must be set before GPUs have been initialized
      print(e)
else:
  tf.config.experimental.set_memory_growth
  print("memory growth set")

##########################################
# Perform one step of inference using dummy input
##########################################
tfa_policy = py_tf_eager_policy.SavedModelPyTFEagerPolicy(
      model_path=saved_model_path,
      load_specs_from_pbtxt=True,
      use_tf_function=True)

# Obtain a dummy observation, where the features are all 0
observation = tf_agents.specs.zero_spec_nest(tf_agents.specs.from_spec(tfa_policy.time_step_spec.observation))

# Construct a tf_agents time_step from the dummy observation
tfa_time_step = ts.transition(observation, reward=np.zeros((), dtype=np.float32))

# Initialize the state of the policy
policy_state = tfa_policy.get_initial_state(batch_size=1)

# Run inference using the policy
action = tfa_policy.action(tfa_time_step, policy_state)

#######################################
# Move to initial positions and then take snapshot image
#######################################
robot_camera = camera_snapshot.CameraSnapshot()
time.sleep(2)
# first snapshot isn't properly tuned; take snapshot & throw away.
im, im_file, im_time = robot_camera.snapshot(True)
robot_images = [] # init robot arm
# robot_arm =  widowx.WidowX()
robot_arm =  WidowX()
# initialize by starting from Rest Position
robot_arm.moveRest()
wdw = widowx_client()
wdw.set_move_mode('Absolute')

#########################################################
# Move to Initial Arm Position as taken from config file
#########################################################

state = get_initial_state(config)
print("initial config state:", state)
# [-127,127] for vx, vy and vz and [-255,255] for vg
# 41cm horizontal reach and 55cm verticle
# Values not normalized: already has the 127/255 factored in 
px = state["x"]
py = state["y"]
pz = state["z"]
pg = state["gamma"] 
pq5 = state["rot"] 
gripper_open = state["gripper"] 
wdw.set_move_mode('Relative')
wdw.move(robot_arm, px, py, pz, pg, pq5, gripper_open)
im, im_file, im_time = robot_camera.snapshot(True)
robot_image = Image.fromarray(np.array(im))
robot_images.append(im)
s = []   # state history

#########################################################
# Run as many steps as necessary
#########################################################
while True:
  #########################################################
  # set up input to Run Model to get predicted action
  #########################################################
  # BridgeData V2 dataset description:
  # step: <_VariantDataset element_spec= {
  # 'action': TensorSpec(shape=(7,), dtype=tf .float32, name=None),
  # 'discount': TensorSpec(shape=(), dtype=tf.float32, name=None),
  # 'is_first': TensorSpec(shape=(), dtype=tf.bool, name=None),
  # 'is_last': TensorSpec(shape=(), dtype=tf.bool, name=None),
  # 'is_terminal': TensorSpec(shape=(), dtype=tf.bool, name=None),
  # 'language_embedding': TensorSpec(shape=(512,), dtype= tf.float32, name=None),
  # 'language_instruction': TensorSpec(shape=(), dtype=tf.string, name=None),
  # 'observation':
  #    {'image_0': TensorSpec(shape=(256, 256, 3), dty pe=tf.uint8, name=None),
  #    'image_1': TensorSpec(shape=(256, 256, 3), dtype=tf.uin t8, name=None),
  #    'image_2': TensorSpec(shape=(256, 256, 3), dtype=tf.uint8, name= None),
  #    'image_3': TensorSpec(shape=(256, 256, 3), dtype=tf.uint8, name=None),
  #    'state': TensorSpec(shape=(7,), dtype=tf.float32, name=None)},
  # 'reward': TensorSpec(shape=(), dtype=tf.float32, name=None)}>
    
  predicted_actions = []
  print("instr:", config['language_instruction'])
  observation['natural_language_instruction'] = config['language_instruction']
  observation['image'] = robot_image

  # observation state for dataset
  s.append(copy.deepcopy(robot_arm.state))

  # ts is timestep
  tfa_time_step = ts.transition(observation, reward=np.zeros((), dtype=np.float32))
  
  #############################################################
  # Run inference using the policy rt_1_x
  #############################################################
  policy_step = tfa_policy.action(tfa_time_step, policy_state)
  
  ####################
  # result from rt_1_x
  robot_action = policy_step.action
  predicted_actions.append(robot_action)
  print("action:",robot_action)
  # ts is timestep class
  tfa_time_step = ts.transition(observation, reward=np.zeros((), dtype=np.float32))
  
  policy_state = policy_step.state
  
  wv = robot_action['world_vector']
  euler = robot_action['rotation_delta']
  gripper_action = robot_action['gripper_closedness_action']
  if gripper_action < 0:
    gripper_open = True
  else:
    gripper_open = False
  print("wv:", wv)
  
  ############################################################
  # Rescale the predicted action for the Bridge Dataset 
  # and Widowx 250 arm. 
  ############################################################
  # denormalize action 
  # widowx has 41cm horizontal reach and 55cm verticle (up).
  # We use the Bridge dataset scale factor even with the different
  # arm sizes.
  #
  # Per the paper, they scale each action dimension to range -1 and 1 
  # and use a vocabulary size of 256 to tokenize the actions.
  #
  # The World Vector is the center of the gripper.
  # The action represents the change in the World Vector.
  # The arm action moves to follow the change in world vector.
  #
  # From the minimal example from DeepMind, the Bridge Dataset
  # is rescaled as follows:
  # -1.76 < resc_wv_actions = ((actions + 0.05) * 3.5 / .1 - 1.75) < 1.76
  # -1.41  < resc_rot_actions = ((actions + .25) * 5.6 - 1.4) < 1.41
  #
  # When communicating with the WidowX, the above values are encoded:
  # -127 <= resc_wv_action <= 127  
  # -255 <= resc_rot_action <= 255  
  #
  # The Bridge rescale wv and rot rescale values (1.75 & 1.4) are 
  # known by the Controller to decode the wv/rot actions.

  # step 1: rescale to bridge dataset values
  vx = min(max(-1.75, ((wv[x] + 0.05) * 35 - 1.75)), 1.75)
  vy = min(max(-1.75, ((wv[y] + 0.05) * 35 - 1.75)), 1.75)
  vz = min(max(-1.75, ((wv[z] + 0.05) * 35 - 1.75)), 1.75)
  vg = min(max(-1.4, ((euler[0] + 0.25) * 5.6 - 1.4)), 1.4)
  vq5 = min(max(-1.4, ((euler[1] + 0.25) * 5.6 - 1.4)), 1.4)
  print("scale 1:", vx,vy,vz,vg,vq5)

  # step 2: Encode to [-127, 127] and [-255, 255]
  # vx = min(max(-127, round(vx * 127.0 / 1.75)), 127)
  # vy = min(max(-127, round(vy * 127.0 / 1.75)), 127)
  # vz = min(max(-127, round(vz * 127.0 / 1.75)), 127)
  # vg = min(max(-255, round(vg * 255.0 / 1.4)), 255)
  # vq5 = min(max(-255, round(vq5 * 255.0 / 1.4)), 255)  # rot
  limPi_2 = 181 * math.pi / 360
  lim5Pi_6 = 5 * math.pi / 6;
  # q2Lim[] = {-limPi_2, limPi_2};
  # q3Lim[] = {-limPi_2, lim5Pi_6};
  # q4Lim[] = {-11 * M_PI / 18, limPi_2};

  XYZ_SCALE = 40.0
  vx = min(max(-XYZ_SCALE, round(vx * XYZ_SCALE / 1.75)), XYZ_SCALE)
  vy = min(max(-XYZ_SCALE, round(vy * XYZ_SCALE / 1.75)), XYZ_SCALE)
  vz = min(max(-XYZ_SCALE, round(vz * XYZ_SCALE / 1.75)), XYZ_SCALE)
  vg = min(max(-limPi_2, round(vg * 255.0 / 1.4)), lim5Pi_6)
  # vr = min(max(-limPi_2, round(vg * 255.0 / 1.4)), limPi_2)
  vq5 = min(max(-limPi_2, round(vq5 * 255.0 / 1.4)), limPi_2)  # rot

  print("scale 2:", vx,vy,vz,vg,vq5)

  ############################################################
  # Move the robot based on predicted action and take snapshot
  ############################################################
  # [success, err_msg] = robot_arm.move(vx, vy, vz, vg, vq5, gripper_open)
  # if not success:
    # print("Bad start point", px, py, pz, pg, pq5, gripper_open)
    # print("err_msg:", err_msg)
    # exit()
  wdw.move(robot_arm, vx, vy, vz, vg, vq5, gripper_open)
  
  im, im_file, im_time = robot_camera.snapshot(True)
  robot_image = Image.fromarray(np.array(im))
  robot_images.append(robot_image)
  for n,st in enumerate(s):
    print(n,st)
  
  observation = tf_agents.specs.zero_spec_nest(tf_agents.specs.from_spec(tfa_policy.time_step_spec.observation))
  tfa_time_step = ts.transition(observation, reward=np.zeros((), dtype=np.float32))

  print('is_terminal:', robot_action['terminate_episode'])
  is_term = robot_action['terminate_episode']
  if is_term[0] == 1:
    break

display.Image(as_gif(robot_images, True))
print("sleeping")
time.sleep(180)

##################################
