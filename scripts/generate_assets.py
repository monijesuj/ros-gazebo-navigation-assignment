from pathlib import Path
import json, math
from xml.etree import ElementTree as E
root=Path(__file__).resolve().parents[1]
p=root/'src/robot_demo'
obstacles=[{'name':'west_block','x':-2.,'y':-2.,'sx':1.2,'sy':2.8,'sz':1.2}, {'name':'middle_block','x':0.,'y':1.,'sx':1.8,'sy':2.,'sz':1.3}, {'name':'east_block','x':2.8,'y':0.,'sx':1.2,'sy':3.5,'sz':1.1}]
config={'bounds':[-7.,7.,-5.,5.],'resolution':0.1,'robot_radius':0.39,'safety_margin':0.25,'start':[-5.,-3.], 'datum':[55.75,37.62,0.], 'waypoint_mode':'gps','waypoints_world':[[5.,3.]],'obstacles':obstacles,'max_speed':0.32,'goal_tolerance':0.20}
# WGS84 tangent-plane radii, shared with navigation module.
lat=math.radians(config['datum'][0]); a=6378137.; e2=6.69437999014e-3
n=a/math.sqrt(1-e2*math.sin(lat)**2); m=a*(1-e2)/(1-e2*math.sin(lat)**2)**1.5
config['waypoints_gps']=[[55.75+math.degrees(3/m),37.62+math.degrees(5/(n*math.cos(lat)))]]
if (p/'config/mission.json').exists():
 config=json.loads((p/'config/mission.json').read_text())
 obstacles=config['obstacles']
else:
 (p/'config/mission.json').write_text(json.dumps(config,indent=2)+'\n')
robot=E.Element('robot',name='rover')
def tag(parent,element_name,text=None,**attrs):
 el=E.SubElement(parent,element_name,{k:str(v) for k,v in attrs.items()})
 if text is not None: el.text=str(text)
 return el
materials={'blue':'0.08 0.35 0.72 1','black':'0.12 0.14 0.17 1','green':'0.1 0.8 0.4 1'}
for name,rgba in materials.items(): tag(tag(robot,'material',name=name),'color',rgba=rgba)
def link(name,shape,size,mass,color,visual_xyz='0 0 0',rpy='0 0 0',collision=True):
 l=tag(robot,'link',name=name)
 inertial=tag(l,'inertial'); tag(inertial,'mass',value=mass)
 tag(inertial,'inertia',ixx=max(.0001,mass*.01),ixy=0,ixz=0,iyy=max(.0001,mass*.015),iyz=0,izz=max(.0001,mass*.02))
 for kind in ('visual','collision') if collision else ('visual',):
  v=tag(l,kind);tag(v,'origin',xyz=visual_xyz,rpy=rpy);g=tag(v,'geometry')
  if shape=='box': tag(g,'box',size=size)
  elif shape=='cylinder':tag(g,'cylinder',radius=size[0],length=size[1])
  elif shape=='sphere':tag(g,'sphere',radius=size)
  if kind=='visual':tag(v,'material',name=color)
 gz=tag(robot,'gazebo',reference=name);tag(gz,'material',{'blue':'Gazebo/Blue','black':'Gazebo/Black','green':'Gazebo/Green'}[color])
 return l
link('base_link','box','0.55 0.40 0.20',8,'blue')
for side,y in [('left',.26),('right',-.26)]:
 link(side+'_wheel','cylinder',(.12,.06),.6,'black',rpy='1.57079632679 0 0')
 j=tag(robot,'joint',name=side+'_wheel_joint',type='continuous');tag(j,'parent',link='base_link');tag(j,'child',link=side+'_wheel');tag(j,'origin',xyz=f'0 {y} -0.08');tag(j,'axis',xyz='0 1 0');tag(j,'dynamics',damping=.01,friction=.01)
 gz=tag(robot,'gazebo',reference=side+'_wheel');tag(gz,'mu1',1.0);tag(gz,'mu2',1.0)
for name,x in [('front_caster',.22),('rear_caster',-.22)]:
 link(name,'sphere',.04,.1,'black')
 j=tag(robot,'joint',name=name+'_joint',type='fixed');tag(j,'parent',link='base_link');tag(j,'child',link=name);tag(j,'origin',xyz=f'{x} 0 -0.16')
 gz=tag(robot,'gazebo',reference=name);tag(gz,'mu1',0.);tag(gz,'mu2',0.)
for name,shape,size,xyz in [('lidar_link','cylinder',(.055,.06),'0 0 0.15'),('camera_link','box','0.04 0.14 0.04','0.29 0 0.08'),('gps_link','box','0.04 0.04 0.02','0 0 0.12')]:
 link(name,shape,size,.05,'green',collision=False)
 j=tag(robot,'joint',name=name+'_joint',type='fixed');tag(j,'parent',link='base_link');tag(j,'child',link=name);tag(j,'origin',xyz=xyz)
 # Preserve the sensor link during URDF->SDF fixed-joint reduction.
 gz=tag(robot,'gazebo',reference=name+'_joint');tag(gz,'preserveFixedJoint','true')
tag(robot,'link',name='camera_optical_link')
j=tag(robot,'joint',name='camera_optical_joint',type='fixed');tag(j,'parent',link='camera_link');tag(j,'child',link='camera_optical_link');tag(j,'origin',xyz='0 0 0',rpy='-1.57079632679 0 -1.57079632679')
gz=tag(robot,'gazebo',reference='lidar_link');sensor=tag(gz,'sensor',name='lidar',type='gpu_lidar');tag(sensor,'always_on','true');tag(sensor,'update_rate',10);tag(sensor,'topic','/scan');tag(sensor,'visualize','false')
lidar=tag(sensor,'lidar');scan=tag(lidar,'scan');h=tag(scan,'horizontal');tag(h,'samples',360);tag(h,'resolution',1);tag(h,'min_angle',-math.pi);tag(h,'max_angle',math.pi)
r=tag(lidar,'range');tag(r,'min',.12);tag(r,'max',12);tag(r,'resolution',.01)
gz=tag(robot,'gazebo',reference='camera_link');sensor=tag(gz,'sensor',name='astra',type='rgbd_camera');tag(sensor,'always_on','true');tag(sensor,'update_rate',10);tag(sensor,'topic','/camera');camera=tag(sensor,'camera');tag(camera,'horizontal_fov',1.0472);im=tag(camera,'image');tag(im,'width',320);tag(im,'height',240);clip=tag(camera,'clip');tag(clip,'near',.15);tag(clip,'far',6)
gz=tag(robot,'gazebo',reference='gps_link');sensor=tag(gz,'sensor',name='gps',type='navsat');tag(sensor,'always_on','true');tag(sensor,'update_rate',5);tag(sensor,'topic','/gps/fix')
gz=tag(robot,'gazebo');plugin=tag(gz,'plugin',filename='ignition-gazebo-diff-drive-system',name='ignition::gazebo::systems::DiffDrive')
for k,v in [('left_joint','left_wheel_joint'),('right_joint','right_wheel_joint'),('wheel_separation',.52),('wheel_radius',.12),('topic','/cmd_vel'),('odom_topic','/odom'),('tf_topic','/tf'),('frame_id','odom'),('child_frame_id','base_link'),('odom_publish_frequency',30),('max_linear_acceleration',.5),('max_angular_acceleration',1.5)]:tag(plugin,k,v)
plugin=tag(gz,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher');tag(plugin,'topic','/joint_states');tag(plugin,'joint_name','left_wheel_joint');tag(plugin,'joint_name','right_wheel_joint')
for contact_link, collisions in [('base_link',['base_link_collision','base_link_fixed_joint_lump__front_caster_collision_1','base_link_fixed_joint_lump__rear_caster_collision_2']),('left_wheel',['left_wheel_collision']),('right_wheel',['right_wheel_collision'])]:
 gz=tag(robot,'gazebo',reference=contact_link); sensor=tag(gz,'sensor',name=contact_link+'_contact',type='contact'); tag(sensor,'always_on','true'); tag(sensor,'update_rate',30); contact=tag(sensor,'contact')
 for collision in collisions: tag(contact,'collision',collision)
 tag(contact,'topic','/contacts/'+contact_link)
E.indent(robot);(p/'urdf/rover.urdf').write_text(E.tostring(robot,encoding='unicode'))
sdf=E.Element('sdf',version='1.9');world=tag(sdf,'world',name='navigation')
physics=tag(world,'physics',name='default',type='ignored');tag(physics,'max_step_size',.005);tag(physics,'real_time_factor',1.0)
for file,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('navsat','NavSat'),('contact','Contact')]:
 plugin=tag(world,'plugin',filename=f'ignition-gazebo-{file}-system',name=f'ignition::gazebo::systems::{name}')
 if name=='Sensors':tag(plugin,'render_engine','ogre2')
coords=tag(world,'spherical_coordinates')
for k,v in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),('latitude_deg',config['datum'][0]),('longitude_deg',config['datum'][1]),('elevation',config['datum'][2]),('heading_deg',0)]:tag(coords,k,v)
scene=tag(world,'scene');tag(scene,'ambient','0.6 0.6 0.6 1');tag(scene,'background','0.88 0.91 0.94 1');tag(scene,'shadows','false')
light=tag(world,'light',name='sun',type='directional');tag(light,'pose','0 0 10 0 0 0');tag(light,'diffuse','0.9 0.9 0.9 1');tag(light,'direction','-0.4 -0.3 -1');tag(light,'cast_shadows','false')
def box(name,x,y,z,sx,sy,sz,color,collision=True):
 model=tag(world,'model',name=name);tag(model,'static','true');tag(model,'pose',f'{x} {y} {z} 0 0 0');l=tag(model,'link',name='link')
 for kind in ('visual','collision') if collision else ('visual',):
  el=tag(l,kind,name=kind);g=tag(el,'geometry');tag(tag(g,'box'),'size',f'{sx} {sy} {sz}')
  if kind=='visual':mat=tag(el,'material');tag(mat,'ambient',color);tag(mat,'diffuse',color)
box('ground',0,0,-.05,14,10,.1,'0.77 0.79 0.82 1')
for o in obstacles:box(o['name'],o['x'],o['y'],o['sz']/2,o['sx'],o['sy'],o['sz'],'0.86 0.36 0.14 1')
box('start_A',*config['start'],.005,1.2,1.2,.01,'0.12 0.7 0.3 1',False);box('finish_B',*config['waypoints_world'][-1],.005,1.2,1.2,.01,'0.1 0.45 0.95 1',False)
inc=tag(world,'include');tag(inc,'uri','model://rover');tag(inc,'name','rover');tag(inc,'pose',f"{config['start'][0]} {config['start'][1]} 0.205 0 0 0")
# A stable overhead camera for the recorded GUI.
gui=tag(world,'gui',fullscreen='0');plugin=tag(gui,'plugin',filename='MinimalScene',name='3D View');gzgui=tag(plugin,'ignition-gui');tag(gzgui,'title','3D View');tag(gzgui,'property','false',type='bool',key='showTitleBar');tag(gzgui,'property','docked',type='string',key='state');tag(plugin,'engine','ogre2');tag(plugin,'scene','scene');tag(plugin,'ambient_light','0.6 0.6 0.6');tag(plugin,'background_color','0.88 0.91 0.94');tag(plugin,'camera_pose','0 -9 11 0 0.85 1.5708')
for filename,name in [('GzSceneManager','Scene Manager'),('InteractiveViewControl','Interactive View'),('WorldControl','World Control')]:
 plugin=tag(gui,'plugin',filename=filename,name=name); props=tag(plugin,'ignition-gui')
 for key,type_,value in [('resizable','bool','false'),('showTitleBar','bool','false'),('state','string','floating'),('width','double',5),('height','double',5)]:tag(props,'property',value,key=key,type=type_)
 if filename=='WorldControl':tag(plugin,'start_paused','false')
E.indent(sdf);(p/'worlds/navigation.sdf').write_text(E.tostring(sdf,encoding='unicode'))

import subprocess
with (p/"models/rover/model.sdf").open("w") as output:
 subprocess.run(["ign","sdf","-p",str(p/"urdf/rover.urdf")],check=True,stdout=output)
