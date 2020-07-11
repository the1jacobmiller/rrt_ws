#! /usr/bin/env python3

# Software License Agreement (Proprietary and Confidential)
#
# Copyright (c) 2018, Dataspeed Inc.
# All rights reserved.
#
# NOTICE:  All information contained herein is, and remains the property of
# Dataspeed Inc. The intellectual and technical concepts contained herein are
# proprietary to Dataspeed Inc. and may be covered by U.S. and Foreign Patents,
# patents in process, and are protected by trade secret or copyright law.
# Dissemination of this information or reproduction of this material is strictly
# forbidden unless prior written permission is obtained from Dataspeed Inc.

import rospy
import roslaunch
import math
import re
import os
import shlex
import sys
import xacro
import yaml
import rospkg
import tempfile
import numbers
from io import StringIO
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyRequest
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter


gear_map = {'P': '1', 'R': '2', 'N': '3', 'D': '4', 'L': '5'}

class MkzSpawner:
    def __init__(self):
        rospy.init_node('vehicle_spawner')
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.temp_dae_files = []

        self.state_pub_nodes = []
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.model_year_table = {
            #           13              14              15              16              17              18              19
            'mkz':      ['mkz_13',      'mkz_13',       'mkz_13',       'mkz_13',       'mkz_17',       'mkz_17',       'mkz_17'],
            'fusion':   ['fusion_13',   'fusion_13',    'fusion_13',    'fusion_13',    'fusion_13',    'fusion_13',    'fusion_13'],
            'mondeo':   ['mondeo_15',   'mondeo_15',    'mondeo_15',    'mondeo_15',    'mondeo_15',    'mondeo_15',    'mondeo_15'],
            'f150':     ['f150_14',     'f150_14',      'f150_14',      'f150_14',      'f150_14',      'f150_14',      'f150_14'],
            'pacifica': ['pacifica_18', 'pacifica_18',  'pacifica_18',  'pacifica_18',  'pacifica_18',  'pacifica_18',  'pacifica_18'],
            'jeep':     ['jeep_18',     'jeep_18',      'jeep_18',      'jeep_18',      'jeep_18',      'jeep_18',      'jeep_18']
        }

        self.description_pkg_map = {
            'mkz': rospkg.RosPack().get_path('dbw_mkz_description'),
            'fusion': rospkg.RosPack().get_path('dbw_mkz_description'),
        }

        self.first_person_offsets = {
            'mkz':      {'x_offset': 1.9, 'y_offset': 0.25, 'z_offset': 1.4, 'angle': 12.6},
            'fusion':   {'x_offset': 1.9, 'y_offset': 0.25, 'z_offset': 1.4, 'angle': 12.6},
        }

    # Parse URDF file with xacro
    def parse_xacro(self, temp_filename, urdf_file, structure_urdf, car_model, car_year, topic_ns, tf_prefix, pub_tf, pub_odom, start_gear, ref_lat, ref_lon):
        # Construct xacro argument string
        arg_str = 'placeholder '  # First argv argument is usually the program path

        # --inorder flag only needed in Kinetic
        if os.environ['ROS_DISTRO'] == 'kinetic':
            arg_str += '--inorder '
        arg_str += urdf_file
        arg_str += ' dae_file:=' + temp_filename           # Name of the temporary DAE file for the body mesh
        arg_str += ' start_gear:=' + gear_map[start_gear]  # Transmission gear to initialize with in simulation
        arg_str += ' structure_urdf:=' + structure_urdf    # Name of the particular URDF model to load
        arg_str += ' car_year:=' + str(car_year)           # Year of car being simulated
        arg_str += ' car_model:=' + car_model              # Model of car being simulated
        if len(topic_ns) > 0:
            arg_str += ' topic_ns:=' + topic_ns            # Namespace for ROS topics
        if len(tf_prefix) > 0:
            arg_str += ' tf_prefix:=' + tf_prefix          # Prefix for TF tree
        arg_str += ' production_ref_lat:=' + str(ref_lat)  # Reference GPS coordinates for the production GPS messages
        arg_str += ' production_ref_lon:=' + str(ref_lon)  # Reference GPS coordinates for the production GPS messages
        if pub_tf:                                         # Publish a perfectly accurate transform from world-->footprint
            arg_str += ' pub_tf:=1'
        else:
            arg_str += ' pub_tf:=0'
        if pub_odom:
            arg_str += ' pub_odom:=1'
        else:
            arg_str += ' pub_odom:=0'


        # Make arg_str look like an argv array and set actual argv to it
        sys.argv = shlex.split(arg_str)

        # Redirect stdout from xacro to a StringIO object
        old_std_out = sys.stdout
        sys.stdout = xml_output = StringIO()
        xacro.main()
        sys.stdout = old_std_out
        return xml_output.getvalue()

    # Write a temporary DAE mesh file for specified body color
    def write_temp_dae(self, r, g, b, car_model='mkz', urdf_model='mkz_13'):
        mesh_path = self.description_pkg_map[car_model] + '/meshes/' + urdf_model
        return mesh_path + '/body.dae'
        with open(mesh_path + '/body.dae', 'r') as f:
            # Read text from original dae file
            data = f.read()

            # Regex substitution for carpaint material
            replacement_data = re.sub(r'(<effect id="carpaint-effect">[\S\s]*?<diffuse>[\S\s]*?\n\s*)(.*)',
                         r'\1' + ('<color sid="diffuse">' + str(r) + ' ' + str(g) + ' ' + str(b) + ' 1.0</color>'), data)

            # Regex substitution for windshield texture file path
            windshield_texture_path = mesh_path + '/windshield_texture.png'
            replacement_data = re.sub(r'(<init_from>windshield_texture\.png</init_from>)', '<init_from>..' + windshield_texture_path + '</init_from>', replacement_data)

            # Regex substitution for back window texture file path
            back_window_texture_path = mesh_path + '/back_window_texture.png'
            replacement_data = re.sub(r'(<init_from>back_window_texture\.png</init_from>)', '<init_from>..' + back_window_texture_path + '</init_from>', replacement_data)

            # Regex substitution for back window texture file path
            hood_texture_path = mesh_path + '/hood_logo_texture.png'
            replacement_data = re.sub(r'(<init_from>hood_logo_texture\.png</init_from>)', '<init_from>..' + hood_texture_path + '</init_from>', replacement_data)

            # If regex substitution successful, write a temporary dae file
            if replacement_data:
                self.temp_dae_files.append(tempfile.NamedTemporaryFile(suffix='.dae', delete=False))
                self.temp_dae_files[-1].write(replacement_data)
                self.temp_dae_files[-1].close()
                f.close()
                return self.temp_dae_files[-1].name

            f.close()
        return ''

    # Spawn robot_state_publisher node for given robot model
    def run_state_pub(self, topic_ns, tf_prefix):
        if len(tf_prefix) > 0:
            pub_args = (' _tf_prefix:=' + tf_prefix)
        else:
            pub_args = ''
        new_node = roslaunch.core.Node('robot_state_publisher', 'robot_state_publisher',
                                       name='state_publisher',
                                       namespace=topic_ns,
                                       args=pub_args)
        self.state_pub_nodes.append(self.launch.launch(new_node))

    # Call Gazebo spawn model service
    def call_spawn_service(self, model_xml, name, tf_prefix, pose):
        try:
            self.spawn_srv.wait_for_service()
            self.spawn_srv(model_name=name,
                           model_xml=model_xml,
                           robot_namespace=tf_prefix,
                           reference_frame='world',
                           initial_pose=pose)
        except rospy.ServiceException as e:
            rospy.logwarn('service call failed!' + str(e))

    # Perform all necessary steps to spawn and manage a MKZ model in Gazebo
    def spawn_model(self, name, param_dict):
        # Parse parameters
        x = param_dict.get('x', 0.0)
        y = param_dict.get('y', 0.0)
        z = param_dict.get('z', 0.0)
        yaw = param_dict.get('yaw', 0.0)
        start_gear_char = param_dict.get('start_gear', 'D')
        color = param_dict.get('color', 'silver')
        try:
            r = color['r']
            g = color['g']
            b = color['b']
        except:
            if color == 'silver':
                r = 0.75
                g = 0.75
                b = 0.75
            elif color == 'red':
                r = 0.7
                g = 0.2
                b = 0.2
            elif color == 'green':
                r = 0.38
                g = 0.68
                b = 0.05
            elif color == 'blue':
                r = 0.2
                g = 0.2
                b = 0.4
            elif color == 'pink':
                r = 0.8
                g = 0.0
                b = 0.64
            elif color == 'white':
                r = 1.0
                g = 1.0
                b = 1.0
            elif color == 'black':
                r = 0.1
                g = 0.1
                b = 0.1
            else:
                r = 0.75
                g = 0.75
                b = 0.75

        pub_tf = param_dict.get('pub_tf', False)
        pub_odom = param_dict.get('pub_odom', False)
        car_model = param_dict.get('model', 'mkz')
        car_year = param_dict.get('year', 2013)
        production_ref_lat = param_dict.get('production_ref_lat', 45.0)
        production_ref_lon = param_dict.get('production_ref_lon', -81.0)
        topic_ns = param_dict.get('topic_ns', 'vehicle')
        tf_prefix = param_dict.get('tf_prefix', '')

        if car_model in self.model_year_table.keys():
            if not isinstance(car_year, numbers.Number):
                rospy.logwarn('Car year must a number in the range 2013 - 2018.  Using 2013')
                car_year = 2013
                urdf_model = self.model_year_table[car_model][0]
                structure_urdf = self.description_pkg_map[car_model] + '/urdf/' + urdf_model + '_structure.urdf.xacro'
            else:
                if car_year < 2013:
                    car_year = 2013
                    rospy.logwarn('Supported year range is 2013 - 2018.  Saturating to [2013]')
                elif car_year > 2019:
                    car_year = 2019
                    rospy.logwarn('Supported year range is 2013 - 2019.  Saturating to [2019]')

                urdf_model = self.model_year_table[car_model][car_year - 2013]
                structure_urdf = self.description_pkg_map[car_model] + '/urdf/' + urdf_model + '_structure.urdf.xacro'
        else:
            rospy.logwarn('Car model [' + car_model + '] undefined! Using default 2013 MKZ')
            car_year = 2013
            urdf_model = 'mkz_13'
            structure_urdf = self.description_pkg_map['mkz'] + '/urdf/mkz_13_structure.urdf.xacro'

        # Create temporary dae file for desired color
        temp_filename = self.write_temp_dae(r, g, b, car_model, urdf_model)
        if len(temp_filename) == 0:
            rospy.logwarn('Failed to create temporary dae mesh')
            return

        # Parse xacro with temporary mesh file and configuration parameters
        urdf_package = param_dict.get('urdf_package', 'dataspeed_dbw_gazebo')
        urdf_path = param_dict.get('urdf_path', 'urdf/' + car_model + '_model.urdf.xacro')
        model_xml = self.parse_xacro(temp_filename, rospkg.RosPack().get_path(urdf_package) + '/' + urdf_path, structure_urdf, car_model, car_year, topic_ns, tf_prefix, pub_tf, pub_odom, start_gear_char, production_ref_lat, production_ref_lon)

        # Set robot_description parameter for this model
        rospy.set_param(topic_ns + '/robot_description', model_xml)

        # Set first person view offsets
        # rospy.loginfo(rospy.get_published_topics())
        if ['/gazebo_cam_control/parameter_updates', 'dynamic_reconfigure/Config'] in rospy.get_published_topics():
            reconfig_request = ReconfigureRequest()
            reconfig_request.config.doubles.append(
                DoubleParameter(name='x_offset', value=self.first_person_offsets[car_model]['x_offset'])
            )
            reconfig_request.config.doubles.append(
                DoubleParameter(name='y_offset', value=self.first_person_offsets[car_model]['y_offset'])
            )
            reconfig_request.config.doubles.append(
                DoubleParameter(name='z_offset', value=self.first_person_offsets[car_model]['z_offset'])
            )
            reconfig_request.config.doubles.append(
                DoubleParameter(name='view_angle', value=self.first_person_offsets[car_model]['angle'])
            )

            srv = rospy.ServiceProxy('/gazebo_cam_control/set_parameters', Reconfigure)
            srv.wait_for_service(5)
            srv.call(reconfig_request)

        # Call spawn service
        pose = Pose()
        if isinstance(x, numbers.Number) and isinstance(y, numbers.Number) and isinstance(z, numbers.Number):
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
        else:
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
        if isinstance(yaw, numbers.Number):
            pose.orientation.w = math.cos(0.5 * yaw)
            pose.orientation.z = math.sin(0.5 * yaw)
        else:
            pose.orientation.w = 1
            pose.orientation.z = 0
        self.call_spawn_service(model_xml, name, tf_prefix, pose)

        # Run a robot_state_publisher for this model
        self.run_state_pub(topic_ns, tf_prefix)

    # This method is called when node is terminated
    def shutdown_handler(self):
        # Stop robot state publisher nodes
        for process in self.state_pub_nodes:
            process.stop()

        # Delete temporary dae models
        for f in self.temp_dae_files:
            os.remove(f.name)

        # Delete robot_description parameters because they reference the deleted dae files
        for param in rospy.get_param_names():
            if param.find('robot_description') >= 0:
                rospy.delete_param(param)


if __name__ == '__main__':
    spawner_node = MkzSpawner()

    # Parse simulation YAML file
    vehicles = yaml.load(open(rospy.get_param('~sim_param_file'), 'r'))

    # Spawn one simulation instance for each entry in YAML file
    for vehicle in vehicles:
        try:
            model_name = vehicle['vehicle']['model']
            spawner_node.spawn_model(model_name, vehicle['vehicle'])
        except:
            rospy.loginfo('Failed to spawn model [' + str(vehicle) + ']')

    rospy.ServiceProxy('/gazebo/unpause_physics', Empty).call(EmptyRequest())
    rospy.on_shutdown(spawner_node.shutdown_handler)
    rospy.spin()
