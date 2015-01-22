#!/usr/bin/python
# -*- coding: utf-8 -*-
#==============================================================================
#               ALDEBARAN-ROBOTICS - SOFTWARE QA
#==============================================================================
# FILE : PaNiProblem.py
# DESCRIPTION

"""
Description:
    Tools to check differents information of robot.
"""

#[MODULE INFO]-----------------------------------------------------------------

__author__ = "ddambreville"
__date__ = "16/01/2015"
__copyright__ = "Copyright 2015, Aldebaran-Robotics (c)"
__version__ = "1.1"
__maintainer__ = "ddambreville"
__email__ = "ddambreville@aldebaran.com"

#[IMPORTS]---------------------------------------------------------------------

import ConfigParser
import argparse
import time
import sys
import os
import stat
from threading import Thread
from naoqi import ALProxy

DEFAULT_IP = "127.0.0.1"
try:
    path = os.getcwd()
    os.chmod(path, stat.S_IXGRP)
except:
    pass
'''
path = os.getcwd()
print "path : " + path
os.path.basename(path)
# sys.path.append(path)
#files = os.listdir(path)
'''


def read_section(my_file, section):
    """Read section from a configuration file."""
    config = ConfigParser.ConfigParser()
    config.optionxform = str
    config.read(my_file)

    if config.has_section(section):
        config_section = config._sections[section]
        config_section.pop("__name__")

        for key, value in config_section.items():
            config_section[key] = value.split()

        return config_section

    else:
        return {}


def serial_ID(mem):
    """Head and Body ID"""
    head_ID = mem.getData('RobotConfig/Head/FullHeadId')
    body_ID = mem.getData('Device/DeviceList/ChestBoard/BodyId')
    print "Body ID: " + body_ID
    print "Head ID: " + head_ID
    return


def Head_Config(mem):
    """Read Head's configuration key from DeviceHead"""
    # Variable and List
    prefix = 'RobotConfig/Head/Device/'
    header_head = ('Key', 'Description')
    final_key_list = []
    key_list = ['TouchBoard', 'Camera', 'Micro',
                '3dCamera', 'BaseVersion', 'MicroSD', 'Processor']

    for key in key_list:
        key_check = read_section('Pepper_Head_config.cfg', key)
        if key == 'BaseVersion':
            key_value = mem.getData(prefix.replace('Device/', '') + key)
        elif key == 'MicroSD':
            key_value = mem.getData(prefix + key + '/Size')
        elif key == 'Processor':
            key_value = mem.getData(prefix + key + '/Frequency')
        else:
            key_value = mem.getData(prefix + key + '/Version')
        for check in key_check:
            if check == key_value:
                key_description = key_check[check][0]
                key_list = (key, key_description.replace('_', ' '))
                final_key_list.append(key_list)
                longg_head = dict(
                    zip((0, 1), (len(str(x)) for x in header_head)))

    # Distribute the data in a table
    for tu in final_key_list:
        for x in final_key_list:
            longg_head.update(
                (i, max(longg_head[i], len(str(el))))
                for i, el in enumerate(tu))
            longg_head[1] = max(longg_head[1], len(str(x)))
    fofo_error = ' | '.join('%%-%ss' % longg_head[i] for i in xrange(0, 2))
    print '\n'.join((fofo_error % header_head,
                     '-|-'.join(longg_head[i] * '-' for i in xrange(2)),
                     '\n'.join(fofo_error % (a, b)
                               for (a, b) in final_key_list)))
    return


def Body_Config(mem):
    """Read Body's configuration key from DeviceBody"""
    # Variable and List
    prefix = 'RobotConfig/Body/Device/'
    header_body = ('Key', 'Description')
    final_key_body = []
    key_list = ['LeftHand', 'RightHand', 'LeftArm', 'RightArm',
                'Legs', 'Brakes', 'Wheels', 'Platform', 'Version',
                'Calibration']

    # Exception
    for key in key_list:
        key_check = read_section('Pepper_Body_config.cfg', key)
        if key == 'LeftHand':
            key = "Hand/Left"
        elif key == 'RightHand':
            key = 'Hand/Right'
        elif key == 'Version':
            key = 'BaseVersion'
        elif key == 'Calibration':
            key = 'Joints/Calibration'
        try:
            if key == 'BaseVersion':
                key_value = mem.getData(prefix.replace('Device/', '') + key)
            else:
                key_value = mem.getData(prefix + key + '/Version')
        except:
            pass
        for check in key_check:
            if check == key_value:
                key_description = key_check[check][0]
                key_list = (key, key_description.replace('_', ' '))
                final_key_body.append(key_list)
                longg_body = dict(
                    zip((0, 1), (len(str(x)) for x in header_body)))

    # Distribute the data in a table
    for tu in final_key_body:
        for x in final_key_body:
            longg_body.update(
                (i, max(longg_body[i], len(str(el))))
                for i, el in enumerate(tu))
            longg_body[1] = max(longg_body[1], len(str(x)))
    fofo_error = ' | '.join('%%-%ss' % longg_body[i] for i in xrange(0, 2))
    print '\n'.join((fofo_error % header_body,
                     '-|-'.join(longg_body[i] * '-' for i in xrange(2)),
                     '\n'.join(fofo_error % (a, b)
                               for (a, b) in final_key_body)))

    return


def trad_serial(mem):
    """Read Serial Number from DeviceHead"""
    # Variable
    serial_ = mem.getData('Device/DeviceList/ChestBoard/BodyId')
    serial_date = []
    trad_first = read_section("Serial_trad.cfg", "FirstCharacter")
    trad_second = read_section("Serial_trad.cfg", "SecondCharacter")
    trad_third = read_section("Serial_trad.cfg", "ThirdCharacter")

    first_character = serial_[12]
    second_character = serial_[13]
    third_character = serial_[14]

    if first_character not in trad_first or second_character \
            not in trad_second or third_character not in trad_third:
        print "wrong characters"
        return

    for trad in trad_first:
        if trad == first_character:
            serial_date.append(trad_first[trad][0])
    for trad in trad_second:
        if trad == second_character:
            serial_date.append(trad_second[trad][0])
    for trad in trad_third:
        if trad == third_character:
            serial_date.append(trad_third[trad][0])

    print "Production date : " + serial_date[2] + " " + serial_date[1] + \
        " " + serial_date[0]
    return


class Nack(Thread):

    def __init__(self, board_list, mem):
        Thread.__init__(self)
        self.board_list = board_list
        self.nack_status = []
        self.mem = mem

    def run(self):
        prefix = 'Device/DeviceList'
        suffix = 'Nack'
        first_nack = self.mem.getListData(
            ["/".join([prefix, board, suffix]) for board in self.board_list])
        time.sleep(5)
        second_nack = self.mem.getListData(
            ["/".join([prefix, board, suffix]) for board in self.board_list])

        for index, (board, NackMax, NackMin) in \
                enumerate(zip(self.board_list, second_nack, first_nack)):
            value = (NackMax - NackMin)
            self.nack_status.append(value)

        # print self.nack_status
    def result(self):
        return self.nack_status


class Ack(Thread):

    def __init__(self, board_list, mem):
        Thread.__init__(self)
        self.board_list = board_list
        self.ack_value = []
        self.mem = mem

    def run(self):
        prefix = 'Device/DeviceList'
        suffix = 'Ack'

        first_ack = self.mem.getListData(
            ["/".join([prefix, board, suffix]) for board in self.board_list])
        time.sleep(5)
        last_ack = self.mem.getListData(
            ["/".join([prefix, board, suffix]) for board in self.board_list])

        for index, (board, AckMax, AckMin) \
                in enumerate(zip(self.board_list, last_ack, first_ack)):
            value = (AckMax - AckMin)  # /30
            self.ack_value.append(value)

    def result(self):
        return self.ack_value


class Error(Thread):

    def __init__(self, board, mem, prefix):
        Thread.__init__(self)
        self.board = board
        self.mem = mem
        self.prefix = prefix

    def run(self):
        upper_list = ['HeadBoard', 'LeftArmBoard', 'LeftHandBoard',
                      'LeftShoulderBoard', 'RightArmBoard', 'RightHandBoard',
                      'RightShoulderBoard']
        down_list = ['BackPlatformBoard', 'FrontPlatformBoard',
                     'HipBoard', 'HubBoardPlatform', 'ThighBoard']
        laser_list = ['LaserSensorFrontPlatform',
                      'LaserSensorLeftPlatform', 'LaserSensorRightPlatform']
        board_error = read_section("Config_joint_board_pepper.cfg", "Board")
        self.error_description = []
        error_description = "UNKNOWN ERROR"
        i = 0
        for i in range(0, 5):
            if self.board in upper_list:
                error_list = read_section("Error_list.cfg", "Up")
                code_error = self.mem.getData(
                    self.prefix + str(self.board) + "/Error")
                for error in error_list:
                    if int(code_error) != 0:
                        if int(code_error) != int(error):
                            error_description = str(
                                code_error) + " " + error_description
                            self.error_description.append(
                                error_description)
                        else:
                            error_description = str(
                                code_error) + " " + error_list[error][0]
                            self.error_description.append(error_description)
                        i = i + 1

            elif self.board in down_list:
                error_list = read_section("Error_list.cfg", "Down")
                code_error = self.mem.getData(
                    self.prefix + str(self.board) + "/Error")

                for error in error_list:
                    if int(code_error) != 0:
                        if int(code_error) != int(error):
                            error_description = str(
                                code_error) + " " + error_description
                            self.error_description.append(
                                error_description)
                        else:
                            error_description = str(
                                code_error) + " " + error_list[error][0]
                            self.error_description.append(error_description)
                        i = i + 1

            else:
                error_list = read_section("Error_list.cfg", "Laser")
                code_error = self.mem.getData(
                    self.prefix + str(self.board) + "/Error")

                for error in error_list:
                    if int(code_error) != 0:
                        if int(code_error) != int(error):
                            error_description = str(
                                code_error) + " " + error_description
                            self.error_description.append(
                                error_description)
                        else:
                            error_description = str(
                                code_error) + " " + error_list[error][0]
                            self.error_description.append(error_description)
                        i = i + 1

    def result(self):
        return self.error_description


class Firmware_version(Thread):

    def __init__(self, board, mem, prefix):
        Thread.__init__(self)
        self.board = board
        self.mem = mem
        self.prefix = prefix

    def run(self):
        self.firmware_version = self.mem.getData(
            self.prefix + str(self.board) + '/ProgVersion')

    def result(self):
        return self.firmware_version


def info_battery(mem):
    info = mem.getData('BatteryChargeChanged')
    print 'Battery percent level : ' + str(info) + ' %'
    return


def main():

    #[Argument Parser]---------------------------------------------------------
    parser = argparse.ArgumentParser(description='Information from Robot')

    parser.add_argument("-i", "--IP", dest="robot_ip", default=DEFAULT_IP,
                        help="robot IP or name (default: 127.0.0.1).")

    parser.add_argument("-v", "--version", action="version",
                        version="%(prog)s 1.1")

    parser.add_argument("-w", "--wake", dest="wakeup",
                        help='wakeUp for robot')

    args = parser.parse_args()

    #[VARIABLE]----------------------------------------------------------------
    firmware_version = 0
    nack_status = "ok"
    ack_value = 0
    final_list = []
    final_error = []
    final_ack_list = []

    #[PROXY]-------------------------------------------------------------------
    system = ALProxy('ALSystem', args.robot_ip, 9559)
    mem = ALProxy('ALMemory', args.robot_ip, 9559)
    motion = ALProxy('ALMotion', args.robot_ip, 9559)

    #[DICO]--------------------------------------------------------------------
    prefix = "Device/DeviceList/"
    board_list = ['BackPlatformBoard', 'ChestBoard', 'FaceBoard',
                  'FrontPlatformBoard', 'HeadBoard', 'HipBoard',
                  'HubBoardPlatform', 'InertialSensor',
                  'InertialSensorBase', 'LaserSensorFrontPlatform',
                  'LaserSensorLeftPlatform', 'LaserSensorRightPlatform',
                  'LeftArmBoard', 'LeftHandBoard', 'LeftShoulderBoard',
                  'RightArmBoard', 'RightHandBoard', 'RightShoulderBoard',
                  'ThighBoard', 'TouchBoard']

    print ' ____         _   _ _   ____        _     _    '
    print '|  _ \ __ _  | \ | (_) |  _ \  ___ | |__ | | ___ _ __ ___  '
    print '| |_) / _` | |  \| | | | |_)  / _ \| |_ \| |/ _ \ |_ ` _ \ '
    print '|  __/ (_| | | |\  | | |  __/| (_) | |_) | |  __/ | | | | |'
    print '|_|   \__,_| |_| \_|_| |_|    \___/|_.__/|_|\___|_| |_| |_|'
    print "\n#############################################################"
    print "#                     General Parameters                    #"
    print "#############################################################"

    # WakeUp Robot
    if args.wakeup:
        print 'Robot WakeUp'
        motion.wakeUp()

    # Robot Name
    robot_name = system.robotName()
    print "\nRobot Name : " + robot_name

    # Naoqi Version
    naoqi_version = system.systemVersion()
    print "Naoqi Version : " + naoqi_version

    # Head and Body ID
    serial_ID(mem)

    # Serial
    trad_serial(mem)

    # Battery lvl
    info_battery(mem)

    print "\n#############################################################"
    print "#                     Configuration Key                     #"
    print "#############################################################"

    # Body Config
    Body_Config(mem)

    # Head Config
    print'\n'
    Head_Config(mem)

    print "\n#############################################################"
    print "#                       Robot Analysis                      #"
    print "#############################################################"
    thread_ack = Ack(board_list, mem)
    thread_nack = Nack(board_list, mem)

    thread_ack.start()
    thread_nack.start()
    for board in enumerate(board_list):
        thread_error = Error(board[1], mem, prefix)
        thread_firmware = Firmware_version(board[1], mem, prefix)

        # Start thread
        thread_error.start()
        thread_firmware.start()

        # wait thread is finish
        thread_nack.join()
        thread_ack.join()
        thread_error.join()
        thread_firmware.join()

        # recup result
        nack_result = thread_nack.result()
        ack_result = thread_ack.result()
        error_result = thread_error.result()
        firmware_result = thread_firmware.result()

        # create list for table
        value_list = (board[1], str(firmware_result))
        try:
            if error_result != 'no error':
                if len(error_result) > 1:
                    for error in error_result:
                        error_list = (board[1], str(error.replace('_', ' ')))
                        final_error.append(error_list)
                else:
                    error_list = (
                        board[1], str(error_result[0].replace('_', ' ')))
                    final_error.append(error_list)
            else:
                pass
            error_header = ('Board', 'Error description')
            longg_error = dict(
                zip((0, 1), (len(str(x)) for x in error_header)))
        except:
            pass
        header = ('Board', 'Firmware Version')
        final_list.append(value_list)
        longg = dict(zip((0, 1), (len(str(x)) for x in header)))

    for index, (board, ack, nack) in enumerate(zip(board_list, ack_result,
                                                   nack_result)):
        if nack > 0:
            ack_nack = (ack / nack) * 100
        else:
            ack_nack = None
        ack_nack_list = (board, ack, nack, ack_nack)
        final_ack_list.append(ack_nack_list)

    header_ack_nack = ('Board', 'Ack (5s)', 'Nack Status', 'Ack/Nack (%)')
    longg_ack = dict(zip((0, 1, 2, 3), (len(str(x)) for x in header_ack_nack)))

    # table ack
    for val in final_ack_list:
        for x in final_ack_list:
            longg_ack.update(
                (i, max(longg_ack[i], len(str(el)))) for i, el
                in enumerate(val))
            longg_ack[3] = max(longg_ack[3], len(str(x)))
    fofo = ' | '.join('%%-%ss' % longg_ack[i] for i in xrange(0, 4))
    print '\n'.join((fofo % header_ack_nack,
                     '-|-'.join(longg_ack[i] * '-' for i in xrange(4)),
                     '\n'.join(fofo % (a, b, c, d) for (a, b, c, d) in
                               final_ack_list)))
    print '\n'

    # table
    for tu in final_list:
        for x in final_list:
            longg.update((i, max(longg[i], len(str(el))))
                         for i, el in enumerate(tu))
            longg[1] = max(longg[1], len(str(x)))
    fofo = ' | '.join('%%-%ss' % longg[i] for i in xrange(0, 2))
    print '\n'.join((fofo % header,
                     '-|-'.join(longg[i] * '-' for i in xrange(2)),
                     '\n'.join(fofo % (a, b) for (a, b) in final_list)))

    print "\n#############################################################"
    print "#                            Error                          #"
    print "#############################################################"
    try:
        # table error
        for tu in final_error:
            for x in final_error:
                longg_error.update(
                    (i, max(longg_error[i], len(str(el)))) for i, el
                    in enumerate(tu))
                longg_error[1] = max(longg_error[1], len(str(x)))
        fofo_error = ' | '.join(
            '%%-%ss' % longg_error[i] for i in xrange(0, 2))
        print '\n' + '\n'.join((fofo_error % error_header,
                                '-|-'.join(longg_error[i]
                                           * '-' for i in xrange(2)),
                                '\n'.join(fofo_error % (a, b) for (a, b)
                                          in final_error)))
    except:
        print '\nNo error on your robot'
        pass

if __name__ == '__main__':
    main()
