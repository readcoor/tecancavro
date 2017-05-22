'''
tests.test_cavroxe

Unit + integration tests for the CavroXE class interface.

Prior to use you must set the environment variable CAVRO_XE_PORT to the
path to the CavroXE serial port.

Original author: Anton Zalutsky (anton.zalutsky@readcoor.com)
'''
import os
import sys

from inspect import stack
import traceback

# At the package root to the python path to allow for imports
sys.path.insert(0,
    os.path.dirname(
        os.path.dirname(
            os.path.realpath(__file__)
        )
    )
)

from tecancavro.models import CavroXE
from tecancavro.transport import TecanAPISerial


# Functions to return instantiated XCaliburD objects for testing
def returnSerialCavroXE():
    try:
        obj = XECavro(com_link=TecanAPISerial(
            0, os.environ['CAVRO_XE_PORT'], 9600
        ))
    except KeyError:
        raise RuntimeError('You must set environment variable CAVRO_XE_PORT '
                           'to the CavroXE serial port')
    return obj


def testChangePort(obj):
    print "\tChanging Ports"
    try:
        obj.changePort(1);
        delay = obj.executeChain()
        obj.waitReady(delay)

        obj.changePort(3);
        delay = obj.executeChain()
        obj.waitReady(delay)

        obj.changePort(2);
        delay = obj.executeChain()
        obj.waitReady(delay)

        obj.changePort(3);
        delay = obj.executeChain()
        obj.waitReady(delay)

        obj.changePort(1);
        delay = obj.executeChain()
        obj.waitReady(delay)
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testMovePlunger(obj):
    print "\tMoving Plungers Absolutely"
    try:
        obj.movePlungerAbs(1000);
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerAbs(750);
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerAbs(500);
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerAbs(250);
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerAbs(0);
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerAbs(1000);
        delay = obj.executeChain()
        obj.waitReady(delay)
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testMovePlungerRel(obj):
    try:
        print "\tMoving Plungers Relatively"
        #At 1000
        obj.movePlungerAbs(1000)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerRel(-5)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerRel(-500)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerRel(-495)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.movePlungerRel(1000)
        delay = obj.executeChain()
        obj.waitReady(delay)
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testDispense(obj):
    try:
        print "\tDispensing"
        obj.movePlungerAbs(1000)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.dispense(1, 500)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.extract(1, 500)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.dispense(2, 1000)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.extract(2, 1000)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.dispense(3, 200)
        delay = obj.executeChain()
        obj.waitReady(delay)
        obj.dispenseToWaste()
        delay = obj.executeChain()
        obj.waitReady(delay)
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testPrimePort(obj):
    try:
        print "\tPriming"
        obj.movePlungerAbs(1000)
        delay = obj.executeChain()
        obj.waitReady(delay)

        obj.primePort(1, 1000)

        obj.movePlungerAbs(1000)
        delay = obj.executeChain()
        obj.waitReady(delay)
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testExtractToWaste(obj):
    try:
        print "\tExtracting Waste"
        obj.movePlungerAbs(1000)
        delay = obj.executeChain()
        obj.waitReady(delay)

        delay = obj.extractToWaste(1, 1000)
        print 'Delay: ' + str(delay)
        obj.waitReady(delay)

        obj.movePlungerAbs(1000)
        delay = obj.executeChain()
        obj.waitReady(delay)
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testRepeat(obj):
    try:
        print "\tRepeat Command"
        obj.markRepeatStart()
        obj.movePlungerAbs(0)
        obj.movePlungerAbs(1000)
        obj.repeatCmdSeq(5)
        delay = obj.executeChain()
        obj.waitReady(delay)
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testInit(obj):
    try:
        print "\tInitialization"
        obj.init(direction='CW')
        obj.init(direction='CCW')
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testQueries(obj):
    try:
        print "\tQueries"
        obj.getFirmwareVersion()
        print '\t\tFirmware: ' + str(obj.state['firmware_version'])
        obj.getBufferStatus()
        print '\t\tBuffer Status: ' + str(obj.state['buffer_status'])
        obj.getAuxInput()
        print '\t\tAux Input: ' + str(obj.state['aux_input'])
        obj.getCurOutputLine()
        print '\t\tCurrent Output: ' + str(obj.state['cur_output'])
        obj.getBacklashSteps()
        print '\t\tBacklash Steps: ' + str(obj.state['backlash'])
        obj.getPlungerPos()
        print '\t\tPlunger Position: ' + str(obj.state['plunger_pos'])
        obj.getSpeed()
        print '\t\tSpeed: ' + str(obj.state['speed'])
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


def testXXX(obj):
    try:
        print "\tXXX"
    except Exception as e:
        return [stack()[0][3],False,traceback.format_exc()]
    return [stack()[0][3],True,None]


if __name__ == '__main__':
    obj = returnSerialXECavro()
    print obj

    test_list = []

    '''
    Run all the tests
    '''
    print "TESTING:"
    test_list.append(testInit(obj))
    test_list.append(testChangePort(obj))
    test_list.append(testMovePlunger(obj))
    test_list.append(testMovePlungerRel(obj))
    test_list.append(testDispense(obj))
    test_list.append(testPrimePort(obj))
    test_list.append(testExtractToWaste(obj))
    test_list.append(testRepeat(obj))
    test_list.append(testQueries(obj))


    print "RESULTS:"
    for test in test_list:
        s = str('\t') + str(test[0]) + str(' : ')
        if test[1] == True:
            s += 'PASSED'
        else:
            s += 'FAILED'
            s += ', '
            s += str(test[2])
        print s

