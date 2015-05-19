import openravepy as r
import sys
import time
import numpy
import random


#globals
e = None
m = None 
robot = None
knownstates = None


def getRandomState():
    lower, upper = robot.GetActiveDOFLimits()
    value = []
    for i in xrange( len( upper ) ):
        value.append( lower[i] + (upper[i]-lower[i]) * random.random() )
    return value

def getRandomState():
    lower, upper = robot.GetDOFLimits()
    value = []
    for i in xrange( len( upper ) ):
        value.append( lower[i] + (upper[i]-lower[i]) * random.random() )
    return value
   
def getRandomStates( n ):
    return [ getRandomState() for i in xrange( n )]

def saveState( savefile, state ):
    savefile.write( "\t".join( [ str(i) for i in state] ) + "\n" )

def loadState( savefile ):
    line = savefile.readline()
    words = line.split()
    return [ float( i ) for i in words ]




def userGetRandomArmStates( filename ):


    getmore = True
    savefile = open( filename, 'w' )
    robot.SetActiveDOFs( robot.GetManipulators()[0].GetArmIndices() )
    
    for i in xrange( 100 ):
        
        state = []
        invalid = True
        while invalid:
            state = getRandomState( )
            robot.SetActiveDOFValues( state )
            
            for key, value in knownstates.items():
                invalid = False
                robot.SetDOFValues( value[1], robot.GetManipulators()[1].GetArmIndices() )
                
                if robot.CheckSelfCollision( ):
                    invalid = True
                    break
                
        state = [ state[i] for i in robot.GetManipulators()[0].GetArmIndices() ] 
        print state
        saveState( savefile, state )


def userGetRandomStates( filename ):

    getmore = True
    savefile = open( filename, 'w' )

    while getmore:

        while True:
            state = getRandomState()
            robot.SetDOFValues( state )
            if not robot.CheckSelfCollision( ):
                break

        while True:
            
            s = raw_input( '\n-->' )
            if s == 'q' or s == 'quit' or s == 'quit()' or s == 'end':
                getmore = False
                break

            elif s == 's' or s == 'save':
                saveState( savefile, state )
                break
            elif s =='n' or s=='no' or s=='next':
                break

            else:
                print "\nNot a valid command"

def readKnownStates( filename ):
    datafile = open( filename, 'r')
    data = datafile.read()
    data = data.replace( ',' , '')
    lines = data.split( "\n" )

    knownStates = {}

    for i in xrange( 0, len( lines ), 3 ):
        if i+2 >= len( lines ):
            break 
        tag = lines[i]
        tag = tag[:-1]

        state = []
        state.append( [ float( j ) for j in lines[i+1].split() ] )
        state.append( [ float( j ) for j in lines[i+2].split() ] )
        
        knownStates[ tag ] = state

    return knownStates


def setRightArmState( state ):
    robot.SetDOFValues( state, robot.GetManipulators()[0].GetArmIndices() )

def printKnownStates( known_states ):
    for key, value in known_states.items():
        print key, ":"
        setstate( value )
        print robot.GetDOFValues()

def setstate( state ):

    robot.SetDOFValues( state[0], robot.GetManipulators()[0].GetArmIndices() )
    robot.SetDOFValues( state[1], robot.GetManipulators()[1].GetArmIndices() )

def main():
    global e
    global m
    global robot
    global knownstates

    e = r.Environment()
    m = r.RaveCreateModule( e, 'orcdchomp' )
    


    e.SetViewer( 'qtcoin' )
    #e.Load( 'lab1.env.xml' );

    e.Load( 'herb2_padded_nosensors.robot.xml')
    robot = e.GetRobot( 'Herb2' )
    
    knownstates = readKnownStates( "data/knownstates.data")
    printKnownStates( knownstates )

    return 
    
    
    #robot.SetActiveManipulator( 0 )
    
    filename = "randomstates2.data"
    #filename = "randomManipulatorStates_0.data"

    userGetRandomStates( filename )

    return

def getCommandLineFile():

    name = ''

    if len( sys.argv ) > 1:
        name = sys.argv[1]
    else:
        name = 'test_default.txt'
        
    f = open( name, 'r' )
    return f


def readFileCommands( datafile=None, doCommandLineInputs=True ):
    
    if( datafile == None ):
        datafile = getCommandLineFile()

    data = datafile.read()
    datafile.close()

    lines = data.split('\n')
    
    current_command = []
    for line in lines : 
        
        words = line.split()

        #if the line is empty or the first letter of the first word is #,
        #   do not get the line.
        if len(line) == 0 or words[0][0] == '#':
            continue
        

        current_command += words

        if words[-1][-1] != "/":
            command_string = " ".join(current_command)

            print "Command:" , command_string
            m.SendCommand( command_string )
            current_command = []
        else:
            current_command[-1] = current_command[-1][:-1]

   
    while doCommandLineInputs:
        s = raw_input( '-->' )
        text = s.split()
        if len( text ) < 1:
            continue

        if text[0] == 'q' or text[0] == 'quit' or text[0] == 'quit()' or text[0] == 'end':
            break

        if text[0] == "r" or text[0] == "read":

            if len( text ) >= 2: 
                f = open( text[1], 'r' )
                m.SendCommand( "destroy" )

                if f != None:
                    readFileCommands( f, False )

        try:
            m.SendCommand( s )
        except:
            print "Not a valid command: " + s


main()
