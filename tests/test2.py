import openravepy as r
import sys
import time
import numpy


m = None
e = None


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
        except r.openrave_exception, e:
            print e

def main():
    global m
    global e

    e = r.Environment()
    m = r.RaveCreateModule( e, 'orchomp' )
    e.SetViewer( 'qtcoin' )

    e.Load( "intelkitchen_robotized_herb2_nosensors.env.xml" );
    robot = e.GetRobot( "Herb2" )

    # set the manipulator to leftarm
    #ikmodel = databases.inversekinematics.InverseKinematicsModel(
    #            robot,iktype=IkParameterization.Type.Transform6D)
    #if not ikmodel.load():
    #    ikmodel.autogenerate()

    Tz = r.matrixFromAxisAngle([0,0,numpy.pi/2])
    Tz[0,3] = 0.4  
    Tz[1,3] = 1.6
    Tz[2,3] = -0.01

    print Tz
    with e:
        for body in e.GetBodies():
            body.SetTransform(numpy.dot(Tz,body.GetTransform()))

    readFileCommands()


def getCommandLineFile():

    name = ''

    if len( sys.argv ) > 1:
        name = sys.argv[1]
    else:
        name = 'test_default.txt'
        
    f = open( name, 'r' )
    return f



main()
