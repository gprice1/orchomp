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

    e.Load( "intelkitchen_robotized_herb2_nosensors_2.env.xml" );
    robot = e.GetRobot( "Herb2" )


    start = [
       3.68000000e+00, -1.90000000e+00,  0.00000000e+00,  2.20000000e+00,
      -2.22044605e-16,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.60000000e+00,
      -1.90000000e+00,  0.00000000e+00,  2.20000000e+00,  0.00000000e+00,
      -1.11022302e-16,  0.00000000e+00, -1.66533454e-16, -1.66533454e-16,
      -1.38777878e-16,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00 ]

    robot.SetDOFValues( start )
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
