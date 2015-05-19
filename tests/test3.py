import openravepy as r
import sys
import time
import numpy


m = None
e = None


def readFileCommands( datafile=None, doCommandLineInputs=True ):
    global e

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
        
        elif text[0] == 'a':
            with e:
                v = e.GetViewer()
                xform = v.GetCameraTransform()
                print xform

        elif text[0] == "r" or text[0] == "read":

            if len( text ) >= 2: 
                f = open( text[1], 'r' )
                m.SendCommand( "destroy" )

                if f != None:
                    readFileCommands( f, False )
        elif text[0] == 'b':
            with e: 
                start = [ 5.65, -1.76, -0.26,  1.96, -1.15, 0.87, -1.43,  0.00000000e+00, 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 , -1.66533454e-16, -1.66533454e-16, -1.38777878e-16,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00 ]

                robot = e.GetRobot( "Herb2" )
                robot.SetName( "original" )
                e.Load( "herb2_padded_nosensors.robot.xml" )
                robot2 = e.GetRobot( "Herb2" )
                robot2.SetDOFValues( start )
        
                for link in robot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency( 0.3 )

        else:
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
    v = e.GetViewer()

    start = [
       5.65, -1.76, -0.26,  1.96, -1.15, 0.87, -1.43,  0.00000000e+00,
       0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 
       0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ,
      -1.66533454e-16, -1.66533454e-16,
      -1.38777878e-16,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00 ]

    Tz = numpy.array(
        [[-0.81596105,  0.24022406, -0.52583264,  1.39736962],
         [ 0.57709228,  0.28460075, -0.76548476,  1.4022727 ],
         [-0.03423549, -0.9280597 , -0.37085458,  1.57485259],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])

    v.SetCamera( Tz )

    e.Load( "herb2_padded_nosensors.robot.xml" )

    robot = e.GetRobot( "Herb2" )
    with e:
        robot.SetName( "original" )
    e.Load( "herb2_padded_nosensors.robot.xml" )

    robot2 = e.GetRobot( "Herb2" )


    with e:
        robot.SetDOFValues( start )
        robot2.SetDOFValues( start )
        
        """
        for link in robot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency( 0.3 )
        """
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
