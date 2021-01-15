//might need to change package line, but that's about it.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * IT'S TETRIS
 */

@TeleOp(name ="Tetris by 6547 Cobalt Colts", group = "Tetris")
public class Tetris extends LinearOpMode {

    //TODO : fix rotation function, then polish it (now gotta polish it)
    //TODO : add function where the piece that is currently being controlled will land if it is has been touching a landed piece or the bottom of the screen.
    //TODO : add a function where the player can push the down button on the gamepad to make the piece in control go faster
    //TODO : add a score function
    //TODO : make the screen size customizable, (8x20 might be a bt big)
    //TODO : add a highscore function

    boolean debugMode = false; //debug mode, can be enabled in program by pushing B and Y before the start button is pressed
    String debugModeText = "";
    int rotationErrorCounter = 0; //added this for fun


    //from here to runOpMode(), theses are short summaries of the variables, below are all the methods where all the variables are used, they will have more complex and useful descriptions.

    //change the lines[][] variable to customize the size of the tetris board.  Big boards may result in bugs.  Debug mode is better for smaller boards
    boolean[][] lines = new boolean[20][10];  //a 2d array that is an 20x10 "screen", think of each coordinate (example: lines[0][0]) as a pixel.  if the coordinate value is true then █ displays, but if the value is false then ░ displays.
    //feel free to change the lines[][] variable to whatever you want it to be.  The first value is the Y-axis and the second value is the X-axis.  Just make sure that the the y axis is 3 more than you want it to be.  Trust me on this, that's just how this program works

    String[] sline = new String[(lines.length-1)]; //stands for string lines.  these are the strings that will be displayed via telemetry, one string per lines.length
    int spaces=39; //spaces added before the board is displayed in order to get the board in the center of the screen.
    String bigSpace="";
    int nextDecider=(int) (Math.random()*7);
    int decider=(int) (Math.random()*7);
    int[] pieces = new int[4]; //the coordinates for a piece, all pieces take up 4 values from lines[][].  go to the getPiece() Method for more information about this
    int[] pieces90 = new int[4]; //the piece but rotated 90 degrees
    int[] pieces180 = new int[4]; //the piece but 180 degrees
    int[] pieces270 = new int[4]; //the piece but rotated 270 degrees
    int[] tempPiece = new int[4]; //temp piece, it's used when the piece is rotating
    int rotation = 0;
    int pieceModifery = 0;
    int pieceModiferx = 0;

    String[] nextPieceText = new String[3];

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime otherTime = new ElapsedTime();

    static int cycleLeft = 0; //used to wait for the left D-pad to release for better gameplay.
    static int cycleRight=0;  //used to wait for the right D-pad to release for better gameplay.
    static int cycleX = 0;

    int pieceState = 0;
    final int notTouching = 0;
    final int touching = 1;
    final int touchedFor1Second =2;

    final double landingTime = 1;

    //int realFrameRate = 0;
    //int frameRate = 0;

    String frameRate="";
    int frame = 0;

    String DisplayThis="";

    ElapsedTime frametime = new ElapsedTime();

    public void runOpMode() { //before reading anything under runOpMode(), read all the methods that are called during this method first
        //old code to check if the size of lines[][] is valid
        /*if (lines.length > 20)
        {
            telemetry.log().add("Invalid lineY size, changing it to lines[20][" + lines[0].length + "]");
            lines = new boolean[10][lines[0].length];
        } if (lines.length <=4)
        {
            telemetry.log().add("Invalid lineY size, changing it to lines[5][" + lines[0].length + "]");
            lines = new boolean[5][lines[0].length];
        }if (lines[0].length <= 4)
        {
            telemetry.log().add("Invalid lineX size, changing it to lines["+lines.length+"][5]");
            lines = new boolean[lines.length][5];
        } if (lines[0].length > 100)
        {
            telemetry.log().add("Invalid lineX size, changing it o lines["+lines.length+"][100]");
            lines = new boolean[lines.length][100];
        }
        */
        //calculate spaces
        spaces-=(lines[0].length-4)*1.5;
        for (int i =0; i< spaces; i++) bigSpace+=" ";
        //prep board
        for (int i = 0; i < lines.length; i++) for (int j = 0; j < lines[0].length; j++) lines[i][j] = false; //sets the entire screen to false, so it all will display  ░ once the displayStrings command is called.
        for (int i = 0; i < lines[0].length; i++) lines[lines.length-1][i]=true; //set floor
        for (int i = 0; i < sline.length; i++) sline[i] = ""; //sets all the sline[] values to "" so it can works with the makeStrings() method.
        nextPieceText[0] = "Next Piece:";
        telemetry.log().add("Tetris by 6547 Cobalt Colts");
        telemetry.log().add("");
        telemetry.log().add("Scroll Down to revel the full screen");
        telemetry.log().add("Use the Left, Right, and Down D-pad buttons to move pieces and the X button to rotate ");
        telemetry.log().add("Push B and Y simultaneously to enter debug mode, otherwise push the start button");
        while (!isStarted())
        {
            if (gamepad1.y && gamepad1.b) { //if y or b is pressed, a
                debugMode = !debugMode;
                telemetry.log().add((debugMode) ? "Debug mode enabled" : "Debug mode disabled");
                while (!isStarted() && gamepad1.y && gamepad1.b); //wait for
            }
        }

        waitForStart();

        getPiece();  //gets the coordinates for a new piece.

        setLines(0, 0); //converts the piece so it can be stored in the lines[][] array.

        makeStrings(); //converts the lines[][] array into strings

        displayStrings(); //displays those strings

        while (opModeIsActive()) {

            cycleRight = (gamepad1.dpad_right) ? 1 : 0; //checks the gamepad so this program won't act weird
            cycleLeft = (gamepad1.dpad_left) ? 1 : 0;
            cycleX = (gamepad1.x) ? 1 : 0;
            runtime.reset();
            while (opModeIsActive() && pieceState!=touchedFor1Second)
            {
                if (runtime.seconds() >= 1 && pieceState==notTouching) { //after every second
                    resetStrings(); //clears the strings, and sets the lines[][] values that are true because of the piece currently being controlled by the player to false.
                    setLines(0, -1);

                    if (!pieceHittingSomething()) runtime.reset();
                    else pieceState = touching;
                }
                else if (runtime.seconds() >= (landingTime + 1) && pieceHittingSomething() && pieceState==touching)
                {
                    pieceState = touchedFor1Second;
                }
                if (!pieceHittingSomething() && pieceState == touching)
                {
                    pieceState = notTouching;
                    runtime.reset();
                }
                useGamepadInput(); //enables the use of the right and left and down D-pad buttons
                if (gamepad1.x) rotatePiece();
                if (gamepad1.y && gamepad1.b) if (debugMode)
                {
                    debugMode = false;
                    telemetry.log().add("Debug mode activated");
                    debugModeText = "Debug mode activated";
                } else
                {
                    debugMode = true;
                    telemetry.log().add("Debug mode deactivated");
                    debugModeText = "Debug mode deactivated";
                }
                if (debugMode) {
                    debugModeText = ((pieceState==notTouching) ? "Piece is not touching" : (pieceState==touching) ? "Piece is touching" : (pieceState==touchedFor1Second) ? "Piece touched for one second" : "Piece state is unknown");
                    telemetry.log().add(debugModeText);
                }

            }  //keeps running until the piece hits something
            runtime.reset();
            pieceState = notTouching;
            if (debugMode)
            {
                telemetry.log().add("getting new piece"); //used for debugging, signals the piece in the players control landed (or hit something), so a new piece will be loaded
                debugModeText = "Glitching out for a sec, I'll be back soon";
            }
            getPiece();
            resetStrings();
            setLines(0, 0);

            makeStrings();

            displayStrings();
            if (pieceHittingSomething()) gameOver();
        }

    }
    public boolean pieceHittingSomething() //if the piece is touching the bottom, or there is another piece right below it
    {
        for (int i = 0; i < 4; i++)
        {
            if ((pieces[i]/100) == (lines.length-3)) return true;
            else if (lines[(pieces[i]/100) + 1][pieces[i]%100] && !contains(pieces, (pieces[i]+100))) return true;
        }
        return false;
    }
    public boolean contains(int[] array, int number) //only used with pieces[].  checks if an array contains a number.
    {
        for (int n : array) if (number==n) return true;
        return false;
    }
    public boolean contains(boolean[] array, boolean value)  //same as the other contains() method, but with booleans instead of ints
    {   boolean result = false;
        for (int i = 0; i < array.length; i++)
        {
            if (array[i] == value) result = true;
            else break;
        }
        return result;
    }
    public boolean pieceInBoundsRight() //if the piece is touching the right side of the playing field, used to prevent a piece from going out of bounds
    {
        for (int i = 0; i < 4; i++)
        {
            if (pieces[i]%100 == lines[0].length - 1) return false;
            else if (lines[(pieces[i]/100)][(pieces[i]%100) +1] && !contains(pieces, (pieces[i]+1))) return false;
        }
        if (debugMode)
        {
            telemetry.log().add("out of bounds right");
            debugModeText = "out of bounds right";
        }
        return true;
    }
    public boolean pieceInBoundsLeft() //if the piece is touching the left side of the playing field, used to prevent a piece from going out of bounds
    {
        for (int i = 0; i < 4; i++)
        {
            if (pieces[i]%100 == 0) return false;
            else if (lines[(pieces[i]/100)][(pieces[i]%100) -1] && !contains(pieces, (pieces[i]-1))) return false;
        }
        if (debugMode)
        {
            telemetry.log().add("out of bounds left");
            debugModeText = "out of bounds left";
        }
        return true;
    }
    public void useGamepadInput()  //this is a method so that way it's easier to comment out in runOpMode() for debugging purposes
    {

        if (gamepad1.dpad_right && cycleRight == 0 && pieceInBoundsRight()) //if the right D-pad is pressed, the left D-pad was released on the previous cycle, and the current piece is not hitting the right edge of the playing field
        {
            if (pieceState==touchedFor1Second)
            {
                if (debugMode)
                {
                    telemetry.log().add("Unable to go right");
                    debugModeText = "Unable to go right";
                }
                //if (pieceHittingSomething() && pieceState==touching && runtime.seconds() > landingTime) clearStrings();
                clearStrings();
            }
            else
            {
                if (debugMode)
                {
                    telemetry.log().add("Dpad right pushed");
                    debugModeText = "Dpad right pushed";
                }
                resetStrings();
                setLines(1,0);
                //makeStrings();
                //displayStrings();
                if (pieceHittingSomething() && pieceState==touching && runtime.seconds() > landingTime) clearStrings();
                else resetStrings();
            }
            cycleRight = 1;
        } else if (!gamepad1.dpad_right && cycleRight == 1)
        {
            cycleRight = 0; //wait for the right D-pad to be released before running the code above again.
            if (debugMode)
            {
                telemetry.log().add("Dpad right released");
                debugModeText = "Dpad right released";
            }
        }
        if (gamepad1.dpad_left && cycleLeft == 0 && pieceInBoundsLeft()) //if the left D-pad is pressed, the left D-pad was released on the previous cycle, and the current piece is not hitting the left edge of the playing field
        {
            if (pieceState==touchedFor1Second)
            {
                if (debugMode)
                {
                    telemetry.log().add("Unable to go left");
                    debugModeText = "Unable to go left";
                }
                //if (pieceHittingSomething() && pieceState==touching && runtime.seconds() > landingTime) clearStrings();
                clearStrings();
            }
            else
            {
                if (debugMode)
                {
                    telemetry.log().add("Dpad left pushed");
                    debugModeText = "Dpad left pushed";
                }
                resetStrings();
                setLines(-1,0);
                //makeStrings();
                //displayStrings();
                if (pieceHittingSomething() && pieceState==touching && runtime.seconds() > landingTime) clearStrings();
                else resetStrings();
            }
            cycleLeft = 1;
        } else if (!gamepad1.dpad_left && cycleLeft == 1)
        {
            cycleLeft =0; //wait for the left D-pad to be released before running the code above again.
            if (debugMode)
            {
                telemetry.log().add("Dpad left released");
                debugModeText = "Dpad left released";
            }
        }
        if (gamepad1.dpad_down)
        {
            if (debugMode)
            {
                telemetry.log().add("Dpad Down pressed");
                debugModeText = "Dpad Down pressed";
            }
            while (!pieceHittingSomething())
            {

                if (otherTime.seconds() > .03)
                {
                    resetStrings();

                    setLines(0,-1);

                    //makeStrings();
                    //displayStrings();
                    otherTime.reset();
                }
            }
            setLines(0,1);
        }
        /*while (gamepad1.dpad_down && !pieceHittingSomething() && opModeIsActive())
        {
            if (debugMode)
            {
                telemetry.log().add("Dpad Down pushed");
                debugModeText = "Dpad Down pushed";
            }
            if (otherTime.seconds() > .3 && !pieceHittingSomething())
            {
                resetStrings();
                setLines(0,-1);
                makeStrings();
                displayStrings();
                otherTime.reset();
            }
            if (debugMode && !gamepad1.dpad_down)
            {
                telemetry.log().add("Dpad down released");
                debugModeText = "Dpad down released";
            }
        }
        */
    }

    public void getPiece() {
        if (contains(lines[lines.length-3], true)) //if the bottom line is filled (every value of lines[5] is true), then delete line 5, lower all the lines down by 1, add an empty line[0] by setting every value of line[0] to false.
        {
            if (debugMode)
            {
                telemetry.log().add("cleared line");
                debugModeText = "cleared line";
            }
            boolean[][] templines=lines;
            for (int i=lines.length-1; i >0; i--) lines[i]=lines[i-1]; //lower everything in the board by 1
            for (int i=0; i < lines[0].length; i++) lines[0][i]=false; //clear first line

        }
        decider = nextDecider; //comment out the Math.random() part and replace it with 0 for debugging purposes
        nextDecider = (int) (Math.random() * 7);
        //each piece has four coordinates that form a piece.  the coordinates get put into lines[][] to be displayed.  For example, if the coordinate is 9, it means lines[0][9], 10 means lines[0][10], 11 means lines[0][11]110 means lines[1][10].  the setLines() method changes that coordinate.
        rotation=0;
        pieceModifery = 0;
        pieceModiferx = 0;
        if (decider == 0) //T shaped piece
        {
            pieces[0] = (lines[0].length/2) - 1;// 9;
            pieces[1] = (lines[0].length/2); //10;
            pieces[2] = (lines[0].length/2) + 1; //11;
            pieces[3] = (lines[0].length/2) + 100; //110;
            tempPiece[0] = (lines[0].length/2) - 1; //9; //tempPiece, pieces90, 180, and 270 are for rotating the piece
            tempPiece[1] = (lines[0].length/2); //10;
            tempPiece[2] = (lines[0].length/2) + 1; //11;
            tempPiece[3] = (lines[0].length/2) + 100; //110;
            pieces90[0] = (lines[0].length/2); //10;
            pieces90[1] = (lines[0].length/2) + 100; //110;
            pieces90[2] = (lines[0].length/2) + 200; //210;
            pieces90[3] = (lines[0].length/2) + 101; //111;
            pieces180[0] = (lines[0].length/2) + 101; //111;
            pieces180[1] = (lines[0].length/2) + 100; //110;
            pieces180[2] = (lines[0].length/2) + 99; //109;
            pieces180[3] = (lines[0].length/2); //10;
            pieces270[0] = (lines[0].length/2) + 200; //210;
            pieces270[1] = (lines[0].length/2) + 100; //110;
            pieces270[2] = (lines[0].length/2); //10;
            pieces270[3] = (lines[0].length/2) + 99; //109;
        } else if (decider == 1) //J shaped piece
        {
            pieces[0] = (lines[0].length/2) -1; //9;
            pieces[1] = (lines[0].length/2) + 99; //109;
            pieces[2] = (lines[0].length/2) + 100; //110;
            pieces[3] = (lines[0].length/2) + 101; //111;
            tempPiece[0] = (lines[0].length/2) - 1; //9; //tempPiece, pieces90, 180, and 270 are for rotating the piece
            tempPiece[1] = (lines[0].length/2) + 99; //109;
            tempPiece[2] = (lines[0].length/2) + 100; //110;
            tempPiece[3] = (lines[0].length/2) + 101; //111;
            pieces90[0] = (lines[0].length/2) + 200; //210;
            pieces90[1] = (lines[0].length/2) + 201; //211;
            pieces90[2] = (lines[0].length/2) + 101; //111;
            pieces90[3] = (lines[0].length/2) + 1; //11;
            pieces180[0] = (lines[0].length/2) + 101; //111;
            pieces180[1] = (lines[0].length/2) + 1;//11;
            pieces180[2] = (lines[0].length/2); //10;
            pieces180[3] = (lines[0].length/2)- 1;//9;
            pieces270[0] = (lines[0].length/2); //10;
            pieces270[1] = (lines[0].length/2) - 1; //9;
            pieces270[2] = (lines[0].length/2) + 99; //109;
            pieces270[3] = (lines[0].length/2) + 199; //209;
        } else if (decider == 2) //backward L shaped piece
        {
            pieces[0] = (lines[0].length/2) + 1; //11;
            pieces[1] = (lines[0].length/2) + 101; //111;
            pieces[2] = (lines[0].length/2) + 100; //110;
            pieces[3] = (lines[0].length/2) + 99; //109;
            tempPiece[0] = (lines[0].length/2) + 1; //11; //tempPiece, pieces90, 180, and 270 are for rotating the piece
            tempPiece[1] = (lines[0].length/2) + 101; //111;
            tempPiece[2] = (lines[0].length/2) + 100; //110;
            tempPiece[3] = (lines[0].length/2) + 99; //109;
            pieces90[0] = (lines[0].length/2); //10;
            pieces90[1] = (lines[0].length/2) + 1; //11;
            pieces90[2] = (lines[0].length/2) + 101; //111;
            pieces90[3] = (lines[0].length/2) + 201; //211;
            pieces180[0] = (lines[0].length/2) + 99; //109;
            pieces180[1] = (lines[0].length/2) - 1; //9;
            pieces180[2] = (lines[0].length/2); //10;
            pieces180[3] = (lines[0].length/2) + 1; //11;
            pieces270[0] = (lines[0].length/2) - 1; //9;
            pieces270[1] = (lines[0].length/2) + 99; //109;
            pieces270[2] = (lines[0].length/2) + 199; //209;
            pieces270[3] = (lines[0].length/2) + 200; //210;
        } else if (decider == 3) //square shaped piece
        { //a square does not need to rotate.
            pieces[0] = (lines[0].length/2) -1;//9;
            pieces[1] = (lines[0].length/2); //10;
            pieces[2] = (lines[0].length/2) + 99; //109;
            pieces[3] = (lines[0].length/2) + 100; //110;
            tempPiece[0] = (lines[0].length/2) -1; //9; //tempPiece, pieces90, 180, and 270 are for rotating the piece
            tempPiece[1] = (lines[0].length/2); //10;
            tempPiece[2] = (lines[0].length/2) + 99; //109;
            tempPiece[3] = (lines[0].length/2) + 100;//110;
            pieces90[0] =  pieces[0];
            pieces90[1] = pieces[1];
            pieces90[2] = pieces[2];
            pieces90[3] = pieces[3];
            pieces180[0] = pieces[0];
            pieces180[1] = pieces[1];
            pieces180[2] = pieces[2];
            pieces180[3] = pieces[3];
            pieces270[0] = pieces[0];
            pieces270[1] = pieces[1];
            pieces270[2] = pieces[2];
            pieces270[3] = pieces[3];
        } else if (decider == 4) //Line shaped piece
        {
            pieces[0] = (lines[0].length/2) - 2; //8;
            pieces[1] = (lines[0].length/2) -1 ; //9;
            pieces[2] = (lines[0].length/2); // 10;
            pieces[3] = (lines[0].length/2) + 1; //11;
            tempPiece[0] = (lines[0].length/2) -2; //8; //tempPiece, pieces90, 180, and 270 are for rotating the piece
            tempPiece[1] = (lines[0].length/2) -1;//9;
            tempPiece[2] = (lines[0].length/2);//10;
            tempPiece[3] = (lines[0].length/2)+1;//11;
            pieces90[0] = (lines[0].length/2)-1;//9;
            pieces90[1] = (lines[0].length/2)+99;//109;
            pieces90[2] = (lines[0].length/2)+199;//209;
            pieces90[3] = (lines[0].length/2)+299;//309;
            pieces180[0] = (lines[0].length/2) -2; //8;
            pieces180[1] = (lines[0].length/2) -1; //9;
            pieces180[2] = (lines[0].length/2); //10;
            pieces180[3] = (lines[0].length/2)+1;//11;
            pieces270[0] = (lines[0].length/2);//10;
            pieces270[1] = (lines[0].length/2)+100;//110;
            pieces270[2] = (lines[0].length/2)+200;//210;
            pieces270[3] = (lines[0].length/2)+300;//310;
        } else if (decider == 5) //S shaped piece
        {
            pieces[0] = (lines[0].length/2) + 1;//11;
            pieces[1] = (lines[0].length/2);//10;
            pieces[2] = (lines[0].length/2)+100;//110;
            pieces[3] = (lines[0].length/2)+99;//109;
            tempPiece[0] = (lines[0].length/2)+1;//11; //tempPiece, pieces90, 180, and 270 are for rotating the piece
            tempPiece[1] = (lines[0].length/2);//10;
            tempPiece[2] = (lines[0].length/2) + 100; //110;
            tempPiece[3] = (lines[0].length/2) + 99; //109;
            pieces90[0] = (lines[0].length/2); //10;
            pieces90[1] = (lines[0].length/2)+100; //110;
            pieces90[2] = (lines[0].length/2)+109; //119;
            pieces90[3] = (lines[0].length/2) + 209; //219;
            pieces180[0] = (lines[0].length/2) + 1;//11;
            pieces180[1] = (lines[0].length/2); //10;
            pieces180[2] = (lines[0].length/2) + 100; //110;
            pieces180[3] = (lines[0].length/2) + 99; //109;
            pieces270[0] = (lines[0].length/2); //10;
            pieces270[1] = (lines[0].length/2) + 100; //110;
            pieces270[2] = (lines[0].length/2) + 109; //119;
            pieces270[3] = (lines[0].length/2) + 209; //219;
        } else if (decider == 6) //Z shaped piece
        {
            pieces[0] = (lines[0].length/2)-1; //9;
            pieces[1] = (lines[0].length/2);//10;
            pieces[2] = (lines[0].length/2) + 100;//110;
            pieces[3] = (lines[0].length/2) + 101;//111;
            tempPiece[0] = (lines[0].length/2)-1;//9; //tempPiece, pieces90, 180, and 270 are for rotating the piece
            tempPiece[1] = (lines[0].length/2);//10;
            tempPiece[2] = (lines[0].length/2)+100;//110;
            tempPiece[3] = (lines[0].length/2)+101;//111;
            pieces90[0] = (lines[0].length/2)+1;//11;
            pieces90[1] = (lines[0].length/2) + 101;//111;
            pieces90[2] = (lines[0].length/2)+100;//110;
            pieces90[3] = (lines[0].length/2)+200;//210;
            pieces180[0] = (lines[0].length/2)-1;//9;
            pieces180[1] = (lines[0].length/2);//10;
            pieces180[2] = (lines[0].length/2)+100;//110;
            pieces180[3] = (lines[0].length/2)+101;//111;
            pieces270[0] = (lines[0].length/2)+1;//11;
            pieces270[1] = (lines[0].length/2)+101;//111;
            pieces270[2] = (lines[0].length/2)+100;//110;
            pieces270[3] = (lines[0].length/2)+200;//210;
        }
        if (nextDecider == 0) {
            nextPieceText[1] = "███";
            nextPieceText[2] = "░█░";
        } else if (nextDecider == 1)
        {
            nextPieceText[1] = "█░░";
            nextPieceText[2] = "███";
        } else if (nextDecider==2)
        {
            nextPieceText[1] = "░░█";
            nextPieceText[2] = "███";
        } else if (nextDecider==3)
        {
            nextPieceText[1] = "██";
            nextPieceText[2] = "██";
        } else if (nextDecider==4)
        {
            nextPieceText[1] = "████";
            nextPieceText[2] = "";
        }
        else if (nextDecider == 5)
        {
            nextPieceText[1] = "░██";
            nextPieceText[2] = "██░";
        }
        else if (nextDecider==6)
        {
            nextPieceText[1] = "██░";
            nextPieceText[2] = "░██";
        }
    }
    public void rotatePiece() //attempts to rotate piece, needs work.  There is a rotation variable to store the current rotation of the piece
    {
        resetStrings();
        if (!pieceHittingSomething())
        {
            if (!pieceInBoundsLeft()) setLines(1,0);
            else if (!pieceInBoundsRight()) setLines(-1,0);
            if (rotation == 0)
            {
                for (int i = 0; i < 4; i++) pieces[i] = (pieces90[i] + pieceModiferx + pieceModifery);
                rotation=90;
            }
            else if (rotation == 90)
            {
                for (int i = 0; i < 4; i++) pieces[i] = (pieces180[i] + pieceModiferx + pieceModifery);
                rotation=180;
            }
            else if (rotation == 180)
            {
                for (int i = 0; i < 4; i++) pieces[i] = (pieces270[i] + pieceModiferx + pieceModifery);
                rotation=270;
            }
            else if (rotation == 270)
            {
                for (int i = 0; i < 4; i++) pieces[i] = (tempPiece[i] + pieceModiferx + pieceModifery);
                rotation=0;
            }
            makeStrings();
            displayStrings();

        }
        else
        {
            if (debugMode)
            {
                rotationErrorCounter++;
                telemetry.log().add("Failed to rotate     (" + rotationErrorCounter + ")");
                debugModeText = "Failed to rotate     (" + rotationErrorCounter + ")";
            }
        }
    }
    public void setLines(int xModifier, int yModifier) //changes the location of the piece in control based on the parameters
    {
        pieceModifery-=(yModifier*100);
        pieceModiferx+=xModifier;
        for (int i = 0; i < 4; i++)
        {
            pieces[i]-=(yModifier*100);
            pieces[i]+=xModifier;
            if (pieces[i] >= (lines.length*100)) pieces[i]-=100;
            //telemetry.log().add(Integer.toString(pieces[i]));
            int vertical = (pieces[i] / 100);
            //telemetry.log().add("Vertical is " + vertical); //used for debugging
            int horizontal = (pieces[i] % 100);
            lines[vertical][horizontal]=true;
        }
        makeStrings();
        displayStrings();

    }
    public void makeStrings() //compose all the strings,
    {
        clearStrings();
        for (int i =0; i < sline.length; i++) sline[i] +=bigSpace;
        for (int i =0; i < sline.length; i++) for (int j = 0; j < lines[0].length; j++)
            sline[i] += (!lines[i][j]) ? "░" : "█";
        //the rest of this method puts sline[] into one string, along with the Debug mode text and framerate
        DisplayThis="";
        for (int i = 0; i < 3; i++)
        {
            DisplayThis+=sline[i] + "       " +nextPieceText[i] + "\n";
        }
        if (sline.length==4) DisplayThis+=sline[3] + "      " + debugModeText + "\n";
        else {
            DisplayThis += sline[3] + "\n" + sline[4] + debugModeText + "\n";
            if (sline.length == 5) DisplayThis += frameRate + "\n";
            else {
                DisplayThis += sline[5] + "\n" + sline[6] + "      " + "\n";
                for (int i = 6; i < (sline.length-1); i++) {
                    DisplayThis += sline[i] + "\n";
                }
            }
        }
    }
    public void displayStrings() //pretty self explanatory,
    {
        frame++;
        if (frametime.seconds() >=1)
        {
            frameRate = "Frame Rate: " + frame + " FPS";
            frame=0;
            frametime.reset();
        }
        telemetry.addData(DisplayThis, frameRate);
        //telemetry.addData(sline[0] + "         " + nextPieceText[0] + "\n" + sline[1] + "         " + nextPieceText[1] + "\n" + sline[2] + "         " + nextPieceText[2] +  "\n" + sline[3] + "\n" + sline[4] + "         " + debugModeText + "\n" + sline[5] + "\n" + sline[6], "");
        telemetry.update();
    }
    public void resetStrings() //doesn't exactly reset the strings, it turns lines[][] values that are true because of the current piece controlling it and turn those values to false, then clears the strings
    {
        for (int i = 0; i < 4; i++)
        {
            int vertical = (pieces[i] / 100);
            int horizontal = (pieces[i] % 100);
            lines[vertical][horizontal] = false;
        }
        clearStrings();
        /* old way to reset strings
        for (int i = 0; i<20; i++) line1[i] = false;
        for (int i = 0; i<20; i++) line2[i] = false;
        for (int i = 0; i<20; i++) line3[i] = false;
        for (int i = 0; i<20; i++) line4[i] = false;
        for (int i = 0; i<20; i++) line5[i] = false;
        for (int i = 0; i<20; i++) line6[i] = false;
         */
    }
    public void clearStrings()
    {
        for (int i = 0; i < sline.length; i++) sline[i] = "";
    }

    public void gameOver() //gets called when the game is over
    {
        debugModeText = "GAME OVER";
        telemetry.log().add("GAME OVER");
        DisplayThis= "GAME OVER \n" + DisplayThis;
        displayStrings(); //display the strings
        while (opModeIsActive()); //wait for the user to end the program
    }


}
