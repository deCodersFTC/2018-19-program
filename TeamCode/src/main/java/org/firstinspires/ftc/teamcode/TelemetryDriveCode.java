public class TelemetryDriveCode{
    String driveMovement;
    public double dN;
    public double sN;
    public double tN;
    public int numberCode;

    public void checkPositions(Gamepad gamepad){
        dn = gamepad.left_stick_y;
        sn = gamepad.left_stick_x;
        tn = gamepad.right_stick_x;

        if (dn < 0){
            dn = -1;
        }
        else if (dn == 0){
            dn = 0;
        }
        else{
            dn = 1;
        }






        if (sn < 0){
            sn = -1;
        }
        else if (sn == 0){
            sn = 0;
        }
        else{
            sn = 1;
        }





        if (tn < 0){
            tn = -1;
        }
        else if (tn == 0){
            tn = 0;
        }
        else{
            tn = 1;
        }

    }
   /*
    * dn = Drive
    * sn = Slide
    * tn = turn
    */
    public String TelemetryDriveCode(Gamepad gamepad){
        switch(dn){
            case -1:
                switch (sn){
                    case -1:
                        switch (tn){
                            case -1:
                            case 0:
                                return "Going backwards and left";
                            case 1:
                        }
                        break;
                    case 0:
                        switch (tn){
                            case -1:
                                return "Turning on the back left wheel";
                            case 0:
                                return "Going backwards";
                            case 1:
                                return "Turning on the back right wheel";
                        }
                        break;
                    case 1:
                        switch (tn){
                            case -1:
                            case 0:
                                return "Going backwards and right";
                            case 1:
                        }
                        break;
                }
                break;
            case 0:
                switch (sn){
                    case -1:
                        switch (tn){
                            case -1:
                                return "Turning on the back left wheel";
                            case 0:
                                return "Sliding left";
                            case 1:
                                "Turning on the back right wheel";
                        }
                        break;
                    case 0:
                        switch (tn){
                            case -1:
                                return "Center turning left";
                            case 0:
                                return "Doing nothing";
                            case 1:
                                return "Center turning right";
                        }
                        break;
                    case 1:
                        switch (tn){
                            case -1:
                            case 0:
                                return "Sliding right";
                            case 1:
                        }
                        break;
                }
                break;
            case 1:
                switch (sn){
                    case -1:
                        switch (tn){
                            case -1:
                            case 0:
                                return "Going forwards and left";
                            case 1:
                        }
                        break;
                    case 0:
                        switch (tn){
                            case -1:
                                return "Turning on the front left wheel";
                            case 0:
                                return "Going forwards";
                            case 1:
                                return "Turning on the front right wheel";
                        }
                        break;
                    case 1:
                        switch (tn){
                            case -1:
                            case 0:
                                return "Going forwards and right";
                            case 1:
                        }
                        break;
                }
                break;
        }
    }
}